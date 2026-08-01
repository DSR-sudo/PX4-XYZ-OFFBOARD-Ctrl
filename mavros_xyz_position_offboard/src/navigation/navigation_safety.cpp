#include "mavros_xyz_position_offboard/navigation/navigation.hpp"

#include <cmath>
#include <stdexcept>

namespace mavros_xyz_position_offboard::navigation
{
namespace
{

double normalized_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

bool Navigation::actual_xy_within(
  const common::Telemetry & telemetry, double x, double y, double tolerance) const
{
  return common::finite(telemetry.local_x_m) && common::finite(telemetry.local_y_m) &&
         std::hypot(telemetry.local_x_m - x, telemetry.local_y_m - y) <= tolerance;
}

bool Navigation::stable_at(
  const common::Telemetry & telemetry, const common::PositionSetpoint & target) const
{
  return actual_xy_within(telemetry, target.x_m, target.y_m, config_.target_tolerance_m) &&
         common::finite(telemetry.local_z_m) &&
         std::abs(telemetry.local_z_m - target.z_m) <= config_.target_tolerance_m;
}

bool Navigation::actual_yaw_within(
  const common::Telemetry & telemetry, double yaw_rad, double tolerance_rad) const
{
  try {
    const double actual = common::yaw_from_quaternion(common::normalize_quaternion(
      telemetry.orientation.x, telemetry.orientation.y, telemetry.orientation.z, telemetry.orientation.w));
    return std::abs(normalized_angle(actual - yaw_rad)) <= tolerance_rad;
  } catch (const std::invalid_argument &) {
    return false;
  }
}

common::PositionSetpoint Navigation::measured_hold_setpoint(const NavigationInput & input) const
{
  common::PositionSetpoint hold = control_.commanded_setpoint.value_or(planner_.current());
  if (common::finite(input.telemetry.local_x_m)) {hold.x_m = input.telemetry.local_x_m;}
  if (common::finite(input.telemetry.local_y_m)) {hold.y_m = input.telemetry.local_y_m;}
  if (common::finite(input.telemetry.local_z_m)) {hold.z_m = input.telemetry.local_z_m;}
  hold.vertical_rate_m_s = 0.0;
  return hold;
}

void Navigation::enter_hold(
  const NavigationInput & input, const std::string & hold_phase,
  const std::string & reason, const std::optional<std::string> & resume_phase)
{
  if (!planner_.latched() || phase_ == hold_phase) {return;}
  if (phase_ == "target_lock_following") {pause_target_lock_follow(input.now);}
  const auto hold = measured_hold_setpoint(input);
  planner_.freeze_xy_at(hold.x_m, hold.y_m);
  planner_.freeze_z_at(hold.z_m);
  planner_.set_yaw_rad(common::yaw_from_quaternion(hold.orientation));
  control_.hold_setpoint = hold;
  control_.hold_reason = reason;
  control_.hold_resume_phase = resume_phase.value_or(phase_);
  control_.mission_paused = true;
  pending_release_gripper_ = false;
  transition(hold_phase, input.now);
}

void Navigation::clear_hold()
{
  control_.hold_setpoint.reset();
  control_.hold_reason.clear();
  control_.hold_resume_phase.clear();
  control_.mission_paused = false;
}

void Navigation::resume_hold(double now)
{
  const std::string resume = control_.hold_resume_phase.empty() ?
    "waiting_target" : control_.hold_resume_phase;
  plan_to_mission_goal();
  clear_hold();
  transition(resume, now);
  if (resume == "target_lock_following") {resume_target_lock_follow(now);}
}

void Navigation::begin_landing(double now, const std::string & reason)
{
  pending_release_gripper_ = false;
  latest_car_status_.reset();
  target_lock_follow_elapsed_s_ = 0.0;
  target_lock_follow_started_at_.reset();
  target_lock_follow_completed_ = false;
  if (planner_.latched()) {
    clear_hold();
    planner_.hold_xy();
    if (control_.origin) {planner_.set_z_target(control_.origin->z_m);}
  }
  landing_reason_ = reason;
  normal_completion_ = false;
  transition("landing", now);
}

bool Navigation::lcp_required_in_phase() const
{
  return phase_ == "height_stabilizing" || phase_ == "transit_to_b" ||
    phase_ == "waiting_target" || phase_ == "target_lock_following" ||
    phase_ == "throwing" || phase_ == "returning" ||
    phase_ == "downing";
}

}  // namespace mavros_xyz_position_offboard::navigation
