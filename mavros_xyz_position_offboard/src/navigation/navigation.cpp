#include "mavros_xyz_position_offboard/navigation/navigation.hpp"

#include <stdexcept>
#include <utility>

namespace mavros_xyz_position_offboard::navigation
{

Navigation::Navigation(const common::SafetyConfig & config, MissionConfig mission)
: config_(config), mission_(std::move(mission)), planner_(config)
{
  mission_.validate();
}

void Navigation::reset()
{
  planner_.reset();
  phase_ = "waiting_preflight";
  phase_started_at_ = 0.0;
  flight_started_at_.reset();
  normal_completion_ = false;
  control_ = {};
  pending_messages_.clear();
  pending_rejections_.clear();
  landing_reason_.clear();
  pending_release_gripper_ = false;
  last_car_status_at_.reset();
  latest_car_status_.reset();
  target_lock_follow_elapsed_s_ = 0.0;
  target_lock_follow_started_at_.reset();
  target_lock_follow_completed_ = false;
  tracking_z_offset_m_ = 0.0;
  tracking_z_previous_z_m_.reset();
  tracking_z_previous_at_.reset();
}

void Navigation::transition(const std::string & phase, double now)
{
  phase_ = phase;
  phase_started_at_ = now;
}

void Navigation::emit(communication::MessageType type)
{
  pending_messages_.push_back({type});
}

void Navigation::reject(const std::string & reason)
{
  pending_rejections_.push_back(reason);
}

void Navigation::set_mission_goal(const common::PositionSetpoint & goal)
{
  if (!common::finite(goal.x_m) || !common::finite(goal.y_m) || !common::finite(goal.z_m)) {
    throw std::invalid_argument("mission goal XYZ must be finite");
  }
  auto normalized = goal;
  normalized.orientation = common::normalize_quaternion(
    goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w);
  normalized.vertical_rate_m_s = 0.0;
  control_.mission_goal = normalized;
}

void Navigation::plan_to_mission_goal()
{
  if (!planner_.latched() || !control_.mission_goal) {return;}
  const auto & goal = *control_.mission_goal;
  planner_.set_xy_target(goal.x_m, goal.y_m);
  plan_to_tracking_z_target();
  planner_.set_yaw_rad(common::yaw_from_quaternion(goal.orientation));
}

void Navigation::plan_to_tracking_z_target()
{
  if (!planner_.latched() || !control_.mission_goal) {return;}
  planner_.set_z_target(control_.mission_goal->z_m + tracking_z_offset_m_);
}

void Navigation::reset_tracking_z_sample()
{
  tracking_z_previous_z_m_.reset();
  tracking_z_previous_at_.reset();
}

void Navigation::clear_tracking_z_compensation()
{
  tracking_z_offset_m_ = 0.0;
  control_.tracking_z_offset_m = 0.0;
  control_.tracking_z_last_jump_direction = "none";
  reset_tracking_z_sample();
}

void Navigation::observe_tracking_z(const NavigationInput & input)
{
  if (!common::finite(input.telemetry.local_z_m) || !planner_.latched()) {
    reset_tracking_z_sample();
    return;
  }

  if (!tracking_z_previous_z_m_ || !tracking_z_previous_at_ ||
    input.now < *tracking_z_previous_at_) {
    tracking_z_previous_z_m_ = input.telemetry.local_z_m;
    tracking_z_previous_at_ = input.now;
    return;
  }

  const double elapsed = input.now - *tracking_z_previous_at_;
  const double delta = input.telemetry.local_z_m - *tracking_z_previous_z_m_;
  if (elapsed <= mission_.tracking_z_jump_window_s &&
    std::abs(delta) >= mission_.tracking_z_jump_threshold_m) {
    const bool downward = delta < 0.0;
    tracking_z_offset_m_ += downward ? -mission_.tracking_z_step_m : mission_.tracking_z_step_m;
    control_.tracking_z_offset_m = tracking_z_offset_m_;
    control_.tracking_z_last_jump_direction = downward ? "down" : "up";
    plan_to_tracking_z_target();
  }

  tracking_z_previous_z_m_ = input.telemetry.local_z_m;
  tracking_z_previous_at_ = input.now;
}

}  // namespace mavros_xyz_position_offboard::navigation
