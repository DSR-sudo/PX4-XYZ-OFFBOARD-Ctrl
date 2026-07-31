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

double nearest_cardinal(double angle)
{
  const double half_pi = std::acos(-1.0) / 2.0;
  return normalized_angle(std::round(angle / half_pi) * half_pi);
}

}  // namespace

void Navigation::begin_transit_to_b()
{
  if (!control_.origin || !control_.mission_goal) {
    throw std::logic_error("B-point transit requires a latched origin and climb goal");
  }
  const double psi0 = common::yaw_from_quaternion(control_.origin->orientation);
  auto b_goal = *control_.mission_goal;
  b_goal.x_m = control_.origin->x_m + mission_.b_forward_m * std::cos(psi0) +
    mission_.b_right_m * std::sin(psi0);
  b_goal.y_m = control_.origin->y_m + mission_.b_forward_m * std::sin(psi0) -
    mission_.b_right_m * std::cos(psi0);
  b_goal.orientation = control_.origin->orientation;
  set_mission_goal(b_goal);
  plan_to_mission_goal();
}

void Navigation::update_own_velocity(const NavigationInput & input)
{
  if (common::finite(input.telemetry.velocity_x_m_s) &&
    common::finite(input.telemetry.velocity_y_m_s)) {
    own_vx_m_s_ = input.telemetry.velocity_x_m_s;
    own_vy_m_s_ = input.telemetry.velocity_y_m_s;
  } else if (last_own_pose_at_ && last_own_x_m_ && last_own_y_m_ &&
    input.now > *last_own_pose_at_ && common::finite(input.telemetry.local_x_m) &&
    common::finite(input.telemetry.local_y_m)) {
    const double dt = input.now - *last_own_pose_at_;
    own_vx_m_s_ = (input.telemetry.local_x_m - *last_own_x_m_) / dt;
    own_vy_m_s_ = (input.telemetry.local_y_m - *last_own_y_m_) / dt;
  }
  if (common::finite(input.telemetry.local_x_m) && common::finite(input.telemetry.local_y_m)) {
    last_own_pose_at_ = input.now;
    last_own_x_m_ = input.telemetry.local_x_m;
    last_own_y_m_ = input.telemetry.local_y_m;
  }
}

bool Navigation::apply_car_status(
  const NavigationInput & input, const communication::CarStatus & status)
{
  if (!control_.origin || !common::finite(input.telemetry.local_x_m) ||
    !common::finite(input.telemetry.local_y_m)) {
    reject("car_status_requires_local_pose");
    return false;
  }
  double yaw = 0.0;
  try {
    yaw = common::yaw_from_quaternion(common::normalize_quaternion(
      input.telemetry.orientation.x, input.telemetry.orientation.y,
      input.telemetry.orientation.z, input.telemetry.orientation.w));
  } catch (const std::invalid_argument &) {
    reject("car_status_requires_valid_yaw");
    return false;
  }
  const double world_bearing = yaw + status.bearing_rad;
  const double target_x = input.telemetry.local_x_m + status.distance_m * std::cos(world_bearing);
  const double target_y = input.telemetry.local_y_m + status.distance_m * std::sin(world_bearing);
  if (std::hypot(target_x - control_.origin->x_m, target_y - control_.origin->y_m) >
    mission_.max_tracking_radius_m) {
    reject("car_status_target_outside_tracking_radius");
    target_tracker_.reset();
    control_.target_samples = 0;
    last_car_status_at_.reset();
    intercept_due_at_.reset();
    enter_hold(input, "lcp_hold", "car_status_target_outside_tracking_radius", "waiting_target");
    return false;
  }
  if (!target_tracker_.update(target_x, target_y, input.now)) {
    reject("car_status_innovation_outlier");
    target_tracker_.reset();
    control_.target_samples = 0;
    last_car_status_at_.reset();
    intercept_due_at_.reset();
    enter_hold(input, "lcp_hold", "car_status_innovation_outlier", "waiting_target");
    return false;
  }
  last_car_status_at_ = input.now;
  control_.target_samples = target_tracker_.samples();
  clear_hold();
  if (target_tracker_.ready()) {plan_intercept(input, status.bearing_rad);}
  return true;
}

bool Navigation::car_status_fresh(double now) const
{
  return last_car_status_at_ && now >= *last_car_status_at_ &&
         now - *last_car_status_at_ <= mission_.car_status_timeout_s;
}

void Navigation::plan_intercept(const NavigationInput & input, double bearing_rad)
{
  if (!target_tracker_.ready() || !control_.origin || !common::finite(input.telemetry.local_x_m) ||
    !common::finite(input.telemetry.local_y_m)) {
    return;
  }
  const auto remaining = target_tracker_.time_to_distance(
    input.telemetry.local_x_m, input.telemetry.local_y_m, own_vx_m_s_, own_vy_m_s_,
    mission_.throw_distance_m, input.now, mission_.prediction_horizon_s);
  if (!remaining) {
    intercept_due_at_.reset();
    control_.predicted_intercept_seconds.reset();
    enter_hold(input, "lcp_hold", "intercept_prediction_outside_window", "waiting_target");
    return;
  }
  const auto target = target_tracker_.estimate(input.now + *remaining);
  const double initial_yaw = common::yaw_from_quaternion(control_.origin->orientation);
  auto goal = control_.mission_goal.value_or(*control_.origin);
  goal.z_m = control_.origin->z_m + mission_.takeoff_height_m;
  goal.orientation = control_.origin->orientation;
  control_.predicted_intercept_seconds = *remaining;
  intercept_due_at_ = input.now + *remaining;
  if (*remaining <= mission_.final_intercept_seconds) {
    goal.x_m = target.x_m;
    goal.y_m = target.y_m;
    set_mission_goal(goal);
    if (*remaining <= 0.01) {
      planner_.set_xy_target(goal.x_m, goal.y_m);
      control_.tracking_arrival_time_met = true;
    } else {
      control_.tracking_arrival_time_met = planner_.set_xy_target_with_arrival_time(
        goal.x_m, goal.y_m, *remaining);
    }
    planner_.set_z_target(goal.z_m);
    planner_.set_yaw_rad(initial_yaw);
    cardinal_alignment_achieved_ = true;
    transition("final_intercept", input.now);
    return;
  }

  const double cardinal_relative = nearest_cardinal(bearing_rad);
  cardinal_alignment_achieved_ = std::abs(normalized_angle(bearing_rad - cardinal_relative)) <=
    mission_.cardinal_tolerance_deg * std::acos(-1.0) / 180.0;
  if (cardinal_alignment_achieved_) {
    goal.x_m = target.x_m;
    goal.y_m = target.y_m;
  } else {
    const double range = std::hypot(target.x_m - input.telemetry.local_x_m,
      target.y_m - input.telemetry.local_y_m);
    const double world_cardinal = initial_yaw + cardinal_relative;
    goal.x_m = input.telemetry.local_x_m + range * std::cos(world_cardinal);
    goal.y_m = input.telemetry.local_y_m + range * std::sin(world_cardinal);
  }
  set_mission_goal(goal);
  control_.tracking_arrival_time_met = planner_.set_xy_target_with_arrival_time(
    goal.x_m, goal.y_m, *remaining);
  planner_.set_z_target(goal.z_m);
  planner_.set_yaw_rad(initial_yaw);
  if (!control_.tracking_arrival_time_met) {
    intercept_due_at_.reset();
    enter_hold(input, "lcp_hold", "intercept_unreachable_with_constraints", "waiting_target");
    return;
  }
  transition("cardinal_alignment", input.now);
}

void Navigation::begin_return(double now)
{
  if (!control_.origin) {
    begin_landing(now, "return_without_origin");
    return;
  }
  intercept_due_at_.reset();
  target_tracker_.reset();
  control_.target_samples = 0;
  control_.predicted_intercept_seconds.reset();
  clear_hold();
  auto return_goal = *control_.origin;
  return_goal.z_m = planner_.latched() ? planner_.current().z_m : return_goal.z_m;
  return_goal.orientation = {0.0, 0.0, 0.0, 1.0};
  set_mission_goal(return_goal);
  plan_to_mission_goal();
  transition("returning", now);
}

}  // namespace mavros_xyz_position_offboard::navigation
