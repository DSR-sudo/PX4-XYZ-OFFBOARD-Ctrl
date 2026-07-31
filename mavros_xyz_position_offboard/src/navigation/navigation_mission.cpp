#include "mavros_xyz_position_offboard/navigation/navigation.hpp"

#include <cmath>
#include <stdexcept>

namespace mavros_xyz_position_offboard::navigation
{
void Navigation::begin_transit_to_b()
{
  if (!control_.origin || !control_.mission_goal) {
    throw std::logic_error("B-point transit requires a latched origin and climb goal");
  }
  auto b_goal = *control_.mission_goal;
  b_goal.x_m = mission_.b_right_m;
  b_goal.y_m = mission_.b_forward_m;
  b_goal.orientation = control_.origin->orientation;
  set_mission_goal(b_goal);
  plan_to_mission_goal();
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
  // car_status uses the body frame: forward is zero and positive angles point left.
  // ENU yaw has the same counter-clockwise positive convention, so the angles add.
  // Thus rear is +/-pi.
  const double world_bearing = yaw + status.bearing_rad;
  const double target_x = input.telemetry.local_x_m + status.distance_m * std::cos(world_bearing);
  const double target_y = input.telemetry.local_y_m + status.distance_m * std::sin(world_bearing);
  if (std::hypot(target_x - control_.origin->x_m, target_y - control_.origin->y_m) >
    mission_.max_tracking_radius_m) {
    reject("car_status_target_outside_tracking_radius");
    control_.target_samples = 0;
    last_car_status_at_.reset();
    enter_hold(input, "lcp_hold", "car_status_target_outside_tracking_radius", "waiting_target");
    return false;
  }

  // Each accepted observation is the current vehicle-center target. Keep the
  // ARM-time yaw and the current task altitude unchanged.
  auto goal = control_.mission_goal ? *control_.mission_goal : planner_.current();
  goal.x_m = target_x;
  goal.y_m = target_y;
  goal.orientation = control_.origin->orientation;
  set_mission_goal(goal);
  planner_.set_xy_target_with_limits(
    target_x, target_y, mission_.car_tracking_max_speed_m_s,
    mission_.car_tracking_max_accel_m_s2);
  planner_.set_z_target(goal.z_m);
  planner_.set_yaw_rad(common::yaw_from_quaternion(control_.origin->orientation));

  last_car_status_at_ = input.now;
  control_.target_samples = 0;
  control_.predicted_intercept_seconds.reset();
  clear_hold();
  if (status.distance_m < mission_.throw_distance_m) {
    pending_release_gripper_ = true;
    transition("throwing", input.now);
  }
  return true;
}

bool Navigation::car_status_fresh(double now) const
{
  return last_car_status_at_ && now >= *last_car_status_at_ &&
         now - *last_car_status_at_ <= mission_.car_status_timeout_s;
}

void Navigation::begin_return(double now)
{
  if (!control_.origin) {
    begin_landing(now, "return_without_origin");
    return;
  }
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
