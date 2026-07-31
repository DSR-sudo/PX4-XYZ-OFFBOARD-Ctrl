#include "mavros_xyz_position_offboard/navigation/navigation.hpp"

#include <stdexcept>
#include <utility>

namespace mavros_xyz_position_offboard::navigation
{

Navigation::Navigation(const common::SafetyConfig & config, MissionConfig mission)
: config_(config), mission_(std::move(mission)), planner_(config), target_tracker_(mission_)
{
  mission_.validate();
}

void Navigation::reset()
{
  planner_.reset();
  target_tracker_.reset();
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
  intercept_due_at_.reset();
  cardinal_alignment_achieved_ = false;
  last_own_pose_at_.reset();
  last_own_x_m_.reset();
  last_own_y_m_.reset();
  own_vx_m_s_ = 0.0;
  own_vy_m_s_ = 0.0;
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
  planner_.set_target(goal.x_m, goal.y_m, goal.z_m);
  planner_.set_yaw_rad(common::yaw_from_quaternion(goal.orientation));
}

}  // namespace mavros_xyz_position_offboard::navigation
