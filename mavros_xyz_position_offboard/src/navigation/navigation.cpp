#include "mavros_xyz_position_offboard/navigation/navigation.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

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
  const auto hold = measured_hold_setpoint(input);
  planner_.freeze_xy_at(hold.x_m, hold.y_m);
  planner_.freeze_z_at(hold.z_m);
  planner_.set_yaw_rad(common::yaw_from_quaternion(hold.orientation));
  control_.hold_setpoint = hold;
  control_.hold_reason = reason;
  control_.hold_resume_phase = resume_phase.value_or(phase_);
  control_.mission_paused = true;
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
}

void Navigation::begin_landing(double now, const std::string & reason)
{
  intercept_due_at_.reset();
  if (planner_.latched()) {
    clear_hold();
    planner_.hold_xy();
    if (control_.origin) {planner_.set_z_target(control_.origin->z_m);}
  }
  landing_reason_ = reason;
  normal_completion_ = false;
  transition("landing", now);
}

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

bool Navigation::lcp_required_in_phase() const
{
  return phase_ == "height_stabilizing" || phase_ == "transit_to_b" ||
    phase_ == "waiting_target" || phase_ == "cardinal_alignment" ||
    phase_ == "final_intercept" || phase_ == "throwing" || phase_ == "returning" ||
    phase_ == "downing";
}

void Navigation::process_events(const NavigationInput & input)
{
  for (const auto & event : input.events) {
    if (!event.accepted) {
      if (!event.rejection_reason.empty()) {reject(event.rejection_reason);}
      continue;
    }
    if (event.type == communication::MessageType::ack) {continue;}
    if (event.type == communication::MessageType::run_plan1) {
      if (phase_ == "waiting_run_plan1") {
        transition("offboard_request_pending", input.now);
      } else if (phase_ != "offboard_request_pending" && phase_ != "arming_request_pending" &&
        phase_ != "climb" && phase_ != "height_stabilizing") {
        reject("run_plan1_not_allowed_in_phase");
      }
      continue;
    }
    if (event.type == communication::MessageType::car_status) {
      if (!event.car_status) {
        reject("car_status_missing_measurement");
      } else if (phase_ == "waiting_target" || phase_ == "cardinal_alignment" ||
        phase_ == "final_intercept") {
        apply_car_status(input, *event.car_status);
      } else if (phase_ == "lcp_hold" && control_.hold_reason != "lcp_unhealthy") {
        const auto resume = control_.hold_resume_phase;
        clear_hold();
        transition(resume.empty() ? "waiting_target" : resume, input.now);
        apply_car_status(input, *event.car_status);
      } else {
        reject("car_status_not_allowed_in_phase");
      }
      continue;
    }
    reject("message_not_allowed_in_phase");
  }
}

NavigationDecision Navigation::update(const NavigationInput & input)
{
  if (!common::finite(input.now) || !common::finite(input.dt) || input.dt <= 0.0) {
    throw std::invalid_argument("navigation time and dt must be finite, dt must be positive");
  }
  pending_messages_.clear();
  pending_rejections_.clear();
  pending_release_gripper_ = false;
  update_own_velocity(input);
  process_events(input);

  if (phase_ == "waiting_preflight") {
    if (input.preflight_ready && input.lcp_healthy && common::finite(input.telemetry.local_x_m) &&
      common::finite(input.telemetry.local_y_m) && common::finite(input.telemetry.local_z_m)) {
      transition("setpoint_warmup", input.now);
    }
  } else if (phase_ == "setpoint_warmup") {
    if (input.now - phase_started_at_ >= config_.setpoint_warmup_s) {
      emit(communication::MessageType::ok_wait);
      transition("waiting_run_plan1", input.now);
    }
  } else if (phase_ == "offboard_request_pending") {
    if (input.controller.mode == "OFFBOARD") {transition("arming_request_pending", input.now);}
  } else if (phase_ == "arming_request_pending") {
    if (!input.controller.armed) {
      if (input.controller.mode != "OFFBOARD") {transition("offboard_request_pending", input.now);}
    } else if (input.controller.mode != "OFFBOARD") {
      begin_landing(input.now, "offboard_mode_lost");
    } else if (!input.flight_healthy) {
      begin_landing(input.now, input.health_errors.empty() ?
        "flight_health_failure" : input.health_errors.front());
    } else {
      planner_.latch(input.telemetry.local_x_m, input.telemetry.local_y_m,
        input.telemetry.local_z_m, input.telemetry.orientation);
      control_.origin = planner_.current();
      auto climb_goal = *control_.origin;
      climb_goal.z_m += mission_.takeoff_height_m;
      set_mission_goal(climb_goal);
      plan_to_mission_goal();
      flight_started_at_ = input.now;
      transition("climb", input.now);
    }
  }

  if (lcp_required_in_phase() && !input.lcp_healthy && phase_ != "lcp_hold") {
    enter_hold(input, "lcp_hold", "lcp_unhealthy");
  }

  if (phase_ == "climb") {
    if (planner_.target_reached() && stable_at(input.telemetry, planner_.current())) {
      transition("height_stabilizing", input.now);
    }
  } else if (phase_ == "height_stabilizing") {
    if (!planner_.target_reached() || !stable_at(input.telemetry, planner_.current())) {
      phase_started_at_ = input.now;
    } else if (input.now - phase_started_at_ >= mission_.height_stable_seconds) {
      begin_transit_to_b();
      transition("transit_to_b", input.now);
    }
  } else if (phase_ == "transit_to_b") {
    if (planner_.target_reached() && stable_at(input.telemetry, planner_.current())) {
      emit(communication::MessageType::ok_b);
      transition("waiting_target", input.now);
    }
  } else if (phase_ == "waiting_target" || phase_ == "cardinal_alignment" ||
    phase_ == "final_intercept") {
    if (last_car_status_at_ && !car_status_fresh(input.now)) {
      target_tracker_.reset();
      control_.target_samples = 0;
      intercept_due_at_.reset();
      control_.predicted_intercept_seconds.reset();
      enter_hold(input, "lcp_hold", "car_status_timeout", "waiting_target");
    } else if (phase_ == "final_intercept" && intercept_due_at_ && input.now >= *intercept_due_at_) {
      const auto estimate = target_tracker_.estimate(input.now);
      if (std::hypot(estimate.x_m - input.telemetry.local_x_m,
          estimate.y_m - input.telemetry.local_y_m) <= mission_.throw_distance_m) {
        pending_release_gripper_ = true;
        transition("throwing", input.now);
      } else {
        intercept_due_at_.reset();
        plan_intercept(input, 0.0);
      }
    }
  } else if (phase_ == "throwing") {
    if (input.gripper_failed) {
      reject("gripper_release_failed");
      begin_return(input.now);
    } else if (input.gripper_succeeded) {
      emit(communication::MessageType::ok_throw);
      begin_return(input.now);
    }
  } else if (phase_ == "returning") {
    if (control_.origin && planner_.target_reached() && actual_xy_within(
        input.telemetry, control_.origin->x_m, control_.origin->y_m, config_.target_tolerance_m) &&
      actual_yaw_within(input.telemetry, 0.0, 0.10)) {
      emit(communication::MessageType::ok_return);
      auto descent_goal = *control_.origin;
      descent_goal.orientation = {0.0, 0.0, 0.0, 1.0};
      set_mission_goal(descent_goal);
      plan_to_mission_goal();
      normal_completion_ = true;
      emit(communication::MessageType::ok_downing);
      transition("downing", input.now);
    }
  } else if (phase_ == "lcp_hold") {
    if (input.lcp_healthy && control_.hold_reason == "lcp_unhealthy") {resume_hold(input.now);}
  } else if (phase_ == "downing" || phase_ == "landing") {
    const bool on_ground = input.telemetry.landed_state == common::MAV_LANDED_STATE_ON_GROUND;
    const bool at_origin = common::finite(input.telemetry.local_z_m) && control_.origin &&
      std::abs(input.telemetry.local_z_m - control_.origin->z_m) <= config_.touchdown_z_tolerance_m;
    if (on_ground || (planner_.target_reached() && at_origin)) {transition("disarming", input.now);}
  } else if (phase_ == "disarming") {
    if (!input.controller.armed) {transition("manual_request_pending", input.now);}
  } else if (phase_ == "manual_request_pending" &&
    input.controller.mode == "MANUAL" && !input.controller.armed) {
    if (normal_completion_) {emit(communication::MessageType::ok_down);}
    transition("manual", input.now);
  }

  const bool waiting_gcs_phase = phase_ == "waiting_run_plan1" || phase_ == "setpoint_warmup";
  if (waiting_gcs_phase && !input.preflight_ready) {reset();}
  const bool post_run_prearm_phase =
    phase_ == "offboard_request_pending" || phase_ == "arming_request_pending";
  if (post_run_prearm_phase && !input.controller.armed && !input.preflight_ready) {
    reset();
    transition("manual_request_pending", input.now);
    reject("preflight_lost_before_arm");
  }

  const bool airborne = phase_ != "waiting_preflight" && phase_ != "waiting_run_plan1" &&
    phase_ != "setpoint_warmup" && phase_ != "offboard_request_pending" &&
    phase_ != "arming_request_pending" && phase_ != "manual";
  if (airborne && phase_ != "landing" && phase_ != "downing" && phase_ != "disarming" &&
    phase_ != "manual_request_pending" && !input.flight_healthy) {
    begin_landing(input.now, input.health_errors.empty() ?
      "flight_health_failure" : input.health_errors.front());
  }
  if (airborne && phase_ != "landing" && phase_ != "downing" && phase_ != "disarming" &&
    phase_ != "manual_request_pending" && input.controller.mode != "OFFBOARD") {
    begin_landing(input.now, "offboard_mode_lost");
  }
  if (flight_started_at_ && airborne && phase_ != "landing" && phase_ != "downing" &&
    input.now - *flight_started_at_ > config_.max_flight_seconds) {
    begin_landing(input.now, "maximum_flight_time_exceeded");
  }

  NavigationDecision decision;
  decision.phase = phase_;
  decision.messages = pending_messages_;
  decision.rejections = pending_rejections_;
  decision.release_gripper = pending_release_gripper_;
  const bool ground_hold_stream = phase_ == "setpoint_warmup" || phase_ == "waiting_run_plan1" ||
    phase_ == "offboard_request_pending" || phase_ == "arming_request_pending";
  if (ground_hold_stream && common::finite(input.telemetry.local_x_m) &&
    common::finite(input.telemetry.local_y_m) && common::finite(input.telemetry.local_z_m)) {
    common::PositionSetpoint ground_hold{
      input.telemetry.local_x_m, input.telemetry.local_y_m, input.telemetry.local_z_m,
      common::normalize_quaternion(input.telemetry.orientation.x, input.telemetry.orientation.y,
        input.telemetry.orientation.z, input.telemetry.orientation.w), 0.0};
    decision.setpoint = ground_hold;
    control_.commanded_setpoint = ground_hold;
  } else if (planner_.latched() && phase_ != "waiting_preflight" && phase_ != "manual") {
    decision.setpoint = planner_.update(input.dt);
    control_.commanded_setpoint = decision.setpoint;
  }
  if (phase_ == "offboard_request_pending" || phase_ == "arming_request_pending" ||
    phase_ == "climb" || phase_ == "height_stabilizing" || phase_ == "transit_to_b" ||
    phase_ == "waiting_target" || phase_ == "cardinal_alignment" ||
    phase_ == "final_intercept" || phase_ == "throwing" || phase_ == "returning" ||
    phase_ == "lcp_hold" || phase_ == "downing") {
    decision.target_mode = "OFFBOARD";
  }
  if (phase_ == "landing" && !landing_reason_.empty()) {decision.target_mode = "AUTO.LAND";}
  if (phase_ == "arming_request_pending" || phase_ == "climb" ||
    phase_ == "height_stabilizing" || phase_ == "transit_to_b" ||
    phase_ == "waiting_target" || phase_ == "cardinal_alignment" ||
    phase_ == "final_intercept" || phase_ == "throwing" || phase_ == "returning" ||
    phase_ == "lcp_hold" || phase_ == "downing" || phase_ == "landing") {
    decision.arm_intent = true;
  }
  if (phase_ == "disarming" || phase_ == "manual_request_pending") {decision.arm_intent = false;}
  if (phase_ == "manual_request_pending" || phase_ == "manual") {decision.target_mode = "MANUAL";}
  decision.control = control_;
  return decision;
}

}  // namespace mavros_xyz_position_offboard::navigation
