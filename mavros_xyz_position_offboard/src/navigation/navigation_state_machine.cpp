#include "mavros_xyz_position_offboard/navigation/navigation.hpp"

#include <cmath>
#include <stdexcept>

namespace mavros_xyz_position_offboard::navigation
{

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
      } else if (phase_ == "waiting_target" || phase_ == "target_lock_following") {
        const bool begin_lock = phase_ == "waiting_target" && !target_lock_follow_completed_;
        const bool allow_release = phase_ == "waiting_target" && target_lock_follow_completed_;
        if (apply_car_status(input, *event.car_status)) {
          if (begin_lock) {
            begin_target_lock_follow(input.now);
          } else if (allow_release && event.car_status->distance_m < mission_.throw_distance_m) {
            pending_release_gripper_ = true;
            transition("throwing", input.now);
          }
        }
      } else if (phase_ == "lcp_hold" && control_.hold_reason != "lcp_unhealthy") {
        const std::string resume_phase = control_.hold_resume_phase.empty() ?
          "waiting_target" : control_.hold_resume_phase;
        const bool resume_lock = resume_phase == "target_lock_following";
        clear_hold();
        transition(resume_phase, input.now);
        if (resume_lock) {resume_target_lock_follow(input.now);}
        if (apply_car_status(input, *event.car_status)) {
          if (!resume_lock && !target_lock_follow_completed_) {
            begin_target_lock_follow(input.now);
          } else if (!resume_lock && target_lock_follow_completed_ &&
            event.car_status->distance_m < mission_.throw_distance_m) {
            pending_release_gripper_ = true;
            transition("throwing", input.now);
          }
        }
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
    // B is a commanded transit waypoint; do not add a measured settle gate here.
    if (planner_.target_reached()) {
      emit(communication::MessageType::ok_b);
      transition("waiting_target", input.now);
    }
  } else if (phase_ == "waiting_target") {
    if (last_car_status_at_ && !car_status_fresh(input.now)) {
      control_.target_samples = 0;
      control_.predicted_intercept_seconds.reset();
      last_car_status_at_.reset();
      enter_hold(input, "lcp_hold", "car_status_timeout", "waiting_target");
    }
  } else if (phase_ == "target_lock_following") {
    if (last_car_status_at_ && !car_status_fresh(input.now)) {
      const double timeout_at = *last_car_status_at_ + mission_.car_status_timeout_s;
      pause_target_lock_follow(input.now < timeout_at ? input.now : timeout_at);
      control_.target_samples = 0;
      control_.predicted_intercept_seconds.reset();
      last_car_status_at_.reset();
      enter_hold(input, "lcp_hold", "car_status_timeout", "target_lock_following");
    } else if (target_lock_follow_elapsed(input.now) >= mission_.target_lock_follow_seconds) {
      target_lock_follow_completed_ = true;
      if (latest_car_status_ && car_status_fresh(input.now) &&
        latest_car_status_->distance_m < mission_.throw_distance_m) {
        pending_release_gripper_ = true;
        transition("throwing", input.now);
      } else {
        transition("waiting_target", input.now);
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
    phase_ == "waiting_target" || phase_ == "target_lock_following" ||
    phase_ == "throwing" || phase_ == "returning" ||
    phase_ == "lcp_hold" || phase_ == "downing") {
    decision.target_mode = "OFFBOARD";
  }
  if (phase_ == "landing" && !landing_reason_.empty()) {decision.target_mode = "AUTO.LAND";}
  if (phase_ == "arming_request_pending" || phase_ == "climb" ||
    phase_ == "height_stabilizing" || phase_ == "transit_to_b" ||
    phase_ == "waiting_target" || phase_ == "target_lock_following" ||
    phase_ == "throwing" || phase_ == "returning" ||
    phase_ == "lcp_hold" || phase_ == "downing" || phase_ == "landing") {
    decision.arm_intent = true;
  }
  if (phase_ == "disarming" || phase_ == "manual_request_pending") {decision.arm_intent = false;}
  if (phase_ == "manual_request_pending" || phase_ == "manual") {decision.target_mode = "MANUAL";}
  decision.control = control_;
  return decision;
}

}  // namespace mavros_xyz_position_offboard::navigation
