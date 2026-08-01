#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "mavros_xyz_position_offboard/common/types.hpp"
#include "mavros_xyz_position_offboard/communication/protocol.hpp"
#include "mavros_xyz_position_offboard/navigation/navigation_types.hpp"
#include "mavros_xyz_position_offboard/navigation/navigation.hpp"

namespace
{
using mavros_xyz_position_offboard::common::SafetyConfig;
using mavros_xyz_position_offboard::communication::MessageType;
using mavros_xyz_position_offboard::communication::ProtocolEvent;
using mavros_xyz_position_offboard::navigation::ControlState;
using mavros_xyz_position_offboard::navigation::MissionConfig;
using mavros_xyz_position_offboard::navigation::Navigation;
using mavros_xyz_position_offboard::navigation::NavigationInput;
using mavros_xyz_position_offboard::navigation::NavigationDecision;
using mavros_xyz_position_offboard::navigation::TrajectoryPlanner;
using mavros_xyz_position_offboard::navigation::control_json;

SafetyConfig intercept_safety()
{
  SafetyConfig safety;
  safety.setpoint_warmup_s = 0.05;
  safety.target_xy_max_speed_m_s = 4.0;
  safety.target_xy_max_accel_m_s2 = 8.0;
  safety.max_z_setpoint_rate_m_s = 4.0;
  safety.max_z_setpoint_accel_m_s2 = 8.0;
  safety.target_tolerance_m = 0.03;
  safety.max_flight_seconds = 60.0;
  return safety;
}

ProtocolEvent event(MessageType type, double now)
{
  ProtocolEvent value;
  value.type = type;
  value.received_at = now;
  value.accepted = true;
  return value;
}

ProtocolEvent car_status_event(double distance_m, double bearing_rad, double now)
{
  auto value = event(MessageType::car_status, now);
  value.car_status = {distance_m, bearing_rad};
  return value;
}

bool has_message(const NavigationDecision & decision, MessageType type)
{
  for (const auto & message : decision.messages) {
    if (message.type == type) {return true;}
  }
  return false;
}

NavigationInput base_input(double now)
{
  NavigationInput input;
  input.now = now;
  input.dt = 0.05;
  input.preflight_ready = true;
  input.lcp_healthy = true;
  input.flight_healthy = true;
  input.telemetry.local_x_m = 0.0;
  input.telemetry.local_y_m = 0.0;
  input.telemetry.local_z_m = 0.0;
  input.telemetry.orientation = {0.0, 0.0, 0.0, 1.0};
  input.telemetry.velocity_x_m_s = 0.0;
  input.telemetry.velocity_y_m_s = 0.0;
  input.telemetry.velocity_z_m_s = 0.0;
  input.controller.mode = "MANUAL";
  return input;
}

void follow_planner(Navigation & navigation, NavigationInput & input)
{
  if (!navigation.planner().latched()) {return;}
  const auto setpoint = navigation.planner().current();
  input.telemetry.local_x_m = setpoint.x_m;
  input.telemetry.local_y_m = setpoint.y_m;
  input.telemetry.local_z_m = setpoint.z_m;
  input.telemetry.orientation = setpoint.orientation;
}

void advance_to_transit_to_b(Navigation & navigation, NavigationInput & input, double & now)
{
  const auto tick = [&]() {
      follow_planner(navigation, input);
      input.now = now;
      input.events.clear();
      navigation.update(input);
      now += input.dt;
    };
  tick();
  tick();
  input.events = {event(MessageType::run_plan1, now)};
  input.now = now;
  navigation.update(input);
  now += input.dt;
  input.controller.mode = "OFFBOARD";
  tick();
  input.controller.armed = true;
  for (int count = 0; count < 400 && navigation.phase() != "height_stabilizing"; ++count) {tick();}
  ASSERT_EQ(navigation.phase(), "height_stabilizing");
  for (int count = 0; count < 100 && navigation.phase() != "transit_to_b"; ++count) {tick();}
  ASSERT_EQ(navigation.phase(), "transit_to_b");
}

void finish_b_trajectory(Navigation & navigation, NavigationInput & input, double & now)
{
  for (int count = 0; count < 400 && !navigation.planner().target_reached(); ++count) {
    follow_planner(navigation, input);
    input.now = now;
    input.events.clear();
    navigation.update(input);
    now += input.dt;
  }
  ASSERT_TRUE(navigation.planner().target_reached());
}

void advance_to_b(Navigation & navigation, NavigationInput & input, double & now)
{
  advance_to_transit_to_b(navigation, input, now);
  finish_b_trajectory(navigation, input, now);
  follow_planner(navigation, input);
  input.now = now;
  input.events.clear();
  navigation.update(input);
  now += input.dt;
  ASSERT_EQ(navigation.phase(), "waiting_target");
}

TEST(NavigationTypesTest, EncodesControlJsonWithoutStateMachineDependency)
{
  ControlState control;
  control.mission_paused = true;
  control.hold_reason = "lcp_unhealthy";
  control.target_samples = 3;
  const std::string encoded = control_json(control);
  EXPECT_NE(encoded.find("\"mission_paused\":true"), std::string::npos);
  EXPECT_NE(encoded.find("\"hold_reason\":\"lcp_unhealthy\""), std::string::npos);
  EXPECT_NE(encoded.find("\"target_samples\":3"), std::string::npos);
}

TEST(NavigationV3Test, StabilizesThreeSecondsThenUsesFixedBPointAndEmitsOkBOnce)
{
  auto safety = intercept_safety();
  MissionConfig mission;
  mission.height_stable_seconds = 3.0;
  Navigation navigation(safety, mission);
  auto input = base_input(0.0);
  const double psi0 = std::acos(-1.0) / 2.0;
  input.telemetry.orientation = {0.0, 0.0, std::sin(psi0 / 2.0), std::cos(psi0 / 2.0)};
  input.dt = 0.05;
  double now = 0.0;

  const auto tick = [&]() {
      follow_planner(navigation, input);
      input.now = now;
      input.events.clear();
      const auto decision = navigation.update(input);
      now += input.dt;
      return decision;
    };
  tick();
  tick();
  input.events = {event(MessageType::run_plan1, now)};
  input.now = now;
  navigation.update(input);
  now += input.dt;
  input.controller.mode = "OFFBOARD";
  tick();
  input.controller.armed = true;
  for (int count = 0; count < 400 && navigation.phase() != "height_stabilizing"; ++count) {tick();}
  ASSERT_EQ(navigation.phase(), "height_stabilizing");
  const double stabilized_at = now;
  for (int count = 0; count < 59; ++count) {EXPECT_EQ(tick().phase, "height_stabilizing");}
  EXPECT_LT(now - stabilized_at, mission.height_stable_seconds + input.dt);
  for (int count = 0; count < 4 && navigation.phase() == "height_stabilizing"; ++count) {tick();}
  ASSERT_EQ(navigation.phase(), "transit_to_b");
  EXPECT_NEAR(navigation.planner().target_x_m(), mission.b_right_m, 1e-6);
  EXPECT_NEAR(navigation.planner().target_y_m(), mission.b_forward_m, 1e-6);
  EXPECT_NEAR(std::hypot(navigation.planner().target_x_m(), navigation.planner().target_y_m()),
    std::hypot(mission.b_forward_m, mission.b_right_m), 1e-6);

  int ok_b_count = 0;
  for (int count = 0; count < 400 && navigation.phase() != "waiting_target"; ++count) {
    const auto decision = tick();
    ok_b_count += has_message(decision, MessageType::ok_b) ? 1 : 0;
  }
  ASSERT_EQ(navigation.phase(), "waiting_target");
  for (int count = 0; count < 20; ++count) {
    ok_b_count += has_message(tick(), MessageType::ok_b) ? 1 : 0;
  }
  EXPECT_EQ(ok_b_count, 1);
}

TEST(NavigationV3Test, UsesFixedLocalEnuBPointForDifferentInitialYaws)
{
  const double pi = std::acos(-1.0);
  for (const double yaw : {0.0, 0.7, pi / 2.0, pi, -1.1}) {
    auto safety = intercept_safety();
    MissionConfig mission;
    mission.height_stable_seconds = 0.05;
    Navigation navigation(safety, mission);
    auto input = base_input(0.0);
    input.telemetry.local_x_m = 1.20;
    input.telemetry.local_y_m = -0.80;
    input.telemetry.orientation = {
      0.0, 0.0, std::sin(yaw / 2.0), std::cos(yaw / 2.0)};
    double now = 0.0;

    advance_to_transit_to_b(navigation, input, now);

    EXPECT_NEAR(navigation.planner().target_x_m(), mission.b_right_m, 1e-9);
    EXPECT_NEAR(navigation.planner().target_y_m(), mission.b_forward_m, 1e-9);
  }
}

TEST(NavigationV3Test, SendsOkBWithoutMeasuredBPointStabilityGate)
{
  auto safety = intercept_safety();
  MissionConfig mission;
  mission.height_stable_seconds = 0.05;
  Navigation navigation(safety, mission);
  auto input = base_input(0.0);
  input.dt = 0.05;
  double now = 0.0;
  advance_to_transit_to_b(navigation, input, now);
  finish_b_trajectory(navigation, input, now);

  follow_planner(navigation, input);
  input.telemetry.local_x_m += 0.20;
  input.telemetry.velocity_x_m_s = 0.50;
  input.telemetry.velocity_y_m_s = 0.50;
  input.telemetry.velocity_z_m_s = 0.50;
  input.now = now;
  input.events.clear();
  const auto decision = navigation.update(input);
  now += input.dt;
  EXPECT_EQ(decision.phase, "waiting_target");
  EXPECT_TRUE(has_message(decision, MessageType::ok_b));

  input.now = now;
  const auto after_ack = navigation.update(input);
  EXPECT_EQ(after_ack.phase, "waiting_target");
  EXPECT_FALSE(has_message(after_ack, MessageType::ok_b));
}

TEST(NavigationV3Test, SingleCarStatusDirectlyTargetsVehicleCenterWithYawConversion)
{
  auto safety = intercept_safety();
  MissionConfig mission;
  mission.height_stable_seconds = 0.05;
  Navigation navigation(safety, mission);
  auto input = base_input(0.0);
  input.dt = 0.05;
  double now = 0.0;
  const double yaw = 0.70;
  const double distance = 1.00;
  const double bearing = -0.25;
  input.telemetry.orientation = {0.0, 0.0, std::sin(yaw / 2.0), std::cos(yaw / 2.0)};
  advance_to_b(navigation, input, now);

  input.now = now;
  input.events = {car_status_event(distance, bearing, now)};
  const auto decision = navigation.update(input);

  const double expected_x = input.telemetry.local_x_m + distance * std::cos(yaw + bearing);
  const double expected_y = input.telemetry.local_y_m + distance * std::sin(yaw + bearing);
  EXPECT_EQ(decision.phase, "target_lock_following");
  EXPECT_FALSE(decision.release_gripper);
  EXPECT_NEAR(navigation.planner().target_x_m(), expected_x, 1e-9);
  EXPECT_NEAR(navigation.planner().target_y_m(), expected_y, 1e-9);
  ASSERT_TRUE(decision.control.mission_goal);
  EXPECT_NEAR(decision.control.mission_goal->x_m, expected_x, 1e-9);
  EXPECT_NEAR(decision.control.mission_goal->y_m, expected_y, 1e-9);
  EXPECT_NEAR(decision.control.mission_goal->z_m, 1.5, 1e-9);
  EXPECT_NEAR(mavros_xyz_position_offboard::common::yaw_from_quaternion(
      decision.control.mission_goal->orientation), yaw, 1e-9);
  EXPECT_EQ(decision.control.target_samples, 0);
  EXPECT_FALSE(decision.control.predicted_intercept_seconds);
}

TEST(NavigationV3Test, CarStatusUsesMissionLimitsWhileBUsesGlobalXyLimits)
{
  auto safety = intercept_safety();
  MissionConfig mission;
  mission.height_stable_seconds = 0.05;
  mission.car_tracking_max_speed_m_s = 0.35;
  mission.car_tracking_max_accel_m_s2 = 0.65;
  Navigation navigation(safety, mission);
  auto input = base_input(0.0);
  input.dt = 0.05;
  double now = 0.0;
  advance_to_transit_to_b(navigation, input, now);

  const double b_duration = navigation.planner().xy_trajectory_duration_s();
  TrajectoryPlanner expected_b(safety);
  expected_b.latch(0.0, 0.0, mission.takeoff_height_m, {0.0, 0.0, 0.0, 1.0});
  expected_b.set_xy_target(mission.b_right_m, mission.b_forward_m);
  EXPECT_NEAR(b_duration, expected_b.xy_trajectory_duration_s(), 1e-12);

  finish_b_trajectory(navigation, input, now);
  follow_planner(navigation, input);
  input.now = now;
  input.events.clear();
  ASSERT_EQ(navigation.update(input).phase, "waiting_target");
  now += input.dt;

  const double distance = 1.0;
  const double bearing = 0.0;
  const double target_x = input.telemetry.local_x_m + distance;
  const double target_y = input.telemetry.local_y_m;
  TrajectoryPlanner expected_global(safety);
  expected_global.latch(
    input.telemetry.local_x_m, input.telemetry.local_y_m, input.telemetry.local_z_m,
    input.telemetry.orientation);
  expected_global.set_xy_target(target_x, target_y);
  TrajectoryPlanner expected_tracking(safety);
  expected_tracking.latch(
    input.telemetry.local_x_m, input.telemetry.local_y_m, input.telemetry.local_z_m,
    input.telemetry.orientation);
  expected_tracking.set_xy_target_with_limits(
    target_x, target_y, mission.car_tracking_max_speed_m_s,
    mission.car_tracking_max_accel_m_s2);

  input.now = now;
  input.events = {car_status_event(distance, bearing, now)};
  const auto decision = navigation.update(input);
  EXPECT_EQ(decision.phase, "target_lock_following");
  EXPECT_NEAR(
    navigation.planner().xy_trajectory_duration_s(), expected_tracking.xy_trajectory_duration_s(),
    1e-12);
  EXPECT_GT(
    navigation.planner().xy_trajectory_duration_s(), expected_global.xy_trajectory_duration_s());
}

TEST(NavigationV3Test, SuccessfulThrowReturnsAndCompletesLandingWithoutAGcsReturnCommand)
{
  auto safety = intercept_safety();
  MissionConfig mission;
  mission.height_stable_seconds = 0.05;
  mission.target_lock_follow_seconds = 0.04;
  mission.return_max_speed_m_s = 0.35;
  mission.return_max_accel_m_s2 = 0.65;
  Navigation navigation(safety, mission);
  auto input = base_input(0.0);
  input.dt = 0.05;
  double now = 0.0;
  advance_to_b(navigation, input, now);

  input.now = now;
  input.events = {car_status_event(0.0, 0.0, now)};
  const auto locked_decision = navigation.update(input);
  EXPECT_EQ(locked_decision.phase, "target_lock_following");
  EXPECT_FALSE(locked_decision.release_gripper);
  now += input.dt;
  input.now = now;
  input.events.clear();
  const auto throw_decision = navigation.update(input);
  EXPECT_EQ(throw_decision.phase, "throwing");
  EXPECT_TRUE(throw_decision.release_gripper);
  EXPECT_EQ(throw_decision.control.target_samples, 0);
  EXPECT_FALSE(throw_decision.control.predicted_intercept_seconds);
  now += input.dt;

  const auto return_start = navigation.planner().current();
  const auto origin = *navigation.control_state().origin;
  TrajectoryPlanner expected_return(safety);
  expected_return.latch(
    return_start.x_m, return_start.y_m, return_start.z_m, return_start.orientation);
  expected_return.set_xy_target_with_limits(
    origin.x_m, origin.y_m, mission.return_max_speed_m_s, mission.return_max_accel_m_s2);
  expected_return.set_z_target(return_start.z_m);
  expected_return.set_yaw_rad(0.0);
  expected_return.update(input.dt);

  TrajectoryPlanner expected_global(safety);
  expected_global.latch(
    return_start.x_m, return_start.y_m, return_start.z_m, return_start.orientation);
  expected_global.set_xy_target(origin.x_m, origin.y_m);
  expected_global.set_z_target(return_start.z_m);
  expected_global.set_yaw_rad(0.0);
  expected_global.update(input.dt);

  input.now = now;
  input.events.clear();
  input.gripper_succeeded = true;
  const auto released = navigation.update(input);
  input.gripper_succeeded = false;
  EXPECT_EQ(released.phase, "returning");
  EXPECT_TRUE(has_message(released, MessageType::ok_throw));
  EXPECT_NEAR(navigation.planner().target_x_m(), origin.x_m, 1e-12);
  EXPECT_NEAR(navigation.planner().target_y_m(), origin.y_m, 1e-12);
  EXPECT_NEAR(
    navigation.planner().xy_trajectory_duration_s(), expected_return.xy_trajectory_duration_s(),
    1e-12);
  EXPECT_GT(
    navigation.planner().xy_trajectory_duration_s(), expected_global.xy_trajectory_duration_s());

  bool saw_return = false;
  bool saw_downing = false;
  for (int count = 0; count < 400 && navigation.phase() != "downing"; ++count) {
    follow_planner(navigation, input);
    input.now = now;
    input.events.clear();
    const auto decision = navigation.update(input);
    saw_return = saw_return || has_message(decision, MessageType::ok_return);
    saw_downing = saw_downing || has_message(decision, MessageType::ok_downing);
    now += input.dt;
  }
  ASSERT_EQ(navigation.phase(), "downing");
  EXPECT_TRUE(saw_return);
  EXPECT_TRUE(saw_downing);

  for (int count = 0; count < 400 && navigation.phase() != "disarming"; ++count) {
    follow_planner(navigation, input);
    input.now = now;
    input.events.clear();
    navigation.update(input);
    now += input.dt;
  }
  input.telemetry.landed_state = mavros_xyz_position_offboard::common::MAV_LANDED_STATE_ON_GROUND;
  input.now = now;
  navigation.update(input);
  EXPECT_EQ(navigation.phase(), "disarming");
  input.controller.armed = false;
  input.now += input.dt;
  navigation.update(input);
  EXPECT_EQ(navigation.phase(), "manual_request_pending");
  input.controller.mode = "MANUAL";
  input.now += input.dt;
  const auto completed = navigation.update(input);
  EXPECT_EQ(completed.phase, "manual");
  EXPECT_TRUE(has_message(completed, MessageType::ok_down));
}

TEST(NavigationV3Test, NewCarStatusImmediatelyUpdatesRawVehicleCenterTarget)
{
  auto safety = intercept_safety();
  MissionConfig mission;
  mission.height_stable_seconds = 0.05;
  Navigation navigation(safety, mission);
  auto input = base_input(0.0);
  input.dt = 0.05;
  double now = 0.0;
  advance_to_b(navigation, input, now);

  const double uav_x = input.telemetry.local_x_m;
  const double uav_y = input.telemetry.local_y_m;
  const double yaw = 0.4;
  input.telemetry.orientation = {0.0, 0.0, std::sin(yaw / 2.0), std::cos(yaw / 2.0)};
  const double first_distance = 1.0;
  const double first_bearing = 0.2;
  input.now = now;
  input.events = {car_status_event(first_distance, first_bearing, now)};
  const auto first = navigation.update(input);
  const double first_x = uav_x + first_distance * std::cos(yaw + first_bearing);
  const double first_y = uav_y + first_distance * std::sin(yaw + first_bearing);
  EXPECT_EQ(first.phase, "target_lock_following");
  EXPECT_NEAR(navigation.planner().target_x_m(), first_x, 1e-9);
  EXPECT_NEAR(navigation.planner().target_y_m(), first_y, 1e-9);

  const double second_distance = 2.0;
  const double second_bearing = 2.4;
  now += input.dt;
  input.now = now;
  input.events = {car_status_event(second_distance, second_bearing, now)};
  const auto second = navigation.update(input);
  const double second_x = uav_x + second_distance * std::cos(yaw + second_bearing);
  const double second_y = uav_y + second_distance * std::sin(yaw + second_bearing);
  EXPECT_EQ(second.phase, "target_lock_following");
  EXPECT_TRUE(second.rejections.empty());
  EXPECT_NEAR(navigation.planner().target_x_m(), second_x, 1e-9);
  EXPECT_NEAR(navigation.planner().target_y_m(), second_y, 1e-9);
  EXPECT_EQ(second.control.target_samples, 0);
  EXPECT_FALSE(second.control.predicted_intercept_seconds);
}

TEST(NavigationV3Test, RawDistanceBelowThrowThresholdWaitsForTargetLock)
{
  auto safety = intercept_safety();
  MissionConfig mission;
  mission.height_stable_seconds = 0.05;
  mission.target_lock_follow_seconds = 0.09;
  Navigation navigation(safety, mission);
  auto input = base_input(0.0);
  input.dt = 0.05;
  double now = 0.0;
  advance_to_b(navigation, input, now);

  input.now = now;
  input.events = {car_status_event(mission.throw_distance_m - 0.01, 0.0, now)};
  const auto locked = navigation.update(input);
  EXPECT_EQ(locked.phase, "target_lock_following");
  EXPECT_FALSE(locked.release_gripper);

  now += input.dt;
  input.now = now;
  input.events.clear();
  const auto still_locked = navigation.update(input);
  EXPECT_EQ(still_locked.phase, "target_lock_following");
  EXPECT_FALSE(still_locked.release_gripper);

  now += input.dt;
  input.now = now;
  const auto decision = navigation.update(input);
  EXPECT_EQ(decision.phase, "throwing");
  EXPECT_TRUE(decision.release_gripper);
}

TEST(NavigationV3Test, LcpFailureCancelsSameCycleRawReleaseAndEntersHold)
{
  auto safety = intercept_safety();
  MissionConfig mission;
  mission.height_stable_seconds = 0.05;
  mission.target_lock_follow_seconds = 0.04;
  Navigation navigation(safety, mission);
  auto input = base_input(0.0);
  input.dt = 0.05;
  double now = 0.0;
  advance_to_b(navigation, input, now);

  input.now = now;
  input.lcp_healthy = false;
  input.events = {car_status_event(0.15, 0.0, now)};
  const auto decision = navigation.update(input);
  EXPECT_EQ(decision.phase, "lcp_hold");
  EXPECT_FALSE(decision.release_gripper);
  EXPECT_TRUE(decision.control.mission_paused);
  EXPECT_EQ(decision.control.hold_reason, "lcp_unhealthy");
}

TEST(NavigationV3Test, ThrowDistanceBoundaryDoesNotRelease)
{
  auto safety = intercept_safety();
  MissionConfig mission;
  mission.height_stable_seconds = 0.05;
  mission.target_lock_follow_seconds = 0.04;
  Navigation navigation(safety, mission);
  auto input = base_input(0.0);
  input.dt = 0.05;
  double now = 0.0;
  advance_to_b(navigation, input, now);

  input.now = now;
  input.events = {car_status_event(mission.throw_distance_m, 1.2, now)};
  const auto locked = navigation.update(input);
  EXPECT_EQ(locked.phase, "target_lock_following");
  EXPECT_FALSE(locked.release_gripper);
  now += input.dt;
  input.now = now;
  input.events.clear();
  const auto decision = navigation.update(input);
  EXPECT_EQ(decision.phase, "waiting_target");
  EXPECT_FALSE(decision.release_gripper);
  EXPECT_EQ(decision.control.target_samples, 0);
  EXPECT_FALSE(decision.control.predicted_intercept_seconds);

  now += input.dt;
  input.now = now;
  input.events = {car_status_event(mission.throw_distance_m - 0.01, 1.2, now)};
  const auto normal_tracking_release = navigation.update(input);
  EXPECT_EQ(normal_tracking_release.phase, "throwing");
  EXPECT_TRUE(normal_tracking_release.release_gripper);
}

TEST(NavigationV3Test, TrackingRadiusViolationStillEntersLcpHold)
{
  auto safety = intercept_safety();
  MissionConfig mission;
  mission.height_stable_seconds = 0.05;
  Navigation navigation(safety, mission);
  auto input = base_input(0.0);
  input.dt = 0.05;
  double now = 0.0;
  advance_to_b(navigation, input, now);

  input.now = now;
  input.events = {car_status_event(5.0, std::acos(-1.0) / 2.0, now)};
  const auto decision = navigation.update(input);
  EXPECT_EQ(decision.phase, "lcp_hold");
  EXPECT_FALSE(decision.release_gripper);
  EXPECT_TRUE(decision.control.mission_paused);
  ASSERT_FALSE(decision.rejections.empty());
  EXPECT_EQ(decision.rejections.back(), "car_status_target_outside_tracking_radius");
}

TEST(NavigationV3Test, CarStatusTimeoutStillEntersLcpHold)
{
  auto safety = intercept_safety();
  MissionConfig mission;
  mission.height_stable_seconds = 0.05;
  Navigation navigation(safety, mission);
  auto input = base_input(0.0);
  input.dt = 0.10;
  double now = 0.0;
  advance_to_b(navigation, input, now);

  input.now = now;
  input.events = {car_status_event(1.0, 0.0, now)};
  const auto observed = navigation.update(input);
  EXPECT_EQ(observed.phase, "target_lock_following");
  now += mission.car_status_timeout_s + input.dt;
  input.now = now;
  input.events.clear();
  const auto timed_out = navigation.update(input);
  EXPECT_EQ(timed_out.phase, "lcp_hold");
  EXPECT_TRUE(timed_out.control.mission_paused);
  EXPECT_EQ(timed_out.control.hold_reason, "car_status_timeout");
}

TEST(NavigationV3Test, TargetLockLcpHoldPausesAndResumesRemainingTime)
{
  auto safety = intercept_safety();
  MissionConfig mission;
  mission.height_stable_seconds = 0.05;
  mission.target_lock_follow_seconds = 1.0;
  Navigation navigation(safety, mission);
  auto input = base_input(0.0);
  input.dt = 0.10;
  double now = 0.0;
  advance_to_b(navigation, input, now);

  input.now = now;
  input.events = {car_status_event(1.0, 0.0, now)};
  ASSERT_EQ(navigation.update(input).phase, "target_lock_following");

  now += 0.4;
  input.now = now;
  input.lcp_healthy = false;
  input.events.clear();
  const auto held = navigation.update(input);
  EXPECT_EQ(held.phase, "lcp_hold");
  EXPECT_FALSE(held.release_gripper);

  now += 1.0;
  input.now = now;
  input.lcp_healthy = true;
  const auto resumed = navigation.update(input);
  EXPECT_EQ(resumed.phase, "target_lock_following");
  EXPECT_FALSE(resumed.release_gripper);

  now += 0.4;
  input.now = now;
  input.events = {car_status_event(1.0, 0.0, now)};
  const auto active = navigation.update(input);
  EXPECT_EQ(active.phase, "target_lock_following");
  EXPECT_FALSE(active.release_gripper);

  now += 0.2;
  input.now = now;
  input.events.clear();
  const auto completed = navigation.update(input);
  EXPECT_EQ(completed.phase, "waiting_target");
  EXPECT_FALSE(completed.release_gripper);
}

TEST(NavigationV3Test, TargetLockTimeoutPausesAtObservationTimeoutAndResumesRemainingTime)
{
  auto safety = intercept_safety();
  MissionConfig mission;
  mission.height_stable_seconds = 0.05;
  mission.target_lock_follow_seconds = 3.0;
  Navigation navigation(safety, mission);
  auto input = base_input(0.0);
  input.dt = 0.10;
  double now = 0.0;
  advance_to_b(navigation, input, now);

  input.now = now;
  input.events = {car_status_event(1.0, 0.0, now)};
  ASSERT_EQ(navigation.update(input).phase, "target_lock_following");

  now += 0.4;
  input.now = now;
  input.events = {car_status_event(1.0, 0.0, now)};
  ASSERT_EQ(navigation.update(input).phase, "target_lock_following");

  now += mission.car_status_timeout_s + 0.1;
  input.now = now;
  input.events.clear();
  const auto timed_out = navigation.update(input);
  EXPECT_EQ(timed_out.phase, "lcp_hold");
  EXPECT_FALSE(timed_out.release_gripper);

  input.events = {car_status_event(1.0, 0.0, now)};
  const auto resumed = navigation.update(input);
  EXPECT_EQ(resumed.phase, "target_lock_following");
  EXPECT_FALSE(resumed.release_gripper);

  now += 0.5;
  input.now = now;
  input.events = {car_status_event(1.0, 0.0, now)};
  EXPECT_EQ(navigation.update(input).phase, "target_lock_following");

  now += 0.1;
  input.now = now;
  input.events.clear();
  const auto completed = navigation.update(input);
  EXPECT_EQ(completed.phase, "waiting_target");
  EXPECT_FALSE(completed.release_gripper);
}

}  // namespace
