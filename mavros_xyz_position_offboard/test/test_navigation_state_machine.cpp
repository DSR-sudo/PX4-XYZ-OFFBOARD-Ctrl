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

void advance_to_b(Navigation & navigation, NavigationInput & input, double & now)
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
  for (int count = 0; count < 400 && navigation.phase() != "waiting_target"; ++count) {tick();}
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

TEST(NavigationV3Test, StabilizesThreeSecondsThenUsesTheDiagonalBPointAndEmitsOkBOnce)
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

TEST(NavigationV3Test, ShapesCardinalTranslationThenUnlocksForFinalInterceptAndRejectsBadObservations)
{
  auto safety = intercept_safety();
  MissionConfig mission;
  mission.height_stable_seconds = 0.05;
  Navigation navigation(safety, mission);
  auto input = base_input(0.0);
  input.dt = 0.05;
  double now = 0.0;
  advance_to_b(navigation, input, now);

  for (int sample = 0; sample < 3; ++sample) {
    input.now = now;
    input.events = {car_status_event(1.414 - 0.071 * sample, std::acos(-1.0) / 4.0, now)};
    const auto decision = navigation.update(input);
    follow_planner(navigation, input);
    now += input.dt;
    if (sample == 2) {
      EXPECT_EQ(decision.phase, "cardinal_alignment");
      EXPECT_NEAR(mavros_xyz_position_offboard::common::yaw_from_quaternion(
          navigation.commanded_setpoint()->orientation), 0.0, 1e-9);
    }
  }
  EXPECT_NEAR(navigation.planner().target_x_m(), input.telemetry.local_x_m, 0.15);
  EXPECT_GT(navigation.planner().target_y_m(), input.telemetry.local_y_m);

  const double b_x = mission.b_forward_m;
  const double b_y = -mission.b_right_m;
  input.telemetry.local_x_m = b_x + 0.85 - 0.45 * std::cos(0.35);
  input.telemetry.local_y_m = b_y + 0.85 - 0.45 * std::sin(0.35);
  input.telemetry.velocity_x_m_s = 0.0;
  input.telemetry.velocity_y_m_s = 0.0;
  input.now = now;
  input.events = {car_status_event(0.45, 0.35, now)};
  const auto final = navigation.update(input);
  EXPECT_EQ(final.phase, "final_intercept");
  EXPECT_NEAR(mavros_xyz_position_offboard::common::yaw_from_quaternion(final.setpoint->orientation), 0.0, 1e-9);

  input.now += mission.car_status_timeout_s + input.dt;
  input.events.clear();
  const auto timed_out = navigation.update(input);
  EXPECT_EQ(timed_out.phase, "lcp_hold");
  EXPECT_TRUE(timed_out.control.mission_paused);
}

TEST(NavigationV3Test, SuccessfulThrowReturnsAndCompletesLandingWithoutAGcsReturnCommand)
{
  auto safety = intercept_safety();
  MissionConfig mission;
  mission.height_stable_seconds = 0.05;
  Navigation navigation(safety, mission);
  auto input = base_input(0.0);
  input.dt = 0.05;
  double now = 0.0;
  advance_to_b(navigation, input, now);

  for (int sample = 0; sample < 3; ++sample) {
    input.now = now;
    input.events = {car_status_event(0.15, 0.0, now)};
    const auto decision = navigation.update(input);
    follow_planner(navigation, input);
    now += input.dt;
    if (sample == 2) {
      EXPECT_EQ(decision.phase, "throwing");
      EXPECT_TRUE(decision.release_gripper);
    }
  }

  input.now = now;
  input.events.clear();
  input.gripper_succeeded = true;
  const auto released = navigation.update(input);
  input.gripper_succeeded = false;
  EXPECT_EQ(released.phase, "returning");
  EXPECT_TRUE(has_message(released, MessageType::ok_throw));

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

}  // namespace
