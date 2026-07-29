#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>

#include <gtest/gtest.h>
#include <json/json.h>
#include <rclcpp/rclcpp.hpp>

#include "mavros_xyz_position_offboard/bridge/lcp_vision_bridge.hpp"
#include "mavros_xyz_position_offboard/common/cli.hpp"
#include "mavros_xyz_position_offboard/communication/ground_station_link.hpp"
#include "mavros_xyz_position_offboard/gripper/pwm_gripper.hpp"
#include "mavros_xyz_position_offboard/initialization/initialization.hpp"
#include "mavros_xyz_position_offboard/navigation/navigation.hpp"
#include "mavros_xyz_position_offboard/offboard/offboard.hpp"

namespace
{
using mavros_xyz_position_offboard::common::AppOptions;
using mavros_xyz_position_offboard::common::SafetyConfig;
using mavros_xyz_position_offboard::communication::CarStatus;
using mavros_xyz_position_offboard::communication::GroundStationConfig;
using mavros_xyz_position_offboard::communication::GroundStationLink;
using mavros_xyz_position_offboard::communication::MessageType;
using mavros_xyz_position_offboard::communication::OutgoingMessage;
using mavros_xyz_position_offboard::communication::ProtocolEvent;
using mavros_xyz_position_offboard::communication::RosHeader;
using mavros_xyz_position_offboard::communication::XyzStatus;
using mavros_xyz_position_offboard::gripper::PwmGripper;
using mavros_xyz_position_offboard::gripper::PwmGripperConfig;
using mavros_xyz_position_offboard::gripper::ReleaseState;
using mavros_xyz_position_offboard::initialization::Initialization;
using mavros_xyz_position_offboard::navigation::MissionConfig;
using mavros_xyz_position_offboard::navigation::Navigation;
using mavros_xyz_position_offboard::navigation::NavigationInput;
using mavros_xyz_position_offboard::navigation::TrajectoryPlanner;
using mavros_xyz_position_offboard::navigation::control_json;

GroundStationLink disabled_link(double max_distance_m = 5.0)
{
  GroundStationConfig config;
  config.enabled = false;
  config.whitelist_ip = "127.0.0.1";
  config.whitelist_port = 5010;
  config.max_tracking_distance_m = max_distance_m;
  return GroundStationLink(config);
}

ProtocolEvent event(MessageType type, double now)
{
  ProtocolEvent value;
  value.type = type;
  value.received_at = now;
  value.accepted = true;
  return value;
}

ProtocolEvent car_event(double distance_m, double angle_deg, double now)
{
  auto value = event(MessageType::car_status, now);
  value.car_status = CarStatus{distance_m, angle_deg};
  return value;
}

bool has_message(
  const mavros_xyz_position_offboard::navigation::NavigationDecision & decision, MessageType type)
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

TEST(ProtocolV2Test, AcceptsEveryInboundMessageWithStrictDataRules)
{
  auto link = disabled_link();
  for (const std::string json : {
      R"({"header":"run_plan1","data":{}})",
      R"({"header":"match_car_ok","data":{}})",
      R"({"header":"b_ok","data":{}})",
      R"({"header":"ack","data":{}})",
      R"({"header":"car_status","data":{"distance":0.1,"angle":-180}})"}) {
    EXPECT_TRUE(link.decode_datagram(json, "127.0.0.1", 5010, 1.0).accepted) << json;
  }
}

TEST(ProtocolV2Test, RejectsWrongDirectionMalformedAndOutOfRangePackets)
{
  auto link = disabled_link();
  const auto reject = [&link](const std::string & json, const std::string & reason) {
      const auto decoded = link.decode_datagram(json, "127.0.0.1", 5010, 1.0);
      EXPECT_FALSE(decoded.accepted) << json;
      EXPECT_EQ(decoded.rejection_reason, reason);
    };
  reject(R"({"header":"ok_wait","data":{}})", "unknown_or_wrong_direction_header");
  reject(R"({"header":"run_plan1"})", "invalid_envelope");
  reject(R"({"header":"run_plan1","data":{"unexpected":1}})", "nonempty_event_data");
  reject(R"({"header":"car_status","data":{"distance":1}})", "invalid_car_status_data");
  reject(R"({"header":"car_status","data":{"distance":"1","angle":0}})", "invalid_car_status_data");
  reject(R"({"header":"car_status","data":{"distance":1e999,"angle":0}})", "invalid_json");
  reject(R"({"header":"car_status","data":{"distance":1,"angle":181}})", "car_status_out_of_range");
  reject(R"({"header":"car_status","data":{"distance":1,"angle":0,"extra":1}})", "invalid_car_status_data");
  EXPECT_FALSE(link.decode_datagram(
    R"({"header":"ack","data":{}})", "127.0.0.2", 5010, 1.0).accepted);
  EXPECT_FALSE(link.decode_datagram(
    R"({"header":"ack","data":{}})", "127.0.0.1", 5011, 1.0).accepted);
  EXPECT_FALSE(link.decode_datagram(
    R"({"header":"ack","header":"ack","data":{}})", "127.0.0.1", 5010, 1.0).accepted);
}

TEST(ProtocolV2Test, AckRemovesOnlyEarliestQueuedEventAndXyzstatusIsNeverQueued)
{
  auto link = disabled_link();
  EXPECT_FALSE(link.send({MessageType::ok_wait}, 0.0));
  EXPECT_FALSE(link.send({MessageType::ok_height}, 0.1));
  ASSERT_EQ(link.pending_event_count(), 2U);
  EXPECT_EQ(*link.pending_event_json(), R"({"header":"ok_wait","data":{}})");
  EXPECT_FALSE(link.retry_events(0.49));
  EXPECT_FALSE(link.retry_events(0.50));
  EXPECT_TRUE(link.decode_datagram(
    R"({"header":"ack","data":{}})", "127.0.0.1", 5010, 0.6).accepted);
  ASSERT_EQ(link.pending_event_count(), 1U);
  EXPECT_EQ(*link.pending_event_json(), R"({"header":"ok_height","data":{}})");
  EXPECT_TRUE(link.decode_datagram(
    R"({"header":"ack","data":{}})", "127.0.0.1", 5010, 0.7).accepted);
  EXPECT_EQ(link.pending_event_count(), 0U);
  EXPECT_TRUE(link.decode_datagram(
    R"({"header":"ack","data":{}})", "127.0.0.1", 5010, 0.8).accepted);
  EXPECT_EQ(link.pending_event_count(), 0U);
}

TEST(ProtocolV2Test, EncodesFullLcpXyzstatusAndRequiredInvalidZNulls)
{
  auto link = disabled_link();
  XyzStatus status;
  status.header = {1784984021, 999549102, "lcp_map"};
  status.status = 2;
  status.map_locked = true;
  status.pose_valid = true;
  status.position_x_m = 1.2;
  status.position_y_m = -0.4;
  status.yaw_rad = 0.3;
  status.front_distance_m = 2.0;
  status.rear_distance_m = 1.0;
  status.left_distance_m = 3.0;
  status.right_distance_m = 4.0;
  status.map_size_x_m = 5.0;
  status.map_size_y_m = 6.0;
  Json::CharReaderBuilder reader;
  Json::Value root;
  std::string errors;
  std::istringstream invalid(link.encode_xyzstatus(status));
  ASSERT_TRUE(Json::parseFromStream(reader, invalid, &root, &errors));
  EXPECT_EQ(root["header"].asString(), "xyzstatus");
  const auto & data = root["data"];
  EXPECT_EQ(data["header"]["stamp"]["sec"].asInt(), 1784984021);
  EXPECT_EQ(data["header"]["stamp"]["nanosec"].asUInt(), 999549102U);
  EXPECT_EQ(data["header"]["frame_id"].asString(), "lcp_map");
  EXPECT_DOUBLE_EQ(data["front_distance_m"].asDouble(), 2.0);
  EXPECT_DOUBLE_EQ(data["map_size_y_m"].asDouble(), 6.0);
  EXPECT_TRUE(data["position_z_m"].isNull());
  EXPECT_EQ(data["z_source"].asString(), "none");
  EXPECT_TRUE(data["z_source_stamp"].isNull());
  EXPECT_TRUE(data["z_quality"].isNull());
  EXPECT_FALSE(data["z_valid"].asBool());

  status.position_z_m = 1.5;
  status.z_source = "local_pose";
  status.z_source_stamp = mavros_xyz_position_offboard::common::RosTimestamp{9, 7};
  status.z_valid = true;
  std::istringstream valid(link.encode_xyzstatus(status));
  ASSERT_TRUE(Json::parseFromStream(reader, valid, &root, &errors));
  EXPECT_DOUBLE_EQ(root["data"]["position_z_m"].asDouble(), 1.5);
  EXPECT_EQ(root["data"]["z_source_stamp"]["sec"].asInt(), 9);
  EXPECT_TRUE(root["data"]["z_valid"].asBool());
}

TEST(TrajectoryPlannerTest, QuinticSetpointsObserveBoundsAndReplanContinuously)
{
  SafetyConfig config;
  config.max_z_setpoint_rate_m_s = 0.20;
  config.max_z_setpoint_accel_m_s2 = 0.40;
  TrajectoryPlanner planner(config);
  planner.latch(0.0, 0.0, 0.0, {0.0, 0.0, 0.0, 1.0});
  planner.set_target(1.0, 1.0, 0.8);
  auto previous = planner.current();
  for (int i = 0; i < 200; ++i) {
    const auto current = planner.update(0.01);
    EXPECT_LE(std::hypot(current.x_m - previous.x_m, current.y_m - previous.y_m) / 0.01,
      config.target_xy_max_speed_m_s * 1.002);
    EXPECT_LE(std::abs(current.vertical_rate_m_s), config.max_z_setpoint_rate_m_s * 1.002);
    previous = current;
  }
}

TEST(NavigationV2Test, CompleteMissionUsesPolarTrackingAndOrderedEvents)
{
  SafetyConfig safety;
  safety.setpoint_warmup_s = 0.01;
  safety.max_flight_seconds = 200.0;
  MissionConfig mission;
  mission.takeoff_height_m = 1.5;
  mission.standoff_m = 0.10;
  mission.match_tolerance_m = 0.10;
  mission.car_status_timeout_s = 0.5;
  Navigation navigation(safety, mission);
  auto input = base_input(0.0);
  auto decision = navigation.update(input);
  EXPECT_EQ(navigation.phase(), "setpoint_warmup");
  EXPECT_FALSE(decision.control.origin);
  EXPECT_FALSE(decision.control.mission_goal);
  EXPECT_FALSE(has_message(decision, MessageType::ok_wait));
  ASSERT_TRUE(decision.setpoint);
  EXPECT_NEAR(decision.setpoint->z_m, 0.0, 1e-9);

  input.now += 0.01;
  decision = navigation.update(input);
  EXPECT_EQ(navigation.phase(), "waiting_run_plan1");
  EXPECT_TRUE(has_message(decision, MessageType::ok_wait));
  ASSERT_TRUE(decision.setpoint);
  EXPECT_NEAR(decision.setpoint->z_m, 0.0, 1e-9);

  input.now += 0.20;
  decision = navigation.update(input);
  EXPECT_EQ(navigation.phase(), "waiting_run_plan1");
  EXPECT_FALSE(decision.target_mode);
  EXPECT_FALSE(decision.arm_intent);
  ASSERT_TRUE(decision.setpoint);
  EXPECT_NEAR(decision.setpoint->z_m, 0.0, 1e-9);

  input.now += 0.01;
  input.events = {event(MessageType::run_plan1, input.now)};
  decision = navigation.update(input);
  input.events.clear();
  EXPECT_EQ(navigation.phase(), "offboard_request_pending");
  input.controller.mode = "OFFBOARD"; input.now += 0.01;
  navigation.update(input);
  input.telemetry.local_x_m = 0.25;
  input.telemetry.local_y_m = -0.10;
  input.telemetry.local_z_m = 0.40;
  input.controller.armed = true; input.now += 0.01;
  decision = navigation.update(input);
  EXPECT_EQ(navigation.phase(), "climb");
  ASSERT_TRUE(decision.control.origin);
  ASSERT_TRUE(decision.control.mission_goal);
  EXPECT_NEAR(decision.control.origin->x_m, 0.25, 1e-9);
  EXPECT_NEAR(decision.control.origin->y_m, -0.10, 1e-9);
  EXPECT_NEAR(decision.control.origin->z_m, 0.40, 1e-9);
  EXPECT_NEAR(decision.control.mission_goal->z_m, 1.90, 1e-9);
  for (int i = 0; i < 800 && navigation.phase() == "climb"; ++i) {
    input.now += 0.05;
    follow_planner(navigation, input);
    decision = navigation.update(input);
  }
  ASSERT_EQ(navigation.phase(), "height_stabilizing");
  const double height_stabilizing_started_at = input.now;
  for (int i = 0; i < 59; ++i) {
    input.now += 0.05;
    follow_planner(navigation, input);
    decision = navigation.update(input);
    EXPECT_EQ(navigation.phase(), "height_stabilizing");
    EXPECT_FALSE(has_message(decision, MessageType::ok_height));
  }
  for (int i = 0; i < 4 && navigation.phase() == "height_stabilizing"; ++i) {
    input.now += 0.05;
    follow_planner(navigation, input);
    decision = navigation.update(input);
  }
  EXPECT_GE(input.now - height_stabilizing_started_at, mission.height_stable_seconds);
  ASSERT_EQ(navigation.phase(), "tracking");
  EXPECT_TRUE(has_message(decision, MessageType::ok_height));

  input.now += 0.01;
  input.events = {car_event(1.1, 0.0, input.now)};
  decision = navigation.update(input);
  EXPECT_NEAR(navigation.planner().target_x_m(), 1.25, 1e-9);
  EXPECT_NEAR(navigation.planner().target_y_m(), -0.10, 1e-9);
  input.now += 0.01;
  input.events = {car_event(0.10, 0.0, input.now), event(MessageType::match_car_ok, input.now)};
  decision = navigation.update(input);
  EXPECT_EQ(navigation.phase(), "throwing");
  EXPECT_TRUE(decision.release_gripper);
  input.events.clear(); input.gripper_succeeded = true; input.now += 1.1;
  decision = navigation.update(input);
  EXPECT_EQ(navigation.phase(), "awaiting_b_ok");
  EXPECT_TRUE(has_message(decision, MessageType::ok_throw));
  input.gripper_succeeded = false; input.events = {event(MessageType::b_ok, input.now)};
  decision = navigation.update(input);
  EXPECT_EQ(navigation.phase(), "returning");
  EXPECT_TRUE(has_message(decision, MessageType::ok_return));
  ASSERT_TRUE(decision.control.origin);
  ASSERT_TRUE(decision.control.mission_goal);
  EXPECT_NEAR(decision.control.mission_goal->x_m, decision.control.origin->x_m, 1e-9);
  EXPECT_NEAR(decision.control.mission_goal->y_m, decision.control.origin->y_m, 1e-9);
  EXPECT_NEAR(decision.control.mission_goal->z_m, decision.control.origin->z_m + mission.takeoff_height_m, 1e-9);

  for (int i = 0; i < 800 && navigation.phase() == "returning"; ++i) {
    input.now += 0.05;
    follow_planner(navigation, input);
    decision = navigation.update(input);
  }
  ASSERT_EQ(navigation.phase(), "downing");
  EXPECT_TRUE(has_message(decision, MessageType::ok_downing));
  for (int i = 0; i < 800 && navigation.phase() == "downing"; ++i) {
    input.now += 0.05;
    follow_planner(navigation, input);
    decision = navigation.update(input);
  }
  ASSERT_EQ(navigation.phase(), "disarming");
  input.controller.armed = false; input.now += 0.05;
  navigation.update(input);
  ASSERT_EQ(navigation.phase(), "manual_request_pending");
  input.controller.mode = "MANUAL"; input.now += 0.05;
  decision = navigation.update(input);
  EXPECT_EQ(navigation.phase(), "manual");
  EXPECT_TRUE(has_message(decision, MessageType::ok_down));
}

TEST(NavigationV2Test, PreflightLossBeforeArmForcesManualAndDisarm)
{
  SafetyConfig safety;
  safety.setpoint_warmup_s = 0.01;
  Navigation navigation(safety);
  auto input = base_input(0.0);
  navigation.update(input);
  ASSERT_EQ(navigation.phase(), "setpoint_warmup");
  input.now = 0.01;
  navigation.update(input);
  ASSERT_EQ(navigation.phase(), "waiting_run_plan1");

  input.now = 0.02;
  input.events = {event(MessageType::run_plan1, input.now)};
  navigation.update(input);
  input.events.clear();
  ASSERT_EQ(navigation.phase(), "offboard_request_pending");

  input.controller.mode = "OFFBOARD";
  input.now = 0.04;
  navigation.update(input);
  ASSERT_EQ(navigation.phase(), "arming_request_pending");

  // 模拟尚未解锁时预检丢失；仍应执行既有的地面安全回退。
  input.controller.mode = "OFFBOARD";
  input.controller.armed = false;
  input.preflight_ready = false;
  input.now = 0.05;
  const auto decision = navigation.update(input);
  EXPECT_EQ(navigation.phase(), "manual_request_pending");
  ASSERT_TRUE(decision.target_mode);
  EXPECT_EQ(*decision.target_mode, "MANUAL");
  ASSERT_TRUE(decision.arm_intent);
  EXPECT_FALSE(*decision.arm_intent);
  EXPECT_FALSE(decision.setpoint);
  EXPECT_FALSE(decision.control.origin);
  EXPECT_TRUE(std::find(decision.rejections.begin(), decision.rejections.end(),
    "preflight_lost_before_arm") != decision.rejections.end());

  input.controller.mode = "MANUAL";
  input.controller.armed = false;
  input.now = 0.10;
  navigation.update(input);
  EXPECT_EQ(navigation.phase(), "manual");
}

TEST(NavigationV2Test, ArmedConfirmationIgnoresGroundPreflightAndStartsClimb)
{
  SafetyConfig safety;
  safety.setpoint_warmup_s = 0.01;
  MissionConfig mission;
  mission.takeoff_height_m = 1.5;
  Navigation navigation(safety, mission);
  auto input = base_input(0.0);
  navigation.update(input);
  input.now = 0.01;
  navigation.update(input);
  input.now = 0.02;
  input.events = {event(MessageType::run_plan1, input.now)};
  navigation.update(input);
  input.events.clear();
  input.controller.mode = "OFFBOARD";
  input.now = 0.03;
  navigation.update(input);
  ASSERT_EQ(navigation.phase(), "arming_request_pending");

  input.controller.armed = true;
  input.preflight_ready = false;
  input.now = 0.04;
  const auto decision = navigation.update(input);
  EXPECT_EQ(navigation.phase(), "climb");
  ASSERT_TRUE(decision.target_mode);
  EXPECT_EQ(*decision.target_mode, "OFFBOARD");
  ASSERT_TRUE(decision.arm_intent);
  EXPECT_TRUE(*decision.arm_intent);
  ASSERT_TRUE(decision.control.origin);
  ASSERT_TRUE(decision.control.mission_goal);
  EXPECT_NEAR(decision.control.origin->x_m, 0.0, 1e-9);
  EXPECT_NEAR(decision.control.origin->y_m, 0.0, 1e-9);
  EXPECT_NEAR(decision.control.origin->z_m, 0.0, 1e-9);
  EXPECT_NEAR(decision.control.mission_goal->z_m, mission.takeoff_height_m, 1e-9);
  EXPECT_FALSE(std::find(decision.rejections.begin(), decision.rejections.end(),
    "preflight_lost_before_arm") != decision.rejections.end());
}

TEST(NavigationV2Test, FlightHealthFailureAfterArmRequestsAutoLand)
{
  SafetyConfig safety;
  safety.setpoint_warmup_s = 0.01;
  Navigation navigation(safety);
  auto input = base_input(0.0);
  navigation.update(input);
  input.now = 0.01;
  navigation.update(input);
  input.now = 0.02;
  input.events = {event(MessageType::run_plan1, input.now)};
  navigation.update(input);
  input.events.clear();
  input.controller.mode = "OFFBOARD";
  input.now = 0.03;
  navigation.update(input);
  ASSERT_EQ(navigation.phase(), "arming_request_pending");

  input.controller.armed = true;
  input.flight_healthy = false;
  input.health_errors = {"range data stale"};
  input.now = 0.04;
  const auto decision = navigation.update(input);
  EXPECT_EQ(navigation.phase(), "landing");
  ASSERT_TRUE(decision.target_mode);
  EXPECT_EQ(*decision.target_mode, "AUTO.LAND");
  ASSERT_TRUE(decision.arm_intent);
  EXPECT_TRUE(*decision.arm_intent);
  EXPECT_FALSE(decision.control.origin);
  EXPECT_FALSE(std::find(decision.rejections.begin(), decision.rejections.end(),
    "preflight_lost_before_arm") != decision.rejections.end());
}

TEST(NavigationV2Test, OffboardLossAfterArmRequestsAutoLand)
{
  SafetyConfig safety;
  safety.setpoint_warmup_s = 0.01;
  Navigation navigation(safety);
  auto input = base_input(0.0);
  navigation.update(input);
  input.now = 0.01;
  navigation.update(input);
  input.now = 0.02;
  input.events = {event(MessageType::run_plan1, input.now)};
  navigation.update(input);
  input.events.clear();
  input.controller.mode = "OFFBOARD";
  input.now = 0.03;
  navigation.update(input);
  ASSERT_EQ(navigation.phase(), "arming_request_pending");

  input.controller.armed = true;
  input.controller.mode = "POSCTL";
  input.now = 0.04;
  const auto decision = navigation.update(input);
  EXPECT_EQ(navigation.phase(), "landing");
  ASSERT_TRUE(decision.target_mode);
  EXPECT_EQ(*decision.target_mode, "AUTO.LAND");
  ASSERT_TRUE(decision.arm_intent);
  EXPECT_TRUE(*decision.arm_intent);
  EXPECT_FALSE(decision.control.origin);
}

TEST(NavigationV2Test, LcpLossDuringClimbDoesNotInterruptClimb)
{
  SafetyConfig safety;
  safety.setpoint_warmup_s = 0.01;
  safety.max_z_setpoint_rate_m_s = 0.20;
  safety.max_z_setpoint_accel_m_s2 = 0.40;
  MissionConfig mission;
  mission.takeoff_height_m = 1.5;
  Navigation navigation(safety, mission);
  auto input = base_input(0.0);

  navigation.update(input);
  input.now = 0.01;
  navigation.update(input);
  input.now = 0.02;
  input.events = {event(MessageType::run_plan1, input.now)};
  navigation.update(input);
  input.events.clear();
  input.controller.mode = "OFFBOARD";
  input.now = 0.03;
  navigation.update(input);
  input.controller.armed = true;
  input.now = 0.04;
  navigation.update(input);
  ASSERT_EQ(navigation.phase(), "climb");
  ASSERT_NEAR(navigation.planner().target_z_m(), mission.takeoff_height_m, 1e-9);
  ASSERT_TRUE(navigation.control_state().mission_goal);
  const auto climb_goal = *navigation.control_state().mission_goal;
  ASSERT_TRUE(navigation.control_state().commanded_setpoint);
  const double command_before_lcp_loss = navigation.control_state().commanded_setpoint->z_m;

  input.now = 0.09;
  follow_planner(navigation, input);
  input.lcp_healthy = false;
  const auto decision = navigation.update(input);
  EXPECT_EQ(navigation.phase(), "climb");
  EXPECT_NEAR(navigation.planner().target_z_m(), mission.takeoff_height_m, 1e-9);
  EXPECT_FALSE(navigation.planner().target_reached());
  ASSERT_TRUE(decision.control.mission_goal);
  ASSERT_TRUE(decision.control.commanded_setpoint);
  EXPECT_FALSE(decision.control.hold_setpoint);
  EXPECT_FALSE(decision.control.mission_paused);
  EXPECT_NEAR(decision.control.mission_goal->z_m, climb_goal.z_m, 1e-9);
  EXPECT_GT(decision.control.commanded_setpoint->z_m, command_before_lcp_loss);
}

TEST(NavigationV2Test, LcpLossAfterClimbHoldsMeasuredPositionUntilRecovery)
{
  SafetyConfig safety;
  safety.setpoint_warmup_s = 0.01;
  safety.max_z_setpoint_rate_m_s = 1.0;
  safety.max_z_setpoint_accel_m_s2 = 1.5;
  MissionConfig mission;
  mission.takeoff_height_m = 0.20;
  mission.height_stable_seconds = 0.01;
  Navigation navigation(safety, mission);
  auto input = base_input(0.0);

  navigation.update(input);
  input.now = 0.01;
  navigation.update(input);
  input.now = 0.02;
  input.events = {event(MessageType::run_plan1, input.now)};
  navigation.update(input);
  input.events.clear();
  input.controller.mode = "OFFBOARD";
  input.now = 0.03;
  navigation.update(input);
  input.controller.armed = true;
  input.now = 0.04;
  navigation.update(input);
  for (int i = 0; i < 100 && navigation.phase() != "tracking"; ++i) {
    input.now += 0.05;
    follow_planner(navigation, input);
    navigation.update(input);
  }
  ASSERT_EQ(navigation.phase(), "tracking");

  input.now += 0.01;
  input.events = {car_event(1.10, 0.0, input.now)};
  navigation.update(input);
  input.events.clear();
  ASSERT_NEAR(navigation.planner().target_x_m(), 1.0, 1e-9);
  ASSERT_TRUE(navigation.control_state().mission_goal);
  const auto mission_before_hold = *navigation.control_state().mission_goal;

  input.now += 0.05;
  input.telemetry.local_x_m = 0.25;
  input.telemetry.local_y_m = -0.10;
  input.telemetry.local_z_m = 0.15;
  input.lcp_healthy = false;
  auto decision = navigation.update(input);
  ASSERT_EQ(navigation.phase(), "lcp_hold");
  ASSERT_TRUE(decision.setpoint);
  EXPECT_NEAR(decision.setpoint->x_m, 0.25, 1e-9);
  EXPECT_NEAR(decision.setpoint->y_m, -0.10, 1e-9);
  EXPECT_NEAR(decision.setpoint->z_m, 0.15, 1e-9);
  ASSERT_TRUE(decision.control.hold_setpoint);
  ASSERT_TRUE(decision.control.mission_goal);
  EXPECT_TRUE(decision.control.mission_paused);
  EXPECT_EQ(decision.control.hold_reason, "lcp_unhealthy");
  EXPECT_NEAR(decision.control.hold_setpoint->x_m, 0.25, 1e-9);
  EXPECT_NEAR(decision.control.hold_setpoint->y_m, -0.10, 1e-9);
  EXPECT_NEAR(decision.control.hold_setpoint->z_m, 0.15, 1e-9);
  EXPECT_NEAR(decision.control.mission_goal->x_m, mission_before_hold.x_m, 1e-9);
  EXPECT_NEAR(decision.control.mission_goal->y_m, mission_before_hold.y_m, 1e-9);
  EXPECT_NEAR(decision.control.mission_goal->z_m, mission_before_hold.z_m, 1e-9);

  input.now += 1.0;
  decision = navigation.update(input);
  EXPECT_EQ(navigation.phase(), "lcp_hold");
  ASSERT_TRUE(decision.setpoint);
  EXPECT_NEAR(decision.setpoint->x_m, 0.25, 1e-9);
  EXPECT_NEAR(decision.setpoint->y_m, -0.10, 1e-9);
  EXPECT_NEAR(decision.setpoint->z_m, 0.15, 1e-9);

  input.now += 0.05;
  input.lcp_healthy = true;
  navigation.update(input);
  EXPECT_EQ(navigation.phase(), "tracking");
  EXPECT_NEAR(navigation.planner().target_x_m(), 1.0, 1e-9);
  EXPECT_NEAR(navigation.planner().target_y_m(), 0.0, 1e-9);
  EXPECT_NEAR(navigation.planner().target_z_m(), mission.takeoff_height_m, 1e-9);
  EXPECT_FALSE(navigation.planner().target_reached());
  ASSERT_TRUE(navigation.control_state().mission_goal);
  EXPECT_NEAR(navigation.control_state().mission_goal->x_m, mission_before_hold.x_m, 1e-9);
  EXPECT_FALSE(navigation.control_state().mission_paused);
}

TEST(NavigationV2Test, HeightStabilizationRestartsAfterLcpRecovery)
{
  SafetyConfig safety;
  safety.setpoint_warmup_s = 0.01;
  safety.max_z_setpoint_rate_m_s = 1.0;
  safety.max_z_setpoint_accel_m_s2 = 1.5;
  MissionConfig mission;
  mission.takeoff_height_m = 0.20;
  mission.height_stable_seconds = 3.0;
  Navigation navigation(safety, mission);
  auto input = base_input(0.0);

  navigation.update(input);
  input.now = 0.01;
  navigation.update(input);
  input.now = 0.02;
  input.events = {event(MessageType::run_plan1, input.now)};
  navigation.update(input);
  input.events.clear();
  input.controller.mode = "OFFBOARD";
  input.now = 0.03;
  navigation.update(input);
  input.controller.armed = true;
  input.now = 0.04;
  navigation.update(input);
  for (int i = 0; i < 100 && navigation.phase() != "height_stabilizing"; ++i) {
    input.now += 0.05;
    follow_planner(navigation, input);
    navigation.update(input);
  }
  ASSERT_EQ(navigation.phase(), "height_stabilizing");

  input.now += 0.05;
  follow_planner(navigation, input);
  input.lcp_healthy = false;
  navigation.update(input);
  ASSERT_EQ(navigation.phase(), "lcp_hold");

  input.now += 0.05;
  follow_planner(navigation, input);
  input.lcp_healthy = true;
  navigation.update(input);
  ASSERT_EQ(navigation.phase(), "height_stabilizing");
  const double lcp_recovered_at = input.now;
  for (int i = 0; i < 59; ++i) {
    input.now += 0.05;
    follow_planner(navigation, input);
    const auto decision = navigation.update(input);
    EXPECT_EQ(navigation.phase(), "height_stabilizing");
    EXPECT_FALSE(has_message(decision, MessageType::ok_height));
  }
  for (int i = 0; i < 2 && navigation.phase() == "height_stabilizing"; ++i) {
    input.now += 0.05;
    follow_planner(navigation, input);
    navigation.update(input);
  }
  EXPECT_GE(input.now - lcp_recovered_at, mission.height_stable_seconds);
  EXPECT_EQ(navigation.phase(), "tracking");
}

TEST(NavigationV2Test, StaleCarHoldsPositionAndFailedGripperCanRetry)
{
  SafetyConfig safety;
  safety.setpoint_warmup_s = 0.01;
  Navigation navigation(safety);
  auto input = base_input(0.0);
  navigation.update(input);
  input.now = 0.01; navigation.update(input);
  input.events = {event(MessageType::run_plan1, 0.02)}; input.now = 0.02; navigation.update(input);
  input.events.clear(); input.controller.mode = "OFFBOARD"; input.now = 0.03; navigation.update(input);
  input.controller.armed = true; input.now = 0.04; navigation.update(input);
  for (int i = 0; i < 800 &&
    (navigation.phase() == "climb" || navigation.phase() == "height_stabilizing"); ++i) {
    input.now += 0.05; follow_planner(navigation, input); navigation.update(input);
  }
  ASSERT_EQ(navigation.phase(), "tracking");
  input.events = {car_event(0.1, 0.0, input.now)}; navigation.update(input);
  input.events = {event(MessageType::match_car_ok, input.now)}; navigation.update(input);
  ASSERT_EQ(navigation.phase(), "throwing");
  input.events.clear(); input.gripper_failed = true; input.now += 0.01;
  auto decision = navigation.update(input);
  EXPECT_EQ(navigation.phase(), "tracking");
  EXPECT_FALSE(has_message(decision, MessageType::ok_throw));
  input.gripper_failed = false; input.now += 0.6;
  input.telemetry.local_x_m = 0.3; input.telemetry.local_y_m = -0.2;
  navigation.update(input);
  EXPECT_NEAR(navigation.planner().target_x_m(), 0.3, 1e-9);
  EXPECT_NEAR(navigation.planner().target_y_m(), -0.2, 1e-9);
}

TEST(NavigationV3Test, CarTimeoutWaitsForNewStatusAndNeverRestoresOldGoal)
{
  SafetyConfig safety;
  safety.setpoint_warmup_s = 0.01;
  safety.max_z_setpoint_rate_m_s = 1.0;
  safety.max_z_setpoint_accel_m_s2 = 1.5;
  MissionConfig mission;
  mission.takeoff_height_m = 0.20;
  mission.height_stable_seconds = 0.01;
  mission.car_status_timeout_s = 0.50;
  Navigation navigation(safety, mission);
  auto input = base_input(0.0);

  navigation.update(input);
  input.now = 0.01; navigation.update(input);
  input.events = {event(MessageType::run_plan1, 0.02)}; input.now = 0.02; navigation.update(input);
  input.events.clear(); input.controller.mode = "OFFBOARD"; input.now = 0.03; navigation.update(input);
  input.controller.armed = true; input.now = 0.04; navigation.update(input);
  for (int i = 0; i < 100 && navigation.phase() != "tracking"; ++i) {
    input.now += 0.05; follow_planner(navigation, input); navigation.update(input);
  }
  ASSERT_EQ(navigation.phase(), "tracking");

  input.now += 0.01;
  input.events = {car_event(1.10, 0.0, input.now)};
  navigation.update(input);
  input.events.clear();
  ASSERT_TRUE(navigation.control_state().mission_goal);
  const auto old_goal = *navigation.control_state().mission_goal;
  EXPECT_NEAR(old_goal.x_m, 1.0, 1e-9);

  input.now += 0.60;
  input.telemetry.local_x_m = 0.25;
  input.telemetry.local_y_m = -0.10;
  input.telemetry.local_z_m = 0.20;
  auto decision = navigation.update(input);
  ASSERT_EQ(navigation.phase(), "waiting_car_status");
  ASSERT_TRUE(decision.control.hold_setpoint);
  ASSERT_TRUE(decision.control.mission_goal);
  EXPECT_EQ(decision.control.hold_reason, "car_status_timeout");
  EXPECT_TRUE(decision.control.mission_paused);
  EXPECT_NEAR(decision.control.hold_setpoint->x_m, 0.25, 1e-9);
  EXPECT_NEAR(decision.control.hold_setpoint->y_m, -0.10, 1e-9);
  EXPECT_NEAR(decision.control.mission_goal->x_m, old_goal.x_m, 1e-9);

  input.now += 0.05;
  input.lcp_healthy = false;
  navigation.update(input);
  ASSERT_EQ(navigation.phase(), "lcp_hold");
  input.now += 0.05;
  input.lcp_healthy = true;
  decision = navigation.update(input);
  ASSERT_EQ(navigation.phase(), "waiting_car_status");
  ASSERT_TRUE(decision.control.mission_goal);
  EXPECT_EQ(decision.control.hold_reason, "car_status_timeout");
  EXPECT_TRUE(decision.control.mission_paused);
  EXPECT_NEAR(decision.control.mission_goal->x_m, old_goal.x_m, 1e-9);
  EXPECT_NEAR(navigation.planner().target_x_m(), 0.25, 1e-9);
  EXPECT_NEAR(navigation.planner().target_y_m(), -0.10, 1e-9);

  input.now += 0.01;
  input.events = {car_event(0.60, 90.0, input.now)};
  decision = navigation.update(input);
  ASSERT_EQ(navigation.phase(), "tracking");
  ASSERT_TRUE(decision.control.mission_goal);
  EXPECT_FALSE(decision.control.hold_setpoint);
  EXPECT_FALSE(decision.control.mission_paused);
  EXPECT_NEAR(decision.control.mission_goal->x_m, 0.25, 1e-9);
  EXPECT_NEAR(decision.control.mission_goal->y_m, 0.40, 1e-9);
  EXPECT_NE(decision.control.mission_goal->x_m, old_goal.x_m);
}

TEST(NavigationV3Test, ControlJsonMatchesPublishedEnuPositionTarget)
{
  SafetyConfig safety;
  Navigation navigation(safety);
  auto input = base_input(0.0);
  input.telemetry.local_x_m = 1.25;
  input.telemetry.local_y_m = -2.50;
  input.telemetry.local_z_m = 0.75;
  const auto decision = navigation.update(input);
  ASSERT_TRUE(decision.setpoint);

  Json::CharReaderBuilder reader;
  Json::Value root;
  std::string errors;
  std::istringstream encoded(control_json(decision.control));
  ASSERT_TRUE(Json::parseFromStream(reader, encoded, &root, &errors));
  EXPECT_TRUE(root["origin"].isNull());
  EXPECT_TRUE(root["mission_goal"].isNull());
  EXPECT_TRUE(root["hold_setpoint"].isNull());
  EXPECT_EQ(root["hold_reason"].asString(), "");
  EXPECT_FALSE(root["mission_paused"].asBool());
  ASSERT_TRUE(root["commanded_setpoint"].isObject());

  builtin_interfaces::msg::Time stamp;
  const auto target = mavros_xyz_position_offboard::offboard::Offboard::make_position_target(
    *decision.setpoint, stamp);
  EXPECT_DOUBLE_EQ(root["commanded_setpoint"]["x_m"].asDouble(), target.position.x);
  EXPECT_DOUBLE_EQ(root["commanded_setpoint"]["y_m"].asDouble(), target.position.y);
  EXPECT_DOUBLE_EQ(root["commanded_setpoint"]["z_m"].asDouble(), target.position.z);
  EXPECT_DOUBLE_EQ(root["commanded_setpoint"]["yaw_rad"].asDouble(), target.yaw);
}

class PwmGripperTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    char pattern[] = "/tmp/mavros-pwm-test-XXXXXX";
    root_ = ::mkdtemp(pattern);
    ASSERT_FALSE(root_.empty());
    std::filesystem::create_directories(root_ + "/pwmchip0/pwm0");
    std::ofstream(root_ + "/pwmchip0/export") << "0";
    std::ofstream(root_ + "/pinmux") << "gpio18 pwm0";
    for (const std::string name : {"enable", "period", "duty_cycle"}) {
      std::ofstream(root_ + "/pwmchip0/pwm0/" + name) << "0";
    }
  }
  void TearDown() override {std::filesystem::remove_all(root_);}
  PwmGripperConfig config() const
  {
    PwmGripperConfig value;
    value.enabled = true;
    value.chip_path = root_ + "/pwmchip0";
    value.channel = 0;
    value.period_ns = 20000000;
    value.idle_duty_ns = 1500000;
    value.release_duty_ns = 2000000;
    value.release_delay_ms = 500;
    value.release_hold_ms = 200;
    value.pinmux_path = root_ + "/pinmux";
    value.pinmux_expected = "pwm0";
    return value;
  }
  std::string read(const std::string & path) const
  {
    std::ifstream stream(path);
    std::string value;
    stream >> value;
    return value;
  }
  std::string root_;
};

TEST_F(PwmGripperTest, WritesSysfsInReleaseOrderAndRestoresIdle)
{
  PwmGripper gripper(config());
  ASSERT_TRUE(gripper.begin_release(0.0));
  EXPECT_EQ(read(root_ + "/pwmchip0/pwm0/period"), "20000000");
  EXPECT_EQ(read(root_ + "/pwmchip0/pwm0/duty_cycle"), "1500000");
  EXPECT_EQ(gripper.update(0.49), ReleaseState::waiting_delay);
  EXPECT_EQ(gripper.update(0.50), ReleaseState::holding_release);
  EXPECT_EQ(read(root_ + "/pwmchip0/pwm0/duty_cycle"), "2000000");
  EXPECT_EQ(gripper.update(0.71), ReleaseState::succeeded);
  EXPECT_EQ(read(root_ + "/pwmchip0/pwm0/duty_cycle"), "1500000");
}

TEST_F(PwmGripperTest, PinmuxFailureDoesNotReportSuccessAndCanRetry)
{
  auto value = config();
  value.pinmux_expected = "missing";
  PwmGripper gripper(value);
  EXPECT_FALSE(gripper.begin_release(0.0));
  EXPECT_EQ(gripper.state(), ReleaseState::failed);
  ASSERT_TRUE(gripper.fault());
  std::ofstream(root_ + "/pinmux") << "missing";
  EXPECT_TRUE(gripper.begin_release(1.0));
  EXPECT_EQ(gripper.update(1.5), ReleaseState::holding_release);
  EXPECT_EQ(gripper.update(1.71), ReleaseState::succeeded);
}

TEST(UdpIntegrationTest, LoopbackDatagramsDeliverEventThenAck)
{
  const int gcs_socket = ::socket(AF_INET, SOCK_DGRAM, 0);
  ASSERT_GE(gcs_socket, 0);
  sockaddr_in gcs{};
  gcs.sin_family = AF_INET;
  gcs.sin_port = 0;
  ::inet_pton(AF_INET, "127.0.0.1", &gcs.sin_addr);
  ASSERT_EQ(::bind(gcs_socket, reinterpret_cast<const sockaddr *>(&gcs), sizeof(gcs)), 0);
  socklen_t gcs_length = sizeof(gcs);
  ASSERT_EQ(::getsockname(gcs_socket, reinterpret_cast<sockaddr *>(&gcs), &gcs_length), 0);

  const int probe = ::socket(AF_INET, SOCK_DGRAM, 0);
  ASSERT_GE(probe, 0);
  sockaddr_in uav{};
  uav.sin_family = AF_INET;
  uav.sin_port = 0;
  ::inet_pton(AF_INET, "127.0.0.1", &uav.sin_addr);
  ASSERT_EQ(::bind(probe, reinterpret_cast<const sockaddr *>(&uav), sizeof(uav)), 0);
  socklen_t uav_length = sizeof(uav);
  ASSERT_EQ(::getsockname(probe, reinterpret_cast<sockaddr *>(&uav), &uav_length), 0);
  ::close(probe);

  GroundStationConfig config;
  config.bind_ip = "127.0.0.1";
  config.bind_port = ntohs(uav.sin_port);
  config.remote_ip = "127.0.0.1";
  config.remote_port = ntohs(gcs.sin_port);
  config.whitelist_ip = "127.0.0.1";
  config.whitelist_port = ntohs(gcs.sin_port);
  GroundStationLink link(config);
  ASSERT_TRUE(link.bound());
  timeval timeout{};
  timeout.tv_sec = 1;
  ASSERT_EQ(::setsockopt(gcs_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)), 0);

  const std::string run_plan = R"({"header":"run_plan1","data":{}})";
  ASSERT_EQ(::sendto(gcs_socket, run_plan.data(), run_plan.size(), 0,
    reinterpret_cast<const sockaddr *>(&uav), sizeof(uav)), static_cast<ssize_t>(run_plan.size()));
  const auto inbound = link.poll(0.9);
  ASSERT_EQ(inbound.size(), 1U);
  EXPECT_TRUE(inbound.front().accepted);
  EXPECT_EQ(inbound.front().type, MessageType::run_plan1);

  ASSERT_TRUE(link.send({MessageType::ok_wait}, 1.0));
  char buffer[256]{};
  sockaddr_in source{};
  socklen_t source_length = sizeof(source);
  const auto bytes = ::recvfrom(gcs_socket, buffer, sizeof(buffer), 0,
    reinterpret_cast<sockaddr *>(&source), &source_length);
  ASSERT_GT(bytes, 0);
  EXPECT_EQ(std::string(buffer, static_cast<std::size_t>(bytes)), R"({"header":"ok_wait","data":{}})");
  EXPECT_FALSE(link.retry_events(1.49));
  ASSERT_TRUE(link.retry_events(1.50));
  const auto retried = ::recvfrom(gcs_socket, buffer, sizeof(buffer), 0,
    reinterpret_cast<sockaddr *>(&source), &source_length);
  ASSERT_GT(retried, 0);
  EXPECT_EQ(std::string(buffer, static_cast<std::size_t>(retried)), R"({"header":"ok_wait","data":{}})");
  const std::string ack = R"({"header":"ack","data":{}})";
  ASSERT_EQ(::sendto(gcs_socket, ack.data(), ack.size(), 0,
    reinterpret_cast<const sockaddr *>(&uav), sizeof(uav)), static_cast<ssize_t>(ack.size()));
  const auto events = link.poll(1.1);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_TRUE(events.front().accepted);
  EXPECT_EQ(link.pending_event_count(), 0U);
  ::close(gcs_socket);
}

TEST(InitializationTest, LcpInitializationStillRequiresFreshPostRequestSamples)
{
  auto node = std::make_shared<rclcpp::Node>(
    "mavros_xyz_initialization_test", rclcpp::NodeOptions().use_global_arguments(false));
  AppOptions options;
  options.range_topic = "/test/range";
  options.optical_flow_topic = "/test/flow";
  options.lcp_status_topic = "/test/lcp/status";
  options.lcp_odometry_topic = "/test/lcp/odometry";
  options.lcp_start_service = "/test/lcp/start";
  SafetyConfig config;
  Initialization initialization(*node, options, config);
  initialization.update_lcp_status(2, 1.0);
  initialization.update_lcp_odometry(1.0, 2.0, 0.0, 1.0);
  initialization.begin_lcp_initialization(2.0);
  initialization.update_lcp_init_state("accepted", 2.0, "started");
  EXPECT_FALSE(initialization.lcp_ready(2.1));
  for (const double stamp : {2.1, 2.2, 2.3}) {
    initialization.update_lcp_status(2, stamp);
    initialization.update_lcp_odometry(1.0, 2.0, 0.0, stamp);
  }
  EXPECT_TRUE(initialization.lcp_ready(2.3));
}

TEST(InitializationTest, LcpStartPrerequisitesPermitGroundCommissioningWithoutBattery)
{
  auto node = std::make_shared<rclcpp::Node>(
    "mavros_xyz_lcp_start_gate_test", rclcpp::NodeOptions().use_global_arguments(false));
  AppOptions options;
  options.range_topic = "/test/range";
  options.optical_flow_topic = "/test/flow";
  options.lcp_status_topic = "/test/lcp/status";
  options.lcp_odometry_topic = "/test/lcp/odometry";
  options.lcp_start_service = "/test/lcp/start";
  SafetyConfig config;
  Initialization initialization(*node, options, config);
  initialization.update_state(true, false, "MANUAL", 0, 10.0);
  initialization.update_landed(
    mavros_xyz_position_offboard::common::MAV_LANDED_STATE_ON_GROUND, 10.0);
  initialization.update_battery(true, 0.0, NAN, 10.0);
  EXPECT_TRUE(initialization.lcp_start_prerequisite_errors(10.1).empty());
  EXPECT_FALSE(initialization.preflight_errors(10.1).empty());

  initialization.update_state(true, true, "MANUAL", 0, 10.2);
  EXPECT_FALSE(initialization.lcp_start_prerequisite_errors(10.2).empty());
}

TEST(InitializationTest, ArmedTelemetryProducesFlightErrorsBeforeStateMachineEntersFlight)
{
  auto node = std::make_shared<rclcpp::Node>(
    "mavros_xyz_armed_health_snapshot_test", rclcpp::NodeOptions().use_global_arguments(false));
  AppOptions options;
  options.range_topic = "/test/range";
  options.optical_flow_topic = "/test/flow";
  options.lcp_status_topic = "/test/lcp/status";
  options.lcp_odometry_topic = "/test/lcp/odometry";
  options.lcp_start_service = "/test/lcp/start";
  SafetyConfig config;
  Initialization initialization(*node, options, config);
  initialization.update_state(true, true, "OFFBOARD", 0, 10.0);

  const auto snapshot = initialization.health_snapshot(10.1, false, NAN, NAN);
  EXPECT_TRUE(snapshot.telemetry.armed);
  EXPECT_FALSE(snapshot.flight_errors.empty());
}

TEST(SafetyConfigTest, DefaultBatteryTelemetryTimeoutIsFiveSeconds)
{
  SafetyConfig config;
  EXPECT_DOUBLE_EQ(config.battery_timeout_s, 5.0);
  EXPECT_FALSE(mavros_xyz_position_offboard::common::stale(10.0, 14.99, config.battery_timeout_s));
  EXPECT_TRUE(mavros_xyz_position_offboard::common::stale(10.0, 15.01, config.battery_timeout_s));
}

TEST(LcpVisionBridgeTest, ConvertsNwuToEnu)
{
  nav_msgs::msg::Odometry source;
  source.header.frame_id = "lcp_nwu";
  source.pose.pose.position.x = 2.0;
  source.pose.pose.position.y = 3.0;
  source.pose.pose.orientation.w = 1.0;
  const auto output = mavros_xyz_position_offboard::bridge::LcpVisionBridge::nwu_to_enu(
    source, 0.20, 0.20);
  EXPECT_EQ(output.header.frame_id, "lcp_enu");
  EXPECT_DOUBLE_EQ(output.pose.pose.position.x, -3.0);
  EXPECT_DOUBLE_EQ(output.pose.pose.position.y, 2.0);
}

TEST(OffboardMappingTest, PositionTargetPreservesRosEnuForMavrosConversion)
{
  mavros_xyz_position_offboard::common::PositionSetpoint setpoint;
  setpoint.x_m = 1.0; setpoint.y_m = 2.0; setpoint.z_m = 3.0;
  setpoint.orientation = {0.0, 0.0, 0.0, 1.0};
  builtin_interfaces::msg::Time stamp;
  const auto target = mavros_xyz_position_offboard::offboard::Offboard::make_position_target(setpoint, stamp);
  EXPECT_EQ(target.coordinate_frame, mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED);
  EXPECT_EQ(target.type_mask & mavros_msgs::msg::PositionTarget::IGNORE_PX, 0U);
  EXPECT_EQ(target.type_mask & mavros_msgs::msg::PositionTarget::IGNORE_YAW, 0U);
  EXPECT_DOUBLE_EQ(target.position.x, 1.0);
  EXPECT_DOUBLE_EQ(target.position.y, 2.0);
  EXPECT_DOUBLE_EQ(target.position.z, 3.0);
  EXPECT_NEAR(target.yaw, 0.0, 1e-6);
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
