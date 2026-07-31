#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <json/json.h>
#include <rclcpp/rclcpp.hpp>

#include "mavros_xyz_position_offboard/application/application_node.hpp"
#include "mavros_xyz_position_offboard/bridge/lcp_vision_bridge.hpp"
#include "mavros_xyz_position_offboard/common/cli.hpp"
#include "mavros_xyz_position_offboard/communication/ground_station_link.hpp"
#include "mavros_xyz_position_offboard/gripper/pwm_gripper.hpp"
#include "mavros_xyz_position_offboard/initialization/initialization.hpp"
#include "mavros_xyz_position_offboard/navigation/navigation.hpp"
#include "mavros_xyz_position_offboard/offboard/offboard.hpp"

namespace
{
using mavros_xyz_position_offboard::application::ApplicationNode;
using mavros_xyz_position_offboard::common::AppOptions;
using mavros_xyz_position_offboard::common::SafetyConfig;
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

struct DoubleSafetyOverride
{
  const char * name;
  double SafetyConfig::* field;
  double value;
};

struct IntSafetyOverride
{
  const char * name;
  int SafetyConfig::* field;
  int value;
};

struct BoolSafetyOverride
{
  const char * name;
  bool SafetyConfig::* field;
  bool value;
};

const std::array<DoubleSafetyOverride, 38> kDoubleSafetyOverrides{{
  {"state_timeout_s", &SafetyConfig::state_timeout_s, 1.1},
  {"sys_status_timeout_s", &SafetyConfig::sys_status_timeout_s, 1.2},
  {"battery_timeout_s", &SafetyConfig::battery_timeout_s, 5.1},
  {"landed_timeout_s", &SafetyConfig::landed_timeout_s, 1.3},
  {"local_pose_timeout_s", &SafetyConfig::local_pose_timeout_s, 0.4},
  {"local_velocity_timeout_s", &SafetyConfig::local_velocity_timeout_s, 0.45},
  {"estimator_timeout_s", &SafetyConfig::estimator_timeout_s, 1.4},
  {"range_timeout_s", &SafetyConfig::range_timeout_s, 0.31},
  {"optical_flow_timeout_s", &SafetyConfig::optical_flow_timeout_s, 0.32},
  {"sensor_loss_grace_s", &SafetyConfig::sensor_loss_grace_s, 1.6},
  {"lcp_status_timeout_s", &SafetyConfig::lcp_status_timeout_s, 0.7},
  {"lcp_odometry_timeout_s", &SafetyConfig::lcp_odometry_timeout_s, 0.8},
  {"range_boundary_tolerance_m", &SafetyConfig::range_boundary_tolerance_m, 0.002},
  {"configured_min_range_m", &SafetyConfig::configured_min_range_m, 0.03},
  {"configured_max_range_m", &SafetyConfig::configured_max_range_m, 11.0},
  {"max_range_jump_m", &SafetyConfig::max_range_jump_m, 0.4},
  {"jump_window_s", &SafetyConfig::jump_window_s, 0.4},
  {"jump_settle_tolerance_m", &SafetyConfig::jump_settle_tolerance_m, 0.07},
  {"min_battery_voltage_v", &SafetyConfig::min_battery_voltage_v, 15.0},
  {"min_battery_fraction", &SafetyConfig::min_battery_fraction, 0.4},
  {"max_preflight_horizontal_speed_m_s", &SafetyConfig::max_preflight_horizontal_speed_m_s, 0.3},
  {"max_preflight_vertical_speed_m_s", &SafetyConfig::max_preflight_vertical_speed_m_s, 0.31},
  {"max_flight_horizontal_speed_m_s", &SafetyConfig::max_flight_horizontal_speed_m_s, 0.6},
  {"max_flight_vertical_speed_m_s", &SafetyConfig::max_flight_vertical_speed_m_s, 0.9},
  {"max_flight_horizontal_drift_m", &SafetyConfig::max_flight_horizontal_drift_m, 1.1},
  {"climb_horizontal_speed_limit_m_s", &SafetyConfig::climb_horizontal_speed_limit_m_s, 3.1},
  {"climb_horizontal_drift_limit_m", &SafetyConfig::climb_horizontal_drift_limit_m, 3.2},
  {"hover_min_height_m", &SafetyConfig::hover_min_height_m, 0.25},
  {"publish_rate_hz", &SafetyConfig::publish_rate_hz, 25.0},
  {"setpoint_warmup_s", &SafetyConfig::setpoint_warmup_s, 2.5},
  {"max_z_setpoint_rate_m_s", &SafetyConfig::max_z_setpoint_rate_m_s, 0.3},
  {"max_z_setpoint_accel_m_s2", &SafetyConfig::max_z_setpoint_accel_m_s2, 0.5},
  {"target_xy_max_speed_m_s", &SafetyConfig::target_xy_max_speed_m_s, 0.3},
  {"target_xy_max_accel_m_s2", &SafetyConfig::target_xy_max_accel_m_s2, 0.6},
  {"target_tolerance_m", &SafetyConfig::target_tolerance_m, 0.05},
  {"touchdown_z_tolerance_m", &SafetyConfig::touchdown_z_tolerance_m, 0.09},
  {"max_flight_seconds", &SafetyConfig::max_flight_seconds, 61.0},
  {"flow_effective_min_height_m", &SafetyConfig::flow_effective_min_height_m, 0.4},
}};

const std::array<IntSafetyOverride, 4> kIntSafetyOverrides{{
  {"lcp_ready_samples", &SafetyConfig::lcp_ready_samples, 5},
  {"jump_recovery_samples", &SafetyConfig::jump_recovery_samples, 4},
  {"min_optical_flow_quality", &SafetyConfig::min_optical_flow_quality, 42},
  {"flow_effective_min_quality", &SafetyConfig::flow_effective_min_quality, 44},
}};

const std::array<BoolSafetyOverride, 1> kBoolSafetyOverrides{{
  {"ignore_declared_min_range", &SafetyConfig::ignore_declared_min_range, false},
}};

AppOptions application_test_options()
{
  AppOptions options;
  options.range_topic = "/test/range";
  options.optical_flow_topic = "/test/flow";
  options.lcp_vision_bridge_enabled = false;
  return options;
}

rclcpp::NodeOptions application_test_node_options(std::vector<rclcpp::Parameter> overrides = {})
{
  overrides.emplace_back("udp.enabled", false);
  rclcpp::NodeOptions options;
  options.use_global_arguments(false);
  options.parameter_overrides(overrides);
  return options;
}

GroundStationLink disabled_link()
{
  GroundStationConfig config;
  config.enabled = false;
  config.whitelist_ip = "127.0.0.1";
  config.whitelist_port = 5010;
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

ProtocolEvent car_status_event(double distance_m, double bearing_rad, double now)
{
  auto value = event(MessageType::car_status, now);
  value.car_status = {distance_m, bearing_rad};
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
      R"({"header":"go_ahead_ok","data":{}})",
      R"({"header":"match_car_ok","data":{}})",
      R"({"header":"b_ok","data":{}})",
      R"({"header":"ack","data":{}})"}) {
    EXPECT_TRUE(link.decode_datagram(json, "127.0.0.1", 5010, 1.0).accepted) << json;
  }
  const auto car = link.decode_datagram(
    R"({"header":"car_status","data":{"distance_m":5.0,"bearing_rad":-3.141592653589793}})",
    "127.0.0.1", 5010, 1.0);
  ASSERT_TRUE(car.accepted);
  ASSERT_EQ(car.type, MessageType::car_status);
  ASSERT_TRUE(car.car_status);
  EXPECT_DOUBLE_EQ(car.car_status->distance_m, 5.0);
  EXPECT_NEAR(car.car_status->bearing_rad, -std::acos(-1.0), 1e-12);
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
  reject(R"({"header":"go_ahead_ok","data":{"unexpected":1}})", "nonempty_event_data");
  reject(R"({"header":"car_status","data":{"distance":0.1,"angle":0}})",
    "invalid_car_status_data");
  reject(R"({"header":"car_status","data":{"distance_m":0.1,"bearing_rad":0,"distance_unit":"m"}})",
    "invalid_car_status_data");
  reject(R"({"header":"car_status","data":{"distance_m":-0.001,"bearing_rad":0}})",
    "car_status_distance_out_of_range");
  reject(R"({"header":"car_status","data":{"distance_m":5.001,"bearing_rad":0}})",
    "car_status_distance_out_of_range");
  reject(R"({"header":"car_status","data":{"distance_m":0.1,"bearing_rad":3.15}})",
    "car_status_bearing_out_of_range");
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

TEST(TrajectoryPlannerTest, TimedXyTargetMeetsFeasibleDeadlineAndExtendsUnsafeDeadline)
{
  SafetyConfig feasible;
  feasible.target_xy_max_speed_m_s = 10.0;
  feasible.target_xy_max_accel_m_s2 = 100.0;
  TrajectoryPlanner planner(feasible);
  planner.latch(0.0, 0.0, 1.5, {0.0, 0.0, 0.0, 1.0});
  EXPECT_TRUE(planner.set_xy_target_with_arrival_time(1.0, 0.0, 1.0));
  EXPECT_TRUE(planner.xy_arrival_time_met());
  EXPECT_NEAR(planner.xy_trajectory_duration_s(), 1.0, 1e-9);

  SafetyConfig constrained;
  constrained.target_xy_max_speed_m_s = 0.25;
  constrained.target_xy_max_accel_m_s2 = 0.50;
  TrajectoryPlanner limited(constrained);
  limited.latch(0.0, 0.0, 1.5, {0.0, 0.0, 0.0, 1.0});
  EXPECT_FALSE(limited.set_xy_target_with_arrival_time(1.0, 0.0, 1.0));
  EXPECT_FALSE(limited.xy_arrival_time_met());
  EXPECT_GT(limited.xy_trajectory_duration_s(), 1.0);
  auto previous = limited.current();
  for (int i = 0; i < 1000 && !limited.xy_target_reached(); ++i) {
    const auto current = limited.update(0.01);
    EXPECT_LE(std::hypot(current.x_m - previous.x_m, current.y_m - previous.y_m) / 0.01,
      constrained.target_xy_max_speed_m_s * 1.002);
    previous = current;
  }
}

TEST(TrajectoryPlannerTest, ForwardPursuitCanUseASpecificSpeedLimit)
{
  SafetyConfig config;
  config.target_xy_max_speed_m_s = 0.25;
  config.target_xy_max_accel_m_s2 = 0.50;

  TrajectoryPlanner default_limited(config);
  default_limited.latch(0.0, 0.0, 1.5, {0.0, 0.0, 0.0, 1.0});
  default_limited.set_xy_target(5.0, 0.0);

  TrajectoryPlanner forward_limited(config);
  forward_limited.latch(0.0, 0.0, 1.5, {0.0, 0.0, 0.0, 1.0});
  forward_limited.set_xy_target_with_max_speed(5.0, 0.0, 0.50);

  EXPECT_NEAR(default_limited.xy_trajectory_duration_s(), 40.0, 1e-9);
  EXPECT_NEAR(forward_limited.xy_trajectory_duration_s(), 20.0, 1e-9);
  auto previous = forward_limited.current();
  for (int i = 0; i < 2500 && !forward_limited.xy_target_reached(); ++i) {
    const auto current = forward_limited.update(0.01);
    EXPECT_LE(std::hypot(current.x_m - previous.x_m, current.y_m - previous.y_m) / 0.01,
      0.50 * 1.002);
    previous = current;
  }
}

TEST(NavigationV2Test, ForwardPursuitSpeedCannotExceedFlightHealthLimit)
{
  SafetyConfig safety;
  safety.max_flight_horizontal_speed_m_s = 0.50;
  MissionConfig mission;
  mission.forward_max_speed_m_s = 0.51;
  EXPECT_THROW(Navigation(safety, mission), std::invalid_argument);
}

TEST(NavigationV2Test, CompleteMissionAlignsRightThenPursuesAfterGcsApproval)
{
  SafetyConfig safety;
  safety.setpoint_warmup_s = 0.01;
  safety.max_flight_seconds = 200.0;
  MissionConfig mission;
  mission.takeoff_height_m = 1.5;
  mission.right_shift_m = 0.375;
  mission.forward_distance_m = 0.80;
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
  ASSERT_EQ(navigation.phase(), "right_shift");
  EXPECT_TRUE(has_message(decision, MessageType::ok_height));
  EXPECT_NEAR(navigation.planner().target_x_m(), 0.25, 1e-9);
  EXPECT_NEAR(navigation.planner().target_y_m(), -0.475, 1e-9);

  for (int i = 0; i < 200 && navigation.phase() == "right_shift"; ++i) {
    input.now += 0.05;
    follow_planner(navigation, input);
    decision = navigation.update(input);
  }
  ASSERT_EQ(navigation.phase(), "waiting_go_ahead");
  const double right_shift = input.telemetry.local_y_m - (-0.10);
  EXPECT_NEAR(input.telemetry.local_x_m, 0.25, safety.target_tolerance_m);
  EXPECT_GE(-right_shift, 0.35);
  EXPECT_LE(-right_shift, 0.40);

  input.now += 0.01;
  input.events = {event(MessageType::go_ahead_ok, input.now)};
  decision = navigation.update(input);
  input.events.clear();
  ASSERT_EQ(navigation.phase(), "pursuing_car");
  EXPECT_NEAR(navigation.planner().target_x_m(), 1.05, 1e-9);
  EXPECT_NEAR(navigation.planner().target_y_m(), -0.475, 1e-9);
  EXPECT_NEAR(navigation.planner().xy_trajectory_duration_s(),
    2.0 * mission.forward_distance_m / mission.forward_max_speed_m_s, 1e-9);

  // The first visual update takes over the bounded search using actual ENU position and yaw.
  input.now += 0.05;
  input.telemetry.local_x_m = 0.50;
  input.telemetry.local_y_m = -0.475;
  input.telemetry.local_z_m = 1.90;
  input.events = {car_status_event(0.80, 0.15, input.now)};
  decision = navigation.update(input);
  input.events.clear();
  ASSERT_EQ(navigation.phase(), "tracking_to_match");
  EXPECT_NEAR(navigation.planner().target_x_m(), 0.50 + 0.80 * std::cos(0.15), 1e-9);
  EXPECT_NEAR(navigation.planner().target_y_m(), -0.475 + 0.80 * std::sin(0.15), 1e-9);
  EXPECT_NEAR(navigation.planner().target_z_m(), 1.90, 1e-9);
  EXPECT_NEAR(navigation.planner().yaw_rad(), 0.0, 1e-9);

  // A fresh 0.2 m status confirms matching and starts the measured-position release hold.
  input.now += 0.05;
  input.events = {car_status_event(0.20, -0.10, input.now), event(MessageType::match_car_ok, input.now)};
  decision = navigation.update(input);
  input.events.clear();
  ASSERT_EQ(navigation.phase(), "match_hold");
  EXPECT_FALSE(decision.release_gripper);
  EXPECT_FALSE(has_message(decision, MessageType::ok_throw));

  input.now += mission.match_hold_seconds - 0.01;
  decision = navigation.update(input);
  EXPECT_EQ(navigation.phase(), "match_hold");
  EXPECT_FALSE(decision.release_gripper);
  EXPECT_FALSE(has_message(decision, MessageType::ok_throw));

  input.now += 0.02;
  decision = navigation.update(input);
  ASSERT_EQ(navigation.phase(), "throwing");
  EXPECT_TRUE(decision.release_gripper);
  EXPECT_FALSE(has_message(decision, MessageType::ok_throw));

  input.gripper_succeeded = true;
  input.now += 0.05;
  decision = navigation.update(input);
  ASSERT_EQ(navigation.phase(), "accompanying_car");
  EXPECT_TRUE(has_message(decision, MessageType::ok_throw));

  input.gripper_succeeded = false;
  input.now += 0.05;
  input.telemetry.local_x_m = 0.60;
  input.events = {car_status_event(0.30, 0.20, input.now)};
  decision = navigation.update(input);
  input.events.clear();
  EXPECT_EQ(navigation.phase(), "accompanying_car");
  EXPECT_NEAR(navigation.planner().target_x_m(), 0.60 + 0.30 * std::cos(0.20), 1e-9);
  EXPECT_NEAR(navigation.planner().target_y_m(), -0.475 + 0.30 * std::sin(0.20), 1e-9);

  // A sudden EKF local-Z drop from target-intercepted range is offset by one 11 cm step.
  input.now += 0.05;
  input.telemetry.local_z_m = 1.79;
  decision = navigation.update(input);
  EXPECT_EQ(navigation.phase(), "accompanying_car");
  EXPECT_NEAR(decision.control.accompanying_z_offset_m, -0.11, 1e-9);
  EXPECT_NEAR(navigation.planner().target_z_m(), 1.79, 1e-9);

  // A reverse local-Z jump removes the temporary offset rather than accumulating an error.
  input.now += 0.05;
  input.telemetry.local_z_m = 1.90;
  decision = navigation.update(input);
  EXPECT_NEAR(decision.control.accompanying_z_offset_m, 0.0, 1e-9);
  EXPECT_NEAR(navigation.planner().target_z_m(), 1.90, 1e-9);

  input.events = {event(MessageType::b_ok, input.now)};
  decision = navigation.update(input);
  input.events.clear();
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

TEST(NavigationV2Test, FlightTimeoutUsesConfiguredThresholdAndExistingAutoLandPath)
{
  for (const double max_flight_seconds : {0.10, 0.20}) {
    SafetyConfig safety;
    safety.setpoint_warmup_s = 0.01;
    safety.max_flight_seconds = max_flight_seconds;
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
    input.now = 0.04;
    navigation.update(input);
    ASSERT_EQ(navigation.phase(), "climb");
    input.now += max_flight_seconds - 0.01;
    navigation.update(input);
    EXPECT_EQ(navigation.phase(), "climb");
    input.now += 0.02;
    const auto decision = navigation.update(input);
    EXPECT_EQ(navigation.phase(), "landing");
    ASSERT_TRUE(decision.target_mode);
    EXPECT_EQ(*decision.target_mode, "AUTO.LAND");
  }
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
  for (int i = 0; i < 200 && navigation.phase() != "waiting_go_ahead"; ++i) {
    input.now += 0.05;
    follow_planner(navigation, input);
    navigation.update(input);
  }
  ASSERT_EQ(navigation.phase(), "waiting_go_ahead");

  input.now += 0.01;
  input.events = {event(MessageType::go_ahead_ok, input.now)};
  navigation.update(input);
  input.events.clear();
  ASSERT_EQ(navigation.phase(), "pursuing_car");
  ASSERT_NEAR(navigation.planner().target_x_m(), mission.forward_distance_m, 1e-9);
  ASSERT_NEAR(navigation.planner().target_y_m(), -mission.right_shift_m, 1e-9);
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
  EXPECT_EQ(navigation.phase(), "pursuing_car");
  EXPECT_NEAR(navigation.planner().target_x_m(), mission.forward_distance_m, 1e-9);
  EXPECT_NEAR(navigation.planner().target_y_m(), -mission.right_shift_m, 1e-9);
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
  EXPECT_EQ(navigation.phase(), "right_shift");
}

TEST(NavigationV2Test, MatchRequiresFreshVisualMeasurementAndReleasesAfterHold)
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
  input.now = 0.01; navigation.update(input);
  input.events = {event(MessageType::run_plan1, 0.02)}; input.now = 0.02; navigation.update(input);
  input.events.clear(); input.controller.mode = "OFFBOARD"; input.now = 0.03; navigation.update(input);
  input.controller.armed = true; input.now = 0.04; navigation.update(input);
  for (int i = 0; i < 200 && navigation.phase() != "waiting_go_ahead"; ++i) {
    input.now += 0.05; follow_planner(navigation, input); navigation.update(input);
  }
  ASSERT_EQ(navigation.phase(), "waiting_go_ahead");

  input.now += 0.01;
  input.events = {event(MessageType::match_car_ok, input.now)};
  auto decision = navigation.update(input);
  EXPECT_EQ(navigation.phase(), "waiting_go_ahead");
  EXPECT_TRUE(std::find(decision.rejections.begin(), decision.rejections.end(),
    "match_car_ok_not_allowed_in_phase") != decision.rejections.end());

  input.now += 0.01;
  input.events = {event(MessageType::go_ahead_ok, input.now)};
  navigation.update(input);
  ASSERT_EQ(navigation.phase(), "pursuing_car");
  input.now += 0.01;
  input.events = {event(MessageType::match_car_ok, input.now)};
  decision = navigation.update(input);
  EXPECT_EQ(navigation.phase(), "pursuing_car");
  EXPECT_TRUE(std::find(decision.rejections.begin(), decision.rejections.end(),
    "match_car_ok_not_allowed_in_phase") != decision.rejections.end());

  input.now += 0.01;
  input.events = {car_status_event(0.21, 0.0, input.now)};
  navigation.update(input);
  ASSERT_EQ(navigation.phase(), "tracking_to_match");
  input.now += 0.01;
  input.events = {event(MessageType::match_car_ok, input.now)};
  decision = navigation.update(input);
  EXPECT_EQ(navigation.phase(), "tracking_to_match");
  EXPECT_TRUE(std::find(decision.rejections.begin(), decision.rejections.end(),
    "match_car_ok_distance_above_tracking_tolerance") != decision.rejections.end());

  input.now += 0.01;
  input.events = {car_status_event(0.20, 0.0, input.now), event(MessageType::match_car_ok, input.now)};
  decision = navigation.update(input);
  input.events.clear();
  EXPECT_EQ(navigation.phase(), "match_hold");
  EXPECT_FALSE(decision.release_gripper);
  EXPECT_FALSE(has_message(decision, MessageType::ok_throw));

  input.now += mission.match_hold_seconds + 0.01;
  decision = navigation.update(input);
  ASSERT_EQ(navigation.phase(), "throwing");
  EXPECT_TRUE(decision.release_gripper);

  input.gripper_succeeded = true;
  input.now += 0.01;
  decision = navigation.update(input);
  ASSERT_EQ(navigation.phase(), "accompanying_car");
  EXPECT_TRUE(has_message(decision, MessageType::ok_throw));

  input.gripper_succeeded = false;
  input.events = {event(MessageType::b_ok, input.now)};
  decision = navigation.update(input);
  EXPECT_EQ(navigation.phase(), "returning");
  EXPECT_TRUE(has_message(decision, MessageType::ok_return));
}

TEST(NavigationV2Test, VisualTimeoutHoldsMeasuredPositionAndFreshStatusRestoresTracking)
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
  input.now = 0.01; navigation.update(input);
  input.events = {event(MessageType::run_plan1, 0.02)}; input.now = 0.02; navigation.update(input);
  input.events.clear(); input.controller.mode = "OFFBOARD"; input.now = 0.03; navigation.update(input);
  input.controller.armed = true; input.now = 0.04; navigation.update(input);
  for (int i = 0; i < 200 && navigation.phase() != "waiting_go_ahead"; ++i) {
    input.now += 0.05; follow_planner(navigation, input); navigation.update(input);
  }
  ASSERT_EQ(navigation.phase(), "waiting_go_ahead");

  input.now += 0.01;
  input.events = {event(MessageType::go_ahead_ok, input.now)};
  navigation.update(input);
  input.now += 0.01;
  input.telemetry.local_x_m = 0.40;
  input.telemetry.local_y_m = -mission.right_shift_m;
  const double current_yaw = std::acos(-1.0) / 2.0;
  input.telemetry.orientation = {0.0, 0.0, std::sin(current_yaw / 2.0), std::cos(current_yaw / 2.0)};
  input.events = {car_status_event(0.50, 0.0, input.now)};
  navigation.update(input);
  input.events.clear();
  ASSERT_EQ(navigation.phase(), "tracking_to_match");
  EXPECT_NEAR(navigation.planner().target_x_m(), 0.40, 1e-9);
  EXPECT_NEAR(navigation.planner().target_y_m(), -mission.right_shift_m + 0.50, 1e-9);
  EXPECT_NEAR(navigation.planner().yaw_rad(), current_yaw, 1e-9);

  input.now += 0.01;
  input.telemetry.local_y_m = 0.0;
  input.events = {car_status_event(5.0, 0.0, input.now)};
  auto decision = navigation.update(input);
  input.events.clear();
  EXPECT_EQ(navigation.phase(), "tracking_to_match");
  EXPECT_TRUE(std::find(decision.rejections.begin(), decision.rejections.end(),
    "car_status_target_outside_tracking_radius") != decision.rejections.end());

  input.now += mission.car_status_timeout_s + 0.01;
  input.telemetry.local_x_m = 0.45;
  input.telemetry.local_y_m = -0.30;
  decision = navigation.update(input);
  ASSERT_EQ(navigation.phase(), "tracking_timeout_hold");
  ASSERT_TRUE(decision.control.hold_setpoint);
  EXPECT_NEAR(decision.control.hold_setpoint->x_m, 0.45, 1e-9);
  EXPECT_NEAR(decision.control.hold_setpoint->y_m, -0.30, 1e-9);

  input.now += 0.01;
  input.events = {car_status_event(0.40, 0.5, input.now)};
  decision = navigation.update(input);
  input.events.clear();
  EXPECT_EQ(navigation.phase(), "tracking_to_match");
  EXPECT_FALSE(decision.control.mission_paused);
  EXPECT_NEAR(navigation.planner().target_x_m(),
    0.45 + 0.40 * std::cos(current_yaw + 0.5), 1e-9);
  EXPECT_NEAR(navigation.planner().target_y_m(),
    -0.30 + 0.40 * std::sin(current_yaw + 0.5), 1e-9);
  EXPECT_NEAR(navigation.planner().yaw_rad(), current_yaw, 1e-9);
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

struct PwmCommand
{
  int handle;
  int bcm_gpio;
  double frequency_hz;
  double duty_cycle;
};

class FakePwmGpioBackend final : public mavros_xyz_position_offboard::gripper::PwmGpioBackend
{
public:
  std::optional<int> discovered_gpiochip{4};
  int opened_handle{42};
  int claim_status{0};
  std::vector<int> pwm_statuses{};
  std::vector<int> opened_gpiochips{};
  std::vector<PwmCommand> pwm_commands{};
  int close_count{0};
  int free_count{0};

  std::optional<int> find_rp1_gpiochip() override {return discovered_gpiochip;}
  int gpiochip_open(int gpiochip) override
  {
    opened_gpiochips.push_back(gpiochip);
    return opened_handle;
  }
  int gpiochip_close(int) override {++close_count; return 0;}
  int gpio_claim_output(int, int, int) override {return claim_status;}
  int gpio_free(int, int) override {++free_count; return 0;}
  int tx_pwm(int handle, int bcm_gpio, double frequency_hz, double duty_cycle) override
  {
    pwm_commands.push_back({handle, bcm_gpio, frequency_hz, duty_cycle});
    if (pwm_statuses.empty()) {return 0;}
    const int status = pwm_statuses.front();
    pwm_statuses.erase(pwm_statuses.begin());
    return status;
  }
  std::string error_text(int status) const override {return "fake error " + std::to_string(status);}
};

PwmGripperConfig sg90_config()
{
  PwmGripperConfig value;
  value.enabled = true;
  value.bcm_gpio = 18;
  value.pwm_frequency_hz = 50.0;
  value.closed_duty_cycle = 4.0;
  value.open_duty_cycle = 7.0;
  value.open_hold_ms = 500;
  return value;
}

TEST(PwmGripperTest, InitializesClosedThenOpensForConfiguredHoldAndCloses)
{
  const auto backend = std::make_shared<FakePwmGpioBackend>();
  PwmGripper gripper(sg90_config(), backend);

  ASSERT_TRUE(gripper.initialize());
  ASSERT_EQ(backend->opened_gpiochips, std::vector<int>({4}));
  ASSERT_EQ(backend->pwm_commands.size(), 1U);
  EXPECT_EQ(backend->pwm_commands.front().handle, 42);
  EXPECT_EQ(backend->pwm_commands.front().bcm_gpio, 18);
  EXPECT_DOUBLE_EQ(backend->pwm_commands.front().frequency_hz, 50.0);
  EXPECT_DOUBLE_EQ(backend->pwm_commands.front().duty_cycle, 4.0);

  ASSERT_TRUE(gripper.begin_release(10.0));
  ASSERT_EQ(backend->pwm_commands.size(), 2U);
  EXPECT_DOUBLE_EQ(backend->pwm_commands.back().duty_cycle, 7.0);
  EXPECT_EQ(gripper.update(10.49), ReleaseState::holding_release);
  EXPECT_EQ(gripper.update(10.50), ReleaseState::succeeded);
  ASSERT_EQ(backend->pwm_commands.size(), 3U);
  EXPECT_DOUBLE_EQ(backend->pwm_commands.back().duty_cycle, 4.0);
}

TEST(PwmGripperTest, DisabledModeNeverTouchesGpio)
{
  auto config = sg90_config();
  config.enabled = false;
  const auto backend = std::make_shared<FakePwmGpioBackend>();
  PwmGripper gripper(config, backend);

  ASSERT_TRUE(gripper.initialize());
  ASSERT_TRUE(gripper.begin_release(0.0));
  EXPECT_EQ(gripper.update(0.49), ReleaseState::holding_release);
  EXPECT_EQ(gripper.update(0.50), ReleaseState::succeeded);
  EXPECT_TRUE(backend->opened_gpiochips.empty());
  EXPECT_TRUE(backend->pwm_commands.empty());
}

TEST(PwmGripperTest, InvalidConfigAndDiscoveryFailureAreRejected)
{
  auto invalid = sg90_config();
  invalid.open_duty_cycle = 100.0;
  EXPECT_THROW(invalid.validate(), std::invalid_argument);

  const auto backend = std::make_shared<FakePwmGpioBackend>();
  backend->discovered_gpiochip.reset();
  PwmGripper gripper(sg90_config(), backend);
  EXPECT_FALSE(gripper.initialize());
  EXPECT_EQ(gripper.state(), ReleaseState::failed);
  ASSERT_TRUE(gripper.fault());
  EXPECT_NE(gripper.fault()->find("RP1"), std::string::npos);

  backend->discovered_gpiochip = 4;
  EXPECT_TRUE(gripper.begin_release(1.0));
  EXPECT_EQ(gripper.state(), ReleaseState::holding_release);
}

TEST(PwmGripperTest, GpioClaimFailureReleasesHandleAndCanRetry)
{
  const auto backend = std::make_shared<FakePwmGpioBackend>();
  backend->claim_status = -6;
  PwmGripper gripper(sg90_config(), backend);

  EXPECT_FALSE(gripper.initialize());
  EXPECT_EQ(gripper.state(), ReleaseState::failed);
  EXPECT_TRUE(backend->pwm_commands.empty());
  EXPECT_EQ(backend->free_count, 0);
  EXPECT_EQ(backend->close_count, 1);

  backend->claim_status = 0;
  EXPECT_TRUE(gripper.begin_release(1.0));
  EXPECT_EQ(gripper.state(), ReleaseState::holding_release);
}

TEST(PwmGripperTest, OpenPwmFailureReleasesHardwareAndCanRetry)
{
  const auto backend = std::make_shared<FakePwmGpioBackend>();
  PwmGripper gripper(sg90_config(), backend);
  ASSERT_TRUE(gripper.initialize());

  backend->pwm_statuses = {-7};
  EXPECT_FALSE(gripper.begin_release(0.0));
  EXPECT_EQ(gripper.state(), ReleaseState::failed);
  ASSERT_TRUE(gripper.fault());
  EXPECT_EQ(backend->free_count, 1);
  EXPECT_EQ(backend->close_count, 1);

  EXPECT_TRUE(gripper.begin_release(1.0));
  EXPECT_EQ(gripper.update(1.50), ReleaseState::succeeded);
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

TEST(ApplicationNodeSafetyParameterTest, StartupOverridesEveryActiveSafetyField)
{
  std::vector<rclcpp::Parameter> overrides;
  for (const auto & definition : kDoubleSafetyOverrides) {
    overrides.emplace_back(std::string("safety.") + definition.name, definition.value);
  }
  for (const auto & definition : kIntSafetyOverrides) {
    overrides.emplace_back(std::string("safety.") + definition.name, definition.value);
  }
  for (const auto & definition : kBoolSafetyOverrides) {
    overrides.emplace_back(std::string("safety.") + definition.name, definition.value);
  }

  const auto node = std::make_shared<ApplicationNode>(
    application_test_options(), SafetyConfig{}, application_test_node_options(std::move(overrides)));
  const auto & config = node->safety_config();
  for (const auto & definition : kDoubleSafetyOverrides) {
    EXPECT_DOUBLE_EQ(config.*(definition.field), definition.value) << definition.name;
  }
  for (const auto & definition : kIntSafetyOverrides) {
    EXPECT_EQ(config.*(definition.field), definition.value) << definition.name;
  }
  for (const auto & definition : kBoolSafetyOverrides) {
    EXPECT_EQ(config.*(definition.field), definition.value) << definition.name;
  }
}

TEST(ApplicationNodeSafetyParameterTest, CliDefaultsRemainWhenNoSafetyOverrideIsProvided)
{
  SafetyConfig cli_config;
  cli_config.max_flight_seconds = 72.0;
  cli_config.relative_z_m = 1.23;
  cli_config.hold_seconds = 9.87;
  const auto node = std::make_shared<ApplicationNode>(
    application_test_options(), cli_config, application_test_node_options());

  EXPECT_DOUBLE_EQ(node->safety_config().max_flight_seconds, cli_config.max_flight_seconds);
  EXPECT_DOUBLE_EQ(node->safety_config().relative_z_m, cli_config.relative_z_m);
  EXPECT_DOUBLE_EQ(node->safety_config().hold_seconds, cli_config.hold_seconds);
  EXPECT_FALSE(node->has_parameter("safety.relative_z_m"));
  EXPECT_FALSE(node->has_parameter("safety.hold_seconds"));
}

TEST(ApplicationNodeSafetyParameterTest, YamlParametersOverrideCliDefaultsAtStartup)
{
  SafetyConfig cli_config;
  cli_config.max_flight_seconds = 72.0;
  cli_config.target_xy_max_speed_m_s = 0.70;
  rclcpp::NodeOptions options;
  options.use_global_arguments(false);
  options.arguments({
    "--ros-args", "--params-file",
    std::string(MAVROS_XYZ_SOURCE_DIR) + "/config/udp_ground_station.yaml"});
  options.append_parameter_override("udp.enabled", false);
  options.append_parameter_override("gripper_pwm.enabled", false);
  const auto node = std::make_shared<ApplicationNode>(application_test_options(), cli_config, options);

  EXPECT_DOUBLE_EQ(node->safety_config().max_flight_seconds, 120.0);
  EXPECT_DOUBLE_EQ(node->safety_config().target_xy_max_speed_m_s, 0.25);
  EXPECT_DOUBLE_EQ(node->get_parameter("udp.max_tracking_distance_m").as_double(), 5.0);
  EXPECT_DOUBLE_EQ(node->get_parameter("mission.forward_max_speed_m_s").as_double(), 0.50);
  EXPECT_DOUBLE_EQ(node->get_parameter("mission.tracking_arrival_seconds").as_double(), 1.0);
  EXPECT_DOUBLE_EQ(node->get_parameter("mission.tracking_tolerance_m").as_double(), 0.2);
  EXPECT_DOUBLE_EQ(node->get_parameter("mission.match_hold_seconds").as_double(), 0.5);
  EXPECT_DOUBLE_EQ(node->get_parameter("mission.car_status_timeout_s").as_double(), 2.0);
  EXPECT_DOUBLE_EQ(node->get_parameter("mission.max_tracking_radius_m").as_double(), 5.0);
  EXPECT_DOUBLE_EQ(node->get_parameter("mission.accompanying_z_jump_threshold_m").as_double(), 0.10);
  EXPECT_DOUBLE_EQ(node->get_parameter("mission.accompanying_z_step_m").as_double(), 0.11);
  EXPECT_DOUBLE_EQ(node->get_parameter("mission.accompanying_z_jump_window_s").as_double(), 0.10);
  EXPECT_EQ(node->get_parameter("gripper_pwm.bcm_gpio").as_int(), 18);
  EXPECT_DOUBLE_EQ(node->get_parameter("gripper_pwm.pwm_frequency_hz").as_double(), 50.0);
  EXPECT_DOUBLE_EQ(node->get_parameter("gripper_pwm.closed_duty_cycle").as_double(), 4.0);
  EXPECT_DOUBLE_EQ(node->get_parameter("gripper_pwm.open_duty_cycle").as_double(), 7.0);
  EXPECT_EQ(node->get_parameter("gripper_pwm.open_hold_ms").as_int(), 500);
}

TEST(ApplicationNodeSafetyParameterTest, InvalidStartupOverrideIsRejected)
{
  const std::vector<rclcpp::Parameter> overrides{
    rclcpp::Parameter("safety.max_flight_seconds", 0.0)};
  EXPECT_THROW(
    std::make_shared<ApplicationNode>(
      application_test_options(), SafetyConfig{}, application_test_node_options(overrides)),
    std::invalid_argument);
}

TEST(ApplicationNodeSafetyParameterTest, RuntimeChangesAreRejectedAndEffectiveConfigStaysFixed)
{
  SafetyConfig cli_config;
  cli_config.max_flight_seconds = 72.0;
  const auto node = std::make_shared<ApplicationNode>(
    application_test_options(), cli_config, application_test_node_options());

  const auto results = node->set_parameters({rclcpp::Parameter("safety.max_flight_seconds", 30.0)});
  ASSERT_EQ(results.size(), 1U);
  EXPECT_FALSE(results.front().successful);
  EXPECT_DOUBLE_EQ(node->get_parameter("safety.max_flight_seconds").as_double(), 72.0);
  EXPECT_DOUBLE_EQ(node->safety_config().max_flight_seconds, 72.0);
}

TEST(ApplicationNodeSafetyParameterTest, SensorSourceConfirmationRemainsCliOnly)
{
  SafetyConfig cli_config;
  cli_config.range_source_confirmed = true;
  cli_config.optical_flow_source_confirmed = true;
  const std::vector<rclcpp::Parameter> overrides{
    rclcpp::Parameter("safety.range_source_confirmed", false),
    rclcpp::Parameter("safety.optical_flow_source_confirmed", false)};
  const auto node = std::make_shared<ApplicationNode>(
    application_test_options(), cli_config, application_test_node_options(overrides));

  EXPECT_TRUE(node->safety_config().range_source_confirmed);
  EXPECT_TRUE(node->safety_config().optical_flow_source_confirmed);
  EXPECT_FALSE(node->has_parameter("safety.range_source_confirmed"));
  EXPECT_FALSE(node->has_parameter("safety.optical_flow_source_confirmed"));
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
