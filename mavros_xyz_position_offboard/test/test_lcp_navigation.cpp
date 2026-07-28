#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <memory>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "mavros_xyz_position_offboard/bridge/lcp_vision_bridge.hpp"
#include "mavros_xyz_position_offboard/common/cli.hpp"
#include "mavros_xyz_position_offboard/communication/ground_station_link.hpp"
#include "mavros_xyz_position_offboard/initialization/initialization.hpp"
#include "mavros_xyz_position_offboard/navigation/navigation.hpp"
#include "mavros_xyz_position_offboard/offboard/offboard.hpp"

namespace
{
using mavros_xyz_position_offboard::common::AppOptions;
using mavros_xyz_position_offboard::common::SafetyConfig;
using mavros_xyz_position_offboard::communication::GroundStationConfig;
using mavros_xyz_position_offboard::communication::GroundStationLink;
using mavros_xyz_position_offboard::communication::MessageType;
using mavros_xyz_position_offboard::communication::Point3;
using mavros_xyz_position_offboard::communication::ProtocolEvent;
using mavros_xyz_position_offboard::initialization::Initialization;
using mavros_xyz_position_offboard::navigation::Navigation;
using mavros_xyz_position_offboard::navigation::NavigationInput;
using mavros_xyz_position_offboard::navigation::TrajectoryPlanner;

class InitializationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>(
      "mavros_xyz_initialization_test", rclcpp::NodeOptions().use_global_arguments(false));
    options_.range_topic = "/test/range";
    options_.optical_flow_topic = "/test/flow";
    options_.lcp_status_topic = "/test/lcp/status";
    options_.lcp_odometry_topic = "/test/lcp/odometry";
    options_.lcp_start_service = "/test/lcp/start";
  }
  std::shared_ptr<rclcpp::Node> node_;
  AppOptions options_;
};

TEST_F(InitializationTest, OldStatusCannotSatisfyNewInitialization)
{
  SafetyConfig config;
  Initialization initialization(*node_, options_, config);
  initialization.update_lcp_status(2, 1.0);
  initialization.update_lcp_odometry(1.0, 2.0, 0.0, 1.0);
  initialization.begin_lcp_initialization(2.0);
  initialization.update_lcp_init_state("accepted", 2.0, "started");
  EXPECT_FALSE(initialization.lcp_ready(2.1));
}

TEST_F(InitializationTest, ThreeFreshStatusSamplesAndOdometryAreReady)
{
  SafetyConfig config;
  Initialization initialization(*node_, options_, config);
  initialization.begin_lcp_initialization(1.0);
  initialization.update_lcp_init_state("accepted", 1.0, "started");
  for (const double stamp : {1.1, 1.2, 1.3}) {
    initialization.update_lcp_status(2, stamp);
    initialization.update_lcp_odometry(1.0, 2.0, 0.0, stamp);
  }
  EXPECT_TRUE(initialization.lcp_ready(1.3));
  const auto snapshot = initialization.health_snapshot(1.3, false, 0.0, 0.0);
  EXPECT_TRUE(snapshot.lcp_ready);
  EXPECT_DOUBLE_EQ(snapshot.telemetry.lcp_x_m, 1.0);
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
  EXPECT_NEAR(output.pose.pose.orientation.z, std::sqrt(0.5), 1e-12);
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
  const auto before = planner.current();
  planner.set_target(0.2, 2.0, 0.4);
  EXPECT_DOUBLE_EQ(planner.current().x_m, before.x_m);
  EXPECT_DOUBLE_EQ(planner.current().z_m, before.z_m);
}

/// 创建不打开真实 socket、仅用于编解码单元测试的通信层实例。
GroundStationLink disabled_link()
{
  GroundStationConfig config;
  config.enabled = false;
  return GroundStationLink(GroundStationConfig{config});
}

TEST(GroundStationLinkTest, RequiresV1WhitelistAndDeduplicatesWithoutOrdering)
{
  auto link = disabled_link();
  auto event = link.decode_datagram(
    R"({"version":1,"type":"start","seq":8,"height_start":0.8})",
    "192.168.10.60", 5005, 1.0);
  EXPECT_FALSE(event.accepted);
  EXPECT_EQ(event.rejection_reason, "source_not_whitelisted");
  event = link.decode_datagram(
    R"({"version":1,"type":"start","seq":8,"height_start":0.8})",
    "192.168.10.59", 5005, 1.1);
  ASSERT_TRUE(event.accepted);
  event = link.decode_datagram(
    R"({"version":1,"type":"ACK","seq":3})", "192.168.10.59", 5005, 1.2);
  EXPECT_TRUE(event.accepted);  // Out of order is allowed.
  event = link.decode_datagram(
    R"({"version":1,"type":"ACK","seq":3})", "192.168.10.59", 5005, 1.3);
  EXPECT_FALSE(event.accepted);
  EXPECT_EQ(event.rejection_reason, "duplicate_seq");
}

TEST(GroundStationLinkTest, DecodesAllMissionMessagesAndStrictPointCounts)
{
  auto link = disabled_link();
  EXPECT_TRUE(link.decode_datagram(
    R"({"version":1,"type":"navigation_and_point","seq":1,"plan_mode":1,"point":{"x":1,"y":2,"z":0.8}})",
    "192.168.10.59", 5005, 1.0).accepted);
  EXPECT_TRUE(link.decode_datagram(
    R"({"version":1,"type":"navigation_nfz","seq":2,"nfz_point_count":1,"nfz_points":[{"x":1,"y":2,"z":0}]})",
    "192.168.10.59", 5005, 1.0).accepted);
  EXPECT_TRUE(link.decode_datagram(
    R"({"version":1,"type":"navigation_plan","seq":3,"waypoint_count":2,"waypoints":[{"x":1,"y":2,"z":0.8},{"x":2,"y":2,"z":0.8}]})",
    "192.168.10.59", 5005, 1.0).accepted);
  EXPECT_TRUE(link.decode_datagram(
    R"({"version":1,"type":"navigation_fly_plan_send_ok","seq":4})",
    "192.168.10.59", 5005, 1.0).accepted);
  EXPECT_TRUE(link.decode_datagram(
    R"({"version":1,"type":"ok_fly_plan_succeed","seq":5})",
    "192.168.10.59", 5005, 1.0).accepted);
  const auto bad = link.decode_datagram(
    R"({"version":1,"type":"navigation_plan","seq":6,"waypoint_count":2,"waypoints":[{"x":1,"y":2,"z":0.8}]})",
    "192.168.10.59", 5005, 1.0);
  EXPECT_FALSE(bad.accepted);
  EXPECT_EQ(bad.rejection_reason, "point_count_mismatch");
}

TEST(GroundStationLinkTest, RejectsDuplicateFieldsWrongTypesAndNonFiniteNumbers)
{
  auto link = disabled_link();
  EXPECT_FALSE(link.decode_datagram(
    R"({"version":1,"version":1,"type":"ACK","seq":1})",
    "192.168.10.59", 5005, 1.0).accepted);
  EXPECT_FALSE(link.decode_datagram(
    R"({"version":"1","type":"ACK","seq":2})",
    "192.168.10.59", 5005, 1.0).accepted);
  EXPECT_FALSE(link.decode_datagram(
    R"({"version":1,"type":"start","seq":3,"height_start":1e999})",
    "192.168.10.59", 5005, 1.0).accepted);
}

TEST(GroundStationLinkTest, EnforcesConfiguredTargetEnvelopeAfterOriginIsKnown)
{
  auto link = disabled_link();
  link.set_navigation_origin(10.0, 20.0);
  EXPECT_TRUE(link.decode_datagram(
    R"({"version":1,"type":"navigation_and_point","seq":1,"plan_mode":1,"point":{"x":14,"y":25,"z":1}})",
    "192.168.10.59", 5005, 1.0).accepted);
  const auto outside = link.decode_datagram(
    R"({"version":1,"type":"navigation_plan","seq":2,"waypoint_count":1,"waypoints":[{"x":14.01,"y":20,"z":0.8}]})",
    "192.168.10.59", 5005, 1.0);
  EXPECT_FALSE(outside.accepted);
  EXPECT_EQ(outside.rejection_reason, "point_out_of_safety_envelope");
}

/// 构造具备有限原点和健康标志的 Navigation 测试周期输入。
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

/// 构造已经由通信层接受的最小协议事件测试值。
ProtocolEvent event(MessageType type, std::uint64_t seq, double now)
{
  ProtocolEvent value;
  value.type = type;
  value.seq = seq;
  value.received_at = now;
  value.accepted = true;
  return value;
}

TEST(NavigationStateMachineTest, StartAndFlightRequireExplicitEventsAndMavrosConfirmation)
{
  SafetyConfig config;
  config.setpoint_warmup_s = 0.1;
  Navigation navigation(config);
  auto input = base_input(1.0);
  auto decision = navigation.update(input);
  EXPECT_EQ(decision.phase, "waiting_start");
  EXPECT_FALSE(decision.setpoint);
  input.now = 1.1;
  decision = navigation.update(input);
  EXPECT_EQ(decision.phase, "waiting_start");
  auto start = event(MessageType::start, 1, 1.2);
  start.height_start_m = 0.8;
  input.events = {start}; input.now = 1.2;
  decision = navigation.update(input);
  EXPECT_EQ(decision.phase, "setpoint_warmup");
  input.events.clear(); input.now = 1.31;
  decision = navigation.update(input);
  EXPECT_EQ(decision.phase, "offboard_request_pending");
  EXPECT_FALSE(std::any_of(decision.messages.begin(), decision.messages.end(),
    [](const auto & message) {return message.type == MessageType::ok_flight;}));
  input.controller.mode = "OFFBOARD"; input.now = 1.4;
  EXPECT_EQ(navigation.update(input).phase, "arming_request_pending");
  input.controller.armed = true; input.now = 1.5;
  decision = navigation.update(input);
  EXPECT_EQ(decision.phase, "climb");
  EXPECT_TRUE(std::any_of(decision.messages.begin(), decision.messages.end(),
    [](const auto & message) {return message.type == MessageType::ok_flight;}));
}

/// 驱动假飞控反馈和实测位置，直到状态机进入导航配置等待阶段。
void reach_waiting_config(Navigation & navigation, NavigationInput & input)
{
  navigation.update(input);
  auto start = event(MessageType::start, 1, input.now + 0.1);
  start.height_start_m = 0.2;
  input.now += 0.1; input.events = {start}; navigation.update(input);
  input.events.clear(); input.now += 0.02; navigation.update(input);
  input.controller.mode = "OFFBOARD"; input.now += 0.02; navigation.update(input);
  input.controller.armed = true; input.now += 0.02; navigation.update(input);
  for (int i = 0; i < 500 && navigation.phase() == "climb"; ++i) {
    input.now += 0.05;
    if (navigation.planner().latched()) {
      const auto current = navigation.planner().current();
      input.telemetry.local_x_m = current.x_m;
      input.telemetry.local_y_m = current.y_m;
      input.telemetry.local_z_m = current.z_m;
    }
    navigation.update(input);
  }
  for (int i = 0; i < 10 && navigation.phase() == "stabilize"; ++i) {
    input.now += 0.05;
    const auto current = navigation.planner().current();
    input.telemetry.local_x_m = current.x_m;
    input.telemetry.local_y_m = current.y_m;
    input.telemetry.local_z_m = current.z_m;
    navigation.update(input);
  }
}

/// 构造三类载荷各三份且一致、最后带发送完成标志的导航批次。
std::vector<ProtocolEvent> complete_batch(double now, int plan_mode = 1)
{
  std::vector<ProtocolEvent> result;
  std::uint64_t seq = 10;
  for (int copy = 0; copy < 3; ++copy) {
    auto value = event(MessageType::navigation_and_point, seq++, now);
    value.plan_mode = plan_mode;
    value.final_point = Point3{2.0, 0.0, 0.2};
    result.push_back(value);
  }
  for (int copy = 0; copy < 3; ++copy) {
    auto value = event(MessageType::navigation_nfz, seq++, now);
    value.points = {{5.0, 5.0, 0.0}};
    result.push_back(value);
  }
  for (int copy = 0; copy < 3; ++copy) {
    auto value = event(MessageType::navigation_plan, seq++, now);
    value.points = {{1.0, 0.0, 0.2}};
    result.push_back(value);
  }
  result.push_back(event(MessageType::navigation_fly_plan_send_ok, seq, now));
  return result;
}

TEST(NavigationStateMachineTest, AcceptsExactlyThreeMatchingPacketsAndRejectsPlanModeZero)
{
  SafetyConfig config;
  config.setpoint_warmup_s = 0.01;
  config.hold_seconds = 0.1;
  Navigation navigation(config);
  auto input = base_input(1.0);
  reach_waiting_config(navigation, input);
  ASSERT_EQ(navigation.phase(), "waiting_navigation_config");
  input.events = complete_batch(input.now);
  auto decision = navigation.update(input);
  EXPECT_EQ(decision.phase, "run_fly_plan");
  EXPECT_TRUE(std::any_of(decision.messages.begin(), decision.messages.end(),
    [](const auto & message) {return message.type == MessageType::ok_receive;}));

  Navigation unsupported(config);
  input = base_input(1.0);
  reach_waiting_config(unsupported, input);
  input.events = complete_batch(input.now, 0);
  decision = unsupported.update(input);
  EXPECT_EQ(decision.phase, "waiting_navigation_config");
  EXPECT_NE(std::find(decision.rejections.begin(), decision.rejections.end(),
    "autonomous_planning_not_supported"), decision.rejections.end());
}

TEST(NavigationStateMachineTest, AckTimeoutFreezesAndRecoveryReplans)
{
  SafetyConfig config;
  config.setpoint_warmup_s = 0.01;
  config.hold_seconds = 0.1;
  Navigation navigation(config);
  auto input = base_input(1.0);
  reach_waiting_config(navigation, input);
  input.events = complete_batch(input.now);
  navigation.update(input);
  input.events.clear(); input.now += 2.01;
  auto decision = navigation.update(input);
  EXPECT_EQ(decision.phase, "link_hold");
  input.events = {event(MessageType::ack, 100, input.now + 0.01)};
  input.now += 0.01;
  decision = navigation.update(input);
  EXPECT_EQ(decision.phase, "run_fly_plan");
}

TEST(NavigationStateMachineTest, IncompleteOrInconsistentBatchDoesNotAcknowledgeReceipt)
{
  SafetyConfig config;
  config.setpoint_warmup_s = 0.01;
  config.hold_seconds = 0.1;
  Navigation navigation(config);
  auto input = base_input(1.0);
  reach_waiting_config(navigation, input);
  auto packets = complete_batch(input.now);
  packets.erase(packets.begin());
  input.events = packets;
  auto decision = navigation.update(input);
  EXPECT_EQ(decision.phase, "waiting_navigation_config");
  EXPECT_FALSE(std::any_of(decision.messages.begin(), decision.messages.end(),
    [](const auto & message) {return message.type == MessageType::ok_receive;}));
  EXPECT_NE(std::find(decision.rejections.begin(), decision.rejections.end(),
    "navigation_batch_incomplete_or_inconsistent"), decision.rejections.end());

  packets = complete_batch(input.now + 0.1);
  packets[1].final_point = Point3{2.1, 0.0, 0.2};
  input.now += 0.1;
  input.events = packets;
  decision = navigation.update(input);
  EXPECT_EQ(decision.phase, "waiting_navigation_config");
  EXPECT_FALSE(std::any_of(decision.messages.begin(), decision.messages.end(),
    [](const auto & message) {return message.type == MessageType::ok_receive;}));
}

TEST(NavigationStateMachineTest, ExecutesAllWaypointsThenWaitsForSuccessBeforeLandingAndManual)
{
  SafetyConfig config;
  config.setpoint_warmup_s = 0.01;
  config.hold_seconds = 0.1;
  config.max_flight_seconds = 200.0;
  Navigation navigation(config);
  auto input = base_input(1.0);
  reach_waiting_config(navigation, input);
  input.events = complete_batch(input.now);
  auto decision = navigation.update(input);
  std::uint64_t ack_seq = 100;
  bool saw_second_waypoint = false;
  for (int i = 0; i < 2000 && navigation.phase() == "run_fly_plan"; ++i) {
    input.now += 0.05;
    const auto current = navigation.planner().current();
    input.telemetry.local_x_m = current.x_m;
    input.telemetry.local_y_m = current.y_m;
    input.telemetry.local_z_m = current.z_m;
    input.events = {event(MessageType::ack, ack_seq++, input.now)};
    decision = navigation.update(input);
    saw_second_waypoint = saw_second_waypoint || decision.waypoint_index == 1U;
  }
  EXPECT_TRUE(saw_second_waypoint);
  ASSERT_EQ(navigation.phase(), "awaiting_fly_plan_succeed");
  EXPECT_EQ(navigation.waypoint_index(), 2U);
  input.events.clear(); input.now += 1.0;
  decision = navigation.update(input);
  EXPECT_EQ(decision.phase, "awaiting_fly_plan_succeed");

  input.events = {event(MessageType::ok_fly_plan_succeed, ack_seq++, input.now + 0.01)};
  input.now += 0.01;
  decision = navigation.update(input);
  EXPECT_EQ(decision.phase, "landing");
  for (int i = 0; i < 500 && navigation.phase() == "landing"; ++i) {
    input.now += 0.05;
    const auto current = navigation.planner().current();
    input.telemetry.local_z_m = current.z_m;
    input.events.clear();
    decision = navigation.update(input);
  }
  ASSERT_EQ(navigation.phase(), "disarming");
  EXPECT_EQ(decision.arm_intent, false);
  input.controller.armed = false; input.now += 0.05;
  decision = navigation.update(input);
  EXPECT_EQ(decision.phase, "manual_request_pending");
  EXPECT_EQ(decision.target_mode, "MANUAL");
  input.controller.mode = "MANUAL"; input.now += 0.05;
  EXPECT_EQ(navigation.update(input).phase, "manual");
}

TEST(NavigationStateMachineTest, LcpTimeoutRequestsSafeLanding)
{
  SafetyConfig config;
  config.setpoint_warmup_s = 0.01;
  config.hold_seconds = 0.1;
  config.lcp_unhealthy_hold_timeout_s = 0.2;
  Navigation navigation(config);
  auto input = base_input(1.0);
  reach_waiting_config(navigation, input);
  input.events = complete_batch(input.now);
  navigation.update(input);
  input.events.clear(); input.lcp_healthy = false; input.now += 0.05;
  EXPECT_EQ(navigation.update(input).phase, "lcp_hold");
  input.now += 0.21;
  const auto decision = navigation.update(input);
  EXPECT_EQ(decision.phase, "landing");
  EXPECT_EQ(decision.target_mode, "AUTO.LAND");
}

TEST(OffboardMappingTest, PositionTargetUsesLocalNedFullPositionHold)
{
  const mavros_xyz_position_offboard::common::PositionSetpoint input{
    1.0, -2.0, 3.0, {0.0, 0.0, std::sqrt(0.5), std::sqrt(0.5)}, 0.4};
  builtin_interfaces::msg::Time stamp;
  const auto output = mavros_xyz_position_offboard::offboard::Offboard::make_position_target(input, stamp);
  EXPECT_EQ(output.coordinate_frame, mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED);
  EXPECT_EQ(output.type_mask & mavros_msgs::msg::PositionTarget::IGNORE_PX, 0U);
  EXPECT_NE(output.type_mask & mavros_msgs::msg::PositionTarget::IGNORE_VX, 0U);
  EXPECT_NE(output.type_mask & mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE, 0U);
  EXPECT_DOUBLE_EQ(output.position.x, 1.0);
}

TEST(OffboardStructureTest, AdapterHeaderDoesNotReferenceMissionOrUdp)
{
  const std::string header = MAVROS_XYZ_SOURCE_DIR "/include/mavros_xyz_position_offboard/offboard/offboard.hpp";
  std::ifstream stream(header);
  const std::string text((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
  EXPECT_EQ(text.find("navigation/"), std::string::npos);
  EXPECT_EQ(text.find("GroundStation"), std::string::npos);
  EXPECT_EQ(text.find("phase"), std::string::npos);
}

TEST(CliTest, PreservesSafetyOptInGates)
{
  const std::vector<std::string> basic{
    "node", "--confirmed-fcu-url", "udp://127.0.0.1:14540", "--range-topic", "/range",
    "--range-source-label", "downward", "--optical-flow-topic", "/flow",
    "--optical-flow-source-label", "flow"};
  const auto parsed = mavros_xyz_position_offboard::common::parse_options(basic);
  EXPECT_FALSE(mavros_xyz_position_offboard::common::setpoint_enabled(parsed.options));
  auto incomplete = basic;
  incomplete.emplace_back("--enable-position-setpoints");
  EXPECT_THROW(mavros_xyz_position_offboard::common::parse_options(incomplete), std::invalid_argument);
}

}  // namespace

/// 初始化隔离的 ROS 测试上下文，运行全部 gtest 后正常关闭上下文。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
