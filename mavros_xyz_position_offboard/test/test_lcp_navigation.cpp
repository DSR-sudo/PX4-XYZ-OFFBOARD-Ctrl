#include <cmath>
#include <memory>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "mavros_xyz_position_offboard/common/cli.hpp"
#include "mavros_xyz_position_offboard/common/types.hpp"
#include "mavros_xyz_position_offboard/bridge/lcp_vision_bridge.hpp"
#include "mavros_xyz_position_offboard/initialization/initialization.hpp"
#include "mavros_xyz_position_offboard/navigation/navigation.hpp"
#include "mavros_xyz_position_offboard/offboard/offboard.hpp"

namespace
{
using mavros_xyz_position_offboard::common::AppOptions;
using mavros_xyz_position_offboard::common::Quaternion;
using mavros_xyz_position_offboard::common::SafetyConfig;
using mavros_xyz_position_offboard::initialization::Initialization;
using mavros_xyz_position_offboard::navigation::Navigation;

class InitializationTest : public ::testing::Test
{
protected:
  /// 为每项 Initialization 测试创建隔离的 ROS 节点和测试话题。
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("mavros_xyz_initialization_test", rclcpp::NodeOptions().use_global_arguments(false));
    options_.range_topic = "/test/range";
    options_.optical_flow_topic = "/test/flow";
    options_.lcp_status_topic = "/test/lcp/status";
    options_.lcp_odometry_topic = "/test/lcp/odometry";
    options_.lcp_start_service = "/test/lcp/start";
  }

  std::shared_ptr<rclcpp::Node> node_;
  AppOptions options_;
};

/// 验证服务请求之前的旧 STATUS=2 不能越过新的 LCP 基线。
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

/// 验证足量的新鲜状态和里程计样本可以满足 LCP 就绪条件。
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
}

/// 验证服务失败和超时的 LCP 遥测均不能通过健康门禁。
TEST_F(InitializationTest, FailedOrStaleLcpIsRejected)
{
  SafetyConfig config;
  Initialization initialization(*node_, options_, config);
  initialization.begin_lcp_initialization(1.0);
  initialization.update_lcp_init_state("failed", 1.0, std::nullopt, "service failure");
  EXPECT_FALSE(initialization.lcp_ready(1.1));
  const auto errors = initialization.lcp_errors(1.1);
  EXPECT_NE(std::find(errors.begin(), errors.end(), "service failure"), errors.end());
  initialization.update_lcp_init_state("accepted", 2.0, "started");
  initialization.update_lcp_status(2, 2.1);
  initialization.update_lcp_odometry(1.0, 2.0, 0.0, 2.1);
  EXPECT_FALSE(initialization.lcp_runtime_healthy(3.0));
}

/// 验证 LCP-hold 冻结使用本地位置，并且 yaw=0 不借用 LCP 坐标。
TEST(NavigationTest, FreezeAndYawZeroDoNotUseLcpCoordinates)
{
  SafetyConfig config;
  Navigation navigation(config);
  navigation.latch(10.0, 20.0, 0.0, {0.0, 0.0, 0.7071, 0.7071});
  navigation.set_relative_target(1.0);
  navigation.update(0.2);
  navigation.freeze_xy_at(10.25, 20.5);
  navigation.freeze_z();
  EXPECT_DOUBLE_EQ(navigation.x_m(), 10.25);
  EXPECT_DOUBLE_EQ(navigation.y_m(), 20.5);
  EXPECT_DOUBLE_EQ(navigation.target_z_m(), navigation.command_z_m());
  navigation.set_yaw_rad(0.0);
  EXPECT_NEAR(navigation.yaw_rad(), 0.0, 1e-9);
}

/// 验证人工北向 lcp_nwu 同时正确转换 XY、yaw 和协方差到 MAVROS 的 ROS ENU。
TEST(LcpVisionBridgeTest, ConvertsNwuToEnuWithConservativeCovariance)
{
  nav_msgs::msg::Odometry source;
  source.header.frame_id = "lcp_nwu";
  source.pose.pose.position.x = 2.0;   // north
  source.pose.pose.position.y = 3.0;   // west
  source.pose.pose.position.z = 0.0;
  source.pose.pose.orientation.w = 1.0;  // north-facing yaw=0 in lcp_nwu
  const auto output = mavros_xyz_position_offboard::bridge::LcpVisionBridge::nwu_to_enu(source, 0.20, 0.20);
  EXPECT_EQ(output.header.frame_id, "lcp_enu");
  EXPECT_DOUBLE_EQ(output.pose.pose.position.x, -3.0);  // east
  EXPECT_DOUBLE_EQ(output.pose.pose.position.y, 2.0);   // north
  EXPECT_NEAR(output.pose.pose.orientation.z, std::sqrt(0.5), 1e-12);
  EXPECT_NEAR(output.pose.pose.orientation.w, std::sqrt(0.5), 1e-12);
  EXPECT_DOUBLE_EQ(output.pose.covariance[0], 0.04);
  EXPECT_DOUBLE_EQ(output.pose.covariance[7], 0.04);
  EXPECT_DOUBLE_EQ(output.pose.covariance[14], 10000.0);
  EXPECT_DOUBLE_EQ(output.pose.covariance[35], 0.04);
}

/// 验证五次轨迹的速度和加速度不超过配置上限。
TEST(NavigationTest, QuinticSetpointsObserveConfiguredBounds)
{
  SafetyConfig config;
  config.max_z_setpoint_rate_m_s = 0.20;
  config.max_z_setpoint_accel_m_s2 = 0.40;
  config.waypoint_max_speed_m_s = 0.25;
  config.waypoint_max_accel_m_s2 = 0.50;
  Navigation navigation(config);
  navigation.latch(0.0, 0.0, 0.0, {0.0, 0.0, 0.0, 1.0});
  navigation.set_relative_target(1.0);
  navigation.set_xy_target(0.5, 0.0);
  auto previous = navigation.current();
  double previous_vertical_rate = previous.vertical_rate_m_s;
  for (int i = 0; i < 2000; ++i) {
    const auto current = navigation.update(0.01);
    EXPECT_LE(std::abs(current.vertical_rate_m_s), config.max_z_setpoint_rate_m_s * 1.001);
    EXPECT_LE(std::abs(current.vertical_rate_m_s - previous_vertical_rate) / 0.01, config.max_z_setpoint_accel_m_s2 * 1.01);
    EXPECT_LE(std::hypot(current.x_m - previous.x_m, current.y_m - previous.y_m) / 0.01, config.waypoint_max_speed_m_s * 1.001);
    previous = current;
    previous_vertical_rate = current.vertical_rate_m_s;
  }
}

/// 验证 OFFBOARD 映射固定使用 LOCAL_NED 和完整 XYZ+yaw 保持掩码。
TEST(OffboardMappingTest, PositionTargetUsesLocalNedFullPositionHold)
{
  const mavros_xyz_position_offboard::common::PositionSetpoint input{1.0, -2.0, 3.0, {0.0, 0.0, std::sqrt(0.5), std::sqrt(0.5)}, 0.4};
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 12;
  const auto output = mavros_xyz_position_offboard::offboard::MavrosNativeXYZNode::make_position_target(input, stamp);
  EXPECT_EQ(output.coordinate_frame, mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED);
  EXPECT_EQ(output.type_mask & mavros_msgs::msg::PositionTarget::IGNORE_PX, 0U);
  EXPECT_EQ(output.type_mask & mavros_msgs::msg::PositionTarget::IGNORE_PY, 0U);
  EXPECT_EQ(output.type_mask & mavros_msgs::msg::PositionTarget::IGNORE_PZ, 0U);
  EXPECT_NE(output.type_mask & mavros_msgs::msg::PositionTarget::IGNORE_VX, 0U);
  EXPECT_NE(output.type_mask & mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE, 0U);
  EXPECT_DOUBLE_EQ(output.position.x, 1.0);
  EXPECT_DOUBLE_EQ(output.position.y, -2.0);
  EXPECT_DOUBLE_EQ(output.position.z, 3.0);
  EXPECT_NEAR(output.yaw, std::acos(-1.0) / 2.0, 1e-6);
}

/// 验证应用默认约束及残缺危险操作确认都会被拒绝。
TEST(CliTest, PreservesApplicationDefaultsAndRejectsPartialOptIn)
{
  const std::vector<std::string> basic{
    "mavros_xyz_position_node", "--confirmed-fcu-url", "udp://127.0.0.1:14540",
    "--range-topic", "/range", "--range-source-label", "downward",
    "--optical-flow-topic", "/flow", "--optical-flow-source-label", "flow"};
  const auto parsed = mavros_xyz_position_offboard::common::parse_options(basic);
  EXPECT_DOUBLE_EQ(parsed.config.max_z_setpoint_rate_m_s, 0.20);
  EXPECT_DOUBLE_EQ(parsed.config.max_z_setpoint_accel_m_s2, 0.40);
  EXPECT_DOUBLE_EQ(parsed.config.max_flight_horizontal_speed_m_s, 0.50);
  EXPECT_TRUE(parsed.options.lcp_vision_bridge_enabled);
  EXPECT_EQ(parsed.options.lcp_vision_input_frame, "lcp_nwu");
  EXPECT_FALSE(mavros_xyz_position_offboard::common::setpoint_enabled(parsed.options));
  auto incomplete = basic;
  incomplete.emplace_back("--enable-position-setpoints");
  EXPECT_THROW(mavros_xyz_position_offboard::common::parse_options(incomplete), std::invalid_argument);
}

}  // namespace

/// 初始化 rclcpp 后运行全部 C++ 单元测试并正常关闭上下文。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
