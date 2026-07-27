#pragma once

#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "mavros_xyz_position_offboard/common/cli.hpp"

namespace mavros_xyz_position_offboard::bridge
{

/// 将已经人工北向建系的 LCP 位姿安全转换为 MAVROS 所需的 ROS ENU 外部视觉位姿。
class LcpVisionBridge
{
public:
  /// 使用主节点的 LCP/MAVROS 话题配置创建自动桥接；默认启用，不创建飞行设定点。
  LcpVisionBridge(rclcpp::Node & node, const common::AppOptions & options);

  /// 将一条 lcp_nwu 里程计转换为 lcp_enu 外部视觉消息，供单元测试和回调共用。
  static geometry_msgs::msg::PoseWithCovarianceStamped nwu_to_enu(
    const nav_msgs::msg::Odometry & source, double xy_stddev_m, double yaw_stddev_rad);

  bool enabled() const {return enabled_;}
  std::uint64_t published_count() const {return published_count_;}
  const std::string & last_reject_reason() const {return last_reject_reason_;}

private:
  /// 接收 LCP 健康状态，并以单调时钟判断其新鲜度。
  void status_callback(const std_msgs::msg::UInt8::SharedPtr message);
  /// 在新鲜 STATUS=2、帧名正确且数值有限时执行一次完整 NWU->ENU 桥接。
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr message);
  /// 记录拒绝原因；相同原因只记录一次，防止 10 Hz 日志刷屏。
  void reject(const std::string & reason);
  static double monotonic_now();

  rclcpp::Node & node_;
  bool enabled_{false};
  std::string input_frame_;
  double xy_stddev_m_{0.20};
  double yaw_stddev_rad_{0.20};
  double max_status_age_s_{0.35};
  std::uint8_t lcp_status_{0};
  double lcp_status_at_{-std::numeric_limits<double>::infinity()};
  std::uint64_t published_count_{0};
  std::string last_reject_reason_{"bridge not initialized"};
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr status_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
};

}  // namespace mavros_xyz_position_offboard::bridge
