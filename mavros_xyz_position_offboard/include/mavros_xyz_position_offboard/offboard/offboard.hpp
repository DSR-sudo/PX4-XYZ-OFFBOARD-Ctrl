#pragma once

#include <optional>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mavros_xyz_position_offboard/common/cli.hpp"
#include "mavros_xyz_position_offboard/common/types.hpp"

namespace mavros_xyz_position_offboard::offboard
{

struct OffboardCommand
{
  std::optional<common::PositionSetpoint> setpoint{};
  std::optional<std::string> target_mode{};
  std::optional<bool> arm_intent{};
};

struct RequestStatus
{
  std::string state{"never_requested"};
  std::optional<bool> accepted{};
  std::optional<std::string> detail{};
  std::optional<double> at{};
};

struct OffboardStatus
{
  bool connected{false};
  bool armed{false};
  std::string mode{};
  RequestStatus mode_request{};
  RequestStatus arm_request{};
};

/// Thin MAVROS execution adapter. It contains no mission or communication state.
class Offboard
{
public:
  /// 按 CLI 权限创建设定点发布器以及模式和 ARM 服务客户端。
  Offboard(rclcpp::Node & node, const common::AppOptions & options);

  /// 幂等应用可选设定点、目标模式和 ARM/Disarm 控制意图。
  void apply(const OffboardCommand & command, double now);
  /// 非阻塞轮询模式和 ARM 异步请求的响应或超时。
  void poll(double now);
  /// 返回飞控实际状态和最近服务请求结果的值拷贝。
  OffboardStatus status() const {return status_;}
  /// 从 Initialization 遥测同步飞控连接、ARM 和实际模式确认状态。
  void observe_flight_state(const common::Telemetry & telemetry);

  /// 构造 ROS ENU XYZ+yaw 的 MAVROS LOCAL_NED PositionTarget，由 MAVROS 转换给 PX4。
  static mavros_msgs::msg::PositionTarget make_position_target(
    const common::PositionSetpoint & setpoint, const builtin_interfaces::msg::Time & stamp);

private:
  /// 在服务可用、无在途请求且满足限频时异步请求指定飞控模式。
  void request_mode(const std::string & mode, double now);
  /// 在服务可用、无在途请求且满足限频时异步请求普通 ARM 或 Disarm。
  void request_arm(bool value, double now);

  rclcpp::Node & node_;
  const common::AppOptions & options_;
  bool publish_enabled_{false};
  bool mode_enabled_{false};
  bool arming_enabled_{false};
  OffboardStatus status_{};
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_publisher_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  std::optional<rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture> mode_future_;
  std::optional<rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture> arm_future_;
  std::optional<double> mode_future_started_at_{};
  std::optional<double> arm_future_started_at_{};
  double last_mode_request_at_{-1.0e100};
  double last_arm_request_at_{-1.0e100};
  std::optional<std::string> pending_mode_{};
  std::optional<bool> pending_arm_{};
};

}  // namespace mavros_xyz_position_offboard::offboard
