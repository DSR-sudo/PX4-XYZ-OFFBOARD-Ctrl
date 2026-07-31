#pragma once

#include <optional>
#include <string>
#include <vector>

#include "mavros_xyz_position_offboard/common/types.hpp"
#include "mavros_xyz_position_offboard/communication/protocol.hpp"

namespace mavros_xyz_position_offboard::navigation
{

struct ControllerFeedback
{
  bool connected{false};
  bool armed{false};
  std::string mode{};
  std::string mode_request_status{"never_requested"};
  std::string arm_request_status{"never_requested"};
};

/// 导航状态机在一个控制周期内消费的健康、协议和飞控反馈。
struct NavigationInput
{
  double now{0.0};
  double dt{0.05};
  common::Telemetry telemetry{};
  bool preflight_ready{false};
  bool flight_healthy{true};
  bool lcp_healthy{false};
  std::vector<std::string> health_errors{};
  std::vector<communication::ProtocolEvent> events{};
  ControllerFeedback controller{};
  bool gripper_succeeded{false};
  bool gripper_failed{false};
};

/// 任务语义与实际发布命令严格分离的可审计控制状态。
struct ControlState
{
  /// 解锁确认后一次性锁定的本地 ENU 飞行原点；预热期间为空。
  std::optional<common::PositionSetpoint> origin{};
  /// 当前任务最终 XYZ+yaw；临时保持永不改写它。
  std::optional<common::PositionSetpoint> mission_goal{};
  /// 最近一次交给 Offboard 发布的 ENU XYZ+yaw 命令。
  std::optional<common::PositionSetpoint> commanded_setpoint{};
  /// LCP 失效或视觉超时时锁定的实测 XYZ 与最近命令偏航。
  std::optional<common::PositionSetpoint> hold_setpoint{};
  std::string hold_reason{};
  std::string hold_resume_phase{};
  bool mission_paused{false};
  bool tracking_arrival_time_met{true};
  int target_samples{0};
  std::optional<double> predicted_intercept_seconds{};
};

/// 将完整控制状态编码为 JSON 对象，供 JSONL 审计和单元测试共同使用。
std::string control_json(const ControlState & control);

/// 导航状态机为当前控制周期生成的控制与通信决策。
struct NavigationDecision
{
  std::string phase{"waiting_preflight"};
  std::optional<common::PositionSetpoint> setpoint{};
  std::optional<std::string> target_mode{};
  std::optional<bool> arm_intent{};
  std::vector<communication::OutgoingMessage> messages{};
  std::vector<std::string> rejections{};
  bool release_gripper{false};
  ControlState control{};
};

}  // namespace mavros_xyz_position_offboard::navigation
