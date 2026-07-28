#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace mavros_xyz_position_offboard::communication
{

struct Point3
{
  double x_m{0.0};
  double y_m{0.0};
  double z_m{0.0};

  /// 精确比较两个协议点的三个坐标，用于三份业务载荷一致性判断。
  bool operator==(const Point3 & other) const;
  /// 返回两个协议点是否存在任一坐标差异。
  bool operator!=(const Point3 & other) const {return !(*this == other);}
};

enum class MessageType
{
  start,
  navigation_and_point,
  navigation_nfz,
  navigation_plan,
  navigation_fly_plan_send_ok,
  ack,
  ok_fly_plan_succeed,
  ok_preflight,
  ok_flight,
  wait_plan,
  ok_receive,
  ok_fly_plan,
  xyz_state,
  battery_state,
  invalid
};

/// 将内部消息枚举转换为 JSON V1 规定的 type 字符串。
std::string to_string(MessageType type);

struct ProtocolEvent
{
  MessageType type{MessageType::invalid};
  std::uint64_t seq{0};
  double received_at{0.0};
  bool accepted{false};
  std::string rejection_reason{};
  std::optional<double> height_start_m{};
  std::optional<int> plan_mode{};
  std::optional<Point3> final_point{};
  std::vector<Point3> points{};
};

struct OutgoingMessage
{
  MessageType type{MessageType::invalid};
  std::string detail{};
};

}  // namespace mavros_xyz_position_offboard::communication
