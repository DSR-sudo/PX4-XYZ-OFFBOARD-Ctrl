#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "mavros_xyz_position_offboard/common/types.hpp"

namespace mavros_xyz_position_offboard::communication
{

/// Names registered by the V2 UAV--GCS JSON protocol.
enum class MessageType
{
  run_plan1,
  go_ahead_ok,
  car_status,
  match_car_ok,
  b_ok,
  ack,
  ok_wait,
  ok_height,
  ok_throw,
  ok_return,
  ok_downing,
  ok_down,
  xyzstatus,
  invalid
};

/// Converts a protocol enum to its exact wire-level header value.
std::string to_string(MessageType type);
/// True only for the ordered, ACK-required UAV event headers.
bool is_discrete_event(MessageType type);

/// Continuous visual vehicle measurement in the UAV body frame.
struct CarStatus
{
  double distance_m{0.0};
  double bearing_rad{0.0};
};

struct ProtocolEvent
{
  MessageType type{MessageType::invalid};
  double received_at{0.0};
  bool accepted{false};
  std::string rejection_reason{};
  std::optional<CarStatus> car_status{};
};

/// A ROS header represented without a ROS dependency in the protocol value layer.
struct RosHeader
{
  std::int32_t stamp_sec{0};
  std::uint32_t stamp_nanosec{0};
  std::string frame_id{};
};

/// Full LCP debug sample with its task-relative altitude metadata.
struct XyzStatus
{
  RosHeader header{};
  std::uint8_t status{0};
  bool map_locked{false};
  bool pose_valid{false};
  double position_x_m{0.0};
  double position_y_m{0.0};
  std::optional<double> position_z_m{};
  std::string z_source{"none"};
  std::optional<common::RosTimestamp> z_source_stamp{};
  std::optional<double> z_quality{};
  bool z_valid{false};
  double yaw_rad{0.0};
  double front_distance_m{0.0};
  double rear_distance_m{0.0};
  double left_distance_m{0.0};
  double right_distance_m{0.0};
  double map_size_x_m{0.0};
  double map_size_y_m{0.0};
};

struct OutgoingMessage
{
  MessageType type{MessageType::invalid};
};

}  // namespace mavros_xyz_position_offboard::communication
