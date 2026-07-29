#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace mavros_xyz_position_offboard::common
{

constexpr int MAV_STATE_UNINIT = 0;
constexpr int MAV_STATE_BOOT = 1;
constexpr int MAV_STATE_CALIBRATING = 2;
constexpr int MAV_STATE_STANDBY = 3;
constexpr int MAV_STATE_ACTIVE = 4;
constexpr int MAV_LANDED_STATE_ON_GROUND = 1;
constexpr std::uint32_t MAV_SYS_STATUS_PREARM_CHECK = 1U << 28U;
constexpr std::uint32_t MAV_SYS_STATUS_ACCEPTABLE_UNHEALTHY_MASK =
  (1U << 8U) | (1U << 9U) | (1U << 10U) | (1U << 11U) | (1U << 14U) | (1U << 16U);

/// 判断数值是否为可用于控制计算的有限值。
inline bool finite(double value) {return std::isfinite(value);}
/// 将数值限制在给定的闭区间内。
inline double clamp(double value, double low, double high) {return std::min(std::max(value, low), high);}

struct Quaternion
{
  double x{NAN};
  double y{NAN};
  double z{NAN};
  double w{NAN};
};

/// 校验并归一化局部位姿四元数，失败时抛出异常。
Quaternion normalize_quaternion(double x, double y, double z, double w);
/// 从规范化四元数计算绕 Z 轴的偏航角。
double yaw_from_quaternion(const Quaternion & orientation);

struct PositionSetpoint
{
  double x_m{NAN};
  double y_m{NAN};
  double z_m{NAN};
  Quaternion orientation{};
  double vertical_rate_m_s{0.0};
};

/// Original ROS time retained with a sensor sample for protocol forwarding.
struct RosTimestamp
{
  std::int32_t sec{0};
  std::uint32_t nanosec{0};
};

struct RangeResult
{
  bool accepted{false};
  std::optional<std::string> reason{};
};

struct SafetyConfig
{
  double state_timeout_s{1.5};
  double sys_status_timeout_s{2.0};
  double battery_timeout_s{2.0};
  double landed_timeout_s{2.0};
  double local_pose_timeout_s{0.5};
  double local_velocity_timeout_s{0.5};
  double estimator_timeout_s{1.5};
  double range_timeout_s{0.35};
  double optical_flow_timeout_s{0.35};
  double sensor_loss_grace_s{2.0};
  double lcp_status_timeout_s{0.75};
  double lcp_odometry_timeout_s{0.75};
  int lcp_ready_samples{3};
  double lcp_unhealthy_hold_timeout_s{2.0};
  double range_boundary_tolerance_m{0.001};
  double configured_min_range_m{0.02};
  double configured_max_range_m{12.0};
  double max_range_jump_m{0.30};
  double jump_window_s{0.30};
  int jump_recovery_samples{3};
  double jump_settle_tolerance_m{0.06};
  int min_optical_flow_quality{20};
  double min_battery_voltage_v{14.0};
  double min_battery_fraction{0.30};
  double max_preflight_horizontal_speed_m_s{0.20};
  double max_preflight_vertical_speed_m_s{0.20};
  double max_flight_horizontal_speed_m_s{1.0};
  double max_flight_vertical_speed_m_s{1.5};
  double max_flight_horizontal_drift_m{1.5};
  double climb_horizontal_speed_limit_m_s{3.0};
  double climb_horizontal_drift_limit_m{3.0};
  double hover_min_height_m{0.20};
  double publish_rate_hz{20.0};
  double setpoint_warmup_s{2.0};
  double relative_z_m{0.80};
  double max_z_setpoint_rate_m_s{1.0};
  double max_z_setpoint_accel_m_s2{1.50};
  double target_xy_max_speed_m_s{0.25};
  double target_xy_max_accel_m_s2{0.50};
  double target_tolerance_m{0.04};
  double touchdown_z_tolerance_m{0.08};
  double hold_seconds{10.0};
  double max_flight_seconds{60.0};
  double flow_effective_min_height_m{0.35};
  int flow_effective_min_quality{20};
  bool range_source_confirmed{false};
  bool optical_flow_source_confirmed{false};
  bool ignore_declared_min_range{true};

  /// 校验所有安全阈值和控制约束的取值范围。
  void validate() const;
};

struct Telemetry
{
  std::optional<double> state_at{};
  bool connected{false};
  bool armed{false};
  std::string mode{};
  int system_status{-1};
  std::optional<double> sys_status_at{};
  std::uint32_t sensors_present{0};
  std::uint32_t sensors_enabled{0};
  std::uint32_t sensors_health{0};
  bool prearm_bit_advisory{false};

  std::optional<double> battery_at{};
  bool battery_present{false};
  double battery_voltage_v{NAN};
  double battery_fraction{NAN};
  std::optional<double> landed_at{};
  int landed_state{0};

  std::optional<double> local_pose_at{};
  std::optional<RosTimestamp> local_pose_stamp{};
  double local_x_m{NAN};
  double local_y_m{NAN};
  double local_z_m{NAN};
  Quaternion orientation{};
  std::optional<double> local_velocity_at{};
  double velocity_x_m_s{NAN};
  double velocity_y_m_s{NAN};
  double velocity_z_m_s{NAN};

  std::optional<double> estimator_at{};
  bool estimator_attitude_valid{false};
  bool estimator_velocity_horiz_valid{false};
  bool estimator_velocity_vert_valid{false};
  bool estimator_pos_horiz_rel_valid{false};
  bool estimator_pos_horiz_abs_valid{false};
  bool estimator_pos_vert_abs_valid{false};
  bool estimator_pos_vert_agl_valid{false};
  bool estimator_const_pos_mode{false};
  bool estimator_gps_glitch{false};
  bool estimator_accel_error{false};

  std::optional<double> range_at{};
  std::optional<RosTimestamp> range_stamp{};
  double range_m{NAN};
  double range_min_m{NAN};
  double range_max_m{NAN};
  std::optional<double> optical_flow_at{};
  std::uint32_t optical_flow_integration_time_us{0};
  double optical_flow_integrated_x_rad{NAN};
  double optical_flow_integrated_y_rad{NAN};
  int optical_flow_quality{-1};
  double optical_flow_distance_m{NAN};
  std::uint32_t optical_flow_distance_delta_us{0};
  double optical_flow_temperature_c{NAN};

  std::optional<int> lcp_status{};
  std::optional<double> lcp_status_at{};
  std::uint64_t lcp_status_sequence{0};
  std::optional<double> lcp_odometry_at{};
  std::uint64_t lcp_odometry_sequence{0};
  double lcp_x_m{NAN};
  double lcp_y_m{NAN};
  double lcp_yaw_rad{NAN};
  int lcp_healthy_samples{0};
  std::uint64_t lcp_init_status_sequence_baseline{0};
  std::uint64_t lcp_init_odometry_sequence_baseline{0};
  std::string lcp_init_request_state{"not_requested"};
  std::optional<double> lcp_init_requested_at{};
  std::optional<double> lcp_init_response_at{};
  std::optional<std::string> lcp_init_response_message{};
  std::optional<std::string> lcp_init_failure_reason{};
};

/// 根据单调时间戳判断遥测是否缺失、倒退或超时。
inline bool stale(const std::optional<double> & timestamp, double now, double timeout_s)
{
  return !timestamp || now < *timestamp || now - *timestamp > timeout_s;
}

}  // namespace mavros_xyz_position_offboard::common
