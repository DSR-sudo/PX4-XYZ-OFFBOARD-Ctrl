#include "mavros_xyz_position_offboard/common/types.hpp"

#include <algorithm>
#include <array>
#include <sstream>

namespace mavros_xyz_position_offboard::common
{

/// 校验有限性和范数后返回单位四元数。
Quaternion normalize_quaternion(double x, double y, double z, double w)
{
  if (!finite(x) || !finite(y) || !finite(z) || !finite(w)) {
    throw std::invalid_argument("local-pose quaternion must be finite");
  }
  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  if (norm < 1e-6) {
    throw std::invalid_argument("local-pose quaternion norm is too small");
  }
  return {x / norm, y / norm, z / norm, w / norm};
}

/// 使用标准 ZYX 公式提取四元数的偏航分量。
double yaw_from_quaternion(const Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

/// 逐项检查安全配置，拒绝无效阈值和不兼容范围。
void SafetyConfig::validate() const
{
  const std::array<std::pair<const char *, double>, 38> positive{{
    {"state_timeout_s", state_timeout_s}, {"sys_status_timeout_s", sys_status_timeout_s},
    {"battery_timeout_s", battery_timeout_s}, {"landed_timeout_s", landed_timeout_s},
    {"local_pose_timeout_s", local_pose_timeout_s}, {"local_velocity_timeout_s", local_velocity_timeout_s},
    {"estimator_timeout_s", estimator_timeout_s}, {"range_timeout_s", range_timeout_s},
    {"optical_flow_timeout_s", optical_flow_timeout_s}, {"sensor_loss_grace_s", sensor_loss_grace_s},
    {"lcp_status_timeout_s", lcp_status_timeout_s}, {"lcp_odometry_timeout_s", lcp_odometry_timeout_s},
    {"range_boundary_tolerance_m", range_boundary_tolerance_m},
    {"configured_min_range_m", configured_min_range_m}, {"configured_max_range_m", configured_max_range_m},
    {"max_range_jump_m", max_range_jump_m}, {"jump_window_s", jump_window_s},
    {"jump_settle_tolerance_m", jump_settle_tolerance_m}, {"min_battery_voltage_v", min_battery_voltage_v},
    {"max_preflight_horizontal_speed_m_s", max_preflight_horizontal_speed_m_s},
    {"max_preflight_vertical_speed_m_s", max_preflight_vertical_speed_m_s},
    {"max_flight_horizontal_speed_m_s", max_flight_horizontal_speed_m_s},
    {"max_flight_vertical_speed_m_s", max_flight_vertical_speed_m_s},
    {"max_flight_horizontal_drift_m", max_flight_horizontal_drift_m},
    {"climb_horizontal_speed_limit_m_s", climb_horizontal_speed_limit_m_s},
    {"climb_horizontal_drift_limit_m", climb_horizontal_drift_limit_m}, {"hover_min_height_m", hover_min_height_m},
    {"setpoint_warmup_s", setpoint_warmup_s}, {"relative_z_m", relative_z_m},
    {"max_z_setpoint_rate_m_s", max_z_setpoint_rate_m_s}, {"max_z_setpoint_accel_m_s2", max_z_setpoint_accel_m_s2},
    {"target_xy_max_speed_m_s", target_xy_max_speed_m_s},
    {"target_xy_max_accel_m_s2", target_xy_max_accel_m_s2},
    {"target_tolerance_m", target_tolerance_m}, {"touchdown_z_tolerance_m", touchdown_z_tolerance_m},
    {"hold_seconds", hold_seconds}, {"max_flight_seconds", max_flight_seconds},
    {"flow_effective_min_height_m", flow_effective_min_height_m},
  }};
  std::vector<std::string> bad;
  for (const auto & [name, value] : positive) {
    if (!finite(value) || value <= 0.0) {
      bad.emplace_back(name);
    }
  }
  if (!bad.empty()) {
    std::ostringstream stream;
    stream << "configuration values must be positive: ";
    for (std::size_t i = 0; i < bad.size(); ++i) {
      if (i) {stream << ", ";}
      stream << bad[i];
    }
    throw std::invalid_argument(stream.str());
  }
  if (configured_max_range_m <= configured_min_range_m) {
    throw std::invalid_argument("configured maximum range must exceed minimum range");
  }
  if (jump_recovery_samples < 2) {
    throw std::invalid_argument("jump recovery requires at least two samples");
  }
  if (lcp_ready_samples < 1) {
    throw std::invalid_argument("LCP ready samples must be at least one");
  }
  if (min_optical_flow_quality < 1 || min_optical_flow_quality > 255) {
    throw std::invalid_argument("optical-flow quality must be within 1..255");
  }
  if (publish_rate_hz < 10.0 || publish_rate_hz > 50.0) {
    throw std::invalid_argument("setpoint publication must be within 10..50 Hz");
  }
  if (min_battery_fraction <= 0.0 || min_battery_fraction > 1.0) {
    throw std::invalid_argument("minimum battery fraction must be within (0, 1]");
  }
}

}  // namespace mavros_xyz_position_offboard::common
