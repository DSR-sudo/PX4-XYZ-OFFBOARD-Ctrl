#include "mavros_xyz_position_offboard/navigation/mission_config.hpp"

#include <cmath>
#include <stdexcept>

#include "mavros_xyz_position_offboard/common/types.hpp"

namespace mavros_xyz_position_offboard::navigation
{

void MissionConfig::validate() const
{
  const double positive_values[] = {
    takeoff_height_m, height_stable_seconds, b_right_m, b_forward_m, b_arrival_speed_m_s,
    car_tracking_max_speed_m_s, car_tracking_max_accel_m_s2, return_max_speed_m_s,
    return_max_accel_m_s2, target_lock_follow_seconds, throw_distance_m,
    throw_bearing_tolerance_rad, filter_measurement_noise_m,
    filter_acceleration_noise_m_s2, prediction_horizon_s, cardinal_tolerance_deg,
    final_intercept_seconds, car_status_timeout_s, max_tracking_radius_m};
  for (const double value : positive_values) {
    if (!common::finite(value) || value <= 0.0) {
      throw std::invalid_argument("mission configuration values must be finite and positive");
    }
  }
  const double pi = std::acos(-1.0);
  if (!common::finite(throw_bearing_rad) || throw_bearing_rad < -pi || throw_bearing_rad > pi) {
    throw std::invalid_argument("throw bearing must be finite and within [-pi, pi]");
  }
  if (throw_bearing_tolerance_rad > pi) {
    throw std::invalid_argument("throw bearing tolerance must not exceed pi");
  }
  if (filter_min_samples < 3) {
    throw std::invalid_argument("target tracker requires at least three observations");
  }
  if (cardinal_tolerance_deg > 45.0) {
    throw std::invalid_argument("cardinal tolerance must not exceed 45 degrees");
  }
  if (final_intercept_seconds >= prediction_horizon_s) {
    throw std::invalid_argument("final intercept window must be shorter than prediction horizon");
  }
}

}  // namespace mavros_xyz_position_offboard::navigation
