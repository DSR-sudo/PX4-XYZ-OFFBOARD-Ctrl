#include "mavros_xyz_position_offboard/navigation/mission_config.hpp"

#include <stdexcept>

#include "mavros_xyz_position_offboard/common/types.hpp"

namespace mavros_xyz_position_offboard::navigation
{

void MissionConfig::validate() const
{
  const double values[] = {
    takeoff_height_m, height_stable_seconds, b_right_m, b_forward_m,
    throw_distance_m, filter_measurement_noise_m,
    filter_acceleration_noise_m_s2, prediction_horizon_s, cardinal_tolerance_deg,
    final_intercept_seconds, car_status_timeout_s, max_tracking_radius_m};
  for (const double value : values) {
    if (!common::finite(value) || value <= 0.0) {
      throw std::invalid_argument("mission configuration values must be finite and positive");
    }
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
