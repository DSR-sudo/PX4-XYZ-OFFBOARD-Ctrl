#include "mavros_xyz_position_offboard/navigation/target_tracker.hpp"

#include <algorithm>
#include <cmath>

#include "mavros_xyz_position_offboard/common/types.hpp"

namespace mavros_xyz_position_offboard::navigation
{
namespace
{

constexpr double kInnovationGate99Percent2d = 9.210340371976184;
constexpr double kSmall = 1e-9;

}  // namespace

TargetTracker::TargetTracker(const MissionConfig & config) : config_(config) {}

void TargetTracker::reset()
{
  initialized_ = false;
  samples_ = 0;
  state_time_ = 0.0;
  state_ = {};
  covariance_ = {};
}

void TargetTracker::predict_to(double at)
{
  if (!initialized_ || at <= state_time_) {return;}
  const double dt = at - state_time_;
  const std::array<std::array<double, 4>, 4> transition{{
    {{1.0, 0.0, dt, 0.0}}, {{0.0, 1.0, 0.0, dt}},
    {{0.0, 0.0, 1.0, 0.0}}, {{0.0, 0.0, 0.0, 1.0}}}};
  std::array<double, 4> predicted{};
  std::array<std::array<double, 4>, 4> propagated{};
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 4; ++column) {
      predicted[row] += transition[row][column] * state_[column];
      for (int inner = 0; inner < 4; ++inner) {
        propagated[row][column] += transition[row][inner] * covariance_[inner][column];
      }
    }
  }
  std::array<std::array<double, 4>, 4> predicted_covariance{};
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 4; ++column) {
      for (int inner = 0; inner < 4; ++inner) {
        predicted_covariance[row][column] += propagated[row][inner] * transition[column][inner];
      }
    }
  }
  const double acceleration_variance = std::pow(config_.filter_acceleration_noise_m_s2, 2);
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt2 * dt2;
  const double position_noise = 0.25 * dt4 * acceleration_variance;
  const double cross_noise = 0.5 * dt3 * acceleration_variance;
  const double velocity_noise = dt2 * acceleration_variance;
  predicted_covariance[0][0] += position_noise;
  predicted_covariance[1][1] += position_noise;
  predicted_covariance[0][2] += cross_noise;
  predicted_covariance[2][0] += cross_noise;
  predicted_covariance[1][3] += cross_noise;
  predicted_covariance[3][1] += cross_noise;
  predicted_covariance[2][2] += velocity_noise;
  predicted_covariance[3][3] += velocity_noise;
  state_ = predicted;
  covariance_ = predicted_covariance;
  state_time_ = at;
}

bool TargetTracker::update(double x_m, double y_m, double received_at)
{
  if (!common::finite(x_m) || !common::finite(y_m) || !common::finite(received_at)) {
    return false;
  }
  const double measurement_variance = std::pow(config_.filter_measurement_noise_m, 2);
  if (!initialized_) {
    reset();
    initialized_ = true;
    samples_ = 1;
    state_time_ = received_at;
    state_ = {{x_m, y_m, 0.0, 0.0}};
    covariance_[0][0] = measurement_variance;
    covariance_[1][1] = measurement_variance;
    covariance_[2][2] = 4.0;
    covariance_[3][3] = 4.0;
    return true;
  }
  predict_to(received_at);
  const double innovation_x = x_m - state_[0];
  const double innovation_y = y_m - state_[1];
  const double s00 = covariance_[0][0] + measurement_variance;
  const double s01 = covariance_[0][1];
  const double s11 = covariance_[1][1] + measurement_variance;
  const double determinant = s00 * s11 - s01 * s01;
  if (determinant <= kSmall) {return false;}
  const double nis = (innovation_x * innovation_x * s11 -
    2.0 * innovation_x * innovation_y * s01 + innovation_y * innovation_y * s00) / determinant;
  if (!common::finite(nis) || nis > kInnovationGate99Percent2d) {return false;}
  std::array<std::array<double, 2>, 4> gain{};
  for (int row = 0; row < 4; ++row) {
    gain[row][0] = (covariance_[row][0] * s11 - covariance_[row][1] * s01) / determinant;
    gain[row][1] = (covariance_[row][1] * s00 - covariance_[row][0] * s01) / determinant;
  }
  for (int row = 0; row < 4; ++row) {
    state_[row] += gain[row][0] * innovation_x + gain[row][1] * innovation_y;
  }
  std::array<std::array<double, 4>, 4> corrected{};
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 4; ++column) {
      corrected[row][column] = covariance_[row][column] - gain[row][0] * covariance_[0][column] -
        gain[row][1] * covariance_[1][column];
    }
  }
  for (int row = 0; row < 4; ++row) {
    for (int column = row; column < 4; ++column) {
      const double symmetric = 0.5 * (corrected[row][column] + corrected[column][row]);
      covariance_[row][column] = symmetric;
      covariance_[column][row] = symmetric;
    }
  }
  ++samples_;
  return true;
}

TargetEstimate TargetTracker::estimate(double at) const
{
  TargetEstimate result;
  result.initialized = initialized_;
  result.samples = samples_;
  if (!initialized_) {return result;}
  const double dt = std::max(0.0, at - state_time_);
  result.x_m = state_[0] + state_[2] * dt;
  result.y_m = state_[1] + state_[3] * dt;
  result.vx_m_s = state_[2];
  result.vy_m_s = state_[3];
  return result;
}

std::optional<double> TargetTracker::time_to_distance(
  double own_x_m, double own_y_m, double own_vx_m_s, double own_vy_m_s,
  double distance_m, double now, double horizon_s) const
{
  if (!initialized_ || !common::finite(own_x_m) || !common::finite(own_y_m) ||
    !common::finite(own_vx_m_s) || !common::finite(own_vy_m_s) ||
    !common::finite(distance_m) || !common::finite(now) || !common::finite(horizon_s) ||
    distance_m <= 0.0 || horizon_s <= 0.0) {
    return std::nullopt;
  }
  const auto target = estimate(now);
  const double relative_x = target.x_m - own_x_m;
  const double relative_y = target.y_m - own_y_m;
  const double relative_vx = target.vx_m_s - own_vx_m_s;
  const double relative_vy = target.vy_m_s - own_vy_m_s;
  const double a = relative_vx * relative_vx + relative_vy * relative_vy;
  const double b = relative_x * relative_vx + relative_y * relative_vy;
  const double c = relative_x * relative_x + relative_y * relative_y - distance_m * distance_m;
  if (c <= 0.0) {return 0.0;}
  if (a <= kSmall) {return std::nullopt;}
  const double discriminant = b * b - a * c;
  if (discriminant < 0.0) {return std::nullopt;}
  const double root = (-b - std::sqrt(discriminant)) / a;
  if (root < 0.0 || root > horizon_s) {return std::nullopt;}
  return root;
}

}  // namespace mavros_xyz_position_offboard::navigation
