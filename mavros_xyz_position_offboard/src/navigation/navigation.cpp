#include "mavros_xyz_position_offboard/navigation/navigation.hpp"

#include "mavros_xyz_position_offboard/common/artifact_log.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace mavros_xyz_position_offboard::navigation
{

/// 用与 Offboard 相同的 ENU XYZ+yaw 表达审计每一个控制状态点。
std::string control_json(const ControlState & control)
{
  const auto setpoint_json = [](const std::optional<common::PositionSetpoint> & setpoint) {
      if (!setpoint) {return std::string("null");}
      std::ostringstream encoded;
      encoded << std::setprecision(12) << "{\"x_m\":" << setpoint->x_m
              << ",\"y_m\":" << setpoint->y_m << ",\"z_m\":" << setpoint->z_m
              << ",\"yaw_rad\":" << common::yaw_from_quaternion(setpoint->orientation)
              << ",\"vertical_rate_m_s\":" << setpoint->vertical_rate_m_s << '}';
      return encoded.str();
    };
  std::ostringstream encoded;
  encoded << "{\"origin\":" << setpoint_json(control.origin)
          << ",\"mission_goal\":" << setpoint_json(control.mission_goal)
          << ",\"commanded_setpoint\":" << setpoint_json(control.commanded_setpoint)
          << ",\"hold_setpoint\":" << setpoint_json(control.hold_setpoint)
          << ",\"hold_reason\":\"" << common::json_escape(control.hold_reason)
          << "\",\"hold_resume_phase\":\"" << common::json_escape(control.hold_resume_phase)
          << "\",\"mission_paused\":" << (control.mission_paused ? "true" : "false")
          << ",\"tracking_arrival_time_met\":" <<
    (control.tracking_arrival_time_met ? "true" : "false")
          << ",\"target_samples\":" << control.target_samples
          << ",\"predicted_intercept_seconds\":";
  if (control.predicted_intercept_seconds) {encoded << *control.predicted_intercept_seconds;}
  else {encoded << "null";}
  encoded << '}';
  return encoded.str();
}

/// 保存共享配置引用，供所有轨迹约束计算使用。
TrajectoryPlanner::TrajectoryPlanner(const common::SafetyConfig & config) : config_(config) {}

/// 恢复未锁存状态并丢弃所有进行中的轨迹。
void TrajectoryPlanner::reset()
{
  latched_ = false; flow_effective_ = false; x_m_ = NAN; y_m_ = NAN;
  target_x_m_ = NAN; target_y_m_ = NAN; xy_velocity_x_m_s_ = 0.0; xy_velocity_y_m_s_ = 0.0;
  xy_trajectory_elapsed_s_ = 0.0; xy_trajectory_duration_s_ = 0.0; xy_arrival_time_met_ = true;
  xy_coefficients_ = {};
  command_z_m_ = NAN; target_z_m_ = NAN; vertical_rate_m_s_ = 0.0; orientation_ = {};
  trajectory_elapsed_s_ = 0.0; trajectory_duration_s_ = 0.0; coefficients_ = {};
}

/// 对依赖当前命令点的操作实施“必须先初始化”的不变量。
void TrajectoryPlanner::require_latched(const char * action) const
{
  if (!latched_) {throw std::runtime_error(std::string("position must be latched before ") + action);}
}

/// 计算零初始加速度、零终点速度/加速度的五次轨迹系数。
TrajectoryPlanner::Coefficients TrajectoryPlanner::quintic_coefficients(double start, double rate, double target, double duration_s)
{
  const double delta = target - start;
  const double t = duration_s;
  return {start, rate, 0.0,
    (20.0 * delta - 12.0 * rate * t) / (2.0 * std::pow(t, 3)),
    (-30.0 * delta + 16.0 * rate * t) / (2.0 * std::pow(t, 4)),
    (12.0 * delta - 6.0 * rate * t) / (2.0 * std::pow(t, 5))};
}

/// 求出五次曲线在指定时间的位置、速度和加速度。
std::array<double, 3> TrajectoryPlanner::evaluate(const Coefficients & a, double t)
{
  const double p = a[0] + a[1] * t + a[2] * t * t + a[3] * std::pow(t, 3) + a[4] * std::pow(t, 4) + a[5] * std::pow(t, 5);
  const double v = a[1] + 2.0 * a[2] * t + 3.0 * a[3] * t * t + 4.0 * a[4] * std::pow(t, 3) + 5.0 * a[5] * std::pow(t, 4);
  const double acceleration = 2.0 * a[2] + 6.0 * a[3] * t + 12.0 * a[4] * t * t + 20.0 * a[5] * std::pow(t, 3);
  return {p, v, acceleration};
}

/// 校验当前位姿后锁存为轨迹的初始命令点。
void TrajectoryPlanner::latch(double x_m, double y_m, double z_m, const common::Quaternion & orientation)
{
  if (!common::finite(x_m) || !common::finite(y_m) || !common::finite(z_m)) {
    throw std::invalid_argument("local XYZ pose must be finite before latching");
  }
  orientation_ = common::normalize_quaternion(orientation.x, orientation.y, orientation.z, orientation.w);
  x_m_ = x_m; y_m_ = y_m; target_x_m_ = x_m; target_y_m_ = y_m;
  xy_velocity_x_m_s_ = 0.0; xy_velocity_y_m_s_ = 0.0; xy_trajectory_elapsed_s_ = 0.0; xy_trajectory_duration_s_ = 0.0;
  xy_coefficients_ = {{{x_m, 0, 0, 0, 0, 0}, {y_m, 0, 0, 0, 0, 0}}};
  command_z_m_ = z_m; target_z_m_ = z_m; vertical_rate_m_s_ = 0.0;
  trajectory_elapsed_s_ = 0.0; trajectory_duration_s_ = 0.0; coefficients_ = {z_m, 0, 0, 0, 0, 0};
  latched_ = true; flow_effective_ = false;
}

/// 从当前输出姿态返回偏航角。
double TrajectoryPlanner::yaw_rad() const {return common::yaw_from_quaternion(orientation_);}

/// 校验并开始一段受限的水平移动。
void TrajectoryPlanner::set_xy_target(double x_m, double y_m)
{
  require_latched("setting an XY target");
  if (!common::finite(x_m) || !common::finite(y_m)) {throw std::invalid_argument("XY target must be finite");}
  begin_xy_trajectory(
    x_m, y_m, std::nullopt, config_.target_xy_max_speed_m_s, config_.target_xy_max_accel_m_s2);
}

/// 使用调用方提供的速度上限和默认安全加速度约束规划 XY 轨迹。
void TrajectoryPlanner::set_xy_target_with_max_speed(double x_m, double y_m, double max_speed_m_s)
{
  require_latched("setting an XY target with a speed limit");
  if (!common::finite(x_m) || !common::finite(y_m) || !common::finite(max_speed_m_s) ||
    max_speed_m_s <= 0.0) {
    throw std::invalid_argument("XY target and speed limit must be finite and positive");
  }
  begin_xy_trajectory(
    x_m, y_m, std::nullopt, max_speed_m_s, config_.target_xy_max_accel_m_s2);
}

/// 在不突破既有速度和加速度约束的前提下尝试按指定时长到达 XY 目标。
bool TrajectoryPlanner::set_xy_target_with_arrival_time(
  double x_m, double y_m, double arrival_seconds)
{
  require_latched("setting a timed XY target");
  if (!common::finite(x_m) || !common::finite(y_m) || !common::finite(arrival_seconds) ||
    arrival_seconds <= 0.0) {
    throw std::invalid_argument("timed XY target and arrival time must be finite and positive");
  }
  return begin_xy_trajectory(
    x_m, y_m, arrival_seconds, config_.target_xy_max_speed_m_s, config_.target_xy_max_accel_m_s2);
}

/// 立即把水平控制冻结在最新可靠的实测位置。
void TrajectoryPlanner::freeze_xy_at(double x_m, double y_m)
{
  require_latched("freezing XY");
  if (!common::finite(x_m) || !common::finite(y_m)) {throw std::invalid_argument("frozen XY pose must be finite");}
  x_m_ = x_m; y_m_ = y_m; target_x_m_ = x_m; target_y_m_ = y_m; xy_velocity_x_m_s_ = 0.0; xy_velocity_y_m_s_ = 0.0;
  xy_trajectory_elapsed_s_ = 0.0; xy_trajectory_duration_s_ = 0.0; xy_coefficients_ = {{{x_m, 0, 0, 0, 0, 0}, {y_m, 0, 0, 0, 0, 0}}};
}

/// 在当前规划位置建立零速度水平保持。
void TrajectoryPlanner::hold_xy() {require_latched("holding XY"); freeze_xy_at(x_m_, y_m_);}

/// 校验并开始一段受限的绝对高度轨迹。
void TrajectoryPlanner::set_z_target(double z_m)
{
  require_latched("setting a Z target");
  if (!common::finite(z_m)) {throw std::invalid_argument("Z target must be finite");}
  begin_z_trajectory(z_m);
}

/// 从当前规划位置和速度连续重规划新的绝对 XYZ 地面站目标。
void TrajectoryPlanner::set_target(double x_m, double y_m, double z_m)
{
  set_xy_target(x_m, y_m); set_z_target(z_m);
}

/// 停止垂直轨迹并将当前命令高度作为目标。
void TrajectoryPlanner::freeze_z()
{
  require_latched("freezing Z");
  target_z_m_ = command_z_m_; vertical_rate_m_s_ = 0.0; trajectory_elapsed_s_ = 0.0; trajectory_duration_s_ = 0.0;
  coefficients_ = {command_z_m_, 0, 0, 0, 0, 0};
}

/// 立即将垂直保持点对齐到可靠的实测高度。
void TrajectoryPlanner::freeze_z_at(double z_m)
{
  require_latched("freezing Z at a measurement");
  if (!common::finite(z_m)) {throw std::invalid_argument("frozen Z must be finite");}
  command_z_m_ = z_m; target_z_m_ = z_m; vertical_rate_m_s_ = 0.0;
  trajectory_elapsed_s_ = 0.0; trajectory_duration_s_ = 0.0; coefficients_ = {z_m, 0, 0, 0, 0, 0};
}

/// 从给定偏航构造纯 Z 轴旋转四元数。
void TrajectoryPlanner::set_yaw_rad(double yaw)
{
  if (!common::finite(yaw)) {throw std::invalid_argument("yaw target must be finite");}
  orientation_ = {0.0, 0.0, std::sin(0.5 * yaw), std::cos(0.5 * yaw)};
}

/// 迭代拉长五次 Z 轨迹，直到速度与加速度峰值均满足配置。
void TrajectoryPlanner::begin_z_trajectory(double target)
{
  target_z_m_ = target;
  const double distance = std::abs(target - command_z_m_);
  if (distance < 1e-9 && std::abs(vertical_rate_m_s_) < 1e-9) {
    command_z_m_ = target; vertical_rate_m_s_ = 0.0; trajectory_elapsed_s_ = 0.0; trajectory_duration_s_ = 0.0; coefficients_ = {target, 0, 0, 0, 0, 0}; return;
  }
  double duration = std::max({0.5, 2.0 * distance / config_.max_z_setpoint_rate_m_s,
    std::sqrt(6.0 * distance / config_.max_z_setpoint_accel_m_s2), 2.0 * std::abs(vertical_rate_m_s_) / config_.max_z_setpoint_accel_m_s2});
  auto candidate = quintic_coefficients(command_z_m_, vertical_rate_m_s_, target, duration);
  for (int iteration = 0; iteration < 12; ++iteration) {
    double peak_rate = 0.0; double peak_acceleration = 0.0;
    for (int index = 0; index <= 200; ++index) {
      const auto state = evaluate(candidate, duration * static_cast<double>(index) / 200.0);
      peak_rate = std::max(peak_rate, std::abs(state[1])); peak_acceleration = std::max(peak_acceleration, std::abs(state[2]));
    }
    const double scale = std::max({1.0, peak_rate / config_.max_z_setpoint_rate_m_s, std::sqrt(peak_acceleration / config_.max_z_setpoint_accel_m_s2)});
    if (scale <= 1.000001) {break;}
    duration *= scale * 1.01; candidate = quintic_coefficients(command_z_m_, vertical_rate_m_s_, target, duration);
  }
  trajectory_elapsed_s_ = 0.0; trajectory_duration_s_ = duration; coefficients_ = candidate;
}

/// 迭代拉长二维五次轨迹，直到合成速度与加速度满足配置。
bool TrajectoryPlanner::begin_xy_trajectory(
  double target_x, double target_y, const std::optional<double> & arrival_seconds,
  double max_speed_m_s, double max_accel_m_s2)
{
  const double distance = std::hypot(target_x - x_m_, target_y - y_m_);
  const double speed = std::hypot(xy_velocity_x_m_s_, xy_velocity_y_m_s_);
  target_x_m_ = target_x; target_y_m_ = target_y;
  if (distance < 1e-9 && speed < 1e-9) {
    x_m_ = target_x; y_m_ = target_y; xy_velocity_x_m_s_ = 0.0; xy_velocity_y_m_s_ = 0.0; xy_trajectory_elapsed_s_ = 0.0; xy_trajectory_duration_s_ = 0.0;
    xy_coefficients_ = {{{target_x, 0, 0, 0, 0, 0}, {target_y, 0, 0, 0, 0, 0}}};
    xy_arrival_time_met_ = true;
    return true;
  }
  double duration = arrival_seconds.value_or(std::max({0.5,
    2.0 * distance / max_speed_m_s,
    std::sqrt(6.0 * distance / max_accel_m_s2),
    2.0 * speed / max_accel_m_s2}));
  auto candidate = std::array<Coefficients, 2>{quintic_coefficients(x_m_, xy_velocity_x_m_s_, target_x, duration), quintic_coefficients(y_m_, xy_velocity_y_m_s_, target_y, duration)};
  for (int iteration = 0; iteration < 12; ++iteration) {
    double peak_rate = 0.0; double peak_acceleration = 0.0;
    for (int index = 0; index <= 200; ++index) {
      const auto x = evaluate(candidate[0], duration * static_cast<double>(index) / 200.0);
      const auto y = evaluate(candidate[1], duration * static_cast<double>(index) / 200.0);
      peak_rate = std::max(peak_rate, std::hypot(x[1], y[1])); peak_acceleration = std::max(peak_acceleration, std::hypot(x[2], y[2]));
    }
    const double scale = std::max({
      1.0, peak_rate / max_speed_m_s, std::sqrt(peak_acceleration / max_accel_m_s2)});
    if (scale <= 1.000001) {break;}
    duration *= scale * 1.01;
    candidate = {quintic_coefficients(x_m_, xy_velocity_x_m_s_, target_x, duration), quintic_coefficients(y_m_, xy_velocity_y_m_s_, target_y, duration)};
  }
  xy_trajectory_elapsed_s_ = 0.0;
  xy_trajectory_duration_s_ = duration;
  xy_coefficients_ = candidate;
  xy_arrival_time_met_ = !arrival_seconds || duration <= *arrival_seconds * 1.000001;
  return xy_arrival_time_met_;
}

/// 按有限正 dt 推进所有活动轨迹并钳制异常长控制周期。
common::PositionSetpoint TrajectoryPlanner::update(double dt_s)
{
  require_latched("updating the planner");
  if (!common::finite(dt_s) || dt_s <= 0.0) {throw std::invalid_argument("planner dt must be finite and positive");}
  dt_s = std::min(dt_s, 0.25);
  if (xy_trajectory_duration_s_ > 0.0) {
    xy_trajectory_elapsed_s_ = std::min(xy_trajectory_elapsed_s_ + dt_s, xy_trajectory_duration_s_);
    const auto x = evaluate(xy_coefficients_[0], xy_trajectory_elapsed_s_); const auto y = evaluate(xy_coefficients_[1], xy_trajectory_elapsed_s_);
    x_m_ = x[0]; y_m_ = y[0]; xy_velocity_x_m_s_ = x[1]; xy_velocity_y_m_s_ = y[1];
    if (xy_trajectory_elapsed_s_ >= xy_trajectory_duration_s_) {x_m_ = target_x_m_; y_m_ = target_y_m_; xy_velocity_x_m_s_ = 0.0; xy_velocity_y_m_s_ = 0.0; xy_trajectory_duration_s_ = 0.0;}
  }
  if (trajectory_duration_s_ > 0.0) {
    trajectory_elapsed_s_ = std::min(trajectory_elapsed_s_ + dt_s, trajectory_duration_s_);
    const auto state = evaluate(coefficients_, trajectory_elapsed_s_); command_z_m_ = state[0]; vertical_rate_m_s_ = state[1];
    if (trajectory_elapsed_s_ >= trajectory_duration_s_) {command_z_m_ = target_z_m_; vertical_rate_m_s_ = 0.0; trajectory_duration_s_ = 0.0;}
  }
  return current();
}

/// 封装当前内部规划状态为跨层传递的 PositionSetpoint。
common::PositionSetpoint TrajectoryPlanner::current() const
{
  require_latched("reading the planner");
  return {x_m_, y_m_, command_z_m_, orientation_, vertical_rate_m_s_};
}

namespace
{

constexpr double kInnovationGate99Percent2d = 9.210340371976184;
constexpr double kSmall = 1e-9;

double normalized_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double nearest_cardinal(double angle)
{
  const double half_pi = std::acos(-1.0) / 2.0;
  return normalized_angle(std::round(angle / half_pi) * half_pi);
}

}  // namespace

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
  const double nis = (innovation_x * innovation_x * s11 - 2.0 * innovation_x * innovation_y * s01 +
    innovation_y * innovation_y * s00) / determinant;
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

Navigation::Navigation(const common::SafetyConfig & config, MissionConfig mission)
: config_(config), mission_(std::move(mission)), planner_(config), target_tracker_(mission_)
{
  mission_.validate();
}

void Navigation::reset()
{
  planner_.reset();
  target_tracker_.reset();
  phase_ = "waiting_preflight";
  phase_started_at_ = 0.0;
  flight_started_at_.reset();
  normal_completion_ = false;
  control_ = {};
  pending_messages_.clear();
  pending_rejections_.clear();
  landing_reason_.clear();
  pending_release_gripper_ = false;
  last_car_status_at_.reset();
  intercept_due_at_.reset();
  cardinal_alignment_achieved_ = false;
  last_own_pose_at_.reset();
  last_own_x_m_.reset();
  last_own_y_m_.reset();
  own_vx_m_s_ = 0.0;
  own_vy_m_s_ = 0.0;
}

void Navigation::transition(const std::string & phase, double now)
{
  phase_ = phase;
  phase_started_at_ = now;
}

void Navigation::emit(communication::MessageType type) {pending_messages_.push_back({type});}

void Navigation::reject(const std::string & reason) {pending_rejections_.push_back(reason);}

bool Navigation::actual_xy_within(
  const common::Telemetry & telemetry, double x, double y, double tolerance) const
{
  return common::finite(telemetry.local_x_m) && common::finite(telemetry.local_y_m) &&
         std::hypot(telemetry.local_x_m - x, telemetry.local_y_m - y) <= tolerance;
}

bool Navigation::stable_at(
  const common::Telemetry & telemetry, const common::PositionSetpoint & target) const
{
  return actual_xy_within(telemetry, target.x_m, target.y_m, config_.target_tolerance_m) &&
         common::finite(telemetry.local_z_m) &&
         std::abs(telemetry.local_z_m - target.z_m) <= config_.target_tolerance_m;
}

bool Navigation::actual_yaw_within(
  const common::Telemetry & telemetry, double yaw_rad, double tolerance_rad) const
{
  try {
    const double actual = common::yaw_from_quaternion(common::normalize_quaternion(
      telemetry.orientation.x, telemetry.orientation.y, telemetry.orientation.z, telemetry.orientation.w));
    return std::abs(normalized_angle(actual - yaw_rad)) <= tolerance_rad;
  } catch (const std::invalid_argument &) {
    return false;
  }
}

void Navigation::set_mission_goal(const common::PositionSetpoint & goal)
{
  if (!common::finite(goal.x_m) || !common::finite(goal.y_m) || !common::finite(goal.z_m)) {
    throw std::invalid_argument("mission goal XYZ must be finite");
  }
  auto normalized = goal;
  normalized.orientation = common::normalize_quaternion(
    goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w);
  normalized.vertical_rate_m_s = 0.0;
  control_.mission_goal = normalized;
}

void Navigation::plan_to_mission_goal()
{
  if (!planner_.latched() || !control_.mission_goal) {return;}
  const auto & goal = *control_.mission_goal;
  planner_.set_target(goal.x_m, goal.y_m, goal.z_m);
  planner_.set_yaw_rad(common::yaw_from_quaternion(goal.orientation));
}

common::PositionSetpoint Navigation::measured_hold_setpoint(const NavigationInput & input) const
{
  common::PositionSetpoint hold = control_.commanded_setpoint.value_or(planner_.current());
  if (common::finite(input.telemetry.local_x_m)) {hold.x_m = input.telemetry.local_x_m;}
  if (common::finite(input.telemetry.local_y_m)) {hold.y_m = input.telemetry.local_y_m;}
  if (common::finite(input.telemetry.local_z_m)) {hold.z_m = input.telemetry.local_z_m;}
  hold.vertical_rate_m_s = 0.0;
  return hold;
}

void Navigation::enter_hold(
  const NavigationInput & input, const std::string & hold_phase,
  const std::string & reason, const std::optional<std::string> & resume_phase)
{
  if (!planner_.latched() || phase_ == hold_phase) {return;}
  const auto hold = measured_hold_setpoint(input);
  planner_.freeze_xy_at(hold.x_m, hold.y_m);
  planner_.freeze_z_at(hold.z_m);
  planner_.set_yaw_rad(common::yaw_from_quaternion(hold.orientation));
  control_.hold_setpoint = hold;
  control_.hold_reason = reason;
  control_.hold_resume_phase = resume_phase.value_or(phase_);
  control_.mission_paused = true;
  transition(hold_phase, input.now);
}

void Navigation::clear_hold()
{
  control_.hold_setpoint.reset();
  control_.hold_reason.clear();
  control_.hold_resume_phase.clear();
  control_.mission_paused = false;
}

void Navigation::resume_hold(double now)
{
  const std::string resume = control_.hold_resume_phase.empty() ? "waiting_target" : control_.hold_resume_phase;
  plan_to_mission_goal();
  clear_hold();
  transition(resume, now);
}

void Navigation::begin_landing(double now, const std::string & reason)
{
  intercept_due_at_.reset();
  if (planner_.latched()) {
    clear_hold();
    planner_.hold_xy();
    if (control_.origin) {planner_.set_z_target(control_.origin->z_m);}
  }
  landing_reason_ = reason;
  normal_completion_ = false;
  transition("landing", now);
}

void Navigation::begin_transit_to_b()
{
  if (!control_.origin || !control_.mission_goal) {
    throw std::logic_error("B-point transit requires a latched origin and climb goal");
  }
  const double psi0 = common::yaw_from_quaternion(control_.origin->orientation);
  auto b_goal = *control_.mission_goal;
  b_goal.x_m = control_.origin->x_m + mission_.b_forward_m * std::cos(psi0) +
    mission_.b_right_m * std::sin(psi0);
  b_goal.y_m = control_.origin->y_m + mission_.b_forward_m * std::sin(psi0) -
    mission_.b_right_m * std::cos(psi0);
  b_goal.orientation = control_.origin->orientation;
  set_mission_goal(b_goal);
  plan_to_mission_goal();
}

void Navigation::update_own_velocity(const NavigationInput & input)
{
  if (common::finite(input.telemetry.velocity_x_m_s) && common::finite(input.telemetry.velocity_y_m_s)) {
    own_vx_m_s_ = input.telemetry.velocity_x_m_s;
    own_vy_m_s_ = input.telemetry.velocity_y_m_s;
  } else if (last_own_pose_at_ && last_own_x_m_ && last_own_y_m_ &&
    input.now > *last_own_pose_at_ && common::finite(input.telemetry.local_x_m) &&
    common::finite(input.telemetry.local_y_m)) {
    const double dt = input.now - *last_own_pose_at_;
    own_vx_m_s_ = (input.telemetry.local_x_m - *last_own_x_m_) / dt;
    own_vy_m_s_ = (input.telemetry.local_y_m - *last_own_y_m_) / dt;
  }
  if (common::finite(input.telemetry.local_x_m) && common::finite(input.telemetry.local_y_m)) {
    last_own_pose_at_ = input.now;
    last_own_x_m_ = input.telemetry.local_x_m;
    last_own_y_m_ = input.telemetry.local_y_m;
  }
}

bool Navigation::apply_car_status(const NavigationInput & input, const communication::CarStatus & status)
{
  if (!control_.origin || !common::finite(input.telemetry.local_x_m) ||
    !common::finite(input.telemetry.local_y_m)) {
    reject("car_status_requires_local_pose");
    return false;
  }
  double yaw = 0.0;
  try {
    yaw = common::yaw_from_quaternion(common::normalize_quaternion(
      input.telemetry.orientation.x, input.telemetry.orientation.y,
      input.telemetry.orientation.z, input.telemetry.orientation.w));
  } catch (const std::invalid_argument &) {
    reject("car_status_requires_valid_yaw");
    return false;
  }
  const double world_bearing = yaw + status.bearing_rad;
  const double target_x = input.telemetry.local_x_m + status.distance_m * std::cos(world_bearing);
  const double target_y = input.telemetry.local_y_m + status.distance_m * std::sin(world_bearing);
  if (std::hypot(target_x - control_.origin->x_m, target_y - control_.origin->y_m) >
    mission_.max_tracking_radius_m) {
    reject("car_status_target_outside_tracking_radius");
    target_tracker_.reset();
    control_.target_samples = 0;
    last_car_status_at_.reset();
    intercept_due_at_.reset();
    enter_hold(input, "lcp_hold", "car_status_target_outside_tracking_radius", "waiting_target");
    return false;
  }
  if (!target_tracker_.update(target_x, target_y, input.now)) {
    reject("car_status_innovation_outlier");
    target_tracker_.reset();
    control_.target_samples = 0;
    last_car_status_at_.reset();
    intercept_due_at_.reset();
    enter_hold(input, "lcp_hold", "car_status_innovation_outlier", "waiting_target");
    return false;
  }
  last_car_status_at_ = input.now;
  control_.target_samples = target_tracker_.samples();
  clear_hold();
  if (target_tracker_.ready()) {plan_intercept(input, status.bearing_rad);}
  return true;
}

bool Navigation::car_status_fresh(double now) const
{
  return last_car_status_at_ && now >= *last_car_status_at_ &&
         now - *last_car_status_at_ <= mission_.car_status_timeout_s;
}

void Navigation::plan_intercept(const NavigationInput & input, double bearing_rad)
{
  if (!target_tracker_.ready() || !control_.origin || !common::finite(input.telemetry.local_x_m) ||
    !common::finite(input.telemetry.local_y_m)) {
    return;
  }
  const auto remaining = target_tracker_.time_to_distance(
    input.telemetry.local_x_m, input.telemetry.local_y_m, own_vx_m_s_, own_vy_m_s_,
    mission_.throw_distance_m, input.now, mission_.prediction_horizon_s);
  if (!remaining) {
    intercept_due_at_.reset();
    control_.predicted_intercept_seconds.reset();
    enter_hold(input, "lcp_hold", "intercept_prediction_outside_window", "waiting_target");
    return;
  }
  const auto target = target_tracker_.estimate(input.now + *remaining);
  const double initial_yaw = common::yaw_from_quaternion(control_.origin->orientation);
  auto goal = control_.mission_goal.value_or(*control_.origin);
  goal.z_m = control_.origin->z_m + mission_.takeoff_height_m;
  goal.orientation = control_.origin->orientation;
  control_.predicted_intercept_seconds = *remaining;
  intercept_due_at_ = input.now + *remaining;
  if (*remaining <= mission_.final_intercept_seconds) {
    goal.x_m = target.x_m;
    goal.y_m = target.y_m;
    set_mission_goal(goal);
    if (*remaining <= 0.01) {
      planner_.set_xy_target(goal.x_m, goal.y_m);
      control_.tracking_arrival_time_met = true;
    } else {
      control_.tracking_arrival_time_met = planner_.set_xy_target_with_arrival_time(
        goal.x_m, goal.y_m, *remaining);
    }
    planner_.set_z_target(goal.z_m);
    planner_.set_yaw_rad(initial_yaw);
    cardinal_alignment_achieved_ = true;
    transition("final_intercept", input.now);
    return;
  }

  const double cardinal_relative = nearest_cardinal(bearing_rad);
  cardinal_alignment_achieved_ = std::abs(normalized_angle(bearing_rad - cardinal_relative)) <=
    mission_.cardinal_tolerance_deg * std::acos(-1.0) / 180.0;
  if (cardinal_alignment_achieved_) {
    goal.x_m = target.x_m;
    goal.y_m = target.y_m;
  } else {
    const double range = std::hypot(target.x_m - input.telemetry.local_x_m,
      target.y_m - input.telemetry.local_y_m);
    const double world_cardinal = initial_yaw + cardinal_relative;
    goal.x_m = input.telemetry.local_x_m + range * std::cos(world_cardinal);
    goal.y_m = input.telemetry.local_y_m + range * std::sin(world_cardinal);
  }
  set_mission_goal(goal);
  control_.tracking_arrival_time_met = planner_.set_xy_target_with_arrival_time(
    goal.x_m, goal.y_m, *remaining);
  planner_.set_z_target(goal.z_m);
  planner_.set_yaw_rad(initial_yaw);
  if (!control_.tracking_arrival_time_met) {
    intercept_due_at_.reset();
    enter_hold(input, "lcp_hold", "intercept_unreachable_with_constraints", "waiting_target");
    return;
  }
  transition("cardinal_alignment", input.now);
}

void Navigation::begin_return(double now)
{
  if (!control_.origin) {
    begin_landing(now, "return_without_origin");
    return;
  }
  intercept_due_at_.reset();
  target_tracker_.reset();
  control_.target_samples = 0;
  control_.predicted_intercept_seconds.reset();
  clear_hold();
  auto return_goal = *control_.origin;
  return_goal.z_m = planner_.latched() ? planner_.current().z_m : return_goal.z_m;
  return_goal.orientation = {0.0, 0.0, 0.0, 1.0};
  set_mission_goal(return_goal);
  plan_to_mission_goal();
  transition("returning", now);
}

bool Navigation::lcp_required_in_phase() const
{
  return phase_ == "height_stabilizing" || phase_ == "transit_to_b" ||
    phase_ == "waiting_target" || phase_ == "cardinal_alignment" ||
    phase_ == "final_intercept" || phase_ == "throwing" || phase_ == "returning" ||
    phase_ == "downing";
}

void Navigation::process_events(const NavigationInput & input)
{
  for (const auto & event : input.events) {
    if (!event.accepted) {
      if (!event.rejection_reason.empty()) {reject(event.rejection_reason);}
      continue;
    }
    if (event.type == communication::MessageType::ack) {continue;}
    if (event.type == communication::MessageType::run_plan1) {
      if (phase_ == "waiting_run_plan1") {
        transition("offboard_request_pending", input.now);
      } else if (phase_ != "offboard_request_pending" && phase_ != "arming_request_pending" &&
        phase_ != "climb" && phase_ != "height_stabilizing") {
        reject("run_plan1_not_allowed_in_phase");
      }
      continue;
    }
    if (event.type == communication::MessageType::car_status) {
      if (!event.car_status) {
        reject("car_status_missing_measurement");
      } else if (phase_ == "waiting_target" || phase_ == "cardinal_alignment" ||
        phase_ == "final_intercept") {
        apply_car_status(input, *event.car_status);
      } else if (phase_ == "lcp_hold" && control_.hold_reason != "lcp_unhealthy") {
        const auto resume = control_.hold_resume_phase;
        clear_hold();
        transition(resume.empty() ? "waiting_target" : resume, input.now);
        apply_car_status(input, *event.car_status);
      } else {
        reject("car_status_not_allowed_in_phase");
      }
      continue;
    }
    reject("message_not_allowed_in_phase");
  }
}

NavigationDecision Navigation::update(const NavigationInput & input)
{
  if (!common::finite(input.now) || !common::finite(input.dt) || input.dt <= 0.0) {
    throw std::invalid_argument("navigation time and dt must be finite, dt must be positive");
  }
  pending_messages_.clear();
  pending_rejections_.clear();
  pending_release_gripper_ = false;
  update_own_velocity(input);
  process_events(input);

  if (phase_ == "waiting_preflight") {
    if (input.preflight_ready && input.lcp_healthy && common::finite(input.telemetry.local_x_m) &&
      common::finite(input.telemetry.local_y_m) && common::finite(input.telemetry.local_z_m)) {
      transition("setpoint_warmup", input.now);
    }
  } else if (phase_ == "setpoint_warmup") {
    if (input.now - phase_started_at_ >= config_.setpoint_warmup_s) {
      emit(communication::MessageType::ok_wait);
      transition("waiting_run_plan1", input.now);
    }
  } else if (phase_ == "offboard_request_pending") {
    if (input.controller.mode == "OFFBOARD") {transition("arming_request_pending", input.now);}
  } else if (phase_ == "arming_request_pending") {
    if (!input.controller.armed) {
      if (input.controller.mode != "OFFBOARD") {transition("offboard_request_pending", input.now);}
    } else if (input.controller.mode != "OFFBOARD") {
      begin_landing(input.now, "offboard_mode_lost");
    } else if (!input.flight_healthy) {
      begin_landing(input.now, input.health_errors.empty() ? "flight_health_failure" : input.health_errors.front());
    } else {
      planner_.latch(input.telemetry.local_x_m, input.telemetry.local_y_m,
        input.telemetry.local_z_m, input.telemetry.orientation);
      control_.origin = planner_.current();
      auto climb_goal = *control_.origin;
      climb_goal.z_m += mission_.takeoff_height_m;
      set_mission_goal(climb_goal);
      plan_to_mission_goal();
      flight_started_at_ = input.now;
      transition("climb", input.now);
    }
  }

  if (lcp_required_in_phase() && !input.lcp_healthy && phase_ != "lcp_hold") {
    enter_hold(input, "lcp_hold", "lcp_unhealthy");
  }

  if (phase_ == "climb") {
    if (planner_.target_reached() && stable_at(input.telemetry, planner_.current())) {
      transition("height_stabilizing", input.now);
    }
  } else if (phase_ == "height_stabilizing") {
    if (!planner_.target_reached() || !stable_at(input.telemetry, planner_.current())) {
      phase_started_at_ = input.now;
    } else if (input.now - phase_started_at_ >= mission_.height_stable_seconds) {
      begin_transit_to_b();
      transition("transit_to_b", input.now);
    }
  } else if (phase_ == "transit_to_b") {
    if (planner_.target_reached() && stable_at(input.telemetry, planner_.current())) {
      emit(communication::MessageType::ok_b);
      transition("waiting_target", input.now);
    }
  } else if (phase_ == "waiting_target" || phase_ == "cardinal_alignment" ||
    phase_ == "final_intercept") {
    if (last_car_status_at_ && !car_status_fresh(input.now)) {
      target_tracker_.reset();
      control_.target_samples = 0;
      intercept_due_at_.reset();
      control_.predicted_intercept_seconds.reset();
      enter_hold(input, "lcp_hold", "car_status_timeout", "waiting_target");
    } else if (phase_ == "final_intercept" && intercept_due_at_ && input.now >= *intercept_due_at_) {
      const auto estimate = target_tracker_.estimate(input.now);
      if (std::hypot(estimate.x_m - input.telemetry.local_x_m, estimate.y_m - input.telemetry.local_y_m) <=
        mission_.throw_distance_m) {
        pending_release_gripper_ = true;
        transition("throwing", input.now);
      } else {
        intercept_due_at_.reset();
        plan_intercept(input, 0.0);
      }
    }
  } else if (phase_ == "throwing") {
    if (input.gripper_failed) {
      reject("gripper_release_failed");
      begin_return(input.now);
    } else if (input.gripper_succeeded) {
      emit(communication::MessageType::ok_throw);
      begin_return(input.now);
    }
  } else if (phase_ == "returning") {
    if (control_.origin && planner_.target_reached() && actual_xy_within(
        input.telemetry, control_.origin->x_m, control_.origin->y_m, config_.target_tolerance_m) &&
      actual_yaw_within(input.telemetry, 0.0, 0.10)) {
      emit(communication::MessageType::ok_return);
      auto descent_goal = *control_.origin;
      descent_goal.orientation = {0.0, 0.0, 0.0, 1.0};
      set_mission_goal(descent_goal);
      plan_to_mission_goal();
      normal_completion_ = true;
      emit(communication::MessageType::ok_downing);
      transition("downing", input.now);
    }
  } else if (phase_ == "lcp_hold") {
    if (input.lcp_healthy && control_.hold_reason == "lcp_unhealthy") {resume_hold(input.now);}
  } else if (phase_ == "downing" || phase_ == "landing") {
    const bool on_ground = input.telemetry.landed_state == common::MAV_LANDED_STATE_ON_GROUND;
    const bool at_origin = common::finite(input.telemetry.local_z_m) && control_.origin &&
      std::abs(input.telemetry.local_z_m - control_.origin->z_m) <= config_.touchdown_z_tolerance_m;
    if (on_ground || (planner_.target_reached() && at_origin)) {transition("disarming", input.now);}
  } else if (phase_ == "disarming") {
    if (!input.controller.armed) {transition("manual_request_pending", input.now);}
  } else if (phase_ == "manual_request_pending" &&
    input.controller.mode == "MANUAL" && !input.controller.armed) {
    if (normal_completion_) {emit(communication::MessageType::ok_down);}
    transition("manual", input.now);
  }

  const bool waiting_gcs_phase = phase_ == "waiting_run_plan1" || phase_ == "setpoint_warmup";
  if (waiting_gcs_phase && !input.preflight_ready) {reset();}
  const bool post_run_prearm_phase = phase_ == "offboard_request_pending" || phase_ == "arming_request_pending";
  if (post_run_prearm_phase && !input.controller.armed && !input.preflight_ready) {
    reset();
    transition("manual_request_pending", input.now);
    reject("preflight_lost_before_arm");
  }

  const bool airborne = phase_ != "waiting_preflight" && phase_ != "waiting_run_plan1" &&
    phase_ != "setpoint_warmup" && phase_ != "offboard_request_pending" &&
    phase_ != "arming_request_pending" && phase_ != "manual";
  if (airborne && phase_ != "landing" && phase_ != "downing" && phase_ != "disarming" &&
    phase_ != "manual_request_pending" && !input.flight_healthy) {
    begin_landing(input.now, input.health_errors.empty() ? "flight_health_failure" : input.health_errors.front());
  }
  if (airborne && phase_ != "landing" && phase_ != "downing" && phase_ != "disarming" &&
    phase_ != "manual_request_pending" && input.controller.mode != "OFFBOARD") {
    begin_landing(input.now, "offboard_mode_lost");
  }
  if (flight_started_at_ && airborne && phase_ != "landing" && phase_ != "downing" &&
    input.now - *flight_started_at_ > config_.max_flight_seconds) {
    begin_landing(input.now, "maximum_flight_time_exceeded");
  }

  NavigationDecision decision;
  decision.phase = phase_;
  decision.messages = pending_messages_;
  decision.rejections = pending_rejections_;
  decision.release_gripper = pending_release_gripper_;
  const bool ground_hold_stream = phase_ == "setpoint_warmup" || phase_ == "waiting_run_plan1" ||
    phase_ == "offboard_request_pending" || phase_ == "arming_request_pending";
  if (ground_hold_stream && common::finite(input.telemetry.local_x_m) &&
    common::finite(input.telemetry.local_y_m) && common::finite(input.telemetry.local_z_m)) {
    common::PositionSetpoint ground_hold{
      input.telemetry.local_x_m, input.telemetry.local_y_m, input.telemetry.local_z_m,
      common::normalize_quaternion(input.telemetry.orientation.x, input.telemetry.orientation.y,
        input.telemetry.orientation.z, input.telemetry.orientation.w), 0.0};
    decision.setpoint = ground_hold;
    control_.commanded_setpoint = ground_hold;
  } else if (planner_.latched() && phase_ != "waiting_preflight" && phase_ != "manual") {
    decision.setpoint = planner_.update(input.dt);
    control_.commanded_setpoint = decision.setpoint;
  }
  if (phase_ == "offboard_request_pending" || phase_ == "arming_request_pending" || phase_ == "climb" ||
    phase_ == "height_stabilizing" || phase_ == "transit_to_b" || phase_ == "waiting_target" ||
    phase_ == "cardinal_alignment" || phase_ == "final_intercept" || phase_ == "throwing" ||
    phase_ == "returning" || phase_ == "lcp_hold" || phase_ == "downing") {
    decision.target_mode = "OFFBOARD";
  }
  if (phase_ == "landing" && !landing_reason_.empty()) {decision.target_mode = "AUTO.LAND";}
  if (phase_ == "arming_request_pending" || phase_ == "climb" || phase_ == "height_stabilizing" ||
    phase_ == "transit_to_b" || phase_ == "waiting_target" || phase_ == "cardinal_alignment" ||
    phase_ == "final_intercept" || phase_ == "throwing" || phase_ == "returning" ||
    phase_ == "lcp_hold" || phase_ == "downing" || phase_ == "landing") {
    decision.arm_intent = true;
  }
  if (phase_ == "disarming" || phase_ == "manual_request_pending") {decision.arm_intent = false;}
  if (phase_ == "manual_request_pending" || phase_ == "manual") {decision.target_mode = "MANUAL";}
  decision.control = control_;
  return decision;
}

}  // mavros_xyz_position_offboard::navigation namespace
