#include "mavros_xyz_position_offboard/navigation/trajectory_planner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mavros_xyz_position_offboard::navigation
{

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
TrajectoryPlanner::Coefficients TrajectoryPlanner::quintic_coefficients(
  double start, double rate, double target, double duration_s)
{
  const double delta = target - start;
  const double t = duration_s;
  return {start, rate, 0.0,
    (20.0 * delta - 12.0 * rate * t) / (2.0 * std::pow(t, 3)),
    (-30.0 * delta + 16.0 * rate * t) / (2.0 * std::pow(t, 4)),
    (12.0 * delta - 6.0 * rate * t) / (2.0 * std::pow(t, 5))};
}

/// 求出五次曲线在 t 时刻的位置、速度和加速度。
std::array<double, 3> TrajectoryPlanner::evaluate(const Coefficients & a, double t)
{
  const double p = a[0] + a[1] * t + a[2] * t * t + a[3] * std::pow(t, 3) +
    a[4] * std::pow(t, 4) + a[5] * std::pow(t, 5);
  const double v = a[1] + 2.0 * a[2] * t + 3.0 * a[3] * t * t +
    4.0 * a[4] * std::pow(t, 3) + 5.0 * a[5] * std::pow(t, 4);
  const double acceleration = 2.0 * a[2] + 6.0 * a[3] * t +
    12.0 * a[4] * t * t + 20.0 * a[5] * std::pow(t, 3);
  return {p, v, acceleration};
}

/// 校验当前位姿后锁存为轨迹的初始命令点。
void TrajectoryPlanner::latch(
  double x_m, double y_m, double z_m, const common::Quaternion & orientation)
{
  if (!common::finite(x_m) || !common::finite(y_m) || !common::finite(z_m)) {
    throw std::invalid_argument("local XYZ pose must be finite before latching");
  }
  orientation_ = common::normalize_quaternion(orientation.x, orientation.y, orientation.z, orientation.w);
  x_m_ = x_m; y_m_ = y_m; target_x_m_ = x_m; target_y_m_ = y_m;
  xy_velocity_x_m_s_ = 0.0; xy_velocity_y_m_s_ = 0.0;
  xy_trajectory_elapsed_s_ = 0.0; xy_trajectory_duration_s_ = 0.0;
  xy_coefficients_ = {{{x_m, 0, 0, 0, 0, 0}, {y_m, 0, 0, 0, 0, 0}}};
  command_z_m_ = z_m; target_z_m_ = z_m; vertical_rate_m_s_ = 0.0;
  trajectory_elapsed_s_ = 0.0; trajectory_duration_s_ = 0.0;
  coefficients_ = {z_m, 0, 0, 0, 0, 0};
  latched_ = true; flow_effective_ = false;
}

/// 从当前输出姿态返回偏航角。
double TrajectoryPlanner::yaw_rad() const {return common::yaw_from_quaternion(orientation_);}

/// 校验并开始一段受限的水平移动。
void TrajectoryPlanner::set_xy_target(double x_m, double y_m)
{
  require_latched("setting an XY target");
  if (!common::finite(x_m) || !common::finite(y_m)) {
    throw std::invalid_argument("XY target must be finite");
  }
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

/// 使用调用方提供的二维合速度和合加速度约束规划 XY 轨迹。
void TrajectoryPlanner::set_xy_target_with_limits(
  double x_m, double y_m, double max_speed_m_s, double max_accel_m_s2)
{
  require_latched("setting an XY target with custom limits");
  if (!common::finite(x_m) || !common::finite(y_m) || !common::finite(max_speed_m_s) ||
    !common::finite(max_accel_m_s2) || max_speed_m_s <= 0.0 || max_accel_m_s2 <= 0.0) {
    throw std::invalid_argument("XY target and limits must be finite and positive");
  }
  begin_xy_trajectory(x_m, y_m, std::nullopt, max_speed_m_s, max_accel_m_s2);
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
  if (!common::finite(x_m) || !common::finite(y_m)) {
    throw std::invalid_argument("frozen XY pose must be finite");
  }
  x_m_ = x_m; y_m_ = y_m; target_x_m_ = x_m; target_y_m_ = y_m;
  xy_velocity_x_m_s_ = 0.0; xy_velocity_y_m_s_ = 0.0;
  xy_trajectory_elapsed_s_ = 0.0; xy_trajectory_duration_s_ = 0.0;
  xy_coefficients_ = {{{x_m, 0, 0, 0, 0, 0}, {y_m, 0, 0, 0, 0, 0}}};
}

/// 在当前规划位置建立零速度水平保持。
void TrajectoryPlanner::hold_xy()
{
  require_latched("holding XY");
  freeze_xy_at(x_m_, y_m_);
}

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
  set_xy_target(x_m, y_m);
  set_z_target(z_m);
}

/// 停止垂直轨迹并将当前命令高度作为目标。
void TrajectoryPlanner::freeze_z()
{
  require_latched("freezing Z");
  target_z_m_ = command_z_m_; vertical_rate_m_s_ = 0.0;
  trajectory_elapsed_s_ = 0.0; trajectory_duration_s_ = 0.0;
  coefficients_ = {command_z_m_, 0, 0, 0, 0, 0};
}

/// 立即将垂直保持点对齐到可靠的实测高度。
void TrajectoryPlanner::freeze_z_at(double z_m)
{
  require_latched("freezing Z at a measurement");
  if (!common::finite(z_m)) {throw std::invalid_argument("frozen Z must be finite");}
  command_z_m_ = z_m; target_z_m_ = z_m; vertical_rate_m_s_ = 0.0;
  trajectory_elapsed_s_ = 0.0; trajectory_duration_s_ = 0.0;
  coefficients_ = {z_m, 0, 0, 0, 0, 0};
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
    command_z_m_ = target; vertical_rate_m_s_ = 0.0;
    trajectory_elapsed_s_ = 0.0; trajectory_duration_s_ = 0.0;
    coefficients_ = {target, 0, 0, 0, 0, 0};
    return;
  }
  double duration = std::max({0.5, 2.0 * distance / config_.max_z_setpoint_rate_m_s,
    std::sqrt(6.0 * distance / config_.max_z_setpoint_accel_m_s2),
    2.0 * std::abs(vertical_rate_m_s_) / config_.max_z_setpoint_accel_m_s2});
  auto candidate = quintic_coefficients(command_z_m_, vertical_rate_m_s_, target, duration);
  for (int iteration = 0; iteration < 12; ++iteration) {
    double peak_rate = 0.0;
    double peak_acceleration = 0.0;
    for (int index = 0; index <= 200; ++index) {
      const auto state = evaluate(candidate, duration * static_cast<double>(index) / 200.0);
      peak_rate = std::max(peak_rate, std::abs(state[1]));
      peak_acceleration = std::max(peak_acceleration, std::abs(state[2]));
    }
    const double scale = std::max({
      1.0, peak_rate / config_.max_z_setpoint_rate_m_s,
      std::sqrt(peak_acceleration / config_.max_z_setpoint_accel_m_s2)});
    if (scale <= 1.000001) {break;}
    duration *= scale * 1.01;
    candidate = quintic_coefficients(command_z_m_, vertical_rate_m_s_, target, duration);
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
    x_m_ = target_x; y_m_ = target_y;
    xy_velocity_x_m_s_ = 0.0; xy_velocity_y_m_s_ = 0.0;
    xy_trajectory_elapsed_s_ = 0.0; xy_trajectory_duration_s_ = 0.0;
    xy_coefficients_ = {{{target_x, 0, 0, 0, 0, 0}, {target_y, 0, 0, 0, 0, 0}}};
    xy_arrival_time_met_ = true;
    return true;
  }
  double duration = arrival_seconds.value_or(std::max({0.5,
    2.0 * distance / max_speed_m_s,
    std::sqrt(6.0 * distance / max_accel_m_s2),
    2.0 * speed / max_accel_m_s2}));
  auto candidate = std::array<Coefficients, 2>{
    quintic_coefficients(x_m_, xy_velocity_x_m_s_, target_x, duration),
    quintic_coefficients(y_m_, xy_velocity_y_m_s_, target_y, duration)};
  for (int iteration = 0; iteration < 12; ++iteration) {
    double peak_rate = 0.0;
    double peak_acceleration = 0.0;
    for (int index = 0; index <= 200; ++index) {
      const auto x = evaluate(candidate[0], duration * static_cast<double>(index) / 200.0);
      const auto y = evaluate(candidate[1], duration * static_cast<double>(index) / 200.0);
      peak_rate = std::max(peak_rate, std::hypot(x[1], y[1]));
      peak_acceleration = std::max(peak_acceleration, std::hypot(x[2], y[2]));
    }
    const double scale = std::max({
      1.0, peak_rate / max_speed_m_s, std::sqrt(peak_acceleration / max_accel_m_s2)});
    if (scale <= 1.000001) {break;}
    duration *= scale * 1.01;
    candidate = {
      quintic_coefficients(x_m_, xy_velocity_x_m_s_, target_x, duration),
      quintic_coefficients(y_m_, xy_velocity_y_m_s_, target_y, duration)};
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
  if (!common::finite(dt_s) || dt_s <= 0.0) {
    throw std::invalid_argument("planner dt must be finite and positive");
  }
  dt_s = std::min(dt_s, 0.25);
  if (xy_trajectory_duration_s_ > 0.0) {
    xy_trajectory_elapsed_s_ = std::min(
      xy_trajectory_elapsed_s_ + dt_s, xy_trajectory_duration_s_);
    const auto x = evaluate(xy_coefficients_[0], xy_trajectory_elapsed_s_);
    const auto y = evaluate(xy_coefficients_[1], xy_trajectory_elapsed_s_);
    x_m_ = x[0]; y_m_ = y[0]; xy_velocity_x_m_s_ = x[1]; xy_velocity_y_m_s_ = y[1];
    if (xy_trajectory_elapsed_s_ >= xy_trajectory_duration_s_) {
      x_m_ = target_x_m_; y_m_ = target_y_m_;
      xy_velocity_x_m_s_ = 0.0; xy_velocity_y_m_s_ = 0.0;
      xy_trajectory_duration_s_ = 0.0;
    }
  }
  if (trajectory_duration_s_ > 0.0) {
    trajectory_elapsed_s_ = std::min(trajectory_elapsed_s_ + dt_s, trajectory_duration_s_);
    const auto state = evaluate(coefficients_, trajectory_elapsed_s_);
    command_z_m_ = state[0]; vertical_rate_m_s_ = state[1];
    if (trajectory_elapsed_s_ >= trajectory_duration_s_) {
      command_z_m_ = target_z_m_; vertical_rate_m_s_ = 0.0; trajectory_duration_s_ = 0.0;
    }
  }
  return current();
}

/// 封装当前内部规划状态为跨层传递的 PositionSetpoint。
common::PositionSetpoint TrajectoryPlanner::current() const
{
  require_latched("reading the planner");
  return {x_m_, y_m_, command_z_m_, orientation_, vertical_rate_m_s_};
}

}  // namespace mavros_xyz_position_offboard::navigation
