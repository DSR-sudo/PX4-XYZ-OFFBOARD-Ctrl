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
    (control.tracking_arrival_time_met ? "true" : "false") << '}';
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
  begin_xy_trajectory(x_m, y_m);
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
  return begin_xy_trajectory(x_m, y_m, arrival_seconds);
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
  double target_x, double target_y, const std::optional<double> & arrival_seconds)
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
    2.0 * distance / config_.target_xy_max_speed_m_s,
    std::sqrt(6.0 * distance / config_.target_xy_max_accel_m_s2),
    2.0 * speed / config_.target_xy_max_accel_m_s2}));
  auto candidate = std::array<Coefficients, 2>{quintic_coefficients(x_m_, xy_velocity_x_m_s_, target_x, duration), quintic_coefficients(y_m_, xy_velocity_y_m_s_, target_y, duration)};
  for (int iteration = 0; iteration < 12; ++iteration) {
    double peak_rate = 0.0; double peak_acceleration = 0.0;
    for (int index = 0; index <= 200; ++index) {
      const auto x = evaluate(candidate[0], duration * static_cast<double>(index) / 200.0);
      const auto y = evaluate(candidate[1], duration * static_cast<double>(index) / 200.0);
      peak_rate = std::max(peak_rate, std::hypot(x[1], y[1])); peak_acceleration = std::max(peak_acceleration, std::hypot(x[2], y[2]));
    }
    const double scale = std::max({1.0, peak_rate / config_.target_xy_max_speed_m_s, std::sqrt(peak_acceleration / config_.target_xy_max_accel_m_s2)});
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

/// 校验任务的起飞、右移、前飞和视觉跟踪参数。
void MissionConfig::validate() const
{
  const double values[] = {
    takeoff_height_m, height_stable_seconds, right_shift_m, forward_distance_m,
    tracking_arrival_seconds, tracking_tolerance_m, match_hold_seconds, car_status_timeout_s,
    max_tracking_radius_m};
  for (const double value : values) {
    if (!common::finite(value) || value <= 0.0) {
      throw std::invalid_argument("mission configuration values must be finite and positive");
    }
  }
  if (right_shift_m < 0.35 || right_shift_m > 0.40) {
    throw std::invalid_argument("right shift must be within 0.35..0.40 m");
  }
}

/// 保存安全与任务配置，并初始化对应的轨迹规划器。
Navigation::Navigation(const common::SafetyConfig & config, MissionConfig mission)
: config_(config), mission_(std::move(mission)), planner_(config)
{
  mission_.validate();
}

/// 清除任务执行上下文并恢复到等待预检阶段。
void Navigation::reset()
{
  planner_.reset();
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
  latest_car_distance_m_.reset();
}

/// 切换任务阶段，并记录新阶段的开始时间。
void Navigation::transition(const std::string & phase, double now)
{
  phase_ = phase;
  phase_started_at_ = now;
}

/// 将一条待发送的 GCS 协议消息加入本周期输出队列。
void Navigation::emit(communication::MessageType type) {pending_messages_.push_back({type});}

/// 将机器可读的拒绝原因加入本周期输出队列。
void Navigation::reject(const std::string & reason) {pending_rejections_.push_back(reason);}

/// 判断实测本地 XY 与目标点的欧氏距离是否在给定容差内。
bool Navigation::actual_xy_within(
  const common::Telemetry & telemetry, double x, double y, double tolerance) const
{
  return common::finite(telemetry.local_x_m) && common::finite(telemetry.local_y_m) &&
         std::hypot(telemetry.local_x_m - x, telemetry.local_y_m - y) <= tolerance;
}

/// 判断实测 XY 和 Z 是否均已达到设定点稳定容差。
bool Navigation::stable_at(
  const common::Telemetry & telemetry, const common::PositionSetpoint & target) const
{
  return actual_xy_within(telemetry, target.x_m, target.y_m, config_.target_tolerance_m) &&
         common::finite(telemetry.local_z_m) &&
         std::abs(telemetry.local_z_m - target.z_m) <= config_.target_tolerance_m;
}

/// 判断实测姿态换算出的偏航角是否接近目标偏航角。
bool Navigation::actual_yaw_within(
  const common::Telemetry & telemetry, double yaw_rad, double tolerance_rad) const
{
  try {
    const double actual = common::yaw_from_quaternion(common::normalize_quaternion(
      telemetry.orientation.x, telemetry.orientation.y, telemetry.orientation.z, telemetry.orientation.w));
    return std::abs(std::atan2(std::sin(actual - yaw_rad), std::cos(actual - yaw_rad))) <= tolerance_rad;
  } catch (const std::invalid_argument &) {
    return false;
  }
}

/// 固定当前水平位置、规划下降到初始高度并进入降落阶段。
void Navigation::begin_landing(double now, const std::string & reason)
{
  if (planner_.latched()) {
    clear_hold();
    planner_.hold_xy();
    if (control_.origin) {planner_.set_z_target(control_.origin->z_m);}
  }
  landing_reason_ = reason;
  normal_completion_ = false;
  transition("landing", now);
}

/// 写入经校验的任务最终目标，保持点和轨迹中间点不允许调用此函数。
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

/// 规划器只接受当前命令点到调用方给定最终目标的轨迹计算。
void Navigation::plan_to_mission_goal()
{
  if (!planner_.latched() || !control_.mission_goal) {return;}
  const auto & goal = *control_.mission_goal;
  planner_.set_target(goal.x_m, goal.y_m, goal.z_m);
  planner_.set_yaw_rad(common::yaw_from_quaternion(goal.orientation));
}

/// 保持使用实测 XYZ，偏航始终沿用最近实际发布的命令。
common::PositionSetpoint Navigation::measured_hold_setpoint(const NavigationInput & input) const
{
  common::PositionSetpoint fallback = control_.commanded_setpoint.value_or(planner_.current());
  common::PositionSetpoint hold = fallback;
  if (common::finite(input.telemetry.local_x_m)) {hold.x_m = input.telemetry.local_x_m;}
  if (common::finite(input.telemetry.local_y_m)) {hold.y_m = input.telemetry.local_y_m;}
  if (common::finite(input.telemetry.local_z_m)) {hold.z_m = input.telemetry.local_z_m;}
  hold.vertical_rate_m_s = 0.0;
  return hold;
}

/// 冻结发布命令但保留 mission_goal，供后续恢复时重新规划。
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

/// 保持元数据与任务最终目标有独立的生命周期。
void Navigation::clear_hold()
{
  control_.hold_setpoint.reset();
  control_.hold_reason.clear();
  control_.hold_resume_phase.clear();
  control_.mission_paused = false;
}

/// LCP 恢复后从冻结的实测位置连续重规划到中断前的任务目标。
void Navigation::resume_hold(double now)
{
  const std::string resume = control_.hold_resume_phase.empty() ? "pursuing_car" : control_.hold_resume_phase;
  plan_to_mission_goal();
  clear_hold();
  transition(resume, now);
}

/// 使用锁存起飞航向的右侧方向，为地面站的黑线对线检查规划横移。
void Navigation::begin_right_shift()
{
  if (!control_.origin || !control_.mission_goal) {
    throw std::logic_error("right shift requires a latched mission origin and climb goal");
  }
  const double yaw = common::yaw_from_quaternion(control_.origin->orientation);
  auto goal = *control_.mission_goal;
  goal.x_m = control_.origin->x_m + mission_.right_shift_m * std::sin(yaw);
  goal.y_m = control_.origin->y_m - mission_.right_shift_m * std::cos(yaw);
  goal.orientation = control_.origin->orientation;
  set_mission_goal(goal);
  plan_to_mission_goal();
}

/// 从横移后的目标沿锁存起飞航向前飞；距离由部署参数限制。
void Navigation::begin_forward_pursuit()
{
  if (!control_.mission_goal || !control_.origin) {
    throw std::logic_error("forward pursuit requires a right-shift mission goal");
  }
  const double yaw = common::yaw_from_quaternion(control_.origin->orientation);
  auto goal = *control_.mission_goal;
  goal.x_m += mission_.forward_distance_m * std::cos(yaw);
  goal.y_m += mission_.forward_distance_m * std::sin(yaw);
  goal.orientation = control_.origin->orientation;
  set_mission_goal(goal);
  plan_to_mission_goal();
}

/// 把相对机体的距离和方位角换算成 ENU 目标，不改变机头指向或任务高度。
bool Navigation::apply_car_status(
  const NavigationInput & input, const communication::CarStatus & status)
{
  if (!control_.origin || !control_.mission_goal || !common::finite(input.telemetry.local_x_m) ||
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
    return false;
  }

  planner_.set_yaw_rad(yaw);
  planner_.set_z_target(control_.mission_goal->z_m);
  control_.tracking_arrival_time_met = planner_.set_xy_target_with_arrival_time(
    target_x, target_y, mission_.tracking_arrival_seconds);
  last_car_status_at_ = input.now;
  latest_car_distance_m_ = status.distance_m;
  return true;
}

bool Navigation::car_status_fresh(double now) const
{
  return last_car_status_at_ && now >= *last_car_status_at_ &&
         now - *last_car_status_at_ <= mission_.car_status_timeout_s;
}

bool Navigation::accompanying_timeout_hold() const
{
  return phase_ == "tracking_timeout_hold" &&
    control_.hold_resume_phase == "accompanying_car";
}

/// 爬升不依赖运行期 LCP，其余需要位置任务语义的阶段必须冻结。
bool Navigation::lcp_required_in_phase() const
{
  return phase_ == "height_stabilizing" || phase_ == "right_shift" ||
    phase_ == "waiting_go_ahead" || phase_ == "pursuing_car" ||
    phase_ == "tracking_to_match" || phase_ == "accompanying_car" || phase_ == "tracking_timeout_hold" ||
    phase_ == "match_hold" || phase_ == "throwing" || phase_ == "returning";
}

/// 按当前任务阶段处理 ACK、前飞放行、连续视觉测量、匹配和返航协议事件。
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
    if (event.type == communication::MessageType::go_ahead_ok) {
      if (phase_ == "waiting_go_ahead") {
        begin_forward_pursuit();
        transition("pursuing_car", input.now);
      } else if (phase_ != "pursuing_car" && phase_ != "tracking_to_match" &&
        phase_ != "accompanying_car" && phase_ != "tracking_timeout_hold" &&
        phase_ != "match_hold" && phase_ != "throwing" &&
        phase_ != "returning" && phase_ != "downing" &&
        phase_ != "disarming" && phase_ != "manual_request_pending" && phase_ != "manual") {
        reject("go_ahead_ok_not_allowed_in_phase");
      }
      continue;
    }
    if (event.type == communication::MessageType::car_status) {
      if (!event.car_status) {
        reject("car_status_missing_measurement");
      } else if (phase_ == "pursuing_car") {
        if (apply_car_status(input, *event.car_status)) {transition("tracking_to_match", input.now);}
      } else if (phase_ == "tracking_to_match" || phase_ == "accompanying_car") {
        apply_car_status(input, *event.car_status);
      } else if (phase_ == "tracking_timeout_hold" &&
        (control_.hold_resume_phase == "tracking_to_match" ||
        control_.hold_resume_phase == "accompanying_car")) {
        const std::string resume_phase = control_.hold_resume_phase;
        if (apply_car_status(input, *event.car_status)) {
          clear_hold();
          transition(resume_phase, input.now);
        }
      } else {
        reject("car_status_not_allowed_in_phase");
      }
      continue;
    }
    if (event.type == communication::MessageType::match_car_ok) {
      if (phase_ == "tracking_to_match") {
        if (!car_status_fresh(input.now) || !latest_car_distance_m_) {
          reject("match_car_ok_requires_fresh_car_status");
        } else if (*latest_car_distance_m_ > mission_.tracking_tolerance_m) {
          reject("match_car_ok_distance_above_tracking_tolerance");
        } else {
          enter_hold(input, "match_hold", "match_confirmed_hold", "tracking_to_match");
        }
      } else if (phase_ != "match_hold" && phase_ != "throwing" && phase_ != "accompanying_car" &&
        phase_ != "returning" &&
        phase_ != "downing" && phase_ != "disarming" &&
        phase_ != "manual_request_pending" && phase_ != "manual") {
        reject("match_car_ok_not_allowed_in_phase");
      }
      continue;
    }
    if (event.type == communication::MessageType::b_ok) {
      if ((phase_ == "accompanying_car" || accompanying_timeout_hold()) && control_.origin) {
        clear_hold();
        auto return_goal = *control_.origin;
        // Return horizontally at the current task altitude; the normal descent event owns origin Z.
        if (control_.mission_goal) {return_goal.z_m = control_.mission_goal->z_m;}
        return_goal.orientation = {0.0, 0.0, 0.0, 1.0};
        set_mission_goal(return_goal);
        plan_to_mission_goal();
        emit(communication::MessageType::ok_return);
        transition("returning", input.now);
      } else if (phase_ != "returning" && phase_ != "downing" && phase_ != "disarming" &&
        phase_ != "manual_request_pending" && phase_ != "manual") {
        reject("b_ok_not_allowed_in_phase");
      }
    }
  }
}

/// 推进完整任务状态机，输出位置、飞控模式、解锁和通信决策。
NavigationDecision Navigation::update(const NavigationInput & input)
{
  if (!common::finite(input.now) || !common::finite(input.dt) || input.dt <= 0.0) {
    throw std::invalid_argument("navigation time and dt must be finite, dt must be positive");
  }
  pending_messages_.clear();
  pending_rejections_.clear();
  pending_release_gripper_ = false;
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
      begin_landing(input.now,
        input.health_errors.empty() ? "flight_health_failure" : input.health_errors.front());
    } else {
      // The flight origin is immutable and is not created by ground setpoint warmup.
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

  if (lcp_required_in_phase() && !input.lcp_healthy) {
    enter_hold(input, "lcp_hold", "lcp_unhealthy");
  }

  if (phase_ == "climb") {
    if (planner_.target_reached() && stable_at(input.telemetry, planner_.current())) {
      transition("height_stabilizing", input.now);
    }
  } else if (phase_ == "height_stabilizing") {
    if (!planner_.target_reached() || !stable_at(input.telemetry, planner_.current())) {
      // 高度、水平位置任一离开容差时，必须重新满足连续稳定时长。
      phase_started_at_ = input.now;
    } else if (input.now - phase_started_at_ >= mission_.height_stable_seconds) {
      emit(communication::MessageType::ok_height);
      begin_right_shift();
      transition("right_shift", input.now);
    }
  } else if (phase_ == "right_shift") {
    if (planner_.target_reached() && stable_at(input.telemetry, planner_.current())) {
      transition("waiting_go_ahead", input.now);
    }
  } else if (phase_ == "tracking_to_match" || phase_ == "accompanying_car") {
    if (!car_status_fresh(input.now)) {
      enter_hold(input, "tracking_timeout_hold", "car_status_timeout", phase_);
    }
  } else if (phase_ == "match_hold") {
    if (input.now - phase_started_at_ >= mission_.match_hold_seconds) {
      pending_release_gripper_ = true;
      transition("throwing", input.now);
    }
  } else if (phase_ == "throwing") {
    if (input.gripper_failed) {
      reject("gripper_release_failed");
      plan_to_mission_goal();
      clear_hold();
      last_car_status_at_.reset();
      latest_car_distance_m_.reset();
      transition("pursuing_car", input.now);
    } else if (input.gripper_succeeded) {
      clear_hold();
      emit(communication::MessageType::ok_throw);
      transition("accompanying_car", input.now);
    }
  } else if (phase_ == "returning") {
    if (control_.origin && planner_.target_reached() && actual_xy_within(
        input.telemetry, control_.origin->x_m, control_.origin->y_m, config_.target_tolerance_m) &&
      actual_yaw_within(input.telemetry, 0.0, 0.10)) {
      auto descent_goal = *control_.origin;
      descent_goal.orientation = {0.0, 0.0, 0.0, 1.0};
      set_mission_goal(descent_goal);
      plan_to_mission_goal();
      normal_completion_ = true;
      emit(communication::MessageType::ok_downing);
      transition("downing", input.now);
    }
  } else if (phase_ == "lcp_hold") {
    if (input.lcp_healthy) {resume_hold(input.now);}
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
  if (waiting_gcs_phase && !input.preflight_ready) {
    reset();
  }

  const bool post_run_prearm_phase = phase_ == "offboard_request_pending" || phase_ == "arming_request_pending";
  if (post_run_prearm_phase && !input.controller.armed && !input.preflight_ready) {
    // 已发出的 OFFBOARD/ARM 服务请求不能撤销；复用既有安全收尾状态显式反向请求。
    reset();
    transition("manual_request_pending", input.now);
    reject("preflight_lost_before_arm");
  }

  const bool airborne = phase_ != "waiting_preflight" && phase_ != "waiting_run_plan1" &&
    phase_ != "setpoint_warmup" && phase_ != "offboard_request_pending" &&
    phase_ != "arming_request_pending" && phase_ != "preflight_abort" && phase_ != "manual";
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
    phase_ == "height_stabilizing" || phase_ == "right_shift" || phase_ == "waiting_go_ahead" ||
    phase_ == "pursuing_car" || phase_ == "tracking_to_match" || phase_ == "accompanying_car" ||
    phase_ == "tracking_timeout_hold" || phase_ == "match_hold" || phase_ == "throwing" ||
    phase_ == "returning" ||
    phase_ == "lcp_hold" || phase_ == "downing") {
    decision.target_mode = "OFFBOARD";
  }
  if (phase_ == "landing" && !landing_reason_.empty()) {decision.target_mode = "AUTO.LAND";}
  if (phase_ == "arming_request_pending" || phase_ == "climb" || phase_ == "height_stabilizing" ||
    phase_ == "right_shift" || phase_ == "waiting_go_ahead" || phase_ == "pursuing_car" ||
    phase_ == "tracking_to_match" || phase_ == "accompanying_car" ||
    phase_ == "tracking_timeout_hold" || phase_ == "match_hold" || phase_ == "throwing" ||
    phase_ == "returning" || phase_ == "lcp_hold" ||
    phase_ == "downing" || phase_ == "landing") {
    decision.arm_intent = true;
  }
  if (phase_ == "disarming" || phase_ == "manual_request_pending") {decision.arm_intent = false;}
  if (phase_ == "manual_request_pending" || phase_ == "manual") {decision.target_mode = "MANUAL";}
  decision.control = control_;
  return decision;
}

}  // mavros_xyz_position_offboard::navigation 命名空间
