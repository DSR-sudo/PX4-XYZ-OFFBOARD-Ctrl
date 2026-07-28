#include "mavros_xyz_position_offboard/navigation/navigation.hpp"

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
  latched_ = false; flow_effective_ = false; x_m_ = NAN; y_m_ = NAN; origin_x_m_ = NAN; origin_y_m_ = NAN;
  target_x_m_ = NAN; target_y_m_ = NAN; xy_velocity_x_m_s_ = 0.0; xy_velocity_y_m_s_ = 0.0;
  xy_trajectory_elapsed_s_ = 0.0; xy_trajectory_duration_s_ = 0.0; xy_coefficients_ = {};
  origin_z_m_ = NAN; command_z_m_ = NAN; target_z_m_ = NAN; vertical_rate_m_s_ = 0.0; orientation_ = {};
  trajectory_elapsed_s_ = 0.0; trajectory_duration_s_ = 0.0; coefficients_ = {};
}

/// 对依赖原点的操作实施“必须先锁存”的不变量。
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

/// 校验当前位姿后锁存为导航原点和初始保持设定点。
void TrajectoryPlanner::latch(double x_m, double y_m, double z_m, const common::Quaternion & orientation)
{
  if (!common::finite(x_m) || !common::finite(y_m) || !common::finite(z_m)) {
    throw std::invalid_argument("local XYZ pose must be finite before latching");
  }
  orientation_ = common::normalize_quaternion(orientation.x, orientation.y, orientation.z, orientation.w);
  origin_x_m_ = x_m; origin_y_m_ = y_m; x_m_ = x_m; y_m_ = y_m; target_x_m_ = x_m; target_y_m_ = y_m;
  xy_velocity_x_m_s_ = 0.0; xy_velocity_y_m_s_ = 0.0; xy_trajectory_elapsed_s_ = 0.0; xy_trajectory_duration_s_ = 0.0;
  xy_coefficients_ = {{{x_m, 0, 0, 0, 0, 0}, {y_m, 0, 0, 0, 0, 0}}};
  origin_z_m_ = z_m; command_z_m_ = z_m; target_z_m_ = z_m; vertical_rate_m_s_ = 0.0;
  trajectory_elapsed_s_ = 0.0; trajectory_duration_s_ = 0.0; coefficients_ = {z_m, 0, 0, 0, 0, 0};
  latched_ = true; flow_effective_ = false;
}

/// 在飞行器实际解锁位置重新对齐水平原点。
void TrajectoryPlanner::recenter_xy(double x_m, double y_m)
{
  require_latched("recentering XY");
  if (!common::finite(x_m) || !common::finite(y_m)) {throw std::invalid_argument("recenter XY pose must be finite");}
  origin_x_m_ = x_m; origin_y_m_ = y_m; x_m_ = x_m; y_m_ = y_m; target_x_m_ = x_m; target_y_m_ = y_m;
  xy_velocity_x_m_s_ = 0.0; xy_velocity_y_m_s_ = 0.0; xy_trajectory_elapsed_s_ = 0.0; xy_trajectory_duration_s_ = 0.0;
  xy_coefficients_ = {{{x_m, 0, 0, 0, 0, 0}, {y_m, 0, 0, 0, 0, 0}}};
}

/// 从当前输出姿态返回偏航角。
double TrajectoryPlanner::yaw_rad() const {return common::yaw_from_quaternion(orientation_);}

/// 以锁存高度为参考设置相对上升或下降目标。
void TrajectoryPlanner::set_relative_target(double relative_z_m)
{
  require_latched("setting a target");
  if (!common::finite(relative_z_m)) {throw std::invalid_argument("relative Z target must be finite");}
  begin_z_trajectory(origin_z_m_ + relative_z_m);
}

/// 校验并开始一段受限的水平移动。
void TrajectoryPlanner::set_xy_target(double x_m, double y_m)
{
  require_latched("setting an XY target");
  if (!common::finite(x_m) || !common::finite(y_m)) {throw std::invalid_argument("XY target must be finite");}
  begin_xy_trajectory(x_m, y_m);
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

/// 创建返回锁存地面高度的垂直轨迹。
void TrajectoryPlanner::set_ground_target() {require_latched("setting a target"); begin_z_trajectory(origin_z_m_);}

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
void TrajectoryPlanner::begin_xy_trajectory(double target_x, double target_y)
{
  const double distance = std::hypot(target_x - x_m_, target_y - y_m_);
  const double speed = std::hypot(xy_velocity_x_m_s_, xy_velocity_y_m_s_);
  target_x_m_ = target_x; target_y_m_ = target_y;
  if (distance < 1e-9 && speed < 1e-9) {
    x_m_ = target_x; y_m_ = target_y; xy_velocity_x_m_s_ = 0.0; xy_velocity_y_m_s_ = 0.0; xy_trajectory_elapsed_s_ = 0.0; xy_trajectory_duration_s_ = 0.0;
    xy_coefficients_ = {{{target_x, 0, 0, 0, 0, 0}, {target_y, 0, 0, 0, 0, 0}}}; return;
  }
  double duration = std::max({0.5, 2.0 * distance / config_.target_xy_max_speed_m_s,
    std::sqrt(6.0 * distance / config_.target_xy_max_accel_m_s2), 2.0 * speed / config_.target_xy_max_accel_m_s2});
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
  xy_trajectory_elapsed_s_ = 0.0; xy_trajectory_duration_s_ = duration; xy_coefficients_ = candidate;
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

/// 保存统一安全配置并创建同生命周期的纯轨迹规划器。
Navigation::Navigation(const common::SafetyConfig & config) : config_(config), planner_(config) {}

/// 将任务状态机及全部跨周期上下文恢复到 waiting_preflight 初始状态。
void Navigation::reset()
{
  planner_.reset();
  phase_ = "waiting_preflight";
  phase_started_at_ = 0.0;
  preflight_sent_ = false;
  takeoff_height_m_ = 0.0;
  reset_batch();
  waypoints_.clear();
  waypoint_index_ = 0;
  last_ack_at_.reset();
  stable_since_.reset();
  flight_started_at_.reset();
  held_target_.reset();
  hold_resume_phase_.clear();
  pending_messages_.clear();
  pending_rejections_.clear();
  landing_reason_.clear();
}

/// 原子更新阶段名称、阶段开始时间，并取消上一阶段的稳定计时。
void Navigation::transition(const std::string & phase, double now)
{
  phase_ = phase;
  phase_started_at_ = now;
  stable_since_.reset();
}

/// 将 Navigation 产生的一条业务消息加入当前周期输出。
void Navigation::emit(communication::MessageType type) {pending_messages_.push_back({type, {}});}

/// 将协议或阶段拒绝原因加入当前周期审计输出。
void Navigation::reject(const std::string & reason) {pending_rejections_.push_back(reason);}

/// 清空三类导航配置副本，确保失败批次不能与后续批次混合。
void Navigation::reset_batch()
{
  and_point_packets_.clear();
  nfz_packets_.clear();
  plan_packets_.clear();
}

/// 使用 MAVROS 实测 local ENU XY 计算到目标的欧氏距离并应用闭区间容差。
bool Navigation::actual_xy_within(
  const common::Telemetry & telemetry, double x, double y, double tolerance) const
{
  return common::finite(telemetry.local_x_m) && common::finite(telemetry.local_y_m) &&
         std::hypot(telemetry.local_x_m - x, telemetry.local_y_m - y) <= tolerance;
}

/// 同时检查实测 XY 欧氏距离和 Z 误差是否满足稳定位置容差。
bool Navigation::stable_at(
  const common::Telemetry & telemetry, const common::PositionSetpoint & target) const
{
  return actual_xy_within(telemetry, target.x_m, target.y_m, config_.target_tolerance_m) &&
         common::finite(telemetry.local_z_m) &&
         std::abs(telemetry.local_z_m - target.z_m) <= config_.target_tolerance_m;
}

/// 保存正常或故障降落原因，固定 XY 并规划下降到初始化 Z。
void Navigation::begin_landing(double now, const std::string & reason)
{
  if (planner_.latched()) {
    planner_.hold_xy();
    planner_.set_ground_target();
  }
  landing_reason_ = reason;
  transition("landing", now);
}

/// 保存当前活动目标，在可靠实测 XY 和当前命令 Z 建立 link/LCP 冻结状态。
void Navigation::enter_hold(const NavigationInput & input, const std::string & hold_phase)
{
  if (!planner_.latched()) {return;}
  held_target_ = communication::Point3{
    planner_.target_x_m(), planner_.target_y_m(), planner_.target_z_m()};
  hold_resume_phase_ = phase_;
  if (common::finite(input.telemetry.local_x_m) && common::finite(input.telemetry.local_y_m)) {
    planner_.freeze_xy_at(input.telemetry.local_x_m, input.telemetry.local_y_m);
  } else {
    planner_.hold_xy();
  }
  planner_.freeze_z();
  transition(hold_phase, input.now);
}

/// 从冻结设定点连续重规划到保存目标，并返回被中断的任务阶段。
void Navigation::resume_hold(double now)
{
  if (held_target_) {
    planner_.set_target(held_target_->x_m, held_target_->y_m, held_target_->z_m);
  }
  const std::string resume = hold_resume_phase_.empty() ? "run_fly_plan" : hold_resume_phase_;
  held_target_.reset();
  hold_resume_phase_.clear();
  transition(resume, now);
}

/// 验证三类数据各恰好三份且完全一致，构建航点队列并启动 plan_mode=1 任务。
bool Navigation::finish_batch(double now)
{
  const auto same_and_point = [this]() {
      if (and_point_packets_.size() != 3U) {return false;}
      const auto & first = and_point_packets_.front();
      for (const auto & packet : and_point_packets_) {
        if (packet.plan_mode != first.plan_mode || packet.final_point != first.final_point) {return false;}
      }
      return true;
    };
  const auto same_points = [](const std::vector<communication::ProtocolEvent> & packets) {
      if (packets.size() != 3U) {return false;}
      for (const auto & packet : packets) {if (packet.points != packets.front().points) {return false;}}
      return true;
    };
  if (!same_and_point() || !same_points(nfz_packets_) || !same_points(plan_packets_)) {
    reject("navigation_batch_incomplete_or_inconsistent");
    reset_batch();
    return false;
  }
  const auto & mission = and_point_packets_.front();
  if (!mission.plan_mode || *mission.plan_mode != 1) {
    reject("autonomous_planning_not_supported");
    reset_batch();
    return false;
  }
  waypoints_ = plan_packets_.front().points;
  if (!mission.final_point) {
    reject("navigation_final_point_missing");
    reset_batch();
    return false;
  }
  if (waypoints_.empty() || !(waypoints_.back() == *mission.final_point)) {
    waypoints_.push_back(*mission.final_point);
  }
  waypoint_index_ = 0;
  planner_.set_target(waypoints_[0].x_m, waypoints_[0].y_m, waypoints_[0].z_m);
  last_ack_at_ = now;
  emit(communication::MessageType::ok_receive);
  transition("run_fly_plan", now);
  reset_batch();
  return true;
}

/// 处理 ACK、start、任务完成确认及当前阶段允许的导航批次消息。
void Navigation::process_events(const NavigationInput & input)
{
  for (const auto & event : input.events) {
    if (!event.accepted) {if (!event.rejection_reason.empty()) {reject(event.rejection_reason);} continue;}
    if (event.type == communication::MessageType::ack) {
      last_ack_at_ = input.now;
      continue;
    }
    if (event.type == communication::MessageType::start) {
      if (phase_ != "waiting_start" || !event.height_start_m) {
        reject("start_not_allowed_in_phase");
      } else {
        takeoff_height_m_ = *event.height_start_m;
        planner_.set_relative_target(0.0);
        transition("setpoint_warmup", input.now);
      }
      continue;
    }
    if (event.type == communication::MessageType::ok_fly_plan_succeed) {
      if (phase_ == "awaiting_fly_plan_succeed") {begin_landing(input.now);}
      else {reject("ok_fly_plan_succeed_not_allowed_in_phase");}
      continue;
    }
    if (phase_ != "waiting_navigation_config") {
      reject("navigation_config_not_allowed_in_phase");
      continue;
    }
    auto append = [this](std::vector<communication::ProtocolEvent> & packets,
        const communication::ProtocolEvent & value) {
        if (packets.size() >= 3U) {
          reject("navigation_batch_count_exceeded");
          reset_batch();
          return;
        }
        packets.push_back(value);
      };
    switch (event.type) {
      case communication::MessageType::navigation_and_point: append(and_point_packets_, event); break;
      case communication::MessageType::navigation_nfz: append(nfz_packets_, event); break;
      case communication::MessageType::navigation_plan: append(plan_packets_, event); break;
      case communication::MessageType::navigation_fly_plan_send_ok: finish_batch(input.now); break;
      default: reject("message_not_allowed_in_phase"); break;
    }
  }
}

/// 推进完整任务和故障状态机，生成本周期设定点、模式、ARM 与 GCS 输出意图。
NavigationDecision Navigation::update(const NavigationInput & input)
{
  if (!common::finite(input.now) || !common::finite(input.dt) || input.dt <= 0.0) {
    throw std::invalid_argument("navigation time and dt must be finite, dt must be positive");
  }
  pending_messages_.clear();
  pending_rejections_.clear();
  process_events(input);

  if (phase_ == "waiting_preflight") {
    if (input.preflight_ready && input.lcp_healthy && common::finite(input.telemetry.local_x_m) &&
      common::finite(input.telemetry.local_y_m) && common::finite(input.telemetry.local_z_m)) {
      planner_.latch(input.telemetry.local_x_m, input.telemetry.local_y_m,
        input.telemetry.local_z_m, input.telemetry.orientation);
      planner_.set_yaw_rad(1.57079632679489661923);
      if (!preflight_sent_) {emit(communication::MessageType::ok_preflight); preflight_sent_ = true;}
      transition("waiting_start", input.now);
    }
  } else if (phase_ == "setpoint_warmup") {
    if (input.now - phase_started_at_ >= config_.setpoint_warmup_s) {
      transition("offboard_request_pending", input.now);
    }
  } else if (phase_ == "offboard_request_pending") {
    if (input.controller.mode == "OFFBOARD") {transition("arming_request_pending", input.now);}
  } else if (phase_ == "arming_request_pending") {
    if (input.controller.mode != "OFFBOARD") {transition("offboard_request_pending", input.now);}
    else if (input.controller.armed) {
      emit(communication::MessageType::ok_flight);
      planner_.set_relative_target(takeoff_height_m_);
      flight_started_at_ = input.now;
      transition("climb", input.now);
    }
  } else if (phase_ == "climb") {
    if (planner_.target_reached() && stable_at(input.telemetry, planner_.current())) {
      transition("stabilize", input.now);
    }
  } else if (phase_ == "stabilize") {
    if (stable_at(input.telemetry, planner_.current())) {
      if (!stable_since_) {stable_since_ = input.now;}
      if (input.now - *stable_since_ >= config_.hold_seconds) {
        emit(communication::MessageType::wait_plan);
        transition("waiting_navigation_config", input.now);
      }
    } else {stable_since_.reset();}
  } else if (phase_ == "run_fly_plan") {
    if (!input.lcp_healthy) {
      enter_hold(input, "lcp_hold");
    } else if (last_ack_at_ && input.now - *last_ack_at_ > 2.0) {
      enter_hold(input, "link_hold");
    } else if (!waypoints_.empty() && planner_.xy_target_reached() &&
      actual_xy_within(input.telemetry, waypoints_[waypoint_index_].x_m,
        waypoints_[waypoint_index_].y_m, 0.2)) {
      ++waypoint_index_;
      if (waypoint_index_ >= waypoints_.size()) {
        planner_.hold_xy();
        planner_.freeze_z();
        emit(communication::MessageType::ok_fly_plan);
        transition("awaiting_fly_plan_succeed", input.now);
      } else {
        const auto & target = waypoints_[waypoint_index_];
        planner_.set_target(target.x_m, target.y_m, target.z_m);
      }
    }
  } else if (phase_ == "link_hold") {
    if (last_ack_at_ && input.now - *last_ack_at_ <= 2.0) {resume_hold(input.now);}
  } else if (phase_ == "lcp_hold") {
    if (input.lcp_healthy) {resume_hold(input.now);}
    else if (input.now - phase_started_at_ > config_.lcp_unhealthy_hold_timeout_s) {
      begin_landing(input.now, "lcp_unhealthy_timeout");
    }
  } else if (phase_ == "landing") {
    const bool on_ground = input.telemetry.landed_state == common::MAV_LANDED_STATE_ON_GROUND;
    const bool at_origin = common::finite(input.telemetry.local_z_m) && planner_.latched() &&
      std::abs(input.telemetry.local_z_m - planner_.origin_z_m()) <= config_.touchdown_z_tolerance_m;
    if (on_ground || (planner_.target_reached() && at_origin)) {transition("disarming", input.now);}
  } else if (phase_ == "disarming") {
    if (!input.controller.armed) {transition("manual_request_pending", input.now);}
  } else if (phase_ == "manual_request_pending") {
    if (input.controller.mode == "MANUAL") {transition("manual", input.now);}
  }

  const bool prearm_phase = phase_ == "waiting_start" || phase_ == "setpoint_warmup" ||
    phase_ == "offboard_request_pending" || phase_ == "arming_request_pending";
  if (prearm_phase && !input.preflight_ready) {
    planner_.reset();
    preflight_sent_ = false;
    transition("waiting_preflight", input.now);
  }

  const bool lcp_required_phase = phase_ == "climb" || phase_ == "stabilize" ||
    phase_ == "waiting_navigation_config" || phase_ == "run_fly_plan" ||
    phase_ == "link_hold" || phase_ == "awaiting_fly_plan_succeed";
  if (lcp_required_phase && !input.lcp_healthy) {enter_hold(input, "lcp_hold");}

  const bool airborne_phase = phase_ != "waiting_preflight" && phase_ != "waiting_start" &&
    phase_ != "setpoint_warmup" && phase_ != "offboard_request_pending" &&
    phase_ != "arming_request_pending" && phase_ != "manual";
  if (airborne_phase && phase_ != "landing" && phase_ != "disarming" &&
    phase_ != "manual_request_pending" && !input.flight_healthy) {
    begin_landing(input.now, input.health_errors.empty() ? "flight_health_failure" : input.health_errors.front());
  }
  if (airborne_phase && phase_ != "landing" && phase_ != "disarming" &&
    phase_ != "manual_request_pending" && input.controller.mode != "OFFBOARD") {
    begin_landing(input.now, "offboard_mode_lost");
  }
  if (flight_started_at_ && airborne_phase && phase_ != "landing" &&
    input.now - *flight_started_at_ > config_.max_flight_seconds) {
    begin_landing(input.now, "maximum_flight_time_exceeded");
  }

  NavigationDecision decision;
  decision.phase = phase_;
  decision.messages = pending_messages_;
  decision.rejections = pending_rejections_;
  decision.waypoint_index = waypoint_index_;
  if (last_ack_at_ && input.now >= *last_ack_at_) {decision.ack_age_s = input.now - *last_ack_at_;}
  if (planner_.latched() && phase_ != "waiting_preflight" && phase_ != "waiting_start" && phase_ != "manual") {
    decision.setpoint = planner_.update(input.dt);
  }
  if (phase_ == "offboard_request_pending" || phase_ == "arming_request_pending" ||
    phase_ == "climb" || phase_ == "stabilize" || phase_ == "waiting_navigation_config" ||
    phase_ == "run_fly_plan" || phase_ == "link_hold" || phase_ == "lcp_hold" ||
    phase_ == "awaiting_fly_plan_succeed" || (phase_ == "landing" && landing_reason_ != "offboard_mode_lost")) {
    decision.target_mode = "OFFBOARD";
  }
  if (phase_ == "landing" && !landing_reason_.empty()) {decision.target_mode = "AUTO.LAND";}
  if (phase_ == "arming_request_pending" || phase_ == "climb" || phase_ == "stabilize" ||
    phase_ == "waiting_navigation_config" || phase_ == "run_fly_plan" || phase_ == "link_hold" ||
    phase_ == "lcp_hold" || phase_ == "awaiting_fly_plan_succeed" || phase_ == "landing") {
    decision.arm_intent = true;
  }
  if (phase_ == "disarming") {decision.arm_intent = false;}
  if (phase_ == "manual_request_pending" || phase_ == "manual") {decision.target_mode = "MANUAL";}
  return decision;
}

}  // namespace mavros_xyz_position_offboard::navigation
