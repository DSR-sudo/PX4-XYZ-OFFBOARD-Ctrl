#include "mavros_xyz_position_offboard/offboard/offboard.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <functional>
#include <future>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace mavros_xyz_position_offboard::offboard
{
namespace
{
/// 将布尔值编码为 JSON 字面量。
std::string bool_json(bool value) {return value ? "true" : "false";}

/// 将有限浮点数编码为 JSON 数字，非有限值编码为 null。
std::string number_json(double value)
{
  if (!common::finite(value)) {return "null";}
  std::ostringstream stream; stream << std::setprecision(12) << value; return stream.str();
}

/// 将可选浮点数编码为数字或 null。
std::string optional_number_json(const std::optional<double> & value) {return value ? number_json(*value) : "null";}
/// 将可选布尔值编码为字面量或 null。
std::string optional_bool_json(const std::optional<bool> & value) {return value ? bool_json(*value) : "null";}
/// 将可选整数编码为 JSON 数字或 null。
std::string optional_int_json(const std::optional<int> & value) {return value ? std::to_string(*value) : "null";}
/// 将可选字符串转义后编码为 JSON 字符串或 null。
std::string optional_string_json(const std::optional<std::string> & value)
{
  return value ? "\"" + common::json_escape(*value) + "\"" : "null";
}

}  // namespace

/// 创建唯一节点、四层协作对象、按权限启用的 MAVROS 资源和控制定时器。
MavrosNativeXYZNode::MavrosNativeXYZNode(const common::AppOptions & options, const common::SafetyConfig & config)
: Node("mavros_native_xyz_position"), options_(options), config_(config), initialization_(*this, options_, config_),
  navigation_(config_), lcp_vision_bridge_(*this, options_), artifact_log_(options_.artifact_dir, options_.output == "jsonl"),
  publish_enabled_(common::setpoint_enabled(options_)), mode_enabled_(common::mode_enabled(options_)),
  arming_enabled_(common::arming_enabled(options_))
{
  const auto sensor_qos = rclcpp::SensorDataQoS();
  if (publish_enabled_) {
    setpoint_publisher_ = create_publisher<mavros_msgs::msg::PositionTarget>(options_.setpoint_topic, sensor_qos);
    phase_ = "waiting_preflight";
  }
  if (mode_enabled_) {mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode"); last_mode_event_.status = "never_requested";}
  if (arming_enabled_) {arm_client_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming"); last_arm_event_.status = "never_requested";}
  timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / config_.publish_rate_hz), std::bind(&MavrosNativeXYZNode::tick, this));
}

/// 确保退出节点时关闭 artifact 日志。
MavrosNativeXYZNode::~MavrosNativeXYZNode() {artifact_log_.close();}

/// 返回用于控制周期和超时判定的单调秒数。
double MavrosNativeXYZNode::monotonic_now()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

/// 将模式字符串标准化为大写。
std::string MavrosNativeXYZNode::upper(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {return static_cast<char>(std::toupper(c));});
  return value;
}

/// 构建忽略速度/加速度/yaw-rate 的全 XYZ+yaw MAVROS 原始设定点。
mavros_msgs::msg::PositionTarget MavrosNativeXYZNode::make_position_target(
  const common::PositionSetpoint & setpoint, const builtin_interfaces::msg::Time & stamp)
{
  mavros_msgs::msg::PositionTarget message;
  message.header.stamp = stamp; message.header.frame_id = "map";
  message.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
  message.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_VX |
    mavros_msgs::msg::PositionTarget::IGNORE_VY | mavros_msgs::msg::PositionTarget::IGNORE_VZ |
    mavros_msgs::msg::PositionTarget::IGNORE_AFX | mavros_msgs::msg::PositionTarget::IGNORE_AFY |
    mavros_msgs::msg::PositionTarget::IGNORE_AFZ | mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
  message.position.x = setpoint.x_m; message.position.y = setpoint.y_m; message.position.z = setpoint.z_m;
  message.yaw = static_cast<float>(common::yaw_from_quaternion(setpoint.orientation));
  return message;
}

/// 重置预检候选、LCP 初始化和导航状态，避免旧数据继续生效。
void MavrosNativeXYZNode::reset_candidate(double now)
{
  initialization_.cancel_lcp_start(); initialization_.reset_lcp_initialization(); navigation_.reset();
  setpoint_stream_since_.reset(); sensor_loss_started_at_.reset(); lcp_hold_started_at_.reset();
  lcp_hold_resume_phase_.reset(); lcp_hold_resume_phase_started_at_.reset(); lcp_hold_resume_target_x_m_.reset();
  lcp_hold_resume_target_y_m_.reset(); lcp_hold_resume_target_z_m_.reset(); last_tick_at_ = now;
  phase_ = "waiting_preflight"; phase_started_at_ = now;
}

/// 将 Initialization 当前本地位姿传给 Navigation 作为控制原点。
void MavrosNativeXYZNode::latch_current_pose()
{
  const auto & telemetry = initialization_.telemetry();
  navigation_.latch(telemetry.local_x_m, telemetry.local_y_m, telemetry.local_z_m, telemetry.orientation);
  // The flight test requires a fixed north-facing heading from the first
  // streamed setpoint through climb, waypoints, landing, and disarm.
  navigation_.set_yaw_rad(0.0);
}

/// 由 OFFBOARD 独占推进轨迹并发布 PositionTarget。
void MavrosNativeXYZNode::publish_setpoint(double dt_s)
{
  if (!setpoint_publisher_ || !navigation_.latched()) {return;}
  const auto target = navigation_.update(dt_s);
  navigation_.set_flow_effective(initialization_.optical_flow_effective(monotonic_now()));
  const auto stamp = get_clock()->now();
  builtin_interfaces::msg::Time stamp_message;
  const auto nanoseconds = stamp.nanoseconds();
  stamp_message.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
  stamp_message.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
  setpoint_publisher_->publish(make_position_target(target, stamp_message));
}

/// 轮询所有异步服务，记录结果但只以后续 heartbeat 推进飞行阶段。
void MavrosNativeXYZNode::poll_service_futures(double now)
{
  initialization_.poll_lcp_start(now, options_.service_timeout);
  if (mode_future_) {
    if (mode_future_->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      try {
        const auto response = mode_future_->get();
        last_mode_event_ = {"response", std::nullopt, response->mode_sent, std::nullopt, now};
      } catch (const std::exception & error) {last_mode_event_ = {"exception", std::nullopt, std::nullopt, error.what(), now};}
      mode_future_.reset(); mode_future_started_at_.reset();
    } else if (mode_future_started_at_ && now - *mode_future_started_at_ > options_.service_timeout) {
      mode_future_.reset(); mode_future_started_at_.reset(); last_mode_event_ = {"timeout", std::nullopt, std::nullopt, std::nullopt, now};
    }
  }
  if (arm_future_) {
    if (arm_future_->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      try {
        const auto response = arm_future_->get();
        last_arm_event_ = {"response", std::nullopt, response->success, static_cast<int>(response->result), std::nullopt, now};
      } catch (const std::exception & error) {last_arm_event_ = {"exception", std::nullopt, std::nullopt, std::nullopt, error.what(), now};}
      arm_future_.reset(); arm_future_started_at_.reset();
    } else if (arm_future_started_at_ && now - *arm_future_started_at_ > options_.service_timeout) {
      arm_future_.reset(); arm_future_started_at_.reset(); last_arm_event_ = {"timeout", std::nullopt, std::nullopt, std::nullopt, std::nullopt, now};
    }
  }
}

/// 以配置的最小间隔向 MAVROS 请求模式切换。
void MavrosNativeXYZNode::request_mode(double now, const std::string & mode)
{
  if (!mode_client_ || mode_future_ || now - last_mode_request_at_ < options_.mode_request_interval) {return;}
  last_mode_request_at_ = now;
  if (!mode_client_->service_is_ready()) {last_mode_event_ = {"service_not_ready", mode, std::nullopt, std::nullopt, now}; return;}
  auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>(); request->base_mode = 0; request->custom_mode = mode;
  mode_future_ = mode_client_->async_send_request(request); mode_future_started_at_ = now;
  last_mode_event_ = {"request_sent", mode, std::nullopt, std::nullopt, now};
}

/// 以配置的最小间隔向 MAVROS 发送普通解锁或上锁请求。
void MavrosNativeXYZNode::request_arm(double now, bool value)
{
  if (!arm_client_ || arm_future_ || now - last_arm_request_at_ < options_.mode_request_interval) {return;}
  last_arm_request_at_ = now;
  if (!arm_client_->service_is_ready()) {last_arm_event_ = {"service_not_ready", value, std::nullopt, std::nullopt, std::nullopt, now}; return;}
  auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>(); request->value = value;
  arm_future_ = arm_client_->async_send_request(request); arm_future_started_at_ = now;
  last_arm_event_ = {"request_sent", value, std::nullopt, std::nullopt, std::nullopt, now};
}

/// 为 LCP 服务未接受或数据未就绪的预解锁阶段生成错误文本。
std::vector<std::string> MavrosNativeXYZNode::lcp_prearm_errors(double now) const
{
  auto errors = initialization_.lcp_errors(now, true);
  const auto & state = initialization_.telemetry().lcp_init_request_state;
  if (state == "not_requested" || state == "waiting_service" || state == "request_sent") {errors.insert(errors.begin(), "LCP initialization service has not been accepted");}
  return errors;
}

/// 在未授权控制时仅运行预检并报告监视结果。
void MavrosNativeXYZNode::monitor_tick(double now)
{
  last_errors_ = initialization_.preflight_errors(now); phase_ = last_errors_.empty() ? "preflight_pass" : "blocked";
}

/// 执行从预检到 OFFBOARD/解锁请求前的全部阶段转换。
void MavrosNativeXYZNode::prearm_control_tick(double now, double dt_s)
{
  last_errors_ = initialization_.preflight_errors(now);
  if (!last_errors_.empty()) {
    if (phase_ != "waiting_preflight" || navigation_.latched()) {reset_candidate(now);}
    return;
  }
  const auto & lcp_state = initialization_.telemetry().lcp_init_request_state;
  if (lcp_state == "not_requested" || lcp_state == "waiting_service") {
    phase_ = "lcp_start_pending"; initialization_.request_lcp_start(now); last_errors_ = {"waiting for LCP initialization service"}; return;
  }
  if (lcp_state == "request_sent") {phase_ = "lcp_start_pending"; last_errors_ = {"waiting for LCP initialization service response"}; return;}
  if (lcp_state == "failed") {phase_ = "lcp_start_pending"; last_errors_ = lcp_prearm_errors(now); return;}
  if (!initialization_.lcp_ready(now)) {
    if (navigation_.latched()) {
      navigation_.reset(); setpoint_stream_since_.reset(); mode_future_.reset(); mode_future_started_at_.reset(); arm_future_.reset(); arm_future_started_at_.reset();
    }
    phase_ = "lcp_initializing"; last_errors_ = lcp_prearm_errors(now); return;
  }
  const auto & telemetry = initialization_.telemetry();
  if (phase_ == "arming_request_pending" && telemetry.armed && upper(telemetry.mode) == "OFFBOARD") {
    flight_started_at_ = now; initialization_.seed_drift_baseline(now);
    if (common::finite(telemetry.local_x_m) && common::finite(telemetry.local_y_m)) {navigation_.recenter_xy(telemetry.local_x_m, telemetry.local_y_m);}
    navigation_.set_relative_target(config_.relative_z_m); result_ = "UNCONFIRMED"; phase_ = "climb"; phase_started_at_ = now; return;
  }
  if (!navigation_.latched()) {
    phase_ = "lcp_ready"; phase_started_at_ = now; latch_current_pose(); setpoint_stream_since_ = now; phase_ = "setpoint_warmup"; phase_started_at_ = now;
  }
  publish_setpoint(dt_s);
  if (!setpoint_stream_since_ || now - *setpoint_stream_since_ < config_.setpoint_warmup_s) {phase_ = "setpoint_warmup"; return;}
  if (!mode_enabled_) {phase_ = "position_stream_ready"; return;}
  if (upper(telemetry.mode) != "OFFBOARD") {phase_ = "offboard_request_pending"; request_mode(now, "OFFBOARD"); return;}
  if (!arming_enabled_) {phase_ = "offboard_disarmed_pass"; return;}
  phase_ = "arming_request_pending"; request_arm(now, true);
}

/// 固定水平位置、开始下降，并在需要时标记安全中止原因。
void MavrosNativeXYZNode::begin_landing(double now, const std::optional<std::string> & reason)
{
  navigation_.hold_xy(); navigation_.set_ground_target();
  if (reason) {result_ = "ABORTED_SAFE_PENDING_LANDING"; abort_reason_ = reason;}
  phase_ = "landing"; phase_started_at_ = now;
}

/// 保存当前任务目标并在 LCP 失效期间冻结 XY/Z。
void MavrosNativeXYZNode::enter_lcp_hold(double now, const std::string & reason)
{
  const auto & telemetry = initialization_.telemetry();
  lcp_hold_resume_phase_ = phase_; lcp_hold_resume_phase_started_at_ = phase_started_at_;
  lcp_hold_resume_target_x_m_ = navigation_.target_x_m(); lcp_hold_resume_target_y_m_ = navigation_.target_y_m(); lcp_hold_resume_target_z_m_ = navigation_.target_z_m();
  if (common::finite(telemetry.local_x_m) && common::finite(telemetry.local_y_m)) {navigation_.freeze_xy_at(telemetry.local_x_m, telemetry.local_y_m);} else {navigation_.hold_xy();}
  navigation_.freeze_z(); lcp_hold_started_at_ = now; phase_ = "lcp_hold"; phase_started_at_ = now; abort_reason_ = reason;
}

/// LCP 恢复后重建冻结前的轨迹并修正阶段计时。
void MavrosNativeXYZNode::resume_lcp_hold(double now)
{
  const std::string phase = lcp_hold_resume_phase_.value_or("waypoint");
  const auto previous_started = lcp_hold_resume_phase_started_at_;
  if (lcp_hold_resume_target_x_m_ && lcp_hold_resume_target_y_m_ && common::finite(*lcp_hold_resume_target_x_m_) && common::finite(*lcp_hold_resume_target_y_m_)) {navigation_.set_xy_target(*lcp_hold_resume_target_x_m_, *lcp_hold_resume_target_y_m_);}
  if (lcp_hold_resume_target_z_m_ && common::finite(*lcp_hold_resume_target_z_m_)) {navigation_.set_z_target(*lcp_hold_resume_target_z_m_);}
  phase_ = phase; phase_started_at_ = now;
  if (previous_started && lcp_hold_started_at_) {phase_started_at_ = *previous_started + (now - *lcp_hold_started_at_);}
  lcp_hold_started_at_.reset(); lcp_hold_resume_phase_.reset(); lcp_hold_resume_phase_started_at_.reset(); lcp_hold_resume_target_x_m_.reset(); lcp_hold_resume_target_y_m_.reset(); lcp_hold_resume_target_z_m_.reset();
}

/// 执行飞行期安全策略、LCP 保持策略和分阶段飞行状态机。
void MavrosNativeXYZNode::flight_tick(double now, double dt_s)
{
  const auto & telemetry = initialization_.telemetry();
  const bool at_hover = phase_ == "hold" || phase_ == "lcp_hold" || phase_ == "landing" || phase_ == "normal_disarm_pending";
  const auto errors = initialization_.flight_errors(now, navigation_.x_m(), navigation_.y_m(), true, at_hover);
  std::vector<std::string> stale_errors; std::vector<std::string> other_errors;
  for (const auto & error : errors) {
    if (error == "downward range stale or unavailable" || error == "optical-flow data stale or unavailable") {stale_errors.push_back(error);} else {other_errors.push_back(error);}
  }
  double loss_duration = 0.0;
  if (!stale_errors.empty()) {if (!sensor_loss_started_at_) {sensor_loss_started_at_ = now;} loss_duration = std::max(0.0, now - *sensor_loss_started_at_);} else {sensor_loss_started_at_.reset();}
  auto blocking_errors = errors; auto visible_errors = errors;
  if (!stale_errors.empty() && loss_duration < config_.sensor_loss_grace_s) {
    blocking_errors = other_errors; visible_errors = other_errors;
    std::ostringstream stream; stream << std::fixed << std::setprecision(2) << "range/optical-flow stale for " << loss_duration << "/" << config_.sensor_loss_grace_s << " s; continuing current setpoint"; visible_errors.push_back(stream.str());
  }
  if (flight_started_at_ && now - *flight_started_at_ > config_.max_flight_seconds) {blocking_errors.emplace_back("maximum bounded-flight time exceeded"); visible_errors.emplace_back("maximum bounded-flight time exceeded");}
  last_errors_ = visible_errors;
  /// 将多个阻断错误拼为可审计的单一安全降落原因。
  if (!blocking_errors.empty() && (phase_ == "climb" || phase_ == "hold" || phase_ == "waypoint" || phase_ == "lcp_hold")) {begin_landing(now, [&]() {std::ostringstream s; for (std::size_t i = 0; i < blocking_errors.size(); ++i) {if (i) {s << "; ";} s << blocking_errors[i];} return s.str();}());}
  if (phase_ == "waypoint" && blocking_errors.empty() && !initialization_.lcp_runtime_healthy(now)) {enter_lcp_hold(now, "LCP health loss during waypoint");}
  if (phase_ == "lcp_hold") {
    if (initialization_.lcp_runtime_healthy(now)) {resume_lcp_hold(now);}
    else if (lcp_hold_started_at_ && now - *lcp_hold_started_at_ >= config_.lcp_unhealthy_hold_timeout_s) {begin_landing(now, "LCP health loss");}
  }
  if (phase_ == "climb" || phase_ == "hold" || phase_ == "waypoint" || phase_ == "landing") {
    if (upper(telemetry.mode) == "OFFBOARD") {publish_setpoint(dt_s);} else if (phase_ == "landing") {request_mode(now, "AUTO.LAND");}
  }
  if (phase_ == "lcp_hold") {if (upper(telemetry.mode) == "OFFBOARD") {publish_setpoint(dt_s);} return;}
  if (phase_ == "climb") {
    const bool reached_setpoint = std::abs(navigation_.command_z_m() - navigation_.target_z_m()) <= 1e-4;
    const bool reached_vehicle = common::finite(telemetry.local_z_m) && std::abs(telemetry.local_z_m - navigation_.target_z_m()) <= config_.target_tolerance_m;
    if (reached_setpoint && reached_vehicle) {phase_ = "hold"; phase_started_at_ = now;}
  } else if (phase_ == "hold") {
    if (phase_started_at_ && now - *phase_started_at_ >= config_.hold_seconds) {
      if (initialization_.lcp_runtime_healthy(now)) {navigation_.set_yaw_rad(0.0); navigation_.prepare_waypoints(); if (navigation_.start_next_waypoint()) {phase_ = "waypoint"; phase_started_at_ = now;} else {begin_landing(now);}}
      else {last_errors_.emplace_back("waiting for fresh LCP STATUS=2 before waypoint and yaw=0");}
    }
  } else if (phase_ == "waypoint") {
    if (navigation_.waypoint_reached(telemetry.local_x_m, telemetry.local_y_m)) {if (!navigation_.start_next_waypoint()) {begin_landing(now);} else {phase_ = "waypoint"; phase_started_at_ = now;}}
  } else if (phase_ == "landing") {
    const bool touchdown = telemetry.landed_state == common::MAV_LANDED_STATE_ON_GROUND && !common::stale(telemetry.landed_at, now, config_.landed_timeout_s) && common::finite(telemetry.local_z_m) && std::abs(telemetry.local_z_m - navigation_.origin_z_m()) <= config_.touchdown_z_tolerance_m;
    if (touchdown) {phase_ = "normal_disarm_pending"; phase_started_at_ = now;}
  } else if (phase_ == "normal_disarm_pending") {
    request_arm(now, false);
    if (!telemetry.armed) {result_ = abort_reason_ ? "ABORTED_SAFE" : "PASS"; phase_ = "finished"; phase_started_at_ = now;}
  }
}

/// 定时器入口：计算 dt、轮询服务、推进状态机并限频输出状态。
void MavrosNativeXYZNode::tick()
{
  const double now = monotonic_now(); double dt_s = 1.0 / config_.publish_rate_hz;
  if (last_tick_at_) {dt_s = std::max(0.001, std::min(now - *last_tick_at_, 0.25));}
  last_tick_at_ = now; poll_service_futures(now);
  if (!publish_enabled_) {monitor_tick(now);}
  else if (phase_ == "waiting_preflight" || phase_ == "setpoint_warmup" || phase_ == "position_stream_ready" || phase_ == "offboard_request_pending" || phase_ == "offboard_disarmed_pass" || phase_ == "arming_request_pending" || phase_ == "lcp_start_pending" || phase_ == "lcp_initializing" || phase_ == "lcp_ready") {prearm_control_tick(now, dt_s);}
  else if (phase_ == "climb" || phase_ == "hold" || phase_ == "waypoint" || phase_ == "lcp_hold" || phase_ == "landing" || phase_ == "normal_disarm_pending") {flight_tick(now, dt_s);}
  if (now - last_status_at_ >= options_.status_period || phase_ != last_phase_) {emit_status(now); last_status_at_ = now; last_phase_ = phase_;}
}

/// 将当前安全错误列表转义并编码为 JSON 数组。
std::string MavrosNativeXYZNode::errors_json(const std::vector<std::string> & errors) const
{
  std::ostringstream stream; stream << "[";
  for (std::size_t i = 0; i < errors.size(); ++i) {if (i) {stream << ",";} stream << "\"" << common::json_escape(errors[i]) << "\"";}
  stream << "]"; return stream.str();
}

/// 序列化最近 SetMode 请求的状态、应答或异常。
std::string MavrosNativeXYZNode::mode_event_json() const
{
  std::ostringstream s; s << "{\"status\":\"" << common::json_escape(last_mode_event_.status) << "\",\"mode\":" << optional_string_json(last_mode_event_.mode) << ",\"mode_sent\":" << optional_bool_json(last_mode_event_.mode_sent) << ",\"detail\":" << optional_string_json(last_mode_event_.detail) << ",\"monotonic_s\":" << optional_number_json(last_mode_event_.monotonic_s) << "}"; return s.str();
}

/// 序列化最近 CommandBool 请求的状态、应答或异常。
std::string MavrosNativeXYZNode::arm_event_json() const
{
  std::ostringstream s; s << "{\"status\":\"" << common::json_escape(last_arm_event_.status) << "\",\"value\":" << optional_bool_json(last_arm_event_.value) << ",\"success\":" << optional_bool_json(last_arm_event_.success) << ",\"result\":" << optional_int_json(last_arm_event_.result) << ",\"normal_command_only\":true,\"detail\":" << optional_string_json(last_arm_event_.detail) << ",\"monotonic_s\":" << optional_number_json(last_arm_event_.monotonic_s) << "}"; return s.str();
}

/// 序列化由 Initialization 管理的最近 LCP 服务事件。
std::string MavrosNativeXYZNode::lcp_event_json() const
{
  const auto & event = initialization_.lcp_start_event();
  std::ostringstream s; s << "{\"status\":\"" << common::json_escape(event.status) << "\",\"success\":" << optional_bool_json(event.success) << ",\"message\":" << optional_string_json(event.message) << ",\"monotonic_s\":" << optional_number_json(event.monotonic_s) << "}"; return s.str();
}

/// 生成兼容原日志 schema 的完整 JSONL 状态记录。
std::string MavrosNativeXYZNode::status_json(double now) const
{
  const auto & t = initialization_.telemetry();
  const auto artifact_path = artifact_log_.path();
  std::ostringstream s;
  s << "{\"schema\":\"px4.mavros_native_xyz.v1\",\"logged_at_utc\":\"" << common::utc_timestamp() << "\",\"phase\":\"" << common::json_escape(phase_) << "\",\"result\":\"" << common::json_escape(result_) << "\",\"errors\":" << errors_json(last_errors_) << ",\"abort_reason\":" << optional_string_json(abort_reason_)
    << ",\"waypoint\":";
  if (navigation_.waypoints().empty()) {s << "null";} else {
    const int index = navigation_.waypoint_index(); const auto & target = navigation_.waypoints().at(index >= 0 ? static_cast<std::size_t>(index) : 0U);
    s << "{\"index\":" << (index >= 0 ? index + 1 : 0) << ",\"count\":" << navigation_.waypoints().size() << ",\"target_x_m\":" << number_json(target.first) << ",\"target_y_m\":" << number_json(target.second) << "}";
  }
  s << ",\"control_capabilities\":{\"setpoint_publisher_created\":" << bool_json(static_cast<bool>(setpoint_publisher_)) << ",\"mode_client_created\":" << bool_json(static_cast<bool>(mode_client_)) << ",\"arming_client_created\":" << bool_json(static_cast<bool>(arm_client_)) << ",\"lcp_start_client_created\":true}"
    << ",\"setpoint\":";
  if (!navigation_.latched()) {s << "null";} else {
    const auto setpoint = navigation_.current();
    s << "{\"x_m\":" << number_json(setpoint.x_m) << ",\"y_m\":" << number_json(setpoint.y_m) << ",\"z_m\":" << number_json(setpoint.z_m) << ",\"origin_x_m\":" << number_json(navigation_.origin_x_m()) << ",\"origin_y_m\":" << number_json(navigation_.origin_y_m()) << ",\"origin_z_m\":" << number_json(navigation_.origin_z_m()) << ",\"target_x_m\":" << number_json(navigation_.target_x_m()) << ",\"target_y_m\":" << number_json(navigation_.target_y_m()) << ",\"target_z_m\":" << number_json(navigation_.target_z_m()) << ",\"vertical_rate_m_s\":" << number_json(setpoint.vertical_rate_m_s) << ",\"flow_effective\":" << bool_json(navigation_.flow_effective()) << "}";
  }
  s << ",\"flight_snapshot\":{\"vehicle\":{\"armed\":" << bool_json(t.armed) << ",\"mode\":\"" << common::json_escape(t.mode) << "\",\"connected\":" << bool_json(t.connected) << ",\"landed_state\":" << t.landed_state << "},\"local_xyz_m\":{\"x\":" << number_json(t.local_x_m) << ",\"y\":" << number_json(t.local_y_m) << ",\"z\":" << number_json(t.local_z_m) << "},\"local_velocity_m_s\":{\"x\":" << number_json(t.velocity_x_m_s) << ",\"y\":" << number_json(t.velocity_y_m_s) << ",\"z\":" << number_json(t.velocity_z_m_s) << "},\"relative_target_height_m\":" << number_json(config_.relative_z_m) << "}"
    << ",\"mode_service\":" << mode_event_json() << ",\"arming_service\":" << arm_event_json() << ",\"lcp_start_service\":" << lcp_event_json()
    << ",\"telemetry\":" << initialization_.telemetry_json(now)
    << ",\"audit\":{\"target_hardware\":\"PX4 FMUv6C.x / Pi 5 / MTF02P\",\"target_firmware\":\"PX4 1.17.0\",\"target_ros_mavros\":\"ROS 2 Jazzy / MAVROS 2.14.0\",\"confirmed_fcu_url\":\"" << common::json_escape(options_.confirmed_fcu_url) << "\",\"topics\":{\"state\":\"" << common::json_escape(options_.state_topic) << "\",\"sys_status\":\"" << common::json_escape(options_.sys_status_topic) << "\",\"battery\":\"" << common::json_escape(options_.battery_topic) << "\",\"landed\":\"" << common::json_escape(options_.extended_state_topic) << "\",\"local_pose\":\"" << common::json_escape(options_.local_pose_topic) << "\",\"local_velocity\":\"" << common::json_escape(options_.local_velocity_topic) << "\",\"estimator_status\":\"" << common::json_escape(options_.estimator_status_topic) << "\",\"range\":\"" << common::json_escape(options_.range_topic) << "\",\"optical_flow\":\"" << common::json_escape(options_.optical_flow_topic) << "\",\"setpoint\":\"" << common::json_escape(options_.setpoint_topic) << "\",\"lcp_status\":\"" << common::json_escape(options_.lcp_status_topic) << "\",\"lcp_odometry\":\"" << common::json_escape(options_.lcp_odometry_topic) << "},\"services\":{\"lcp_start_initialization\":\"" << common::json_escape(options_.lcp_start_service) << "\"},\"range_source_label\":\"" << common::json_escape(options_.range_source_label) << "\",\"optical_flow_source_label\":\"" << common::json_escape(options_.optical_flow_source_label) << "\",\"range_source_confirmed\":" << bool_json(options_.confirm_range_source) << ",\"optical_flow_source_confirmed\":" << bool_json(options_.confirm_optical_flow_source) << ",\"ignore_declared_min_range\":" << bool_json(options_.ignore_declared_min_range) << ",\"ack_range_below_declared_min\":" << bool_json(options_.ack_range_below_declared_min) << ",\"sensor_loss_grace_s\":" << number_json(config_.sensor_loss_grace_s) << ",\"lcp_status_timeout_s\":" << number_json(config_.lcp_status_timeout_s) << ",\"lcp_odometry_timeout_s\":" << number_json(config_.lcp_odometry_timeout_s) << ",\"lcp_ready_samples\":" << config_.lcp_ready_samples << ",\"lcp_unhealthy_hold_timeout_s\":" << number_json(config_.lcp_unhealthy_hold_timeout_s) << ",\"lcp_control_policy\":\"STATUS=3 during climb is ignored; after 10 s fixed-height hold, fresh STATUS=2 and odometry are required before waypoints; yaw=0\",\"artifact_log_path\":" << optional_string_json(artifact_path) << ",\"setpoint_frame\":\"ROS ENU PositionTarget on setpoint_raw/local; MAVROS converts to MAVLink LOCAL_NED\",\"setpoint_mav_frame_confirmed_local_ned\":" << bool_json(options_.confirm_setpoint_mav_frame_local_ned) << ",\"setpoint_policy\":\"full position hold (PX/PY/PZ + YAW); Z driven by quintic trajectory\",\"waypoint_policy\":\"four 0.5 m body-relative legs: forward, left, backward, right; final target returns to initial XY before landing\",\"waypoint_frame\":\"initial locked yaw in ROS ENU local XY\",\"waypoint_leg_m\":" << number_json(config_.waypoint_leg_m) << ",\"waypoint_max_speed_m_s\":" << number_json(config_.waypoint_max_speed_m_s) << ",\"waypoint_max_accel_m_s2\":" << number_json(config_.waypoint_max_accel_m_s2) << ",\"waypoint_tolerance_m\":" << number_json(config_.waypoint_tolerance_m) << ",\"parameter_writes\":false,\"force_arm_or_disarm\":false,\"px4_xy_fusion_evidence_label\":" << (options_.px4_xy_fusion_evidence_label.empty() ? "null" : "\"" + common::json_escape(options_.px4_xy_fusion_evidence_label) + "\"") << "}}";
  return s.str();
}

/// 生成便于终端观察的紧凑多行遥测摘要。
std::string MavrosNativeXYZNode::summary(double now) const
{
  const auto & t = initialization_.telemetry();
  /// 将可选样式的浮点遥测格式化为终端摘要文本。
  const auto value = [](double input) {if (!common::finite(input)) {return std::string("null");} std::ostringstream s; s << std::fixed << std::setprecision(3) << input; return s.str();};
  std::ostringstream s;
  s << "[" << std::fixed << std::setprecision(2) << now << "] phase=" << phase_ << " result=" << result_
    << "\n  conn=" << (t.connected ? "true" : "false") << " armed=" << (t.armed ? "true" : "false") << " mode='" << t.mode << "' sys_status=" << t.system_status << " landed=" << t.landed_state
    << "\n  battery=" << value(t.battery_voltage_v) << "V/" << value(t.battery_fraction) << " present=" << (t.battery_present ? "true" : "false")
    << "\n  local=(" << value(t.local_x_m) << "," << value(t.local_y_m) << "," << value(t.local_z_m) << ") vel=(" << value(t.velocity_x_m_s) << "," << value(t.velocity_y_m_s) << "," << value(t.velocity_z_m_s) << ")"
    << "\n  range=" << value(t.range_m) << " (decl " << value(t.range_min_m) << ".." << value(t.range_max_m) << ") fault=" << (initialization_.range_fault() ? *initialization_.range_fault() : "None")
    << "\n  flow_q=" << t.optical_flow_quality << " flow_dt_us=" << t.optical_flow_integration_time_us
    << "\n  lcp_io=status:" << options_.lcp_status_topic << " odom:" << options_.lcp_odometry_topic << " service:" << options_.lcp_start_service
    << "\n  lcp=status:" << (t.lcp_status ? std::to_string(*t.lcp_status) : "None") << " samples:" << t.lcp_healthy_samples << " xy=(" << value(t.lcp_x_m) << "," << value(t.lcp_y_m) << ") yaw=" << value(t.lcp_yaw_rad) << " init=" << t.lcp_init_request_state;
  if (navigation_.latched()) {const auto sp = navigation_.current(); s << "\n  setpoint=(" << value(sp.x_m) << "," << value(sp.y_m) << "," << value(sp.z_m) << ") z0=" << value(navigation_.origin_z_m()) << " target_z=" << value(navigation_.target_z_m()) << " vz=" << value(sp.vertical_rate_m_s) << " flow_eff=" << (navigation_.flow_effective() ? "true" : "false");}
  if (!navigation_.waypoints().empty()) {const int i = navigation_.waypoint_index(); const auto & target = navigation_.waypoints().at(i >= 0 ? static_cast<std::size_t>(i) : 0U); s << "\n  waypoint=" << (i >= 0 ? i + 1 : 0) << "/" << navigation_.waypoints().size() << " target=(" << value(target.first) << "," << value(target.second) << ")";}
  if (abort_reason_) {s << "\n  abort_reason=" << *abort_reason_;}
  s << "\n  errors: ["; for (std::size_t i = 0; i < last_errors_.size(); ++i) {if (i) {s << ", ";} s << last_errors_[i];} s << "]";
  return s.str();
}

/// 根据输出格式写入 artifact，并始终在终端打印摘要。
void MavrosNativeXYZNode::emit_status(double now)
{
  const auto text = summary(now);
  artifact_log_.write(options_.output == "jsonl" ? status_json(now) : text);
  std::cout << text << std::endl;
}

}  // namespace mavros_xyz_position_offboard::offboard
