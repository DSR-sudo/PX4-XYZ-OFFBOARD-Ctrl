#include "mavros_xyz_position_offboard/initialization/initialization.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <functional>
#include <future>
#include <iomanip>
#include <sstream>
#include <utility>

#include "mavros_xyz_position_offboard/common/artifact_log.hpp"

namespace mavros_xyz_position_offboard::initialization
{
namespace
{
/// 返回与 ROS 时间无关的单调秒计时，用于遥测年龄判断。
double monotonic_now()
{
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

/// 将有限浮点数编码为 JSON 数字，非有限值编码为 null。
std::string number_json(double value)
{
  if (!common::finite(value)) {return "null";}
  std::ostringstream stream;
  stream << std::setprecision(12) << value;
  return stream.str();
}

/// 将可选浮点数编码为 JSON 数字或 null。
std::string optional_number_json(const std::optional<double> & value)
{
  return value ? number_json(*value) : "null";
}

/// 将可选字符串安全编码为 JSON 字符串或 null。
std::string optional_string_json(const std::optional<std::string> & value)
{
  return value ? "\"" + common::json_escape(*value) + "\"" : "null";
}

/// 将布尔值转换为 JSON 字面量。
std::string bool_json(bool value) {return value ? "true" : "false";}

}  // namespace

/// 锁存测距故障原因并构造拒绝结果。
common::RangeResult RangeGuard::fault(const std::string & reason)
{
  fault_reason_ = reason;
  return {false, reason};
}

/// 用连续稳定样本确认测距故障已经恢复。
common::RangeResult RangeGuard::recover(double value_m, double now)
{
  if (candidate_m_ && std::abs(value_m - *candidate_m_) <= config_.jump_settle_tolerance_m) {
    ++candidate_count_;
    *candidate_m_ = (*candidate_m_ * static_cast<double>(candidate_count_ - 1) + value_m) /
      static_cast<double>(candidate_count_);
  } else {
    candidate_m_ = value_m;
    candidate_count_ = 1;
  }
  if (candidate_count_ < config_.jump_recovery_samples) {
    return {false, fault_reason_.value_or("downward range recovering")};
  }
  last_accepted_m_ = candidate_m_;
  last_accepted_at_ = now;
  fault_reason_.reset();
  candidate_m_.reset();
  candidate_count_ = 0;
  return {true, std::nullopt};
}

/// 执行声明范围、配置范围、跳变和稳定恢复策略。
common::RangeResult RangeGuard::observe(double value_m, double declared_min_m, double declared_max_m, double now)
{
  if (!common::finite(value_m) || !common::finite(declared_min_m) || !common::finite(declared_max_m)) {
    return fault("downward range or declared limits are non-finite");
  }
  if (declared_min_m < 0.0 || declared_max_m <= declared_min_m) {
    return fault("downward range declared limits are invalid");
  }
  const double low = config_.ignore_declared_min_range ? config_.configured_min_range_m :
    std::max(declared_min_m, config_.configured_min_range_m);
  const double high = std::min(declared_max_m, config_.configured_max_range_m);
  if (high <= low) {return fault("downward range declared and configured limits do not overlap");}
  if (value_m < low - config_.range_boundary_tolerance_m || value_m > high + config_.range_boundary_tolerance_m) {
    candidate_m_.reset();
    candidate_count_ = 0;
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3) << "downward range " << value_m
           << " m outside declared/accepted interval [" << low << ", " << high << "] m";
    return fault(stream.str());
  }
  if (fault_reason_) {return recover(value_m, now);}
  if (last_accepted_m_ && last_accepted_at_) {
    const double dt_s = now - *last_accepted_at_;
    if (dt_s < 0.0) {return fault("monotonic time moved backwards");}
    if (dt_s <= config_.jump_window_s && std::abs(value_m - *last_accepted_m_) > config_.max_range_jump_m) {
      candidate_m_ = value_m;
      candidate_count_ = 1;
      std::ostringstream stream;
      stream << std::fixed << std::setprecision(3) << "downward range jump exceeds " << config_.max_range_jump_m << " m";
      return fault(stream.str());
    }
  }
  last_accepted_m_ = value_m;
  last_accepted_at_ = now;
  return {true, std::nullopt};
}

/// 建立所有输入订阅，保留原 QoS，并创建 LCP 初始化客户端。
Initialization::Initialization(
  rclcpp::Node & node, const common::AppOptions & options, const common::SafetyConfig & config)
: node_(node), options_(options), config_(config), range_guard_(config)
{
  const auto sensor_qos = rclcpp::SensorDataQoS();
  const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
  const auto lcp_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  state_subscription_ = node_.create_subscription<mavros_msgs::msg::State>(
    options_.state_topic, state_qos, std::bind(&Initialization::state_callback, this, std::placeholders::_1));
  sys_status_subscription_ = node_.create_subscription<mavros_msgs::msg::SysStatus>(
    options_.sys_status_topic, sensor_qos, std::bind(&Initialization::sys_status_callback, this, std::placeholders::_1));
  battery_subscription_ = node_.create_subscription<sensor_msgs::msg::BatteryState>(
    options_.battery_topic, sensor_qos, std::bind(&Initialization::battery_callback, this, std::placeholders::_1));
  extended_state_subscription_ = node_.create_subscription<mavros_msgs::msg::ExtendedState>(
    options_.extended_state_topic, state_qos, std::bind(&Initialization::extended_state_callback, this, std::placeholders::_1));
  local_pose_subscription_ = node_.create_subscription<geometry_msgs::msg::PoseStamped>(
    options_.local_pose_topic, sensor_qos, std::bind(&Initialization::local_pose_callback, this, std::placeholders::_1));
  local_velocity_subscription_ = node_.create_subscription<geometry_msgs::msg::TwistStamped>(
    options_.local_velocity_topic, sensor_qos, std::bind(&Initialization::local_velocity_callback, this, std::placeholders::_1));
  estimator_subscription_ = node_.create_subscription<mavros_msgs::msg::EstimatorStatus>(
    options_.estimator_status_topic, sensor_qos, std::bind(&Initialization::estimator_callback, this, std::placeholders::_1));
  range_subscription_ = node_.create_subscription<sensor_msgs::msg::Range>(
    options_.range_topic, sensor_qos, std::bind(&Initialization::range_callback, this, std::placeholders::_1));
  optical_flow_subscription_ = node_.create_subscription<mavros_msgs::msg::OpticalFlowRad>(
    options_.optical_flow_topic, sensor_qos, std::bind(&Initialization::optical_flow_callback, this, std::placeholders::_1));
  lcp_status_subscription_ = node_.create_subscription<std_msgs::msg::UInt8>(
    options_.lcp_status_topic, lcp_qos, std::bind(&Initialization::lcp_status_callback, this, std::placeholders::_1));
  lcp_odometry_subscription_ = node_.create_subscription<nav_msgs::msg::Odometry>(
    options_.lcp_odometry_topic, lcp_qos, std::bind(&Initialization::lcp_odometry_callback, this, std::placeholders::_1));
  lcp_start_client_ = node_.create_client<std_srvs::srv::Trigger>(options_.lcp_start_service);
}

/// 用一条 MAVROS State 消息更新连接、模式和解锁信息。
void Initialization::update_state(bool connected, bool armed, const std::string & mode, int system_status, double now)
{
  telemetry_.state_at = now; telemetry_.connected = connected; telemetry_.armed = armed;
  telemetry_.mode = mode; telemetry_.system_status = system_status;
}

/// 用 SYS_STATUS 位掩码更新传感器健康遥测。
void Initialization::update_sys_status(std::uint32_t present, std::uint32_t enabled, std::uint32_t health, double now)
{
  telemetry_.sys_status_at = now; telemetry_.sensors_present = present; telemetry_.sensors_enabled = enabled; telemetry_.sensors_health = health;
}

/// 记录电池状态及其单调时间戳。
void Initialization::update_battery(bool present, double voltage_v, double fraction, double now)
{
  telemetry_.battery_at = now; telemetry_.battery_present = present; telemetry_.battery_voltage_v = voltage_v; telemetry_.battery_fraction = fraction;
}

/// 记录 MAVROS ExtendedState 的落地状态。
void Initialization::update_landed(int landed_state, double now) {telemetry_.landed_at = now; telemetry_.landed_state = landed_state;}

/// 记录本地位置、姿态及其接收时间。
void Initialization::update_local_pose(
  double x_m, double y_m, double z_m, const common::Quaternion & orientation, double now,
  std::optional<common::RosTimestamp> stamp)
{
  telemetry_.local_pose_at = now; telemetry_.local_pose_stamp = stamp; telemetry_.local_x_m = x_m; telemetry_.local_y_m = y_m; telemetry_.local_z_m = z_m; telemetry_.orientation = orientation;
}

/// 记录本地线速度及其接收时间。
void Initialization::update_local_velocity(double x_m_s, double y_m_s, double z_m_s, double now)
{
  telemetry_.local_velocity_at = now; telemetry_.velocity_x_m_s = x_m_s; telemetry_.velocity_y_m_s = y_m_s; telemetry_.velocity_z_m_s = z_m_s;
}

/// 记录预检和飞行期使用的 MAVROS 估计器标志。
void Initialization::update_estimator(
  bool attitude_valid, bool velocity_horiz_valid, bool velocity_vert_valid, bool pos_horiz_rel_valid,
  bool pos_horiz_abs_valid, bool pos_vert_abs_valid, bool pos_vert_agl_valid, bool const_pos_mode,
  bool gps_glitch, bool accel_error, double now)
{
  telemetry_.estimator_at = now; telemetry_.estimator_attitude_valid = attitude_valid;
  telemetry_.estimator_velocity_horiz_valid = velocity_horiz_valid; telemetry_.estimator_velocity_vert_valid = velocity_vert_valid;
  telemetry_.estimator_pos_horiz_rel_valid = pos_horiz_rel_valid; telemetry_.estimator_pos_horiz_abs_valid = pos_horiz_abs_valid;
  telemetry_.estimator_pos_vert_abs_valid = pos_vert_abs_valid; telemetry_.estimator_pos_vert_agl_valid = pos_vert_agl_valid;
  telemetry_.estimator_const_pos_mode = const_pos_mode; telemetry_.estimator_gps_glitch = gps_glitch; telemetry_.estimator_accel_error = accel_error;
}

/// 记录 Range 消息并交由 RangeGuard 判定有效性。
common::RangeResult Initialization::update_range(
  double range_m, double declared_min_m, double declared_max_m, double now,
  std::optional<common::RosTimestamp> stamp)
{
  telemetry_.range_at = now; telemetry_.range_stamp = stamp; telemetry_.range_m = range_m; telemetry_.range_min_m = declared_min_m; telemetry_.range_max_m = declared_max_m;
  return range_guard_.observe(range_m, declared_min_m, declared_max_m, now);
}

/// 记录 OpticalFlowRad 的积分、质量和距离字段。
void Initialization::update_optical_flow(
  std::uint32_t integration_time_us, double integrated_x_rad, double integrated_y_rad, int quality,
  double distance_m, std::uint32_t distance_delta_us, double temperature_c, double now)
{
  telemetry_.optical_flow_at = now; telemetry_.optical_flow_integration_time_us = integration_time_us;
  telemetry_.optical_flow_integrated_x_rad = integrated_x_rad; telemetry_.optical_flow_integrated_y_rad = integrated_y_rad;
  telemetry_.optical_flow_quality = quality; telemetry_.optical_flow_distance_m = distance_m;
  telemetry_.optical_flow_distance_delta_us = distance_delta_us; telemetry_.optical_flow_temperature_c = temperature_c;
}

/// 记录 LCP 状态，并仅累计初始化基线之后的 STATUS=2 样本。
void Initialization::update_lcp_status(int status, double now)
{
  telemetry_.lcp_status = status; telemetry_.lcp_status_at = now; ++telemetry_.lcp_status_sequence;
  if (status != 2) {telemetry_.lcp_healthy_samples = 0;}
  else if (telemetry_.lcp_status_sequence > telemetry_.lcp_init_status_sequence_baseline) {++telemetry_.lcp_healthy_samples;}
}

/// 记录 LCP 平面里程计，同时递增新鲜度序列号。
void Initialization::update_lcp_odometry(double x_m, double y_m, double yaw_rad, double now)
{
  telemetry_.lcp_odometry_at = now; ++telemetry_.lcp_odometry_sequence;
  telemetry_.lcp_x_m = x_m; telemetry_.lcp_y_m = y_m; telemetry_.lcp_yaw_rad = yaw_rad;
}

/// 锁存 LCP 序列基线，确保旧健康消息不能通过新初始化。
void Initialization::begin_lcp_initialization(double now)
{
  telemetry_.lcp_init_status_sequence_baseline = telemetry_.lcp_status_sequence;
  telemetry_.lcp_init_odometry_sequence_baseline = telemetry_.lcp_odometry_sequence;
  telemetry_.lcp_healthy_samples = 0; telemetry_.lcp_init_requested_at = now; telemetry_.lcp_init_response_at.reset();
  telemetry_.lcp_init_response_message.reset(); telemetry_.lcp_init_failure_reason.reset();
}

/// 清除初始化请求结果，并把当前序列号作为新的基线。
void Initialization::reset_lcp_initialization()
{
  telemetry_.lcp_healthy_samples = 0; telemetry_.lcp_init_status_sequence_baseline = telemetry_.lcp_status_sequence;
  telemetry_.lcp_init_odometry_sequence_baseline = telemetry_.lcp_odometry_sequence; telemetry_.lcp_init_request_state = "not_requested";
  telemetry_.lcp_init_requested_at.reset(); telemetry_.lcp_init_response_at.reset(); telemetry_.lcp_init_response_message.reset(); telemetry_.lcp_init_failure_reason.reset();
}

/// 更新 LCP 服务状态机并保存响应或失败文本。
void Initialization::update_lcp_init_state(
  const std::string & state, double now, const std::optional<std::string> & message,
  const std::optional<std::string> & failure_reason)
{
  telemetry_.lcp_init_request_state = state;
  if (message) {telemetry_.lcp_init_response_message = message;}
  if (failure_reason) {telemetry_.lcp_init_failure_reason = failure_reason;}
  if (state == "accepted" || state == "failed") {telemetry_.lcp_init_response_at = now;}
}

/// 仅在服务就绪且没有已有终态时发送一次 LCP Trigger 请求。
void Initialization::request_lcp_start(double now)
{
  if (lcp_start_future_) {return;}
  if (telemetry_.lcp_init_request_state == "request_sent" || telemetry_.lcp_init_request_state == "accepted" || telemetry_.lcp_init_request_state == "failed") {return;}
  if (!lcp_start_client_) {
    update_lcp_init_state("failed", now, std::nullopt, "LCP initialization client was not created");
    return;
  }
  if (!lcp_start_client_->service_is_ready()) {
    update_lcp_init_state("waiting_service", now);
    lcp_start_event_ = {"service_not_ready", std::nullopt, std::nullopt, now};
    return;
  }
  try {
    begin_lcp_initialization(now); update_lcp_init_state("request_sent", now);
    lcp_start_future_ = lcp_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    lcp_start_future_started_at_ = now; lcp_start_event_ = {"request_sent", std::nullopt, std::nullopt, now};
  } catch (const std::exception & error) {
    const std::string detail = error.what();
    update_lcp_init_state("failed", now, std::nullopt, "LCP initialization request exception: " + detail);
    lcp_start_event_ = {"exception", std::nullopt, detail, now};
  }
}

/// 将 LCP future 的结果、异常或超时归并进遥测状态机。
void Initialization::poll_lcp_start(double now, double timeout_s)
{
  if (!lcp_start_future_) {return;}
  if (lcp_start_future_->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    try {
      const auto response = lcp_start_future_->get();
      const std::string message = response->message;
      if (response->success) {update_lcp_init_state("accepted", now, message);}
      else {update_lcp_init_state("failed", now, message, "LCP initialization service rejected request: " + message);}
      lcp_start_event_ = {"response", response->success, message, now};
    } catch (const std::exception & error) {
      const std::string detail = error.what();
      update_lcp_init_state("failed", now, std::nullopt, "LCP initialization service exception: " + detail);
      lcp_start_event_ = {"exception", std::nullopt, detail, now};
    }
    lcp_start_future_.reset(); lcp_start_future_started_at_.reset();
  } else if (lcp_start_future_started_at_ && now - *lcp_start_future_started_at_ > timeout_s) {
    lcp_start_future_.reset(); lcp_start_future_started_at_.reset();
    update_lcp_init_state("failed", now, std::nullopt, "LCP initialization service request timed out");
    lcp_start_event_ = {"timeout", std::nullopt, std::nullopt, now};
  }
}

/// 丢弃正在等待的 LCP future，供预检候选重置使用。
void Initialization::cancel_lcp_start() {lcp_start_future_.reset(); lcp_start_future_started_at_.reset();}

/// 只检查 LCP 驱动服务允许清空地图的飞控地面条件，不依赖飞行电池和位置传感器。
std::vector<std::string> Initialization::lcp_start_prerequisite_errors(double now) const
{
  std::vector<std::string> errors;
  if (common::stale(telemetry_.state_at, now, config_.state_timeout_s)) {
    errors.emplace_back("MAVROS state/heartbeat stale or unavailable for LCP initialization");
  } else if (!telemetry_.connected) {
    errors.emplace_back("MAVROS is not connected to the flight controller for LCP initialization");
  } else if (telemetry_.armed) {
    errors.emplace_back("vehicle is armed; LCP initialization requires disarmed state");
  }
  if (common::stale(telemetry_.landed_at, now, config_.landed_timeout_s)) {
    errors.emplace_back("landed state stale or unavailable for LCP initialization");
  } else if (telemetry_.landed_state != common::MAV_LANDED_STATE_ON_GROUND) {
    errors.emplace_back("vehicle is not confirmed on ground for LCP initialization");
  }
  return errors;
}

/// 判定 STATUS=2、里程计和时间戳在飞行期均有效。
bool Initialization::lcp_runtime_healthy(double now) const
{
  return telemetry_.lcp_status && *telemetry_.lcp_status == 2 &&
    !common::stale(telemetry_.lcp_status_at, now, config_.lcp_status_timeout_s) &&
    !common::stale(telemetry_.lcp_odometry_at, now, config_.lcp_odometry_timeout_s) &&
    common::finite(telemetry_.lcp_x_m) && common::finite(telemetry_.lcp_y_m) && common::finite(telemetry_.lcp_yaw_rad);
}

/// 判定服务已接受且初始化后累积了足量的新鲜健康样本。
bool Initialization::lcp_ready(double now) const
{
  return telemetry_.lcp_init_request_state == "accepted" && lcp_runtime_healthy(now) &&
    telemetry_.lcp_healthy_samples >= config_.lcp_ready_samples &&
    telemetry_.lcp_status_sequence > telemetry_.lcp_init_status_sequence_baseline &&
    telemetry_.lcp_odometry_sequence > telemetry_.lcp_init_odometry_sequence_baseline;
}

/// 构造可审计的 LCP 不健康或未就绪原因列表。
std::vector<std::string> Initialization::lcp_errors(double now, bool require_samples) const
{
  std::vector<std::string> errors;
  if (telemetry_.lcp_init_request_state == "failed") {errors.emplace_back(telemetry_.lcp_init_failure_reason.value_or("LCP initialization service failed"));}
  if (!telemetry_.lcp_status) {errors.emplace_back("LCP status unavailable");}
  else if (common::stale(telemetry_.lcp_status_at, now, config_.lcp_status_timeout_s)) {errors.emplace_back("LCP status stale or unavailable");}
  else if (*telemetry_.lcp_status != 2) {errors.emplace_back("LCP status is " + std::to_string(*telemetry_.lcp_status) + ", expected 2");}
  if (common::stale(telemetry_.lcp_odometry_at, now, config_.lcp_odometry_timeout_s)) {errors.emplace_back("LCP odometry stale or unavailable");}
  else if (!common::finite(telemetry_.lcp_x_m) || !common::finite(telemetry_.lcp_y_m) || !common::finite(telemetry_.lcp_yaw_rad)) {errors.emplace_back("LCP odometry is non-finite");}
  if (require_samples && telemetry_.lcp_healthy_samples < config_.lcp_ready_samples) {
    errors.emplace_back("LCP healthy samples " + std::to_string(telemetry_.lcp_healthy_samples) + "/" + std::to_string(config_.lcp_ready_samples));
  }
  return errors;
}

/// 校验心跳、FCU 连接、系统状态和传感器健康位掩码。
std::vector<std::string> Initialization::connection_errors(double now, bool require_standby)
{
  std::vector<std::string> errors;
  if (common::stale(telemetry_.state_at, now, config_.state_timeout_s)) {errors.emplace_back("MAVROS state/heartbeat stale or unavailable");}
  else if (!telemetry_.connected) {errors.emplace_back("MAVROS is not connected to the flight controller");}
  else if (require_standby && telemetry_.system_status != common::MAV_STATE_STANDBY && telemetry_.system_status != common::MAV_STATE_ACTIVE) {
    const std::uint32_t unhealthy = telemetry_.sensors_enabled & (~telemetry_.sensors_health) & ~common::MAV_SYS_STATUS_ACCEPTABLE_UNHEALTHY_MASK;
    const bool status_ok = !common::stale(telemetry_.sys_status_at, now, config_.sys_status_timeout_s) && unhealthy == 0;
    if (!status_ok) {errors.emplace_back("PX4 heartbeat system_status is " + std::to_string(telemetry_.system_status) + ", expected STANDBY (3)");}
  }
  if (common::stale(telemetry_.sys_status_at, now, config_.sys_status_timeout_s)) {errors.emplace_back("MAVROS SYS_STATUS stale or unavailable");}
  else {
    const bool prearm_ok = (telemetry_.sensors_health & common::MAV_SYS_STATUS_PREARM_CHECK) != 0;
    const std::uint32_t unhealthy = telemetry_.sensors_enabled & (~telemetry_.sensors_health) & ~common::MAV_SYS_STATUS_ACCEPTABLE_UNHEALTHY_MASK;
    if (unhealthy) {
      std::ostringstream stream; stream << "PX4 SYS_STATUS enabled-but-unhealthy mask is 0x" << std::hex << std::setw(8) << std::setfill('0') << unhealthy;
      errors.emplace_back(stream.str());
    } else if (!prearm_ok) {telemetry_.prearm_bit_advisory = true;}
  }
  return errors;
}

/// 校验电池新鲜度、电压及预检阶段所需的电量比例。
std::vector<std::string> Initialization::battery_errors(double now, bool in_flight) const
{
  if (common::stale(telemetry_.battery_at, now, config_.battery_timeout_s)) {return {"battery telemetry stale or unavailable"};}
  std::vector<std::string> errors;
  if (!telemetry_.battery_present) {errors.emplace_back("battery telemetry reports no battery present");}
  if (!common::finite(telemetry_.battery_voltage_v) || telemetry_.battery_voltage_v < config_.min_battery_voltage_v) {
    std::ostringstream stream; stream << std::fixed << std::setprecision(2) << "battery voltage invalid or below " << config_.min_battery_voltage_v << " V"; errors.emplace_back(stream.str());
  }
  if (!in_flight && (!common::finite(telemetry_.battery_fraction) || telemetry_.battery_fraction < config_.min_battery_fraction)) {
    std::ostringstream stream; stream << "battery fraction invalid or below " << std::lround(config_.min_battery_fraction * 100.0) << "%"; errors.emplace_back(stream.str());
  }
  return errors;
}

/// 校验本地位姿/速度，并根据阶段应用静止、爬升或飞行限速。
std::vector<std::string> Initialization::pose_velocity_errors(double now, bool preflight, bool climbing) const
{
  std::vector<std::string> errors;
  if (common::stale(telemetry_.local_pose_at, now, config_.local_pose_timeout_s)) {errors.emplace_back("local pose stale or unavailable");}
  else {
    if (!common::finite(telemetry_.local_x_m) || !common::finite(telemetry_.local_y_m) || !common::finite(telemetry_.local_z_m)) {errors.emplace_back("local XYZ pose is non-finite");}
    try {static_cast<void>(common::normalize_quaternion(telemetry_.orientation.x, telemetry_.orientation.y, telemetry_.orientation.z, telemetry_.orientation.w));}
    catch (const std::invalid_argument & error) {errors.emplace_back(error.what());}
  }
  if (common::stale(telemetry_.local_velocity_at, now, config_.local_velocity_timeout_s)) {errors.emplace_back("local velocity stale or unavailable");}
  else if (!common::finite(telemetry_.velocity_x_m_s) || !common::finite(telemetry_.velocity_y_m_s) || !common::finite(telemetry_.velocity_z_m_s)) {errors.emplace_back("local XYZ velocity is non-finite");}
  else {
    const double horizontal_speed = std::hypot(telemetry_.velocity_x_m_s, telemetry_.velocity_y_m_s);
    const double horizontal_limit = preflight ? config_.max_preflight_horizontal_speed_m_s :
      (climbing ? config_.climb_horizontal_speed_limit_m_s : config_.max_flight_horizontal_speed_m_s);
    const double vertical_limit = preflight ? config_.max_preflight_vertical_speed_m_s : config_.max_flight_vertical_speed_m_s;
    if (horizontal_speed > horizontal_limit) {std::ostringstream s; s << std::fixed << std::setprecision(2) << "horizontal speed exceeds " << horizontal_limit << " m/s"; errors.emplace_back(s.str());}
    if (std::abs(telemetry_.velocity_z_m_s) > vertical_limit) {std::ostringstream s; s << std::fixed << std::setprecision(2) << "vertical speed exceeds " << vertical_limit << " m/s"; errors.emplace_back(s.str());}
  }
  return errors;
}

/// 校验 MAVROS 可见的估计器有效性和故障位。
std::vector<std::string> Initialization::estimator_errors(double now) const
{
  if (common::stale(telemetry_.estimator_at, now, config_.estimator_timeout_s)) {return {"MAVROS estimator status stale or unavailable"};}
  std::vector<std::string> errors;
  if (!telemetry_.estimator_attitude_valid) {errors.emplace_back("estimator attitude-valid flag is false");}
  if (!telemetry_.estimator_velocity_horiz_valid) {errors.emplace_back("estimator horizontal-velocity-valid flag is false");}
  if (!telemetry_.estimator_velocity_vert_valid) {errors.emplace_back("estimator vertical-velocity-valid flag is false");}
  if (!telemetry_.estimator_pos_horiz_rel_valid && !telemetry_.estimator_pos_horiz_abs_valid) {errors.emplace_back("estimator has no valid relative or absolute horizontal position");}
  if (!telemetry_.estimator_pos_vert_abs_valid && !telemetry_.estimator_pos_vert_agl_valid) {errors.emplace_back("estimator has no valid absolute or AGL vertical position");}
  if (telemetry_.estimator_gps_glitch) {errors.emplace_back("estimator GPS-glitch flag is set");}
  if (telemetry_.estimator_accel_error) {errors.emplace_back("estimator accelerometer-error flag is set");}
  return errors;
}

/// 校验测距与光流来源确认、消息新鲜度和数据质量。
std::vector<std::string> Initialization::range_flow_errors(double now) const
{
  std::vector<std::string> errors;
  if (!config_.range_source_confirmed) {errors.emplace_back("range topic source/direction is not explicitly confirmed");}
  if (common::stale(telemetry_.range_at, now, config_.range_timeout_s)) {errors.emplace_back("downward range stale or unavailable");}
  else if (range_guard_.fault_reason()) {errors.emplace_back(*range_guard_.fault_reason());}
  if (!config_.optical_flow_source_confirmed) {errors.emplace_back("optical-flow topic source is not explicitly confirmed");}
  if (common::stale(telemetry_.optical_flow_at, now, config_.optical_flow_timeout_s)) {errors.emplace_back("optical-flow data stale or unavailable");}
  else {
    if (telemetry_.optical_flow_integration_time_us == 0) {errors.emplace_back("optical-flow integration time is invalid");}
    if (!common::finite(telemetry_.optical_flow_integrated_x_rad) || !common::finite(telemetry_.optical_flow_integrated_y_rad)) {errors.emplace_back("optical-flow integrated pixel angles are non-finite");}
    if (telemetry_.optical_flow_quality < config_.min_optical_flow_quality) {errors.emplace_back("optical-flow quality below " + std::to_string(config_.min_optical_flow_quality));}
  }
  return errors;
}

/// 聚合全部地面预检门禁，并检查 local Z 与测距一致性。
std::vector<std::string> Initialization::preflight_errors(double now)
{
  auto errors = connection_errors(now, true);
  if (telemetry_.armed) {errors.emplace_back("vehicle is armed; preflight requires disarmed state");}
  if (common::stale(telemetry_.landed_at, now, config_.landed_timeout_s)) {errors.emplace_back("landed state stale or unavailable");}
  else if (telemetry_.landed_state != common::MAV_LANDED_STATE_ON_GROUND) {errors.emplace_back("vehicle is not confirmed on ground");}
  /// 将一类门禁错误追加到预检汇总列表。
  const auto append = [&errors](const std::vector<std::string> & more) {errors.insert(errors.end(), more.begin(), more.end());};
  append(battery_errors(now)); append(pose_velocity_errors(now, true)); append(estimator_errors(now)); append(range_flow_errors(now));
  if (!common::stale(telemetry_.local_pose_at, now, config_.local_pose_timeout_s) && !common::stale(telemetry_.range_at, now, config_.range_timeout_s) &&
      common::finite(telemetry_.local_z_m) && common::finite(telemetry_.range_m) && !range_guard_.fault_reason()) {
    const double mismatch = std::abs(telemetry_.local_z_m - telemetry_.range_m);
    if (mismatch > 0.30) {
      std::ostringstream stream; stream << std::fixed << std::setprecision(3) << "preflight Z mismatch: local_z=" << telemetry_.local_z_m << " m vs range=" << telemetry_.range_m << " m (delta " << mismatch << " m > 0.30); EKF height reference is inconsistent, do not take off";
      errors.emplace_back(stream.str());
    }
  }
  return errors;
}

/// 聚合飞行期门禁，并检查相对漂移和 OFFBOARD 状态。
std::vector<std::string> Initialization::flight_errors(
  double now, double commanded_x_m, double commanded_y_m, bool require_offboard, bool at_hover)
{
  auto errors = connection_errors(now, false);
  if (!telemetry_.armed) {errors.emplace_back("vehicle unexpectedly disarmed during flight phase");}
  std::string upper_mode = telemetry_.mode;
  std::transform(upper_mode.begin(), upper_mode.end(), upper_mode.begin(), [](unsigned char c) {return static_cast<char>(std::toupper(c));});
  if (require_offboard && upper_mode != "OFFBOARD") {errors.emplace_back("OFFBOARD mode is not confirmed");}
  /// 将一类门禁错误追加到飞行期汇总列表。
  const auto append = [&errors](const std::vector<std::string> & more) {errors.insert(errors.end(), more.begin(), more.end());};
  append(battery_errors(now, true)); append(pose_velocity_errors(now, false, !at_hover)); append(estimator_errors(now)); append(range_flow_errors(now));
  const double baseline_x = drift_baseline_x_m_.value_or(commanded_x_m);
  const double baseline_y = drift_baseline_y_m_.value_or(commanded_y_m);
  if (common::finite(baseline_x) && common::finite(baseline_y) && common::finite(telemetry_.local_x_m) && common::finite(telemetry_.local_y_m)) {
    const double drift = std::hypot(telemetry_.local_x_m - baseline_x, telemetry_.local_y_m - baseline_y);
    const double limit = at_hover ? config_.max_flight_horizontal_drift_m : config_.climb_horizontal_drift_limit_m;
    if (drift > limit) {std::ostringstream s; s << std::fixed << std::setprecision(2) << "horizontal drift exceeds " << limit << " m"; errors.emplace_back(s.str());}
  }
  return errors;
}

/// 根据高度、估计器定位和质量判断光流是否有效。
bool Initialization::optical_flow_effective(double now) const
{
  return !common::stale(telemetry_.range_at, now, config_.range_timeout_s) && common::finite(telemetry_.range_m) && telemetry_.range_m >= config_.flow_effective_min_height_m &&
    !common::stale(telemetry_.estimator_at, now, config_.estimator_timeout_s) && (telemetry_.estimator_pos_horiz_rel_valid || telemetry_.estimator_pos_horiz_abs_valid) &&
    !common::stale(telemetry_.optical_flow_at, now, config_.optical_flow_timeout_s) && telemetry_.optical_flow_quality >= config_.flow_effective_min_quality;
}

/// 复制本周期遥测，并一次性计算预检、飞行期和 LCP 派生健康结论。
HealthSnapshot Initialization::health_snapshot(
  double now, bool in_flight, double commanded_x_m, double commanded_y_m,
  bool require_offboard, bool at_hover)
{
  HealthSnapshot snapshot;
  snapshot.telemetry = telemetry_;
  snapshot.preflight_errors = preflight_errors(now);
  if (in_flight || telemetry_.armed) {
    snapshot.flight_errors = flight_errors(
      now, commanded_x_m, commanded_y_m, require_offboard, at_hover);
  }
  snapshot.lcp_healthy = lcp_runtime_healthy(now);
  snapshot.lcp_ready = lcp_ready(now);
  return snapshot;
}

/// 将当前有限的本地 XY 保存为漂移比较基准。
void Initialization::seed_drift_baseline(double now)
{
  if (common::finite(telemetry_.local_x_m) && common::finite(telemetry_.local_y_m)) {drift_baseline_x_m_ = telemetry_.local_x_m; drift_baseline_y_m_ = telemetry_.local_y_m; drift_baseline_at_ = now;}
}

/// 转发 MAVROS State 回调中的连接、模式和解锁信息。
void Initialization::state_callback(const mavros_msgs::msg::State::SharedPtr message) {update_state(message->connected, message->armed, message->mode, message->system_status, monotonic_now());}
/// 转发 MAVROS SysStatus 回调中的传感器位掩码。
void Initialization::sys_status_callback(const mavros_msgs::msg::SysStatus::SharedPtr message) {update_sys_status(message->sensors_present, message->sensors_enabled, message->sensors_health, monotonic_now());}
/// 转发 BatteryState 的存在、电压和百分比字段。
void Initialization::battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr message) {update_battery(message->present, message->voltage, message->percentage, monotonic_now());}
/// 转发 MAVROS ExtendedState 的落地状态。
void Initialization::extended_state_callback(const mavros_msgs::msg::ExtendedState::SharedPtr message) {update_landed(message->landed_state, monotonic_now());}
/// 提取 PoseStamped 的位置和四元数并写入遥测。
void Initialization::local_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr message)
{
  const auto & p = message->pose.position; const auto & q = message->pose.orientation;
  update_local_pose(p.x, p.y, p.z, {q.x, q.y, q.z, q.w}, monotonic_now(),
    common::RosTimestamp{message->header.stamp.sec, message->header.stamp.nanosec});
}
/// 提取 TwistStamped 的线速度并写入遥测。
void Initialization::local_velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr message)
{
  const auto & v = message->twist.linear; update_local_velocity(v.x, v.y, v.z, monotonic_now());
}
/// 提取 EstimatorStatus 的全部门禁相关标志。
void Initialization::estimator_callback(const mavros_msgs::msg::EstimatorStatus::SharedPtr message)
{
  update_estimator(message->attitude_status_flag, message->velocity_horiz_status_flag, message->velocity_vert_status_flag,
    message->pos_horiz_rel_status_flag, message->pos_horiz_abs_status_flag, message->pos_vert_abs_status_flag,
    message->pos_vert_agl_status_flag, message->const_pos_mode_status_flag, message->gps_glitch_status_flag,
    message->accel_error_status_flag, monotonic_now());
}
/// 将 Range 消息交由 update_range 和 RangeGuard 处理。
void Initialization::range_callback(const sensor_msgs::msg::Range::SharedPtr message)
{
  update_range(message->range, message->min_range, message->max_range, monotonic_now(),
    common::RosTimestamp{message->header.stamp.sec, message->header.stamp.nanosec});
}
/// 提取 OpticalFlowRad 中用于安全判定和日志的字段。
void Initialization::optical_flow_callback(const mavros_msgs::msg::OpticalFlowRad::SharedPtr message)
{
  update_optical_flow(message->integration_time_us, message->integrated_x, message->integrated_y, message->quality,
    message->distance, message->time_delta_distance_us, message->temperature, monotonic_now());
}
/// 转发 LCP 状态字节并更新时间序列。
void Initialization::lcp_status_callback(const std_msgs::msg::UInt8::SharedPtr message) {update_lcp_status(message->data, monotonic_now());}
/// 从 LCP 里程计提取平面坐标和有效四元数偏航。
void Initialization::lcp_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr message)
{
  const auto & p = message->pose.pose.position; const auto & q = message->pose.pose.orientation;
  double yaw = NAN;
  try {yaw = common::yaw_from_quaternion(common::normalize_quaternion(q.x, q.y, q.z, q.w));} catch (const std::invalid_argument &) {}
  update_lcp_odometry(p.x, p.y, yaw, monotonic_now());
}

/// 输出包含原始遥测、年龄、测距策略和漂移基线的严格 JSON 对象。
std::string Initialization::telemetry_json(double now) const
{
  /// 将可选单调时间戳转换为非负 JSON 遥测年龄。
  const auto age = [now](const std::optional<double> & stamp) -> std::string {
      return stamp ? number_json(std::max(0.0, now - *stamp)) : "null";
    };
  const auto & t = telemetry_;
  std::ostringstream s;
  s << "{"
    << "\"state_at\":" << optional_number_json(t.state_at) << ",\"connected\":" << bool_json(t.connected)
    << ",\"armed\":" << bool_json(t.armed) << ",\"mode\":\"" << common::json_escape(t.mode) << "\",\"system_status\":" << t.system_status
    << ",\"sys_status_at\":" << optional_number_json(t.sys_status_at) << ",\"sensors_present\":" << t.sensors_present << ",\"sensors_enabled\":" << t.sensors_enabled << ",\"sensors_health\":" << t.sensors_health << ",\"prearm_bit_advisory\":" << bool_json(t.prearm_bit_advisory)
    << ",\"battery_at\":" << optional_number_json(t.battery_at) << ",\"battery_present\":" << bool_json(t.battery_present) << ",\"battery_voltage_v\":" << number_json(t.battery_voltage_v) << ",\"battery_fraction\":" << number_json(t.battery_fraction)
    << ",\"landed_at\":" << optional_number_json(t.landed_at) << ",\"landed_state\":" << t.landed_state
    << ",\"local_pose_at\":" << optional_number_json(t.local_pose_at) << ",\"local_x_m\":" << number_json(t.local_x_m) << ",\"local_y_m\":" << number_json(t.local_y_m) << ",\"local_z_m\":" << number_json(t.local_z_m)
    << ",\"orientation_x\":" << number_json(t.orientation.x) << ",\"orientation_y\":" << number_json(t.orientation.y) << ",\"orientation_z\":" << number_json(t.orientation.z) << ",\"orientation_w\":" << number_json(t.orientation.w)
    << ",\"local_velocity_at\":" << optional_number_json(t.local_velocity_at) << ",\"velocity_x_m_s\":" << number_json(t.velocity_x_m_s) << ",\"velocity_y_m_s\":" << number_json(t.velocity_y_m_s) << ",\"velocity_z_m_s\":" << number_json(t.velocity_z_m_s)
    << ",\"estimator_at\":" << optional_number_json(t.estimator_at) << ",\"estimator_attitude_valid\":" << bool_json(t.estimator_attitude_valid) << ",\"estimator_velocity_horiz_valid\":" << bool_json(t.estimator_velocity_horiz_valid) << ",\"estimator_velocity_vert_valid\":" << bool_json(t.estimator_velocity_vert_valid) << ",\"estimator_pos_horiz_rel_valid\":" << bool_json(t.estimator_pos_horiz_rel_valid) << ",\"estimator_pos_horiz_abs_valid\":" << bool_json(t.estimator_pos_horiz_abs_valid) << ",\"estimator_pos_vert_abs_valid\":" << bool_json(t.estimator_pos_vert_abs_valid) << ",\"estimator_pos_vert_agl_valid\":" << bool_json(t.estimator_pos_vert_agl_valid) << ",\"estimator_const_pos_mode\":" << bool_json(t.estimator_const_pos_mode) << ",\"estimator_gps_glitch\":" << bool_json(t.estimator_gps_glitch) << ",\"estimator_accel_error\":" << bool_json(t.estimator_accel_error)
    << ",\"range_at\":" << optional_number_json(t.range_at) << ",\"range_m\":" << number_json(t.range_m) << ",\"range_min_m\":" << number_json(t.range_min_m) << ",\"range_max_m\":" << number_json(t.range_max_m) << ",\"range_fault\":" << optional_string_json(range_guard_.fault_reason())
    << ",\"optical_flow_at\":" << optional_number_json(t.optical_flow_at) << ",\"optical_flow_integration_time_us\":" << t.optical_flow_integration_time_us << ",\"optical_flow_integrated_x_rad\":" << number_json(t.optical_flow_integrated_x_rad) << ",\"optical_flow_integrated_y_rad\":" << number_json(t.optical_flow_integrated_y_rad) << ",\"optical_flow_quality\":" << t.optical_flow_quality << ",\"optical_flow_distance_m\":" << number_json(t.optical_flow_distance_m) << ",\"optical_flow_distance_delta_us\":" << t.optical_flow_distance_delta_us << ",\"optical_flow_temperature_c\":" << number_json(t.optical_flow_temperature_c) << ",\"optical_flow_effective\":" << bool_json(optical_flow_effective(now))
    << ",\"lcp_status\":" << (t.lcp_status ? std::to_string(*t.lcp_status) : "null") << ",\"lcp_status_at\":" << optional_number_json(t.lcp_status_at) << ",\"lcp_status_sequence\":" << t.lcp_status_sequence << ",\"lcp_odometry_at\":" << optional_number_json(t.lcp_odometry_at) << ",\"lcp_odometry_sequence\":" << t.lcp_odometry_sequence << ",\"lcp_x_m\":" << number_json(t.lcp_x_m) << ",\"lcp_y_m\":" << number_json(t.lcp_y_m) << ",\"lcp_yaw_rad\":" << number_json(t.lcp_yaw_rad) << ",\"lcp_healthy_samples\":" << t.lcp_healthy_samples << ",\"lcp_init_status_sequence_baseline\":" << t.lcp_init_status_sequence_baseline << ",\"lcp_init_odometry_sequence_baseline\":" << t.lcp_init_odometry_sequence_baseline << ",\"lcp_init_request_state\":\"" << common::json_escape(t.lcp_init_request_state) << "\",\"lcp_init_requested_at\":" << optional_number_json(t.lcp_init_requested_at) << ",\"lcp_init_response_at\":" << optional_number_json(t.lcp_init_response_at) << ",\"lcp_init_response_message\":" << optional_string_json(t.lcp_init_response_message) << ",\"lcp_init_failure_reason\":" << optional_string_json(t.lcp_init_failure_reason)
    << ",\"telemetry_age_s\":{\"range\":" << age(t.range_at) << ",\"optical_flow\":" << age(t.optical_flow_at) << ",\"lcp_status\":" << age(t.lcp_status_at) << ",\"lcp_odometry\":" << age(t.lcp_odometry_at) << "}"
    << ",\"drift_baseline\":{\"x_m\":" << optional_number_json(drift_baseline_x_m_) << ",\"y_m\":" << optional_number_json(drift_baseline_y_m_) << ",\"at\":" << optional_number_json(drift_baseline_at_) << "}"
    << ",\"range_policy\":{\"ignore_declared_min_range\":" << bool_json(config_.ignore_declared_min_range) << ",\"configured_min_range_m\":" << number_json(config_.configured_min_range_m) << ",\"configured_max_range_m\":" << number_json(config_.configured_max_range_m) << "}"
    << ",\"mavros_visibility_limits\":[\"vehicle_local_position.xy_valid/z_valid/v_xy_valid/v_z_valid/dead_reckoning\",\"estimator_status_flags.cs_opt_flow/cs_rng_hgt/reject_*\",\"estimator_aid_src_* fused/innovation_rejected\",\"sensor_optical_flow.distance_available\",\"DISTANCE_SENSOR orientation through sensor_msgs/Range\"]"
    << "}";
  return s.str();
}

}  // namespace mavros_xyz_position_offboard::initialization
