#include "mavros_xyz_position_offboard/application/application_node.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

namespace mavros_xyz_position_offboard::application
{
namespace
{

const rcl_interfaces::msg::ParameterDescriptor & safety_parameter_descriptor()
{
  static const rcl_interfaces::msg::ParameterDescriptor descriptor = [] {
      rcl_interfaces::msg::ParameterDescriptor value;
      value.read_only = true;
      value.description = "Applied only during node creation; restart the node to change it.";
      return value;
    }();
  return descriptor;
}

}  // namespace

/// 读取不受 ROS 时间跳变影响的 steady_clock 秒数。
double ApplicationNode::monotonic_now()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

/// 创建唯一节点并按声明顺序组装全部运行期模块和 20 Hz wall timer。
ApplicationNode::ApplicationNode(
  const common::AppOptions & options, const common::SafetyConfig & config,
  const rclcpp::NodeOptions & node_options)
: Node("mavros_native_xyz_position", node_options), options_(options), config_(load_safety_config(config)),
  initialization_(*this, options_, config_), mission_config_(load_mission_config()),
  ground_station_(load_ground_station_config()), navigation_(config_, mission_config_), offboard_(*this, options_),
  lcp_vision_bridge_(*this, options_), z_config_(load_z_config()), gripper_(load_gripper_config()),
  artifact_log_(options_.artifact_dir, options_.output == "jsonl")
{
  if (!gripper_.initialize()) {
    throw std::runtime_error("SG90 gripper initialization failed: " + gripper_.fault().value_or("unknown error"));
  }
  const auto lcp_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  lcp_debug_subscription_ = create_subscription<lslidar_msgs::msg::LcpDebug>(
    "/lcp/debug", lcp_qos, std::bind(&ApplicationNode::lcp_debug_callback, this, std::placeholders::_1));
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / config_.publish_rate_hz),
    std::bind(&ApplicationNode::tick, this));
}

/// 在节点销毁时显式落盘并关闭 artifact 日志。
ApplicationNode::~ApplicationNode() {artifact_log_.close();}

common::SafetyConfig ApplicationNode::load_safety_config(const common::SafetyConfig & defaults)
{
  auto value = defaults;
  const auto & descriptor = safety_parameter_descriptor();
  value.state_timeout_s = declare_parameter<double>("safety.state_timeout_s", value.state_timeout_s, descriptor);
  value.sys_status_timeout_s = declare_parameter<double>(
    "safety.sys_status_timeout_s", value.sys_status_timeout_s, descriptor);
  value.battery_timeout_s = declare_parameter<double>("safety.battery_timeout_s", value.battery_timeout_s, descriptor);
  value.landed_timeout_s = declare_parameter<double>("safety.landed_timeout_s", value.landed_timeout_s, descriptor);
  value.local_pose_timeout_s = declare_parameter<double>(
    "safety.local_pose_timeout_s", value.local_pose_timeout_s, descriptor);
  value.local_velocity_timeout_s = declare_parameter<double>(
    "safety.local_velocity_timeout_s", value.local_velocity_timeout_s, descriptor);
  value.estimator_timeout_s = declare_parameter<double>(
    "safety.estimator_timeout_s", value.estimator_timeout_s, descriptor);
  value.range_timeout_s = declare_parameter<double>("safety.range_timeout_s", value.range_timeout_s, descriptor);
  value.optical_flow_timeout_s = declare_parameter<double>(
    "safety.optical_flow_timeout_s", value.optical_flow_timeout_s, descriptor);
  value.sensor_loss_grace_s = declare_parameter<double>(
    "safety.sensor_loss_grace_s", value.sensor_loss_grace_s, descriptor);
  value.lcp_status_timeout_s = declare_parameter<double>(
    "safety.lcp_status_timeout_s", value.lcp_status_timeout_s, descriptor);
  value.lcp_odometry_timeout_s = declare_parameter<double>(
    "safety.lcp_odometry_timeout_s", value.lcp_odometry_timeout_s, descriptor);
  value.lcp_ready_samples = declare_parameter<int>("safety.lcp_ready_samples", value.lcp_ready_samples, descriptor);
  value.range_boundary_tolerance_m = declare_parameter<double>(
    "safety.range_boundary_tolerance_m", value.range_boundary_tolerance_m, descriptor);
  value.configured_min_range_m = declare_parameter<double>(
    "safety.configured_min_range_m", value.configured_min_range_m, descriptor);
  value.configured_max_range_m = declare_parameter<double>(
    "safety.configured_max_range_m", value.configured_max_range_m, descriptor);
  value.max_range_jump_m = declare_parameter<double>(
    "safety.max_range_jump_m", value.max_range_jump_m, descriptor);
  value.jump_window_s = declare_parameter<double>("safety.jump_window_s", value.jump_window_s, descriptor);
  value.jump_recovery_samples = declare_parameter<int>(
    "safety.jump_recovery_samples", value.jump_recovery_samples, descriptor);
  value.jump_settle_tolerance_m = declare_parameter<double>(
    "safety.jump_settle_tolerance_m", value.jump_settle_tolerance_m, descriptor);
  value.min_optical_flow_quality = declare_parameter<int>(
    "safety.min_optical_flow_quality", value.min_optical_flow_quality, descriptor);
  value.min_battery_voltage_v = declare_parameter<double>(
    "safety.min_battery_voltage_v", value.min_battery_voltage_v, descriptor);
  value.min_battery_fraction = declare_parameter<double>(
    "safety.min_battery_fraction", value.min_battery_fraction, descriptor);
  value.max_preflight_horizontal_speed_m_s = declare_parameter<double>(
    "safety.max_preflight_horizontal_speed_m_s", value.max_preflight_horizontal_speed_m_s, descriptor);
  value.max_preflight_vertical_speed_m_s = declare_parameter<double>(
    "safety.max_preflight_vertical_speed_m_s", value.max_preflight_vertical_speed_m_s, descriptor);
  value.max_flight_horizontal_speed_m_s = declare_parameter<double>(
    "safety.max_flight_horizontal_speed_m_s", value.max_flight_horizontal_speed_m_s, descriptor);
  value.max_flight_vertical_speed_m_s = declare_parameter<double>(
    "safety.max_flight_vertical_speed_m_s", value.max_flight_vertical_speed_m_s, descriptor);
  value.max_flight_horizontal_drift_m = declare_parameter<double>(
    "safety.max_flight_horizontal_drift_m", value.max_flight_horizontal_drift_m, descriptor);
  value.climb_horizontal_speed_limit_m_s = declare_parameter<double>(
    "safety.climb_horizontal_speed_limit_m_s", value.climb_horizontal_speed_limit_m_s, descriptor);
  value.climb_horizontal_drift_limit_m = declare_parameter<double>(
    "safety.climb_horizontal_drift_limit_m", value.climb_horizontal_drift_limit_m, descriptor);
  value.hover_min_height_m = declare_parameter<double>(
    "safety.hover_min_height_m", value.hover_min_height_m, descriptor);
  value.publish_rate_hz = declare_parameter<double>("safety.publish_rate_hz", value.publish_rate_hz, descriptor);
  value.setpoint_warmup_s = declare_parameter<double>(
    "safety.setpoint_warmup_s", value.setpoint_warmup_s, descriptor);
  value.max_z_setpoint_rate_m_s = declare_parameter<double>(
    "safety.max_z_setpoint_rate_m_s", value.max_z_setpoint_rate_m_s, descriptor);
  value.max_z_setpoint_accel_m_s2 = declare_parameter<double>(
    "safety.max_z_setpoint_accel_m_s2", value.max_z_setpoint_accel_m_s2, descriptor);
  value.target_xy_max_speed_m_s = declare_parameter<double>(
    "safety.target_xy_max_speed_m_s", value.target_xy_max_speed_m_s, descriptor);
  value.target_xy_max_accel_m_s2 = declare_parameter<double>(
    "safety.target_xy_max_accel_m_s2", value.target_xy_max_accel_m_s2, descriptor);
  value.target_tolerance_m = declare_parameter<double>(
    "safety.target_tolerance_m", value.target_tolerance_m, descriptor);
  value.touchdown_z_tolerance_m = declare_parameter<double>(
    "safety.touchdown_z_tolerance_m", value.touchdown_z_tolerance_m, descriptor);
  value.max_flight_seconds = declare_parameter<double>(
    "safety.max_flight_seconds", value.max_flight_seconds, descriptor);
  value.flow_effective_min_height_m = declare_parameter<double>(
    "safety.flow_effective_min_height_m", value.flow_effective_min_height_m, descriptor);
  value.flow_effective_min_quality = declare_parameter<int>(
    "safety.flow_effective_min_quality", value.flow_effective_min_quality, descriptor);
  value.ignore_declared_min_range = declare_parameter<bool>(
    "safety.ignore_declared_min_range", value.ignore_declared_min_range, descriptor);
  value.validate();
  return value;
}

/// 从 ROS 参数构造并校验 GroundStationLink 的固定启动配置。
communication::GroundStationConfig ApplicationNode::load_ground_station_config()
{
  communication::GroundStationConfig value;
  value.enabled = declare_parameter<bool>("udp.enabled", value.enabled);
  value.bind_ip = declare_parameter<std::string>("udp.bind_ip", value.bind_ip);
  value.bind_port = declare_parameter<int>("udp.bind_port", value.bind_port);
  value.remote_ip = declare_parameter<std::string>("udp.remote_ip", value.remote_ip);
  value.remote_port = declare_parameter<int>("udp.remote_port", value.remote_port);
  value.whitelist_ip = declare_parameter<std::string>("udp.whitelist_ip", value.whitelist_ip);
  value.whitelist_port = declare_parameter<int>("udp.whitelist_port", value.whitelist_port);
  value.event_retry_period_s = declare_parameter<double>(
    "udp.event_retry_period_s", value.event_retry_period_s);
  value.max_tracking_distance_m = declare_parameter<double>(
    "udp.max_tracking_distance_m", value.max_tracking_distance_m);
  value.validate();
  return value;
}

navigation::MissionConfig ApplicationNode::load_mission_config()
{
  navigation::MissionConfig value;
  value.takeoff_height_m = declare_parameter<double>("mission.takeoff_height_m", value.takeoff_height_m);
  value.height_stable_seconds = declare_parameter<double>(
    "mission.height_stable_seconds", value.height_stable_seconds);
  value.b_right_m = declare_parameter<double>("mission.b_right_m", value.b_right_m);
  value.b_forward_m = declare_parameter<double>("mission.b_forward_m", value.b_forward_m);
  value.throw_distance_m = declare_parameter<double>(
    "mission.throw_distance_m", value.throw_distance_m);
  value.filter_measurement_noise_m = declare_parameter<double>(
    "mission.filter_measurement_noise_m", value.filter_measurement_noise_m);
  value.filter_acceleration_noise_m_s2 = declare_parameter<double>(
    "mission.filter_acceleration_noise_m_s2", value.filter_acceleration_noise_m_s2);
  value.filter_min_samples = declare_parameter<int>(
    "mission.filter_min_samples", value.filter_min_samples);
  value.prediction_horizon_s = declare_parameter<double>(
    "mission.prediction_horizon_s", value.prediction_horizon_s);
  value.cardinal_tolerance_deg = declare_parameter<double>(
    "mission.cardinal_tolerance_deg", value.cardinal_tolerance_deg);
  value.final_intercept_seconds = declare_parameter<double>(
    "mission.final_intercept_seconds", value.final_intercept_seconds);
  value.car_status_timeout_s = declare_parameter<double>(
    "mission.car_status_timeout_s", value.car_status_timeout_s);
  value.max_tracking_radius_m = declare_parameter<double>(
    "mission.max_tracking_radius_m", value.max_tracking_radius_m);
  value.validate();
  return value;
}

void ApplicationNode::ZConfig::validate() const
{
  if (!std::isfinite(source_timeout_s) || !std::isfinite(range_cross_check_max_delta_m) ||
    source_timeout_s <= 0.0 || range_cross_check_max_delta_m <= 0.0) {
    throw std::invalid_argument("Z source timeout and range cross-check delta must be finite and positive");
  }
}

ApplicationNode::ZConfig ApplicationNode::load_z_config()
{
  ZConfig value;
  value.prefer_range = declare_parameter<bool>("z.prefer_range", value.prefer_range);
  value.source_timeout_s = declare_parameter<double>("z.source_timeout_s", value.source_timeout_s);
  value.range_cross_check_max_delta_m = declare_parameter<double>(
    "z.range_cross_check_max_delta_m", value.range_cross_check_max_delta_m);
  value.validate();
  return value;
}

gripper::PwmGripperConfig ApplicationNode::load_gripper_config()
{
  gripper::PwmGripperConfig value;
  value.enabled = declare_parameter<bool>("gripper_pwm.enabled", value.enabled);
  value.bcm_gpio = declare_parameter<int>("gripper_pwm.bcm_gpio", value.bcm_gpio);
  value.pwm_frequency_hz = declare_parameter<double>(
    "gripper_pwm.pwm_frequency_hz", value.pwm_frequency_hz);
  value.closed_duty_cycle = declare_parameter<double>(
    "gripper_pwm.closed_duty_cycle", value.closed_duty_cycle);
  value.open_duty_cycle = declare_parameter<double>(
    "gripper_pwm.open_duty_cycle", value.open_duty_cycle);
  value.open_hold_ms = declare_parameter<int>("gripper_pwm.open_hold_ms", value.open_hold_ms);
  value.validate();
  return value;
}

void ApplicationNode::latch_init_height(const common::Telemetry & telemetry)
{
  init_local_z_m_.reset();
  init_range_m_.reset();
  if (common::finite(telemetry.local_z_m)) {init_local_z_m_ = telemetry.local_z_m;}
  if (common::finite(telemetry.range_m) && !initialization_.range_fault()) {init_range_m_ = telemetry.range_m;}
}

void ApplicationNode::lcp_debug_callback(const lslidar_msgs::msg::LcpDebug::SharedPtr message)
{
  const double now = monotonic_now();
  communication::XyzStatus status;
  status.header = {message->header.stamp.sec, message->header.stamp.nanosec, message->header.frame_id};
  status.status = message->status;
  status.map_locked = message->map_locked;
  status.pose_valid = message->pose_valid;
  status.position_x_m = message->position_x_m;
  status.position_y_m = message->position_y_m;
  status.yaw_rad = message->yaw_rad;
  status.front_distance_m = message->front_distance_m;
  status.rear_distance_m = message->rear_distance_m;
  status.left_distance_m = message->left_distance_m;
  status.right_distance_m = message->right_distance_m;
  status.map_size_x_m = message->map_size_x_m;
  status.map_size_y_m = message->map_size_y_m;

  const auto & telemetry = initialization_.telemetry();
  const bool local_fresh = init_local_z_m_ && telemetry.local_pose_stamp &&
    !common::stale(telemetry.local_pose_at, now, z_config_.source_timeout_s) &&
    common::finite(telemetry.local_z_m);
  const bool range_fresh = init_range_m_ && telemetry.range_stamp && !initialization_.range_fault() &&
    !common::stale(telemetry.range_at, now, z_config_.source_timeout_s) && common::finite(telemetry.range_m);
  const std::optional<double> local_z = local_fresh ?
    std::optional<double>(telemetry.local_z_m - *init_local_z_m_) : std::nullopt;
  const std::optional<double> range_z = range_fresh ?
    std::optional<double>(telemetry.range_m - *init_range_m_) : std::nullopt;
  const bool sources_valid = local_z && range_z &&
    std::abs(*local_z - *range_z) <= z_config_.range_cross_check_max_delta_m;

  if (sources_valid) {
    const bool use_range = z_config_.prefer_range;
    status.position_z_m = use_range ? range_z : local_z;
    status.z_source = use_range ? "range" : "local_pose";
    status.z_source_stamp = use_range ? telemetry.range_stamp : telemetry.local_pose_stamp;
    status.z_valid = true;
  }
  // Invalid Z intentionally retains every LCP field and the protocol's null/none metadata.
  try {
    ground_station_.send_xyzstatus(status);
  } catch (const std::exception & error) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "xyzstatus rejected: %s", error.what());
  }
}

/// 执行“轮询、快照、导航、飞控、通信、日志”的单线程控制周期。
void ApplicationNode::tick()
{
  const double now = monotonic_now();
  const double dt = last_tick_at_ ? std::max(0.001, std::min(0.25, now - *last_tick_at_)) :
    1.0 / config_.publish_rate_hz;
  last_tick_at_ = now;

  // 1. Poll asynchronous MAVROS service results and all currently readable UDP packets.
  initialization_.poll_lcp_start(now, options_.service_timeout);
  offboard_.poll(now);
  auto protocol_events = ground_station_.poll(now);

  const bool in_flight = navigation_.phase() != "waiting_preflight" &&
    navigation_.phase() != "waiting_run_plan1" && navigation_.phase() != "setpoint_warmup" &&
    navigation_.phase() != "offboard_request_pending" &&
    navigation_.phase() != "arming_request_pending" && navigation_.phase() != "manual";
  double commanded_x = NAN;
  double commanded_y = NAN;
  if (const auto & commanded = navigation_.commanded_setpoint()) {
    commanded_x = commanded->x_m;
    commanded_y = commanded->y_m;
  }

  // 2. Capture one immutable health snapshot for this control cycle.
  auto health = initialization_.health_snapshot(now, in_flight, commanded_x, commanded_y, false, false);
  const auto & lcp_state = health.telemetry.lcp_init_request_state;
  // LCP 建系仅会清空地面地图，允许在电池未上电的传感器验收阶段先完成。
  // 起飞仍由下方完整 preflight_errors 和 lcp_ready 两个门禁共同限制。
  if (initialization_.lcp_start_prerequisite_errors(now).empty() &&
    (lcp_state == "not_requested" || lcp_state == "waiting_service")) {
    initialization_.request_lcp_start(now);
    health = initialization_.health_snapshot(now, in_flight, commanded_x, commanded_y, false, false);
  }

  bool flight_healthy = health.flight_errors.empty();
  if (in_flight && !flight_healthy) {
    if (!sensor_fault_since_) {sensor_fault_since_ = now;}
    flight_healthy = now - *sensor_fault_since_ <= config_.sensor_loss_grace_s;
  } else {
    sensor_fault_since_.reset();
  }

  offboard_.observe_flight_state(health.telemetry);
  const auto controller = offboard_.status();

  // 3. Run the pure Navigation state machine.
  navigation::NavigationInput input;
  input.now = now;
  input.dt = dt;
  input.telemetry = health.telemetry;
  input.preflight_ready = health.preflight_errors.empty() && health.lcp_ready;
  input.flight_healthy = flight_healthy;
  input.lcp_healthy = health.lcp_healthy;
  input.health_errors = health.flight_errors;
  input.events = std::move(protocol_events);
  input.controller = {controller.connected, controller.armed, controller.mode,
    controller.mode_request.state, controller.arm_request.state};
  const auto gripper_state = gripper_.update(now);
  input.gripper_succeeded = gripper_state == gripper::ReleaseState::succeeded;
  input.gripper_failed = gripper_state == gripper::ReleaseState::failed;
  const bool planner_was_latched = navigation_.planner().latched();
  const auto decision = navigation_.update(input);
  const bool planner_is_latched = navigation_.planner().latched();
  if (!planner_was_latched && planner_is_latched) {
    latch_init_height(health.telemetry);
  } else if (planner_was_latched && !planner_is_latched) {
    init_local_z_m_.reset();
    init_range_m_.reset();
  }

  // 4. Apply the idempotent execution intent to MAVROS.
  offboard_.apply({decision.setpoint, decision.target_mode, decision.arm_intent}, now);

  // 5. Send business messages and periodic telemetry through the communication layer.
  if (decision.release_gripper) {gripper_.begin_release(now);}
  for (const auto & message : decision.messages) {
    ground_station_.send(message, now);
  }
  ground_station_.retry_events(now);

  // 6. Emit the unified audit status.
  emit_status(now, health, decision);
}

/// 按 status_period 限频记录阶段、批次、ACK 年龄、拒绝原因和完整遥测。
void ApplicationNode::emit_status(
  double now, const initialization::HealthSnapshot & health,
  const navigation::NavigationDecision & decision)
{
  if (now - last_log_at_ < options_.status_period) {return;}
  last_log_at_ = now;
  std::ostringstream stream;
  stream << "{\"schema\":\"px4.mavros_native_xyz.v3\",\"phase\":\""
         << common::json_escape(decision.phase) << "\",\"monotonic_s\":"
         << std::setprecision(12) << now << ",\"pending_event_count\":"
         << ground_station_.pending_event_count();
  stream << ",\"communication_rejection\":\""
         << common::json_escape(ground_station_.last_rejection()) << "\",\"navigation_rejections\":[";
  for (std::size_t i = 0; i < decision.rejections.size(); ++i) {
    if (i) {stream << ',';}
    stream << '"' << common::json_escape(decision.rejections[i]) << '"';
  }
  const auto error_array = [](const std::vector<std::string> & errors) {
      std::ostringstream encoded;
      encoded << '[';
      for (std::size_t i = 0; i < errors.size(); ++i) {
        if (i) {encoded << ',';}
        encoded << '"' << common::json_escape(errors[i]) << '"';
      }
      encoded << ']';
      return encoded.str();
    };
  stream << "],\"preflight_errors\":" << error_array(health.preflight_errors)
         << ",\"flight_errors\":" << error_array(health.flight_errors)
         << ",\"control\":" << navigation::control_json(decision.control)
         << ",\"telemetry\":" << initialization_.telemetry_json(now) << "}";
  artifact_log_.write(stream.str());
  std::cout << "phase=" << decision.phase << " pending_events=" << ground_station_.pending_event_count()
            << " errors=" << health.flight_errors.size() << std::endl;
}

}  // namespace mavros_xyz_position_offboard::application
