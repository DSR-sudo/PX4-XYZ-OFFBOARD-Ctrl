#include "mavros_xyz_position_offboard/application/application_node.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace mavros_xyz_position_offboard::application
{

/// 读取不受 ROS 时间跳变影响的 steady_clock 秒数。
double ApplicationNode::monotonic_now()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

/// 创建唯一节点并按声明顺序组装全部运行期模块和 20 Hz wall timer。
ApplicationNode::ApplicationNode(
  const common::AppOptions & options, const common::SafetyConfig & config)
: Node("mavros_native_xyz_position"), options_(options), config_(config),
  initialization_(*this, options_, config_), ground_station_(load_ground_station_config()),
  navigation_(config_), offboard_(*this, options_), lcp_vision_bridge_(*this, options_),
  artifact_log_(options_.artifact_dir, options_.output == "jsonl")
{
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / config_.publish_rate_hz),
    std::bind(&ApplicationNode::tick, this));
}

/// 在节点销毁时显式落盘并关闭 artifact 日志。
ApplicationNode::~ApplicationNode() {artifact_log_.close();}

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
  value.status_period_s = declare_parameter<double>("udp.status_period_s", value.status_period_s);
  value.relative_x_min_m = declare_parameter<double>("udp.relative_x_min_m", value.relative_x_min_m);
  value.relative_x_max_m = declare_parameter<double>("udp.relative_x_max_m", value.relative_x_max_m);
  value.relative_y_min_m = declare_parameter<double>("udp.relative_y_min_m", value.relative_y_min_m);
  value.relative_y_max_m = declare_parameter<double>("udp.relative_y_max_m", value.relative_y_max_m);
  value.absolute_z_min_m = declare_parameter<double>("udp.absolute_z_min_m", value.absolute_z_min_m);
  value.absolute_z_max_m = declare_parameter<double>("udp.absolute_z_max_m", value.absolute_z_max_m);
  value.validate();
  return value;
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
    navigation_.phase() != "waiting_start" && navigation_.phase() != "setpoint_warmup" &&
    navigation_.phase() != "offboard_request_pending" &&
    navigation_.phase() != "arming_request_pending" && navigation_.phase() != "manual";
  double hold_x = NAN;
  double hold_y = NAN;
  if (navigation_.planner().latched()) {
    hold_x = navigation_.planner().target_x_m();
    hold_y = navigation_.planner().target_y_m();
  }

  // 2. Capture one immutable health snapshot for this control cycle.
  auto health = initialization_.health_snapshot(now, in_flight, hold_x, hold_y, false, false);
  const auto & lcp_state = health.telemetry.lcp_init_request_state;
  if (health.preflight_errors.empty() &&
    (lcp_state == "not_requested" || lcp_state == "waiting_service")) {
    initialization_.request_lcp_start(now);
    health = initialization_.health_snapshot(now, in_flight, hold_x, hold_y, false, false);
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
  const auto decision = navigation_.update(input);
  if (navigation_.planner().latched()) {
    ground_station_.set_navigation_origin(
      navigation_.planner().origin_x_m(), navigation_.planner().origin_y_m());
  }

  // 4. Apply the idempotent execution intent to MAVROS.
  offboard_.apply({decision.setpoint, decision.target_mode, decision.arm_intent}, now);

  // 5. Send business messages and periodic telemetry through the communication layer.
  for (const auto & message : decision.messages) {ground_station_.send(message);}
  ground_station_.send_status_if_due(health.telemetry, now);

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
  stream << "{\"schema\":\"px4.mavros_native_xyz.v1\",\"phase\":\""
         << common::json_escape(decision.phase) << "\",\"monotonic_s\":"
         << std::setprecision(12) << now << ",\"navigation_batch\":{\"waypoint_index\":"
         << decision.waypoint_index << "},\"ack_age_s\":";
  if (decision.ack_age_s) {stream << *decision.ack_age_s;} else {stream << "null";}
  stream << ",\"communication_rejection\":\""
         << common::json_escape(ground_station_.last_rejection()) << "\",\"navigation_rejections\":[";
  for (std::size_t i = 0; i < decision.rejections.size(); ++i) {
    if (i) {stream << ',';}
    stream << '"' << common::json_escape(decision.rejections[i]) << '"';
  }
  stream << "],\"telemetry\":" << initialization_.telemetry_json(now) << "}";
  artifact_log_.write(stream.str());
  std::cout << "phase=" << decision.phase << " waypoint=" << decision.waypoint_index
            << " ack_age=" << (decision.ack_age_s ? std::to_string(*decision.ack_age_s) : "n/a")
            << " errors=" << health.flight_errors.size() << std::endl;
}

}  // namespace mavros_xyz_position_offboard::application
