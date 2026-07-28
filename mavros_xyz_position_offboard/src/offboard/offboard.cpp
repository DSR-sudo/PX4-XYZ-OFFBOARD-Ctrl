#include "mavros_xyz_position_offboard/offboard/offboard.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <future>

namespace mavros_xyz_position_offboard::offboard
{
namespace
{
/// 将 MAVROS 模式名称规范化为大写，避免大小写差异导致重复请求。
std::string upper(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(),
    [](unsigned char c) {return static_cast<char>(std::toupper(c));});
  return value;
}
}  // namespace

/// 依据三级 CLI 授权分别创建发布器、模式客户端和 ARM 客户端。
Offboard::Offboard(rclcpp::Node & node, const common::AppOptions & options)
: node_(node), options_(options), publish_enabled_(common::setpoint_enabled(options)),
  mode_enabled_(common::mode_enabled(options)), arming_enabled_(common::arming_enabled(options))
{
  if (publish_enabled_) {
    setpoint_publisher_ = node_.create_publisher<mavros_msgs::msg::PositionTarget>(
      options_.setpoint_topic, rclcpp::SensorDataQoS());
  }
  if (mode_enabled_) {mode_client_ = node_.create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");}
  if (arming_enabled_) {arm_client_ = node_.create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");}
}

/// 同步 MAVROS state 的连接、实际 ARM 和实际模式反馈，供幂等判断使用。
void Offboard::observe_flight_state(const common::Telemetry & telemetry)
{
  status_.connected = telemetry.connected;
  status_.armed = telemetry.armed;
  status_.mode = upper(telemetry.mode);
}

/// 构造保留 XYZ+yaw 并忽略速度、加速度和 yaw-rate 的 LOCAL_NED 原始设定点。
mavros_msgs::msg::PositionTarget Offboard::make_position_target(
  const common::PositionSetpoint & setpoint, const builtin_interfaces::msg::Time & stamp)
{
  mavros_msgs::msg::PositionTarget message;
  message.header.stamp = stamp;
  message.header.frame_id = "map";
  message.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
  message.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_VX |
    mavros_msgs::msg::PositionTarget::IGNORE_VY | mavros_msgs::msg::PositionTarget::IGNORE_VZ |
    mavros_msgs::msg::PositionTarget::IGNORE_AFX | mavros_msgs::msg::PositionTarget::IGNORE_AFY |
    mavros_msgs::msg::PositionTarget::IGNORE_AFZ | mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
  message.position.x = setpoint.x_m;
  message.position.y = setpoint.y_m;
  message.position.z = setpoint.z_m;
  message.yaw = static_cast<float>(common::yaw_from_quaternion(setpoint.orientation));
  return message;
}

/// 在限频和单一在途请求约束下提交异步 SetMode 请求并记录状态。
void Offboard::request_mode(const std::string & mode, double now)
{
  if (!mode_client_ || mode_future_ || now - last_mode_request_at_ < options_.mode_request_interval) {return;}
  last_mode_request_at_ = now;
  pending_mode_ = mode;
  if (!mode_client_->service_is_ready()) {
    status_.mode_request = {"service_not_ready", false, {}, now};
    return;
  }
  auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  request->base_mode = 0;
  request->custom_mode = mode;
  mode_future_ = mode_client_->async_send_request(request);
  mode_future_started_at_ = now;
  status_.mode_request = {"request_sent", {}, {}, now};
}

/// 在限频和单一在途请求约束下提交普通 CommandBool ARM/Disarm 请求。
void Offboard::request_arm(bool value, double now)
{
  if (!arm_client_ || arm_future_ || now - last_arm_request_at_ < options_.mode_request_interval) {return;}
  last_arm_request_at_ = now;
  pending_arm_ = value;
  if (!arm_client_->service_is_ready()) {
    status_.arm_request = {"service_not_ready", false, {}, now};
    return;
  }
  auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  request->value = value;
  arm_future_ = arm_client_->async_send_request(request);
  arm_future_started_at_ = now;
  status_.arm_request = {"request_sent", {}, {}, now};
}

/// 发布可选设定点，并仅在意图与实际反馈不同时请求模式或 ARM 状态。
void Offboard::apply(const OffboardCommand & command, double now)
{
  if (command.setpoint && setpoint_publisher_) {
    const auto stamp = node_.get_clock()->now();
    builtin_interfaces::msg::Time message_stamp;
    const auto nanoseconds = stamp.nanoseconds();
    message_stamp.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
    message_stamp.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
    setpoint_publisher_->publish(make_position_target(*command.setpoint, message_stamp));
  }
  if (command.target_mode && upper(*command.target_mode) != status_.mode) {
    request_mode(*command.target_mode, now);
  }
  if (command.arm_intent && *command.arm_intent != status_.armed) {
    request_arm(*command.arm_intent, now);
  }
}

/// 非阻塞收割异步服务响应，捕获异常，并按 service_timeout 关闭超时句柄。
void Offboard::poll(double now)
{
  if (mode_future_) {
    if (mode_future_->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      try {
        const auto response = mode_future_->get();
        status_.mode_request = {"response", response->mode_sent, pending_mode_, now};
      } catch (const std::exception & error) {
        status_.mode_request = {"exception", false, error.what(), now};
      }
      mode_future_.reset(); mode_future_started_at_.reset(); pending_mode_.reset();
    } else if (mode_future_started_at_ && now - *mode_future_started_at_ > options_.service_timeout) {
      status_.mode_request = {"timeout", false, pending_mode_, now};
      mode_future_.reset(); mode_future_started_at_.reset(); pending_mode_.reset();
    }
  }
  if (arm_future_) {
    if (arm_future_->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      try {
        const auto response = arm_future_->get();
        status_.arm_request = {"response", response->success, std::to_string(response->result), now};
      } catch (const std::exception & error) {
        status_.arm_request = {"exception", false, error.what(), now};
      }
      arm_future_.reset(); arm_future_started_at_.reset(); pending_arm_.reset();
    } else if (arm_future_started_at_ && now - *arm_future_started_at_ > options_.service_timeout) {
      status_.arm_request = {"timeout", false, pending_arm_ && *pending_arm_ ? "arm" : "disarm", now};
      arm_future_.reset(); arm_future_started_at_.reset(); pending_arm_.reset();
    }
  }
}

}  // namespace mavros_xyz_position_offboard::offboard
