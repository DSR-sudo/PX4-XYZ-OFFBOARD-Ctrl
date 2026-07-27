#include "mavros_xyz_position_offboard/bridge/lcp_vision_bridge.hpp"

#include <cmath>
#include <functional>
#include <stdexcept>

#include "mavros_xyz_position_offboard/common/types.hpp"

namespace mavros_xyz_position_offboard::bridge
{
namespace
{
constexpr std::uint8_t kLcpLockedStatus = 2;
constexpr double kPi = 3.14159265358979323846;
constexpr double kHalfPi = 1.57079632679489661923;
constexpr double kLargeVariance = 10000.0;

double wrap_pi(double angle)
{
  while (angle > kPi) {angle -= 2.0 * kPi;}
  while (angle <= -kPi) {angle += 2.0 * kPi;}
  return angle;
}
}  // namespace

LcpVisionBridge::LcpVisionBridge(rclcpp::Node & node, const common::AppOptions & options)
: node_(node), enabled_(options.lcp_vision_bridge_enabled), input_frame_(options.lcp_vision_input_frame),
  xy_stddev_m_(options.lcp_vision_xy_stddev_m), yaw_stddev_rad_(options.lcp_vision_yaw_stddev_rad),
  max_status_age_s_(options.lcp_vision_max_status_age_s)
{
  if (!enabled_) {
    last_reject_reason_ = "bridge disabled by --disable-lcp-vision-bridge";
    RCLCPP_INFO(node_.get_logger(), "LCP vision bridge disabled");
    return;
  }
  const auto lcp_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  publisher_ = node_.create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    options.lcp_vision_pose_topic, lcp_qos);
  status_subscription_ = node_.create_subscription<std_msgs::msg::UInt8>(
    options.lcp_status_topic, lcp_qos, std::bind(&LcpVisionBridge::status_callback, this, std::placeholders::_1));
  odometry_subscription_ = node_.create_subscription<nav_msgs::msg::Odometry>(
    options.lcp_odometry_topic, lcp_qos, std::bind(&LcpVisionBridge::odometry_callback, this, std::placeholders::_1));
  last_reject_reason_ = "waiting for LCP STATUS=2";
  RCLCPP_INFO(
    node_.get_logger(), "LCP vision bridge enabled: %s (%s) -> %s",
    options.lcp_odometry_topic.c_str(), input_frame_.c_str(), options.lcp_vision_pose_topic.c_str());
}

geometry_msgs::msg::PoseWithCovarianceStamped LcpVisionBridge::nwu_to_enu(
  const nav_msgs::msg::Odometry & source, double xy_stddev_m, double yaw_stddev_rad)
{
  if (!common::finite(source.pose.pose.position.x) || !common::finite(source.pose.pose.position.y) ||
      !common::finite(source.pose.pose.position.z) || xy_stddev_m <= 0.0 || yaw_stddev_rad <= 0.0) {
    throw std::invalid_argument("LCP pose and standard deviations must be finite and positive");
  }
  const auto orientation = common::normalize_quaternion(
    source.pose.pose.orientation.x, source.pose.pose.orientation.y,
    source.pose.pose.orientation.z, source.pose.pose.orientation.w);
  const double enu_yaw = wrap_pi(kHalfPi + common::yaw_from_quaternion(orientation));

  geometry_msgs::msg::PoseWithCovarianceStamped output;
  output.header = source.header;
  output.header.frame_id = "lcp_enu";
  // lcp_nwu is North/West/Up; ROS/MAVROS pose topics use East/North/Up.
  output.pose.pose.position.x = -source.pose.pose.position.y;
  output.pose.pose.position.y = source.pose.pose.position.x;
  output.pose.pose.position.z = source.pose.pose.position.z;
  output.pose.pose.orientation.z = std::sin(0.5 * enu_yaw);
  output.pose.pose.orientation.w = std::cos(0.5 * enu_yaw);
  output.pose.covariance.fill(0.0);
  output.pose.covariance[0] = xy_stddev_m * xy_stddev_m;
  output.pose.covariance[7] = xy_stddev_m * xy_stddev_m;
  output.pose.covariance[14] = kLargeVariance;
  output.pose.covariance[21] = kLargeVariance;
  output.pose.covariance[28] = kLargeVariance;
  output.pose.covariance[35] = yaw_stddev_rad * yaw_stddev_rad;
  return output;
}

void LcpVisionBridge::status_callback(const std_msgs::msg::UInt8::SharedPtr message)
{
  lcp_status_ = message->data;
  lcp_status_at_ = monotonic_now();
}

void LcpVisionBridge::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr message)
{
  if (lcp_status_ != kLcpLockedStatus || monotonic_now() - lcp_status_at_ > max_status_age_s_) {
    reject("LCP is not in fresh STATUS=2");
    return;
  }
  if (message->header.frame_id != input_frame_) {
    reject("LCP frame is '" + message->header.frame_id + "', expected '" + input_frame_ + "'");
    return;
  }
  try {
    publisher_->publish(nwu_to_enu(*message, xy_stddev_m_, yaw_stddev_rad_));
    ++published_count_;
    last_reject_reason_.clear();
  } catch (const std::exception & error) {
    reject(std::string("invalid LCP odometry: ") + error.what());
  }
}

void LcpVisionBridge::reject(const std::string & reason)
{
  if (reason != last_reject_reason_) {
    RCLCPP_WARN(node_.get_logger(), "LCP vision bridge: %s", reason.c_str());
  }
  last_reject_reason_ = reason;
}

double LcpVisionBridge::monotonic_now()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace mavros_xyz_position_offboard::bridge
