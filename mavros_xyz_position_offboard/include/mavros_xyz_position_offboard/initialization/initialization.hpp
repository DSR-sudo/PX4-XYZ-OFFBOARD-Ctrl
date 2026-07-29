#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/estimator_status.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/optical_flow_rad.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/sys_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "mavros_xyz_position_offboard/common/cli.hpp"
#include "mavros_xyz_position_offboard/common/types.hpp"

namespace mavros_xyz_position_offboard::initialization
{

class RangeGuard
{
public:
  /// 用统一配置创建测距有效性与跳变恢复守卫。
  explicit RangeGuard(const common::SafetyConfig & config) : config_(config) {}
  /// 接收一条测距消息并返回接受或拒绝结果。
  common::RangeResult observe(double value_m, double declared_min_m, double declared_max_m, double now);
  /// 返回当前锁存的测距故障原因。
  const std::optional<std::string> & fault_reason() const {return fault_reason_;}

private:
  /// 记录并返回不可恢复或待恢复的测距故障。
  common::RangeResult fault(const std::string & reason);
  /// 累积稳定样本，满足数量后解除测距故障。
  common::RangeResult recover(double value_m, double now);
  const common::SafetyConfig & config_;
  std::optional<double> last_accepted_m_;
  std::optional<double> last_accepted_at_;
  std::optional<std::string> fault_reason_;
  std::optional<double> candidate_m_;
  int candidate_count_{0};
};

struct ServiceEvent
{
  std::string status{"never_requested"};
  std::optional<bool> success{};
  std::optional<std::string> message{};
  std::optional<double> monotonic_s{};
};

struct HealthSnapshot
{
  common::Telemetry telemetry{};
  std::vector<std::string> preflight_errors{};
  std::vector<std::string> flight_errors{};
  bool lcp_healthy{false};
  bool lcp_ready{false};
};

class Initialization
{
public:
  /// 创建全部遥测订阅及 LCP 初始化客户端。
  Initialization(
    rclcpp::Node & node, const common::AppOptions & options, const common::SafetyConfig & config);

  /// 返回可被测试夹具注入的可变遥测快照。
  common::Telemetry & telemetry() {return telemetry_;}
  /// 返回只读遥测快照。
  const common::Telemetry & telemetry() const {return telemetry_;}
  /// 返回 RangeGuard 当前的故障说明。
  const std::optional<std::string> & range_fault() const {return range_guard_.fault_reason();}
  /// 返回最近一次 LCP 初始化服务事件。
  const ServiceEvent & lcp_start_event() const {return lcp_start_event_;}

  /// 写入 MAVROS state/heartbeat 遥测。
  void update_state(bool connected, bool armed, const std::string & mode, int system_status, double now);
  /// 写入 MAVROS SYS_STATUS 位掩码遥测。
  void update_sys_status(std::uint32_t present, std::uint32_t enabled, std::uint32_t health, double now);
  /// 写入电池存在性、电压和电量。
  void update_battery(bool present, double voltage_v, double fraction, double now);
  /// 写入飞行器落地状态。
  void update_landed(int landed_state, double now);
  /// 写入本地位置及姿态四元数。
  void update_local_pose(
    double x_m, double y_m, double z_m, const common::Quaternion & orientation, double now,
    std::optional<common::RosTimestamp> stamp = std::nullopt);
  /// 写入本地 XYZ 速度。
  void update_local_velocity(double x_m_s, double y_m_s, double z_m_s, double now);
  /// 写入估计器有效性与故障标志。
  void update_estimator(
    bool attitude_valid, bool velocity_horiz_valid, bool velocity_vert_valid,
    bool pos_horiz_rel_valid, bool pos_horiz_abs_valid, bool pos_vert_abs_valid,
    bool pos_vert_agl_valid, bool const_pos_mode, bool gps_glitch, bool accel_error, double now);
  /// 写入测距消息，并执行边界、跳变和恢复检查。
  common::RangeResult update_range(
    double range_m, double declared_min_m, double declared_max_m, double now,
    std::optional<common::RosTimestamp> stamp = std::nullopt);
  /// 写入光流积分、质量和距离遥测。
  void update_optical_flow(
    std::uint32_t integration_time_us, double integrated_x_rad, double integrated_y_rad, int quality,
    double distance_m, std::uint32_t distance_delta_us, double temperature_c, double now);
  /// 写入 LCP 状态并累计初始化后的健康样本。
  void update_lcp_status(int status, double now);
  /// 写入 LCP 里程计平面位置及偏航。
  void update_lcp_odometry(double x_m, double y_m, double yaw_rad, double now);

  /// 锁存初始化前的 LCP 序列号基线并清空健康计数。
  void begin_lcp_initialization(double now);
  /// 取消当前 LCP 初始化候选并恢复未请求状态。
  void reset_lcp_initialization();
  /// 更新 LCP 服务请求状态及可选响应/失败详情。
  void update_lcp_init_state(
    const std::string & state, double now, const std::optional<std::string> & message = std::nullopt,
    const std::optional<std::string> & failure_reason = std::nullopt);
  /// 在服务可用且尚未请求时异步请求 LCP 初始化。
  void request_lcp_start(double now);
  /// 处理 LCP 服务响应或请求超时。
  void poll_lcp_start(double now, double timeout_s);
  /// 丢弃尚未完成的 LCP 请求句柄。
  void cancel_lcp_start();
  /// 返回允许请求 LCP 建系服务的地面飞控条件错误，不检查电池或飞行传感器门禁。
  std::vector<std::string> lcp_start_prerequisite_errors(double now) const;

  /// 判断 LCP 的状态和里程计在飞行期是否新鲜且健康。
  bool lcp_runtime_healthy(double now) const;
  /// 判断已接受服务且拥有足够的新鲜 LCP 样本，能否解除预检门禁。
  bool lcp_ready(double now) const;
  /// 返回解释 LCP 未就绪或失效原因的错误列表。
  std::vector<std::string> lcp_errors(double now, bool require_samples = false) const;
  /// 执行起飞前的完整 MAVROS、传感器和估计器门禁。
  std::vector<std::string> preflight_errors(double now);
  /// 执行飞行期的完整安全门禁和水平漂移检查。
  std::vector<std::string> flight_errors(
    double now, double commanded_x_m, double commanded_y_m,
    bool require_offboard = true, bool at_hover = false);
  /// 判断光流是否在高度、估计器和质量条件下实际可用。
  bool optical_flow_effective(double now) const;
  /// 以当前本地 XY 建立飞行期漂移检测基线。
  void seed_drift_baseline(double now);
  /// 将完整遥测和派生状态序列化为严格 JSON 对象。
  std::string telemetry_json(double now) const;
  /// Capture one immutable telemetry and derived-health value for an application cycle.
  HealthSnapshot health_snapshot(
    double now, bool in_flight, double commanded_x_m, double commanded_y_m,
    bool require_offboard = false, bool at_hover = false);

private:
  /// 校验连接、心跳、SYS_STATUS 和预解锁健康掩码。
  std::vector<std::string> connection_errors(double now, bool require_standby);
  /// 校验电池新鲜度和阈值；飞行期允许未知电量比例。
  std::vector<std::string> battery_errors(double now, bool in_flight = false) const;
  /// 校验本地位姿、四元数和速度，并应用当前阶段速度限制。
  std::vector<std::string> pose_velocity_errors(double now, bool preflight, bool climbing = false) const;
  /// 校验 MAVROS 暴露的估计器状态位。
  std::vector<std::string> estimator_errors(double now) const;
  /// 校验测距和光流来源、新鲜度、质量及 RangeGuard 状态。
  std::vector<std::string> range_flow_errors(double now) const;
  /// 将 MAVROS state 回调转为内部遥测。
  void state_callback(const mavros_msgs::msg::State::SharedPtr message);
  /// 将 MAVROS SYS_STATUS 回调转为内部遥测。
  void sys_status_callback(const mavros_msgs::msg::SysStatus::SharedPtr message);
  /// 将 BatteryState 回调转为内部遥测。
  void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr message);
  /// 将 ExtendedState 回调转为落地状态。
  void extended_state_callback(const mavros_msgs::msg::ExtendedState::SharedPtr message);
  /// 将 PoseStamped 回调转为本地位姿。
  void local_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr message);
  /// 将 TwistStamped 回调转为本地速度。
  void local_velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr message);
  /// 将 EstimatorStatus 回调转为内部有效性标志。
  void estimator_callback(const mavros_msgs::msg::EstimatorStatus::SharedPtr message);
  /// 将 Range 回调交给测距守卫。
  void range_callback(const sensor_msgs::msg::Range::SharedPtr message);
  /// 将 OpticalFlowRad 回调转为内部光流遥测。
  void optical_flow_callback(const mavros_msgs::msg::OpticalFlowRad::SharedPtr message);
  /// 将 LCP UInt8 状态回调转为内部状态样本。
  void lcp_status_callback(const std_msgs::msg::UInt8::SharedPtr message);
  /// 将 LCP Odometry 回调转为平面位置和偏航。
  void lcp_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr message);

  rclcpp::Node & node_;
  const common::AppOptions & options_;
  const common::SafetyConfig & config_;
  common::Telemetry telemetry_{};
  RangeGuard range_guard_;
  std::optional<double> drift_baseline_x_m_;
  std::optional<double> drift_baseline_y_m_;
  std::optional<double> drift_baseline_at_;
  ServiceEvent lcp_start_event_{};
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr lcp_start_client_;
  std::optional<rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture> lcp_start_future_;
  std::optional<double> lcp_start_future_started_at_;

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_subscription_;
  rclcpp::Subscription<mavros_msgs::msg::SysStatus>::SharedPtr sys_status_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_subscription_;
  rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr extended_state_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr local_velocity_subscription_;
  rclcpp::Subscription<mavros_msgs::msg::EstimatorStatus>::SharedPtr estimator_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_subscription_;
  rclcpp::Subscription<mavros_msgs::msg::OpticalFlowRad>::SharedPtr optical_flow_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr lcp_status_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lcp_odometry_subscription_;
};

}  // namespace mavros_xyz_position_offboard::initialization
