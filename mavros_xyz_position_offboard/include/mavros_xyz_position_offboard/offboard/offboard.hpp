#pragma once

#include <chrono>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mavros_xyz_position_offboard/common/artifact_log.hpp"
#include "mavros_xyz_position_offboard/common/cli.hpp"
#include "mavros_xyz_position_offboard/initialization/initialization.hpp"
#include "mavros_xyz_position_offboard/navigation/navigation.hpp"

namespace mavros_xyz_position_offboard::offboard
{

class MavrosNativeXYZNode final : public rclcpp::Node
{
public:
  /// 创建唯一 ROS 节点，并组装 Initialization、Navigation 与 OFFBOARD 职责。
  MavrosNativeXYZNode(const common::AppOptions & options, const common::SafetyConfig & config);
  /// 关闭节点拥有的飞行日志。
  ~MavrosNativeXYZNode() override;

  /// 将内部 PositionSetpoint 映射为 MAVROS FRAME_LOCAL_NED 的 PositionTarget。
  static mavros_msgs::msg::PositionTarget make_position_target(
    const common::PositionSetpoint & setpoint, const builtin_interfaces::msg::Time & stamp);

private:
  struct ModeEvent
  {
    std::string status{"not_enabled"};
    std::optional<std::string> mode{};
    std::optional<bool> mode_sent{};
    std::optional<std::string> detail{};
    std::optional<double> monotonic_s{};
  };
  struct ArmEvent
  {
    std::string status{"not_enabled"};
    std::optional<bool> value{};
    std::optional<bool> success{};
    std::optional<int> result{};
    std::optional<std::string> detail{};
    std::optional<double> monotonic_s{};
  };

  /// 读取进程单调时钟，供控制循环和超时判断使用。
  static double monotonic_now();
  /// 将模式字符串转换为大写以便进行大小写无关比较。
  static std::string upper(std::string value);
  /// 执行一次按固定 wall timer 触发的控制循环。
  void tick();
  /// 在只监视模式运行预检并更新阶段。
  void monitor_tick(double now);
  /// 执行预检、LCP 初始化、设定点预热、模式请求和解锁请求。
  void prearm_control_tick(double now, double dt_s);
  /// 执行爬升、悬停、航点、LCP 保持和降落状态机。
  void flight_tick(double now, double dt_s);
  /// 清除当前预检候选及所有未完成的控制状态。
  void reset_candidate(double now);
  /// 从 Initialization 遥测锁存 Navigation 的初始位姿。
  void latch_current_pose();
  /// 推进 Navigation 并由 OFFBOARD 独占发布一条 MAVROS 设定点。
  void publish_setpoint(double dt_s);
  /// 轮询 LCP、SetMode 和 CommandBool 异步请求的响应或超时。
  void poll_service_futures(double now);
  /// 在限频和服务就绪条件满足时请求飞控模式。
  void request_mode(double now, const std::string & mode);
  /// 在限频和服务就绪条件满足时请求普通解锁或上锁。
  void request_arm(double now, bool value);
  /// 固定 XY、规划下降并进入安全降落阶段。
  void begin_landing(double now, const std::optional<std::string> & reason = std::nullopt);
  /// 在航点期间 LCP 失效时冻结位置并保存恢复上下文。
  void enter_lcp_hold(double now, const std::string & reason);
  /// LCP 恢复后重新规划被中断的目标并恢复原飞行阶段。
  void resume_lcp_hold(double now);
  /// 生成预解锁阶段使用的 LCP 门禁错误列表。
  std::vector<std::string> lcp_prearm_errors(double now) const;
  /// 生成完整状态 JSONL 记录。
  std::string status_json(double now) const;
  /// 生成人类可读的终端和摘要日志文本。
  std::string summary(double now) const;
  /// 同时向终端和配置的 artifact 日志输出状态。
  void emit_status(double now);
  /// 将错误列表编码为 JSON 字符串数组。
  std::string errors_json(const std::vector<std::string> & errors) const;
  /// 序列化最后一次 SetMode 服务事件。
  std::string mode_event_json() const;
  /// 序列化最后一次 CommandBool 服务事件。
  std::string arm_event_json() const;
  /// 序列化最后一次 LCP 初始化服务事件。
  std::string lcp_event_json() const;

  const common::AppOptions options_;
  const common::SafetyConfig config_;
  initialization::Initialization initialization_;
  navigation::Navigation navigation_;
  common::ArtifactLogger artifact_log_;
  bool publish_enabled_{false};
  bool mode_enabled_{false};
  bool arming_enabled_{false};
  std::string phase_{"monitor_only"};
  std::string last_phase_;
  std::string result_{"UNCONFIRMED"};
  std::optional<std::string> abort_reason_;
  std::optional<double> phase_started_at_;
  std::optional<double> flight_started_at_;
  std::optional<double> setpoint_stream_since_;
  std::optional<double> last_tick_at_;
  double last_status_at_{-INFINITY};
  std::vector<std::string> last_errors_;
  std::optional<double> sensor_loss_started_at_;
  std::optional<double> lcp_hold_started_at_;
  std::optional<std::string> lcp_hold_resume_phase_;
  std::optional<double> lcp_hold_resume_phase_started_at_;
  std::optional<double> lcp_hold_resume_target_x_m_;
  std::optional<double> lcp_hold_resume_target_y_m_;
  std::optional<double> lcp_hold_resume_target_z_m_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_publisher_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  std::optional<rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture> mode_future_;
  std::optional<rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture> arm_future_;
  std::optional<double> mode_future_started_at_;
  std::optional<double> arm_future_started_at_;
  double last_mode_request_at_{-INFINITY};
  double last_arm_request_at_{-INFINITY};
  ModeEvent last_mode_event_{};
  ArmEvent last_arm_event_{};
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mavros_xyz_position_offboard::offboard
