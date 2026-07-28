#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "mavros_xyz_position_offboard/common/types.hpp"
#include "mavros_xyz_position_offboard/communication/protocol.hpp"

namespace mavros_xyz_position_offboard::navigation
{

/// Pure bounded XYZ+yaw trajectory generator. It has no task or protocol state.
class TrajectoryPlanner
{
public:
  /// 使用统一安全配置创建无任务状态的 XYZ+yaw 轨迹规划器。
  explicit TrajectoryPlanner(const common::SafetyConfig & config);

  /// 清除锁存位置和轨迹，使规划器回到未初始化状态。
  void reset();
  /// 锁存当前局部 XYZ 与姿态，作为后续相对控制的原点。
  void latch(double x_m, double y_m, double z_m, const common::Quaternion & orientation);
  /// ARM 后以当前实测 XY 重新建立水平原点。
  void recenter_xy(double x_m, double y_m);
  /// 以锁存 Z 原点为基准开始相对高度五次轨迹。
  void set_relative_target(double relative_z_m);
  /// 开始到世界坐标 XY 目标的有界五次轨迹。
  void set_xy_target(double x_m, double y_m);
  /// 将水平设定点立即冻结在指定实测位置。
  void freeze_xy_at(double x_m, double y_m);
  /// 在当前规划水平位置保持。
  void hold_xy();
  /// 开始到绝对 Z 目标的有界五次轨迹。
  void set_z_target(double z_m);
  /// 冻结当前 Z 设定点并清除垂直速度。
  void freeze_z();
  /// 规划返回锁存地面高度的下降轨迹。
  void set_ground_target();
  /// 将输出姿态改为指定世界偏航角。
  void set_yaw_rad(double yaw_rad);
  /// 同时开始到绝对 local ENU XYZ 目标的有界五次轨迹。
  void set_target(double x_m, double y_m, double z_m);

  /// 按 dt 推进 XY/Z 轨迹并返回下一条内部位置设定点。
  common::PositionSetpoint update(double dt_s);
  /// 返回当前设定点而不推进轨迹。
  common::PositionSetpoint current() const;
  /// 返回当前锁存或命令姿态的偏航角。
  double yaw_rad() const;

  /// 指示是否已锁存可用的初始局部位姿。
  bool latched() const {return latched_;}
  /// 指示当前水平五次轨迹是否已经结束。
  bool xy_target_reached() const {return xy_trajectory_duration_s_ <= 0.0;}
  /// 指示水平和垂直五次轨迹是否都已经结束。
  bool target_reached() const {return xy_trajectory_duration_s_ <= 0.0 && trajectory_duration_s_ <= 0.0;}
  /// 返回当前光流有效性标志，仅供状态日志使用。
  bool flow_effective() const {return flow_effective_;}
  /// 写入由 Initialization 判定的光流有效性状态。
  void set_flow_effective(bool effective) {flow_effective_ = effective;}
  /// 返回当前规划 X 设定点。
  double x_m() const {return x_m_;}
  /// 返回当前规划 Y 设定点。
  double y_m() const {return y_m_;}
  /// 返回锁存的 X 原点。
  double origin_x_m() const {return origin_x_m_;}
  /// 返回锁存的 Y 原点。
  double origin_y_m() const {return origin_y_m_;}
  /// 返回锁存的 Z 原点。
  double origin_z_m() const {return origin_z_m_;}
  /// 返回当前水平 X 目标。
  double target_x_m() const {return target_x_m_;}
  /// 返回当前水平 Y 目标。
  double target_y_m() const {return target_y_m_;}
  /// 返回当前垂直目标。
  double target_z_m() const {return target_z_m_;}
  /// 返回当前命令 Z 设定点。
  double command_z_m() const {return command_z_m_;}
  /// 返回当前命令垂直速度。
  double vertical_rate_m_s() const {return vertical_rate_m_s_;}
  /// 返回用于发布 yaw 的当前姿态四元数。
  const common::Quaternion & orientation() const {return orientation_;}

private:
  using Coefficients = std::array<double, 6>;
  /// 求解满足初始速度、终点静止条件的五次多项式系数。
  static Coefficients quintic_coefficients(double start, double rate, double target, double duration_s);
  /// 计算五次多项式在 t 时刻的位置、速度和加速度。
  static std::array<double, 3> evaluate(const Coefficients & coefficients, double t);
  /// 根据垂直速度/加速度约束初始化 Z 轨迹。
  void begin_z_trajectory(double target_z_m);
  /// 根据平面速度/加速度约束初始化 XY 轨迹。
  void begin_xy_trajectory(double target_x_m, double target_y_m);
  /// 确保调用者先锁存位姿，否则报告逻辑错误。
  void require_latched(const char * action) const;

  const common::SafetyConfig & config_;
  bool latched_{false};
  bool flow_effective_{false};
  double x_m_{NAN};
  double y_m_{NAN};
  double origin_x_m_{NAN};
  double origin_y_m_{NAN};
  double target_x_m_{NAN};
  double target_y_m_{NAN};
  double xy_velocity_x_m_s_{0.0};
  double xy_velocity_y_m_s_{0.0};
  double xy_trajectory_elapsed_s_{0.0};
  double xy_trajectory_duration_s_{0.0};
  std::array<Coefficients, 2> xy_coefficients_{};
  double origin_z_m_{NAN};
  double command_z_m_{NAN};
  double target_z_m_{NAN};
  double vertical_rate_m_s_{0.0};
  common::Quaternion orientation_{};
  double trajectory_elapsed_s_{0.0};
  double trajectory_duration_s_{0.0};
  Coefficients coefficients_{};
};

struct ControllerFeedback
{
  bool connected{false};
  bool armed{false};
  std::string mode{};
  std::string mode_request_status{"never_requested"};
  std::string arm_request_status{"never_requested"};
};

struct NavigationInput
{
  double now{0.0};
  double dt{0.05};
  common::Telemetry telemetry{};
  bool preflight_ready{false};
  bool flight_healthy{true};
  bool lcp_healthy{false};
  std::vector<std::string> health_errors{};
  std::vector<communication::ProtocolEvent> events{};
  ControllerFeedback controller{};
};

struct NavigationDecision
{
  std::string phase{"waiting_preflight"};
  std::optional<common::PositionSetpoint> setpoint{};
  std::optional<std::string> target_mode{};
  std::optional<bool> arm_intent{};
  std::vector<communication::OutgoingMessage> messages{};
  std::vector<std::string> rejections{};
  std::size_t waypoint_index{0};
  std::optional<double> ack_age_s{};
};

/// Pure mission state machine. It depends only on values and parsed protocol events.
class Navigation
{
public:
  /// 使用安全配置创建纯值类型任务状态机及其轨迹规划器。
  explicit Navigation(const common::SafetyConfig & config);
  /// 消费一个周期的健康、协议和飞控反馈，返回完整任务决策。
  NavigationDecision update(const NavigationInput & input);
  /// 清空任务、轨迹、批次、航点和 hold 上下文并返回预检等待阶段。
  void reset();

  /// 返回当前协议任务阶段名称。
  const std::string & phase() const {return phase_;}
  /// 返回只读轨迹规划器，供组装层读取原点和当前控制目标。
  const TrajectoryPlanner & planner() const {return planner_;}
  /// 返回当前活动航点索引；任务完成时等于航点总数。
  std::size_t waypoint_index() const {return waypoint_index_;}

private:
  /// 切换任务阶段、记录阶段开始时间并清除稳定计时。
  void transition(const std::string & phase, double now);
  /// 将一条待发送 GCS 业务消息加入本周期输出队列。
  void emit(communication::MessageType type);
  /// 将机器可读拒绝原因加入本周期状态输出。
  void reject(const std::string & reason);
  /// 按当前任务阶段处理已解析且去重的地面站协议事件。
  void process_events(const NavigationInput & input);
  /// 清空当前三类导航配置副本，避免残缺批次继续生效。
  void reset_batch();
  /// 校验三份配置一致性、构建航点队列并启动受支持任务。
  bool finish_batch(double now);
  /// 保存恢复目标并在指定 link/LCP hold 阶段冻结可靠实测位置。
  void enter_hold(const NavigationInput & input, const std::string & hold_phase);
  /// 从冻结位置连续重规划到保存目标并恢复被中断阶段。
  void resume_hold(double now);
  /// 固定水平位置、规划返回初始化 Z，并进入正常或故障降落阶段。
  void begin_landing(double now, const std::string & reason = {});
  /// 判断 MAVROS 实测 XY 到指定目标的欧氏距离是否不超过容差。
  bool actual_xy_within(const common::Telemetry & telemetry, double x, double y, double tolerance) const;
  /// 判断实测 XYZ 是否位于稳定阶段要求的位置容差内。
  bool stable_at(const common::Telemetry & telemetry, const common::PositionSetpoint & target) const;

  const common::SafetyConfig & config_;
  TrajectoryPlanner planner_;
  std::string phase_{"waiting_preflight"};
  double phase_started_at_{0.0};
  bool preflight_sent_{false};
  double takeoff_height_m_{0.0};
  std::vector<communication::ProtocolEvent> and_point_packets_{};
  std::vector<communication::ProtocolEvent> nfz_packets_{};
  std::vector<communication::ProtocolEvent> plan_packets_{};
  std::vector<communication::Point3> waypoints_{};
  std::size_t waypoint_index_{0};
  std::optional<double> last_ack_at_{};
  std::optional<double> stable_since_{};
  std::optional<double> flight_started_at_{};
  std::optional<communication::Point3> held_target_{};
  std::string hold_resume_phase_{};
  std::vector<communication::OutgoingMessage> pending_messages_{};
  std::vector<std::string> pending_rejections_{};
  std::string landing_reason_{};
};

}  // namespace mavros_xyz_position_offboard::navigation
