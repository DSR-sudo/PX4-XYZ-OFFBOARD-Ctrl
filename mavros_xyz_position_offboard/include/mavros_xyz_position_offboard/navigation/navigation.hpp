#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "mavros_xyz_position_offboard/common/types.hpp"
#include "mavros_xyz_position_offboard/communication/protocol.hpp"

namespace mavros_xyz_position_offboard::navigation
{

/// 纯值类型的有界 XYZ 加偏航轨迹生成器，不包含任务或协议状态。
///
/// Navigation owns the mission origin and final goal. This class only advances the
/// commanded point from its current value to a target supplied by its caller.
class TrajectoryPlanner
{
public:
  /// 使用统一安全配置创建无任务状态的 XYZ 加偏航轨迹规划器。
  explicit TrajectoryPlanner(const common::SafetyConfig & config);

  /// 清除锁存位置和轨迹，使规划器回到未初始化状态。
  void reset();
  /// 以当前局部 XYZ 与姿态初始化轨迹的命令起点。
  void latch(double x_m, double y_m, double z_m, const common::Quaternion & orientation);
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
  /// 立即将 Z 设定点冻结在指定实测高度并清除垂直速度。
  void freeze_z_at(double z_m);
  /// 将输出姿态改为指定世界偏航角。
  void set_yaw_rad(double yaw_rad);
  /// 同时开始到绝对本地 ENU XYZ 目标的有界五次轨迹。
  void set_target(double x_m, double y_m, double z_m);

  /// 按 dt 推进 XY/Z 轨迹并返回下一条内部位置设定点。
  common::PositionSetpoint update(double dt_s);
  /// 返回当前设定点而不推进轨迹。
  common::PositionSetpoint current() const;
  /// 返回当前锁存或命令姿态的偏航角。
  double yaw_rad() const;

  /// 指示是否已初始化可用的命令起点。
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
  /// 返回用于发布偏航角的当前姿态四元数。
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
  double target_x_m_{NAN};
  double target_y_m_{NAN};
  double xy_velocity_x_m_s_{0.0};
  double xy_velocity_y_m_s_{0.0};
  double xy_trajectory_elapsed_s_{0.0};
  double xy_trajectory_duration_s_{0.0};
  std::array<Coefficients, 2> xy_coefficients_{};
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

/// 与部署 YAML 文件共享默认值的任务参数。
struct MissionConfig
{
  double takeoff_height_m{1.5};
  double height_stable_seconds{3.0};
  double standoff_m{0.10};
  double match_tolerance_m{0.10};
  double car_status_timeout_s{0.5};
  double max_distance_m{5.0};

  /// 校验起飞、跟车和距离限制参数均为有效的正值。
  void validate() const;
};

/// 导航状态机在一个控制周期内消费的健康、协议和飞控反馈。
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
  bool gripper_succeeded{false};
  bool gripper_failed{false};
};

/// 任务语义与实际发布命令严格分离的可审计控制状态。
struct ControlState
{
  /// 解锁确认后一次性锁定的本地 ENU 飞行原点；预热期间为空。
  std::optional<common::PositionSetpoint> origin{};
  /// 当前任务最终 XYZ+yaw；临时保持永不改写它。
  std::optional<common::PositionSetpoint> mission_goal{};
  /// 最近一次交给 Offboard 发布的 ENU XYZ+yaw 命令。
  std::optional<common::PositionSetpoint> commanded_setpoint{};
  /// LCP 或跟车数据失效时锁定的实测 XYZ 与最近命令偏航。
  std::optional<common::PositionSetpoint> hold_setpoint{};
  std::string hold_reason{};
  std::string hold_resume_phase{};
  bool mission_paused{false};
};

/// 将完整控制状态编码为 JSON 对象，供 JSONL 审计和单元测试共同使用。
std::string control_json(const ControlState & control);

/// 导航状态机为当前控制周期生成的控制与通信决策。
struct NavigationDecision
{
  std::string phase{"waiting_preflight"};
  std::optional<common::PositionSetpoint> setpoint{};
  std::optional<std::string> target_mode{};
  std::optional<bool> arm_intent{};
  std::vector<communication::OutgoingMessage> messages{};
  std::vector<std::string> rejections{};
  bool release_gripper{false};
  ControlState control{};
};

/// 纯值类型任务状态机，只依赖输入值和已解析的协议事件。
class Navigation
{
public:
  /// 使用安全配置创建纯值类型任务状态机及其轨迹规划器。
  explicit Navigation(const common::SafetyConfig & config, MissionConfig mission = {});
  /// 消费一个周期的健康、协议和飞控反馈，返回完整任务决策。
  NavigationDecision update(const NavigationInput & input);
  /// 清空任务、轨迹和保持上下文，并返回预检等待阶段。
  void reset();

  /// 返回当前协议任务阶段名称。
  const std::string & phase() const {return phase_;}
  /// 返回只读轨迹规划器，供测试和组装层读取当前命令轨迹。
  const TrajectoryPlanner & planner() const {return planner_;}
  /// 返回完整控制状态，供健康检查与审计读取。
  const ControlState & control_state() const {return control_;}
  /// 返回最近一次实际发布的命令点；没有命令流时为空。
  const std::optional<common::PositionSetpoint> & commanded_setpoint() const
  {
    return control_.commanded_setpoint;
  }

private:
  /// 切换任务阶段并记录新阶段的开始时间。
  void transition(const std::string & phase, double now);
  /// 将一条待发送 GCS 业务消息加入本周期输出队列。
  void emit(communication::MessageType type);
  /// 将机器可读拒绝原因加入本周期状态输出。
  void reject(const std::string & reason);
  /// 按当前任务阶段处理已解析且去重的地面站协议事件。
  void process_events(const NavigationInput & input);
  /// 更新最终任务目标；只有明确的任务事件可以调用它。
  void set_mission_goal(const common::PositionSetpoint & goal);
  /// 从当前命令点重新规划到任务最终目标。
  void plan_to_mission_goal();
  /// 使用新鲜实测 XYZ 和最近命令偏航构建临时保持点。
  common::PositionSetpoint measured_hold_setpoint(const NavigationInput & input) const;
  /// 在指定保持阶段冻结可靠实测位置，且绝不改写任务最终目标。
  void enter_hold(
    const NavigationInput & input, const std::string & hold_phase,
    const std::string & reason, const std::optional<std::string> & resume_phase = std::nullopt);
  /// 清除保持元数据，不触碰任务最终目标。
  void clear_hold();
  /// 从冻结位置连续重规划到任务最终目标，或恢复等待新车辆状态。
  void resume_hold(double now);
  /// 固定水平位置、规划返回初始化 Z，并进入正常或故障降落阶段。
  void begin_landing(double now, const std::string & reason = {});
  /// 判断 MAVROS 实测 XY 到指定目标的欧氏距离是否不超过容差。
  bool actual_xy_within(const common::Telemetry & telemetry, double x, double y, double tolerance) const;
  /// 判断实测 XYZ 是否位于稳定阶段要求的位置容差内。
  bool stable_at(const common::Telemetry & telemetry, const common::PositionSetpoint & target) const;
  /// 判断实测偏航角是否已接近目标世界航向。
  bool actual_yaw_within(const common::Telemetry & telemetry, double yaw_rad, double tolerance_rad) const;
  /// 依据最新 car_status 生成一次受安全半径保护的 ENU 跟踪任务最终目标。
  bool apply_car_target(const NavigationInput & input);
  /// 判断最后一条车辆状态在本周期是否仍然新鲜。
  bool car_status_fresh(double now) const;
  /// 判断阶段是否必须因 LCP 不健康冻结位置。
  bool lcp_required_in_phase() const;

  const common::SafetyConfig & config_;
  const MissionConfig mission_;
  TrajectoryPlanner planner_;
  std::string phase_{"waiting_preflight"};
  double phase_started_at_{0.0};
  std::optional<double> flight_started_at_{};
  std::optional<communication::CarStatus> latest_car_status_{};
  std::optional<double> latest_car_status_at_{};
  bool car_target_pending_{false};
  bool normal_completion_{false};
  ControlState control_{};
  std::vector<communication::OutgoingMessage> pending_messages_{};
  std::vector<std::string> pending_rejections_{};
  std::string landing_reason_{};
  bool pending_release_gripper_{false};
};

}  // mavros_xyz_position_offboard::navigation 命名空间
