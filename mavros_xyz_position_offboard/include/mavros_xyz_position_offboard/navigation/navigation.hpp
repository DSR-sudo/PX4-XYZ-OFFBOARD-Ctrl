#pragma once

#include <optional>
#include <string>

#include "mavros_xyz_position_offboard/navigation/mission_config.hpp"
#include "mavros_xyz_position_offboard/navigation/navigation_types.hpp"
#include "mavros_xyz_position_offboard/navigation/trajectory_planner.hpp"

namespace mavros_xyz_position_offboard::navigation
{

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
  /// 从冻结位置连续重规划到任务最终目标。
  void resume_hold(double now);
  /// 固定水平位置、规划返回初始化 Z，并进入正常或故障降落阶段。
  void begin_landing(double now, const std::string & reason = {});
  /// 判断 MAVROS 实测 XY 到指定目标的欧氏距离是否不超过容差。
  bool actual_xy_within(
    const common::Telemetry & telemetry, double x, double y, double tolerance) const;
  /// 判断实测 XYZ 是否位于稳定阶段要求的位置容差内。
  bool stable_at(
    const common::Telemetry & telemetry, const common::PositionSetpoint & target) const;
  /// 判断 B 点实测 XYZ 位置和水平/垂直速度是否都满足到达门限。
  bool b_arrival_stable(
    const common::Telemetry & telemetry, const common::PositionSetpoint & target) const;
  /// 判断实测偏航角是否已接近目标世界航向。
  bool actual_yaw_within(
    const common::Telemetry & telemetry, double yaw_rad, double tolerance_rad) const;
  /// 从当前爬升目标规划到固定本地 ENU B 点的一次性二维轨迹。
  void begin_transit_to_b();
  /// 将车体相对视觉测量转换为 ENU，并把车辆中心作为当前 XY 目标。
  bool apply_car_status(const NavigationInput & input, const communication::CarStatus & status);
  /// 开始锁定同一 car_status 数据流并清零可暂停的跟随计时。
  void begin_target_lock_follow(double now);
  /// 暂停锁定跟随的有效计时，用于 LCP/视觉安全保持。
  void pause_target_lock_follow(double now);
  /// 从安全保持恢复锁定跟随计时。
  void resume_target_lock_follow(double now);
  /// 返回截至当前时刻已经消耗的有效锁定跟随时长。
  double target_lock_follow_elapsed(double now) const;
  /// 判断最近一次有效视觉测量在当前控制周期仍然新鲜。
  bool car_status_fresh(double now) const;
  /// 取消投掷计时并开始从当前高度返回锁存原点。
  void begin_return(double now);
  /// 判断阶段是否必须因 LCP 不健康冻结位置。
  bool lcp_required_in_phase() const;

  const common::SafetyConfig & config_;
  const MissionConfig mission_;
  TrajectoryPlanner planner_;
  std::string phase_{"waiting_preflight"};
  double phase_started_at_{0.0};
  std::optional<double> flight_started_at_{};
  bool normal_completion_{false};
  ControlState control_{};
  std::vector<communication::OutgoingMessage> pending_messages_{};
  std::vector<std::string> pending_rejections_{};
  std::string landing_reason_{};
  bool pending_release_gripper_{false};
  std::optional<double> last_car_status_at_{};
  std::optional<communication::CarStatus> latest_car_status_{};
  double target_lock_follow_elapsed_s_{0.0};
  std::optional<double> target_lock_follow_started_at_{};
  bool target_lock_follow_completed_{false};
};

}  // namespace mavros_xyz_position_offboard::navigation
