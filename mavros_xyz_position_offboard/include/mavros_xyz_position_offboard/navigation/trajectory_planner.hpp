#pragma once

#include <array>
#include <optional>

#include "mavros_xyz_position_offboard/common/types.hpp"

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
  /// 在调用方给定的水平速度上限内开始 XY 轨迹，仍使用统一加速度约束。
  void set_xy_target_with_max_speed(double x_m, double y_m, double max_speed_m_s);
  /// 使用调用方给定的二维合速度和合加速度上限开始 XY 轨迹。
  void set_xy_target_with_limits(
    double x_m, double y_m, double max_speed_m_s, double max_accel_m_s2);
  /// 以期望到达时间重规划 XY；返回 false 表示安全约束要求更长时间。
  bool set_xy_target_with_arrival_time(double x_m, double y_m, double arrival_seconds);
  /// 将水平设定点立即冻结在指定实测位置。
  void freeze_xy_at(double x_m, double y_m);
  /// 在当前规划水平位置保持。
  void hold_xy();
  /// 开始到绝对 Z 目标的有界五次轨迹。
  void set_z_target(double z_m);
  /// 使用调用方给定的 Z 轴速度和加速度上限开始五次轨迹。
  void set_z_target_with_limits(
    double z_m, double max_speed_m_s, double max_accel_m_s2);
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
  /// 返回当前 XY 轨迹的计划持续时间。
  double xy_trajectory_duration_s() const {return xy_trajectory_duration_s_;}
  /// 返回最近一次带到达时间约束的重规划是否满足该时间。
  bool xy_arrival_time_met() const {return xy_arrival_time_met_;}
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
  /// 根据给定的垂直速度/加速度约束初始化 Z 轨迹。
  void begin_z_trajectory(double target_z_m, double max_speed_m_s, double max_accel_m_s2);
  /// 根据平面速度/加速度约束初始化 XY 轨迹。
  bool begin_xy_trajectory(
    double target_x_m, double target_y_m,
    const std::optional<double> & arrival_seconds, double max_speed_m_s,
    double max_accel_m_s2);
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
  bool xy_arrival_time_met_{true};
  std::array<Coefficients, 2> xy_coefficients_{};
  double command_z_m_{NAN};
  double target_z_m_{NAN};
  double vertical_rate_m_s_{0.0};
  common::Quaternion orientation_{};
  double trajectory_elapsed_s_{0.0};
  double trajectory_duration_s_{0.0};
  Coefficients coefficients_{};
};

}  // namespace mavros_xyz_position_offboard::navigation
