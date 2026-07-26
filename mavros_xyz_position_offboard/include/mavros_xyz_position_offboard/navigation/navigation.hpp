#pragma once

#include <array>
#include <optional>
#include <utility>
#include <vector>

#include "mavros_xyz_position_offboard/common/types.hpp"

namespace mavros_xyz_position_offboard::navigation
{

class Navigation
{
public:
  /// 使用共享安全配置创建纯值类型的导航规划器。
  explicit Navigation(const common::SafetyConfig & config);

  /// 清除锁存位置、轨迹和航点，使规划器回到未初始化状态。
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
  /// 按锁存偏航生成前、左、后、右四个方形航点。
  void prepare_waypoints();
  /// 激活下一个航点；所有航点完成时返回 false。
  bool start_next_waypoint();
  /// 判断规划已完成且飞行器实测位置进入航点容差。
  bool waypoint_reached(double vehicle_x_m, double vehicle_y_m) const;

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
  /// 返回当前已生成的方形航点序列。
  const std::vector<std::pair<double, double>> & waypoints() const {return waypoints_;}
  /// 返回当前激活航点的从零开始索引。
  int waypoint_index() const {return waypoint_index_;}

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
  std::vector<std::pair<double, double>> waypoints_;
  int waypoint_index_{-1};
};

}  // namespace mavros_xyz_position_offboard::navigation
