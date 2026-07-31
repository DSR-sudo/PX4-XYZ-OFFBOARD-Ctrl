#pragma once

#include <array>
#include <optional>

#include "mavros_xyz_position_offboard/navigation/mission_config.hpp"

namespace mavros_xyz_position_offboard::navigation
{

/// 目标的 ENU 匀速卡尔曼滤波输出。
struct TargetEstimate
{
  bool initialized{false};
  int samples{0};
  double x_m{0.0};
  double y_m{0.0};
  double vx_m_s{0.0};
  double vy_m_s{0.0};
};

/// 不依赖 ROS 的二维常速度目标滤波器，测量与状态均在 ENU 世界系。
class TargetTracker
{
public:
  explicit TargetTracker(const MissionConfig & config);
  void reset();
  /// 吸收一个世界系位置测量；创新超过 99% 二维门限时返回 false。
  bool update(double x_m, double y_m, double received_at);
  TargetEstimate estimate(double at) const;
  /// 求相对直线运动第一次进入给定半径的时刻，超出 horizon 返回空。
  std::optional<double> time_to_distance(
    double own_x_m, double own_y_m, double own_vx_m_s, double own_vy_m_s,
    double distance_m, double now, double horizon_s) const;
  bool ready() const {return initialized_ && samples_ >= config_.filter_min_samples;}
  int samples() const {return samples_;}

private:
  void predict_to(double at);

  const MissionConfig & config_;
  bool initialized_{false};
  int samples_{0};
  double state_time_{0.0};
  std::array<double, 4> state_{};
  std::array<std::array<double, 4>, 4> covariance_{};
};

}  // namespace mavros_xyz_position_offboard::navigation
