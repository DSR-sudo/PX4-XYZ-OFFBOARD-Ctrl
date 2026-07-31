#pragma once

namespace mavros_xyz_position_offboard::navigation
{

/// 与部署 YAML 文件共享默认值的拦截投放任务参数。
struct MissionConfig
{
  double takeoff_height_m{1.5};
  double height_stable_seconds{3.0};
  double b_right_m{0.375};
  double b_forward_m{2.375};
  double throw_distance_m{0.20};
  double filter_measurement_noise_m{0.05};
  double filter_acceleration_noise_m_s2{0.50};
  int filter_min_samples{3};
  double prediction_horizon_s{2.0};
  double cardinal_tolerance_deg{5.0};
  double final_intercept_seconds{0.5};
  double car_status_timeout_s{2.0};
  double max_tracking_radius_m{5.0};

  /// 校验 B 点、卡尔曼滤波与投放门限参数。
  void validate() const;
};

}  // namespace mavros_xyz_position_offboard::navigation
