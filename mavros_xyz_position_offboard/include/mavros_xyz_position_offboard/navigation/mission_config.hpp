#pragma once

namespace mavros_xyz_position_offboard::navigation
{

/// 与部署 YAML 文件共享默认值的车辆中心投放任务参数。
struct MissionConfig
{
  double takeoff_height_m{1.5};
  double height_stable_seconds{3.0};
  /// 固定本地 ENU B 点的 X 分量，保留历史参数名以兼容部署配置。
  double b_right_m{0.375};
  /// 固定本地 ENU B 点的 Y 分量，保留历史参数名以兼容部署配置。
  double b_forward_m{2.375};
  /// B 点到达判定使用的水平合速度和垂直速度上限。
  double b_arrival_speed_m_s{0.05};
  /// car_status 车辆中心轨迹的二维合速度上限，单位：米/秒。
  double car_tracking_max_speed_m_s{10.0};
  /// car_status 车辆中心轨迹的二维合加速度上限，单位：米/秒²。
  double car_tracking_max_accel_m_s2{5.0};
  /// 返航原点轨迹的二维合速度上限，单位：米/秒。
  double return_max_speed_m_s{10.0};
  /// 返航原点轨迹的二维合加速度上限，单位：米/秒²。
  double return_max_accel_m_s2{5.0};
  /// 首条有效 car_status 后锁定动态目标并跟随的时长，单位：秒。
  double target_lock_follow_seconds{10.0};
  double throw_distance_m{0.20};
  double throw_bearing_rad{1.57079632679};
  double throw_bearing_tolerance_rad{0.08};
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
