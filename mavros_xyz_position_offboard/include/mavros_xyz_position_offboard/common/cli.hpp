#pragma once

#include <string>
#include <vector>

#include "mavros_xyz_position_offboard/common/types.hpp"

namespace mavros_xyz_position_offboard::common
{

struct AppOptions
{
  std::string confirmed_fcu_url;
  std::string range_topic;
  std::string range_source_label;
  std::string optical_flow_topic;
  std::string optical_flow_source_label;
  std::string state_topic{"/mavros/state"};
  std::string sys_status_topic{"/mavros/sys_status"};
  std::string battery_topic{"/mavros/battery"};
  std::string extended_state_topic{"/mavros/extended_state"};
  std::string local_pose_topic{"/mavros/local_position/pose"};
  std::string local_velocity_topic{"/mavros/local_position/velocity_local"};
  std::string estimator_status_topic{"/mavros/estimator_status"};
  std::string setpoint_topic{"/mavros/setpoint_raw/local"};
  std::string lcp_start_service{"/lcp/start_initialization"};
  std::string lcp_status_topic{"/lcp/status"};
  std::string lcp_odometry_topic{"/lcp/odometry"};
  std::string output{"summary"};
  std::string artifact_dir{"artifacts"};
  std::string px4_xy_fusion_evidence_label;
  double status_period{0.5};
  double mode_request_interval{2.0};
  double service_timeout{3.0};
  bool enable_position_setpoints{false};
  bool ack_native_xyz_position_control{false};
  bool ack_setpoint_streaming_risk{false};
  bool confirm_setpoint_mav_frame_local_ned{false};
  bool confirm_range_source{false};
  bool confirm_optical_flow_source{false};
  bool request_offboard_mode{false};
  bool ack_disarmed_mode_switch{false};
  bool execute_bounded_flight{false};
  bool ack_normal_arm_only{false};
  bool ack_propeller_configuration_safe{false};
  bool ack_area_and_personnel_clear{false};
  bool ack_independent_emergency_stop_ready{false};
  bool ack_valid_flight_battery_installed{false};
  bool ack_direct_px4_xy_fusion_evidence{false};
  bool ack_range_below_declared_min{false};
  bool ignore_declared_min_range{true};
};

struct ParsedOptions
{
  AppOptions options;
  SafetyConfig config;
};

/// 解析非 ROS 参数、应用默认值和全部确认门禁，失败时抛出异常。
ParsedOptions parse_options(const std::vector<std::string> & argv);
/// 判断 PositionTarget 发布所需的全部显式确认是否齐全。
bool setpoint_enabled(const AppOptions & options);
/// 判断 OFFBOARD 模式请求所需确认是否齐全。
bool mode_enabled(const AppOptions & options);
/// 判断受限飞行与解锁所需确认是否齐全。
bool arming_enabled(const AppOptions & options);
/// 返回命令行入口的简要用法文本。
std::string usage();

}  // namespace mavros_xyz_position_offboard::common
