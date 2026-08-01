#include "mavros_xyz_position_offboard/navigation/navigation_types.hpp"

#include <iomanip>
#include <sstream>

#include "mavros_xyz_position_offboard/common/artifact_log.hpp"

namespace mavros_xyz_position_offboard::navigation
{

/// 用与 Offboard 相同的 ENU XYZ+yaw 表达审计每一个控制状态点。
std::string control_json(const ControlState & control)
{
  const auto setpoint_json = [](const std::optional<common::PositionSetpoint> & setpoint) {
      if (!setpoint) {return std::string("null");}
      std::ostringstream encoded;
      encoded << std::setprecision(12) << "{\"x_m\":" << setpoint->x_m
              << ",\"y_m\":" << setpoint->y_m << ",\"z_m\":" << setpoint->z_m
              << ",\"yaw_rad\":" << common::yaw_from_quaternion(setpoint->orientation)
              << ",\"vertical_rate_m_s\":" << setpoint->vertical_rate_m_s << '}';
      return encoded.str();
    };
  std::ostringstream encoded;
  encoded << "{\"origin\":" << setpoint_json(control.origin)
          << ",\"mission_goal\":" << setpoint_json(control.mission_goal)
          << ",\"commanded_setpoint\":" << setpoint_json(control.commanded_setpoint)
          << ",\"hold_setpoint\":" << setpoint_json(control.hold_setpoint)
          << ",\"hold_reason\":\"" << common::json_escape(control.hold_reason)
          << "\",\"hold_resume_phase\":\"" << common::json_escape(control.hold_resume_phase)
          << "\",\"mission_paused\":" << (control.mission_paused ? "true" : "false")
          << ",\"tracking_z_offset_m\":" << control.tracking_z_offset_m
          << ",\"tracking_z_last_jump_direction\":\"" <<
    common::json_escape(control.tracking_z_last_jump_direction) << "\""
          << ",\"tracking_arrival_time_met\":" <<
    (control.tracking_arrival_time_met ? "true" : "false")
          << ",\"target_samples\":" << control.target_samples
          << ",\"predicted_intercept_seconds\":";
  if (control.predicted_intercept_seconds) {encoded << *control.predicted_intercept_seconds;}
  else {encoded << "null";}
  encoded << '}';
  return encoded.str();
}

}  // namespace mavros_xyz_position_offboard::navigation
