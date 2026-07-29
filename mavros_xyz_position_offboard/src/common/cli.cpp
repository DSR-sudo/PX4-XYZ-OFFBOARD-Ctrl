#include "mavros_xyz_position_offboard/common/cli.hpp"

#include <algorithm>
#include <charconv>
#include <cctype>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace mavros_xyz_position_offboard::common
{
namespace
{
/// 判断字符串是否为空或仅包含空白字符。
bool blank(const std::string & value)
{
  return std::all_of(value.begin(), value.end(), [](unsigned char c) {return std::isspace(c) != 0;});
}

/// 解析并校验一个有限的浮点命令行值。
double parse_double(const std::string & name, const std::string & value)
{
  std::size_t parsed = 0;
  try {
    const double result = std::stod(value, &parsed);
    if (parsed != value.size() || !finite(result)) {throw std::invalid_argument("invalid");}
    return result;
  } catch (const std::exception &) {
    throw std::invalid_argument(name + " requires a finite number");
  }
}

/// 解析不接受尾随字符的整数命令行值。
int parse_int(const std::string & name, const std::string & value)
{
  int result{};
  const auto [end, error] = std::from_chars(value.data(), value.data() + value.size(), result);
  if (error != std::errc{} || end != value.data() + value.size()) {
    throw std::invalid_argument(name + " requires an integer");
  }
  return result;
}

/// 取得 --option=value 或紧随选项的必需参数值。
std::string require_value(
  const std::vector<std::string> & argv, std::size_t & index, const std::string & option,
  const std::optional<std::string> & assigned)
{
  if (assigned) {return *assigned;}
  if (++index >= argv.size()) {throw std::invalid_argument(option + " requires a value");}
  return argv[index];
}

}  // namespace

/// 汇总发布 PositionTarget 所需的全部确认位。
bool setpoint_enabled(const AppOptions & o)
{
  return o.enable_position_setpoints && o.ack_native_xyz_position_control &&
         o.ack_setpoint_streaming_risk && o.confirm_setpoint_mav_frame_local_ned &&
         o.confirm_range_source && o.confirm_optical_flow_source;
}

/// 汇总在设定点确认之上请求 OFFBOARD 所需的确认位。
bool mode_enabled(const AppOptions & o)
{
  return setpoint_enabled(o) && o.request_offboard_mode && o.ack_disarmed_mode_switch;
}

/// 汇总在模式确认之上执行解锁飞行所需的确认位。
bool arming_enabled(const AppOptions & o)
{
  return mode_enabled(o) && o.execute_bounded_flight && o.ack_normal_arm_only &&
         o.ack_propeller_configuration_safe && o.ack_area_and_personnel_clear &&
         o.ack_independent_emergency_stop_ready && o.ack_valid_flight_battery_installed &&
         o.ack_direct_px4_xy_fusion_evidence;
}

/// 返回程序启动或 --help 使用的最短命令行提示。
std::string usage()
{
  return "Usage: mavros_xyz_position_node --confirmed-fcu-url URL --range-topic TOPIC "
         "--range-source-label LABEL --optical-flow-topic TOPIC --optical-flow-source-label LABEL [options]";
}

/// 解析、映射和验证所有应用参数及安全默认值。
ParsedOptions parse_options(const std::vector<std::string> & argv)
{
  AppOptions o;
  SafetyConfig c;
  // These are application defaults, deliberately narrower than the reusable SafetyConfig defaults.
  c.max_z_setpoint_rate_m_s = 0.20;
  c.max_z_setpoint_accel_m_s2 = 0.40;
  c.max_flight_horizontal_speed_m_s = 0.50;
  c.max_flight_vertical_speed_m_s = 0.80;
  c.max_flight_horizontal_drift_m = 1.0;
  std::unordered_map<std::string, bool *> flags{
    {"--enable-position-setpoints", &o.enable_position_setpoints},
    {"--ack-native-xyz-position-control", &o.ack_native_xyz_position_control},
    {"--ack-setpoint-streaming-risk", &o.ack_setpoint_streaming_risk},
    {"--confirm-setpoint-mav-frame-local-ned", &o.confirm_setpoint_mav_frame_local_ned},
    {"--confirm-range-source", &o.confirm_range_source},
    {"--confirm-optical-flow-source", &o.confirm_optical_flow_source},
    {"--request-offboard-mode", &o.request_offboard_mode},
    {"--ack-disarmed-mode-switch", &o.ack_disarmed_mode_switch},
    {"--execute-bounded-flight", &o.execute_bounded_flight},
    {"--ack-normal-arm-only", &o.ack_normal_arm_only},
    {"--ack-propeller-configuration-safe", &o.ack_propeller_configuration_safe},
    {"--ack-area-and-personnel-clear", &o.ack_area_and_personnel_clear},
    {"--ack-independent-emergency-stop-ready", &o.ack_independent_emergency_stop_ready},
    {"--ack-valid-flight-battery-installed", &o.ack_valid_flight_battery_installed},
    {"--ack-direct-px4-xy-fusion-evidence", &o.ack_direct_px4_xy_fusion_evidence},
    {"--ack-range-below-declared-min", &o.ack_range_below_declared_min},
  };
  /// 将一个字符串型 CLI 选项路由到对应的 AppOptions 字段。
  const auto set_string = [&](const std::string & option, const std::string & value) {
      if (option == "--confirmed-fcu-url") {o.confirmed_fcu_url = value;}
      else if (option == "--range-topic") {o.range_topic = value;}
      else if (option == "--range-source-label") {o.range_source_label = value;}
      else if (option == "--optical-flow-topic") {o.optical_flow_topic = value;}
      else if (option == "--optical-flow-source-label") {o.optical_flow_source_label = value;}
      else if (option == "--state-topic") {o.state_topic = value;}
      else if (option == "--sys-status-topic") {o.sys_status_topic = value;}
      else if (option == "--battery-topic") {o.battery_topic = value;}
      else if (option == "--extended-state-topic") {o.extended_state_topic = value;}
      else if (option == "--local-pose-topic") {o.local_pose_topic = value;}
      else if (option == "--local-velocity-topic") {o.local_velocity_topic = value;}
      else if (option == "--estimator-status-topic") {o.estimator_status_topic = value;}
      else if (option == "--setpoint-topic") {o.setpoint_topic = value;}
      else if (option == "--lcp-start-service") {o.lcp_start_service = value;}
      else if (option == "--lcp-status-topic") {o.lcp_status_topic = value;}
      else if (option == "--lcp-odometry-topic") {o.lcp_odometry_topic = value;}
      else if (option == "--lcp-vision-pose-topic") {o.lcp_vision_pose_topic = value;}
      else if (option == "--lcp-vision-input-frame") {o.lcp_vision_input_frame = value;}
      else if (option == "--output") {o.output = value;}
      else if (option == "--artifact-dir") {o.artifact_dir = value;}
      else if (option == "--px4-xy-fusion-evidence-label") {o.px4_xy_fusion_evidence_label = value;}
      else {throw std::invalid_argument("unknown option: " + option);}
    };
  /// 将一个数值型 CLI 选项解析后路由到 AppOptions 或 SafetyConfig 字段。
  const auto set_number = [&](const std::string & option, const std::string & value) {
      const double number = parse_double(option, value);
      if (option == "--status-period") {o.status_period = number;}
      else if (option == "--publish-rate") {c.publish_rate_hz = number;}
      else if (option == "--setpoint-warmup") {c.setpoint_warmup_s = number;}
      else if (option == "--relative-z") {c.relative_z_m = number;}
      else if (option == "--max-z-setpoint-rate") {c.max_z_setpoint_rate_m_s = number;}
      else if (option == "--max-z-setpoint-accel") {c.max_z_setpoint_accel_m_s2 = number;}
      else if (option == "--target-xy-max-speed") {c.target_xy_max_speed_m_s = number;}
      else if (option == "--target-xy-max-accel") {c.target_xy_max_accel_m_s2 = number;}
      else if (option == "--hold-seconds") {c.hold_seconds = number;}
      else if (option == "--max-flight-seconds") {c.max_flight_seconds = number;}
      else if (option == "--min-battery-voltage") {c.min_battery_voltage_v = number;}
      else if (option == "--min-battery-fraction") {c.min_battery_fraction = number;}
      else if (option == "--configured-min-range") {c.configured_min_range_m = number;}
      else if (option == "--configured-max-range") {c.configured_max_range_m = number;}
      else if (option == "--range-boundary-tolerance") {c.range_boundary_tolerance_m = number;}
      else if (option == "--max-preflight-horizontal-speed") {c.max_preflight_horizontal_speed_m_s = number;}
      else if (option == "--max-preflight-vertical-speed") {c.max_preflight_vertical_speed_m_s = number;}
      else if (option == "--max-flight-horizontal-speed") {c.max_flight_horizontal_speed_m_s = number;}
      else if (option == "--max-flight-vertical-speed") {c.max_flight_vertical_speed_m_s = number;}
      else if (option == "--max-flight-horizontal-drift") {c.max_flight_horizontal_drift_m = number;}
      else if (option == "--target-tolerance") {c.target_tolerance_m = number;}
      else if (option == "--touchdown-z-tolerance") {c.touchdown_z_tolerance_m = number;}
      else if (option == "--flow-effective-min-height") {c.flow_effective_min_height_m = number;}
      else if (option == "--mode-request-interval") {o.mode_request_interval = number;}
      else if (option == "--service-timeout") {o.service_timeout = number;}
      else if (option == "--sensor-loss-grace-seconds") {c.sensor_loss_grace_s = number;}
      else if (option == "--lcp-status-timeout") {c.lcp_status_timeout_s = number;}
      else if (option == "--lcp-odometry-timeout") {c.lcp_odometry_timeout_s = number;}
      else if (option == "--lcp-vision-xy-stddev") {o.lcp_vision_xy_stddev_m = number;}
      else if (option == "--lcp-vision-yaw-stddev") {o.lcp_vision_yaw_stddev_rad = number;}
      else if (option == "--lcp-vision-max-status-age") {o.lcp_vision_max_status_age_s = number;}
      else {throw std::invalid_argument("unknown option: " + option);}
    };

  const std::vector<std::string> string_options{
    "--confirmed-fcu-url", "--range-topic", "--range-source-label", "--optical-flow-topic", "--optical-flow-source-label",
    "--state-topic", "--sys-status-topic", "--battery-topic", "--extended-state-topic", "--local-pose-topic",
    "--local-velocity-topic", "--estimator-status-topic", "--setpoint-topic", "--lcp-start-service", "--lcp-status-topic",
    "--lcp-odometry-topic", "--lcp-vision-pose-topic", "--lcp-vision-input-frame", "--output", "--artifact-dir",
    "--px4-xy-fusion-evidence-label"};
  const std::vector<std::string> number_options{
    "--status-period", "--publish-rate", "--setpoint-warmup", "--relative-z", "--max-z-setpoint-rate", "--max-z-setpoint-accel",
    "--target-xy-max-speed", "--target-xy-max-accel", "--hold-seconds",
    "--max-flight-seconds", "--min-battery-voltage", "--min-battery-fraction", "--configured-min-range", "--configured-max-range",
    "--range-boundary-tolerance", "--max-preflight-horizontal-speed", "--max-preflight-vertical-speed", "--max-flight-horizontal-speed",
    "--max-flight-vertical-speed", "--max-flight-horizontal-drift", "--target-tolerance", "--touchdown-z-tolerance",
    "--flow-effective-min-height", "--mode-request-interval", "--service-timeout", "--sensor-loss-grace-seconds",
    "--lcp-status-timeout", "--lcp-odometry-timeout", "--lcp-vision-xy-stddev",
    "--lcp-vision-yaw-stddev", "--lcp-vision-max-status-age"};

  for (std::size_t i = 1; i < argv.size(); ++i) {
    std::string option = argv[i];
    std::optional<std::string> assigned;
    if (const auto equal = option.find('='); equal != std::string::npos) {
      assigned = option.substr(equal + 1);
      option.resize(equal);
    }
    if (option == "--help" || option == "-h") {throw std::invalid_argument(usage());}
    if (const auto flag = flags.find(option); flag != flags.end()) {
      if (assigned) {throw std::invalid_argument(option + " does not take a value");}
      *flag->second = true;
    } else if (option == "--enforce-declared-min-range") {
      if (assigned) {throw std::invalid_argument(option + " does not take a value");}
      c.ignore_declared_min_range = false; o.ignore_declared_min_range = false;
    } else if (option == "--ignore-declared-min-range") {
      if (assigned) {throw std::invalid_argument(option + " does not take a value");}
      c.ignore_declared_min_range = true; o.ignore_declared_min_range = true;
    } else if (option == "--disable-lcp-vision-bridge") {
      if (assigned) {throw std::invalid_argument(option + " does not take a value");}
      o.lcp_vision_bridge_enabled = false;
    } else if (std::find(string_options.begin(), string_options.end(), option) != string_options.end()) {
      set_string(option, require_value(argv, i, option, assigned));
    } else if (option == "--min-optical-flow-quality" || option == "--flow-effective-min-quality" || option == "--lcp-ready-samples") {
      const int number = parse_int(option, require_value(argv, i, option, assigned));
      if (option == "--min-optical-flow-quality") {c.min_optical_flow_quality = number;}
      else if (option == "--flow-effective-min-quality") {c.flow_effective_min_quality = number;}
      else {c.lcp_ready_samples = number;}
    } else if (std::find(number_options.begin(), number_options.end(), option) != number_options.end()) {
      set_number(option, require_value(argv, i, option, assigned));
    } else {
      throw std::invalid_argument("unknown option: " + option);
    }
  }

  if (o.confirmed_fcu_url.empty() || o.range_topic.empty() || o.range_source_label.empty() ||
      o.optical_flow_topic.empty() || o.optical_flow_source_label.empty()) {
    throw std::invalid_argument("--confirmed-fcu-url, --range-topic, --range-source-label, --optical-flow-topic, and --optical-flow-source-label are required");
  }
  if (blank(o.range_source_label) || blank(o.optical_flow_source_label)) {
    throw std::invalid_argument("range and optical-flow source labels must be non-empty");
  }
  if (blank(o.lcp_start_service) || blank(o.lcp_status_topic) || blank(o.lcp_odometry_topic) ||
      blank(o.lcp_vision_pose_topic) || blank(o.lcp_vision_input_frame)) {
    throw std::invalid_argument("LCP service, topic, frame, and bridge topic names must be non-empty");
  }
  if (o.output != "summary" && o.output != "jsonl") {
    throw std::invalid_argument("--output must be summary or jsonl");
  }
  const bool partial_setpoint = o.enable_position_setpoints || o.ack_native_xyz_position_control || o.ack_setpoint_streaming_risk;
  if (partial_setpoint && !setpoint_enabled(o)) {
    throw std::invalid_argument("missing setpoint flags: --enable-position-setpoints, --ack-native-xyz-position-control, --ack-setpoint-streaming-risk, --confirm-setpoint-mav-frame-local-ned, --confirm-range-source, --confirm-optical-flow-source");
  }
  if ((o.request_offboard_mode || o.ack_disarmed_mode_switch) && !mode_enabled(o)) {
    throw std::invalid_argument("missing mode flags: --request-offboard-mode, --ack-disarmed-mode-switch");
  }
  if ((o.execute_bounded_flight || o.ack_normal_arm_only || o.ack_propeller_configuration_safe ||
       o.ack_area_and_personnel_clear || o.ack_independent_emergency_stop_ready ||
       o.ack_valid_flight_battery_installed || o.ack_direct_px4_xy_fusion_evidence) && !arming_enabled(o)) {
    throw std::invalid_argument("missing flight flags: --execute-bounded-flight, --ack-normal-arm-only, --ack-propeller-configuration-safe, --ack-area-and-personnel-clear, --ack-independent-emergency-stop-ready, --ack-valid-flight-battery-installed, --ack-direct-px4-xy-fusion-evidence");
  }
  if (arming_enabled(o) && blank(o.px4_xy_fusion_evidence_label)) {
    throw std::invalid_argument("bounded flight requires --px4-xy-fusion-evidence-label");
  }
  if (o.status_period <= 0.0 || o.mode_request_interval <= 0.0 || o.service_timeout <= 0.0) {
    throw std::invalid_argument("status-period, mode-request-interval, and service-timeout must be positive");
  }
  if (o.lcp_vision_xy_stddev_m <= 0.0 || o.lcp_vision_yaw_stddev_rad <= 0.0 ||
      o.lcp_vision_max_status_age_s <= 0.0) {
    throw std::invalid_argument("LCP vision standard deviations and status age must be positive");
  }
  c.range_source_confirmed = o.confirm_range_source;
  c.optical_flow_source_confirmed = o.confirm_optical_flow_source;
  c.validate();
  return {o, c};
}

}  // namespace mavros_xyz_position_offboard::common
