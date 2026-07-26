"""Command-line policy and configuration for the XYZ node."""

from __future__ import annotations

import argparse

try:
    from .core import SafetyConfig
except ImportError:
    from core import SafetyConfig


SETPOINT_FLAGS = (
    "enable_position_setpoints",
    "ack_native_xyz_position_control",
    "ack_setpoint_streaming_risk",
    "confirm_setpoint_mav_frame_local_ned",
    "confirm_range_source",
    "confirm_optical_flow_source",
)
SETPOINT_OPT_IN_FLAGS = SETPOINT_FLAGS[:3]
MODE_FLAGS = ("request_offboard_mode", "ack_disarmed_mode_switch")
ARM_FLAGS = (
    "execute_bounded_flight",
    "ack_normal_arm_only",
    "ack_propeller_configuration_safe",
    "ack_area_and_personnel_clear",
    "ack_independent_emergency_stop_ready",
    "ack_valid_flight_battery_installed",
    "ack_direct_px4_xy_fusion_evidence",
)
TOLERATED_SENSOR_STALE_ERRORS = frozenset(
    {"downward range stale or unavailable", "optical-flow data stale or unavailable"}
)


def _all(args: argparse.Namespace, names: tuple[str, ...]) -> bool:
    return all(bool(getattr(args, name)) for name in names)


def _any(args: argparse.Namespace, names: tuple[str, ...]) -> bool:
    return any(bool(getattr(args, name)) for name in names)


def setpoint_enabled(args: argparse.Namespace) -> bool:
    return _all(args, SETPOINT_FLAGS)


def mode_enabled(args: argparse.Namespace) -> bool:
    return setpoint_enabled(args) and _all(args, MODE_FLAGS)


def arming_enabled(args: argparse.Namespace) -> bool:
    return mode_enabled(args) and _all(args, ARM_FLAGS)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Subscribe-only MAVROS native XYZ preflight monitor by default"
    )
    parser.add_argument("--confirmed-fcu-url", required=True)
    parser.add_argument("--range-topic", required=True)
    parser.add_argument("--range-source-label", required=True)
    parser.add_argument("--optical-flow-topic", required=True)
    parser.add_argument("--optical-flow-source-label", required=True)
    parser.add_argument("--state-topic", default="/mavros/state")
    parser.add_argument("--sys-status-topic", default="/mavros/sys_status")
    parser.add_argument("--battery-topic", default="/mavros/battery")
    parser.add_argument("--extended-state-topic", default="/mavros/extended_state")
    parser.add_argument("--local-pose-topic", default="/mavros/local_position/pose")
    parser.add_argument("--local-velocity-topic", default="/mavros/local_position/velocity_local")
    parser.add_argument("--estimator-status-topic", default="/mavros/estimator_status")
    parser.add_argument("--setpoint-topic", default="/mavros/setpoint_raw/local")
    parser.add_argument("--lcp-start-service", default="/lcp/start_initialization")
    parser.add_argument("--lcp-status-topic", default="/lcp/status")
    parser.add_argument("--lcp-odometry-topic", default="/lcp/odometry")
    parser.add_argument(
        "--output",
        choices=("summary", "jsonl"),
        default="summary",
        help="summary: human-readable artifact (default); jsonl: JSON artifact; terminal always uses summary",
    )
    parser.add_argument("--artifact-dir", default="artifacts")
    parser.add_argument("--status-period", type=float, default=0.5)
    parser.add_argument("--publish-rate", type=float, default=20.0)
    parser.add_argument("--setpoint-warmup", type=float, default=2.0)
    parser.add_argument("--relative-z", type=float, default=0.80)
    parser.add_argument("--max-z-setpoint-rate", type=float, default=0.20)
    parser.add_argument("--max-z-setpoint-accel", type=float, default=0.40)
    parser.add_argument("--waypoint-leg", type=float, default=0.50)
    parser.add_argument("--waypoint-max-speed", type=float, default=0.25)
    parser.add_argument("--waypoint-max-accel", type=float, default=0.50)
    parser.add_argument("--waypoint-tolerance", type=float, default=0.08)
    parser.add_argument("--hold-seconds", type=float, default=10.0)
    parser.add_argument("--max-flight-seconds", type=float, default=60.0)
    parser.add_argument("--min-battery-voltage", type=float, default=14.0)
    parser.add_argument("--min-battery-fraction", type=float, default=0.30)
    parser.add_argument("--min-optical-flow-quality", type=int, default=20)
    parser.add_argument("--configured-min-range", type=float, default=0.02)
    parser.add_argument("--configured-max-range", type=float, default=12.0)
    parser.add_argument("--ignore-declared-min-range", action="store_true", default=True)
    parser.add_argument(
        "--enforce-declared-min-range",
        action="store_false",
        dest="ignore_declared_min_range",
    )
    parser.add_argument("--range-boundary-tolerance", type=float, default=0.001)
    parser.add_argument("--max-preflight-horizontal-speed", type=float, default=0.20)
    parser.add_argument("--max-preflight-vertical-speed", type=float, default=0.20)
    parser.add_argument("--max-flight-horizontal-speed", type=float, default=0.50)
    parser.add_argument("--max-flight-vertical-speed", type=float, default=0.80)
    parser.add_argument("--max-flight-horizontal-drift", type=float, default=0.50)
    parser.add_argument("--target-tolerance", type=float, default=0.04)
    parser.add_argument("--touchdown-z-tolerance", type=float, default=0.08)
    parser.add_argument("--flow-effective-min-height", type=float, default=0.35)
    parser.add_argument("--flow-effective-min-quality", type=int, default=20)
    parser.add_argument("--mode-request-interval", type=float, default=2.0)
    parser.add_argument("--service-timeout", type=float, default=3.0)
    parser.add_argument("--sensor-loss-grace-seconds", type=float, default=2.0)
    parser.add_argument("--lcp-status-timeout", type=float, default=0.75)
    parser.add_argument("--lcp-odometry-timeout", type=float, default=0.75)
    parser.add_argument("--lcp-ready-samples", type=int, default=3)
    parser.add_argument("--lcp-unhealthy-hold-timeout", type=float, default=2.0)
    parser.add_argument("--enable-position-setpoints", action="store_true")
    parser.add_argument("--ack-native-xyz-position-control", action="store_true")
    parser.add_argument("--ack-setpoint-streaming-risk", action="store_true")
    parser.add_argument("--confirm-setpoint-mav-frame-local-ned", action="store_true")
    parser.add_argument("--confirm-range-source", action="store_true")
    parser.add_argument("--confirm-optical-flow-source", action="store_true")
    parser.add_argument("--request-offboard-mode", action="store_true")
    parser.add_argument("--ack-disarmed-mode-switch", action="store_true")
    parser.add_argument("--execute-bounded-flight", action="store_true")
    parser.add_argument("--ack-normal-arm-only", action="store_true")
    parser.add_argument("--ack-propeller-configuration-safe", action="store_true")
    parser.add_argument("--ack-area-and-personnel-clear", action="store_true")
    parser.add_argument("--ack-independent-emergency-stop-ready", action="store_true")
    parser.add_argument("--ack-valid-flight-battery-installed", action="store_true")
    parser.add_argument("--ack-direct-px4-xy-fusion-evidence", action="store_true")
    parser.add_argument("--ack-range-below-declared-min", action="store_true")
    parser.add_argument("--px4-xy-fusion-evidence-label", default="")
    return parser


def parse_args(argv: list[str]) -> tuple[argparse.Namespace, list[str]]:
    args, ros_args = build_parser().parse_known_args(argv)
    if not args.range_source_label.strip() or not args.optical_flow_source_label.strip():
        raise SystemExit("range and optical-flow source labels must be non-empty")
    if (
        not args.lcp_start_service.strip()
        or not args.lcp_status_topic.strip()
        or not args.lcp_odometry_topic.strip()
    ):
        raise SystemExit("LCP service and topic names must be non-empty")
    if _any(args, SETPOINT_OPT_IN_FLAGS) and not setpoint_enabled(args):
        missing = [name for name in SETPOINT_FLAGS if not getattr(args, name)]
        raise SystemExit("missing setpoint flags: " + ", ".join(missing))
    if _any(args, MODE_FLAGS) and not mode_enabled(args):
        missing = [name for name in MODE_FLAGS if not getattr(args, name)]
        raise SystemExit("missing mode flags: " + ", ".join(missing))
    if _any(args, ARM_FLAGS) and not arming_enabled(args):
        missing = [name for name in ARM_FLAGS if not getattr(args, name)]
        raise SystemExit("missing flight flags: " + ", ".join(missing))
    if arming_enabled(args) and not args.px4_xy_fusion_evidence_label.strip():
        raise SystemExit("bounded flight requires --px4-xy-fusion-evidence-label")
    positive = (
        "status_period", "publish_rate", "setpoint_warmup", "relative_z",
        "max_z_setpoint_rate", "max_z_setpoint_accel", "hold_seconds",
        "max_flight_seconds", "min_battery_voltage", "min_battery_fraction",
        "configured_min_range", "configured_max_range", "range_boundary_tolerance",
        "max_preflight_horizontal_speed", "max_preflight_vertical_speed",
        "max_flight_horizontal_speed", "max_flight_vertical_speed",
        "max_flight_horizontal_drift", "target_tolerance", "touchdown_z_tolerance",
        "waypoint_leg", "waypoint_max_speed", "waypoint_max_accel",
        "waypoint_tolerance", "mode_request_interval", "service_timeout",
        "sensor_loss_grace_seconds", "lcp_status_timeout",
        "lcp_odometry_timeout", "lcp_unhealthy_hold_timeout",
    )
    for name in positive:
        if getattr(args, name) <= 0.0:
            raise SystemExit(f"--{name.replace('_', '-')} must be positive")
    if args.configured_max_range <= args.configured_min_range:
        raise SystemExit("--configured-max-range must exceed --configured-min-range")
    if not 1 <= args.min_optical_flow_quality <= 255:
        raise SystemExit("--min-optical-flow-quality must be within 1..255")
    if args.min_battery_fraction > 1.0:
        raise SystemExit("--min-battery-fraction must not exceed 1.0")
    if args.lcp_ready_samples < 1:
        raise SystemExit("--lcp-ready-samples must be at least 1")
    return args, ros_args


def make_config(args: argparse.Namespace) -> SafetyConfig:
    config = SafetyConfig(
        publish_rate_hz=args.publish_rate,
        setpoint_warmup_s=args.setpoint_warmup,
        relative_z_m=args.relative_z,
        max_z_setpoint_rate_m_s=args.max_z_setpoint_rate,
        max_z_setpoint_accel_m_s2=args.max_z_setpoint_accel,
        waypoint_leg_m=args.waypoint_leg,
        waypoint_max_speed_m_s=args.waypoint_max_speed,
        waypoint_max_accel_m_s2=args.waypoint_max_accel,
        waypoint_tolerance_m=args.waypoint_tolerance,
        hold_seconds=args.hold_seconds,
        max_flight_seconds=args.max_flight_seconds,
        sensor_loss_grace_s=args.sensor_loss_grace_seconds,
        min_battery_voltage_v=args.min_battery_voltage,
        min_battery_fraction=args.min_battery_fraction,
        min_optical_flow_quality=args.min_optical_flow_quality,
        configured_min_range_m=args.configured_min_range,
        configured_max_range_m=args.configured_max_range,
        ignore_declared_min_range=args.ignore_declared_min_range,
        range_boundary_tolerance_m=args.range_boundary_tolerance,
        max_preflight_horizontal_speed_m_s=args.max_preflight_horizontal_speed,
        max_preflight_vertical_speed_m_s=args.max_preflight_vertical_speed,
        max_flight_horizontal_speed_m_s=args.max_flight_horizontal_speed,
        max_flight_vertical_speed_m_s=args.max_flight_vertical_speed,
        max_flight_horizontal_drift_m=args.max_flight_horizontal_drift,
        target_tolerance_m=args.target_tolerance,
        touchdown_z_tolerance_m=args.touchdown_z_tolerance,
        flow_effective_min_height_m=args.flow_effective_min_height,
        flow_effective_min_quality=args.flow_effective_min_quality,
        range_source_confirmed=args.confirm_range_source,
        optical_flow_source_confirmed=args.confirm_optical_flow_source,
        lcp_status_timeout_s=args.lcp_status_timeout,
        lcp_odometry_timeout_s=args.lcp_odometry_timeout,
        lcp_ready_samples=args.lcp_ready_samples,
        lcp_unhealthy_hold_timeout_s=args.lcp_unhealthy_hold_timeout,
    )
    config.validate()
    return config
