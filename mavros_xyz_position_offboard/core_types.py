"""Shared data types and constants for the MAVROS XYZ controller."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional


MAV_STATE_UNINIT = 0
MAV_STATE_BOOT = 1
MAV_STATE_CALIBRATING = 2
MAV_STATE_STANDBY = 3
MAV_STATE_ACTIVE = 4
MAV_LANDED_STATE_ON_GROUND = 1
MAV_SYS_STATUS_PREARM_CHECK = 1 << 28
MAV_SYS_STATUS_ACCEPTABLE_UNHEALTHY_MASK = (
    (1 << 8)    # 3D mag (often unused indoors)
    | (1 << 9)  # absolute pressure (baro) when not used for altitude
    | (1 << 10) # differential pressure (airspeed sensor)
    | (1 << 11) # GPS (indoor flights)
    | (1 << 14) # vision position
    | (1 << 16) # estimator subsystem reported separately via EstimatorStatus
)


def finite(value: float) -> bool:
    return math.isfinite(value)


def clamp(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


def normalize_quaternion(
    x: float, y: float, z: float, w: float
) -> tuple[float, float, float, float]:
    values = (x, y, z, w)
    if not all(finite(value) for value in values):
        raise ValueError("local-pose quaternion must be finite")
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-6:
        raise ValueError("local-pose quaternion norm is too small")
    return tuple(value / norm for value in values)  # type: ignore[return-value]


@dataclass
class SafetyConfig:
    state_timeout_s: float = 1.5
    sys_status_timeout_s: float = 2.0
    battery_timeout_s: float = 2.0
    landed_timeout_s: float = 2.0
    local_pose_timeout_s: float = 0.5
    local_velocity_timeout_s: float = 0.5
    estimator_timeout_s: float = 1.5
    range_timeout_s: float = 0.35
    optical_flow_timeout_s: float = 0.35
    sensor_loss_grace_s: float = 2.0
    lcp_status_timeout_s: float = 0.75
    lcp_odometry_timeout_s: float = 0.75
    lcp_ready_samples: int = 3
    lcp_unhealthy_hold_timeout_s: float = 2.0
    range_boundary_tolerance_m: float = 0.001
    configured_min_range_m: float = 0.02
    configured_max_range_m: float = 12.0
    max_range_jump_m: float = 0.30
    jump_window_s: float = 0.30
    jump_recovery_samples: int = 3
    jump_settle_tolerance_m: float = 0.06
    min_optical_flow_quality: int = 20
    min_battery_voltage_v: float = 14.0
    min_battery_fraction: float = 0.30
    max_preflight_horizontal_speed_m_s: float = 0.20
    max_preflight_vertical_speed_m_s: float = 0.20
    max_flight_horizontal_speed_m_s: float = 1.0
    max_flight_vertical_speed_m_s: float = 1.5
    max_flight_horizontal_drift_m: float = 1.5
    climb_horizontal_speed_limit_m_s: float = 3.0
    climb_horizontal_drift_limit_m: float = 3.0
    hover_min_height_m: float = 0.20
    publish_rate_hz: float = 20.0
    setpoint_warmup_s: float = 2.0
    relative_z_m: float = 0.80
    max_z_setpoint_rate_m_s: float = 1.00
    max_z_setpoint_accel_m_s2: float = 1.50
    waypoint_leg_m: float = 0.50
    waypoint_max_speed_m_s: float = 0.25
    waypoint_max_accel_m_s2: float = 0.50
    waypoint_tolerance_m: float = 0.08
    target_tolerance_m: float = 0.04
    touchdown_z_tolerance_m: float = 0.08
    hold_seconds: float = 10.0
    max_flight_seconds: float = 60.0
    flow_effective_min_height_m: float = 0.35
    flow_effective_min_quality: int = 20
    range_source_confirmed: bool = False
    optical_flow_source_confirmed: bool = False
    ignore_declared_min_range: bool = True

    def validate(self) -> None:
        positive = {
            "state_timeout_s": self.state_timeout_s,
            "sys_status_timeout_s": self.sys_status_timeout_s,
            "battery_timeout_s": self.battery_timeout_s,
            "landed_timeout_s": self.landed_timeout_s,
            "local_pose_timeout_s": self.local_pose_timeout_s,
            "local_velocity_timeout_s": self.local_velocity_timeout_s,
            "estimator_timeout_s": self.estimator_timeout_s,
            "range_timeout_s": self.range_timeout_s,
            "optical_flow_timeout_s": self.optical_flow_timeout_s,
            "sensor_loss_grace_s": self.sensor_loss_grace_s,
            "lcp_status_timeout_s": self.lcp_status_timeout_s,
            "lcp_odometry_timeout_s": self.lcp_odometry_timeout_s,
            "lcp_unhealthy_hold_timeout_s": self.lcp_unhealthy_hold_timeout_s,
            "range_boundary_tolerance_m": self.range_boundary_tolerance_m,
            "configured_min_range_m": self.configured_min_range_m,
            "configured_max_range_m": self.configured_max_range_m,
            "max_range_jump_m": self.max_range_jump_m,
            "jump_window_s": self.jump_window_s,
            "jump_settle_tolerance_m": self.jump_settle_tolerance_m,
            "min_battery_voltage_v": self.min_battery_voltage_v,
            "max_preflight_horizontal_speed_m_s": self.max_preflight_horizontal_speed_m_s,
            "max_preflight_vertical_speed_m_s": self.max_preflight_vertical_speed_m_s,
            "max_flight_horizontal_speed_m_s": self.max_flight_horizontal_speed_m_s,
            "max_flight_vertical_speed_m_s": self.max_flight_vertical_speed_m_s,
            "max_flight_horizontal_drift_m": self.max_flight_horizontal_drift_m,
            "setpoint_warmup_s": self.setpoint_warmup_s,
            "relative_z_m": self.relative_z_m,
            "max_z_setpoint_rate_m_s": self.max_z_setpoint_rate_m_s,
            "max_z_setpoint_accel_m_s2": self.max_z_setpoint_accel_m_s2,
            "waypoint_leg_m": self.waypoint_leg_m,
            "waypoint_max_speed_m_s": self.waypoint_max_speed_m_s,
            "waypoint_max_accel_m_s2": self.waypoint_max_accel_m_s2,
            "waypoint_tolerance_m": self.waypoint_tolerance_m,
            "target_tolerance_m": self.target_tolerance_m,
            "touchdown_z_tolerance_m": self.touchdown_z_tolerance_m,
            "hold_seconds": self.hold_seconds,
            "max_flight_seconds": self.max_flight_seconds,
        }
        bad = [name for name, value in positive.items() if value <= 0.0]
        if bad:
            raise ValueError(f"configuration values must be positive: {', '.join(bad)}")
        if self.configured_max_range_m <= self.configured_min_range_m:
            raise ValueError("configured maximum range must exceed minimum range")
        if self.jump_recovery_samples < 2:
            raise ValueError("jump recovery requires at least two samples")
        if self.lcp_ready_samples < 1:
            raise ValueError("LCP ready samples must be at least one")
        if not 1 <= self.min_optical_flow_quality <= 255:
            raise ValueError("optical-flow quality must be within 1..255")
        if not 10.0 <= self.publish_rate_hz <= 50.0:
            raise ValueError("setpoint publication must be within 10..50 Hz")
        if not 0.0 < self.min_battery_fraction <= 1.0:
            raise ValueError("minimum battery fraction must be within (0, 1]")


@dataclass
class Telemetry:
    state_at: Optional[float] = None
    connected: bool = False
    armed: bool = False
    mode: str = ""
    system_status: int = -1
    sys_status_at: Optional[float] = None
    sensors_present: int = 0
    sensors_enabled: int = 0
    sensors_health: int = 0
    prearm_bit_advisory: bool = False
    battery_at: Optional[float] = None
    battery_present: bool = False
    battery_voltage_v: float = math.nan
    battery_fraction: float = math.nan
    landed_at: Optional[float] = None
    landed_state: int = 0
    local_pose_at: Optional[float] = None
    local_x_m: float = math.nan
    local_y_m: float = math.nan
    local_z_m: float = math.nan
    orientation_x: float = math.nan
    orientation_y: float = math.nan
    orientation_z: float = math.nan
    orientation_w: float = math.nan
    local_velocity_at: Optional[float] = None
    velocity_x_m_s: float = math.nan
    velocity_y_m_s: float = math.nan
    velocity_z_m_s: float = math.nan
    estimator_at: Optional[float] = None
    estimator_attitude_valid: bool = False
    estimator_velocity_horiz_valid: bool = False
    estimator_velocity_vert_valid: bool = False
    estimator_pos_horiz_rel_valid: bool = False
    estimator_pos_horiz_abs_valid: bool = False
    estimator_pos_vert_abs_valid: bool = False
    estimator_pos_vert_agl_valid: bool = False
    estimator_const_pos_mode: bool = False
    estimator_gps_glitch: bool = False
    estimator_accel_error: bool = False
    range_at: Optional[float] = None
    range_m: float = math.nan
    range_min_m: float = math.nan
    range_max_m: float = math.nan
    optical_flow_at: Optional[float] = None
    optical_flow_integration_time_us: int = 0
    optical_flow_integrated_x_rad: float = math.nan
    optical_flow_integrated_y_rad: float = math.nan
    optical_flow_quality: int = -1
    optical_flow_distance_m: float = math.nan
    optical_flow_distance_delta_us: int = 0
    optical_flow_temperature_c: float = math.nan
    lcp_status: Optional[int] = None
    lcp_status_at: Optional[float] = None
    lcp_status_sequence: int = 0
    lcp_odometry_at: Optional[float] = None
    lcp_odometry_sequence: int = 0
    lcp_x_m: float = math.nan
    lcp_y_m: float = math.nan
    lcp_yaw_rad: float = math.nan
    lcp_healthy_samples: int = 0
    lcp_init_status_sequence_baseline: int = 0
    lcp_init_odometry_sequence_baseline: int = 0
    lcp_init_request_state: str = "not_requested"
    lcp_init_requested_at: Optional[float] = None
    lcp_init_response_at: Optional[float] = None
    lcp_init_response_message: Optional[str] = None
    lcp_init_failure_reason: Optional[str] = None


@dataclass(frozen=True)
class RangeResult:
    accepted: bool
    reason: Optional[str]


@dataclass(frozen=True)
class PositionSetpoint:
    x_m: float
    y_m: float
    z_m: float
    orientation_x: float
    orientation_y: float
    orientation_z: float
    orientation_w: float
    vertical_rate_m_s: float
