"""Telemetry storage and the public ROS-independent safety core."""

from __future__ import annotations

import math
from dataclasses import asdict
from typing import Optional

try:
    from .core_types import RangeResult, SafetyConfig, Telemetry, finite
    from .range_guard import RangeGuard
    from .safety_checks import SafetyChecksMixin
except ImportError:
    from core_types import RangeResult, SafetyConfig, Telemetry, finite
    from range_guard import RangeGuard
    from safety_checks import SafetyChecksMixin


class PositionSafetyCore(SafetyChecksMixin):
    """Collect MAVROS-visible telemetry and make conservative gate decisions."""

    def __init__(self, config: SafetyConfig):
        config.validate()
        self.config = config
        self.telemetry = Telemetry()
        self.range_guard = RangeGuard(config)
        self.drift_baseline_x_m: Optional[float] = None
        self.drift_baseline_y_m: Optional[float] = None
        self.drift_baseline_at: Optional[float] = None

    def seed_drift_baseline(self, now: float) -> None:
        t = self.telemetry
        if all(finite(value) for value in (t.local_x_m, t.local_y_m)):
            self.drift_baseline_x_m = t.local_x_m
            self.drift_baseline_y_m = t.local_y_m
            self.drift_baseline_at = now

    def update_state(
        self,
        connected: bool,
        armed: bool,
        mode: str,
        system_status: int,
        now: float,
    ) -> None:
        t = self.telemetry
        t.state_at = now
        t.connected = connected
        t.armed = armed
        t.mode = mode
        t.system_status = system_status

    def update_sys_status(
        self, present: int, enabled: int, health: int, now: float
    ) -> None:
        t = self.telemetry
        t.sys_status_at = now
        t.sensors_present = present
        t.sensors_enabled = enabled
        t.sensors_health = health

    def update_battery(
        self, present: bool, voltage_v: float, fraction: float, now: float
    ) -> None:
        t = self.telemetry
        t.battery_at = now
        t.battery_present = present
        t.battery_voltage_v = voltage_v
        t.battery_fraction = fraction

    def update_landed(self, landed_state: int, now: float) -> None:
        self.telemetry.landed_at = now
        self.telemetry.landed_state = landed_state

    def update_local_pose(
        self,
        x_m: float,
        y_m: float,
        z_m: float,
        orientation_x: float,
        orientation_y: float,
        orientation_z: float,
        orientation_w: float,
        now: float,
    ) -> None:
        t = self.telemetry
        t.local_pose_at = now
        t.local_x_m = x_m
        t.local_y_m = y_m
        t.local_z_m = z_m
        t.orientation_x = orientation_x
        t.orientation_y = orientation_y
        t.orientation_z = orientation_z
        t.orientation_w = orientation_w

    def update_local_velocity(
        self,
        velocity_x_m_s: float,
        velocity_y_m_s: float,
        velocity_z_m_s: float,
        now: float,
    ) -> None:
        t = self.telemetry
        t.local_velocity_at = now
        t.velocity_x_m_s = velocity_x_m_s
        t.velocity_y_m_s = velocity_y_m_s
        t.velocity_z_m_s = velocity_z_m_s

    def update_estimator(self, *, now: float, **flags: bool) -> None:
        t = self.telemetry
        t.estimator_at = now
        for name, value in flags.items():
            setattr(t, f"estimator_{name}", value)

    def update_range(
        self, range_m: float, declared_min_m: float, declared_max_m: float, now: float
    ) -> RangeResult:
        t = self.telemetry
        t.range_at = now
        t.range_m = range_m
        t.range_min_m = declared_min_m
        t.range_max_m = declared_max_m
        return self.range_guard.observe(range_m, declared_min_m, declared_max_m, now)

    def update_optical_flow(
        self,
        *,
        integration_time_us: int,
        integrated_x_rad: float,
        integrated_y_rad: float,
        quality: int,
        distance_m: float,
        distance_delta_us: int,
        temperature_c: float,
        now: float,
    ) -> None:
        t = self.telemetry
        t.optical_flow_at = now
        t.optical_flow_integration_time_us = integration_time_us
        t.optical_flow_integrated_x_rad = integrated_x_rad
        t.optical_flow_integrated_y_rad = integrated_y_rad
        t.optical_flow_quality = quality
        t.optical_flow_distance_m = distance_m
        t.optical_flow_distance_delta_us = distance_delta_us
        t.optical_flow_temperature_c = temperature_c

    def begin_lcp_initialization(self, now: float) -> None:
        t = self.telemetry
        t.lcp_init_status_sequence_baseline = t.lcp_status_sequence
        t.lcp_init_odometry_sequence_baseline = t.lcp_odometry_sequence
        t.lcp_healthy_samples = 0
        t.lcp_init_requested_at = now
        t.lcp_init_response_at = None
        t.lcp_init_response_message = None
        t.lcp_init_failure_reason = None

    def reset_lcp_initialization(self) -> None:
        t = self.telemetry
        t.lcp_healthy_samples = 0
        t.lcp_init_status_sequence_baseline = t.lcp_status_sequence
        t.lcp_init_odometry_sequence_baseline = t.lcp_odometry_sequence
        t.lcp_init_request_state = "not_requested"
        t.lcp_init_requested_at = None
        t.lcp_init_response_at = None
        t.lcp_init_response_message = None
        t.lcp_init_failure_reason = None

    def update_lcp_init_state(
        self,
        state: str,
        now: float,
        *,
        message: Optional[str] = None,
        failure_reason: Optional[str] = None,
    ) -> None:
        t = self.telemetry
        t.lcp_init_request_state = state
        if message is not None:
            t.lcp_init_response_message = message
        if failure_reason is not None:
            t.lcp_init_failure_reason = failure_reason
        if state in ("accepted", "failed"):
            t.lcp_init_response_at = now

    def update_lcp_status(self, status: int, now: float) -> None:
        t = self.telemetry
        t.lcp_status = int(status)
        t.lcp_status_at = now
        t.lcp_status_sequence += 1
        if status != 2:
            t.lcp_healthy_samples = 0
        elif t.lcp_status_sequence > t.lcp_init_status_sequence_baseline:
            t.lcp_healthy_samples += 1

    def update_lcp_odometry(
        self, x_m: float, y_m: float, yaw_rad: float, now: float
    ) -> None:
        t = self.telemetry
        t.lcp_odometry_at = now
        t.lcp_odometry_sequence += 1
        t.lcp_x_m = x_m
        t.lcp_y_m = y_m
        t.lcp_yaw_rad = yaw_rad

    def lcp_runtime_healthy(self, now: float) -> bool:
        t = self.telemetry
        return (
            t.lcp_status == 2
            and not self._stale(t.lcp_status_at, now, self.config.lcp_status_timeout_s)
            and not self._stale(
                t.lcp_odometry_at, now, self.config.lcp_odometry_timeout_s
            )
            and all(finite(value) for value in (t.lcp_x_m, t.lcp_y_m, t.lcp_yaw_rad))
        )

    def lcp_ready(self, now: float) -> bool:
        t = self.telemetry
        return (
            t.lcp_init_request_state == "accepted"
            and self.lcp_runtime_healthy(now)
            and t.lcp_healthy_samples >= self.config.lcp_ready_samples
            and t.lcp_status_sequence > t.lcp_init_status_sequence_baseline
            and t.lcp_odometry_sequence > t.lcp_init_odometry_sequence_baseline
        )

    def lcp_errors(self, now: float, require_samples: bool = False) -> list[str]:
        t = self.telemetry
        errors: list[str] = []
        if t.lcp_init_request_state == "failed":
            errors.append(
                t.lcp_init_failure_reason or "LCP initialization service failed"
            )
        if t.lcp_status is None:
            errors.append("LCP status unavailable")
        elif self._stale(t.lcp_status_at, now, self.config.lcp_status_timeout_s):
            errors.append("LCP status stale or unavailable")
        elif t.lcp_status != 2:
            errors.append(f"LCP status is {t.lcp_status}, expected 2")
        if self._stale(t.lcp_odometry_at, now, self.config.lcp_odometry_timeout_s):
            errors.append("LCP odometry stale or unavailable")
        elif not all(finite(value) for value in (t.lcp_x_m, t.lcp_y_m, t.lcp_yaw_rad)):
            errors.append("LCP odometry is non-finite")
        if require_samples and t.lcp_healthy_samples < self.config.lcp_ready_samples:
            errors.append(
                f"LCP healthy samples {t.lcp_healthy_samples}/"
                f"{self.config.lcp_ready_samples}"
            )
        return errors

    def status(self, now: float) -> dict:
        raw = asdict(self.telemetry)
        sanitized = {
            key: (None if isinstance(value, float) and not finite(value) else value)
            for key, value in raw.items()
        }
        sanitized["monotonic_s"] = round(now, 6)
        sanitized["range_fault"] = self.range_guard.fault_reason
        sanitized["optical_flow_effective"] = self.optical_flow_effective(now)

        def age(timestamp: Optional[float]) -> Optional[float]:
            if timestamp is None:
                return None
            value = now - timestamp
            return round(max(0.0, value), 6) if finite(value) else None

        sanitized["telemetry_age_s"] = {
            "range": age(self.telemetry.range_at),
            "optical_flow": age(self.telemetry.optical_flow_at),
            "lcp_status": age(self.telemetry.lcp_status_at),
            "lcp_odometry": age(self.telemetry.lcp_odometry_at),
        }
        sanitized["drift_baseline"] = {
            "x_m": self.drift_baseline_x_m,
            "y_m": self.drift_baseline_y_m,
            "at": self.drift_baseline_at,
        }
        sanitized["range_policy"] = {
            "ignore_declared_min_range": self.config.ignore_declared_min_range,
            "configured_min_range_m": self.config.configured_min_range_m,
            "configured_max_range_m": self.config.configured_max_range_m,
        }
        sanitized["mavros_visibility_limits"] = [
            "vehicle_local_position.xy_valid/z_valid/v_xy_valid/v_z_valid/dead_reckoning",
            "estimator_status_flags.cs_opt_flow/cs_rng_hgt/reject_*",
            "estimator_aid_src_* fused/innovation_rejected",
            "sensor_optical_flow.distance_available",
            "DISTANCE_SENSOR orientation through sensor_msgs/Range",
        ]
        return sanitized
