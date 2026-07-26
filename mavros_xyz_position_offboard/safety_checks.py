"""Safety gates operating on the shared telemetry state."""

from __future__ import annotations

import math
from typing import Optional

try:
    from .core_types import (
        MAV_LANDED_STATE_ON_GROUND,
        MAV_STATE_ACTIVE,
        MAV_STATE_STANDBY,
        MAV_SYS_STATUS_ACCEPTABLE_UNHEALTHY_MASK,
        MAV_SYS_STATUS_PREARM_CHECK,
        finite,
        normalize_quaternion,
    )
except ImportError:
    from core_types import (
        MAV_LANDED_STATE_ON_GROUND,
        MAV_STATE_ACTIVE,
        MAV_STATE_STANDBY,
        MAV_SYS_STATUS_ACCEPTABLE_UNHEALTHY_MASK,
        MAV_SYS_STATUS_PREARM_CHECK,
        finite,
        normalize_quaternion,
    )


class SafetyChecksMixin:
    """Preflight and in-flight checks used by PositionSafetyCore."""

    @staticmethod
    def _stale(timestamp: Optional[float], now: float, timeout_s: float) -> bool:
        return timestamp is None or now < timestamp or now - timestamp > timeout_s

    def _connection_errors(self, now: float, require_standby: bool) -> list[str]:
        t = self.telemetry
        c = self.config
        errors: list[str] = []
        if self._stale(t.state_at, now, c.state_timeout_s):
            errors.append("MAVROS state/heartbeat stale or unavailable")
        elif not t.connected:
            errors.append("MAVROS is not connected to the flight controller")
        elif require_standby and t.system_status not in (
            MAV_STATE_STANDBY,
            MAV_STATE_ACTIVE,
        ):
            sys_status_ok = (
                not self._stale(t.sys_status_at, now, c.sys_status_timeout_s)
                and not (
                    t.sensors_enabled
                    & (~t.sensors_health & 0xFFFFFFFF)
                    & ~MAV_SYS_STATUS_ACCEPTABLE_UNHEALTHY_MASK
                )
            )
            if not sys_status_ok:
                errors.append(
                    f"PX4 heartbeat system_status is {t.system_status}, "
                    "expected STANDBY (3)"
                )

        if self._stale(t.sys_status_at, now, c.sys_status_timeout_s):
            errors.append("MAVROS SYS_STATUS stale or unavailable")
        else:
            prearm_ok = bool(t.sensors_health & MAV_SYS_STATUS_PREARM_CHECK)
            unhealthy_enabled = (
                t.sensors_enabled
                & (~t.sensors_health & 0xFFFFFFFF)
                & ~MAV_SYS_STATUS_ACCEPTABLE_UNHEALTHY_MASK
            )
            if unhealthy_enabled:
                errors.append(
                    "PX4 SYS_STATUS enabled-but-unhealthy mask is "
                    f"0x{unhealthy_enabled:08x}"
                )
            elif not prearm_ok:
                t.prearm_bit_advisory = True
        return errors

    def _battery_errors(self, now: float, in_flight: bool = False) -> list[str]:
        t = self.telemetry
        c = self.config
        if self._stale(t.battery_at, now, c.battery_timeout_s):
            return ["battery telemetry stale or unavailable"]
        errors = []
        if not t.battery_present:
            errors.append("battery telemetry reports no battery present")
        if not finite(t.battery_voltage_v) or t.battery_voltage_v < c.min_battery_voltage_v:
            errors.append(
                f"battery voltage invalid or below {c.min_battery_voltage_v:.2f} V"
            )
        if not in_flight:
            if not finite(t.battery_fraction) or t.battery_fraction < c.min_battery_fraction:
                errors.append(
                    f"battery fraction invalid or below {c.min_battery_fraction:.0%}"
                )
        return errors

    def _pose_velocity_errors(
        self, now: float, preflight: bool, climbing: bool = False
    ) -> list[str]:
        t = self.telemetry
        c = self.config
        errors = []
        if self._stale(t.local_pose_at, now, c.local_pose_timeout_s):
            errors.append("local pose stale or unavailable")
        else:
            if not all(finite(value) for value in (t.local_x_m, t.local_y_m, t.local_z_m)):
                errors.append("local XYZ pose is non-finite")
            try:
                normalize_quaternion(
                    t.orientation_x,
                    t.orientation_y,
                    t.orientation_z,
                    t.orientation_w,
                )
            except ValueError as exc:
                errors.append(str(exc))

        if self._stale(t.local_velocity_at, now, c.local_velocity_timeout_s):
            errors.append("local velocity stale or unavailable")
        elif not all(
            finite(value)
            for value in (t.velocity_x_m_s, t.velocity_y_m_s, t.velocity_z_m_s)
        ):
            errors.append("local XYZ velocity is non-finite")
        else:
            horizontal_speed = math.hypot(t.velocity_x_m_s, t.velocity_y_m_s)
            horizontal_limit = c.max_preflight_horizontal_speed_m_s if preflight else (
                c.climb_horizontal_speed_limit_m_s
                if climbing
                else c.max_flight_horizontal_speed_m_s
            )
            vertical_limit = (
                c.max_preflight_vertical_speed_m_s
                if preflight
                else c.max_flight_vertical_speed_m_s
            )
            if horizontal_speed > horizontal_limit:
                errors.append(f"horizontal speed exceeds {horizontal_limit:.2f} m/s")
            if abs(t.velocity_z_m_s) > vertical_limit:
                errors.append(f"vertical speed exceeds {vertical_limit:.2f} m/s")
        return errors

    def _estimator_errors(self, now: float, during_flight: bool) -> list[str]:
        del during_flight
        t = self.telemetry
        if self._stale(t.estimator_at, now, self.config.estimator_timeout_s):
            return ["MAVROS estimator status stale or unavailable"]
        errors = []
        if not t.estimator_attitude_valid:
            errors.append("estimator attitude-valid flag is false")
        if not t.estimator_velocity_horiz_valid:
            errors.append("estimator horizontal-velocity-valid flag is false")
        if not t.estimator_velocity_vert_valid:
            errors.append("estimator vertical-velocity-valid flag is false")
        if not (t.estimator_pos_horiz_rel_valid or t.estimator_pos_horiz_abs_valid):
            errors.append("estimator has no valid relative or absolute horizontal position")
        if not (t.estimator_pos_vert_abs_valid or t.estimator_pos_vert_agl_valid):
            errors.append("estimator has no valid absolute or AGL vertical position")
        if t.estimator_gps_glitch:
            errors.append("estimator GPS-glitch flag is set")
        if t.estimator_accel_error:
            errors.append("estimator accelerometer-error flag is set")
        return errors

    def _range_flow_errors(self, now: float) -> list[str]:
        t = self.telemetry
        c = self.config
        errors = []
        if not c.range_source_confirmed:
            errors.append("range topic source/direction is not explicitly confirmed")
        if self._stale(t.range_at, now, c.range_timeout_s):
            errors.append("downward range stale or unavailable")
        elif self.range_guard.fault_reason is not None:
            errors.append(self.range_guard.fault_reason)

        if not c.optical_flow_source_confirmed:
            errors.append("optical-flow topic source is not explicitly confirmed")
        if self._stale(t.optical_flow_at, now, c.optical_flow_timeout_s):
            errors.append("optical-flow data stale or unavailable")
        else:
            if t.optical_flow_integration_time_us <= 0:
                errors.append("optical-flow integration time is invalid")
            if not all(
                finite(value)
                for value in (
                    t.optical_flow_integrated_x_rad,
                    t.optical_flow_integrated_y_rad,
                )
            ):
                errors.append("optical-flow integrated pixel angles are non-finite")
            if t.optical_flow_quality < c.min_optical_flow_quality:
                errors.append(
                    f"optical-flow quality below {c.min_optical_flow_quality}"
                )
        return errors

    def preflight_errors(self, now: float) -> list[str]:
        t = self.telemetry
        errors = self._connection_errors(now, require_standby=True)
        if t.armed:
            errors.append("vehicle is armed; preflight requires disarmed state")
        if self._stale(t.landed_at, now, self.config.landed_timeout_s):
            errors.append("landed state stale or unavailable")
        elif t.landed_state != MAV_LANDED_STATE_ON_GROUND:
            errors.append("vehicle is not confirmed on ground")
        errors.extend(self._battery_errors(now))
        errors.extend(self._pose_velocity_errors(now, preflight=True))
        errors.extend(self._estimator_errors(now, during_flight=False))
        errors.extend(self._range_flow_errors(now))
        if (
            not self._stale(t.local_pose_at, now, self.config.local_pose_timeout_s)
            and not self._stale(t.range_at, now, self.config.range_timeout_s)
            and finite(t.local_z_m)
            and finite(t.range_m)
            and self.range_guard.fault_reason is None
        ):
            z_mismatch = abs(t.local_z_m - t.range_m)
            if z_mismatch > 0.30:
                errors.append(
                    f"preflight Z mismatch: local_z={t.local_z_m:.3f} m vs "
                    f"range={t.range_m:.3f} m (delta {z_mismatch:.3f} m > 0.30); "
                    "EKF height reference is inconsistent, do not take off"
                )
        return errors

    def flight_errors(
        self,
        now: float,
        hold_x_m: float,
        hold_y_m: float,
        require_offboard: bool = True,
        at_hover: bool = False,
    ) -> list[str]:
        t = self.telemetry
        c = self.config
        errors = self._connection_errors(now, require_standby=False)
        if not t.armed:
            errors.append("vehicle unexpectedly disarmed during flight phase")
        if require_offboard and t.mode.upper() != "OFFBOARD":
            errors.append("OFFBOARD mode is not confirmed")
        errors.extend(self._battery_errors(now, in_flight=True))
        errors.extend(self._pose_velocity_errors(now, preflight=False, climbing=not at_hover))
        errors.extend(self._estimator_errors(now, during_flight=True))
        errors.extend(self._range_flow_errors(now))
        baseline_x = self.drift_baseline_x_m
        baseline_y = self.drift_baseline_y_m
        if baseline_x is None or baseline_y is None:
            baseline_x = hold_x_m
            baseline_y = hold_y_m
        if all(finite(value) for value in (baseline_x, baseline_y, t.local_x_m, t.local_y_m)):
            drift_m = math.hypot(t.local_x_m - baseline_x, t.local_y_m - baseline_y)
            drift_limit = c.max_flight_horizontal_drift_m if at_hover else c.climb_horizontal_drift_limit_m
            if drift_m > drift_limit:
                errors.append(f"horizontal drift exceeds {drift_limit:.2f} m")
        return errors

    def optical_flow_effective(self, now: float) -> bool:
        t = self.telemetry
        c = self.config
        if self._stale(t.range_at, now, c.range_timeout_s):
            return False
        if not finite(t.range_m) or t.range_m < c.flow_effective_min_height_m:
            return False
        if self._stale(t.estimator_at, now, c.estimator_timeout_s):
            return False
        if not (t.estimator_pos_horiz_rel_valid or t.estimator_pos_horiz_abs_valid):
            return False
        if self._stale(t.optical_flow_at, now, c.optical_flow_timeout_s):
            return False
        return t.optical_flow_quality >= c.flow_effective_min_quality
