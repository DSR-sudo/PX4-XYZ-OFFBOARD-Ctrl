"""Flight state machine for the modular MAVROS XYZ node."""

from __future__ import annotations

import math

try:
    from .cli import TOLERATED_SENSOR_STALE_ERRORS
    from .core_types import MAV_LANDED_STATE_ON_GROUND, finite
    from .planner import SmoothPositionPlanner
except ImportError:
    from cli import TOLERATED_SENSOR_STALE_ERRORS
    from core_types import MAV_LANDED_STATE_ON_GROUND, finite
    from planner import SmoothPositionPlanner


class FlightStateMixin:
    def _enter(self, phase: str, now: float, reason: str | None = None) -> None:
        self.phase = phase
        self.phase_started_at = now
        if reason is not None:
            self.abort_reason = reason

    def _reset_candidate(self, now: float) -> None:
        if hasattr(self, "_cancel_lcp_start"):
            self._cancel_lcp_start()
        self.core.reset_lcp_initialization()
        self.planner = SmoothPositionPlanner(self.config)
        self.waypoints = []
        self.waypoint_index = -1
        self.setpoint_stream_since = None
        self.sensor_loss_started_at = None
        self.lcp_hold_started_at = None
        self.lcp_hold_resume_phase = None
        self.lcp_hold_resume_phase_started_at = None
        self.lcp_hold_resume_target_x_m = None
        self.lcp_hold_resume_target_y_m = None
        self.lcp_hold_resume_target_z_m = None
        self.last_tick_at = now
        self._enter("waiting_preflight", now)

    def _monitor_tick(self, now: float) -> None:
        errors = self.core.preflight_errors(now)
        self.last_errors = errors
        self.phase = "preflight_pass" if not errors else "blocked"

    def _prepare_waypoints(self) -> None:
        x0 = self.planner.origin_x_m
        y0 = self.planner.origin_y_m
        yaw = self.planner.yaw_rad()
        leg = self.config.waypoint_leg_m
        forward_x = math.cos(yaw) * leg
        forward_y = math.sin(yaw) * leg
        left_x = -math.sin(yaw) * leg
        left_y = math.cos(yaw) * leg

        first = (x0 + forward_x, y0 + forward_y)
        second = (first[0] + left_x, first[1] + left_y)
        third = (second[0] - forward_x, second[1] - forward_y)
        fourth = (third[0] - left_x, third[1] - left_y)
        self.waypoints = [first, second, third, fourth]
        self.waypoint_index = -1

    def _start_next_waypoint(self, now: float) -> None:
        next_index = self.waypoint_index + 1
        if next_index >= len(self.waypoints):
            self._begin_landing(now)
            return
        target_x_m, target_y_m = self.waypoints[next_index]
        self.planner.set_xy_target(target_x_m, target_y_m)
        self.waypoint_index = next_index
        self._enter("waypoint", now)

    def _waypoint_reached(self) -> bool:
        if self.waypoint_index < 0 or not self.planner.xy_target_reached():
            return False
        t = self.core.telemetry
        if not all(finite(value) for value in (t.local_x_m, t.local_y_m)):
            return False
        distance_m = math.hypot(
            t.local_x_m - self.planner.target_x_m,
            t.local_y_m - self.planner.target_y_m,
        )
        return distance_m <= self.config.waypoint_tolerance_m

    def _enter_lcp_hold(self, now: float, reason: str) -> None:
        t = self.core.telemetry
        self.lcp_hold_resume_phase = self.phase
        self.lcp_hold_resume_phase_started_at = self.phase_started_at
        self.lcp_hold_resume_target_x_m = self.planner.target_x_m
        self.lcp_hold_resume_target_y_m = self.planner.target_y_m
        self.lcp_hold_resume_target_z_m = self.planner.target_z_m
        if all(finite(value) for value in (t.local_x_m, t.local_y_m)):
            self.planner.freeze_xy_at(t.local_x_m, t.local_y_m)
        else:
            self.planner.hold_xy()
        self.planner.freeze_z()
        self.lcp_hold_started_at = now
        self._enter("lcp_hold", now, reason)

    def _resume_lcp_hold(self, now: float) -> None:
        phase = self.lcp_hold_resume_phase or "waypoint"
        previous_started_at = self.lcp_hold_resume_phase_started_at
        if all(
            value is not None and finite(value)
            for value in (
                self.lcp_hold_resume_target_x_m,
                self.lcp_hold_resume_target_y_m,
            )
        ):
            self.planner.set_xy_target(
                self.lcp_hold_resume_target_x_m,
                self.lcp_hold_resume_target_y_m,
            )
        if (
            self.lcp_hold_resume_target_z_m is not None
            and finite(self.lcp_hold_resume_target_z_m)
        ):
            self.planner.set_z_target(self.lcp_hold_resume_target_z_m)
        self._enter(phase, now)
        if previous_started_at is not None and self.lcp_hold_started_at is not None:
            self.phase_started_at = previous_started_at + (
                now - self.lcp_hold_started_at
            )
        self.lcp_hold_started_at = None
        self.lcp_hold_resume_phase = None
        self.lcp_hold_resume_phase_started_at = None
        self.lcp_hold_resume_target_x_m = None
        self.lcp_hold_resume_target_y_m = None
        self.lcp_hold_resume_target_z_m = None

    def _lcp_prearm_errors(self, now: float) -> list[str]:
        t = self.core.telemetry
        errors = self.core.lcp_errors(now, require_samples=True)
        if t.lcp_init_request_state in (
            "not_requested", "waiting_service", "request_sent"
        ):
            errors.insert(0, "LCP initialization service has not been accepted")
        return errors

    def _prearm_control_tick(self, now: float, dt_s: float) -> None:
        errors = self.core.preflight_errors(now)
        self.last_errors = errors
        if errors:
            if self.phase != "waiting_preflight" or self.planner.latched:
                self._reset_candidate(now)
            return

        lcp_state = self.core.telemetry.lcp_init_request_state
        if lcp_state in ("not_requested", "waiting_service"):
            self.phase = "lcp_start_pending"
            self._request_lcp_start(now)
            self.last_errors = ["waiting for LCP initialization service"]
            return
        if lcp_state == "request_sent":
            self.phase = "lcp_start_pending"
            self.last_errors = ["waiting for LCP initialization service response"]
            return
        if lcp_state == "failed":
            self.phase = "lcp_start_pending"
            self.last_errors = self._lcp_prearm_errors(now)
            return
        if not self.core.lcp_ready(now):
            if self.planner.latched:
                self.planner = SmoothPositionPlanner(self.config)
                self.setpoint_stream_since = None
                if self.mode_future is not None:
                    self.mode_future.cancel()
                    self.mode_future = None
                    self.mode_future_started_at = None
                if self.arm_future is not None:
                    self.arm_future.cancel()
                    self.arm_future = None
                    self.arm_future_started_at = None
            self.phase = "lcp_initializing"
            self.last_errors = self._lcp_prearm_errors(now)
            return

        if (
            self.phase == "arming_request_pending"
            and self.core.telemetry.armed
            and self.core.telemetry.mode.upper() == "OFFBOARD"
        ):
            t = self.core.telemetry
            self.flight_started_at = now
            self.core.seed_drift_baseline(now)
            if all(finite(value) for value in (t.local_x_m, t.local_y_m)):
                self.planner.recenter_xy(t.local_x_m, t.local_y_m)
            self.planner.set_relative_target(self.config.relative_z_m)
            self.result = "UNCONFIRMED"
            self._enter("climb", now)
            return
        if not self.planner.latched:
            self._enter("lcp_ready", now)
            self._latch_current_pose()
            self.setpoint_stream_since = now
            self._enter("setpoint_warmup", now)
        self._publish_setpoint(dt_s)
        if self.setpoint_stream_since is None:
            return
        if now - self.setpoint_stream_since < self.config.setpoint_warmup_s:
            self.phase = "setpoint_warmup"
            return
        if not self.mode_enabled:
            self.phase = "position_stream_ready"
            return
        if self.core.telemetry.mode.upper() != "OFFBOARD":
            self.phase = "offboard_request_pending"
            self._request_mode(now, "OFFBOARD")
            return
        if not self.arming_enabled:
            self.phase = "offboard_disarmed_pass"
            return
        self.phase = "arming_request_pending"
        self._request_arm(now, True)

    def _begin_landing(self, now: float, reason: str | None = None) -> None:
        self.planner.hold_xy()
        self.planner.set_ground_target()
        if reason is not None:
            self.result = "ABORTED_SAFE_PENDING_LANDING"
        self._enter("landing", now, reason)

    def _flight_tick(self, now: float, dt_s: float) -> None:
        t = self.core.telemetry
        at_hover = self.phase in (
            "hold", "lcp_hold", "landing", "normal_disarm_pending"
        )
        errors = self.core.flight_errors(
            now, self.planner.x_m, self.planner.y_m, at_hover=at_hover
        )
        stale_errors = [
            error for error in errors if error in TOLERATED_SENSOR_STALE_ERRORS
        ]
        other_errors = [
            error for error in errors if error not in TOLERATED_SENSOR_STALE_ERRORS
        ]
        if stale_errors:
            if self.sensor_loss_started_at is None:
                self.sensor_loss_started_at = now
            loss_duration = max(0.0, now - self.sensor_loss_started_at)
        else:
            self.sensor_loss_started_at = None
            loss_duration = 0.0

        blocking_errors = list(errors)
        visible_errors = list(errors)
        if stale_errors and loss_duration < self.config.sensor_loss_grace_s:
            blocking_errors = other_errors
            visible_errors = list(other_errors)
            visible_errors.append(
                f"range/optical-flow stale for {loss_duration:.2f}/"
                f"{self.config.sensor_loss_grace_s:.2f} s; continuing current setpoint"
            )
        if (
            self.flight_started_at is not None
            and now - self.flight_started_at > self.config.max_flight_seconds
        ):
            timeout_error = "maximum bounded-flight time exceeded"
            blocking_errors.append(timeout_error)
            visible_errors.append(timeout_error)
        self.last_errors = visible_errors
        if blocking_errors and self.phase in ("climb", "hold", "waypoint", "lcp_hold"):
            self._begin_landing(now, "; ".join(blocking_errors))

        if (
            self.phase == "waypoint"
            and not blocking_errors
            and not self.core.lcp_runtime_healthy(now)
        ):
            self._enter_lcp_hold(now, "LCP health loss during waypoint")

        if self.phase == "lcp_hold":
            if self.core.lcp_runtime_healthy(now):
                self._resume_lcp_hold(now)
            elif (
                self.lcp_hold_started_at is not None
                and now - self.lcp_hold_started_at
                >= self.config.lcp_unhealthy_hold_timeout_s
            ):
                self._begin_landing(now, "LCP health loss")

        if self.phase in ("climb", "hold", "waypoint", "landing"):
            if t.mode.upper() == "OFFBOARD":
                self._publish_setpoint(dt_s)
            elif self.phase == "landing":
                self._request_mode(now, "AUTO.LAND")

        if self.phase == "lcp_hold":
            if t.mode.upper() == "OFFBOARD":
                self._publish_setpoint(dt_s)
            return

        if self.phase == "climb":
            reached_setpoint = abs(
                self.planner.command_z_m - self.planner.target_z_m
            ) <= 1e-4
            reached_vehicle = (
                math.isfinite(t.local_z_m)
                and abs(t.local_z_m - self.planner.target_z_m)
                <= self.config.target_tolerance_m
            )
            if reached_setpoint and reached_vehicle:
                self._enter("hold", now)
        elif self.phase == "hold":
            if (
                self.phase_started_at is not None
                and now - self.phase_started_at >= self.config.hold_seconds
            ):
                if self.core.lcp_runtime_healthy(now):
                    self.planner.set_yaw_rad(0.0)
                    self._prepare_waypoints()
                    self._start_next_waypoint(now)
                else:
                    self.last_errors.append(
                        "waiting for fresh LCP STATUS=2 before waypoint and yaw=0"
                    )
        elif self.phase == "waypoint":
            if self._waypoint_reached():
                self._start_next_waypoint(now)
        elif self.phase == "landing":
            touchdown = (
                t.landed_state == MAV_LANDED_STATE_ON_GROUND
                and not self.core._stale(t.landed_at, now, self.config.landed_timeout_s)
                and math.isfinite(t.local_z_m)
                and abs(t.local_z_m - self.planner.origin_z_m)
                <= self.config.touchdown_z_tolerance_m
            )
            if touchdown:
                self._enter("normal_disarm_pending", now)
        elif self.phase == "normal_disarm_pending":
            self._request_arm(now, False)
            if not t.armed:
                self.result = (
                    "ABORTED_SAFE" if self.abort_reason is not None else "PASS"
                )
                self._enter("finished", now)

    def _tick(self) -> None:
        now = self._now()
        dt_s = 1.0 / self.config.publish_rate_hz
        if self.last_tick_at is not None:
            dt_s = max(0.001, min(now - self.last_tick_at, 0.25))
        self.last_tick_at = now
        self._poll_service_futures(now)
        if not self.publish_enabled:
            self._monitor_tick(now)
        elif self.phase in {
            "waiting_preflight",
            "setpoint_warmup",
            "position_stream_ready",
            "offboard_request_pending",
            "offboard_disarmed_pass",
            "arming_request_pending",
            "lcp_start_pending",
            "lcp_initializing",
            "lcp_ready",
        }:
            self._prearm_control_tick(now, dt_s)
        elif self.phase in {
            "climb",
            "hold",
            "waypoint",
            "lcp_hold",
            "landing",
            "normal_disarm_pending",
        }:
            self._flight_tick(now, dt_s)

        if (
            now - self.last_status_at >= self.args.status_period
            or self.phase != self.last_phase
        ):
            self._emit_status(now)
            self.last_status_at = now
            self.last_phase = self.phase
