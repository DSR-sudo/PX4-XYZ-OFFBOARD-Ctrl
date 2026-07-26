"""Bounded XYZ setpoint planning."""

from __future__ import annotations

import math

try:
    from .core_types import PositionSetpoint, SafetyConfig, finite, normalize_quaternion
except ImportError:
    from core_types import PositionSetpoint, SafetyConfig, finite, normalize_quaternion


class SmoothPositionPlanner:
    """Latch local ENU pose and generate bounded quintic XY/Z setpoints."""

    def __init__(self, config: SafetyConfig):
        self.config = config
        self.latched = False
        self.flow_effective = False
        self.x_m = math.nan
        self.y_m = math.nan
        self.origin_x_m = math.nan
        self.origin_y_m = math.nan
        self.target_x_m = math.nan
        self.target_y_m = math.nan
        self.xy_velocity_x_m_s = 0.0
        self.xy_velocity_y_m_s = 0.0
        self.xy_trajectory_elapsed_s = 0.0
        self.xy_trajectory_duration_s = 0.0
        self.xy_trajectory_coefficients = ((math.nan,) * 6, (math.nan,) * 6)
        self.origin_z_m = math.nan
        self.command_z_m = math.nan
        self.target_z_m = math.nan
        self.vertical_rate_m_s = 0.0
        self.orientation = (math.nan, math.nan, math.nan, math.nan)
        self.trajectory_elapsed_s = 0.0
        self.trajectory_duration_s = 0.0
        self.trajectory_coefficients = (math.nan,) * 6

    def latch(
        self,
        x_m: float,
        y_m: float,
        z_m: float,
        orientation_x: float,
        orientation_y: float,
        orientation_z: float,
        orientation_w: float,
    ) -> None:
        if not all(finite(value) for value in (x_m, y_m, z_m)):
            raise ValueError("local XYZ pose must be finite before latching")
        self.orientation = normalize_quaternion(
            orientation_x, orientation_y, orientation_z, orientation_w
        )
        self.origin_x_m = x_m
        self.origin_y_m = y_m
        self.x_m = x_m
        self.y_m = y_m
        self.target_x_m = x_m
        self.target_y_m = y_m
        self.xy_velocity_x_m_s = 0.0
        self.xy_velocity_y_m_s = 0.0
        self.xy_trajectory_elapsed_s = 0.0
        self.xy_trajectory_duration_s = 0.0
        self.xy_trajectory_coefficients = (
            (x_m, 0.0, 0.0, 0.0, 0.0, 0.0),
            (y_m, 0.0, 0.0, 0.0, 0.0, 0.0),
        )
        self.origin_z_m = z_m
        self.command_z_m = z_m
        self.target_z_m = z_m
        self.vertical_rate_m_s = 0.0
        self.trajectory_elapsed_s = 0.0
        self.trajectory_duration_s = 0.0
        self.trajectory_coefficients = (z_m, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.latched = True
        self.flow_effective = False

    def recenter_xy(self, x_m: float, y_m: float) -> None:
        if not self.latched:
            raise RuntimeError("position must be latched before recentering XY")
        if not all(finite(value) for value in (x_m, y_m)):
            raise ValueError("recenter XY pose must be finite")
        self.origin_x_m = x_m
        self.origin_y_m = y_m
        self.x_m = x_m
        self.y_m = y_m
        self.target_x_m = x_m
        self.target_y_m = y_m
        self.xy_velocity_x_m_s = 0.0
        self.xy_velocity_y_m_s = 0.0
        self.xy_trajectory_elapsed_s = 0.0
        self.xy_trajectory_duration_s = 0.0
        self.xy_trajectory_coefficients = (
            (x_m, 0.0, 0.0, 0.0, 0.0, 0.0),
            (y_m, 0.0, 0.0, 0.0, 0.0, 0.0),
        )

    def yaw_rad(self) -> float:
        qx, qy, qz, qw = self.orientation
        return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

    def set_relative_target(self, relative_z_m: float) -> None:
        if not self.latched:
            raise RuntimeError("position must be latched before setting a target")
        if not finite(relative_z_m):
            raise ValueError("relative Z target must be finite")
        self._begin_trajectory(self.origin_z_m + relative_z_m)

    def set_xy_target(self, x_m: float, y_m: float) -> None:
        if not self.latched:
            raise RuntimeError("position must be latched before setting an XY target")
        if not all(finite(value) for value in (x_m, y_m)):
            raise ValueError("XY target must be finite")
        self._begin_xy_trajectory(x_m, y_m)

    def freeze_xy_at(self, x_m: float, y_m: float) -> None:
        if not self.latched:
            raise RuntimeError("position must be latched before freezing XY")
        if not all(finite(value) for value in (x_m, y_m)):
            raise ValueError("frozen XY pose must be finite")
        self.x_m = x_m
        self.y_m = y_m
        self.target_x_m = x_m
        self.target_y_m = y_m
        self.xy_velocity_x_m_s = 0.0
        self.xy_velocity_y_m_s = 0.0
        self.xy_trajectory_elapsed_s = 0.0
        self.xy_trajectory_duration_s = 0.0
        self.xy_trajectory_coefficients = (
            (x_m, 0.0, 0.0, 0.0, 0.0, 0.0),
            (y_m, 0.0, 0.0, 0.0, 0.0, 0.0),
        )

    def set_z_target(self, z_m: float) -> None:
        if not self.latched:
            raise RuntimeError("position must be latched before setting a Z target")
        if not finite(z_m):
            raise ValueError("Z target must be finite")
        self._begin_trajectory(z_m)

    def freeze_z(self) -> None:
        if not self.latched:
            raise RuntimeError("position must be latched before freezing Z")
        self.target_z_m = self.command_z_m
        self.vertical_rate_m_s = 0.0
        self.trajectory_elapsed_s = 0.0
        self.trajectory_duration_s = 0.0
        self.trajectory_coefficients = (
            self.command_z_m, 0.0, 0.0, 0.0, 0.0, 0.0
        )

    def set_yaw_rad(self, yaw_rad: float) -> None:
        if not finite(yaw_rad):
            raise ValueError("yaw target must be finite")
        self.orientation = (
            0.0,
            0.0,
            math.sin(0.5 * yaw_rad),
            math.cos(0.5 * yaw_rad),
        )

    def hold_xy(self) -> None:
        if not self.latched:
            raise RuntimeError("position must be latched before holding XY")
        self.target_x_m = self.x_m
        self.target_y_m = self.y_m
        self.xy_velocity_x_m_s = 0.0
        self.xy_velocity_y_m_s = 0.0
        self.xy_trajectory_elapsed_s = 0.0
        self.xy_trajectory_duration_s = 0.0
        self.xy_trajectory_coefficients = (
            (self.x_m, 0.0, 0.0, 0.0, 0.0, 0.0),
            (self.y_m, 0.0, 0.0, 0.0, 0.0, 0.0),
        )

    def set_ground_target(self) -> None:
        if not self.latched:
            raise RuntimeError("position must be latched before setting a target")
        self._begin_trajectory(self.origin_z_m)

    @staticmethod
    def _quintic_coefficients(
        start_z_m: float, start_rate_m_s: float, target_z_m: float, duration_s: float
    ) -> tuple[float, float, float, float, float, float]:
        t = duration_s
        delta = target_z_m - start_z_m
        a0 = start_z_m
        a1 = start_rate_m_s
        a2 = 0.0
        a3 = (20.0 * delta - 12.0 * start_rate_m_s * t) / (2.0 * t**3)
        a4 = (-30.0 * delta + 16.0 * start_rate_m_s * t) / (2.0 * t**4)
        a5 = (12.0 * delta - 6.0 * start_rate_m_s * t) / (2.0 * t**5)
        return a0, a1, a2, a3, a4, a5

    @staticmethod
    def _evaluate(
        coefficients: tuple[float, float, float, float, float, float], t: float
    ) -> tuple[float, float, float]:
        a0, a1, a2, a3, a4, a5 = coefficients
        position = a0 + a1 * t + a2 * t**2 + a3 * t**3 + a4 * t**4 + a5 * t**5
        velocity = a1 + 2.0 * a2 * t + 3.0 * a3 * t**2 + 4.0 * a4 * t**3 + 5.0 * a5 * t**4
        acceleration = 2.0 * a2 + 6.0 * a3 * t + 12.0 * a4 * t**2 + 20.0 * a5 * t**3
        return position, velocity, acceleration

    def _begin_trajectory(self, target_z_m: float) -> None:
        self.target_z_m = target_z_m
        distance_m = abs(target_z_m - self.command_z_m)
        if distance_m < 1e-9 and abs(self.vertical_rate_m_s) < 1e-9:
            self.command_z_m = target_z_m
            self.vertical_rate_m_s = 0.0
            self.trajectory_elapsed_s = 0.0
            self.trajectory_duration_s = 0.0
            return
        duration_s = max(
            0.5,
            2.0 * distance_m / self.config.max_z_setpoint_rate_m_s,
            math.sqrt(6.0 * distance_m / self.config.max_z_setpoint_accel_m_s2),
            2.0 * abs(self.vertical_rate_m_s) / self.config.max_z_setpoint_accel_m_s2,
        )
        coefficients = self._quintic_coefficients(
            self.command_z_m, self.vertical_rate_m_s, target_z_m, duration_s
        )
        for _ in range(12):
            samples = [
                self._evaluate(coefficients, duration_s * index / 200.0)
                for index in range(201)
            ]
            peak_rate = max(abs(sample[1]) for sample in samples)
            peak_accel = max(abs(sample[2]) for sample in samples)
            scale = max(
                1.0,
                peak_rate / self.config.max_z_setpoint_rate_m_s,
                math.sqrt(peak_accel / self.config.max_z_setpoint_accel_m_s2),
            )
            if scale <= 1.000001:
                break
            duration_s *= scale * 1.01
            coefficients = self._quintic_coefficients(
                self.command_z_m, self.vertical_rate_m_s, target_z_m, duration_s
            )
        self.trajectory_elapsed_s = 0.0
        self.trajectory_duration_s = duration_s
        self.trajectory_coefficients = coefficients

    def _begin_xy_trajectory(self, target_x_m: float, target_y_m: float) -> None:
        start_x_m = self.x_m
        start_y_m = self.y_m
        start_speed_m_s = math.hypot(
            self.xy_velocity_x_m_s, self.xy_velocity_y_m_s
        )
        distance_m = math.hypot(target_x_m - start_x_m, target_y_m - start_y_m)
        self.target_x_m = target_x_m
        self.target_y_m = target_y_m
        if distance_m < 1e-9 and start_speed_m_s < 1e-9:
            self.x_m = target_x_m
            self.y_m = target_y_m
            self.xy_velocity_x_m_s = 0.0
            self.xy_velocity_y_m_s = 0.0
            self.xy_trajectory_elapsed_s = 0.0
            self.xy_trajectory_duration_s = 0.0
            self.xy_trajectory_coefficients = (
                (target_x_m, 0.0, 0.0, 0.0, 0.0, 0.0),
                (target_y_m, 0.0, 0.0, 0.0, 0.0, 0.0),
            )
            return

        duration_s = max(
            0.5,
            2.0 * distance_m / self.config.waypoint_max_speed_m_s,
            math.sqrt(6.0 * distance_m / self.config.waypoint_max_accel_m_s2),
            2.0 * start_speed_m_s / self.config.waypoint_max_accel_m_s2,
        )
        coefficients = (
            self._quintic_coefficients(
                start_x_m,
                self.xy_velocity_x_m_s,
                target_x_m,
                duration_s,
            ),
            self._quintic_coefficients(
                start_y_m,
                self.xy_velocity_y_m_s,
                target_y_m,
                duration_s,
            ),
        )
        for _ in range(12):
            samples = [
                (
                    self._evaluate(coefficients[0], duration_s * index / 200.0),
                    self._evaluate(coefficients[1], duration_s * index / 200.0),
                )
                for index in range(201)
            ]
            peak_rate = max(
                math.hypot(sample[0][1], sample[1][1]) for sample in samples
            )
            peak_accel = max(
                math.hypot(sample[0][2], sample[1][2]) for sample in samples
            )
            scale = max(
                1.0,
                peak_rate / self.config.waypoint_max_speed_m_s,
                math.sqrt(peak_accel / self.config.waypoint_max_accel_m_s2),
            )
            if scale <= 1.000001:
                break
            duration_s *= scale * 1.01
            coefficients = (
                self._quintic_coefficients(
                    start_x_m,
                    self.xy_velocity_x_m_s,
                    target_x_m,
                    duration_s,
                ),
                self._quintic_coefficients(
                    start_y_m,
                    self.xy_velocity_y_m_s,
                    target_y_m,
                    duration_s,
                ),
            )
        self.xy_trajectory_elapsed_s = 0.0
        self.xy_trajectory_duration_s = duration_s
        self.xy_trajectory_coefficients = coefficients

    def xy_target_reached(self) -> bool:
        return self.xy_trajectory_duration_s <= 0.0

    def update(self, dt_s: float) -> PositionSetpoint:
        if not self.latched:
            raise RuntimeError("position planner is not latched")
        if not finite(dt_s) or dt_s <= 0.0:
            raise ValueError("planner dt must be finite and positive")
        dt_s = min(dt_s, 0.25)
        if self.xy_trajectory_duration_s > 0.0:
            self.xy_trajectory_elapsed_s = min(
                self.xy_trajectory_elapsed_s + dt_s,
                self.xy_trajectory_duration_s,
            )
            x_state = self._evaluate(
                self.xy_trajectory_coefficients[0], self.xy_trajectory_elapsed_s
            )
            y_state = self._evaluate(
                self.xy_trajectory_coefficients[1], self.xy_trajectory_elapsed_s
            )
            self.x_m, self.xy_velocity_x_m_s = x_state[0], x_state[1]
            self.y_m, self.xy_velocity_y_m_s = y_state[0], y_state[1]
            if self.xy_trajectory_elapsed_s >= self.xy_trajectory_duration_s:
                self.x_m = self.target_x_m
                self.y_m = self.target_y_m
                self.xy_velocity_x_m_s = 0.0
                self.xy_velocity_y_m_s = 0.0
                self.xy_trajectory_duration_s = 0.0

        if self.trajectory_duration_s > 0.0:
            self.trajectory_elapsed_s = min(
                self.trajectory_elapsed_s + dt_s, self.trajectory_duration_s
            )
            position, velocity, _ = self._evaluate(
                self.trajectory_coefficients, self.trajectory_elapsed_s
            )
            self.command_z_m = position
            self.vertical_rate_m_s = velocity
            if self.trajectory_elapsed_s >= self.trajectory_duration_s:
                self.command_z_m = self.target_z_m
                self.vertical_rate_m_s = 0.0
                self.trajectory_duration_s = 0.0
        return self.current()

    def current(self) -> PositionSetpoint:
        if not self.latched:
            raise RuntimeError("position planner is not latched")
        qx, qy, qz, qw = self.orientation
        return PositionSetpoint(
            self.x_m,
            self.y_m,
            self.command_z_m,
            qx,
            qy,
            qz,
            qw,
            self.vertical_rate_m_s,
        )
