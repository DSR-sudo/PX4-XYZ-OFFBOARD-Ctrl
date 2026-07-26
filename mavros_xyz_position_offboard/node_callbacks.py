"""MAVROS subscription callbacks."""

from __future__ import annotations

import math
from typing import Any


class MavrosCallbacksMixin:
    """Translate ROS messages into PositionSafetyCore updates."""

    def _state(self, message: Any) -> None:
        self.core.update_state(
            bool(message.connected),
            bool(message.armed),
            str(message.mode),
            int(message.system_status),
            self._now(),
        )

    def _sys_status(self, message: Any) -> None:
        self.core.update_sys_status(
            int(message.sensors_present),
            int(message.sensors_enabled),
            int(message.sensors_health),
            self._now(),
        )

    def _battery(self, message: Any) -> None:
        self.core.update_battery(
            bool(message.present),
            float(message.voltage),
            float(message.percentage),
            self._now(),
        )

    def _extended_state(self, message: Any) -> None:
        self.core.update_landed(int(message.landed_state), self._now())

    def _local_pose(self, message: Any) -> None:
        p = message.pose.position
        q = message.pose.orientation
        self.core.update_local_pose(
            float(p.x),
            float(p.y),
            float(p.z),
            float(q.x),
            float(q.y),
            float(q.z),
            float(q.w),
            self._now(),
        )

    def _local_velocity(self, message: Any) -> None:
        v = message.twist.linear
        self.core.update_local_velocity(
            float(v.x), float(v.y), float(v.z), self._now()
        )

    def _estimator_status(self, message: Any) -> None:
        self.core.update_estimator(
            attitude_valid=bool(message.attitude_status_flag),
            velocity_horiz_valid=bool(message.velocity_horiz_status_flag),
            velocity_vert_valid=bool(message.velocity_vert_status_flag),
            pos_horiz_rel_valid=bool(message.pos_horiz_rel_status_flag),
            pos_horiz_abs_valid=bool(message.pos_horiz_abs_status_flag),
            pos_vert_abs_valid=bool(message.pos_vert_abs_status_flag),
            pos_vert_agl_valid=bool(message.pos_vert_agl_status_flag),
            const_pos_mode=bool(message.const_pos_mode_status_flag),
            gps_glitch=bool(message.gps_glitch_status_flag),
            accel_error=bool(message.accel_error_status_flag),
            now=self._now(),
        )

    def _range(self, message: Any) -> None:
        self.core.update_range(
            float(message.range),
            float(message.min_range),
            float(message.max_range),
            self._now(),
        )

    def _optical_flow(self, message: Any) -> None:
        self.core.update_optical_flow(
            integration_time_us=int(message.integration_time_us),
            integrated_x_rad=float(message.integrated_x),
            integrated_y_rad=float(message.integrated_y),
            quality=int(message.quality),
            distance_m=float(message.distance),
            distance_delta_us=int(message.time_delta_distance_us),
            temperature_c=float(message.temperature),
            now=self._now(),
        )

    def _lcp_status(self, message: Any) -> None:
        self.core.update_lcp_status(int(message.data), self._now())

    def _lcp_odometry(self, message: Any) -> None:
        pose = message.pose.pose
        position = pose.position
        orientation = pose.orientation
        x = float(orientation.x)
        y = float(orientation.y)
        z = float(orientation.z)
        w = float(orientation.w)
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if not math.isfinite(norm) or norm < 1e-6:
            yaw = math.nan
        else:
            x, y, z, w = x / norm, y / norm, z / norm, w / norm
            yaw = math.atan2(
                2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)
            )
        self.core.update_lcp_odometry(
            float(position.x), float(position.y), yaw, self._now()
        )
