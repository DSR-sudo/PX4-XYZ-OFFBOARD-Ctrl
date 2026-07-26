"""Setpoint publication and MAVROS service requests."""

from __future__ import annotations

import math

try:
    from .core_types import finite
except ImportError:
    from core_types import finite
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from std_srvs.srv import Trigger


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class MavrosServicesMixin:
    def _latch_current_pose(self) -> None:
        t = self.core.telemetry
        self.planner.latch(
            t.local_x_m,
            t.local_y_m,
            t.local_z_m,
            t.orientation_x,
            t.orientation_y,
            t.orientation_z,
            t.orientation_w,
        )

    def _publish_setpoint(self, dt_s: float) -> None:
        if self.setpoint_publisher is None or not self.planner.latched:
            return
        target = self.planner.update(dt_s)
        self.planner.flow_effective = self.core.optical_flow_effective(self._now())
        message = PositionTarget()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        qx, qy, qz, qw = self.planner.orientation
        message.type_mask = (
            PositionTarget.IGNORE_VX
            | PositionTarget.IGNORE_VY
            | PositionTarget.IGNORE_VZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW_RATE
        )
        message.position.x = target.x_m
        message.position.y = target.y_m
        message.position.z = target.z_m
        message.yaw = yaw_from_quaternion(qx, qy, qz, qw)
        self.setpoint_publisher.publish(message)

    def _poll_service_futures(self, now: float) -> None:
        if self.lcp_start_future is not None:
            if self.lcp_start_future.done():
                try:
                    response = self.lcp_start_future.result()
                    message = str(response.message)
                    if bool(response.success):
                        self.core.update_lcp_init_state(
                            "accepted", now, message=message
                        )
                    else:
                        self.core.update_lcp_init_state(
                            "failed",
                            now,
                            message=message,
                            failure_reason=(
                                f"LCP initialization service rejected request: {message}"
                            ),
                        )
                    self.last_lcp_start_event = {
                        "status": "response",
                        "success": bool(response.success),
                        "message": message,
                        "monotonic_s": round(now, 6),
                    }
                except Exception as exc:
                    detail = str(exc)
                    self.core.update_lcp_init_state(
                        "failed",
                        now,
                        failure_reason=f"LCP initialization service exception: {detail}",
                    )
                    self.last_lcp_start_event = {
                        "status": "exception",
                        "detail": detail,
                        "monotonic_s": round(now, 6),
                    }
                self.lcp_start_future = None
                self.lcp_start_future_started_at = None
            elif (
                self.lcp_start_future_started_at is not None
                and now - self.lcp_start_future_started_at > self.args.service_timeout
            ):
                self.lcp_start_future.cancel()
                self.lcp_start_future = None
                self.lcp_start_future_started_at = None
                self.core.update_lcp_init_state(
                    "failed",
                    now,
                    failure_reason="LCP initialization service request timed out",
                )
                self.last_lcp_start_event = {
                    "status": "timeout",
                    "monotonic_s": round(now, 6),
                }
        if self.mode_future is not None:
            if self.mode_future.done():
                try:
                    response = self.mode_future.result()
                    self.last_mode_event = {
                        "status": "response",
                        "mode_sent": bool(response.mode_sent),
                        "monotonic_s": round(now, 6),
                    }
                except Exception as exc:
                    self.last_mode_event = {
                        "status": "exception",
                        "detail": str(exc),
                        "monotonic_s": round(now, 6),
                    }
                self.mode_future = None
                self.mode_future_started_at = None
            elif now - self.mode_future_started_at > self.args.service_timeout:
                self.mode_future.cancel()
                self.mode_future = None
                self.mode_future_started_at = None
                self.last_mode_event = {"status": "timeout", "monotonic_s": round(now, 6)}
        if self.arm_future is not None:
            if self.arm_future.done():
                try:
                    response = self.arm_future.result()
                    self.last_arm_event = {
                        "status": "response",
                        "success": bool(response.success),
                        "result": int(response.result),
                        "monotonic_s": round(now, 6),
                    }
                except Exception as exc:
                    self.last_arm_event = {
                        "status": "exception",
                        "detail": str(exc),
                        "monotonic_s": round(now, 6),
                    }
                self.arm_future = None
                self.arm_future_started_at = None
            elif now - self.arm_future_started_at > self.args.service_timeout:
                self.arm_future.cancel()
                self.arm_future = None
                self.arm_future_started_at = None
                self.last_arm_event = {"status": "timeout", "monotonic_s": round(now, 6)}

    def _request_lcp_start(self, now: float) -> None:
        if self.lcp_start_client is None:
            self.core.update_lcp_init_state(
                "failed",
                now,
                failure_reason="LCP initialization client was not created",
            )
            return
        if self.lcp_start_future is not None:
            return
        if self.core.telemetry.lcp_init_request_state in (
            "request_sent", "accepted", "failed"
        ):
            return
        if not self.lcp_start_client.service_is_ready():
            self.core.update_lcp_init_state("waiting_service", now)
            self.last_lcp_start_event = {
                "status": "service_not_ready",
                "monotonic_s": round(now, 6),
            }
            return
        try:
            request = Trigger.Request()
            self.core.begin_lcp_initialization(now)
            self.core.update_lcp_init_state("request_sent", now)
            self.lcp_start_future = self.lcp_start_client.call_async(request)
            self.lcp_start_future_started_at = now
            self.last_lcp_start_event = {
                "status": "request_sent",
                "monotonic_s": round(now, 6),
            }
        except Exception as exc:
            detail = str(exc)
            self.core.update_lcp_init_state(
                "failed",
                now,
                failure_reason=f"LCP initialization request exception: {detail}",
            )
            self.last_lcp_start_event = {
                "status": "exception",
                "detail": detail,
                "monotonic_s": round(now, 6),
            }

    def _cancel_lcp_start(self) -> None:
        if self.lcp_start_future is not None:
            self.lcp_start_future.cancel()
            self.lcp_start_future = None
            self.lcp_start_future_started_at = None

    def _request_mode(self, now: float, mode: str) -> None:
        if self.mode_client is None or self.mode_future is not None:
            return
        if now - self.last_mode_request_at < self.args.mode_request_interval:
            return
        self.last_mode_request_at = now
        if not self.mode_client.service_is_ready():
            self.last_mode_event = {"status": "service_not_ready", "mode": mode}
            return
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = mode
        self.mode_future = self.mode_client.call_async(request)
        self.mode_future_started_at = now
        self.last_mode_event = {
            "status": "request_sent",
            "mode": mode,
            "monotonic_s": round(now, 6),
        }

    def _request_arm(self, now: float, value: bool) -> None:
        if self.arm_client is None or self.arm_future is not None:
            return
        if now - self.last_arm_request_at < self.args.mode_request_interval:
            return
        self.last_arm_request_at = now
        if not self.arm_client.service_is_ready():
            self.last_arm_event = {"status": "service_not_ready", "value": value}
            return
        request = CommandBool.Request()
        request.value = value
        self.arm_future = self.arm_client.call_async(request)
        self.arm_future_started_at = now
        self.last_arm_event = {
            "status": "request_sent",
            "value": value,
            "normal_command_only": True,
            "monotonic_s": round(now, 6),
        }
