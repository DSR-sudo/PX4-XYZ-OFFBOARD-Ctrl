"""Status payloads, terminal formatting, and artifact emission."""

from __future__ import annotations

from datetime import datetime, timezone
import math
import shutil
import textwrap

try:
    from .artifact_log import build_flight_snapshot
except ImportError:
    from artifact_log import build_flight_snapshot


class StatusLoggingMixin:
    def _setpoint_status(self) -> dict | None:
        if not self.planner.latched:
            return None
        current = self.planner.current()
        return {
            "x_m": current.x_m,
            "y_m": current.y_m,
            "z_m": current.z_m,
            "origin_x_m": self.planner.origin_x_m,
            "origin_y_m": self.planner.origin_y_m,
            "origin_z_m": self.planner.origin_z_m,
            "target_x_m": self.planner.target_x_m,
            "target_y_m": self.planner.target_y_m,
            "target_z_m": self.planner.target_z_m,
            "vertical_rate_m_s": current.vertical_rate_m_s,
            "flow_effective": self.planner.flow_effective,
        }

    def _waypoint_status(self) -> dict | None:
        waypoints = getattr(self, "waypoints", [])
        if not waypoints:
            return None
        index = getattr(self, "waypoint_index", -1)
        target = waypoints[index] if 0 <= index < len(waypoints) else waypoints[0]
        return {
            "index": index + 1 if index >= 0 else 0,
            "count": len(waypoints),
            "target_x_m": target[0],
            "target_y_m": target[1],
        }

    def _status(self, now: float, errors: list[str]) -> dict:
        telemetry = self.core.status(now)
        setpoint = self._setpoint_status()
        return {
            "schema": "px4.mavros_native_xyz.v1",
            "logged_at_utc": datetime.now(timezone.utc).isoformat(
                timespec="milliseconds"
            ),
            "phase": self.phase,
            "result": self.result,
            "errors": errors,
            "abort_reason": self.abort_reason,
            "waypoint": self._waypoint_status(),
            "control_capabilities": {
                "setpoint_publisher_created": self.setpoint_publisher is not None,
                "mode_client_created": self.mode_client is not None,
                "arming_client_created": self.arm_client is not None,
                "lcp_start_client_created": self.lcp_start_client is not None,
            },
            "setpoint": setpoint,
            "flight_snapshot": build_flight_snapshot(
                telemetry, setpoint, self.config.relative_z_m
            ),
            "mode_service": self.last_mode_event,
            "arming_service": self.last_arm_event,
            "lcp_start_service": self.last_lcp_start_event,
            "telemetry": telemetry,
            "audit": self.audit,
        }

    def _format_summary(self, now: float, errors: list[str]) -> str:
        t = self.core.telemetry

        def rounded(value, nd=3):
            if isinstance(value, float):
                if not math.isfinite(value):
                    return None
                return round(value, nd)
            return value

        def age(timestamp):
            if timestamp is None:
                return None
            return max(0.0, now - timestamp)

        sp = self._setpoint_status()
        lines = [
            f"[{now:8.2f}] phase={self.phase} result={self.result}",
            f"  conn={t.connected} armed={t.armed} mode='{t.mode}' sys_status={t.system_status} landed={t.landed_state}",
            f"  battery={rounded(t.battery_voltage_v)}V/{rounded(t.battery_fraction)} present={t.battery_present}",
            f"  local=({rounded(t.local_x_m)},{rounded(t.local_y_m)},{rounded(t.local_z_m)}) vel=({rounded(t.velocity_x_m_s)},{rounded(t.velocity_y_m_s)},{rounded(t.velocity_z_m_s)})",
            f"  range={rounded(t.range_m)} (decl {rounded(t.range_min_m)}..{rounded(t.range_max_m)}) fault={self.core.range_guard.fault_reason}",
            f"  flow_q={t.optical_flow_quality} flow_dt_us={t.optical_flow_integration_time_us}",
            f"  lcp_io=status:{self.args.lcp_status_topic} "
            f"odom:{self.args.lcp_odometry_topic} "
            f"service:{self.args.lcp_start_service}",
            f"  lcp=status:{t.lcp_status} samples:{t.lcp_healthy_samples} "
            f"xy=({rounded(t.lcp_x_m)},{rounded(t.lcp_y_m)}) "
            f"yaw={rounded(t.lcp_yaw_rad)} "
            f"age=({rounded(age(t.lcp_status_at))},"
            f"{rounded(age(t.lcp_odometry_at))}) "
            f"init={t.lcp_init_request_state}",
        ]
        if sp is not None:
            lines.append(
                f"  setpoint=({rounded(sp['x_m'])},{rounded(sp['y_m'])},{rounded(sp['z_m'])}) "
                f"z0={rounded(sp['origin_z_m'])} target_z={rounded(sp['target_z_m'])} "
                f"vz={rounded(sp['vertical_rate_m_s'])} flow_eff={sp['flow_effective']}"
            )
        waypoint = self._waypoint_status()
        if waypoint is not None:
            lines.append(
                f"  waypoint={waypoint['index']}/{waypoint['count']} "
                f"target=({rounded(waypoint['target_x_m'])},{rounded(waypoint['target_y_m'])})"
            )
        lines.append(f"  mode_srv={self.last_mode_event}")
        lines.append(f"  arm_srv={self.last_arm_event}")
        if self.abort_reason is not None:
            lines.append(f"  abort_reason={self.abort_reason}")
        lines.append(f"  errors: {errors}")

        width = max(40, shutil.get_terminal_size(fallback=(120, 24)).columns)
        formatted_lines: list[str] = []
        for line in lines:
            prefix = "  " if line.startswith("  ") else ""
            body = line[len(prefix) :]
            formatted_lines.extend(
                textwrap.wrap(
                    body,
                    width=max(1, width - len(prefix)),
                    initial_indent=prefix,
                    subsequent_indent=prefix + "  ",
                    break_long_words=False,
                    break_on_hyphens=False,
                )
                or [prefix]
            )
        return "\n".join(formatted_lines)

    def _emit_status(self, now: float) -> None:
        summary = self._format_summary(now, self.last_errors)
        if self.args.output == "jsonl":
            payload = self._status(now, self.last_errors)
            self.artifact_log.write(payload)
        else:
            self.artifact_log.write(summary)
        print(summary, flush=True)
