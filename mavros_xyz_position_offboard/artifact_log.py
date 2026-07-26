"""JSONL flight artifact writer and compact telemetry snapshots."""

from __future__ import annotations

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys
from typing import Optional, TextIO

try:
    from .core_types import finite
except ImportError:
    from core_types import finite


def json_value(value):
    return None if isinstance(value, float) and not finite(value) else value


class ArtifactJsonlLogger:
    """Write one strict JSON record per status update."""

    def __init__(self, directory: str):
        self.stream: Optional[TextIO] = None
        self.path: Optional[str] = None
        self.write_error_reported = False
        artifact_dir = Path(directory).expanduser()
        if not artifact_dir.is_absolute():
            artifact_dir = Path.cwd() / artifact_dir
        try:
            artifact_dir.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            self.path = str(
                artifact_dir / f"mavros-xyz-flight-{stamp}-{os.getpid()}.jsonl"
            )
            self.stream = open(self.path, "a", encoding="utf-8", buffering=1)
        except OSError as exc:
            print(
                f"warning: unable to open artifact JSONL log in {artifact_dir}: {exc}",
                file=sys.stderr,
                flush=True,
            )

    def write(self, payload: dict) -> None:
        if self.stream is None:
            return
        try:
            json.dump(
                payload,
                self.stream,
                sort_keys=True,
                separators=(",", ":"),
                allow_nan=False,
            )
            self.stream.write("\n")
            self.stream.flush()
        except (OSError, TypeError, ValueError) as exc:
            if not self.write_error_reported:
                print(f"warning: artifact JSONL write failed: {exc}", file=sys.stderr)
                self.write_error_reported = True

    def close(self) -> None:
        if self.stream is not None:
            try:
                self.stream.flush()
                self.stream.close()
            finally:
                self.stream = None


class ArtifactSummaryLogger:
    """Write one human-readable, multi-line status summary per update."""

    def __init__(self, directory: str):
        self.stream: Optional[TextIO] = None
        self.path: Optional[str] = None
        self.write_error_reported = False
        artifact_dir = Path(directory).expanduser()
        if not artifact_dir.is_absolute():
            artifact_dir = Path.cwd() / artifact_dir
        try:
            artifact_dir.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            self.path = str(
                artifact_dir / f"mavros-xyz-flight-{stamp}-{os.getpid()}.log"
            )
            self.stream = open(self.path, "a", encoding="utf-8", buffering=1)
        except OSError as exc:
            print(
                f"warning: unable to open artifact summary log in {artifact_dir}: {exc}",
                file=sys.stderr,
                flush=True,
            )

    def write(self, summary: str) -> None:
        if self.stream is None:
            return
        try:
            self.stream.write(summary.rstrip("\n") + "\n")
            self.stream.flush()
        except OSError as exc:
            if not self.write_error_reported:
                print(f"warning: artifact summary write failed: {exc}", file=sys.stderr)
                self.write_error_reported = True

    def close(self) -> None:
        if self.stream is not None:
            try:
                self.stream.flush()
                self.stream.close()
            finally:
                self.stream = None


def build_flight_snapshot(
    telemetry: dict, setpoint: Optional[dict], relative_target_height_m: float
) -> dict:
    age = telemetry.get("telemetry_age_s", {})
    snapshot = {
        "vehicle": {
            "armed": telemetry.get("armed"),
            "mode": telemetry.get("mode"),
            "connected": telemetry.get("connected"),
            "landed_state": telemetry.get("landed_state"),
        },
        "local_xyz_m": {
            "x": json_value(telemetry.get("local_x_m")),
            "y": json_value(telemetry.get("local_y_m")),
            "z": json_value(telemetry.get("local_z_m")),
        },
        "local_velocity_m_s": {
            "x": json_value(telemetry.get("velocity_x_m_s")),
            "y": json_value(telemetry.get("velocity_y_m_s")),
            "z": json_value(telemetry.get("velocity_z_m_s")),
        },
        "setpoint_xyz_m": None,
        "setpoint_vertical_rate_m_s": None,
        "target_height_m": None,
        "relative_target_height_m": relative_target_height_m,
        "battery": {
            "present": telemetry.get("battery_present"),
            "voltage_v": json_value(telemetry.get("battery_voltage_v")),
            "fraction": json_value(telemetry.get("battery_fraction")),
        },
        "range": {
            "distance_m": json_value(telemetry.get("range_m")),
            "age_s": age.get("range"),
            "fault": telemetry.get("range_fault"),
        },
        "optical_flow": {
            "quality": telemetry.get("optical_flow_quality"),
            "age_s": age.get("optical_flow"),
            "effective": telemetry.get("optical_flow_effective"),
        },
        "lcp": {
            "status": telemetry.get("lcp_status"),
            "status_age_s": age.get("lcp_status"),
            "odometry_age_s": age.get("lcp_odometry"),
            "x_m": json_value(telemetry.get("lcp_x_m")),
            "y_m": json_value(telemetry.get("lcp_y_m")),
            "yaw_rad": json_value(telemetry.get("lcp_yaw_rad")),
            "healthy_samples": telemetry.get("lcp_healthy_samples"),
            "init_request_state": telemetry.get("lcp_init_request_state"),
            "init_failure_reason": telemetry.get("lcp_init_failure_reason"),
        },
    }
    if setpoint is not None:
        snapshot["setpoint_xyz_m"] = {
            "x": json_value(setpoint.get("x_m")),
            "y": json_value(setpoint.get("y_m")),
            "z": json_value(setpoint.get("z_m")),
        }
        snapshot["setpoint_vertical_rate_m_s"] = json_value(
            setpoint.get("vertical_rate_m_s")
        )
        snapshot["target_height_m"] = json_value(setpoint.get("target_z_m"))
    return snapshot
