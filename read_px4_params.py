#!/usr/bin/env python3
"""Read PX4 parameter values via MAVROS and check against expected values.

Uses the (deprecated but still functional) /mavros/param/get service to fetch
each parameter by name from the FCU, then prints current vs. expected values.

Prerequisite:
    ros2 launch mavros px4.launch fcu_url:=...   # MAVROS already running

Run:
    python3 read_px4_params.py
"""

from __future__ import annotations

import sys

import rclpy
from rclpy.node import Node

from mavros_msgs.srv import ParamGet


# param_id -> (description, expected raw value or None if info-only)
EXPECTED: dict[str, tuple[str, int | float | None]] = {
    "MAV_1_CONFIG":   ("TELEM n (restart)",       None),
    "MAV_1_MODE":     ("Normal",                  None),
    "MAV_PROTO_VER":  ("1 (MAVLink v1)",          1),
    "SER_TEL1_BAUD":  ("115200 8N1",              115200),
    "EKF2_OF_CTRL":   ("Enabled",                 1),
    "EKF2_RNG_CTRL":  ("Enabled",                 1),
    "EKF2_HGT_REF":   ("Range sensor",            1),
    "SENS_FLOW_ROT":  ("No rotation",             0),
}


def _raw(value) -> int | float | str:
    if value.integer != 0:
        return value.integer
    if value.real != 0.0:
        return value.real
    return 0


class ParamReader(Node):
    def __init__(self) -> None:
        super().__init__("px4_param_reader")
        self._cli = self.create_client(ParamGet, "/mavros/param/get")

    def fetch(self, param_id: str):
        if not self._cli.wait_for_service(timeout_sec=5.0):
            return None
        req = ParamGet.Request()
        req.param_id = param_id
        future = self._cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or future.result() is None:
            return None
        res = future.result()
        if not res.success:
            return None
        return res.value


def main() -> int:
    rclpy.init(args=None)
    node = ParamReader()
    print(
        f"\n{'PARAM':<18}{'CURRENT':<20}{'EXPECTED':<22}DESCRIPTION"
    )
    print("-" * 90)

    rc = 0
    for param_id, (desc, expected) in EXPECTED.items():
        value = node.fetch(param_id)
        if value is None:
            current = "<missing>"
            status = "NOT FOUND"
            rc = 1
        else:
            current = str(_raw(value))
            if expected is None:
                status = "(info-only)"
            elif str(expected) == current:
                status = "OK"
            else:
                status = "MISMATCH"
                rc = 1
        expected_str = "" if expected is None else f"{expected}"
        print(
            f"{param_id:<18}{current:<20}{expected_str:<22}"
            f"{desc} [{status}]"
        )

    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
