#!/usr/bin/env python3
"""Console entry point for the modular MAVROS XYZ node."""

from __future__ import annotations

import json
import sys

try:
    from .cli import parse_args
except ImportError:
    from cli import parse_args


def main(argv: list[str] | None = None) -> int:
    args, ros_args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        try:
            from .ros_app import run
        except ImportError:
            from ros_app import run
    except ImportError as exc:
        print(
            json.dumps(
                {
                    "schema": "px4.mavros_native_xyz.v1",
                    "phase": "startup_error",
                    "errors": [f"ROS 2 Jazzy/MAVROS import failed: {exc}"],
                },
                sort_keys=True,
            ),
            file=sys.stderr,
        )
        return 2
    return run(args, ros_args)


if __name__ == "__main__":
    raise SystemExit(main())
