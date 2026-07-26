"""Compatibility facade for the modular ROS-independent safety core."""

try:
    from .core_types import (  # noqa: F401
        MAV_LANDED_STATE_ON_GROUND,
        MAV_STATE_ACTIVE,
        MAV_STATE_BOOT,
        MAV_STATE_CALIBRATING,
        MAV_STATE_STANDBY,
        MAV_STATE_UNINIT,
        MAV_SYS_STATUS_ACCEPTABLE_UNHEALTHY_MASK,
        MAV_SYS_STATUS_PREARM_CHECK,
        PositionSetpoint,
        RangeResult,
        SafetyConfig,
        Telemetry,
        clamp,
        finite,
        normalize_quaternion,
    )
    from .planner import SmoothPositionPlanner  # noqa: F401
    from .range_guard import RangeGuard  # noqa: F401
    from .safety_core import PositionSafetyCore  # noqa: F401
except ImportError:
    from core_types import (  # noqa: F401
    MAV_LANDED_STATE_ON_GROUND,
    MAV_STATE_ACTIVE,
    MAV_STATE_BOOT,
    MAV_STATE_CALIBRATING,
    MAV_STATE_STANDBY,
    MAV_STATE_UNINIT,
    MAV_SYS_STATUS_ACCEPTABLE_UNHEALTHY_MASK,
    MAV_SYS_STATUS_PREARM_CHECK,
    PositionSetpoint,
    RangeResult,
    SafetyConfig,
    Telemetry,
    clamp,
    finite,
    normalize_quaternion,
    )
    from planner import SmoothPositionPlanner  # noqa: F401
    from range_guard import RangeGuard  # noqa: F401
    from safety_core import PositionSafetyCore  # noqa: F401


__all__ = [
    "MAV_LANDED_STATE_ON_GROUND",
    "MAV_STATE_ACTIVE",
    "MAV_STATE_BOOT",
    "MAV_STATE_CALIBRATING",
    "MAV_STATE_STANDBY",
    "MAV_STATE_UNINIT",
    "MAV_SYS_STATUS_ACCEPTABLE_UNHEALTHY_MASK",
    "MAV_SYS_STATUS_PREARM_CHECK",
    "PositionSafetyCore",
    "PositionSetpoint",
    "RangeGuard",
    "RangeResult",
    "SafetyConfig",
    "SmoothPositionPlanner",
    "Telemetry",
    "clamp",
    "finite",
    "normalize_quaternion",
]
