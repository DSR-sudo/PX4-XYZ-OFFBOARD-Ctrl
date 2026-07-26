"""ROS 2 application wiring for the modular XYZ controller."""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import (
    EstimatorStatus,
    ExtendedState,
    OpticalFlowRad,
    PositionTarget,
    State,
    SysStatus,
)
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import BatteryState, Range
from std_msgs.msg import UInt8
from std_srvs.srv import Trigger

try:
    from .artifact_log import ArtifactJsonlLogger, ArtifactSummaryLogger
    from .cli import arming_enabled, make_config, mode_enabled, setpoint_enabled
    from .core import PositionSafetyCore
    from .node_callbacks import MavrosCallbacksMixin
    from .node_flight import FlightStateMixin
    from .node_services import MavrosServicesMixin
    from .node_status import StatusLoggingMixin
    from .planner import SmoothPositionPlanner
except ImportError:
    from artifact_log import ArtifactJsonlLogger, ArtifactSummaryLogger
    from cli import arming_enabled, make_config, mode_enabled, setpoint_enabled
    from core import PositionSafetyCore
    from node_callbacks import MavrosCallbacksMixin
    from node_flight import FlightStateMixin
    from node_services import MavrosServicesMixin
    from node_status import StatusLoggingMixin
    from planner import SmoothPositionPlanner


class MavrosNativeXYZNode(
    Node,
    MavrosCallbacksMixin,
    MavrosServicesMixin,
    StatusLoggingMixin,
    FlightStateMixin,
):
    def __init__(self, args, config) -> None:
        super().__init__("mavros_native_xyz_position")
        self.args = args
        self.config = config
        self.core = PositionSafetyCore(config)
        self.planner = SmoothPositionPlanner(config)
        self.phase = "monitor_only"
        self.last_phase = ""
        self.result = "UNCONFIRMED"
        self.abort_reason = None
        self.phase_started_at = None
        self.flight_started_at = None
        self.waypoints: list[tuple[float, float]] = []
        self.waypoint_index = -1
        self.setpoint_stream_since = None
        self.last_tick_at = None
        self.last_status_at = float("-inf")
        self.last_errors: list[str] = []
        self.sensor_loss_started_at = None
        self.lcp_hold_started_at = None
        self.lcp_hold_resume_phase = None
        self.lcp_hold_resume_phase_started_at = None
        self.lcp_hold_resume_target_x_m = None
        self.lcp_hold_resume_target_y_m = None
        self.lcp_hold_resume_target_z_m = None
        self.artifact_log = (
            ArtifactJsonlLogger(args.artifact_dir)
            if args.output == "jsonl"
            else ArtifactSummaryLogger(args.artifact_dir)
        )

        self.publish_enabled = setpoint_enabled(args)
        self.mode_enabled = mode_enabled(args)
        self.arming_enabled = arming_enabled(args)
        self.setpoint_publisher = None
        self.mode_client = None
        self.arm_client = None
        self.mode_future = None
        self.arm_future = None
        self.lcp_start_client = None
        self.lcp_start_future = None
        self.mode_future_started_at = None
        self.arm_future_started_at = None
        self.lcp_start_future_started_at = None
        self.last_mode_request_at = float("-inf")
        self.last_arm_request_at = float("-inf")
        self.last_mode_event = {"status": "not_enabled"}
        self.last_arm_event = {"status": "not_enabled"}
        self.last_lcp_start_event = {"status": "never_requested"}

        sensor_qos = qos_profile_sensor_data
        state_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(State, args.state_topic, self._state, state_qos)
        self.create_subscription(SysStatus, args.sys_status_topic, self._sys_status, sensor_qos)
        self.create_subscription(BatteryState, args.battery_topic, self._battery, sensor_qos)
        self.create_subscription(
            ExtendedState, args.extended_state_topic, self._extended_state, state_qos
        )
        self.create_subscription(PoseStamped, args.local_pose_topic, self._local_pose, sensor_qos)
        self.create_subscription(
            TwistStamped, args.local_velocity_topic, self._local_velocity, sensor_qos
        )
        self.create_subscription(
            EstimatorStatus, args.estimator_status_topic, self._estimator_status, sensor_qos
        )
        self.create_subscription(Range, args.range_topic, self._range, sensor_qos)
        self.create_subscription(
            OpticalFlowRad, args.optical_flow_topic, self._optical_flow, sensor_qos
        )
        lcp_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            UInt8, args.lcp_status_topic, self._lcp_status, lcp_qos
        )
        self.create_subscription(
            Odometry, args.lcp_odometry_topic, self._lcp_odometry, lcp_qos
        )

        if self.publish_enabled:
            self.setpoint_publisher = self.create_publisher(
                PositionTarget, args.setpoint_topic, sensor_qos
            )
            self.phase = "waiting_preflight"
        if self.mode_enabled:
            self.mode_client = self.create_client(SetMode, "/mavros/set_mode")
            self.last_mode_event = {"status": "never_requested"}
        if self.arming_enabled:
            self.arm_client = self.create_client(CommandBool, "/mavros/cmd/arming")
            self.last_arm_event = {"status": "never_requested"}
        self.lcp_start_client = self.create_client(Trigger, args.lcp_start_service)

        self.audit = {
            "target_hardware": "PX4 FMUv6C.x / Pi 5 / MTF02P",
            "target_firmware": "PX4 1.17.0",
            "target_ros_mavros": "ROS 2 Jazzy / MAVROS 2.14.0",
            "confirmed_fcu_url": args.confirmed_fcu_url,
            "topics": {
                "state": args.state_topic,
                "sys_status": args.sys_status_topic,
                "battery": args.battery_topic,
                "landed": args.extended_state_topic,
                "local_pose": args.local_pose_topic,
                "local_velocity": args.local_velocity_topic,
                "estimator_status": args.estimator_status_topic,
                "range": args.range_topic,
                "optical_flow": args.optical_flow_topic,
                "setpoint": args.setpoint_topic,
                "lcp_status": args.lcp_status_topic,
                "lcp_odometry": args.lcp_odometry_topic,
            },
            "services": {
                "lcp_start_initialization": args.lcp_start_service,
            },
            "range_source_label": args.range_source_label,
            "optical_flow_source_label": args.optical_flow_source_label,
            "range_source_confirmed": args.confirm_range_source,
            "optical_flow_source_confirmed": args.confirm_optical_flow_source,
            "ignore_declared_min_range": args.ignore_declared_min_range,
            "ack_range_below_declared_min": args.ack_range_below_declared_min,
            "sensor_loss_grace_s": config.sensor_loss_grace_s,
            "lcp_status_timeout_s": config.lcp_status_timeout_s,
            "lcp_odometry_timeout_s": config.lcp_odometry_timeout_s,
            "lcp_ready_samples": config.lcp_ready_samples,
            "lcp_unhealthy_hold_timeout_s": config.lcp_unhealthy_hold_timeout_s,
            "lcp_control_policy": (
                "STATUS=3 during climb is ignored; after 10 s fixed-height hold, "
                "fresh STATUS=2 and odometry are required before waypoints; yaw=0"
            ),
            "artifact_log_path": self.artifact_log.path,
            "setpoint_frame": "ROS ENU PositionTarget on setpoint_raw/local; MAVROS converts to MAVLink LOCAL_NED",
            "setpoint_mav_frame_confirmed_local_ned": args.confirm_setpoint_mav_frame_local_ned,
            "setpoint_policy": "full position hold (PX/PY/PZ + YAW); Z driven by quintic trajectory",
            "waypoint_policy": (
                "four 0.5 m body-relative legs: forward, left, backward, right; "
                "final target returns to initial XY before landing"
            ),
            "waypoint_frame": "initial locked yaw in ROS ENU local XY",
            "waypoint_leg_m": config.waypoint_leg_m,
            "waypoint_max_speed_m_s": config.waypoint_max_speed_m_s,
            "waypoint_max_accel_m_s2": config.waypoint_max_accel_m_s2,
            "waypoint_tolerance_m": config.waypoint_tolerance_m,
            "parameter_writes": False,
            "force_arm_or_disarm": False,
            "px4_xy_fusion_evidence_label": args.px4_xy_fusion_evidence_label or None,
        }
        self.timer = self.create_timer(1.0 / config.publish_rate_hz, self._tick)

    @staticmethod
    def _now() -> float:
        return time.monotonic()


def run(args, ros_args: list[str]) -> int:
    config = make_config(args)
    rclpy.init(args=ros_args)
    node = MavrosNativeXYZNode(args, config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.artifact_log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
