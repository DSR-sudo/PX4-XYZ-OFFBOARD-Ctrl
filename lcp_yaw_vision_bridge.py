#!/usr/bin/env python3
"""Ground-test bridge from LCP yaw to MAVROS external-vision yaw.

The bridge intentionally never enables PX4 fusion and never changes a PX4
parameter.  It accepts LCP data only while it is locked/healthy, aligns the
arbitrary LCP map heading to the current PX4 heading while disarmed on ground,
and sends a conservative PoseWithCovariance message to MAVROS.

Use only with PX4 EKF2_EV_CTRL configured for yaw fusion (bit 3) and only
after confirming the LiDAR's +X axis is aligned with the vehicle's forward
axis.  Position, height, roll and pitch covariance are deliberately large so
that this process is suitable for yaw-only testing.
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from mavros_msgs.msg import ExtendedState, State
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt8


LOCKED_STATUS = 2
ON_GROUND = 1


def wrap_pi(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return ENU yaw from a normalized or non-normalized quaternion."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-6:
        raise ValueError("invalid quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class LcpYawVisionBridge(Node):
    def __init__(self) -> None:
        super().__init__("lcp_yaw_vision_bridge")
        self.declare_parameter("lcp_yaw_offset_rad", math.nan)
        self.declare_parameter("yaw_stddev_rad", 0.20)
        self.declare_parameter("max_status_age_s", 0.35)

        configured_offset = self.get_parameter("lcp_yaw_offset_rad").value
        self.yaw_offset_rad = configured_offset if math.isfinite(configured_offset) else None
        self.yaw_stddev_rad = max(0.01, float(self.get_parameter("yaw_stddev_rad").value))
        self.max_status_age_s = max(0.05, float(self.get_parameter("max_status_age_s").value))

        self.lcp_status = 0
        self.lcp_status_at = -math.inf
        self.mavros_state = State()
        self.extended_state = ExtendedState()
        self.have_state = False
        self.have_extended_state = False
        self.mavros_yaw_rad = None
        self.mavros_imu_at = -math.inf
        self.last_reject_reason = "waiting for LCP/MAVROS data"

        self.vision_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/mavros/vision_pose/pose_cov", 10)
        self.create_subscription(UInt8, "/lcp/status", self.status_callback, 10)
        self.create_subscription(Odometry, "/lcp/odometry", self.odometry_callback, 10)
        self.create_subscription(State, "/mavros/state", self.state_callback, 10)
        self.create_subscription(ExtendedState, "/mavros/extended_state", self.extended_state_callback, 10)
        self.create_subscription(
            Imu, "/mavros/imu/data", self.imu_callback,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_timer(1.0, self.report_waiting_state)

    def status_callback(self, message: UInt8) -> None:
        self.lcp_status = message.data
        self.lcp_status_at = time.monotonic()

    def state_callback(self, message: State) -> None:
        self.mavros_state = message
        self.have_state = True

    def extended_state_callback(self, message: ExtendedState) -> None:
        self.extended_state = message
        self.have_extended_state = True

    def imu_callback(self, message: Imu) -> None:
        try:
            self.mavros_yaw_rad = yaw_from_quaternion(
                message.orientation.x, message.orientation.y,
                message.orientation.z, message.orientation.w)
            self.mavros_imu_at = time.monotonic()
        except ValueError:
            return

    def can_calibrate_offset(self) -> bool:
        return (
            self.have_state and self.have_extended_state
            and self.mavros_state.connected
            and not self.mavros_state.armed
            and self.extended_state.landed_state == ON_GROUND
            and self.mavros_yaw_rad is not None
            and time.monotonic() - self.mavros_imu_at <= self.max_status_age_s
        )

    def odometry_callback(self, message: Odometry) -> None:
        if self.lcp_status != LOCKED_STATUS or time.monotonic() - self.lcp_status_at > self.max_status_age_s:
            self.last_reject_reason = "LCP is not in fresh STATUS=2"
            return
        try:
            lcp_yaw = yaw_from_quaternion(
                message.pose.pose.orientation.x, message.pose.pose.orientation.y,
                message.pose.pose.orientation.z, message.pose.pose.orientation.w)
        except ValueError:
            self.last_reject_reason = "LCP odometry has an invalid quaternion"
            return

        if self.yaw_offset_rad is None:
            if not self.can_calibrate_offset():
                self.last_reject_reason = "waiting to calibrate on connected, disarmed ON_GROUND vehicle"
                return
            self.yaw_offset_rad = wrap_pi(self.mavros_yaw_rad - lcp_yaw)
            self.get_logger().info(
                "LCP yaw offset locked at %.4f rad; PX4 and LCP headings are now aligned" %
                self.yaw_offset_rad)

        yaw = wrap_pi(lcp_yaw + self.yaw_offset_rad)
        output = PoseWithCovarianceStamped()
        output.header.stamp = message.header.stamp
        output.header.frame_id = "lcp_map"
        # Preserve LCP XY for traceability, but EKF2_EV_CTRL remains yaw-only.
        output.pose.pose.position = message.pose.pose.position
        output.pose.pose.orientation.z = math.sin(0.5 * yaw)
        output.pose.pose.orientation.w = math.cos(0.5 * yaw)

        # Yaw has a deliberately conservative 0.20 rad standard deviation.
        # All non-yaw fields have high variance because this bridge must not be
        # used to fuse position/height/roll/pitch.
        covariance = [0.0] * 36
        for index in (0, 7, 14, 21, 28):
            covariance[index] = 10000.0
        covariance[35] = self.yaw_stddev_rad * self.yaw_stddev_rad
        output.pose.covariance = covariance
        self.vision_pub.publish(output)
        self.last_reject_reason = "publishing aligned yaw"

    def report_waiting_state(self) -> None:
        if self.yaw_offset_rad is None:
            self.get_logger().info("LCP yaw bridge: %s", self.last_reject_reason)


def main() -> None:
    rclpy.init()
    node = LcpYawVisionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
