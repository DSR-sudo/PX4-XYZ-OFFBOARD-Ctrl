#!/usr/bin/env python3
"""Ground-test bridge from north-referenced LCP XY/yaw to MAVROS external vision.

The bridge intentionally never enables PX4 fusion and never changes a PX4
parameter.  It accepts only locked LCP data in the LCP source's ``lcp_nwu``
frame.  That frame is created when an operator places the vehicle forward axis
on true north before initialization: +X=north, +Y=west/left, +Z=up and yaw=0
faces north.  The bridge converts all of XY and yaw together into ROS ENU for
MAVROS; it never estimates or silently applies a PX4-heading offset.

Use only with PX4 EKF2_EV_CTRL configured for the intended XY/yaw fusion and
only after confirming the LiDAR's +X axis is aligned with the vehicle's
forward axis.  Height, roll and pitch covariance remain deliberately large;
XY and yaw use conservative configurable uncertainties.
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import UInt8


LOCKED_STATUS = 2


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
        self.declare_parameter("lcp_input_frame", "lcp_nwu")
        self.declare_parameter("xy_stddev_m", 0.20)
        self.declare_parameter("yaw_stddev_rad", 0.20)
        self.declare_parameter("max_status_age_s", 0.35)

        self.lcp_input_frame = str(self.get_parameter("lcp_input_frame").value)
        self.xy_stddev_m = max(0.02, float(self.get_parameter("xy_stddev_m").value))
        self.yaw_stddev_rad = max(0.01, float(self.get_parameter("yaw_stddev_rad").value))
        self.max_status_age_s = max(0.05, float(self.get_parameter("max_status_age_s").value))

        self.lcp_status = 0
        self.lcp_status_at = -math.inf
        self.last_publish_at = -math.inf
        self.last_reject_reason = "waiting for LCP data"

        self.vision_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/mavros/vision_pose/pose_cov", 10)
        self.create_subscription(UInt8, "/lcp/status", self.status_callback, 10)
        self.create_subscription(Odometry, "/lcp/odometry", self.odometry_callback, 10)
        self.create_timer(1.0, self.report_waiting_state)

    def status_callback(self, message: UInt8) -> None:
        self.lcp_status = message.data
        self.lcp_status_at = time.monotonic()

    def odometry_callback(self, message: Odometry) -> None:
        if self.lcp_status != LOCKED_STATUS or time.monotonic() - self.lcp_status_at > self.max_status_age_s:
            self.last_reject_reason = "LCP is not in fresh STATUS=2"
            return
        if message.header.frame_id != self.lcp_input_frame:
            self.last_reject_reason = (
                "LCP frame is %r, expected %r; restart LCP with "
                "lcp_initial_heading_is_north=true" %
                (message.header.frame_id, self.lcp_input_frame))
            return
        try:
            lcp_yaw = yaw_from_quaternion(
                message.pose.pose.orientation.x, message.pose.pose.orientation.y,
                message.pose.pose.orientation.z, message.pose.pose.orientation.w)
        except ValueError:
            self.last_reject_reason = "LCP odometry has an invalid quaternion"
            return

        # lcp_nwu (+X north, +Y west) -> ROS ENU (+X east, +Y north).
        # The same +pi/2 rotation applies to position axes and yaw.
        yaw = wrap_pi(0.5 * math.pi + lcp_yaw)
        output = PoseWithCovarianceStamped()
        output.header.stamp = message.header.stamp
        output.header.frame_id = "lcp_enu"
        output.pose.pose.position.x = -message.pose.pose.position.y
        output.pose.pose.position.y = message.pose.pose.position.x
        output.pose.pose.position.z = message.pose.pose.position.z
        output.pose.pose.orientation.z = math.sin(0.5 * yaw)
        output.pose.pose.orientation.w = math.cos(0.5 * yaw)

        # XY and yaw have deliberately conservative uncertainties.  Height,
        # roll and pitch remain high variance because this test only fuses XY/yaw.
        covariance = [0.0] * 36
        for index in (14, 21, 28):
            covariance[index] = 10000.0
        covariance[0] = self.xy_stddev_m * self.xy_stddev_m
        covariance[7] = self.xy_stddev_m * self.xy_stddev_m
        covariance[35] = self.yaw_stddev_rad * self.yaw_stddev_rad
        output.pose.covariance = covariance
        self.vision_pub.publish(output)
        self.last_publish_at = time.monotonic()
        self.last_reject_reason = "publishing north-referenced LCP XY/yaw as ENU"

    def report_waiting_state(self) -> None:
        if time.monotonic() - self.last_publish_at > self.max_status_age_s:
            self.get_logger().info(f"LCP yaw bridge: {self.last_reject_reason}")


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
