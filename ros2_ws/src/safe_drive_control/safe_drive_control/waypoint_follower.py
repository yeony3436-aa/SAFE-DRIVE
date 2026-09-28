"""ROS 2 node that follows a configured waypoint route."""

import math
from typing import Optional

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

from safe_drive_control.controller import WaypointController


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return planar yaw from a quaternion without an extra TF dependency."""
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


class WaypointFollower(Node):
    """Convert odometry and configured waypoints into Twist commands."""

    def __init__(self) -> None:
        super().__init__('waypoint_follower')

        self.declare_parameter('target_speed', 2.0)
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('lookahead_distance', 1.5)
        self.declare_parameter('max_linear_speed', 3.0)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('max_linear_deceleration', 1.0)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('odom_timeout', 0.5)
        self.declare_parameter('waypoint_x', [-10.0, 0.0, 10.0])
        self.declare_parameter('waypoint_y', [-1.75, -1.75, -1.75])

        waypoint_x = list(self.get_parameter('waypoint_x').value)
        waypoint_y = list(self.get_parameter('waypoint_y').value)
        if len(waypoint_x) != len(waypoint_y):
            raise ValueError('waypoint_x and waypoint_y must have equal lengths')

        waypoints = [
            (float(x), float(y))
            for x, y in zip(waypoint_x, waypoint_y)
        ]
        self._controller = WaypointController(
            waypoints=waypoints,
            target_speed=float(self.get_parameter('target_speed').value),
            goal_tolerance=float(self.get_parameter('goal_tolerance').value),
            lookahead_distance=float(self.get_parameter('lookahead_distance').value),
            max_linear_speed=float(self.get_parameter('max_linear_speed').value),
            max_angular_speed=float(self.get_parameter('max_angular_speed').value),
            max_linear_deceleration=float(
                self.get_parameter('max_linear_deceleration').value
            ),
        )

        control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        if control_rate_hz <= 0.0:
            raise ValueError('control_rate_hz must be positive')
        self._odom_timeout = float(self.get_parameter('odom_timeout').value)
        if self._odom_timeout <= 0.0:
            raise ValueError('odom_timeout must be positive')

        self._position: Optional[tuple[float, float, float]] = None
        self._last_odom_time = None
        self._last_reported_waypoint = -1
        self._completion_reported = False
        self._waiting_for_odom_reported = False

        self._command_publisher = self.create_publisher(
            Twist,
            '/safe_drive/ego/cmd_vel',
            10,
        )
        self.create_subscription(
            Odometry,
            '/safe_drive/ego/odom',
            self._on_odometry,
            10,
        )
        self.create_timer(1.0 / control_rate_hz, self._control_step)

        self.get_logger().info(
            f'Loaded {len(waypoints)} waypoints; target_speed='
            f'{self.get_parameter("target_speed").value:.2f} m/s'
        )

    def _on_odometry(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        self._position = (position.x, position.y, yaw)
        self._last_odom_time = self.get_clock().now()

    def _control_step(self) -> None:
        if self._position is None or self._last_odom_time is None:
            self._publish_stop()
            if not self._waiting_for_odom_reported:
                self.get_logger().info('Waiting for ego odometry')
                self._waiting_for_odom_reported = True
            return

        odom_age = (self.get_clock().now() - self._last_odom_time).nanoseconds / 1e9
        if odom_age > self._odom_timeout:
            self._publish_stop()
            self.get_logger().error(
                f'Odometry stale for {odom_age:.2f} s; commanding stop',
                throttle_duration_sec=2.0,
            )
            return

        command = self._controller.calculate_command(*self._position)
        message = Twist()
        message.linear.x = command.linear
        message.angular.z = command.angular
        self._command_publisher.publish(message)

        if self._controller.completed:
            if not self._completion_reported:
                x, y, _ = self._position
                self.get_logger().info(
                    f'Final waypoint reached at x={x:.2f}, y={y:.2f}; vehicle stopped'
                )
                self._completion_reported = True
            return

        if self._controller.waypoint_index != self._last_reported_waypoint:
            self._last_reported_waypoint = self._controller.waypoint_index
            self.get_logger().info(
                f'Following waypoint {self._last_reported_waypoint + 1}'
            )

    def _publish_stop(self) -> None:
        self._command_publisher.publish(Twist())

    def stop(self) -> None:
        """Publish an explicit stop before node shutdown."""
        self._publish_stop()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
