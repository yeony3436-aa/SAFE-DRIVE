"""Save a fixed-red stop-line result independently of the route controller."""

import json
from pathlib import Path
from typing import Optional

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from safe_drive_safety.red_stop import RedStopObservation


class RedStopMonitor(Node):
    def __init__(self) -> None:
        super().__init__('red_stop_monitor')
        self.declare_parameter('output_file', '/tmp/safe_drive_red_stop.json')
        self.declare_parameter('stop_line_x', -9.25)
        self.declare_parameter('front_offset', 2.1)
        self.declare_parameter('required_margin', 0.6)
        self.declare_parameter('approach_window', 3.0)
        self.declare_parameter('speed_threshold', 0.15)
        self.declare_parameter('settle_seconds', 5.0)
        self._output_file = Path(str(self.get_parameter('output_file').value))
        self._observation = RedStopObservation(
            stop_line_x=float(self.get_parameter('stop_line_x').value),
            front_offset=float(self.get_parameter('front_offset').value),
            required_margin=float(self.get_parameter('required_margin').value),
            approach_window=float(self.get_parameter('approach_window').value),
            speed_threshold=float(self.get_parameter('speed_threshold').value),
            settle_seconds=float(self.get_parameter('settle_seconds').value),
        )
        self._signal: str | None = None
        self._finished = False
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(String, '/safe_drive/vehicle_signal/state', self._on_signal, qos)
        self.create_subscription(Odometry, '/safe_drive/ego/odom', self._on_odom, 10)

    def _on_signal(self, message: String) -> None:
        self._signal = message.data

    def _on_odom(self, message: Odometry) -> None:
        if self._finished:
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        result = self._observation.observe(
            stamp, message.pose.pose.position.x,
            message.twist.twist.linear.x, self._signal,
        )
        if result is None:
            return
        self._output_file.parent.mkdir(parents=True, exist_ok=True)
        self._output_file.write_text(json.dumps(result, indent=2) + '\n')
        self.get_logger().info(
            f'Red stop result saved: {self._output_file}; '
            f'passed={result["passed"]}; gap={result["minimum_front_gap_m"]:.2f} m'
        )
        self._finished = True


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = RedStopMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
