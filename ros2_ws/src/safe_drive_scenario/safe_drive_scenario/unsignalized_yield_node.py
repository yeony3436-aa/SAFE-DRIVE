"""Publish yield permission for a pedestrian at an unsignalized junction."""

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

from safe_drive_scenario.unsignalized_yield import pedestrian_requires_yield


class UnsignalizedYield(Node):
    """Translate scenario state into a fresh yield-required heartbeat."""

    def __init__(self) -> None:
        super().__init__('unsignalized_yield')
        self._state: Optional[str] = None
        self._publisher = self.create_publisher(
            Bool, '/safe_drive/ego/yield_required', 10
        )
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(String, '/safe_drive/scenario/state',
                                 self._on_state, qos)
        self.create_timer(0.1, self._publish)

    def _on_state(self, message: String) -> None:
        if message.data != self._state:
            self._state = message.data
            self.get_logger().info(f'Pedestrian scenario state {self._state}')
        self._publish()

    def _publish(self) -> None:
        message = Bool()
        message.data = pedestrian_requires_yield(self._state)
        self._publisher.publish(message)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = UnsignalizedYield()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
