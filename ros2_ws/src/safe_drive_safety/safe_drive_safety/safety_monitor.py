"""ROS 2 adapter for Gazebo ground-truth safety measurements."""

import json
from pathlib import Path
from typing import Optional

from gazebo_msgs.msg import ContactsState, ModelStates
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64, String
import yaml

from safe_drive_safety.metrics import ActorMotion, pedestrian_contact, SafetyMetrics


class SafetyMonitor(Node):
    """Track one scenario using simulator model states and contact events."""

    def __init__(self) -> None:
        super().__init__('safety_monitor')
        self.declare_parameter('safety_config', '')
        config_path = str(self.get_parameter('safety_config').value)
        if not config_path:
            raise ValueError('safety_config is required')
        with Path(config_path).open(encoding='utf-8') as config_file:
            safety = yaml.safe_load(config_file)['safety']
        self._braking_threshold = float(safety['emergency_braking_deceleration'])
        if self._braking_threshold <= 0.0:
            raise ValueError('emergency_braking_deceleration must be positive')

        self._metrics = SafetyMetrics()
        self._scenario_id: Optional[str] = None
        self._state = 'WAITING_FOR_EGO'
        self._finalized = False
        self._previous_command: Optional[tuple[float, float]] = None
        self._ttc_publisher = self.create_publisher(
            Float64, '/safe_drive/safety/ttc', 10
        )
        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._result_publisher = self.create_publisher(
            String, '/safe_drive/safety/result', latched_qos
        )
        self.create_subscription(
            String, '/safe_drive/scenario/current', self._on_current, latched_qos
        )
        self.create_subscription(
            String, '/safe_drive/scenario/state', self._on_state, latched_qos
        )
        self.create_subscription(
            ModelStates, '/safe_drive/gazebo/model_states', self._on_models, 10
        )
        self.create_subscription(
            ContactsState, '/safe_drive/ego/contacts', self._on_contacts, 10
        )
        self.create_subscription(
            Twist, '/safe_drive/ego/cmd_vel', self._on_command, 10
        )

    def _on_current(self, message: String) -> None:
        self._scenario_id = str(json.loads(message.data)['id'])
        self._publish_result()

    def _on_state(self, message: String) -> None:
        self._state = message.data
        if self._state in ('COMPLETED', 'TIMED_OUT'):
            self._finalized = True
        self._publish_result()

    def _on_models(self, message: ModelStates) -> None:
        if self._finalized or self._state not in ('ARMED', 'ACTIVE'):
            return
        try:
            ego_index = message.name.index('safe_drive_ego')
            pedestrian_index = message.name.index('safe_drive_pedestrian')
        except ValueError:
            return

        def motion(index: int) -> ActorMotion:
            pose = message.pose[index].position
            velocity = message.twist[index].linear
            return ActorMotion(pose.x, pose.y, velocity.x, velocity.y)

        _, ttc = self._metrics.observe(motion(ego_index), motion(pedestrian_index))
        ttc_message = Float64()
        ttc_message.data = ttc if ttc is not None else float('nan')
        self._ttc_publisher.publish(ttc_message)
        self._publish_result()

    def _on_contacts(self, message: ContactsState) -> None:
        if not self._finalized and pedestrian_contact(message.states):
            self._metrics.collision = True
            self._publish_result()

    def _on_command(self, message: Twist) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        speed = message.linear.x
        if self._previous_command is not None and not self._finalized:
            previous_time, previous_speed = self._previous_command
            elapsed = now - previous_time
            if elapsed > 0.0 and previous_speed > speed:
                deceleration = (previous_speed - speed) / elapsed
                if deceleration >= self._braking_threshold:
                    self._metrics.emergency_braking = True
                    self._publish_result()
        self._previous_command = (now, speed)

    def _publish_result(self) -> None:
        if self._scenario_id is None:
            return
        message = String()
        message.data = json.dumps({
            'scenario_id': self._scenario_id,
            'state': self._state,
            'min_distance': self._metrics.min_distance,
            'min_ttc': self._metrics.min_ttc,
            'collision': self._metrics.collision,
            'emergency_braking': self._metrics.emergency_braking,
            'scenario_completed': self._state == 'COMPLETED',
        }, sort_keys=True)
        self._result_publisher.publish(message)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
