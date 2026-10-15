"""Drive separate vehicle and pedestrian signal faces from one phase clock."""

from typing import Optional

from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from safe_drive_scenario.traffic_signal import SignalTiming


class TrafficSignalNode(Node):
    """Publish both indications and update their distinct Gazebo fixtures."""

    # The vehicle face is suspended above the eastbound lane. The pedestrian
    # face sits beside the south crossing entry; neither has collision geometry.
    LAMPS = {
        'vehicle_red_lamp': 'RED',
        'vehicle_yellow_lamp': 'YELLOW',
        'vehicle_green_lamp': 'GREEN',
        'pedestrian_dont_walk_lamp': 'DONT_WALK',
        'pedestrian_walk_lamp': 'WALK',
    }

    def __init__(self) -> None:
        super().__init__('traffic_signal')
        self.declare_parameter('red_seconds', 12.0)
        self.declare_parameter('green_seconds', 16.0)
        self.declare_parameter('yellow_seconds', 3.0)
        self.declare_parameter('pedestrian_walk_seconds', 7.0)
        self.declare_parameter('fixed_state', '')
        self.declare_parameter('lamp_positions', [
            -8.43, -3.55, 3.00, -8.43, -4.10, 3.00,
            -8.43, -4.65, 3.00, -3.30, -4.87, 2.27,
            -3.30, -4.87, 1.89,
        ])
        positions = list(self.get_parameter('lamp_positions').value)
        if len(positions) != 3 * len(self.LAMPS):
            raise ValueError('lamp_positions must contain five XYZ triples')
        self._lamp_positions = [float(value) for value in positions]
        self._timing = SignalTiming(
            red_seconds=float(self.get_parameter('red_seconds').value),
            green_seconds=float(self.get_parameter('green_seconds').value),
            yellow_seconds=float(self.get_parameter('yellow_seconds').value),
            pedestrian_walk_seconds=float(
                self.get_parameter('pedestrian_walk_seconds').value
            ),
        )
        self._fixed_state = str(self.get_parameter('fixed_state').value).upper()
        if self._fixed_state not in ('', 'RED', 'YELLOW', 'GREEN'):
            raise ValueError('fixed_state must be RED, YELLOW, GREEN, or empty')

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._vehicle_publisher = self.create_publisher(
            String, '/safe_drive/vehicle_signal/state', qos
        )
        self._pedestrian_publisher = self.create_publisher(
            String, '/safe_drive/pedestrian_signal/state', qos
        )
        self._set_state_client = self.create_client(
            SetEntityState, '/safe_drive/gazebo/set_entity_state'
        )
        self._start_time = None
        self._states: Optional[tuple[str, str]] = None
        self._visual_states: Optional[tuple[str, str]] = None
        self._requested_visual_states: Optional[tuple[str, str]] = None
        self._pending = 0
        self._visual_failed = False
        self.create_timer(0.2, self._update)

    def _update(self) -> None:
        now = self.get_clock().now()
        if self._start_time is None:
            self._start_time = now
        elapsed = (now - self._start_time).nanoseconds / 1e9
        vehicle = self._fixed_state or self._timing.state_at(elapsed)
        pedestrian = (
            ('WALK' if vehicle == 'RED' else 'DONT_WALK')
            if self._fixed_state else self._timing.pedestrian_state_at(elapsed)
        )
        states = (vehicle, pedestrian)
        if states != self._states:
            self._states = states
            self.get_logger().info(
                f'Vehicle {vehicle}; pedestrian {pedestrian} at t={elapsed:.1f} s'
            )
        self._vehicle_publisher.publish(String(data=vehicle))
        self._pedestrian_publisher.publish(String(data=pedestrian))
        if self._pending or self._visual_states == states:
            return
        if not self._set_state_client.service_is_ready():
            self.get_logger().warning(
                'Waiting for Gazebo lamp service', throttle_duration_sec=5.0
            )
            return
        self._visual_failed = False
        self._pending = len(self.LAMPS)
        self._requested_visual_states = states
        for index, (name, indication) in enumerate(self.LAMPS.items()):
            x, y, z = self._lamp_positions[3 * index:3 * index + 3]
            request = SetEntityState.Request()
            request.state = EntityState()
            request.state.name = name
            request.state.reference_frame = 'world'
            request.state.pose.position.x = x
            request.state.pose.position.y = y
            request.state.pose.position.z = z if indication in states else -2.0
            request.state.pose.orientation.w = 1.0
            self._set_state_client.call_async(request).add_done_callback(
                self._on_lamp_updated
            )

    def _on_lamp_updated(self, future) -> None:
        self._pending -= 1
        try:
            response = future.result()
            if not response.success:
                self._visual_failed = True
        except Exception as error:  # pragma: no cover - ROS service failure path
            self.get_logger().error(f'Lamp update failed: {error}')
            self._visual_failed = True
        if self._pending == 0:
            if self._visual_failed:
                self.get_logger().error(
                    'Gazebo rejected a lamp update; retrying', throttle_duration_sec=2.0
                )
            else:
                self._visual_states = self._requested_visual_states


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = TrafficSignalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
