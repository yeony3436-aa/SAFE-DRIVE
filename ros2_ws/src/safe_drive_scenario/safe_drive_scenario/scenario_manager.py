"""ROS 2 integration for the sudden-pedestrian scenario."""

import json
import math
from pathlib import Path
from typing import Optional

from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState, SpawnEntity
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from safe_drive_scenario.sudden_pedestrian import (
    load_sudden_pedestrian_config,
    ScenarioState,
    SuddenPedestrianScenario,
)


class ScenarioManager(Node):
    """Spawn, trigger, and move one configured pedestrian actor."""

    def __init__(self) -> None:
        super().__init__('scenario_manager')
        self.declare_parameter('scenario_config', '')
        self.declare_parameter('pedestrian_model', '')

        config_path = str(self.get_parameter('scenario_config').value)
        model_path = str(self.get_parameter('pedestrian_model').value)
        if not config_path or not model_path:
            raise ValueError('scenario_config and pedestrian_model are required')

        self._config = load_sudden_pedestrian_config(config_path)
        self._scenario = SuddenPedestrianScenario(self._config)
        self._model_xml = Path(model_path).read_text(encoding='utf-8')
        self._ego_x: Optional[float] = None
        self._spawn_requested = False
        self._spawned = False
        self._set_state_pending = False
        self._last_requested_y: Optional[float] = None
        self._last_update_time = None
        self._last_published_state: Optional[ScenarioState] = None
        self._pedestrian_name = 'safe_drive_pedestrian'

        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._current_publisher = self.create_publisher(
            String, '/safe_drive/scenario/current', latched_qos
        )
        self._state_publisher = self.create_publisher(
            String, '/safe_drive/scenario/state', latched_qos
        )
        self.create_subscription(
            Odometry, '/safe_drive/ego/odom', self._on_odometry, 10
        )
        self._spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self._set_state_client = self.create_client(
            SetEntityState, '/safe_drive/gazebo/set_entity_state'
        )
        self.create_timer(1.0 / self._config.update_rate_hz, self._update)

        self._publish_current_scenario()
        self._publish_state()
        self.get_logger().info(
            f'Loaded {self._config.scenario_id} type=sudden_pedestrian '
            f'seed={self._config.seed}'
        )

    def _publish_current_scenario(self) -> None:
        message = String()
        message.data = json.dumps({
            'id': self._config.scenario_id,
            'type': 'sudden_pedestrian',
            'seed': self._config.seed,
            'ego_speed': self._config.ego_speed,
            'pedestrian_speed': self._config.pedestrian_speed,
            'pedestrian_spawn_distance': self._config.pedestrian_spawn_distance,
            'trigger_distance': self._config.trigger_distance,
        }, sort_keys=True)
        self._current_publisher.publish(message)

    def _publish_state(self) -> None:
        if self._scenario.state == self._last_published_state:
            return
        message = String()
        message.data = self._scenario.state.value
        self._state_publisher.publish(message)
        self._last_published_state = self._scenario.state
        self.get_logger().info(
            f'{self._config.scenario_id} state={self._scenario.state.value}'
        )

    def _on_odometry(self, message: Odometry) -> None:
        self._ego_x = message.pose.pose.position.x
        if self._scenario.state == ScenarioState.WAITING_FOR_EGO:
            self._scenario.arm(self._ego_x)
            self.get_logger().info(
                f'Pedestrian crossing configured at x={self._scenario.crossing_x:.2f} m'
            )
            self._publish_state()

    def _update(self) -> None:
        if self._ego_x is None:
            return
        if not self._spawned:
            self._request_spawn()
            return

        now = self.get_clock().now()
        if self._last_update_time is None:
            self._last_update_time = now
            return
        elapsed = (now - self._last_update_time).nanoseconds / 1e9
        self._last_update_time = now
        if elapsed < 0.0:
            self.get_logger().warning('Simulation time moved backwards; resetting timer')
            return
        if not self._set_state_client.service_is_ready():
            self.get_logger().error(
                'Gazebo entity-state service unavailable; scenario motion paused',
                throttle_duration_sec=2.0,
            )
            return

        previous_state = self._scenario.state
        self._scenario.step(self._ego_x, elapsed)
        if self._scenario.pedestrian_y != self._last_requested_y:
            self._set_pedestrian_pose()
        if self._scenario.state != previous_state:
            self._publish_state()

    def _request_spawn(self) -> None:
        if self._spawn_requested or not self._spawn_client.service_is_ready():
            return
        assert self._scenario.crossing_x is not None
        request = SpawnEntity.Request()
        request.name = self._pedestrian_name
        request.xml = self._model_xml
        request.reference_frame = 'world'
        request.initial_pose = self._pedestrian_pose()
        self._spawn_requested = True
        future = self._spawn_client.call_async(request)
        future.add_done_callback(self._on_spawn_complete)

    def _on_spawn_complete(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:  # pragma: no cover - ROS service failure path
            self.get_logger().error(f'Pedestrian spawn service failed: {error}')
            self._spawn_requested = False
            return
        if not response.success:
            self.get_logger().error(
                f'Pedestrian spawn rejected: {response.status_message}'
            )
            self._spawn_requested = False
            return
        self._spawned = True
        self._last_requested_y = self._scenario.pedestrian_y
        self._last_update_time = self.get_clock().now()
        self.get_logger().info('Pedestrian spawned and scenario armed')

    def _pedestrian_pose(self) -> Pose:
        pose = Pose()
        pose.position.x = float(self._scenario.crossing_x)
        pose.position.y = self._scenario.pedestrian_y
        pose.orientation.z = math.sin(math.pi / 4.0)
        pose.orientation.w = math.cos(math.pi / 4.0)
        return pose

    def _set_pedestrian_pose(self) -> None:
        if self._set_state_pending or not self._set_state_client.service_is_ready():
            return
        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = self._pedestrian_name
        request.state.pose = self._pedestrian_pose()
        request.state.reference_frame = 'world'
        self._set_state_pending = True
        self._last_requested_y = self._scenario.pedestrian_y
        future = self._set_state_client.call_async(request)
        future.add_done_callback(self._on_set_state_complete)

    def _on_set_state_complete(self, future) -> None:
        self._set_state_pending = False
        try:
            response = future.result()
        except Exception as error:  # pragma: no cover - ROS service failure path
            self.get_logger().error(f'Pedestrian state update failed: {error}')
            self._last_requested_y = None
            return
        if not response.success:
            self.get_logger().error('Gazebo rejected pedestrian state update')
            self._last_requested_y = None


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ScenarioManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
