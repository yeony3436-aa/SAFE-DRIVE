"""Spawn and move the intersection pedestrian from the signal ROS topic."""

import json
import math
from pathlib import Path
from typing import Optional
from xml.etree import ElementTree

from gazebo_msgs.msg import ContactsState, EntityState
from gazebo_msgs.srv import SetEntityState, SpawnEntity
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Empty

from safe_drive_scenario.signal_crossing import SignalCrossing
from safe_drive_scenario.sudden_pedestrian import ScenarioState


class SignalCrossingNode(Node):
    def __init__(self) -> None:
        super().__init__('signal_crossing')
        self.declare_parameter('pedestrian_model', '')
        self.declare_parameter(
            'output_file', '/tmp/safe_drive_signal_crossing.json')
        self.declare_parameter('crossing_x', -5.0)
        self.declare_parameter('start_y', -4.25)
        self.declare_parameter('end_y', 5.2)
        self.declare_parameter('pedestrian_speed', 1.5)
        self.declare_parameter('update_rate_hz', 20.0)
        self.declare_parameter('scenario_id', 'SIGNAL-CROSSING-0001')
        self.declare_parameter('late_trigger_distance', 0.0)
        self.declare_parameter('late_trigger_min_speed', 0.05)
        self.declare_parameter('stop_on_contact', False)
        self.declare_parameter('pause_on_contact', False)
        self.declare_parameter('pedestrian_contact_radius', 0.28)
        model_path = str(self.get_parameter('pedestrian_model').value)
        if not model_path:
            raise ValueError('pedestrian_model is required')
        self._model_xml = Path(model_path).read_text(encoding='utf-8')
        contact_radius = float(
            self.get_parameter('pedestrian_contact_radius').value)
        self._contact_radius = contact_radius
        if not 0.0 < contact_radius <= 0.28:
            raise ValueError('pedestrian_contact_radius must be in (0, 0.28]')
        if contact_radius != 0.28:
            model_root = ElementTree.fromstring(self._model_xml)
            radius = model_root.find(
                './/collision[@name="collision"]/geometry/cylinder/radius')
            if radius is None:
                raise ValueError('pedestrian collision cylinder is missing')
            radius.text = str(contact_radius)
            self._model_xml = ElementTree.tostring(
                model_root, encoding='unicode')
        self._output_file = Path(str(self.get_parameter('output_file').value))
        self._x = float(self.get_parameter('crossing_x').value)
        self._crossing = SignalCrossing(
            float(self.get_parameter('start_y').value),
            float(self.get_parameter('end_y').value),
            float(self.get_parameter('pedestrian_speed').value),
        )
        self._scenario_id = str(self.get_parameter('scenario_id').value)
        self._late_trigger_distance = float(
            self.get_parameter('late_trigger_distance').value)
        self._late_trigger_min_speed = float(
            self.get_parameter('late_trigger_min_speed').value)
        self._stop_on_contact = bool(self.get_parameter('stop_on_contact').value)
        self._pause_on_contact = bool(self.get_parameter('pause_on_contact').value)
        if self._late_trigger_distance < 0.0:
            raise ValueError('late_trigger_distance must be nonnegative')
        if self._late_trigger_min_speed < 0.0:
            raise ValueError('late_trigger_min_speed must be nonnegative')
        self._signal: str | None = None
        self._vehicle_signal: str | None = None
        self._ego_x: float | None = None
        self._ego_speed: float | None = None
        self._start_ego_x: float | None = None
        self._start_ego_speed: float | None = None
        self._start_vehicle_signal: str | None = None
        self._spawn_requested = False
        self._spawned = False
        self._pose_pending = False
        self._last_pose_y = self._crossing.y
        self._last_time = None
        self._start_stamp: float | None = None
        self._start_signal: str | None = None
        self._finished = False
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._current_publisher = self.create_publisher(
            String, '/safe_drive/scenario/current', qos
        )
        self._state_publisher = self.create_publisher(
            String, '/safe_drive/scenario/state', qos
        )
        self.create_subscription(
            String, '/safe_drive/pedestrian_signal/state', self._on_signal, qos
        )
        if self._late_trigger_distance > 0.0:
            self.create_subscription(
                String, '/safe_drive/vehicle_signal/state',
                self._on_vehicle_signal, qos)
            self.create_subscription(
                Odometry, '/safe_drive/ego/odom', self._on_ego_odom, 10)
        if self._stop_on_contact:
            self.create_subscription(
                ContactsState, '/safe_drive/ego/contacts',
                self._on_contacts, 10)
        self._pause_client = (self.create_client(Empty, '/pause_physics')
                              if self._pause_on_contact else None)
        self._spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self._pose_client = self.create_client(
            SetEntityState, '/safe_drive/gazebo/set_entity_state'
        )
        self._current_publisher.publish(String(data=json.dumps({
            'id': self._scenario_id, 'type': 'signal_crossing', 'seed': None,
            'crossing_x': self._x, 'pedestrian_speed': self._crossing.speed,
            'late_trigger_distance': self._late_trigger_distance,
            'pedestrian_contact_radius_m': self._contact_radius,
        })))
        self._state_publisher.publish(String(data=ScenarioState.ARMED.value))
        rate = float(self.get_parameter('update_rate_hz').value)
        if rate <= 0.0:
            raise ValueError('update_rate_hz must be positive')
        self.create_timer(1.0 / rate, self._update)

    def _on_signal(self, message: String) -> None:
        self._signal = message.data

    def _on_vehicle_signal(self, message: String) -> None:
        self._vehicle_signal = message.data

    def _on_ego_odom(self, message: Odometry) -> None:
        self._ego_x = message.pose.pose.position.x
        self._ego_speed = message.twist.twist.linear.x

    def _on_contacts(self, message: ContactsState) -> None:
        if self._finished or self._crossing.state != ScenarioState.ACTIVE:
            return
        if any('safe_drive_pedestrian' in state.collision1_name
               or 'safe_drive_pedestrian' in state.collision2_name
               for state in message.states):
            self._finish(self.get_clock().now().nanoseconds / 1e9,
                         contact_terminated=True)
            if self._pause_client is not None:
                if self._pause_client.service_is_ready():
                    self._pause_client.call_async(Empty.Request())
                    self.get_logger().info('Paused Gazebo at first contact')
                else:
                    self.get_logger().warning(
                        'Gazebo pause service unavailable at contact')

    def _pose(self) -> Pose:
        pose = Pose()
        pose.position.x = self._x
        pose.position.y = self._crossing.y
        pose.orientation.z = math.sin(math.pi / 4.0)
        pose.orientation.w = math.cos(math.pi / 4.0)
        return pose

    def _update(self) -> None:
        if self._finished:
            return
        if not self._spawned:
            self._request_spawn()
            return
        now = self.get_clock().now()
        if self._last_time is None:
            self._last_time = now
            return
        elapsed = (now - self._last_time).nanoseconds / 1e9
        self._last_time = now
        if elapsed < 0.0:
            return
        if not self._pose_client.service_is_ready():
            self.get_logger().warning(
                'Gazebo entity-state service unavailable; pedestrian paused',
                throttle_duration_sec=2.0,
            )
            return
        previous = self._crossing.state
        late_trigger = (self._late_trigger_distance > 0.0
                        and self._vehicle_signal == 'GREEN'
                        and self._ego_x is not None
                        and self._ego_speed is not None
                        and self._ego_speed > self._late_trigger_min_speed
                        and 0.0 <= self._x - self._ego_x
                        <= self._late_trigger_distance)
        self._crossing.step(
            self._signal if self._late_trigger_distance == 0.0 else None,
            elapsed, trigger=late_trigger)
        if (previous == ScenarioState.ARMED
                and self._crossing.state == ScenarioState.ACTIVE):
            self._start_stamp = now.nanoseconds / 1e9
            self._start_signal = self._signal
            self._start_vehicle_signal = self._vehicle_signal
            self._start_ego_x = self._ego_x
            self._start_ego_speed = self._ego_speed
            trigger_name = ('late green approach' if late_trigger else 'WALK')
            self.get_logger().info(
                f'Pedestrian entering crossing on {trigger_name}')
            self._state_publisher.publish(
                String(data=ScenarioState.ACTIVE.value))
        if self._crossing.y != self._last_pose_y:
            self._set_pose()
        if (self._crossing.state == ScenarioState.COMPLETED
                and not self._finished and not self._pose_pending
                and self._last_pose_y == self._crossing.y):
            self._finish(now.nanoseconds / 1e9)

    def _request_spawn(self) -> None:
        if self._spawn_requested or not self._spawn_client.service_is_ready():
            return
        request = SpawnEntity.Request()
        request.name = 'safe_drive_pedestrian'
        request.xml = self._model_xml
        request.reference_frame = 'world'
        request.initial_pose = self._pose()
        self._spawn_requested = True
        self._spawn_client.call_async(request).add_done_callback(
            self._on_spawn)

    def _on_spawn(self, future) -> None:
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError(response.status_message)
        except Exception as error:
            self.get_logger().error(f'Pedestrian spawn failed: {error}')
            self._spawn_requested = False
            return
        self._spawned = True
        self._last_time = self.get_clock().now()
        waiting_for = ('green approach' if self._late_trigger_distance > 0.0
                       else 'WALK')
        self.get_logger().info(
            f'Pedestrian waiting for {waiting_for} at crosswalk')

    def _set_pose(self) -> None:
        if self._pose_pending or not self._pose_client.service_is_ready():
            return
        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = 'safe_drive_pedestrian'
        request.state.reference_frame = 'world'
        request.state.pose = self._pose()
        self._pose_pending = True
        self._last_pose_y = self._crossing.y
        self._pose_client.call_async(request).add_done_callback(self._on_pose)

    def _on_pose(self, future) -> None:
        self._pose_pending = False
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError('Gazebo rejected pedestrian pose')
        except Exception as error:
            self.get_logger().error(f'Pedestrian pose update failed: {error}')
            self._last_pose_y = float('nan')

    def _finish(self, stamp: float, contact_terminated: bool = False) -> None:
        result = {
            'scenario_id': self._scenario_id,
            'type': 'signal_crossing',
            'crossing_x_m': self._x,
            'start_y_m': self._crossing.start_y,
            'end_y_m': self._crossing.end_y,
            'pedestrian_speed_mps': self._crossing.speed,
            'start_signal': self._start_signal,
            'start_vehicle_signal': self._start_vehicle_signal,
            'start_ego_x_m': self._start_ego_x,
            'start_ego_speed_mps': self._start_ego_speed,
            'late_trigger_distance_m': self._late_trigger_distance,
            'late_trigger_min_speed_mps': self._late_trigger_min_speed,
            'pedestrian_contact_radius_m': self._contact_radius,
            'start_sim_time': self._start_stamp,
            'end_sim_time': stamp,
            'completed': not contact_terminated,
            'contact_terminated': contact_terminated,
            'final_y_m': self._crossing.y,
            'seed': None,
        }
        self._output_file.parent.mkdir(parents=True, exist_ok=True)
        self._output_file.write_text(json.dumps(result, indent=2) + '\n')
        self._state_publisher.publish(
            String(data='CONTACT' if contact_terminated
                   else ScenarioState.COMPLETED.value))
        self.get_logger().info(
            f'Pedestrian crossing result saved: {self._output_file}')
        self._finished = True


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = SignalCrossingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
