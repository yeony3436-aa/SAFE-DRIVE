"""Record route completion and a map-based lane-corridor measurement."""

import json
import math
from pathlib import Path
from typing import Optional

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
import yaml

from safe_drive_safety.route_geometry import (
    extended_path, point_to_path_distance, vehicle_corners,
)


class RouteMonitor(Node):
    def __init__(self) -> None:
        super().__init__('route_monitor')
        self.declare_parameter('route_file', '')
        self.declare_parameter('output_file', '/tmp/safe_drive_route_result.json')
        self.declare_parameter('lane_half_width', 1.75)
        self.declare_parameter('vehicle_length', 4.2)
        self.declare_parameter('vehicle_width', 1.8)
        self.declare_parameter('goal_tolerance', 0.75)
        route_file = Path(str(self.get_parameter('route_file').value))
        config = yaml.safe_load(route_file.read_text())
        params = config['waypoint_follower']['ros__parameters']
        self._route_parameters = params
        self._waypoints = tuple(zip(map(float, params['waypoint_x']),
                                    map(float, params['waypoint_y'])))
        self._path = extended_path(self._waypoints)
        self._route_file = str(route_file)
        self._output_file = Path(str(self.get_parameter('output_file').value))
        self._lane_half_width = float(self.get_parameter('lane_half_width').value)
        self._length = float(self.get_parameter('vehicle_length').value)
        self._width = float(self.get_parameter('vehicle_width').value)
        self._goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self._samples = 0
        self._outside_samples = 0
        self._severe_outside_samples = 0
        self._max_center_error = 0.0
        self._max_corner_error = 0.0
        self._max_speed = 0.0
        self._worst_pose: Optional[dict] = None
        self._first_stamp: Optional[float] = None
        self._last_trace_stamp: Optional[float] = None
        self._trace: list[dict] = []
        self._near_goal_stamp: Optional[float] = None
        self._finished = False
        self.create_subscription(Odometry, '/safe_drive/ego/odom', self._on_odom, 10)

    def _on_odom(self, message: Odometry) -> None:
        if self._finished:
            return
        position = message.pose.pose.position
        rotation = message.pose.pose.orientation
        yaw = math.atan2(2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                         1.0 - 2.0 * (rotation.y ** 2 + rotation.z ** 2))
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        if self._first_stamp is None:
            self._first_stamp = stamp
        center_error = point_to_path_distance((position.x, position.y), self._path)
        corner_error = max(point_to_path_distance(corner, self._path)
                           for corner in vehicle_corners(position.x, position.y,
                                                         yaw, self._length, self._width))
        self._samples += 1
        self._max_center_error = max(self._max_center_error, center_error)
        if corner_error > self._max_corner_error:
            self._max_corner_error = corner_error
            self._worst_pose = {'x': position.x, 'y': position.y, 'yaw': yaw}
        if corner_error > self._lane_half_width:
            self._outside_samples += 1
        self._severe_outside_samples = (
            self._severe_outside_samples + 1 if corner_error > 4.0 else 0
        )

        goal_distance = math.dist((position.x, position.y), self._waypoints[-1])
        speed = abs(message.twist.twist.linear.x)
        self._max_speed = max(self._max_speed, speed)
        if self._last_trace_stamp is None or stamp - self._last_trace_stamp >= 1.0:
            self._trace.append({
                'sim_time': stamp, 'x': position.x, 'y': position.y,
                'yaw': yaw, 'speed_mps': speed,
            })
            self._last_trace_stamp = stamp
        if self._severe_outside_samples >= 10:
            result = {
                'route_file': self._route_file,
                'route_parameters': self._route_parameters,
                'seed': None,
                'start_sim_time': self._first_stamp,
                'end_sim_time': stamp,
                'sample_count': self._samples,
                'goal_reached': False,
                'goal_position': {'x': position.x, 'y': position.y},
                'goal_distance_m': goal_distance,
                'final_speed_mps': speed,
                'max_speed_mps': self._max_speed,
                'max_corner_error_m': self._max_corner_error,
                'lane_half_width_m': self._lane_half_width,
                'outside_corridor_samples': self._outside_samples,
                'corridor_clear': False,
                'worst_pose': self._worst_pose,
                'passed': False,
                'failure_reason': 'left_route_corridor',
                'trace_1hz': self._trace,
            }
            self._output_file.parent.mkdir(parents=True, exist_ok=True)
            self._output_file.write_text(json.dumps(result, indent=2) + '\n')
            self.get_logger().error(f'Route corridor exited: {self._output_file}')
            self._finished = True
            return
        if goal_distance <= max(1.0, self._goal_tolerance) and self._near_goal_stamp is None:
            self._near_goal_stamp = stamp
        if self._near_goal_stamp is not None and stamp - self._near_goal_stamp >= 10.0:
            result = {
                'route_file': self._route_file,
                'route_parameters': self._route_parameters,
                'seed': None,
                'start_sim_time': self._first_stamp,
                'end_sim_time': stamp,
                'sample_count': self._samples,
                'goal_reached': goal_distance <= self._goal_tolerance and speed < 0.15,
                'goal_position': {'x': position.x, 'y': position.y},
                'goal_distance_m': goal_distance,
                'final_speed_mps': speed,
                'max_speed_mps': self._max_speed,
                'lane_half_width_m': self._lane_half_width,
                'vehicle_length_m': self._length,
                'vehicle_width_m': self._width,
                'goal_tolerance_m': self._goal_tolerance,
                'max_center_error_m': self._max_center_error,
                'max_corner_error_m': self._max_corner_error,
                'outside_corridor_samples': self._outside_samples,
                'corridor_clear': self._outside_samples == 0,
                'passed': goal_distance <= self._goal_tolerance
                          and speed < 0.15 and self._outside_samples == 0,
                'worst_pose': self._worst_pose,
                'measurement': 'distance of chassis corners to configured route polyline',
                'trace_1hz': self._trace,
            }
            self._output_file.parent.mkdir(parents=True, exist_ok=True)
            self._output_file.write_text(json.dumps(result, indent=2) + '\n')
            self.get_logger().info(f'Route result saved: {self._output_file}; '
                                   f'max corner error={self._max_corner_error:.2f} m')
            self._finished = True


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = RouteMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
