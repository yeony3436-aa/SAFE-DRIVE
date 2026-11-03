"""Record painted centerline and lane-edge intrusion independently."""

import hashlib
import json
import math
from pathlib import Path
from typing import Optional

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

from safe_drive_safety.connected_lane_geometry import check_lane_lines


class ConnectedLaneMonitor(Node):
    def __init__(self) -> None:
        super().__init__('connected_lane_monitor')
        self.declare_parameter('markings_file', '')
        self.declare_parameter(
            'output_file', '/tmp/safe_drive_lane_lines.json')
        self.declare_parameter('vehicle_length', 4.2)
        self.declare_parameter('vehicle_width', 1.8)
        markings_file = Path(str(self.get_parameter('markings_file').value))
        self._markings = json.loads(markings_file.read_text())
        if self._markings.get('source_world') != 'connected_road.world':
            raise ValueError('markings must describe connected_road.world')
        self._markings_file = str(markings_file)
        self._markings_sha256 = hashlib.sha256(
            markings_file.read_bytes()).hexdigest()
        self._output_file = Path(str(self.get_parameter('output_file').value))
        self._length = float(self.get_parameter('vehicle_length').value)
        self._width = float(self.get_parameter('vehicle_width').value)
        self._samples = 0
        self._checked_samples = 0
        self._checked_corners = 0
        self._centerline_samples = 0
        self._edge_samples = 0
        self._max_centerline_m = 0.0
        self._max_edge_m = 0.0
        self._worst_centerline: Optional[dict] = None
        self._worst_edge: Optional[dict] = None
        self._first_stamp: Optional[float] = None
        self._last_stamp: Optional[float] = None
        self.create_subscription(
            Odometry, '/safe_drive/ego/odom', self._on_odom, 10)

    def _on_odom(self, message: Odometry) -> None:
        pose = message.pose.pose
        position = pose.position
        q = pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if self._first_stamp is None:
            self._first_stamp = stamp
        self._last_stamp = stamp
        self._samples += 1
        check = check_lane_lines(position.x, position.y, yaw,
                                 self._length, self._width, self._markings)
        if check.checked_corners == 0:
            return
        self._checked_samples += 1
        self._checked_corners += check.checked_corners
        snapshot = {
            'sim_time': stamp, 'x': position.x, 'y': position.y,
            'yaw': yaw, 'road': check.road, 'direction': check.direction,
        }
        if check.centerline_overlap_m > 1e-9:
            self._centerline_samples += 1
        if check.edge_line_overlap_m > 1e-9:
            self._edge_samples += 1
        if check.centerline_overlap_m > self._max_centerline_m:
            self._max_centerline_m = check.centerline_overlap_m
            self._worst_centerline = snapshot
        if check.edge_line_overlap_m > self._max_edge_m:
            self._max_edge_m = check.edge_line_overlap_m
            self._worst_edge = snapshot

    def save(self) -> None:
        valid = self._checked_samples > 0
        result = {
            'measurement': (
                'chassis corners overlapping painted centerline or lane-edge '
                'footprints on marked straight arms; unpainted junctions '
                'excluded'),
            'markings_file': self._markings_file,
            'markings_sha256': self._markings_sha256,
            'vehicle_length_m': self._length,
            'vehicle_width_m': self._width,
            'start_sim_time': self._first_stamp,
            'end_sim_time': self._last_stamp,
            'odom_samples': self._samples,
            'checked_samples': self._checked_samples,
            'checked_corners': self._checked_corners,
            'centerline_overlap_samples': self._centerline_samples,
            'edge_line_overlap_samples': self._edge_samples,
            'max_centerline_overlap_m': self._max_centerline_m,
            'max_edge_line_overlap_m': self._max_edge_m,
            'worst_centerline_pose': self._worst_centerline,
            'worst_edge_pose': self._worst_edge,
            'measurement_valid': valid,
            'passed': (valid and self._centerline_samples == 0
                       and self._edge_samples == 0),
        }
        self._output_file.parent.mkdir(parents=True, exist_ok=True)
        self._output_file.write_text(json.dumps(result, indent=2) + '\n')
        self.get_logger().info(
            f'Painted-lane result saved: {self._output_file}; '
            f'passed={result["passed"]}')


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ConnectedLaneMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
