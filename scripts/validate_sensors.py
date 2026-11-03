#!/usr/bin/env python3
"""Verify camera and LiDAR data against a stationary roadside target."""

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from gazebo_msgs.msg import PerformanceMetrics
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan


ROOT = Path(__file__).resolve().parents[1]
TARGET_XY = (-3.0, -4.65)


def stamp_seconds(message) -> float:
    stamp = message.header.stamp
    return stamp.sec + stamp.nanosec * 1e-9


def red_pixels(image: Image) -> int:
    if image.encoding not in ('rgb8', 'bgr8'):
        return 0
    data = bytes(image.data)
    count = 0
    for offset in range(0, len(data) - 2, 3):
        a, green, c = data[offset:offset + 3]
        red, blue = (a, c) if image.encoding == 'rgb8' else (c, a)
        if red > 110 and red > 1.5 * green and red > 1.5 * blue:
            count += 1
    return count


def save_ppm(image: Image, path: Path) -> None:
    if image.encoding not in ('rgb8', 'bgr8') or image.step != image.width * 3:
        return
    data = bytes(image.data)
    if image.encoding == 'bgr8':
        data = bytes(channel for offset in range(0, len(data), 3)
                     for channel in (data[offset + 2], data[offset + 1],
                                     data[offset]))
    header = f'P6\n{image.width} {image.height}\n255\n'.encode()
    path.write_bytes(header + data)


def target_scan_range(scan: LaserScan, odom: Odometry) -> float | None:
    pose = odom.pose.pose
    q = pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))
    bearing = math.atan2(TARGET_XY[1] - pose.position.y,
                         TARGET_XY[0] - pose.position.x) - yaw
    bearing = (bearing + math.pi) % (2 * math.pi) - math.pi
    values = [value for index, value in enumerate(scan.ranges)
              if abs(scan.angle_min + index * scan.angle_increment - bearing)
              < 0.055 and math.isfinite(value)]
    return min(values) if values else None


class SensorCollector(Node):
    def __init__(self):
        super().__init__('safe_drive_sensor_validation')
        self.images = []
        self.scans = []
        self.camera_info = None
        self.odom = None
        self.factors = []
        self.create_subscription(Image, '/safe_drive/ego/camera/image_raw',
                                 self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo,
                                 '/safe_drive/ego/camera/camera_info',
                                 self.on_info, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/safe_drive/ego/lidar/scan',
                                 self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/safe_drive/ego/odom',
                                 self.on_odom, 10)
        self.create_subscription(PerformanceMetrics, '/performance_metrics',
                                 self.on_performance, 10)

    def on_image(self, message):
        if len(self.images) < 12:
            self.images.append(message)

    def on_scan(self, message):
        if len(self.scans) < 12:
            self.scans.append(message)

    def on_info(self, message):
        self.camera_info = message

    def on_odom(self, message):
        self.odom = message

    def on_performance(self, message):
        if math.isfinite(message.real_time_factor):
            self.factors.append(message.real_time_factor)


def observed_rate(messages: list) -> float | None:
    if len(messages) < 2:
        return None
    elapsed = stamp_seconds(messages[-1]) - stamp_seconds(messages[0])
    return (len(messages) - 1) / elapsed if elapsed > 0 else None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--timeout-wall', type=float, default=90.0)
    args = parser.parse_args()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    command = ['ros2', 'launch', 'safe_drive_sim', 'simulation.launch.py',
               'gui:=false', 'spawn_x:=-15', 'spawn_y:=-1.75']
    environment = os.environ.copy()
    environment['ROS_LOG_DIR'] = str(output / 'ros_logs')
    environment['GAZEBO_LOG_PATH'] = str(output / 'gazebo_logs')
    rclpy.init()
    node = SensorCollector()
    started_utc = datetime.now(timezone.utc).isoformat()
    start = time.monotonic()
    with (output / 'launch.log').open('w', encoding='utf-8') as log:
        process = subprocess.Popen(command, cwd=ROOT / 'ros2_ws',
                                   env=environment, stdout=log,
                                   stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            while time.monotonic() - start < args.timeout_wall:
                rclpy.spin_once(node, timeout_sec=0.2)
                if (len(node.images) >= 12 and len(node.scans) >= 12
                        and node.camera_info and node.odom
                        and len(node.factors) >= 2):
                    break
                if process.poll() is not None:
                    break
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=8)
    image = node.images[-1] if node.images else None
    scan = node.scans[-1] if node.scans else None
    if image:
        save_ppm(image, output / 'camera_sample.ppm')
    target_range = (target_scan_range(scan, node.odom)
                    if scan and node.odom else None)
    result = {
        'started_utc': started_utc,
        'finished_utc': datetime.now(timezone.utc).isoformat(),
        'command': command, 'target_world_xy_m': TARGET_XY,
        'image_messages': len(node.images), 'scan_messages': len(node.scans),
        'image_frame': image.header.frame_id if image else None,
        'scan_frame': scan.header.frame_id if scan else None,
        'camera_info_frame': (node.camera_info.header.frame_id
                              if node.camera_info else None),
        'image_size': [image.width, image.height] if image else None,
        'image_encoding': image.encoding if image else None,
        'image_rate_sim_hz': observed_rate(node.images),
        'scan_rate_sim_hz': observed_rate(node.scans),
        'scan_samples': len(scan.ranges) if scan else None,
        'red_pixels': red_pixels(image) if image else None,
        'target_sector_range_m': target_range,
        'real_time_factor_samples': node.factors,
        'elapsed_wall_seconds': round(time.monotonic() - start, 3),
    }
    result['passed'] = (
        image is not None and scan is not None and node.camera_info is not None
        and result['image_frame'] == 'camera_optical_frame'
        and result['camera_info_frame'] == 'camera_optical_frame'
        and result['scan_frame'] == 'lidar_link'
        and result['image_size'] == [320, 240]
        and result['scan_samples'] == 360
        and result['red_pixels'] is not None and result['red_pixels'] >= 20
        and target_range is not None and 9.0 <= target_range <= 14.0
        and result['image_rate_sim_hz'] is not None
        and 8 <= result['image_rate_sim_hz'] <= 12
        and result['scan_rate_sim_hz'] is not None
        and 8 <= result['scan_rate_sim_hz'] <= 12
        and len(node.factors) >= 2)
    (output / 'result.json').write_text(json.dumps(result, indent=2) + '\n',
                                        encoding='utf-8')
    node.destroy_node()
    rclpy.shutdown()
    print(json.dumps(result, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
