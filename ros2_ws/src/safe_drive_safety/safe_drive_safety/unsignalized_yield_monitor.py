"""Independently measure a pedestrian-yield stop at an unsignalized crossing."""

import json
import math
from pathlib import Path
from typing import Optional

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


class UnsignalizedYieldMonitor(Node):
    """Check the front bumper against the yield line while crossing is active."""

    def __init__(self) -> None:
        super().__init__('unsignalized_yield_monitor')
        self.declare_parameter('yield_line_x', 22.75)
        self.declare_parameter('front_offset', 2.1)
        self.declare_parameter('required_margin', 0.6)
        self.declare_parameter('approach_window', 3.0)
        self.declare_parameter('stop_speed', 0.15)
        self.declare_parameter('stop_seconds', 1.0)
        self.declare_parameter('output_file', '/tmp/safe_drive_unsignalized_yield.json')
        self._line_x = float(self.get_parameter('yield_line_x').value)
        self._front_offset = float(self.get_parameter('front_offset').value)
        self._margin = float(self.get_parameter('required_margin').value)
        self._window = float(self.get_parameter('approach_window').value)
        self._stop_speed = float(self.get_parameter('stop_speed').value)
        self._stop_seconds = float(self.get_parameter('stop_seconds').value)
        self._output_file = Path(str(self.get_parameter('output_file').value))
        self._state: Optional[str] = None
        self._active_seen = False
        self._stop_start: Optional[float] = None
        self._stopped_while_active = False
        self._max_stop_duration = 0.0
        self._min_active_gap = math.inf
        self._active_samples = 0
        self._crossed_after_clear = False
        self._result_saved = False
        self._last_stamp: Optional[float] = None
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(String, '/safe_drive/scenario/state',
                                 self._on_state, qos)
        self.create_subscription(Odometry, '/safe_drive/ego/odom',
                                 self._on_odometry, 10)

    def _on_state(self, message: String) -> None:
        self._state = message.data
        if self._state == 'ACTIVE':
            self._active_seen = True
        if self._state == 'TIMED_OUT':
            self._save_result()

    def _on_odometry(self, message: Odometry) -> None:
        if self._result_saved:
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        self._last_stamp = stamp
        x = message.pose.pose.position.x
        speed = abs(message.twist.twist.linear.x)
        gap = self._line_x - (x + self._front_offset)
        if self._state == 'ACTIVE':
            self._active_samples += 1
            self._min_active_gap = min(self._min_active_gap, gap)
            if self._margin <= gap <= self._window and speed <= self._stop_speed:
                if self._stop_start is None:
                    self._stop_start = stamp
                self._max_stop_duration = max(self._max_stop_duration,
                                              stamp - self._stop_start)
                if self._max_stop_duration >= self._stop_seconds:
                    self._stopped_while_active = True
            else:
                self._stop_start = None
        elif self._state == 'COMPLETED' and x + self._front_offset >= self._line_x:
            self._crossed_after_clear = True
            self._save_result()

    def _save_result(self) -> None:
        if self._result_saved:
            return
        gap = None if math.isinf(self._min_active_gap) else self._min_active_gap
        passed = (self._active_seen and self._stopped_while_active
                  and gap is not None and gap >= self._margin
                  and self._crossed_after_clear)
        result = {
            'scenario_state': self._state,
            'yield_line_x_m': self._line_x,
            'front_offset_m': self._front_offset,
            'required_margin_m': self._margin,
            'approach_window_m': self._window,
            'stop_speed_threshold_mps': self._stop_speed,
            'required_stop_seconds': self._stop_seconds,
            'active_samples': self._active_samples,
            'minimum_front_gap_while_active_m': gap,
            'max_stop_duration_while_active_s': self._max_stop_duration,
            'stopped_while_active': self._stopped_while_active,
            'crossed_after_clear': self._crossed_after_clear,
            'end_sim_time': self._last_stamp,
            'passed': passed,
            'measurement': 'odom x plus configured front offset and scenario state',
        }
        self._output_file.parent.mkdir(parents=True, exist_ok=True)
        self._output_file.write_text(json.dumps(result, indent=2) + '\n')
        self._result_saved = True
        self.get_logger().info(f'Yield result saved: {self._output_file}; passed={passed}')


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = UnsignalizedYieldMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
