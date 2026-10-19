"""Measure a fixed-red approach using odometry and the published signal."""

from dataclasses import dataclass


@dataclass
class RedStopObservation:
    stop_line_x: float
    front_offset: float
    required_margin: float
    approach_window: float = 3.0
    speed_threshold: float = 0.15
    settle_seconds: float = 5.0

    def __post_init__(self) -> None:
        self.first_stamp: float | None = None
        self.settle_start: float | None = None
        self.max_front_x = float('-inf')
        self.samples = 0
        self.max_speed = 0.0

    def observe(self, stamp: float, x: float, speed: float, signal: str | None) -> dict | None:
        """Return a result at violation or after a stable stop near the line."""
        if signal != 'RED':
            return None
        if self.first_stamp is None:
            self.first_stamp = stamp
        self.samples += 1
        self.max_speed = max(self.max_speed, abs(speed))
        front_x = x + self.front_offset
        self.max_front_x = max(self.max_front_x, front_x)
        gap = self.stop_line_x - front_x
        if gap < 0.0:
            return self._result(stamp, x, speed, 'stop_line_crossed')
        if front_x < self.stop_line_x - self.approach_window or abs(speed) >= self.speed_threshold:
            self.settle_start = None
            return None
        if self.settle_start is None:
            self.settle_start = stamp
        if stamp - self.settle_start >= self.settle_seconds:
            reason = ('stopped_with_margin' if gap + 1e-6 >= self.required_margin
                      else 'insufficient_margin')
            return self._result(stamp, x, speed, reason)
        return None

    def _result(self, stamp: float, x: float, speed: float, reason: str) -> dict:
        front_x = x + self.front_offset
        return {
            'scenario': 'fixed_red_stop',
            'signal_observed': 'RED',
            'start_sim_time': self.first_stamp,
            'end_sim_time': stamp,
            'sample_count': self.samples,
            'stop_line_x_m': self.stop_line_x,
            'front_offset_m': self.front_offset,
            'required_margin_m': self.required_margin,
            'approach_window_m': self.approach_window,
            'speed_threshold_mps': self.speed_threshold,
            'settle_seconds': self.settle_seconds,
            'final_center_x_m': x,
            'final_front_x_m': front_x,
            'minimum_front_gap_m': self.stop_line_x - self.max_front_x,
            'final_speed_mps': abs(speed),
            'max_speed_mps': self.max_speed,
            'reason': reason,
            'passed': reason == 'stopped_with_margin',
            'measurement': 'odometry x plus configured front offset; fixed red signal ROS topic',
        }
