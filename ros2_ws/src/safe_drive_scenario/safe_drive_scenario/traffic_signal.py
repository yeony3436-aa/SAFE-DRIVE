"""Deterministic vehicle-signal timing, independent of ROS and Gazebo."""

from dataclasses import dataclass


@dataclass(frozen=True)
class SignalTiming:
    red_seconds: float = 12.0
    green_seconds: float = 16.0
    yellow_seconds: float = 3.0
    pedestrian_walk_seconds: float = 10.5

    def __post_init__(self) -> None:
        if min(self.red_seconds, self.green_seconds, self.yellow_seconds) <= 0.0:
            raise ValueError('All signal phase durations must be positive')
        if not 0.0 < self.pedestrian_walk_seconds < self.red_seconds:
            raise ValueError('Pedestrian walk time must be within the red phase')

    def _offset(self, elapsed_seconds: float) -> float:
        cycle = self.red_seconds + self.green_seconds + self.yellow_seconds
        return max(0.0, elapsed_seconds) % cycle

    def state_at(self, elapsed_seconds: float) -> str:
        """Start red; repeat red, green, yellow in simulation time."""
        offset = self._offset(elapsed_seconds)
        if offset < self.red_seconds:
            return 'RED'
        if offset < self.red_seconds + self.green_seconds:
            return 'GREEN'
        return 'YELLOW'

    def pedestrian_state_at(self, elapsed_seconds: float) -> str:
        """Permit walking early in vehicle red, then reserve clearance time."""
        return ('WALK' if self._offset(elapsed_seconds) < self.pedestrian_walk_seconds
                else 'DONT_WALK')
