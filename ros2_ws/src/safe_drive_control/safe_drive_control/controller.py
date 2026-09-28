"""Simulator-independent waypoint following calculations."""

from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class VelocityCommand:
    """Planar velocity command returned by the controller."""

    linear: float
    angular: float


def normalize_angle(angle: float) -> float:
    """Normalize an angle to the closed-open interval [-pi, pi)."""
    return math.atan2(math.sin(angle), math.cos(angle))


class WaypointController:
    """Small deterministic controller for an ordered planar waypoint route."""

    def __init__(
        self,
        waypoints: Sequence[tuple[float, float]],
        target_speed: float,
        goal_tolerance: float,
        lookahead_distance: float,
        max_linear_speed: float,
        max_angular_speed: float,
        max_linear_deceleration: float,
    ) -> None:
        if not waypoints:
            raise ValueError('at least one waypoint is required')
        if target_speed <= 0.0:
            raise ValueError('target_speed must be positive')
        if goal_tolerance <= 0.0:
            raise ValueError('goal_tolerance must be positive')
        if lookahead_distance <= 0.0:
            raise ValueError('lookahead_distance must be positive')
        if max_linear_speed <= 0.0 or max_angular_speed <= 0.0:
            raise ValueError('velocity limits must be positive')
        if max_linear_deceleration <= 0.0:
            raise ValueError('max_linear_deceleration must be positive')

        self._waypoints = tuple(waypoints)
        self._target_speed = min(target_speed, max_linear_speed)
        self._goal_tolerance = goal_tolerance
        self._lookahead_distance = lookahead_distance
        self._max_angular_speed = max_angular_speed
        self._max_linear_deceleration = max_linear_deceleration
        self._waypoint_index = 0
        self._completed = False

    @property
    def waypoint_index(self) -> int:
        return self._waypoint_index

    @property
    def completed(self) -> bool:
        return self._completed

    def calculate_command(
        self,
        x: float,
        y: float,
        yaw: float,
    ) -> VelocityCommand:
        """Calculate the next command and advance waypoints when reached."""
        if self._completed:
            return VelocityCommand(0.0, 0.0)

        target_x, target_y = self._waypoints[self._waypoint_index]
        distance = math.hypot(target_x - x, target_y - y)

        while distance <= self._goal_tolerance:
            if self._waypoint_index == len(self._waypoints) - 1:
                self._completed = True
                return VelocityCommand(0.0, 0.0)

            self._waypoint_index += 1
            target_x, target_y = self._waypoints[self._waypoint_index]
            distance = math.hypot(target_x - x, target_y - y)

        target_heading = math.atan2(target_y - y, target_x - x)
        heading_error = normalize_angle(target_heading - yaw)

        # Rotate in place when the target is outside the forward field of view.
        if abs(heading_error) > math.pi / 2.0:
            angular = max(
                -self._max_angular_speed,
                min(self._max_angular_speed, 1.5 * heading_error),
            )
            return VelocityCommand(0.0, angular)

        # Pure-pursuit curvature with a lower-bounded lookahead avoids an
        # excessively sharp command when approaching a waypoint.
        effective_lookahead = max(distance, self._lookahead_distance)
        linear = self._target_speed * max(0.2, math.cos(heading_error))

        if self._waypoint_index == len(self._waypoints) - 1:
            stopping_distance = max(0.0, distance - self._goal_tolerance)
            stopping_speed = math.sqrt(
                2.0 * self._max_linear_deceleration * stopping_distance
            )
            linear = min(linear, stopping_speed)

        curvature = 2.0 * math.sin(heading_error) / effective_lookahead
        angular = linear * curvature
        angular = max(-self._max_angular_speed, min(self._max_angular_speed, angular))

        return VelocityCommand(linear, angular)
