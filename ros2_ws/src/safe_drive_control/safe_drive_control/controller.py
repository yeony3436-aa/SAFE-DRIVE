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
        allow_in_place_rotation: bool = True,
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
        self._allow_in_place_rotation = allow_in_place_rotation
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

        while (
            distance <= self._goal_tolerance
            or self._has_passed_intermediate_waypoint(x, y)
            or self._has_passed_final_waypoint(x, y)
        ):
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
            if not self._allow_in_place_rotation:
                return VelocityCommand(0.0, 0.0)
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

    def _has_passed_intermediate_waypoint(self, x: float, y: float) -> bool:
        """Return whether the vehicle crossed the current route progress plane.

        Distance-only waypoint checks can miss a waypoint between control cycles.
        For a non-final waypoint, a positive projection beyond the plane through
        the waypoint means route progress has already carried the vehicle past it.
        The incoming segment defines that direction after the first waypoint; the
        first waypoint uses the direction toward the second waypoint.
        """
        if self._waypoint_index == len(self._waypoints) - 1:
            return False

        target_x, target_y = self._waypoints[self._waypoint_index]
        if self._waypoint_index == 0:
            direction_x = self._waypoints[1][0] - target_x
            direction_y = self._waypoints[1][1] - target_y
        else:
            previous_x, previous_y = self._waypoints[self._waypoint_index - 1]
            direction_x = target_x - previous_x
            direction_y = target_y - previous_y

        progress = (x - target_x) * direction_x + (y - target_y) * direction_y
        return progress > 0.0

    def _has_passed_final_waypoint(self, x: float, y: float) -> bool:
        """Accept a passed final waypoint when still on the route corridor."""
        if self._waypoint_index != len(self._waypoints) - 1 or len(self._waypoints) < 2:
            return False
        target_x, target_y = self._waypoints[-1]
        previous_x, previous_y = self._waypoints[-2]
        direction_x = target_x - previous_x
        direction_y = target_y - previous_y
        segment_length = math.hypot(direction_x, direction_y)
        if segment_length == 0.0:
            return False
        offset_x = x - target_x
        offset_y = y - target_y
        progress = offset_x * direction_x + offset_y * direction_y
        lateral_error = (
            abs(offset_x * direction_y - offset_y * direction_x) / segment_length
        )
        return progress > 0.0 and lateral_error <= self._goal_tolerance
