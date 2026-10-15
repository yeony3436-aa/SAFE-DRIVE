"""Simulator-independent planar safety measurements."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ActorMotion:
    """World-frame planar position and velocity in SI units."""

    x: float
    y: float
    vx: float
    vy: float


def distance_and_ttc(ego: ActorMotion, obstacle: ActorMotion) -> tuple[float, float | None]:
    """Return center distance and constant-velocity line-of-sight TTC.

    TTC is unavailable when separation or closing speed is zero, when actors
    are separating, or when input is non-finite. Lateral motion is projected
    onto the current line of sight; this is not a trajectory-intersection TTC.
    """
    values = (ego.x, ego.y, ego.vx, ego.vy,
              obstacle.x, obstacle.y, obstacle.vx, obstacle.vy)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('actor position and velocity must be finite')
    dx = obstacle.x - ego.x
    dy = obstacle.y - ego.y
    distance = math.hypot(dx, dy)
    if distance == 0.0:
        return 0.0, None
    closing_speed = ((ego.vx - obstacle.vx) * dx
                     + (ego.vy - obstacle.vy) * dy) / distance
    if closing_speed <= 0.0:
        return distance, None
    return distance, distance / closing_speed


@dataclass
class SafetyMetrics:
    """Accumulate raw observations for one scenario execution."""

    min_distance: float | None = None
    min_ttc: float | None = None
    collision: bool = False
    emergency_braking: bool = False

    def observe(self, ego: ActorMotion, pedestrian: ActorMotion) -> tuple[float, float | None]:
        """Incorporate one synchronized model-state observation."""
        distance, ttc = distance_and_ttc(ego, pedestrian)
        if self.min_distance is None or distance < self.min_distance:
            self.min_distance = distance
        if ttc is not None and (self.min_ttc is None or ttc < self.min_ttc):
            self.min_ttc = ttc
        return distance, ttc


def pedestrian_contact(states, pedestrian_name: str = 'safe_drive_pedestrian') -> bool:
    """Identify pedestrian contact among Gazebo ContactState entries."""
    return any(
        pedestrian_name in state.collision1_name
        or pedestrian_name in state.collision2_name
        for state in states
    )
