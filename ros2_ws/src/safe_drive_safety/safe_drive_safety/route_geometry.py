"""Map-based route corridor measurements in metres."""

import math
from typing import Sequence


Point = tuple[float, float]


def extended_path(waypoints: Sequence[Point], before: float = 20.0,
                  after: float = 6.0) -> tuple[Point, ...]:
    """Extend the marked lane through the spawn and beyond the goal."""
    if len(waypoints) < 2:
        raise ValueError('at least two waypoints are required')
    first, second = waypoints[:2]
    previous, last = waypoints[-2:]
    first_length = math.dist(first, second)
    last_length = math.dist(previous, last)
    if first_length == 0.0 or last_length == 0.0:
        raise ValueError('end route segments must have nonzero length')
    entry = (first[0] - before * (second[0] - first[0]) / first_length,
             first[1] - before * (second[1] - first[1]) / first_length)
    exit_point = (last[0] + after * (last[0] - previous[0]) / last_length,
                  last[1] + after * (last[1] - previous[1]) / last_length)
    return (entry, *waypoints, exit_point)


def point_to_path_distance(point: Point, path: Sequence[Point]) -> float:
    """Minimum Euclidean distance from a point to a route polyline."""
    if len(path) < 2:
        raise ValueError('path needs at least two points')
    best = math.inf
    for start, end in zip(path, path[1:]):
        dx, dy = end[0] - start[0], end[1] - start[1]
        length_sq = dx * dx + dy * dy
        if length_sq == 0.0:
            continue
        progress = max(0.0, min(1.0,
            ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_sq))
        best = min(best, math.hypot(point[0] - start[0] - progress * dx,
                                   point[1] - start[1] - progress * dy))
    return best


def vehicle_corners(x: float, y: float, yaw: float,
                    length: float, width: float) -> tuple[Point, ...]:
    """The four chassis corners in map coordinates."""
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return tuple((x + forward * cosine - lateral * sine,
                  y + forward * sine + lateral * cosine)
                 for forward in (-length / 2.0, length / 2.0)
                 for lateral in (-width / 2.0, width / 2.0))
