"""Independent lane-line geometry for the generated connected road."""

from dataclasses import dataclass
import math
from typing import Sequence

from safe_drive_safety.route_geometry import vehicle_corners


@dataclass(frozen=True)
class LaneCheck:
    road: str | None
    direction: str | None
    checked_corners: int
    centerline_overlap_m: float
    edge_line_overlap_m: float


def _on_marked_interval(position: float,
                        intervals: Sequence[Sequence[float]]) -> bool:
    return any(start <= position <= end for start, end in intervals)


def check_lane_lines(x: float, y: float, yaw: float,
                     length: float, width: float,
                     markings: dict) -> LaneCheck:
    """Check chassis corners against painted line footprints on straight arms.

    Junction gaps are excluded because the connected world paints no
    centerline or lane-edge line through those turning areas.
    """
    horizontal = abs(math.cos(yaw)) >= abs(math.sin(yaw))
    axis = 'x' if horizontal else 'y'
    center_position = y if horizontal else x
    roads = [road for road in markings['roads'] if road['axis'] == axis]
    road = min(roads, key=lambda candidate: abs(
        center_position - candidate['center']))
    if abs(center_position - road['center']) > 5.0:
        return LaneCheck(None, None, 0, 0.0, 0.0)

    if horizontal:
        positive = math.cos(yaw) >= 0
        direction = 'east' if positive else 'west'
    else:
        positive = math.sin(yaw) >= 0
        direction = 'north' if positive else 'south'

    edge_offset = float(markings['lane_edge_offset_m'])
    center_half = float(markings['centerline_width_m']) / 2
    edge_half = float(markings['edge_line_width_m']) / 2
    checked = 0
    center_overlap = 0.0
    edge_overlap = 0.0
    for corner_x, corner_y in vehicle_corners(x, y, yaw, length, width):
        along = corner_x if horizontal else corner_y
        if not _on_marked_interval(along, road['intervals']):
            continue
        checked += 1
        lateral = (corner_y if horizontal else corner_x) - road['center']
        # Right-hand driving: east is south of its centerline; north is east.
        signed = (-lateral if horizontal and positive else
                  lateral if horizontal else
                  lateral if positive else -lateral)
        center_overlap = max(center_overlap, center_half - signed, 0.0)
        edge_overlap = max(edge_overlap,
                           signed - (edge_offset - edge_half), 0.0)
    return LaneCheck(road['name'], direction, checked,
                     center_overlap, edge_overlap)
