"""Heading and cross-track correction for a long eastbound lane."""

import math


def straight_yaw_rate(
    y: float, yaw: float, speed: float, lane_y: float,
    lateral_gain: float, heading_gain: float, max_yaw_rate: float,
) -> float:
    """Return an eastbound lane-centering yaw rate in radians per second."""
    heading_error = math.atan2(math.sin(yaw), math.cos(yaw))
    curvature = -lateral_gain * (y - lane_y) - heading_gain * heading_error
    return max(-max_yaw_rate, min(max_yaw_rate, speed * curvature))
