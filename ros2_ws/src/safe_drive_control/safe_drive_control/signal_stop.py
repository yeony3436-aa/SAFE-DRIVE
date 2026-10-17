"""Speed limit for an eastbound vehicle approaching a signal stop line."""

import math


def signal_speed_limit(
    x: float,
    signal_state: str | None,
    stop_line_x: float,
    front_offset: float,
    safety_margin: float,
    deceleration: float,
    max_speed: float,
) -> float:
    """Limit speed so the vehicle front remains behind the painted stop line.

    A missing signal is treated as red. A stop-line violation does not grant
    permission to accelerate through a red signal.
    """
    if signal_state == 'GREEN':
        return max_speed
    remaining = stop_line_x - safety_margin - front_offset - x
    if remaining <= 0.0:
        return 0.0
    return min(max_speed, math.sqrt(2.0 * deceleration * remaining))
