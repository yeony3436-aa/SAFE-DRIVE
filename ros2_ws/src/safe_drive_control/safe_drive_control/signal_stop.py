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

    A missing signal is treated as red. Once the front has passed the line,
    completing the crossing is safer than stopping in the intersection.
    """
    if signal_state == 'GREEN' or x + front_offset >= stop_line_x:
        return max_speed
    remaining = stop_line_x - safety_margin - front_offset - x
    if remaining <= 0.0:
        return 0.0
    return min(max_speed, math.sqrt(2.0 * deceleration * remaining))
