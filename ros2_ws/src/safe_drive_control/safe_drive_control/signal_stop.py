"""Speed limit for an eastbound vehicle approaching a signal stop line."""

import math


def entered_intersection_on_green(
    entered: bool, x: float, signal_state: str | None,
    stop_line_x: float, front_offset: float,
) -> bool:
    """Keep entry permission after the front bumper crosses on green."""
    return entered or (
        signal_state == 'GREEN' and x + front_offset >= stop_line_x
    )


def turn_speed_limit(
    x: float, entry_x: float, turn_speed: float,
    deceleration: float, max_speed: float,
) -> float:
    """Slow an eastbound approach before a tight intersection turn."""
    if turn_speed <= 0.0:
        return max_speed
    remaining = max(0.0, entry_x - x)
    return min(max_speed, math.sqrt(turn_speed ** 2 + 2.0 * deceleration * remaining))


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
