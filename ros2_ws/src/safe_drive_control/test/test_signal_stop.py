"""Stop-line behavior for the eastbound demonstration route."""

import pytest

from safe_drive_control.signal_stop import (
    entered_intersection_on_green, signal_speed_limit, turn_speed_limit,
)


def test_stop_line_uses_front_bumper_and_unknown_is_red():
    kwargs = dict(stop_line_x=-8.2, front_offset=2.1, safety_margin=0.6,
                  deceleration=1.0, max_speed=2.0)
    assert signal_speed_limit(-15, 'RED', **kwargs) == 2.0
    assert signal_speed_limit(-10.7, None, **kwargs) == 0.0
    assert signal_speed_limit(-10.7, 'GREEN', **kwargs) == 2.0
    assert signal_speed_limit(-11.7, 'YELLOW', **kwargs) == pytest.approx(1.6 ** 0.5)
    assert signal_speed_limit(-9.0, 'RED', **kwargs) == 0.0


def test_green_entry_is_retained_after_signal_changes():
    line = -9.25
    assert not entered_intersection_on_green(False, -12.0, 'GREEN', line, 2.1)
    assert not entered_intersection_on_green(False, -11.0, 'RED', line, 2.1)
    assert entered_intersection_on_green(False, -11.0, 'GREEN', line, 2.1)
    assert entered_intersection_on_green(True, -7.0, 'RED', line, 2.1)


def test_turn_speed_is_limited_before_tight_curve():
    assert turn_speed_limit(-100.0, -12.0, 2.0, 2.5, 16.67) == 16.67
    assert turn_speed_limit(-12.0, -12.0, 2.0, 2.5, 16.67) == 2.0
    assert turn_speed_limit(0.0, -12.0, 2.0, 2.5, 16.67) == 2.0
