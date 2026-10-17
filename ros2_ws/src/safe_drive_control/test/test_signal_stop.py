"""Stop-line behavior for the eastbound demonstration route."""

import pytest

from safe_drive_control.signal_stop import signal_speed_limit


def test_stop_line_uses_front_bumper_and_unknown_is_red():
    kwargs = dict(stop_line_x=-8.2, front_offset=2.1, safety_margin=0.6,
                  deceleration=1.0, max_speed=2.0)
    assert signal_speed_limit(-15, 'RED', **kwargs) == 2.0
    assert signal_speed_limit(-10.7, None, **kwargs) == 0.0
    assert signal_speed_limit(-10.7, 'GREEN', **kwargs) == 2.0
    assert signal_speed_limit(-11.7, 'YELLOW', **kwargs) == pytest.approx(1.6 ** 0.5)
    assert signal_speed_limit(-9.0, 'RED', **kwargs) == 0.0
