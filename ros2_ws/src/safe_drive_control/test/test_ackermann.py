import math

import pytest

from safe_drive_control.ackermann import low_speed_command, steering_angle


def test_curvature_to_steering_angle_and_stop():
    assert steering_angle(2.0, 0.4, 2.5, 0.55) == pytest.approx(math.atan(0.5))
    assert steering_angle(0.0, 1.0, 2.5, 0.55) == 0.0
    assert steering_angle(1.0, -2.0, 2.5, 0.55) == -0.55


def test_low_speed_feedback_only_changes_slow_motion_and_braking():
    assert low_speed_command(0.12, 0.01, 1.0, True) == pytest.approx(0.23)
    assert low_speed_command(0.12, 0.01, 1.0, False) == 0.12
    assert low_speed_command(1.0, 0.0, 1.0, True) == 1.0
    assert low_speed_command(0.0, 0.3, 1.0, False) == pytest.approx(-0.3)
