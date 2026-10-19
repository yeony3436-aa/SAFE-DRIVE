"""Long-road correction should steer back toward the lane."""

from safe_drive_control.straight_tracking import straight_yaw_rate


def test_corrects_lateral_and_heading_error():
    args = dict(speed=12.0, lane_y=-1.75, lateral_gain=0.01,
                heading_gain=0.1, max_yaw_rate=1.0)
    assert straight_yaw_rate(-1.75, 0.0, **args) == 0.0
    assert straight_yaw_rate(1.0, 0.08, **args) < 0.0
    assert straight_yaw_rate(-4.0, -0.08, **args) > 0.0
