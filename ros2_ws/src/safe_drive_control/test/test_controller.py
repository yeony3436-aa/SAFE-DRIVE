import math

import pytest

from safe_drive_control.controller import normalize_angle, WaypointController


def make_controller(waypoints=None):
    return WaypointController(
        waypoints=waypoints or [(1.0, 0.0), (2.0, 0.0)],
        target_speed=1.0,
        goal_tolerance=0.1,
        lookahead_distance=0.5,
        max_linear_speed=2.0,
        max_angular_speed=1.0,
        max_linear_deceleration=1.0,
    )


def test_normalize_angle_wraps_to_expected_interval():
    assert normalize_angle(3.0 * math.pi) == pytest.approx(math.pi)
    assert normalize_angle(-3.0 * math.pi) == pytest.approx(-math.pi)


def test_controller_commands_straight_motion_for_waypoint_ahead():
    command = make_controller().calculate_command(0.0, 0.0, 0.0)

    assert command.linear == pytest.approx(1.0)
    assert command.angular == pytest.approx(0.0)


def test_controller_advances_and_stops_at_final_waypoint():
    controller = make_controller()

    controller.calculate_command(1.0, 0.0, 0.0)
    assert controller.waypoint_index == 1
    assert not controller.completed

    command = controller.calculate_command(2.0, 0.0, 0.0)
    assert controller.completed
    assert command.linear == 0.0
    assert command.angular == 0.0


def test_controller_rotates_when_waypoint_is_behind():
    controller = make_controller(waypoints=[(-1.0, 0.0)])

    command = controller.calculate_command(0.0, 0.0, 0.0)

    assert command.linear == 0.0
    assert abs(command.angular) == pytest.approx(1.0)


def test_controller_rejects_empty_route():
    with pytest.raises(ValueError, match='at least one waypoint'):
        WaypointController(
            waypoints=[],
            target_speed=1.0,
            goal_tolerance=0.1,
            lookahead_distance=0.5,
            max_linear_speed=2.0,
            max_angular_speed=1.0,
            max_linear_deceleration=1.0,
        )


def test_controller_slows_for_final_waypoint():
    controller = make_controller(waypoints=[(1.0, 0.0)])

    command = controller.calculate_command(0.6, 0.0, 0.0)

    assert 0.0 < command.linear < 1.0
