import math

import pytest

from safe_drive_control.controller import (
    normalize_angle, waypoint_curvature, WaypointController,
)


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


def test_controller_advances_when_first_waypoint_was_passed():
    controller = make_controller()

    command = controller.calculate_command(1.2, 0.0, 0.0)

    assert controller.waypoint_index == 1
    assert not controller.completed
    assert command.linear > 0.0
    assert command.angular == pytest.approx(0.0)


def test_controller_advances_when_intermediate_waypoint_was_passed():
    controller = make_controller(
        waypoints=[(1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
    )
    controller.calculate_command(1.0, 0.0, 0.0)

    command = controller.calculate_command(2.2, 0.0, 0.0)

    assert controller.waypoint_index == 2
    assert not controller.completed
    assert command.linear > 0.0
    assert command.angular == pytest.approx(0.0)


def test_controller_does_not_mark_passed_final_waypoint_complete():
    controller = make_controller(waypoints=[(1.0, 0.0)])

    command = controller.calculate_command(1.2, 0.0, 0.0)

    assert controller.waypoint_index == 0
    assert not controller.completed
    assert command.linear == 0.0
    assert abs(command.angular) == pytest.approx(1.0)


def test_controller_stops_after_passing_final_waypoint_on_route():
    controller = make_controller(waypoints=[(1.0, 0.0), (2.0, 0.0)])
    controller.calculate_command(1.0, 0.0, 0.0)

    command = controller.calculate_command(2.3, 0.0, 0.0)

    assert controller.completed
    assert command.linear == 0.0
    assert command.angular == 0.0


def test_controller_does_not_complete_after_passing_goal_off_route():
    controller = make_controller(waypoints=[(1.0, 0.0), (2.0, 0.0)])
    controller.calculate_command(1.0, 0.0, 0.0)

    controller.calculate_command(2.3, 0.3, 0.0)

    assert not controller.completed


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


def test_controller_brakes_before_reaching_final_waypoint():
    controller = WaypointController(
        waypoints=[(2.0, 0.0), (4.0, 0.0), (6.0, 0.0)],
        target_speed=4.0, goal_tolerance=0.1,
        lookahead_distance=0.5, max_linear_speed=4.0,
        max_angular_speed=1.0, max_linear_deceleration=1.0,
    )

    command = controller.calculate_command(2.5, 0.0, 0.0)

    assert controller.waypoint_index == 1
    assert 0.0 < command.linear < 4.0
    assert command.linear == pytest.approx(math.sqrt(2.0 * 3.4))


def test_speed_limit_applies_before_turn_curvature():
    controller = make_controller(waypoints=[(2.0, 1.0), (4.0, 1.0)])
    command = controller.calculate_command(0.0, 0.0, 0.0, speed_limit=0.4)

    assert command.linear == pytest.approx(0.4)
    assert command.angular < 0.4


def test_path_heading_gain_corrects_vehicle_yaw_without_changing_default():
    route = [(2.0, 0.0), (4.0, 0.0)]
    standard = make_controller(route).calculate_command(0.0, 0.0, 0.3)
    aligned = WaypointController(
        waypoints=route, target_speed=1.0, goal_tolerance=0.1,
        lookahead_distance=0.5, max_linear_speed=2.0,
        max_angular_speed=1.0, max_linear_deceleration=1.0,
        path_heading_gain=0.5,
    ).calculate_command(0.0, 0.0, 0.3)

    assert aligned.angular < standard.angular < 0.0


def test_path_heading_gain_must_be_finite_and_nonnegative():
    for gain in (-0.1, math.inf):
        with pytest.raises(ValueError, match='path_heading_gain'):
            WaypointController(
                waypoints=[(1.0, 0.0)], target_speed=1.0,
                goal_tolerance=0.1, lookahead_distance=0.5,
                max_linear_speed=2.0, max_angular_speed=1.0,
                max_linear_deceleration=1.0, path_heading_gain=gain,
            )


def test_curve_speed_profile_slows_before_a_bend():
    route = [(1.0, 0.0), (2.0, 0.0), (2.0, 1.0), (2.0, 2.0)]
    assert waypoint_curvature(*route[:3]) > 0.0
    standard = WaypointController(
        route, 1.6, 0.1, 0.5, 2.0, 2.0, 0.8)
    limited = WaypointController(
        route, 1.6, 0.1, 0.5, 2.0, 2.0, 0.8,
        max_lateral_acceleration=0.36)

    standard_command = standard.calculate_command(0.0, 0.0, 0.0)
    limited_command = limited.calculate_command(0.0, 0.0, 0.0)
    assert 0.0 < limited_command.linear < standard_command.linear


def test_lateral_acceleration_limit_rejects_invalid_value():
    with pytest.raises(ValueError, match='max_lateral_acceleration'):
        WaypointController(
            [(1.0, 0.0)], 1.0, 0.1, 0.5, 2.0, 1.0, 1.0,
            max_lateral_acceleration=-0.1)
