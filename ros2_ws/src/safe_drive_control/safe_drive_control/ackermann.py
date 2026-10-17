"""Convert a desired yaw rate to Gazebo Classic Ackermann tyre angle."""

import math


def steering_angle(
    linear_speed: float, yaw_rate: float, wheelbase: float, max_steer: float
) -> float:
    """Return the requested centre tyre angle in radians.

    Gazebo's gazebo_ros_ackermann_drive interprets Twist.angular.z as this
    angle, not as angular velocity. A stopped car cannot rotate in place.
    """
    if wheelbase <= 0.0 or max_steer <= 0.0:
        raise ValueError('wheelbase and max_steer must be positive')
    if abs(linear_speed) < 1e-6:
        return 0.0
    angle = math.atan(wheelbase * yaw_rate / linear_speed)
    return max(-max_steer, min(max_steer, angle))


def low_speed_command(desired: float, measured: float, gain: float,
                      feedback_enabled: bool) -> float:
    """Compensate motor lag near stops without changing cruise commands."""
    if gain < 0.0:
        raise ValueError('speed feedback gain must be nonnegative')
    if desired == 0.0:
        return max(-0.5, min(0.5, -gain * measured))
    if not feedback_enabled or desired >= 0.8:
        return desired
    return max(0.0, min(1.2, desired + gain * (desired - measured)))
