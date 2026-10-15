"""Launch the SAFE-Drive waypoint follower with repository configuration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    target_speed_argument = DeclareLaunchArgument(
        'target_speed',
        default_value='2.0',
        description='Ego target speed in meters per second.',
    )
    controller_config = PathJoinSubstitution([
        FindPackageShare('safe_drive_control'),
        'config',
        'controller.yaml',
    ])

    waypoint_follower = Node(
        package='safe_drive_control',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[
            controller_config,
            {
                'target_speed': ParameterValue(
                    LaunchConfiguration('target_speed'), value_type=float
                )
            },
        ],
    )

    return LaunchDescription([target_speed_argument, waypoint_follower])
