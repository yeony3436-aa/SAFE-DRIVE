"""Launch the SAFE-Drive waypoint follower with repository configuration."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
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
        parameters=[controller_config],
    )

    return LaunchDescription([waypoint_follower])
