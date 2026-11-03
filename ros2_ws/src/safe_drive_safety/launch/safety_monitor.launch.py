"""Launch ground-truth safety evaluation for one scenario."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config_argument = DeclareLaunchArgument(
        'safety_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('safe_drive_safety'), 'config', 'safety.yaml',
        ]),
    )
    output_argument = DeclareLaunchArgument(
        'output_file', default_value='',
        description='Optional JSON result path after the scenario ends.',
    )
    monitor = Node(
        package='safe_drive_safety',
        executable='safety_monitor',
        name='safety_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'safety_config': LaunchConfiguration('safety_config'),
            'output_file': LaunchConfiguration('output_file'),
        }],
    )
    return LaunchDescription([config_argument, output_argument, monitor])
