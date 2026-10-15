"""Launch the configured sudden-pedestrian scenario manager."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config_argument = DeclareLaunchArgument(
        'scenario_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('safe_drive_scenario'),
            'config',
            'sudden_pedestrian.yaml',
        ]),
        description='Structured sudden-pedestrian YAML definition.',
    )
    pedestrian_model = PathJoinSubstitution([
        FindPackageShare('safe_drive_scenario'),
        'models',
        'pedestrian',
        'model.sdf',
    ])
    scenario_manager = Node(
        package='safe_drive_scenario',
        executable='scenario_manager',
        name='scenario_manager',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'scenario_config': LaunchConfiguration('scenario_config'),
            'pedestrian_model': pedestrian_model,
        }],
    )
    return LaunchDescription([config_argument, scenario_manager])
