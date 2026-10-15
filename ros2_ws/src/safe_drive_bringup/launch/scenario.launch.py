"""Launch the ego vehicle and one configured sudden-pedestrian scenario."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from safe_drive_scenario.sudden_pedestrian import load_sudden_pedestrian_config


def _launch_setup(context):
    config_path = Path(LaunchConfiguration('scenario_config').perform(context))
    ego_speed = load_sudden_pedestrian_config(config_path).ego_speed

    vehicle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('safe_drive_bringup'),
            'launch',
            'vehicle.launch.py',
        ])),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'target_speed': str(ego_speed),
        }.items(),
    )
    scenario_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('safe_drive_scenario'),
            'launch',
            'sudden_pedestrian.launch.py',
        ])),
        launch_arguments={
            'scenario_config': str(config_path),
        }.items(),
    )
    safety_monitor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('safe_drive_safety'),
            'launch',
            'safety_monitor.launch.py',
        ])),
    )
    return [vehicle, scenario_manager, safety_monitor]


def generate_launch_description() -> LaunchDescription:
    default_config = str(
        Path(get_package_share_directory('safe_drive_scenario'))
        / 'config'
        / 'sudden_pedestrian.yaml'
    )
    gui_argument = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start the Gazebo client when true.',
    )
    config_argument = DeclareLaunchArgument(
        'scenario_config',
        default_value=default_config,
        description='Absolute path to a sudden-pedestrian YAML definition.',
    )
    return LaunchDescription([
        gui_argument,
        config_argument,
        OpaqueFunction(function=_launch_setup),
    ])
