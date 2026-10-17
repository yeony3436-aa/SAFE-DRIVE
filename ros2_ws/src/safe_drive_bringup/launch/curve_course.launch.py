"""Validate a gentle mapped bend with the visual Prius Ackermann vehicle."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    route_file = PathJoinSubstitution([
        FindPackageShare('safe_drive_control'), 'config', 'curve.yaml'
    ])
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('safe_drive_sim'), 'launch', 'simulation.launch.py'
        ])),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('safe_drive_sim'), 'worlds', 'curve_course.world'
            ]),
            'gui': LaunchConfiguration('gui'),
            'spawn_x': '-25.0',
            'spawn_y': '-1.75',
        }.items(),
    )
    controller = Node(
        package='safe_drive_control', executable='waypoint_follower',
        name='waypoint_follower', output='screen', parameters=[route_file],
    )
    monitor = Node(
        package='safe_drive_safety', executable='route_monitor',
        name='route_monitor', output='screen',
        parameters=[{'route_file': route_file,
                     'output_file': LaunchConfiguration('result_file')}],
    )
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('result_file', default_value='/tmp/safe_drive_curve_result.json'),
        simulation, controller, monitor,
    ])
