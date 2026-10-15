"""Launch the compact intersection, traffic signal, and straight ego route."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    world = PathJoinSubstitution([
        FindPackageShare('safe_drive_sim'), 'worlds', 'signal_intersection.world'
    ])
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('safe_drive_sim'), 'launch', 'simulation.launch.py'
        ])),
        launch_arguments={
            'world': world,
            'gui': LaunchConfiguration('gui'),
        }.items(),
    )
    controller = Node(
        package='safe_drive_control',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('safe_drive_control'), 'config', 'controller.yaml'
            ]),
            PathJoinSubstitution([
                FindPackageShare('safe_drive_control'), 'config',
                'signal_controller.yaml'
            ]),
        ],
    )
    signal = Node(
        package='safe_drive_scenario',
        executable='traffic_signal',
        name='traffic_signal',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('safe_drive_scenario'), 'config', 'traffic_signal.yaml'
            ]),
            {'fixed_state': ParameterValue(
                LaunchConfiguration('fixed_state'), value_type=str
            )},
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument(
            'fixed_state', default_value='',
            description='RED or GREEN for a fixed validation run; empty cycles.',
        ),
        simulation,
        signal,
        controller,
    ])
