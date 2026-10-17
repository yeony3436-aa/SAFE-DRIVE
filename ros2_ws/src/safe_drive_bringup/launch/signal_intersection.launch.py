"""Launch the compact intersection, traffic signal, and selected ego route."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    route_file = PathJoinSubstitution([
        FindPackageShare('safe_drive_control'), 'config', LaunchConfiguration('route_config')
    ])
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
            route_file,
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
    route_monitor = Node(
        package='safe_drive_safety',
        executable='route_monitor',
        name='route_monitor',
        output='screen',
        parameters=[{
            'route_file': route_file,
            'output_file': LaunchConfiguration('result_file'),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument(
            'route_config', default_value='controller.yaml',
            description=(
                'Controller route file: controller.yaml, right_turn.yaml, '
                'or left_turn.yaml.'
            ),
        ),
        DeclareLaunchArgument(
            'result_file', default_value='/tmp/safe_drive_route_result.json',
            description='JSON result path for route completion and corridor measurement.',
        ),
        DeclareLaunchArgument(
            'fixed_state', default_value='',
            description='RED or GREEN for a fixed validation run; empty cycles.',
        ),
        simulation,
        signal,
        controller,
        route_monitor,
    ])
