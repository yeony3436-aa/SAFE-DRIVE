"""Launch an outer-grid right turn that yields to a crossing pedestrian."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    route_file = PathJoinSubstitution([
        FindPackageShare('safe_drive_control'), 'config',
        'connected_unsignalized_right_turn.yaml',
    ])
    scenario_file = PathJoinSubstitution([
        FindPackageShare('safe_drive_scenario'), 'config',
        'unsignalized_crossing.yaml',
    ])
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('safe_drive_sim'), 'launch', 'simulation.launch.py',
        ])),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('safe_drive_sim'), 'worlds', 'connected_road.world',
            ]),
            'gui': LaunchConfiguration('gui'),
            'spawn_x': '10.0',
        }.items(),
    )
    crossing = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('safe_drive_scenario'), 'launch',
            'sudden_pedestrian.launch.py',
        ])),
        launch_arguments={'scenario_config': scenario_file}.items(),
    )
    safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('safe_drive_safety'), 'launch',
            'safety_monitor.launch.py',
        ])),
        launch_arguments={
            'output_file': LaunchConfiguration('safety_result_file'),
        }.items(),
    )
    controller = Node(
        package='safe_drive_control', executable='waypoint_follower',
        name='waypoint_follower', output='screen', parameters=[route_file],
    )
    yield_decision = Node(
        package='safe_drive_scenario', executable='unsignalized_yield',
        name='unsignalized_yield', output='screen',
    )
    yield_monitor = Node(
        package='safe_drive_safety', executable='unsignalized_yield_monitor',
        name='unsignalized_yield_monitor', output='screen',
        parameters=[{'use_sim_time': True,
                     'output_file': LaunchConfiguration('yield_result_file')}],
    )
    route_monitor = Node(
        package='safe_drive_safety', executable='route_monitor',
        name='route_monitor', output='screen',
        parameters=[{'use_sim_time': True, 'route_file': route_file,
                     'output_file': LaunchConfiguration('route_result_file')}],
    )
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('route_result_file',
                              default_value='/tmp/safe_drive_unsignalized_route.json'),
        DeclareLaunchArgument('yield_result_file',
                              default_value='/tmp/safe_drive_unsignalized_yield.json'),
        DeclareLaunchArgument('safety_result_file',
                              default_value='/tmp/safe_drive_unsignalized_safety.json'),
        simulation, crossing, safety, controller,
        yield_decision, yield_monitor, route_monitor,
    ])
