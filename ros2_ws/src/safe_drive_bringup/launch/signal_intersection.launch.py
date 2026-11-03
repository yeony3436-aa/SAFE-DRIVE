"""Launch the compact intersection, traffic signal, and selected ego route."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    route_file = PathJoinSubstitution([
        FindPackageShare('safe_drive_control'), 'config', LaunchConfiguration('route_config')
    ])
    world = PathJoinSubstitution([
        FindPackageShare('safe_drive_sim'), 'worlds', LaunchConfiguration('world_file')
    ])
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('safe_drive_sim'), 'launch', 'simulation.launch.py'
        ])),
        launch_arguments={
            'world': world,
            'gui': LaunchConfiguration('gui'),
            'spawn_x': LaunchConfiguration('spawn_x'),
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
            {'pedestrian_stop_enabled': ParameterValue(
                LaunchConfiguration('pedestrian_stop'), value_type=bool),
             'pedestrian_stop_margin': ParameterValue(
                LaunchConfiguration('pedestrian_stop_margin'), value_type=float)},
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
    lane_monitor = Node(
        package='safe_drive_safety',
        executable='connected_lane_monitor',
        name='connected_lane_monitor',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('world_file'),
            "' == 'connected_road.world'",
        ])),
        parameters=[{
            'markings_file': PathJoinSubstitution([
                FindPackageShare('safe_drive_sim'), 'config',
                'connected_road_markings.json',
            ]),
            'output_file': LaunchConfiguration('lane_result_file'),
        }],
    )
    red_stop_monitor = Node(
        package='safe_drive_safety',
        executable='red_stop_monitor',
        name='red_stop_monitor',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('fixed_state'), "' == 'RED' or '",
            LaunchConfiguration('monitor_red_stop'), "' == 'true'",
        ])),
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('safe_drive_safety'), 'config',
                'red_stop_monitor.yaml'
            ]),
            {'output_file': LaunchConfiguration('red_result_file')},
        ],
    )
    crossing_condition = IfCondition(LaunchConfiguration('pedestrian'))
    crossing = Node(
        package='safe_drive_scenario',
        executable='signal_crossing',
        name='signal_crossing',
        output='screen',
        condition=crossing_condition,
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('safe_drive_scenario'), 'config',
                LaunchConfiguration('crossing_config')
            ]),
            {
                'pedestrian_model': PathJoinSubstitution([
                    FindPackageShare('safe_drive_scenario'), 'models',
                    'pedestrian', 'model.sdf'
                ]),
                'output_file': LaunchConfiguration('pedestrian_result_file'),
            },
        ],
    )
    crossing_safety = Node(
        package='safe_drive_safety',
        executable='safety_monitor',
        name='safety_monitor',
        output='screen',
        condition=IfCondition(LaunchConfiguration('pedestrian')),
        parameters=[{
            'use_sim_time': True,
            'safety_config': PathJoinSubstitution([
                FindPackageShare('safe_drive_safety'), 'config', 'safety.yaml'
            ]),
            'output_file': LaunchConfiguration('pedestrian_safety_file'),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('world_file', default_value='signal_intersection.world',
                              description='World file in safe_drive_sim/worlds.'),
        DeclareLaunchArgument('spawn_x', default_value='-210.0'),
        DeclareLaunchArgument('pedestrian', default_value='false'),
        DeclareLaunchArgument('pedestrian_stop', default_value='false'),
        DeclareLaunchArgument('pedestrian_stop_margin', default_value='0.6'),
        DeclareLaunchArgument('crossing_config',
                              default_value='signal_crossing.yaml'),
        DeclareLaunchArgument(
            'pedestrian_result_file',
            default_value='/tmp/safe_drive_signal_crossing.json',
        ),
        DeclareLaunchArgument(
            'pedestrian_safety_file',
            default_value='/tmp/safe_drive_signal_safety.json',
        ),
        DeclareLaunchArgument(
            'route_config', default_value='right_turn.yaml',
            description=(
                'Controller route file: fast_straight.yaml, right_turn.yaml, '
                'left_turn.yaml, or a connected-road route.'
            ),
        ),
        DeclareLaunchArgument(
            'result_file', default_value='/tmp/safe_drive_route_result.json',
            description='JSON result path for route completion and corridor measurement.',
        ),
        DeclareLaunchArgument(
            'lane_result_file', default_value='/tmp/safe_drive_lane_lines.json',
            description='JSON painted-line result for the connected road.',
        ),
        DeclareLaunchArgument(
            'red_result_file', default_value='/tmp/safe_drive_red_stop.json',
            description='JSON stop-line result path when fixed_state is RED.',
        ),
        DeclareLaunchArgument(
            'fixed_state', default_value='',
            description='RED or GREEN for a fixed validation run; empty cycles.',
        ),
        DeclareLaunchArgument(
            'monitor_red_stop', default_value='false',
            description='Measure an initial stop during a cycling signal.',
        ),
        simulation,
        signal,
        controller,
        route_monitor,
        lane_monitor,
        red_stop_monitor,
        crossing,
        crossing_safety,
    ])
