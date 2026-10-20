"""Launch the minimal SAFE-Drive Gazebo world and spawn the ego vehicle."""

import socket
from urllib.parse import urlsplit

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _check_gazebo_master(context, *args, **kwargs):
    """Fail before launching if a previous Gazebo server owns this address."""
    uri = context.environment.get('GAZEBO_MASTER_URI', 'http://127.0.0.1:11345')
    parsed = urlsplit(uri)
    if not parsed.hostname:
        raise RuntimeError(f'Invalid GAZEBO_MASTER_URI: {uri}')
    address = (parsed.hostname, parsed.port or 11345)
    try:
        with socket.create_connection(address, timeout=0.3):
            pass
    except OSError:
        return []
    raise RuntimeError(
        f'Gazebo master at {uri} is already running. Close the previous '
        'Gazebo launch before starting a new SAFE-Drive simulation.'
    )


def generate_launch_description() -> LaunchDescription:
    world_argument = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('safe_drive_sim'),
            'worlds',
            'minimal_road.world',
        ]),
        description='Absolute path to the Gazebo world file.',
    )
    gui_argument = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start the Gazebo client when true.',
    )
    spawn_x_argument = DeclareLaunchArgument(
        'spawn_x', default_value='-15.0', description='Ego spawn x position in meters.'
    )
    spawn_y_argument = DeclareLaunchArgument(
        'spawn_y', default_value='-1.75', description='Ego spawn y position in meters.'
    )
    spawn_yaw_argument = DeclareLaunchArgument(
        'spawn_yaw', default_value='0.0', description='Ego spawn yaw in radians.'
    )
    drive_mode_argument = DeclareLaunchArgument(
        'drive_mode', default_value='ackermann',
        description='Vehicle dynamics: ackermann or skid_steer.',
    )

    vehicle_xacro = PathJoinSubstitution([
        FindPackageShare('safe_drive_sim'),
        'urdf',
        'ego_vehicle.gazebo.urdf.xacro',
    ])
    robot_description = Command([
        'xacro ', vehicle_xacro, ' drive_mode:=', LaunchConfiguration('drive_mode')
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('gazebo_ros'), 'launch', 'gzserver.launch.py'
        ])),
        launch_arguments={
            'world': LaunchConfiguration('world'),
        }.items(),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('gazebo_ros'), 'launch', 'gzclient.launch.py'
        ])),
        launch_arguments={'gui_required': 'true'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    spawn_ego = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'safe_drive_ego',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', '0.0',
            '-Y', LaunchConfiguration('spawn_yaw'),
        ],
        output='screen',
    )

    def start_gui_after_spawn(event, context):
        if event.returncode != 0:
            return [
                LogInfo(msg='Ego spawn failed; Gazebo GUI was not started.'),
                EmitEvent(event=Shutdown(reason='Ego spawn failed')),
            ]
        # The world camera can only track safe_drive_ego if the model already
        # exists when gzclient first loads the scene.
        return [gazebo_client] if IfCondition(LaunchConfiguration('gui')).evaluate(context) else []

    return LaunchDescription([
        world_argument,
        gui_argument,
        spawn_x_argument,
        spawn_y_argument,
        spawn_yaw_argument,
        drive_mode_argument,
        # gazebo_ros adds package exports to these values. Preserve Gazebo's
        # built-in ground-plane and shader paths when the shell has not sourced
        # /usr/share/gazebo-11/setup.sh.
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            ['/usr/share/gazebo-11/models:',
             EnvironmentVariable('GAZEBO_MODEL_PATH', default_value='')],
        ),
        SetEnvironmentVariable(
            'GAZEBO_RESOURCE_PATH',
            ['/usr/share/gazebo-11:',
             EnvironmentVariable('GAZEBO_RESOURCE_PATH', default_value='')],
        ),
        OpaqueFunction(function=_check_gazebo_master),
        gazebo,
        robot_state_publisher,
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_ego, on_exit=start_gui_after_spawn
        )),
        spawn_ego,
    ])
