"""Launch the minimal SAFE-Drive Gazebo world and spawn the ego vehicle."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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

    vehicle_xacro = PathJoinSubstitution([
        FindPackageShare('safe_drive_description'),
        'urdf',
        'ego_vehicle.urdf.xacro',
    ])
    robot_description = Command(['xacro ', vehicle_xacro])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'
        ])),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
        }.items(),
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

    return LaunchDescription([
        world_argument,
        gui_argument,
        spawn_x_argument,
        spawn_y_argument,
        spawn_yaw_argument,
        gazebo,
        robot_state_publisher,
        spawn_ego,
    ])
