"""Launch the complete MVP-01 simulation and waypoint controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    gui_argument = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start the Gazebo client when true.',
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('safe_drive_sim'),
            'launch',
            'simulation.launch.py',
        ])),
        launch_arguments={'gui': LaunchConfiguration('gui')}.items(),
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('safe_drive_control'),
            'launch',
            'waypoint_follower.launch.py',
        ])),
    )

    return LaunchDescription([gui_argument, simulation, controller])
