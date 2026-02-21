from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, LogInfo

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    # Launch argument: trajectory CSV file name (without .csv extension)
    data_file_arg = DeclareLaunchArgument(
        'data_file',
        default_value='trajectories',
        description='Name of trajectory data CSV file (without .csv extension)'
    )

    # Launch argument: playback rate in Hz
    rate_hz_arg = DeclareLaunchArgument(
        'rate_hz',
        default_value='1.0',
        description='Playback rate in Hz'
    )

    # Build full path to the CSV file inside the installed data/ directory
    data_file_path = PathJoinSubstitution([
        FindPackageShare('trajectory_executor'),
        'data',
        [LaunchConfiguration('data_file'), '.csv']
    ])

    # Single trajectory executor node (handles all agents in one CSV)
    trajectory_executor_node = Node(
        package='trajectory_executor',
        executable='trajectory_executor',
        name='trajectory_executor',
        output='screen',
        parameters=[{
            'data_file': data_file_path,
            'rate_hz': LaunchConfiguration('rate_hz'),
        }]
    )

    return LaunchDescription([
        data_file_arg,
        rate_hz_arg,
        LogInfo(msg=['Launching trajectory_executor with file: ', data_file_path]),
        trajectory_executor_node,
    ])