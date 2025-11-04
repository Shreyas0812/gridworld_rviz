from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, LogInfo

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    data_file_arg = DeclareLaunchArgument(
        'data_file',
        default_value='test_generated_trajectories',
        description='Name of the trajectory data CSV file'
    )

    trajectory_executor_params = PathJoinSubstitution([
        FindPackageShare("trajectory_executor"),
        "data",
        [LaunchConfiguration("data_file"), ".csv"]
    ])

    return LaunchDescription([
        
        data_file_arg,

        LogInfo(msg=['Using trajectory data file: ', trajectory_executor_params]),

        Node(
            package='trajectory_executor',
            executable='trajectory_executor',
            name='trajectory_executor',
            output='screen',
            parameters=[{
                'data_file': trajectory_executor_params
            }]
            # arguments=['--ros-args', '--log-level', 'debug']
        ),
    ])