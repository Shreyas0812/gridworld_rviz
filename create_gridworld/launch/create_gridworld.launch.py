from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, LogInfo

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    rviz_config = PathJoinSubstitution([
        FindPackageShare("create_gridworld"),
        "rviz", 
        "gridworld.rviz"
    ])

    gridworld_params = PathJoinSubstitution([
            FindPackageShare("create_gridworld"),
            "config",
            "gridworld_small_3D.yaml"
        ])

    return LaunchDescription([

        LogInfo(msg=['Loading parameters from: ', gridworld_params]),

        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        SetEnvironmentVariable('QT_QPA_PLATFORM_PLUGIN_PATH', ''),

        Node(
            package="create_gridworld",
            executable="create_gridworld_node",
            name="create_gridworld_node",
            output="screen",
            parameters=[gridworld_params],
            # arguments=['--ros-args', '--log-level', 'debug']
        ),
        # Static transform publisher for rviz
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "map", "base_link"]
        ),
        # RViz2 with config file
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config]
        ),
    ])