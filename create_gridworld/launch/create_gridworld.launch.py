from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, LogInfo

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    rviz_config = PathJoinSubstitution([
        FindPackageShare("create_gridworld"),
        "rviz", 
        "gridworld.rviz"
    ])

    config_type_arg = DeclareLaunchArgument(
        "config_type",
        default_value="small_3D",
        description="Configuration type: small_2D, small_3D, large_2D, large_3D"
    )

    gridworld_params = PathJoinSubstitution([
            FindPackageShare("create_gridworld"),
            "config",
            ["gridworld_", LaunchConfiguration("config_type"), ".yaml"]
        ])

    return LaunchDescription([
        
        LogInfo(msg=['Loading parameters from: ', gridworld_params]),

        config_type_arg,

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