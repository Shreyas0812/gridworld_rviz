from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    rviz_config = PathJoinSubstitution([
        FindPackageShare("create_gridworld"),
        "rviz", 
        "gridworld.rviz"
    ])

    return LaunchDescription([
        # Node(
        #     package="create_gridworld",
        #     executable="create_gridworld_node.py",
        #     name="create_gridworld_node",
        #     output="screen",
        #     parameters=[{
        #         "grid_width": 50,
        #         "grid_height": 50,
        #         "resolution": 0.1
        #     }]
        # ),

        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        SetEnvironmentVariable('QT_QPA_PLATFORM_PLUGIN_PATH', ''),


        Node(
            package="create_gridworld",
            executable="create_gridworld_node",
            name="create_gridworld_node",
            output="screen",
            parameters=[{
                "grid_width": 50,
                "grid_height": 50,
                "resolution": 0.1,
                "publish_rate": 10.0
            }]
        ),
        # Static transform publisher for rviz
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "map", "base_link"]
        ),
        # RViz2
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
        ),
    ])