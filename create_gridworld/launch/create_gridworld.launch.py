from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="create_gridworld",
            executable="create_gridworld_node.py",
            name="create_gridworld_node",
            output="screen",

        ),
        Node(
            package="create_gridworld",
            executable="create_gridworld_node",
            name="create_gridworld_node",
            output="screen",
        )
    ])