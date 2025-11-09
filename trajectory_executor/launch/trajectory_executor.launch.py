from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, LogInfo

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    # Configure your CSV files here - add or remove as needed
    csv_files = [
        'agent_1_trajectory',
        'agent_2_trajectory',
        'agent_3_trajectory',
        'agent_4_trajectory',
        'agent_5_trajectory',
    ]

    launch_description_list = []

    # Create launch arguments and nodes for each CSV file
    for idx, csv_file in enumerate(csv_files):
        # Create a unique launch argument for each CSV file
        data_file_arg = DeclareLaunchArgument(
            f'data_file{idx + 1}',
            default_value=csv_file,
            description=f'Name of trajectory data CSV file {idx + 1}'
        )
        
        # Create the path to the CSV file
        trajectory_executor_params = PathJoinSubstitution([
            FindPackageShare("trajectory_executor"),
            "data",
            [LaunchConfiguration(f"data_file{idx + 1}"), ".csv"]
        ])
        
        # Create a unique node name
        node_name = f'trajectory_executor{idx + 1}' if idx > 0 else 'trajectory_executor'
        
        # Add the launch argument to the list
        launch_description_list.append(data_file_arg)
        
        # Add log info
        launch_description_list.append(
            LogInfo(msg=[f'Launching {node_name} with file: ', trajectory_executor_params])
        )
        
        # Create and add the node
        launch_description_list.append(
            Node(
                package='trajectory_executor',
                executable='trajectory_executor',
                name=node_name,
                output='screen',
                parameters=[{
                    'data_file': trajectory_executor_params
                }]
                # arguments=['--ros-args', '--log-level', 'debug']
            )
        )

    return LaunchDescription(launch_description_list)