import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Path to the Gazebo Spawn Launch File
    gazebo_launch_dir = get_package_share_directory('bme_gazebo_sensors')
    gazebo_spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch_dir, 'launch', 'spawn_robot_ex.launch.py')
        )
    )

    # 2. Your Action Server (in its own window)
    server_node = Node(
        package='assignment_1',
        executable='server_node',
        name='nav_server',
        prefix=['xterm -e'],
        output='screen'
    )

    # 3. Your Action Client (in its own window for std::cin)
    client_node = Node(
        package='assignment_1',
        executable='client_node',
        name='nav_client',
        prefix=['xterm -hold -e'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_spawn_launch,
        server_node,
        client_node
    ])