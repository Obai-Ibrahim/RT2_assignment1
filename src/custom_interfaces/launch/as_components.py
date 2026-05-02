import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # 1. Path to the robot spawn launch file
    spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('bme_gazebo_sensors'),
                'launch',
                'spawn_robot_ex.launch.py'
            )
        )
    )

    # 2. The Container for your Navigation Components
    # Using 'mt' (Multi-Threaded) is critical because of your std::cin menu
    nav_container = ComposableNodeContainer(
        name='nav_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            
            ComposableNode(
                package='assignment_1',
                plugin='nav_system::Server',
                name='nav_server',
             ),
            ComposableNode(
                package='assignment_1',
                plugin='nav_system::NavClient',
                name='nav_client'
            )
        ],
        prefix=['xterm -e'], # Opens the UI menu in a separate window
        output='screen',
    )

    return LaunchDescription([
        spawn_robot_launch,
        nav_container
    ])