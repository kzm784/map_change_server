import os
import launch
import launch_ros

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = LaunchDescription()

    # Sync Template Server Node
    sync_template_server_node = Node(
        package='map_change_server',
        executable='map_change_server_node',
        name='map_change_server_node',
        namespace='waypoint_function',
        output='screen',
    )
    ld.add_action(sync_template_server_node)

    return ld