import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('filter')
    param_file = os.path.join(pkg_share, 'config', 'filter_params.yaml')

    return LaunchDescription([
        Node(
            package='filter',
            executable='pointcloud_filter_node',
            name='pointcloud_filter_node',
            output='screen',
            parameters=[param_file]
        )
    ]) 