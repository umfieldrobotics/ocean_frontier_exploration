from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('octomap_3d_exploration')
    config_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Path to config file'
        ),

        Node(
            package='octomap_3d_exploration',
            executable='octomap_builder_node',
            name='octomap_builder',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
        ),
    ])