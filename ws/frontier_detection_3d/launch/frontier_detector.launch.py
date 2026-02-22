from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('frontier_detection_3d')
    config_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Path to config file'
        ),

        Node(
            package='frontier_detection_3d',
            executable='frontier_detector_node',
            name='frontier_detector',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
        ),
    ])