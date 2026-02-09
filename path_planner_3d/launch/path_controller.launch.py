from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('path_planner_3d')
    controller_params = os.path.join(pkg_dir, 'config', 'controller_params.yaml')

    return LaunchDescription([
        Node(
            package='path_planner_3d',
            executable='path_controller_node',
            name='path_controller',
            output='screen',
            parameters=[controller_params],
            remappings=[
                ('/planned_path', '/planned_path'),
                ('/cmd_vel', '/cmd_vel'),
            ]
        ),
    ])
