from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('path_planner_3d')
    planner_params = os.path.join(pkg_dir, 'config', 'planner_params.yaml')

    return LaunchDescription([
        Node(
            package='path_planner_3d',
            executable='path_planner_node',
            name='path_planner',
            output='screen',
            parameters=[planner_params],
            remappings=[
                ('/exploration_goal', '/exploration_goal'),
                ('/octomap_binary', '/octomap_binary'),
                ('/planned_path', '/planned_path'),
            ]
        ),
    ])
