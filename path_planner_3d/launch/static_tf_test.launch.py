#!/usr/bin/env python3
"""
Launch file to publish static TF frames for testing without Isaac Sim.
This creates a dummy robot at the origin for testing the path planner.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Publish world frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'UW_camera_world']
        ),

        # Publish robot/camera frame at origin (for testing)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_robot',
            arguments=['0', '0', '0', '0', '0', '0', 'UW_camera_world', 'UW_camera']
        ),
    ])
