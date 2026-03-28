"""Launch file for the 17mm projectile simulation.

This script launches the ballistic simulation node and provides a static transform
to represent a mock launcher for testing purposes.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generates the launch description for the simulation system.

    Returns:
        LaunchDescription: The description containing the nodes to be launched.
    """
    config_file = os.path.join(
        get_package_share_directory('launch_sim'),
        'config',
        'rm17mm.yaml'
    )

    return LaunchDescription([
        Node(
            package='auto_aim_solver',
            executable='ballistic_solver',
            name='ballistic_solver',
            output='screen',
            parameters=[config_file]
        ),

        Node(
            package='launch_sim',
            executable='launch_sim',
            name='launch_sim_node',
            output='screen',
            parameters=[config_file]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='launcher_tf',
            arguments=['0', '0', '2.0', '0', '-0.785', '0', 'odom', 'launcher_link']
        )
    ])
