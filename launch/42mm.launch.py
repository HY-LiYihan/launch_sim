"""Launch file for the 17mm projectile simulation.

This script launches the ballistic simulation node
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
    # Resolve the path to the configuration file
    config_file = os.path.join(
        get_package_share_directory('launch_sim'),
        'config',
        'rm42mm.yaml'
    )

    return LaunchDescription([
        # --- Node 1: Projectile Simulation Node ---
        # This node handles the physics calculation and visualization markers.
        Node(
            package='launch_sim',
            executable='launch_sim',
            name='launch_sim_node',
            output='screen',
            parameters=[config_file]
        ),

    ])