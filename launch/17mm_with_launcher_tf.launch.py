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
    # Resolve the path to the configuration file
    config_file = os.path.join(
        get_package_share_directory('launch_sim'),
        'config',
        'ball_params.yaml'
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

        # --- Node 2: Static Transform Publisher (Test Only) ---
        # Publishes a static TF for 'launcher_link' relative to 'map'.
        # This simulates a robot's shooter position without needing a full URDF.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='launcher_tf',
            # Arguments format: x y z yaw pitch roll frame_id child_frame_id
            # Settings:
            #   z = 2.0m: Height of the launcher.
            #   pitch = -0.785 rad (-45 deg): Angled upwards for a parabolic trajectory.
            arguments=['0', '0', '2.0', '0', '-0.785', '0', 'map', 'launcher_link']
        )
    ])