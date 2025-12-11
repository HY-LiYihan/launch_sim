import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class Ball:
    """
    Represents a single projectile in the simulation.

    Attributes:
        id (int): Unique identifier for the ball.
        pos (np.array): Current position [x, y, z] in meters.
        vel (np.array): Current velocity [vx, vy, vz] in m/s.
        creation_time (int): Nanosecond timestamp when the ball was created.
        trail_data (list): List of tuples (timestamp_ns, geometry_msgs/Point).
        is_active (bool): Flag indicating if the ball is still flying.
    """
    def __init__(self, ball_id, pos, vel, creation_time_ns):
        self.id = ball_id
        self.pos = pos.copy()
        self.vel = vel.copy()
        self.creation_time = creation_time_ns
        self.trail_data = []
        self.is_active = True


class LaunchSim(Node):
    """
    A ROS 2 Node that simulates the kinematics and dynamics of 17mm projectiles.

    It handles:
    1. Physics simulation (Gravity + Air Drag).
    2. Visualization of the projectile and its trail in RViz.
    3. Real-time prediction of the ballistic trajectory based on TF.
    """

    def __init__(self):
        super().__init__('launch_sim_node')

        # --- Parameters Configuration ---
        # Physical properties of a standard 17mm projectile
        self.declare_parameter('mass', 0.0032)          # Mass in kg (approx 3.2g)
        self.declare_parameter('radius', 0.0085)        # Radius in meters (8.5mm)
        self.declare_parameter('drag_coeff', 0.47)      # Drag coefficient for a sphere
        self.declare_parameter('air_density', 1.225)    # Air density (kg/m^3) at sea level
        
        # Launch settings
        self.declare_parameter('initial_speed', 28.0)   # Muzzle velocity (m/s)
        self.declare_parameter('shoot_interval', 0.15)  # Time between automatic shots (s)
        
        # Visualization settings
        self.declare_parameter('trail_lifetime', 0.4)   # How long trail points persist (s)
        self.declare_parameter('launch_frame', 'launcher_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('update_frequency', 100.0)

        self.update_params()

        # --- TF & Transform Listener ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Publishers ---
        # Use MarkerArray for efficient batch rendering of multiple objects
        self.pub_ball_array = self.create_publisher(MarkerArray, 'visual/balls', 10)
        self.pub_trail_array = self.create_publisher(MarkerArray, 'visual/trails', 10)
        self.pub_pred = self.create_publisher(Marker, 'visual/prediction', 10)

        # --- State Management ---
        self.active_balls = []
        self.ball_id_counter = 0

        # --- Timers ---
        # 1. Physics Loop (High Frequency): Handles movement and integration
        self.sim_timer = self.create_timer(self.dt, self.physics_loop)
        
        # 2. Shoot Timer: Automatically spawns new balls
        self.shoot_timer = self.create_timer(self.shoot_interval, self.trigger_shoot)
        
        # 3. Prediction Timer (Low Frequency): Updates the trajectory line
        self.pred_timer = self.create_timer(1.0 / 30.0, self.prediction_loop)

        self.get_logger().info('LaunchSim initialized. Real-time prediction active.')

    def update_params(self):
        """Updates internal variables from ROS parameters."""
        self.mass = self.get_parameter('mass').value
        self.radius = self.get_parameter('radius').value
        self.drag_coeff = self.get_parameter('drag_coeff').value
        self.rho = self.get_parameter('air_density').value
        self.v0 = self.get_parameter('initial_speed').value
        self.shoot_interval = self.get_parameter('shoot_interval').value
        self.trail_lifetime = self.get_parameter('trail_lifetime').value
        self.launch_frame = self.get_parameter('launch_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.freq = self.get_parameter('update_frequency').value

        self.dt = 1.0 / self.freq
        self.area = math.pi * (self.radius ** 2)

    def get_launch_state(self):
        """
        Calculates the current position and velocity vector of the muzzle.

        Returns:
            tuple: (position_np_array, velocity_np_array) or (None, None) if TF fails.
        """
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.launch_frame,
                rclpy.time.Time())

            pos = np.array([
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z
            ])

            # Convert quaternion to rotation matrix (only need the X-axis column)
            q = [
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w
            ]
            x, y, z, w = q
            
            # Formula for the first column of the rotation matrix (Local X -> World)
            dir_x = 1 - 2 * y * y - 2 * z * z
            dir_y = 2 * x * y + 2 * z * w
            dir_z = 2 * x * z - 2 * y * w
            
            direction = np.array([dir_x, dir_y, dir_z])
            direction = direction / np.linalg.norm(direction)

            vel = direction * self.v0
            return pos, vel

        except TransformException:
            return None, None

    def trigger_shoot(self):
        """Spawns a new Ball instance based on current TF."""
        start_pos, start_vel = self.get_launch_state()
        if start_pos is None:
            return

        new_ball = Ball(
            ball_id=self.ball_id_counter,
            pos=start_pos,
            vel=start_vel,
            creation_time_ns=self.get_clock().now().nanoseconds
        )
        self.active_balls.append(new_ball)
        self.ball_id_counter += 1

    def prediction_loop(self):
        """Timer callback to update the predicted trajectory line."""
        start_pos, start_vel = self.get_launch_state()
        if start_pos is None:
            return
        
        self.publish_prediction_line(start_pos, start_vel)

    def calculate_acceleration(self, velocity):
        """
        Calculates the net acceleration acting on the ball.
        
        Forces considered:
        1. Gravity (constant down).
        2. Air Drag (proportional to v^2, opposite to velocity).
        """
        v_mag = np.linalg.norm(velocity)
        
        # Calculate Drag Force: Fd = 0.5 * rho * v^2 * Cd * A
        if v_mag > 0.001:
            drag_force = 0.5 * self.rho * (v_mag ** 2) * self.drag_coeff * self.area
            drag_vec = - (velocity / v_mag) * drag_force
        else:
            drag_vec = np.zeros(3)
            
        gravity_vec = np.array([0.0, 0.0, -self.mass * 9.81])
        
        # Newton's Second Law: a = F_net / m
        return (drag_vec + gravity_vec) / self.mass

    def physics_loop(self):
        """Main simulation loop handling physics integration and marker generation."""
        if not self.active_balls:
            return

        current_time_ns = self.get_clock().now().nanoseconds
        lifetime_ns = self.trail_lifetime * 1e9
        
        ball_markers = MarkerArray()
        trail_markers = MarkerArray()
        balls_to_remove = []

        for ball in self.active_balls:
            # 1. Physics Update (Euler Integration)
            acc = self.calculate_acceleration(ball.vel)
            ball.vel += acc * self.dt
            ball.pos += ball.vel * self.dt

            # 2. Ground Collision Check
            if ball.pos[2] <= 0:
                # Mark for removal. We rely on RViz marker auto-expiry to clear visuals.
                balls_to_remove.append(ball)
                continue

            # 3. Trail Update
            p_msg = Point(x=ball.pos[0], y=ball.pos[1], z=ball.pos[2])
            ball.trail_data.append((current_time_ns, p_msg))
            
            # Prune old trail points
            ball.trail_data = [
                (t, p) for t, p in ball.trail_data 
                if (current_time_ns - t) < lifetime_ns
            ]

            # 4. Generate Markers
            self.append_ball_marker(ball_markers, ball)
            self.append_trail_marker(trail_markers, ball)

        # Clean up dead balls from memory
        for b in balls_to_remove:
            self.active_balls.remove(b)

        # Publish batches
        if ball_markers.markers:
            self.pub_ball_array.publish(ball_markers)
        if trail_markers.markers:
            self.pub_trail_array.publish(trail_markers)

    def append_ball_marker(self, marker_array, ball):
        """Creates a sphere marker for the projectile (Cyan)."""
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "balls"
        marker.id = ball.id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = ball.pos[0]
        marker.pose.position.y = ball.pos[1]
        marker.pose.position.z = ball.pos[2]
        marker.pose.orientation.w = 1.0 
        
        marker.scale.x = self.radius * 2
        marker.scale.y = self.radius * 2
        marker.scale.z = self.radius * 2
        
        # Auto-expire if not updated (solves the "stuck on ground" issue)
        marker.lifetime = Duration(seconds=0.05).to_msg()
        
        # Color: Cyan (R=0, G=1, B=1)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        marker_array.markers.append(marker)

    def append_trail_marker(self, marker_array, ball):
        """Creates a line strip marker for the trail (Cyan)."""
        if len(ball.trail_data) < 2:
            return
            
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "trails"
        marker.id = ball.id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        marker.scale.x = 0.015  # Line width
        marker.lifetime = Duration(seconds=0.05).to_msg() # Auto-expire
        
        # Color: Cyan with transparency
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.5 

        marker.points = [p for t, p in ball.trail_data]
        marker_array.markers.append(marker)

    def publish_prediction_line(self, start_pos, start_vel):
        """
        Simulates and visualizes the predicted trajectory.
        Uses the exact same physics logic and timestep as the main loop
        to ensure visual consistency.
        """
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "prediction"
        marker.id = 9999 
        marker.type = Marker.LINE_LIST 
        marker.action = Marker.ADD
        
        # Short lifetime to allow real-time updates without ghosting
        marker.lifetime = Duration(seconds=0.05).to_msg() 
        
        marker.scale.x = 0.02
        
        # Color: Light Grey
        marker.color.r = 0.7
        marker.color.g = 0.7
        marker.color.b = 0.7
        marker.color.a = 0.5

        temp_pos = start_pos.copy()
        temp_vel = start_vel.copy()
        
        # MUST use the same dt as physics_loop for accuracy
        sim_dt = self.dt 
        step_count = 0
        
        # Fast-forward simulation for prediction
        while temp_pos[2] > 0:
            p1 = Point(x=temp_pos[0], y=temp_pos[1], z=temp_pos[2])
            
            acc = self.calculate_acceleration(temp_vel)
            temp_vel += acc * sim_dt
            temp_pos += temp_vel * sim_dt
            
            p2 = Point(x=temp_pos[0], y=temp_pos[1], z=temp_pos[2])
            
            # Draw dashed line (skip some segments)
            step_count += 1
            if step_count % 10 < 5: 
                marker.points.append(p1)
                marker.points.append(p2)
            
            # Safety break
            if temp_pos[2] < -5.0 or len(marker.points) > 1000:
                break
        
        self.pub_pred.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = LaunchSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()