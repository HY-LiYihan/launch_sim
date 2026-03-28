import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PointStamped, Vector3Stamped
from auto_aim_interfaces.msg import Target
from venom_serial_driver.msg import GameStatus
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
        self.declare_parameter('map_frame', 'odom')
        self.declare_parameter('update_frequency', 100.0)
        self.declare_parameter('target_topic', '/tracker/target')
        self.declare_parameter('target_timeout', 0.2)
        self.declare_parameter('solver.min_pitch', -0.35)
        self.declare_parameter('solver.max_pitch', 0.8)
        self.declare_parameter('solver.pitch_samples', 36)
        self.declare_parameter('solver.max_iterations', 18)
        self.declare_parameter('solver.max_time', 5.0)
        self.declare_parameter('solver.ground_z', 0.0)
        self.declare_parameter('speed_topic', '/game_status')
        self.declare_parameter('use_live_speed', True)
        self.declare_parameter('speed_timeout', 0.5)
        self.declare_parameter('min_live_speed', 5.0)
        self.declare_parameter('use_external_solver', True)
        self.declare_parameter('solution_topic', '/auto_aim/gimbal_cmd')
        self.declare_parameter('aim_point_topic', '/auto_aim/aim_point')
        self.declare_parameter('solver_output_timeout', 0.2)

        self.update_params()

        # --- TF & Transform Listener ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Publishers ---
        # Use MarkerArray for efficient batch rendering of multiple objects
        self.pub_ball_array = self.create_publisher(MarkerArray, 'visual/balls', 10)
        self.pub_trail_array = self.create_publisher(MarkerArray, 'visual/trails', 10)
        self.pub_pred = self.create_publisher(Marker, 'visual/prediction', 10)
        self.pub_target = self.create_publisher(MarkerArray, 'visual/target', 10)
        self.pub_solution = self.create_publisher(Marker, 'visual/solution', 10)
        self.pub_diag = self.create_publisher(MarkerArray, 'visual/diagnostics', 10)

        # --- Subscribers ---
        self.target_sub = self.create_subscription(
            Target, self.target_topic, self.target_callback, 10)
        self.speed_sub = self.create_subscription(
            GameStatus, self.speed_topic, self.game_status_callback, 10)
        self.solution_sub = self.create_subscription(
            Vector3Stamped, self.solution_topic, self.solution_callback, 10)
        self.aim_point_sub = self.create_subscription(
            PointStamped, self.aim_point_topic, self.aim_point_callback, 10)

        # --- State Management ---
        self.active_balls = []
        self.ball_id_counter = 0
        self.latest_target = None
        self.latest_target_time = None
        self.latest_target_points = []
        self.selected_target_point = None
        self.current_solution = None
        self.latest_live_speed = None
        self.latest_live_speed_time = None
        self.external_solution = None
        self.external_solution_time = None
        self.external_aim_point = None
        self.external_aim_point_time = None

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
        self.target_topic = self.get_parameter('target_topic').value
        self.target_timeout = self.get_parameter('target_timeout').value
        self.min_pitch = self.get_parameter('solver.min_pitch').value
        self.max_pitch = self.get_parameter('solver.max_pitch').value
        self.pitch_samples = self.get_parameter('solver.pitch_samples').value
        self.max_iterations = self.get_parameter('solver.max_iterations').value
        self.solver_max_time = self.get_parameter('solver.max_time').value
        self.ground_z = self.get_parameter('solver.ground_z').value
        self.speed_topic = self.get_parameter('speed_topic').value
        self.use_live_speed = self.get_parameter('use_live_speed').value
        self.speed_timeout = self.get_parameter('speed_timeout').value
        self.min_live_speed = self.get_parameter('min_live_speed').value
        self.use_external_solver = self.get_parameter('use_external_solver').value
        self.solution_topic = self.get_parameter('solution_topic').value
        self.aim_point_topic = self.get_parameter('aim_point_topic').value
        self.solver_output_timeout = self.get_parameter('solver_output_timeout').value

        self.dt = 1.0 / self.freq
        self.area = math.pi * (self.radius ** 2)

    def target_callback(self, msg):
        """Caches the latest tracked target and reconstructs candidate armor points."""
        self.latest_target = msg if msg.tracking else None
        self.latest_target_time = self.get_clock().now()
        self.latest_target_points = []
        self.selected_target_point = None

        if self.latest_target is None:
            self.publish_target_markers([])
            return

        launch_pos, _ = self.get_launch_state()
        self.latest_target_points = self.reconstruct_armor_points(msg)
        self.selected_target_point = self.select_target_point(
            self.latest_target_points, launch_pos)
        self.publish_target_markers(
            self.latest_target_points, self.selected_target_point)
        self.current_solution = None

    def game_status_callback(self, msg):
        """Caches live projectile speed from the serial driver."""
        if msg.initial_speed >= self.min_live_speed:
            self.latest_live_speed = float(msg.initial_speed)
            self.latest_live_speed_time = self.get_clock().now()

    def solution_callback(self, msg):
        """Caches the latest yaw/pitch solution from the standalone solver."""
        self.external_solution = {
            'yaw': float(msg.vector.x),
            'pitch': float(msg.vector.y),
            'flight_time': float(msg.vector.z),
        }
        self.external_solution_time = self.get_clock().now()

    def aim_point_callback(self, msg):
        """Caches the latest selected aim point from the standalone solver."""
        self.external_aim_point = np.array(
            [msg.point.x, msg.point.y, msg.point.z], dtype=float)
        self.external_aim_point_time = self.get_clock().now()

    def reconstruct_armor_points(self, target_msg):
        """Rebuilds armor positions from the tracked chassis state."""
        points = []
        base_yaw = target_msg.yaw
        armor_num = max(1, target_msg.armors_num)
        is_current_pair = True

        for i in range(armor_num):
            tmp_yaw = base_yaw + i * (2 * math.pi / armor_num)

            if armor_num == 4:
                radius = target_msg.radius_1 if is_current_pair else target_msg.radius_2
                z = target_msg.position.z if is_current_pair else target_msg.position.z + target_msg.dz
                is_current_pair = not is_current_pair
            else:
                radius = target_msg.radius_1
                z = target_msg.position.z

            x = target_msg.position.x - radius * math.cos(tmp_yaw)
            y = target_msg.position.y - radius * math.sin(tmp_yaw)
            points.append(np.array([x, y, z], dtype=float))

        return points

    def select_target_point(self, points, launch_pos):
        """Chooses the armor point closest to the launcher as the active aim point."""
        if not points:
            return None
        if launch_pos is None:
            return points[0]
        return min(points, key=lambda p: np.linalg.norm(p - launch_pos))

    def publish_target_markers(self, points, selected_point=None):
        """Visualizes reconstructed armor points and highlights the active one."""
        marker_array = MarkerArray()

        if not points:
            delete_marker = Marker()
            delete_marker.header.frame_id = self.map_frame
            delete_marker.header.stamp = self.get_clock().now().to_msg()
            delete_marker.ns = "target_points"
            delete_marker.id = 0
            delete_marker.action = Marker.DELETEALL
            marker_array.markers.append(delete_marker)
            self.pub_target.publish(marker_array)
            return

        stamp = self.get_clock().now().to_msg()
        for idx, point in enumerate(points):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = stamp
            marker.ns = "target_points"
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(point[0])
            marker.pose.position.y = float(point[1])
            marker.pose.position.z = float(point[2])
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.08
            marker.scale.y = 0.08
            marker.scale.z = 0.08
            marker.lifetime = Duration(seconds=0.1).to_msg()
            marker.color.a = 0.9

            if selected_point is not None and np.allclose(point, selected_point):
                marker.color.r = 1.0
                marker.color.g = 0.2
                marker.color.b = 0.2
            else:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0

            marker_array.markers.append(marker)

        self.pub_target.publish(marker_array)

    def target_is_fresh(self):
        """Checks whether the cached tracker target is recent enough to use."""
        if self.latest_target is None or self.latest_target_time is None:
            return False
        age = (self.get_clock().now() - self.latest_target_time).nanoseconds / 1e9
        return age <= self.target_timeout

    def get_current_muzzle_speed(self):
        """Returns the live muzzle speed when available, otherwise falls back to the configured value."""
        if not self.use_live_speed:
            return self.v0
        if self.latest_live_speed is None or self.latest_live_speed_time is None:
            return self.v0
        age = (self.get_clock().now() - self.latest_live_speed_time).nanoseconds / 1e9
        if age > self.speed_timeout:
            return self.v0
        return self.latest_live_speed

    def external_solution_is_fresh(self):
        """Checks whether standalone solver outputs are recent enough to visualize."""
        if not self.use_external_solver:
            return False
        if self.external_solution is None or self.external_solution_time is None:
            return False
        if self.external_aim_point is None or self.external_aim_point_time is None:
            return False
        now = self.get_clock().now()
        solution_age = (now - self.external_solution_time).nanoseconds / 1e9
        aim_age = (now - self.external_aim_point_time).nanoseconds / 1e9
        return solution_age <= self.solver_output_timeout and aim_age <= self.solver_output_timeout

    def get_launch_pose(self):
        """
        Returns the launch position and rotation matrix that maps launch-frame vectors to map-frame.
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
            ], dtype=float)

            x, y, z, w = (
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w,
            )

            rot = np.array([
                [1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
                [2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w],
                [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y],
            ], dtype=float)
            return pos, rot

        except TransformException:
            return None, None

    def get_launch_state(self):
        """
        Calculates the current position and velocity vector of the muzzle.

        Returns:
            tuple: (position_np_array, velocity_np_array) or (None, None) if TF fails.
        """
        pos, rot = self.get_launch_pose()
        if pos is None:
            return None, None
        direction = rot[:, 0]
        direction = direction / np.linalg.norm(direction)
        vel = direction * self.get_current_muzzle_speed()
        return pos, vel

    def world_to_launch(self, point_world, launch_pos, launch_rot):
        """Transforms a world-frame point into launch-frame coordinates."""
        return launch_rot.T @ (point_world - launch_pos)

    def direction_from_angles(self, yaw, pitch, launch_rot):
        """Builds a world-frame launch direction from yaw/pitch in launch frame."""
        local_dir = np.array([
            math.cos(pitch) * math.cos(yaw),
            math.cos(pitch) * math.sin(yaw),
            math.sin(pitch)
        ], dtype=float)
        world_dir = launch_rot @ local_dir
        return world_dir / np.linalg.norm(world_dir)

    def evaluate_pitch(self, pitch, yaw, launch_pos, launch_rot, target_world, target_range):
        """
        Simulates one candidate pitch and returns the height error when crossing target range.
        """
        muzzle_speed = self.get_current_muzzle_speed()
        vel = self.direction_from_angles(yaw, pitch, launch_rot) * muzzle_speed
        pos = launch_pos.copy()
        prev_pos = pos.copy()
        prev_range = 0.0
        flight_time = 0.0
        aim_dir_xy = np.array([math.cos(yaw), math.sin(yaw)], dtype=float)

        max_steps = max(1, int(self.solver_max_time / self.dt))
        for _ in range(max_steps):
            prev_pos = pos.copy()
            acc = self.calculate_acceleration(vel)
            vel += acc * self.dt
            pos += vel * self.dt
            flight_time += self.dt

            local_pos = self.world_to_launch(pos, launch_pos, launch_rot)
            horizontal_range = float(np.dot(local_pos[:2], aim_dir_xy))

            if pos[2] <= self.ground_z and horizontal_range < target_range:
                return None

            if horizontal_range >= target_range:
                denom = horizontal_range - prev_range
                ratio = 0.0 if abs(denom) < 1e-6 else (target_range - prev_range) / denom
                ratio = min(max(ratio, 0.0), 1.0)
                hit_pos = prev_pos + ratio * (pos - prev_pos)
                hit_local = self.world_to_launch(hit_pos, launch_pos, launch_rot)
                z_error = float(hit_local[2] - self.world_to_launch(target_world, launch_pos, launch_rot)[2])
                miss_distance = float(np.linalg.norm(hit_pos - target_world))
                return {
                    'pitch': pitch,
                    'yaw': yaw,
                    'flight_time': flight_time - self.dt + ratio * self.dt,
                    'z_error': z_error,
                    'miss_distance': miss_distance,
                    'hit_pos': hit_pos,
                }

            prev_range = horizontal_range

        return None

    def solve_ballistic_arc(self, target_world, launch_pos, launch_rot):
        """Numerically solves the required yaw/pitch for the selected target point."""
        target_local = self.world_to_launch(target_world, launch_pos, launch_rot)
        target_range = float(np.linalg.norm(target_local[:2]))
        if target_range < 1e-4:
            return None

        yaw = math.atan2(target_local[1], target_local[0])
        sample_pitches = np.linspace(self.min_pitch, self.max_pitch, int(self.pitch_samples))
        valid_results = []

        for pitch in sample_pitches:
            result = self.evaluate_pitch(
                float(pitch), yaw, launch_pos, launch_rot, target_world, target_range)
            if result is not None:
                valid_results.append(result)

        if not valid_results:
            return None

        best = min(valid_results, key=lambda item: abs(item['z_error']))
        sign_change = None
        for left, right in zip(valid_results[:-1], valid_results[1:]):
            if left['z_error'] == 0.0:
                sign_change = (left, left)
                break
            if left['z_error'] * right['z_error'] < 0.0:
                sign_change = (left, right)
                break

        if sign_change is None:
            return best

        low_pitch = sign_change[0]['pitch']
        high_pitch = sign_change[1]['pitch']
        low_error = sign_change[0]['z_error']
        refined = best

        for _ in range(int(self.max_iterations)):
            mid_pitch = 0.5 * (low_pitch + high_pitch)
            mid_result = self.evaluate_pitch(
                mid_pitch, yaw, launch_pos, launch_rot, target_world, target_range)
            if mid_result is None:
                break
            refined = mid_result
            if abs(mid_result['z_error']) < abs(best['z_error']):
                best = mid_result
            if abs(mid_result['z_error']) < 1e-3:
                return mid_result
            if low_error * mid_result['z_error'] <= 0.0:
                high_pitch = mid_pitch
            else:
                low_pitch = mid_pitch
                low_error = mid_result['z_error']

        return best if abs(best['z_error']) <= abs(refined['z_error']) else refined

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
        launch_pos, launch_rot = self.get_launch_pose()
        if launch_pos is None:
            return

        current_speed = self.get_current_muzzle_speed()
        current_vel = self.direction_from_angles(0.0, 0.0, launch_rot) * current_speed
        solved_vel = None
        self.current_solution = None

        if self.external_solution_is_fresh():
            self.selected_target_point = self.external_aim_point
            self.current_solution = dict(self.external_solution)
            solved_vel = self.direction_from_angles(
                self.current_solution['yaw'],
                self.current_solution['pitch'],
                launch_rot) * current_speed
            self.publish_target_markers(
                [self.external_aim_point], self.external_aim_point)
        elif self.target_is_fresh():
            self.selected_target_point = self.select_target_point(
                self.latest_target_points, launch_pos)
            self.publish_target_markers(
                self.latest_target_points, self.selected_target_point)
            if self.selected_target_point is not None:
                self.current_solution = self.solve_ballistic_arc(
                    self.selected_target_point, launch_pos, launch_rot)
                if self.current_solution is not None:
                    solved_vel = self.direction_from_angles(
                        self.current_solution['yaw'],
                        self.current_solution['pitch'],
                        launch_rot) * current_speed

        self.publish_prediction_line(launch_pos, current_vel)
        self.publish_solution_line(launch_pos, solved_vel)
        self.publish_diagnostics(launch_pos)

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
        marker = self.build_trajectory_marker(
            start_pos, start_vel, "prediction", 9999, (0.7, 0.7, 0.7, 0.5))
        self.pub_pred.publish(marker)

    def publish_solution_line(self, start_pos, start_vel):
        """Publishes the solved ballistic trajectory, if available."""
        if start_vel is None:
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "solution"
            marker.id = 10001
            marker.action = Marker.DELETE
            self.pub_solution.publish(marker)
            return

        marker = self.build_trajectory_marker(
            start_pos, start_vel, "solution", 10001, (0.1, 1.0, 0.3, 0.8))
        self.pub_solution.publish(marker)

    def build_trajectory_marker(self, start_pos, start_vel, namespace, marker_id, color):
        """Builds a dashed line marker for a forward-simulated ballistic trajectory."""
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_LIST 
        marker.action = Marker.ADD
        
        # Short lifetime to allow real-time updates without ghosting
        marker.lifetime = Duration(seconds=0.05).to_msg() 
        
        marker.scale.x = 0.02

        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

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
        return marker

    def publish_diagnostics(self, launch_pos):
        """Publishes markers that explain the current ballistic solution quality."""
        marker_array = MarkerArray()

        if self.selected_target_point is None or self.current_solution is None:
            delete_marker = Marker()
            delete_marker.header.frame_id = self.map_frame
            delete_marker.header.stamp = self.get_clock().now().to_msg()
            delete_marker.ns = "ballistic_diag"
            delete_marker.id = 0
            delete_marker.action = Marker.DELETEALL
            marker_array.markers.append(delete_marker)
            self.pub_diag.publish(marker_array)
            return

        stamp = self.get_clock().now().to_msg()

        line_marker = Marker()
        line_marker.header.frame_id = self.map_frame
        line_marker.header.stamp = stamp
        line_marker.ns = "ballistic_diag"
        line_marker.id = 0
        line_marker.type = Marker.LINE_LIST
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.015
        line_marker.lifetime = Duration(seconds=0.1).to_msg()
        line_marker.color.r = 1.0
        line_marker.color.g = 0.4
        line_marker.color.b = 0.0
        line_marker.color.a = 0.9
        line_marker.points = [
            Point(x=float(launch_pos[0]), y=float(launch_pos[1]), z=float(launch_pos[2])),
            Point(
                x=float(self.selected_target_point[0]),
                y=float(self.selected_target_point[1]),
                z=float(self.selected_target_point[2]),
            ),
        ]
        marker_array.markers.append(line_marker)

        hit_marker = Marker()
        hit_marker.header.frame_id = self.map_frame
        hit_marker.header.stamp = stamp
        hit_marker.ns = "ballistic_diag"
        hit_marker.id = 1
        hit_marker.type = Marker.SPHERE
        hit_marker.action = Marker.ADD
        hit_marker.pose.position.x = float(self.current_solution['hit_pos'][0])
        hit_marker.pose.position.y = float(self.current_solution['hit_pos'][1])
        hit_marker.pose.position.z = float(self.current_solution['hit_pos'][2])
        hit_marker.pose.orientation.w = 1.0
        hit_marker.scale.x = 0.06
        hit_marker.scale.y = 0.06
        hit_marker.scale.z = 0.06
        hit_marker.lifetime = Duration(seconds=0.1).to_msg()
        hit_marker.color.r = 0.1
        hit_marker.color.g = 0.8
        hit_marker.color.b = 1.0
        hit_marker.color.a = 0.95
        marker_array.markers.append(hit_marker)

        text_marker = Marker()
        text_marker.header.frame_id = self.map_frame
        text_marker.header.stamp = stamp
        text_marker.ns = "ballistic_diag"
        text_marker.id = 2
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = float(self.selected_target_point[0])
        text_marker.pose.position.y = float(self.selected_target_point[1])
        text_marker.pose.position.z = float(self.selected_target_point[2] + 0.2)
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.08
        text_marker.lifetime = Duration(seconds=0.1).to_msg()
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = (
            f"yaw={self.current_solution['yaw']:.3f} rad\n"
            f"pitch={self.current_solution['pitch']:.3f} rad\n"
            f"t={self.current_solution['flight_time']:.3f} s\n"
            f"miss={self.current_solution['miss_distance']:.3f} m\n"
            f"v0={self.get_current_muzzle_speed():.2f} m/s"
        )
        marker_array.markers.append(text_marker)

        self.pub_diag.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = LaunchSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
