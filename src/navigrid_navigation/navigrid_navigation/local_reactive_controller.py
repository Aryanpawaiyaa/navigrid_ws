#!/usr/bin/env python3
"""Local Reactive Controller Node for NaviGrid: Stage 3.

Executes trajectory tracking along the global path with real-time reactive
avoidance of crossing pedestrians using artificial potential fields,
preventing deadlocks and stall loops.
Publishes nominal control on /cmd_vel_nav (fed into Safety Override Node).
"""

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class LocalReactiveController(Node):

    def __init__(self):
        super().__init__('local_reactive_controller')

        # Control Parameters
        self.declare_parameter('max_linear_speed', 0.8)
        self.declare_parameter('max_angular_speed', 1.2)
        self.declare_parameter('lookahead_distance', 1.0)
        self.declare_parameter('goal_tolerance', 0.4)
        self.declare_parameter('repulsion_threshold', 1.8)
        self.declare_parameter('repulsion_gain', 0.6)

        self.max_v = self.get_parameter('max_linear_speed').value
        self.max_w = self.get_parameter('max_angular_speed').value
        self.lookahead = self.get_parameter('lookahead_distance').value
        self.goal_tol = self.get_parameter('goal_tolerance').value
        self.d_rep = self.get_parameter('repulsion_threshold').value
        self.k_rep = self.get_parameter('repulsion_gain').value

        # TF Listener for map-frame localization
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.path_sub = self.create_subscription(
            Path, '/plan', self.path_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        # State (initialized at arena Start Zone A)
        self.current_x = -12.0
        self.current_y = -12.0
        self.current_yaw = 0.785398
        self.path_points = []
        self.obstacle_vectors = []
        self.current_waypoint_idx = 0

        # Control Loop (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Local Reactive Controller Initialized (20 Hz)')

    def path_callback(self, msg: Path):
        self.path_points = [
            (p.pose.position.x, p.pose.position.y) for p in msg.poses
        ]
        if self.current_waypoint_idx >= len(self.path_points):
            self.current_waypoint_idx = 0

    def odom_callback(self, msg: Odometry):
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()
            )
            self.current_x = t.transform.translation.x
            self.current_y = t.transform.translation.y
            q = t.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        except Exception:
            # Fallback transform from local odom to map frame
            ox = msg.pose.pose.position.x
            oy = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            local_yaw = math.atan2(siny_cosp, cosy_cosp)

            cos_a = math.cos(0.785398)
            sin_a = math.sin(0.785398)
            self.current_x = -12.0 + ox * cos_a - oy * sin_a
            self.current_y = -12.0 + ox * sin_a + oy * cos_a
            self.current_yaw = local_yaw + 0.785398

    def scan_callback(self, msg: LaserScan):
        """Extract nearby obstacles in robot body frame for avoidance."""
        vectors = []
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < self.d_rep:
                ox = r * math.cos(angle)
                oy = r * math.sin(angle)
                vectors.append((ox, oy, r))
            angle += msg.angle_increment
        self.obstacle_vectors = vectors

    def control_loop(self):
        cmd = Twist()
        if not self.path_points:
            self.cmd_pub.publish(cmd)
            return

        gx, gy = self.path_points[-1]
        dist_to_goal = math.hypot(gx - self.current_x, gy - self.current_y)
        if dist_to_goal <= self.goal_tol:
            self.cmd_pub.publish(cmd)
            return

        # 1. Monotonic Forward Progress Tracking
        closest_idx = self.current_waypoint_idx
        min_dist = float('inf')
        search_window = min(len(self.path_points), self.current_waypoint_idx + 40)
        for i in range(self.current_waypoint_idx, search_window):
            pt = self.path_points[i]
            d = math.hypot(pt[0] - self.current_x, pt[1] - self.current_y)
            if d < min_dist:
                min_dist = d
                closest_idx = i

        self.current_waypoint_idx = closest_idx

        # 2. Pure Pursuit: Look ahead along trajectory strictly forward from closest_idx
        target_pt = None
        for i in range(self.current_waypoint_idx, len(self.path_points)):
            pt = self.path_points[i]
            d = math.hypot(pt[0] - self.current_x, pt[1] - self.current_y)
            if d >= self.lookahead:
                target_pt = pt
                break

        if target_pt is None:
            target_pt = self.path_points[-1]

        # 3. Attractive Vector toward Path Target in robot frame
        dx_w = target_pt[0] - self.current_x
        dy_w = target_pt[1] - self.current_y

        att_x = (
            dx_w * math.cos(self.current_yaw) +
            dy_w * math.sin(self.current_yaw)
        )
        att_y = (
            -dx_w * math.sin(self.current_yaw) +
            dy_w * math.cos(self.current_yaw)
        )
        att_dist = math.hypot(att_x, att_y) + 1e-6
        v_att = np.array([att_x / att_dist, att_y / att_dist])

        # 4. Reactive Obstacle Avoidance (APF with Deadlock-Free Lateral Evasion)
        fwd_obstacles = [
            (ox, oy, r) for (ox, oy, r) in self.obstacle_vectors
            if ox > 0.15 and abs(oy) < 1.2
        ]

        rep_lateral = 0.0
        min_obs_r = float('inf')
        if fwd_obstacles:
            fwd_obstacles.sort(key=lambda item: item[2])
            ox_c, oy_c, min_obs_r = fwd_obstacles[0]

            # Steer away from obstacle side
            steer_sign = -1.0 if oy_c > 0.0 else 1.0
            if abs(oy_c) < 0.08:
                steer_sign = 1.0  # Break symmetry when obstacle is dead ahead

            rep_factor = max(0.0, 1.0 - (min_obs_r / self.d_rep))
            rep_lateral = steer_sign * self.k_rep * rep_factor

        total_vec = np.array([v_att[0], v_att[1] + rep_lateral])
        heading_error = math.atan2(total_vec[1], total_vec[0])

        angular_z = float(
            np.clip(2.5 * heading_error, -self.max_w, self.max_w)
        )

        speed_scale = max(0.15, math.cos(heading_error))
        if min_obs_r < self.d_rep:
            clearance_factor = max(0.2, (min_obs_r - 0.4) / (self.d_rep - 0.4))
            speed_scale = min(speed_scale, clearance_factor)

        linear_x = float(np.clip(self.max_v * speed_scale, 0.0, self.max_v))

        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LocalReactiveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
