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

        # State
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.path_points = []
        self.obstacle_vectors = []

        # Control Loop (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Local Reactive Controller Initialized (20 Hz)')

    def path_callback(self, msg: Path):
        self.path_points = [
            (p.pose.position.x, p.pose.position.y) for p in msg.poses
        ]

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

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

        # 1. Pure Pursuit Target Waypoint
        target_pt = None
        for pt in self.path_points:
            d = math.hypot(pt[0] - self.current_x, pt[1] - self.current_y)
            if d >= self.lookahead:
                target_pt = pt
                break

        if target_pt is None:
            target_pt = self.path_points[-1]

        # 2. Attractive Vector toward Path Target
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

        # 3. Reactive Repulsion Vector
        v_rep = np.array([0.0, 0.0])
        for ox, oy, r in self.obstacle_vectors:
            strength = self.k_rep * (1.0 / (r + 1e-3) - 1.0 / self.d_rep)
            v_rep += np.array([-ox / r, -oy / r]) * strength

        rep_norm = np.linalg.norm(v_rep)
        if rep_norm > 1.0:
            v_rep = (v_rep / rep_norm) * 1.0

        total_vec = v_att + v_rep
        heading_error = math.atan2(total_vec[1], total_vec[0])

        angular_z = float(
            np.clip(2.0 * heading_error, -self.max_w, self.max_w)
        )
        speed_scale = max(0.2, math.cos(heading_error))
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
