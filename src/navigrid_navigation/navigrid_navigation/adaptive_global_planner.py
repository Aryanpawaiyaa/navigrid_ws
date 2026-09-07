#!/usr/bin/env python3
"""Adaptive Global Planner Node for NaviGrid: Stage 2.

Autonomously computes the optimal global trajectory from Start Zone (A)
to Goal Zone (B), balancing terrain slope/elevation costs against flat
zig-zag travel distance.
"""

import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import numpy as np
import rclpy
from rclpy.node import Node


class AdaptiveGlobalPlanner(Node):

    def __init__(self):
        super().__init__('adaptive_global_planner')

        # Parameters
        self.declare_parameter('slope_cost_multiplier', 4.0)
        self.declare_parameter('payload_weight_kg', 25.0)
        self.declare_parameter('max_incline_payload_limit', 35.0)
        self.declare_parameter('default_goal_x', 12.0)
        self.declare_parameter('default_goal_y', 12.0)

        self.slope_multiplier = (
            self.get_parameter('slope_cost_multiplier').value
        )
        self.payload_kg = self.get_parameter('payload_weight_kg').value
        self.max_payload_limit = (
            self.get_parameter('max_incline_payload_limit').value
        )

        # Publishers & Subscribers
        self.path_pub = self.create_publisher(Path, '/plan', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10
        )

        self.current_pose = (-12.0, -12.0)
        self.goal_pose = (
            self.get_parameter('default_goal_x').value,
            self.get_parameter('default_goal_y').value
        )
        self.has_planned = False

        # Periodic check / publish timer
        self.timer = self.create_timer(1.0, self.plan_and_publish)
        self.get_logger().info(
            f'Adaptive Global Planner Initialized | '
            f'Payload: {self.payload_kg:.1f}kg, '
            f'Slope Multiplier: {self.slope_multiplier:.2f}'
        )

    def odom_callback(self, msg: Odometry):
        self.current_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    def goal_callback(self, msg: PoseStamped):
        self.goal_pose = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(
            f'New Goal Received: ({self.goal_pose[0]:.2f}, '
            f'{self.goal_pose[1]:.2f})'
        )
        self.has_planned = False
        self.plan_and_publish()

    def generate_direct_ramp_path(self, start, goal):
        """Generate trajectory traversing direct 3D incline ramp."""
        waypoints = [
            start,
            (-9.0, -9.0),
            (-5.5, -5.5),   # Ascending ramp base
            (0.0, 0.0),     # Ramp summit platform (elevated z=1.4m)
            (5.5, 5.5),     # Descending ramp base
            (9.0, 9.0),
            goal
        ]
        return self._interpolate_path(waypoints, step=0.3)

    def generate_zigzag_floor_path(self, start, goal):
        """Generate trajectory navigating flat 2D zig-zag corridor."""
        waypoints = [
            start,
            (-12.0, -8.0),
            (-12.0, 0.0),   # Navigating around warehouse rack 1
            (-12.0, 6.0),
            (-5.0, 6.0),    # Chicane through pillar 1 and rack 2
            (-5.0, 0.0),
            (0.0, -6.0),    # Passing around rack 3
            (5.0, -6.0),
            (5.0, 2.0),     # Passing through rack 4 corridor
            (10.0, 2.0),
            (10.0, 10.0),
            goal
        ]
        return self._interpolate_path(waypoints, step=0.3)

    def _interpolate_path(self, waypoints, step=0.3):
        """Interpolate smooth equidistant points along waypoint list."""
        dense_points = []
        for i in range(len(waypoints) - 1):
            p1 = np.array(waypoints[i])
            p2 = np.array(waypoints[i + 1])
            dist = np.linalg.norm(p2 - p1)
            num_steps = max(int(dist / step), 1)
            for s in range(num_steps):
                interp = p1 + (p2 - p1) * (s / float(num_steps))
                dense_points.append(tuple(interp))
        dense_points.append(waypoints[-1])
        return dense_points

    def calculate_path_cost(self, path_points, is_ramp: bool):
        """Evaluate cost function: J = Distance + (is_ramp * SlopeCost)."""
        length = 0.0
        for i in range(len(path_points) - 1):
            dx = path_points[i + 1][0] - path_points[i][0]
            dy = path_points[i + 1][1] - path_points[i][1]
            length += math.hypot(dx, dy)

        if not is_ramp:
            return length, length, 0.0

        elevation_rise = 2.8
        if self.payload_kg > self.max_payload_limit:
            slope_cost = 1e6
        else:
            slope_cost = (
                elevation_rise * self.slope_multiplier *
                (self.payload_kg / 15.0)
            )

        total_cost = length + slope_cost
        return total_cost, length, slope_cost

    def plan_and_publish(self):
        """Evaluate both paths, select optimal route, and publish /plan."""
        ramp_pts = self.generate_direct_ramp_path(
            self.current_pose, self.goal_pose
        )
        zigzag_pts = self.generate_zigzag_floor_path(
            self.current_pose, self.goal_pose
        )

        cost_ramp, len_ramp, elev_ramp = self.calculate_path_cost(
            ramp_pts, is_ramp=True
        )
        cost_zigzag, len_zigzag, _ = self.calculate_path_cost(
            zigzag_pts, is_ramp=False
        )

        if cost_ramp < cost_zigzag:
            chosen_name = 'Direct Incline Ramp (3D)'
            chosen_pts = ramp_pts
        else:
            chosen_name = 'Zig-Zag Ground Floor Corridor (2D)'
            chosen_pts = zigzag_pts

        if not self.has_planned:
            self.get_logger().info('=' * 50)
            self.get_logger().info('--- ADAPTIVE PATH SELECTION ---')
            self.get_logger().info(
                f'Option 1 [Ramp]:   Dist = {len_ramp:.2f}m | '
                f'Elevation = {elev_ramp:.2f} | Total = {cost_ramp:.2f}'
            )
            self.get_logger().info(
                f'Option 2 [ZigZag]: Dist = {len_zigzag:.2f}m | '
                f'Elevation = 0.00 | Total = {cost_zigzag:.2f}'
            )
            self.get_logger().info(f'-> AUTONOMOUS CHOICE: {chosen_name}')
            self.get_logger().info('=' * 50)
            self.has_planned = True

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'

        for pt in chosen_pts:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(pt[0])
            pose.pose.position.y = float(pt[1])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveGlobalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
