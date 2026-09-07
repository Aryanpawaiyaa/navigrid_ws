#!/usr/bin/env python3
"""Safety Override Node for NaviGrid: Stage 3.

Implements a dedicated high-priority safety node enforcing the
velocity-dependent threshold:
    d_safe = k * (v^2) + d_min

If any dynamic obstacle breaches d_safe, it immediately overrides
/cmd_vel_nav and commands a full emergency stop on /cmd_vel.
"""

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray


class SafetyOverrideNode(Node):

    def __init__(self):
        super().__init__('safety_override_node')

        # Parameters from YAML or defaults
        self.declare_parameter('safety_k', 1.25)
        self.declare_parameter('safety_d_min', 0.65)
        self.declare_parameter('detection_fov_deg', 150.0)

        self.k = self.get_parameter('safety_k').value
        self.d_min = self.get_parameter('safety_d_min').value
        fov_deg = self.get_parameter('detection_fov_deg').value
        self.fov_rad = math.radians(fov_deg)

        # State Variables
        self.current_linear_velocity = 0.0
        self.nominal_cmd = Twist()
        self.min_obstacle_distance = float('inf')
        self.emergency_stop_active = False

        # Publishers & Subscribers
        self.cmd_out_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/safety_zone_markers', 10
        )

        self.cmd_in_sub = self.create_subscription(
            Twist, '/cmd_vel_nav', self.cmd_in_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        # High-frequency safety arbitration loop (50 Hz)
        self.loop_timer = self.create_timer(
            0.02, self.safety_arbitration_loop
        )
        self.get_logger().info(
            f'Stage 3 Safety Override Active | '
            f'Formula: d_safe = {self.k:.2f} * v^2 + {self.d_min:.2f}'
        )

    def odom_callback(self, msg: Odometry):
        self.current_linear_velocity = max(0.0, msg.twist.twist.linear.x)

    def cmd_in_callback(self, msg: Twist):
        self.nominal_cmd = msg

    def scan_callback(self, msg: LaserScan):
        """Analyze forward FOV laser returns for minimum obstacle distance."""
        min_dist = float('inf')
        half_fov = self.fov_rad / 2.0

        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                if -half_fov <= angle <= half_fov:
                    if r < min_dist:
                        min_dist = r
            angle += msg.angle_increment

        self.min_obstacle_distance = min_dist

    def compute_d_safe(self, v: float) -> float:
        """Official Stage 3 formula: d_safe = k * (v^2) + d_min."""
        return float(self.k * (v ** 2) + self.d_min)

    def safety_arbitration_loop(self):
        v = self.current_linear_velocity
        d_safe = self.compute_d_safe(v)
        obs_dist = self.min_obstacle_distance

        # Breach condition
        if obs_dist < d_safe:
            if not self.emergency_stop_active:
                self.get_logger().warn(
                    f'EMERGENCY STOP! Obstacle at {obs_dist:.2f}m < '
                    f'd_safe({d_safe:.2f}m) [v = {v:.2f} m/s]'
                )
                self.emergency_stop_active = True

            zero_cmd = Twist()
            self.cmd_out_pub.publish(zero_cmd)
        else:
            if self.emergency_stop_active:
                self.get_logger().info(
                    f'Obstacle clear ({obs_dist:.2f}m >= {d_safe:.2f}m). '
                    'Resuming autonomous control.'
                )
                self.emergency_stop_active = False

            self.cmd_out_pub.publish(self.nominal_cmd)

        self.publish_rviz_safety_markers(d_safe)

    def publish_rviz_safety_markers(self, d_safe: float):
        """Publish dynamic safety zone cylinder in RViz."""
        marker_array = MarkerArray()

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'base_footprint'
        marker.ns = 'safety_envelope'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = d_safe / 2.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.05
        marker.pose.orientation.w = 1.0

        marker.scale.x = float(d_safe)
        marker.scale.y = float(d_safe)
        marker.scale.z = 0.05

        if self.emergency_stop_active:
            marker.color.r = 1.0
            marker.color.g = 0.1
            marker.color.b = 0.1
            marker.color.a = 0.6
        else:
            marker.color.r = 0.1
            marker.color.g = 0.9
            marker.color.b = 0.2
            marker.color.a = 0.35

        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyOverrideNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
