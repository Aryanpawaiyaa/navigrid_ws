#!/usr/bin/env python3
"""Costmap Generator Node for NaviGrid: The Adaptive Path Challenge.

Generates a coherent 2D/3D costmap representing the 30m x 30m arena,
including static obstacles, boundary walls, inflation layers, and
elevation slope costs.
"""

import math

from geometry_msgs.msg import Pose
from nav_msgs.msg import MapMetaData, OccupancyGrid
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class CostmapGenerator(Node):

    def __init__(self):
        super().__init__('costmap_generator')

        # Declare parameters
        self.declare_parameter('grid_resolution', 0.15)
        self.declare_parameter('arena_min_x', -15.0)
        self.declare_parameter('arena_max_x', 15.0)
        self.declare_parameter('arena_min_y', -15.0)
        self.declare_parameter('arena_max_y', 15.0)
        self.declare_parameter('robot_radius', 0.45)
        self.declare_parameter('slope_cost_multiplier', 4.0)

        self.resolution = self.get_parameter('grid_resolution').value
        self.min_x = self.get_parameter('arena_min_x').value
        self.max_x = self.get_parameter('arena_max_x').value
        self.min_y = self.get_parameter('arena_min_y').value
        self.max_y = self.get_parameter('arena_max_y').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.slope_multiplier = (
            self.get_parameter('slope_cost_multiplier').value
        )

        self.width = int(
            math.ceil((self.max_x - self.min_x) / self.resolution)
        )
        self.height = int(
            math.ceil((self.max_y - self.min_y) / self.resolution)
        )

        self.get_logger().info(
            f'Initializing Costmap: {self.width}x{self.height} cells '
            f'at {self.resolution}m/cell'
        )

        # Publishers & Subscribers
        self.costmap_pub = self.create_publisher(
            OccupancyGrid, '/costmap', 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        # Precompute static arena layer
        self.static_grid = np.zeros(
            (self.height, self.width), dtype=np.int8
        )
        self.slope_grid = np.zeros(
            (self.height, self.width), dtype=np.float32
        )
        self._build_static_arena()

        # Dynamic scan points
        self.latest_scan_points = []

        # Periodic publication timer (2 Hz)
        self.timer = self.create_timer(0.5, self.publish_costmap)

    def world_to_grid(self, x: float, y: float):
        """Convert world coordinates (meters) to grid indices (gx, gy)."""
        gx = int((x - self.min_x) / self.resolution)
        gy = int((y - self.min_y) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx: int, gy: int):
        """Convert grid indices (gx, gy) to world coordinates (meters)."""
        x = self.min_x + (gx + 0.5) * self.resolution
        y = self.min_y + (gy + 0.5) * self.resolution
        return x, y

    def _mark_box(self, cx: float, cy: float, sx: float, sy: float,
                  cost: int = 100):
        """Mark a rectangular obstacle in the static grid."""
        gx_min, gy_min = self.world_to_grid(cx - sx / 2.0, cy - sy / 2.0)
        gx_max, gy_max = self.world_to_grid(cx + sx / 2.0, cy + sy / 2.0)

        gx_min = max(0, min(gx_min, self.width - 1))
        gx_max = max(0, min(gx_max, self.width - 1))
        gy_min = max(0, min(gy_min, self.height - 1))
        gy_max = max(0, min(gy_max, self.height - 1))

        self.static_grid[gy_min:gy_max + 1, gx_min:gx_max + 1] = cost

    def _mark_cylinder(self, cx: float, cy: float, radius: float,
                       cost: int = 100):
        """Mark a cylindrical pillar obstacle in the static grid."""
        center_gx, center_gy = self.world_to_grid(cx, cy)
        cell_rad = int(math.ceil(radius / self.resolution))

        for dy in range(-cell_rad, cell_rad + 1):
            for dx in range(-cell_rad, cell_rad + 1):
                if dx * dx + dy * dy <= cell_rad * cell_rad:
                    gx = center_gx + dx
                    gy = center_gy + dy
                    if 0 <= gx < self.width and 0 <= gy < self.height:
                        self.static_grid[gy, gx] = cost

    def _build_static_arena(self):
        """Initialize perimeter walls, racks, pillars, and incline ramp."""
        wall_thick = 0.4
        self._mark_box(0.0, 15.0, 30.0, wall_thick, 100)   # North
        self._mark_box(0.0, -15.0, 30.0, wall_thick, 100)  # South
        self._mark_box(15.0, 0.0, wall_thick, 30.0, 100)   # East
        self._mark_box(-15.0, 0.0, wall_thick, 30.0, 100)  # West

        # Warehouse Storage Racks
        self._mark_box(-8.0, -2.0, 1.2, 8.0, 100)
        self._mark_box(-2.0, 4.0, 8.0, 1.2, 100)
        self._mark_box(4.0, -4.0, 1.2, 8.0, 100)
        self._mark_box(8.0, 2.0, 1.2, 8.0, 100)

        # Structural Pillars
        self._mark_cylinder(-12.0, 3.0, 0.45, 100)
        self._mark_cylinder(-4.0, -8.0, 0.45, 100)
        self._mark_cylinder(6.0, 9.0, 0.45, 100)

        # Incline Ramp Zone (Diagonal Corridor)
        for gy in range(self.height):
            for gx in range(self.width):
                wx, wy = self.grid_to_world(gx, gy)
                diag_dist = abs(wx - wy) / math.sqrt(2.0)
                proj = (wx + wy) / math.sqrt(2.0)
                if diag_dist <= 1.3 and -11.0 <= proj <= 11.0:
                    grade_factor = 25.0 * self.slope_multiplier
                    self.slope_grid[gy, gx] = grade_factor

    def scan_callback(self, msg: LaserScan):
        """Store dynamic obstacles detected by laser scan."""
        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                ox = r * math.cos(angle)
                oy = r * math.sin(angle)
                points.append((ox, oy))
            angle += msg.angle_increment
        self.latest_scan_points = points

    def publish_costmap(self):
        """Construct merged costmap with inflation and publish."""
        costmap_data = np.copy(self.static_grid)

        # Add elevation slope costs to navigable cells
        mask_navigable = (costmap_data < 100)
        costmap_data[mask_navigable] = np.clip(
            costmap_data[mask_navigable] +
            self.slope_grid[mask_navigable].astype(np.int8),
            0, 99
        )

        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'odom'

        meta = MapMetaData()
        meta.resolution = float(self.resolution)
        meta.width = int(self.width)
        meta.height = int(self.height)

        origin_pose = Pose()
        origin_pose.position.x = float(self.min_x)
        origin_pose.position.y = float(self.min_y)
        origin_pose.position.z = 0.0
        origin_pose.orientation.w = 1.0
        meta.origin = origin_pose

        grid_msg.info = meta
        grid_msg.data = costmap_data.flatten().tolist()

        self.costmap_pub.publish(grid_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CostmapGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
