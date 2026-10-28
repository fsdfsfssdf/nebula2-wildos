#!/usr/bin/env python3
"""Feed the elevation map's traversability into the Nav2 costmaps.

WildOS decides *where* to go on a graph whose nodes and edges only exist where
the elevation map says a robot fits (`graphnav_mapper`, 0.5 m clearance). The
repo then hands 2 m-ahead pose goals to the robot's own navigator, which is
what actually avoids terrain. Our simulation replaces that navigator with
Nav2, so Nav2 has to be given the same terrain knowledge.

``elevation_relay.py`` already publishes ``/spot1/local_map`` as an
OccupancyGrid in the odom frame: 0 = traversable, 100 = blocked, -1 = unknown.
This node turns the blocked cells into a point cloud the costmaps can subscribe
to, next to the (ground-filtered) lidar cloud.
"""

from __future__ import annotations

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


class TraversabilityRelay(Node):
    def __init__(self) -> None:
        super().__init__("trav_obstacle_relay")
        self.declare_parameter("input_topic", "/spot1/local_map")
        self.declare_parameter("output_topic", "/spot1/trav_obstacles")
        self.declare_parameter("blocked_threshold", 65)
        self.declare_parameter("max_points", 12000)
        self.declare_parameter("point_height", 0.0)
        # Blocked cells that sit right next to unobserved cells are not
        # trusted: the traversability filter extrapolates slope across the
        # validity boundary and produces a fake cliff there. graphnav_mapper
        # solves the same problem with its unknown_sdf check.
        self.declare_parameter("unknown_margin", 0.6)
        # Terrain must never wall the robot in. WildOS only uses traversability
        # to decide which graph nodes/edges exist; the pose the robot already
        # occupies is always valid. Nav2 has no such notion, so blocked cells
        # inside this radius around the base are dropped - the ground-filtered
        # lidar still marks real geometry there.
        self.declare_parameter("robot_exclude_radius", 2.5)
        self.declare_parameter("odom_topic", "/spot1/odom")

        self.threshold = int(self.get_parameter("blocked_threshold").value)
        self.max_points = int(self.get_parameter("max_points").value)
        self.point_height = float(self.get_parameter("point_height").value)
        self.unknown_margin = float(self.get_parameter("unknown_margin").value)
        self.exclude_radius = float(
            self.get_parameter("robot_exclude_radius").value
        )
        self.robot_xy = None
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self.on_odom,
            qos_profile_sensor_data,
        )

        # The costmaps subscribe with a reliable profile; a best-effort
        # publisher would be dropped as incompatible.
        qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(
            PointCloud2, str(self.get_parameter("output_topic").value), qos
        )
        self.create_subscription(
            OccupancyGrid, str(self.get_parameter("input_topic").value),
            self.on_grid, qos
        )
        self.fields = [
            pc2.PointField(name="x", offset=0,
                           datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name="y", offset=4,
                           datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name="z", offset=8,
                           datatype=pc2.PointField.FLOAT32, count=1),
        ]
        self.last_report = 0.0
        self.get_logger().info(
            "Traversability relay: cells >= %d become costmap obstacles"
            % self.threshold
        )

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.robot_xy = (p.x, p.y)

    def on_grid(self, msg: OccupancyGrid) -> None:
        info = msg.info
        if not msg.data or info.width == 0 or info.height == 0:
            return
        grid = np.asarray(msg.data, dtype=np.int16).reshape(
            info.height, info.width
        )
        blocked = grid >= self.threshold
        unknown = grid < 0
        radius_px = int(round(self.unknown_margin / max(info.resolution, 1e-6)))
        if radius_px > 0 and unknown.any():
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (2 * radius_px + 1, 2 * radius_px + 1)
            )
            unknown_band = cv2.dilate(unknown.astype(np.uint8), kernel) > 0
            blocked = blocked & ~unknown_band
        rows, cols = np.nonzero(blocked)
        res = info.resolution
        xs = info.origin.position.x + (cols + 0.5) * res
        ys = info.origin.position.y + (rows + 0.5) * res
        if self.robot_xy is not None and self.exclude_radius > 0.0:
            keep = (
                np.hypot(xs - self.robot_xy[0], ys - self.robot_xy[1])
                > self.exclude_radius
            )
            xs, ys = xs[keep], ys[keep]
        points = np.column_stack(
            [xs, ys, np.full(len(xs), self.point_height)]
        ).astype(np.float32)
        if len(points) > self.max_points:
            stride = int(np.ceil(len(points) / self.max_points))
            points = points[::stride]

        cloud = pc2.create_cloud(msg.header, self.fields, points)
        self.pub.publish(cloud)

        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_report > 5.0:
            self.last_report = now
            self.get_logger().info(
                "traversability: %d blocked cells -> %d obstacle points"
                % (int(blocked.sum()), len(points))
            )


def main() -> None:
    rclpy.init()
    node = TraversabilityRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()