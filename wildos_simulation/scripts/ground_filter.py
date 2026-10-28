#!/usr/bin/env python3
"""Strip the ground from the lidar cloud so low obstacles survive costmap filtering.

The costmaps used a fixed height gate (`min_obstacle_height`), but that gate is
evaluated in the odom frame whose z origin sits on the robot base, not on the
ground: the terrain floats between -0.61 m (grass) and -0.40 m (raised road).
A 0.24 m rock therefore tops out at -0.32 m and a fixed gate either drops the
rock or marks the road.

This node fits the ground plane from the cloud itself (low envelope sampled
per azimuth/range bin) and republishes only points that stand at least
`min_height_above_ground` above it. Downstream the costmaps can then accept
every height (`min_obstacle_height: -10`) without ever seeing the ground.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


class GroundFilter(Node):
    def __init__(self) -> None:
        super().__init__("ground_filter")
        self.declare_parameter("input_topic", "/spot1/points")
        self.declare_parameter("output_topic", "/spot1/points_obstacles")
        # Kerbs in this scene are 0.12 m tall, exactly the old threshold, so
        # they were classified as ground and the rover drove over them.
        self.declare_parameter("min_height_above_ground", 0.06)
        # The rover body occludes the lidar below ~0.45 m. Anything beyond that
        # has to survive: obstacles that vanish when they get close are worse
        # than no obstacles, because the costmap clears their cells on the next
        # raytrace and the controller drives straight into them.
        self.declare_parameter("min_range", 0.45)
        self.declare_parameter("max_range", 12.0)
        self.declare_parameter("azimuth_bins", 36)
        self.declare_parameter("range_bins", 6)
        self.declare_parameter("ground_percentile", 5.0)

        self.min_height = float(self.get_parameter("min_height_above_ground").value)
        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.azimuth_bins = int(self.get_parameter("azimuth_bins").value)
        self.range_bins = int(self.get_parameter("range_bins").value)
        self.ground_percentile = float(self.get_parameter("ground_percentile").value)

        # RELIABLE on purpose: nav2_costmap_2d subscribes to observation
        # sources with a reliable profile, and a best-effort publisher is
        # silently dropped ("incompatible QoS") - the costmap then sees no
        # obstacles at all.
        qos = QoSProfile(
            depth=5,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pub = self.create_publisher(
            PointCloud2, str(self.get_parameter("output_topic").value), qos
        )
        self.create_subscription(
            PointCloud2, str(self.get_parameter("input_topic").value),
            self.on_cloud, qos
        )
        self.get_logger().info(
            "Ground filter: keeping points %.2f m above the fitted ground"
            % self.min_height
        )
        self.last_report = 0.0

    @staticmethod
    def ground_plane(pts: np.ndarray) -> np.ndarray:
        """Least squares plane through the already-sampled ground points.

        ``pts`` holds the low envelope of the cloud (lowest returns per azimuth
        and range bin), so a trimmed least-squares fit converges on the real
        ground; starting from a global percentile instead latched onto the
        far-away ground and produced a plane metres below the robot.
        """
        design = np.column_stack([pts[:, 0], pts[:, 1], np.ones(len(pts))])
        coeff, *_ = np.linalg.lstsq(design, pts[:, 2], rcond=None)
        for _ in range(3):
            resid = pts[:, 2] - design @ coeff
            keep = np.abs(resid) < 0.15
            if keep.sum() < 30:
                break
            coeff, *_ = np.linalg.lstsq(design[keep], pts[keep, 2], rcond=None)
        return coeff

    def on_cloud(self, msg: PointCloud2) -> None:
        raw = np.array(
            [[x, y, z] for x, y, z in
             pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)],
            dtype=float,
        )
        if raw.size == 0:
            return
        raw = raw[np.isfinite(raw).all(axis=1)]
        if raw.size == 0:
            return

        ranges = np.linalg.norm(raw[:, :2], axis=1)
        usable = raw[(ranges > self.min_range) & (ranges < self.max_range)]
        if len(usable) < 100:
            return

        # Ground samples: lowest returns per azimuth sector and range ring.
        az = np.arctan2(usable[:, 1], usable[:, 0])
        rng = np.linalg.norm(usable[:, :2], axis=1)
        az_idx = np.clip(
            ((az + math.pi) / (2 * math.pi) * self.azimuth_bins).astype(int),
            0, self.azimuth_bins - 1,
        )
        ring = np.clip(
            ((rng - self.min_range) / (self.max_range - self.min_range)
             * self.range_bins).astype(int),
            0, self.range_bins - 1,
        )
        samples = []
        for a in range(self.azimuth_bins):
            for b in range(self.range_bins):
                sel = usable[(az_idx == a) & (ring == b)]
                if len(sel) < 12:
                    continue
                k = max(1, int(len(sel) * self.ground_percentile / 100.0))
                low = sel[np.argsort(sel[:, 2])[:k]]
                samples.append(low)
        if not samples:
            return
        ground_pts = np.vstack(samples)

        coeff = self.ground_plane(ground_pts)
        height = usable[:, 2] - (coeff[0] * usable[:, 0]
                                 + coeff[1] * usable[:, 1] + coeff[2])
        keep = usable[height > self.min_height]

        cloud = pc2.create_cloud(
            msg.header,
            [pc2.PointField(name="x", offset=0,
                            datatype=pc2.PointField.FLOAT32, count=1),
             pc2.PointField(name="y", offset=4,
                            datatype=pc2.PointField.FLOAT32, count=1),
             pc2.PointField(name="z", offset=8,
                            datatype=pc2.PointField.FLOAT32, count=1)],
            keep.astype(np.float32),
        )
        self.pub.publish(cloud)

        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_report > 5.0:
            self.last_report = now
            self.get_logger().info(
                "ground plane a=%.3f b=%.3f c=%.2f | kept %d of %d points"
                % (coeff[0], coeff[1], coeff[2], len(keep), len(raw))
            )


def main() -> None:
    rclpy.init()
    node = GroundFilter()
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