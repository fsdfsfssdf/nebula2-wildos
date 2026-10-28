#!/usr/bin/env python3

import math

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node


class CostmapInspector(Node):
    def __init__(self) -> None:
        super().__init__("costmap_inspector")
        self.costmap = None
        self.odom = None
        self.done = False
        self.create_subscription(
            OccupancyGrid,
            "/local_costmap/costmap",
            self.costmap_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            "/spot1/odom",
            self.odom_callback,
            10,
        )
        self.create_timer(0.5, self.inspect)

    def costmap_callback(self, message: OccupancyGrid) -> None:
        self.costmap = message

    def odom_callback(self, message: Odometry) -> None:
        self.odom = message

    def inspect(self) -> None:
        if self.done or self.costmap is None or self.odom is None:
            return
        message = self.costmap
        info = message.info
        robot = self.odom.pose.pose.position
        cos_yaw = math.cos(2.0 * math.atan2(
            message.info.origin.orientation.z,
            message.info.origin.orientation.w,
        ))
        sin_yaw = math.sin(2.0 * math.atan2(
            message.info.origin.orientation.z,
            message.info.origin.orientation.w,
        ))
        dx = robot.x - info.origin.position.x
        dy = robot.y - info.origin.position.y
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        col = int(local_x / info.resolution)
        row = int(local_y / info.resolution)

        print(
            f"frame={message.header.frame_id} "
            f"size={info.width}x{info.height} "
            f"resolution={info.resolution:.2f} "
            f"origin=({info.origin.position.x:.2f},{info.origin.position.y:.2f}) "
            f"robot_cell=({col},{row})"
        )
        for row_offset in range(-3, 4):
            values = []
            for col_offset in range(-3, 4):
                target_row = row + row_offset
                target_col = col + col_offset
                if (
                    0 <= target_row < info.height
                    and 0 <= target_col < info.width
                ):
                    index = target_row * info.width + target_col
                    values.append(f"{message.data[index]:4d}")
                else:
                    values.append(" out")
            print(" ".join(values))
        self.done = True


def main() -> None:
    rclpy.init()
    node = CostmapInspector()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
