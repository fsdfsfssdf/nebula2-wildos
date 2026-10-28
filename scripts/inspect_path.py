#!/usr/bin/env python3

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node


class PathInspector(Node):
    def __init__(self) -> None:
        super().__init__("path_inspector")
        self.done = False
        self.create_subscription(Path, "/plan", self.callback, 10)

    def callback(self, message: Path) -> None:
        print(f"poses={len(message.poses)} frame={message.header.frame_id}")
        indices = sorted(set([0, 1, 2, 5, 10, 20, len(message.poses) - 1]))
        for index in indices:
            if 0 <= index < len(message.poses):
                position = message.poses[index].pose.position
                print(
                    f"{index}: x={position.x:.2f} "
                    f"y={position.y:.2f} z={position.z:.2f}"
                )
        self.done = True


def main() -> None:
    rclpy.init()
    node = PathInspector()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
