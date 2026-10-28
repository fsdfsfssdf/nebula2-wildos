#!/usr/bin/env python3

import argparse
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageCapture(Node):
    def __init__(self, topic: str, output_path: Path) -> None:
        super().__init__("image_capture")
        self.bridge = CvBridge()
        self.output_path = output_path
        self.done = False
        self.create_subscription(Image, topic, self.callback, 10)

    def callback(self, message: Image) -> None:
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(self.output_path), image)
        print(f"Saved {message.width}x{message.height} image to {self.output_path}")
        self.done = True


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", required=True)
    parser.add_argument("--output", required=True, type=Path)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = ImageCapture(args.topic, args.output)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
