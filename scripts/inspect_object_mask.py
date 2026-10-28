#!/usr/bin/env python3

import rclpy
import numpy as np
from object_search_msgs.msg import ObjectMaskWithTf
from rclpy.node import Node


class ObjectMaskInspector(Node):
    def __init__(self) -> None:
        super().__init__("object_mask_inspector")
        self.done = False
        self.create_subscription(
            ObjectMaskWithTf,
            "/spot1/object_mask",
            self.callback,
            10,
        )

    def callback(self, message: ObjectMaskWithTf) -> None:
        heights = [info.height for info in message.cam_infos]
        widths = [info.width for info in message.cam_infos]
        total = sum(height * width for height, width in zip(heights, widths))
        flat = np.frombuffer(message.object_mask.data, dtype=np.uint8)
        offset = 0
        print(f"mask_cameras={len(message.cam_infos)} total_pixels={total}")
        for index, (height, width) in enumerate(zip(heights, widths)):
            mask = flat[offset:offset + height * width].reshape(height, width)
            offset += height * width
            ys, xs = np.nonzero(mask)
            if len(xs) == 0:
                print(f"camera_{index}: pixels=0")
                continue
            print(
                f"camera_{index}: pixels={len(xs)} "
                f"ratio={len(xs) / (height * width):.4%} "
                f"bbox=({xs.min()},{ys.min()})-({xs.max()},{ys.max()})"
            )
        self.done = True


def main() -> None:
    rclpy.init()
    node = ObjectMaskInspector()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
