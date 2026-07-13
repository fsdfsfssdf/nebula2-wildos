#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import pose_graph_msgs.msg
import random


class DummyPoseGraphPublisher(Node):
    def __init__(self):
        super().__init__("dummy_pose_graph_publisher")
        self.declare_parameter("frame", "map")
        self.frame = self.get_parameter("frame").get_parameter_value().string_value
        msg = pose_graph_msgs.msg.PoseGraph()
        msg.header.frame_id = self.frame
        msg.nodes = [pose_graph_msgs.msg.Node()]
        msg.nodes[0].uuid.id = [
            random.randint(0, 255) for _ in range(len(msg.nodes[0].uuid.id))
        ]
        msg.current_node = msg.nodes[0].uuid
        self.msg = msg
        self.publisher_ = self.create_publisher(
            pose_graph_msgs.msg.PoseGraph, "pose_graph", 10
        )
        self.timer_ = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = self.msg
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)


if __name__ == "__main__":

    rclpy.init()
    node = DummyPoseGraphPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
