#!/usr/bin/env python3

import math
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class OdomTfPublisher(Node):
    def __init__(self):
        super().__init__("wildos_sim_odom_tf")
        self.declare_parameter("odom_topic", "/spot1/odom")
        self.declare_parameter("parent_frame", "odom")
        self.declare_parameter("child_frame", "base_link")
        self.declare_parameter("robot_description", "")

        odom_topic = self.get_parameter("odom_topic").value
        self.parent_frame = self.get_parameter("parent_frame").value
        self.child_frame = self.get_parameter("child_frame").value
        self.broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_fixed_frames(
            self.get_parameter("robot_description").value
        )
        self.subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.on_odometry,
            10,
        )

    def publish_fixed_frames(self, robot_description: str) -> None:
        if not robot_description:
            self.get_logger().error("robot_description is empty")
            return

        robot = ET.fromstring(robot_description)
        transforms = []
        for joint in robot.findall("joint"):
            if joint.attrib.get("type") != "fixed":
                continue
            parent = joint.find("parent")
            child = joint.find("child")
            if parent is None or child is None:
                continue
            origin = joint.find("origin")
            xyz = [0.0, 0.0, 0.0]
            rpy = [0.0, 0.0, 0.0]
            if origin is not None:
                xyz = [float(value) for value in origin.attrib.get("xyz", "0 0 0").split()]
                rpy = [float(value) for value in origin.attrib.get("rpy", "0 0 0").split()]

            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = parent.attrib["link"]
            transform.child_frame_id = child.attrib["link"]
            transform.transform.translation.x = xyz[0]
            transform.transform.translation.y = xyz[1]
            transform.transform.translation.z = xyz[2]

            cr, cp, cy = (math.cos(value / 2.0) for value in rpy)
            sr, sp, sy = (math.sin(value / 2.0) for value in rpy)
            transform.transform.rotation.x = sr * cp * cy - cr * sp * sy
            transform.transform.rotation.y = cr * sp * cy + sr * cp * sy
            transform.transform.rotation.z = cr * cp * sy - sr * sp * cy
            transform.transform.rotation.w = cr * cp * cy + sr * sp * sy
            transforms.append(transform)

        gazebo_odom = TransformStamped()
        gazebo_odom.header = transforms[0].header
        gazebo_odom.header.frame_id = "odom"
        gazebo_odom.child_frame_id = "wildos_rover/odom"
        gazebo_odom.transform.rotation.w = 1.0
        transforms.append(gazebo_odom)

        self.static_broadcaster.sendTransform(transforms)
        self.get_logger().info(f"Published {len(transforms)} fixed TF transforms")

    def on_odometry(self, message: Odometry) -> None:
        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = message.pose.pose.position.z
        transform.transform.rotation = message.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = OdomTfPublisher()
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
