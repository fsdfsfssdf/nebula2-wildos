#!/usr/bin/env python3
"""Create short Nav2 goals until multi-view object localization succeeds."""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


class ObjectSearchBootstrap(Node):
    """Move a short distance when object localization has no first goal yet."""

    def __init__(self) -> None:
        super().__init__("object_search_bootstrap")
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("object_goal_topic", "imgnav_waypoint")
        self.declare_parameter("output_topic", "goal_pose")
        self.declare_parameter("initial_delay", 8.0)
        self.declare_parameter("settle_time", 4.0)
        self.declare_parameter("motion_timeout", 25.0)
        self.declare_parameter("bootstrap_distance", 2.0)
        self.declare_parameter("arrival_tolerance", 0.45)

        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.object_goal_topic = str(
            self.get_parameter("object_goal_topic").value
        )
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.initial_delay = float(self.get_parameter("initial_delay").value)
        self.settle_time = float(self.get_parameter("settle_time").value)
        self.motion_timeout = float(
            self.get_parameter("motion_timeout").value
        )
        self.bootstrap_distance = float(
            self.get_parameter("bootstrap_distance").value
        )
        self.arrival_tolerance = float(
            self.get_parameter("arrival_tolerance").value
        )

        self.latest_odom: Odometry | None = None
        self.object_goal_seen = False
        self.started_at = time.monotonic()
        self.last_goal_time: float | None = None
        self.last_goal_xy: tuple[float, float] | None = None

        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.create_subscription(
            PoseStamped,
            self.object_goal_topic,
            self.on_object_goal,
            10,
        )
        self.goal_pub = self.create_publisher(PoseStamped, self.output_topic, 10)
        self.create_timer(0.2, self.tick)
        self.get_logger().info(
            "Object search bootstrap will move "
            f"{self.bootstrap_distance:.1f} m steps until "
            f"{self.object_goal_topic} is available"
        )

    def on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def on_object_goal(self, msg: PoseStamped) -> None:
        if self.object_goal_seen:
            return
        self.object_goal_seen = True
        self.get_logger().info(
            "Object localization produced its first goal; "
            "bootstrap movement stopped"
        )

    def tick(self) -> None:
        if self.object_goal_seen or self.latest_odom is None:
            return

        now = time.monotonic()
        if self.last_goal_time is None:
            if now - self.started_at < self.initial_delay:
                return
            self.publish_next_goal(now)
            return

        pose = self.latest_odom.pose.pose
        if self.last_goal_xy is None:
            return
        distance = math.hypot(
            pose.position.x - self.last_goal_xy[0],
            pose.position.y - self.last_goal_xy[1],
        )
        elapsed = now - self.last_goal_time
        arrived = distance <= self.arrival_tolerance
        timed_out = elapsed >= self.motion_timeout
        if not arrived and not timed_out:
            return
        if elapsed < self.settle_time:
            return
        if timed_out and not arrived:
            self.get_logger().warn(
                "Bootstrap goal was not reached; requesting one more view"
            )
        self.publish_next_goal(now)

    def publish_next_goal(self, now: float) -> None:
        pose = self.latest_odom.pose.pose
        yaw = yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.latest_odom.header.frame_id
        goal.pose.position.x = (
            pose.position.x + self.bootstrap_distance * math.cos(yaw)
        )
        goal.pose.position.y = (
            pose.position.y + self.bootstrap_distance * math.sin(yaw)
        )
        goal.pose.position.z = 0.0
        goal.pose.orientation = pose.orientation
        self.goal_pub.publish(goal)
        self.last_goal_xy = (goal.pose.position.x, goal.pose.position.y)
        self.last_goal_time = now
        self.get_logger().info(
            "Sent bootstrap goal "
            f"({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f}) "
            "to request a second camera view"
        )


def main() -> None:
    rclpy.init()
    node = ObjectSearchBootstrap()
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
