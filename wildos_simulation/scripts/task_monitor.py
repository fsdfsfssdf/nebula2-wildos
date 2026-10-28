#!/usr/bin/env python3
"""Declare the end of an object-search task, once.

``goal_navigator.py`` ends the search with a single log line
("Reached the last waypoint.") and publishes no further goal; nothing in the
simulation used to translate that into a result. This node watches
``/spot1/goal_reached`` and the odometry and prints exactly one terminal line:

    TASK COMPLETE | arrived after 104.3 s | 2.00 m from the estimated target
    | pose (23.94, 8.00)

It also publishes ``/spot1/task_state`` (latched String: COMPLETE, STALLED or
TIMEOUT) so a script or a dashboard can react without parsing logs.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String


class TaskMonitor(Node):
    def __init__(self) -> None:
        super().__init__("task_monitor")
        self.declare_parameter("odom_topic", "/spot1/odom")
        self.declare_parameter("reached_topic", "/spot1/goal_reached")
        self.declare_parameter("target_topic", "/spot1/imgnav_waypoint")
        self.declare_parameter("stall_window", 20.0)
        self.declare_parameter("stall_distance", 0.30)
        self.declare_parameter("task_timeout", 600.0)

        self.stall_window = float(self.get_parameter("stall_window").value)
        self.stall_distance = float(self.get_parameter("stall_distance").value)
        self.task_timeout = float(self.get_parameter("task_timeout").value)

        latched = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state_pub = self.create_publisher(String, "/spot1/task_state", latched)

        self.odom: Odometry | None = None
        self.target: PoseStamped | None = None
        self.started = None
        self.finished = False
        self.last_stall_check = None
        self.last_stall_xy: tuple[float, float] | None = None

        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self.on_odom,
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("target_topic").value),
            self.on_target,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("reached_topic").value),
            self.on_reached,
            10,
        )
        self.create_timer(1.0, self.watch_stall)
        self.get_logger().info(
            "Task monitor armed; a successful run ends with 'TASK COMPLETE'"
        )

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_target(self, msg: PoseStamped) -> None:
        self.target = msg

    def on_odom(self, msg: Odometry) -> None:
        self.odom = msg
        if self.started is None:
            self.started = self.now()

    def on_reached(self, msg: Bool) -> None:
        if not msg.data or self.finished or self.odom is None:
            return
        self.finished = True
        elapsed = self.now() - self.started if self.started else 0.0
        pose = self.odom.pose.pose.position
        distance = ""
        if self.target is not None:
            dx = self.target.pose.position.x - pose.x
            dy = self.target.pose.position.y - pose.y
            distance = f" | {math.hypot(dx, dy):.2f} m from the estimated target"
        self.get_logger().info(
            "TASK COMPLETE | arrived after "
            f"{elapsed:.1f} s{distance} | pose ({pose.x:.2f}, {pose.y:.2f})"
        )
        self.state_pub.publish(String(data="COMPLETE"))

    def watch_stall(self) -> None:
        if self.finished or self.odom is None or self.target is None:
            return
        now = self.now()
        if now - (self.started or now) > self.task_timeout:
            self.finished = True
            pose = self.odom.pose.pose.position
            self.get_logger().warning(
                "TASK TIMEOUT | still searching after "
                f"{self.task_timeout:.0f} s at ({pose.x:.2f}, {pose.y:.2f})"
            )
            self.state_pub.publish(String(data="TIMEOUT"))
            return
        pose = self.odom.pose.pose.position
        if self.last_stall_check is None:
            self.last_stall_check = now
            self.last_stall_xy = (pose.x, pose.y)
            return
        if now - self.last_stall_check < self.stall_window:
            return
        moved = math.hypot(
            pose.x - self.last_stall_xy[0], pose.y - self.last_stall_xy[1]
        )
        if moved < self.stall_distance:
            self.get_logger().warning(
                "TASK STALLED | moved "
                f"{moved:.2f} m in {self.stall_window:.0f} s at "
                f"({pose.x:.2f}, {pose.y:.2f})"
            )
            self.state_pub.publish(String(data="STALLED"))
        self.last_stall_check = now
        self.last_stall_xy = (pose.x, pose.y)


def main() -> None:
    rclpy.init()
    node = TaskMonitor()
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
