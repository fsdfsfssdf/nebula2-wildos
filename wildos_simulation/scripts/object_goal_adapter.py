#!/usr/bin/env python3
"""Bridge between WildOS object localisation and Nav2.

This mirrors ``visual_navigation/utils/goal_navigator.py`` from the paper
repository: the perceived object position *is* the waypoint, arrival is a
``goal_reach_radius`` (2 m) test, and once the last waypoint is reached the
node logs ``Reached the last waypoint.`` and stops publishing goals instead of
chasing the target.

Before the object is detected the node forwards visual-frontier goals through
the graph planner, which is our simulation's stand-in for the robot-side
exploration behaviour.
"""

from __future__ import annotations

import math
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool


class ObjectGoalAdapter(Node):
    def __init__(self) -> None:
        super().__init__("object_goal_adapter")
        self.declare_parameter("object_goal_topic", "imgnav_waypoint")
        self.declare_parameter("frontier_goal_topic", "frontier_exploration_goal")
        self.declare_parameter("graph_goal_topic", "task_goal_pose")
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("output_topic", "direct_goal")
        self.declare_parameter("goal_reach_radius", 2.0)
        self.declare_parameter("goal_publish_period", 1.0)
        self.declare_parameter("exploration_republish_period", 8.0)
        self.declare_parameter("goal_change_threshold", 0.25)
        self.declare_parameter("target_filter_window", 9)

        self.object_goal_topic = str(self.get_parameter("object_goal_topic").value)
        self.frontier_goal_topic = str(
            self.get_parameter("frontier_goal_topic").value
        )
        self.graph_goal_topic = str(self.get_parameter("graph_goal_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.goal_reach_radius = float(
            self.get_parameter("goal_reach_radius").value
        )
        self.goal_publish_period = float(
            self.get_parameter("goal_publish_period").value
        )
        self.exploration_republish_period = float(
            self.get_parameter("exploration_republish_period").value
        )
        self.goal_change_threshold = float(
            self.get_parameter("goal_change_threshold").value
        )
        self.filter_window = int(self.get_parameter("target_filter_window").value)

        self.estimates: deque[tuple[float, float]] = deque(maxlen=self.filter_window)
        self.frame_id = "odom"
        self.odom: Odometry | None = None
        self.frontier_goal: PoseStamped | None = None
        self.object_search_active = False
        self.arrived = False
        self.last_goal_time = 0.0
        self.last_goal_xy: tuple[float, float] | None = None
        self.last_frontier_time = 0.0
        self.last_frontier_xy: tuple[float, float] | None = None

        self.goal_pub = self.create_publisher(PoseStamped, self.output_topic, 10)
        self.graph_goal_pub = self.create_publisher(
            PoseStamped, self.graph_goal_topic, 10
        )
        self.reached_pub = self.create_publisher(Bool, "goal_reached", 10)

        self.create_subscription(
            PoseStamped, self.object_goal_topic, self.on_object_goal, 10
        )
        self.create_subscription(
            PoseStamped, self.frontier_goal_topic, self.on_frontier_goal, 10
        )
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.create_timer(0.5, self.tick)

        self.get_logger().info(
            "Object goal adapter: "
            f"{self.frontier_goal_topic} before the target, "
            f"{self.object_goal_topic} after -> {self.output_topic}; "
            f"arrival radius {self.goal_reach_radius:.1f} m"
        )

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_object_goal(self, msg: PoseStamped) -> None:
        if self.arrived:
            return
        self.frame_id = msg.header.frame_id
        self.estimates.append((msg.pose.position.x, msg.pose.position.y))
        if not self.object_search_active:
            self.object_search_active = True
            self.get_logger().info(
                "Target detected; following the perceived object position"
            )

    def on_frontier_goal(self, msg: PoseStamped) -> None:
        self.frontier_goal = msg

    def on_odom(self, msg: Odometry) -> None:
        self.odom = msg

    def waypoint(self) -> tuple[float, float]:
        xs = sorted(p[0] for p in self.estimates)
        ys = sorted(p[1] for p in self.estimates)
        return xs[len(xs) // 2], ys[len(ys) // 2]

    def tick(self) -> None:
        if self.odom is None:
            return
        if not self.object_search_active:
            self.publish_frontier_goal()
            return
        if self.arrived or not self.estimates:
            return

        robot = self.odom.pose.pose.position
        wx, wy = self.waypoint()
        distance = math.hypot(wx - robot.x, wy - robot.y)

        if distance <= self.goal_reach_radius:
            self.get_logger().warn("Reached the last waypoint.")
            self.arrived = True
            self.reached_pub.publish(Bool(data=True))
            return

        now = self.now()
        moved = (
            self.last_goal_xy is None
            or math.hypot(wx - self.last_goal_xy[0], wy - self.last_goal_xy[1])
            >= self.goal_change_threshold
        )
        if not moved and now - self.last_goal_time < self.goal_publish_period:
            return

        self.publish_pose(self.goal_pub, wx, wy, wx - robot.x, wy - robot.y)
        self.last_goal_time = now
        self.last_goal_xy = (wx, wy)
        self.get_logger().info(
            f"GOAL LOCK | waypoint ({wx:.2f}, {wy:.2f}), "
            f"{distance:.2f} m to go"
        )

    def publish_frontier_goal(self) -> None:
        if self.frontier_goal is None:
            return
        gx = self.frontier_goal.pose.position.x
        gy = self.frontier_goal.pose.position.y
        now = self.now()
        changed = (
            self.last_frontier_xy is None
            or math.hypot(gx - self.last_frontier_xy[0], gy - self.last_frontier_xy[1])
            >= self.goal_change_threshold
        )
        due = now - self.last_frontier_time >= self.exploration_republish_period
        if not changed and not due:
            return
        self.publish_pose(
            self.graph_goal_pub,
            gx,
            gy,
            self.frontier_goal.pose.position.x - self.odom.pose.pose.position.x,
            self.frontier_goal.pose.position.y - self.odom.pose.pose.position.y,
            orientation=self.frontier_goal.pose.orientation,
        )
        self.last_frontier_time = now
        self.last_frontier_xy = (gx, gy)
        self.get_logger().info(
            f"Exploring toward visual frontier ({gx:.2f}, {gy:.2f})"
        )

    def publish_pose(
        self,
        publisher,
        x: float,
        y: float,
        dx: float,
        dy: float,
        orientation=None,
    ) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        if orientation is not None:
            msg.pose.orientation = orientation
        else:
            yaw = math.atan2(dy, dx)
            msg.pose.orientation.z = math.sin(yaw * 0.5)
            msg.pose.orientation.w = math.cos(yaw * 0.5)
        publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = ObjectGoalAdapter()
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
