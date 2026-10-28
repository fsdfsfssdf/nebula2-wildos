#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from graphnav_msgs.msg import NavigationGraph
from nav_msgs.msg import Odometry
from rclpy.node import Node


class FrontierGoalSelector(Node):
    def __init__(self) -> None:
        super().__init__("frontier_goal_selector")
        self.graph = None
        self.odom = None
        self.last_published_position = None
        self.last_published_time = None
        self.declare_parameter("use_task_goal", False)
        self.declare_parameter("task_goal_x", 28.0)
        self.declare_parameter("task_goal_y", 0.0)
        self.declare_parameter(
            "output_topic", "/spot1/frontier_exploration_goal"
        )
        self.declare_parameter("target_distance", 8.0)
        self.declare_parameter("republish_period", 10.0)
        self.declare_parameter("selection_hold_time", 30.0)
        self.declare_parameter("frontier_reached_radius", 1.2)
        self.declare_parameter("prefer_forward", True)
        self.declare_parameter("min_forward_progress", -0.5)
        self.declare_parameter("forward_bonus_weight", 0.6)
        self.declare_parameter("max_lateral_ratio", 1.5)
        self.declare_parameter("corridor_center_y", 0.0)
        self.declare_parameter("corridor_half_width", 1.5)
        self.declare_parameter("corridor_fallback_distance", 4.0)
        self.use_task_goal = bool(
            self.get_parameter("use_task_goal").value
        )
        self.task_goal_x = float(self.get_parameter("task_goal_x").value)
        self.task_goal_y = float(self.get_parameter("task_goal_y").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.target_distance = float(
            self.get_parameter("target_distance").value
        )
        self.republish_period = float(
            self.get_parameter("republish_period").value
        )
        self.selection_hold_time = float(
            self.get_parameter("selection_hold_time").value
        )
        self.frontier_reached_radius = float(
            self.get_parameter("frontier_reached_radius").value
        )
        self.prefer_forward = bool(
            self.get_parameter("prefer_forward").value
        )
        self.min_forward_progress = float(
            self.get_parameter("min_forward_progress").value
        )
        self.forward_bonus_weight = float(
            self.get_parameter("forward_bonus_weight").value
        )
        self.max_lateral_ratio = float(
            self.get_parameter("max_lateral_ratio").value
        )
        self.corridor_center_y = float(
            self.get_parameter("corridor_center_y").value
        )
        self.corridor_half_width = float(
            self.get_parameter("corridor_half_width").value
        )
        self.corridor_fallback_distance = float(
            self.get_parameter("corridor_fallback_distance").value
        )
        self.goal_pub = self.create_publisher(
            PoseStamped,
            self.output_topic,
            10,
        )
        self.create_subscription(
            NavigationGraph,
            "/spot1/scored_nav_graph",
            self.graph_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            "/spot1/odom",
            self.odom_callback,
            10,
        )
        self.create_timer(2.0, self.publish_best_frontier)

    def graph_callback(self, message: NavigationGraph) -> None:
        self.graph = message

    def odom_callback(self, message: Odometry) -> None:
        self.odom = message

    def publish_best_frontier(self) -> None:
        if self.use_task_goal:
            self.publish_goal(
                self.task_goal_x,
                self.task_goal_y,
                0.0,
                "task goal",
            )
            return

        if self.graph is None or self.odom is None or not self.graph.nodes:
            return

        robot = self.odom.pose.pose.position
        if (
            self.last_published_position is not None
            and self.last_published_time is not None
        ):
            elapsed = time.monotonic() - self.last_published_time
            distance_to_selected = math.hypot(
                self.last_published_position[0] - robot.x,
                self.last_published_position[1] - robot.y,
            )
            if (
                distance_to_selected > self.frontier_reached_radius
                and elapsed < self.selection_hold_time
            ):
                return
            if (
                distance_to_selected <= self.frontier_reached_radius
                and elapsed < 5.0
            ):
                return

        trav_class_idx = self.graph.trav_classes.index("default")
        candidates = []
        fallback_nodes = []
        for node in self.graph.nodes:
            dx = node.pose.position.x - robot.x
            dy = node.pose.position.y - robot.y
            distance = math.hypot(dx, dy)
            in_forward_corridor = (
                dx >= self.min_forward_progress
                and abs(dy) <= self.max_lateral_ratio * max(dx, 0.0)
                and (
                    self.corridor_half_width <= 0.0
                    or abs(node.pose.position.y - self.corridor_center_y)
                    <= self.corridor_half_width
                )
            )
            if in_forward_corridor and 2.5 <= distance <= 12.0:
                fallback_nodes.append(node)

            if not node.trav_properties[trav_class_idx].is_frontier:
                continue
            score_prop = next(
                (prop for prop in node.properties if prop.key == "frontier_scores"),
                None,
            )
            default_prop = next(
                (prop for prop in node.properties if prop.key == "is_default_scored"),
                None,
            )
            if score_prop is None or not score_prop.value:
                continue
            if default_prop is not None and default_prop.value[0] > 0.5:
                continue

            if distance < 2.5 or distance > 18.0:
                continue
            if self.prefer_forward and dx < self.min_forward_progress:
                continue
            if (
                self.prefer_forward
                and abs(dy) > self.max_lateral_ratio * dx
            ):
                continue
            if (
                self.corridor_half_width > 0.0
                and abs(node.pose.position.y - self.corridor_center_y)
                > self.corridor_half_width
            ):
                continue
            max_score = max(score_prop.value)
            distance_score = 1.0 - abs(distance - self.target_distance) / 15.0
            forward_score = max(0.0, dx / (distance + 1e-6))
            candidates.append(
                (
                    max_score
                    + self.forward_bonus_weight * forward_score
                    + 0.15 * distance_score,
                    node,
                )
            )

        if not candidates:
            if fallback_nodes:
                best = max(
                    fallback_nodes,
                    key=lambda node: node.pose.position.x,
                )
                self.publish_goal(
                    best.pose.position.x,
                    best.pose.position.y,
                    best.pose.position.z,
                    "forward graph node",
                )
                return
            fallback_x = robot.x + self.corridor_fallback_distance
            self.publish_goal(
                fallback_x,
                self.corridor_center_y,
                0.0,
                "corridor fallback",
            )
            return

        _, best = max(candidates, key=lambda item: item[0])
        position = (
            best.pose.position.x,
            best.pose.position.y,
        )
        if self.last_published_position is not None:
            same_goal = math.hypot(
                position[0] - self.last_published_position[0],
                position[1] - self.last_published_position[1],
            ) < 0.75
            recently_published = (
                self.last_published_time is not None
                and time.monotonic() - self.last_published_time
                < self.republish_period
            )
            if same_goal and recently_published:
                return

        self.publish_goal(
            position[0],
            position[1],
            best.pose.position.z,
            "frontier",
        )
        self.last_published_position = position

    def publish_goal(
        self,
        x: float,
        y: float,
        z: float,
        description: str,
    ) -> None:
        if self.odom is None:
            return
        position = (x, y)

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.odom.header.frame_id
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.last_published_position = position
        self.last_published_time = time.monotonic()
        self.get_logger().info(
            f"Published {description} x={x:.2f}, y={y:.2f}"
        )

def main() -> None:
    rclpy.init()
    node = FrontierGoalSelector()
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
