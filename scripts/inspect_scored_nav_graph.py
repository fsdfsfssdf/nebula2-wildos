#!/usr/bin/env python3

import sys

import rclpy
from graphnav_msgs.msg import NavigationGraph
from rclpy.node import Node


class InspectScoredNavGraph(Node):
    def __init__(self) -> None:
        super().__init__("inspect_scored_nav_graph")
        self.done = False
        topic = sys.argv[1] if len(sys.argv) > 1 else "/spot1/scored_nav_graph"
        self.create_subscription(
            NavigationGraph,
            topic,
            self.callback,
            10,
        )

    def callback(self, message: NavigationGraph) -> None:
        trav_class_idx = message.trav_classes.index("default")
        frontier_nodes = [
            node
            for node in message.nodes
            if node.trav_properties[trav_class_idx].is_frontier
        ]
        scored_frontiers = []
        for node in frontier_nodes:
            score_property = next(
                (prop for prop in node.properties if prop.key == "frontier_scores"),
                None,
            )
            default_property = next(
                (prop for prop in node.properties if prop.key == "is_default_scored"),
                None,
            )
            if score_property is not None:
                scored_frontiers.append(
                    {
                        "position": (
                            round(node.pose.position.x, 2),
                            round(node.pose.position.y, 2),
                        ),
                        "scores": len(score_property.value),
                        "max_score": round(max(score_property.value), 4),
                        "default": bool(default_property.value[0])
                        if default_property is not None
                        else None,
                    }
                )

        print(f"frame_id: {message.header.frame_id}")
        print(f"nodes: {len(message.nodes)}")
        current = message.nodes[message.current_node_idx]
        print(
            "current_node: "
            f"idx={message.current_node_idx} "
            f"pos=({current.pose.position.x:.2f}, "
            f"{current.pose.position.y:.2f})"
        )
        current_edges = [
            (edge.from_idx, edge.to_idx)
            for edge in message.edges
            if message.current_node_idx in (edge.from_idx, edge.to_idx)
        ]
        print(f"current_node_edges: {current_edges}")
        print(f"frontiers: {len(frontier_nodes)}")
        print(f"frontiers_with_scores: {len(scored_frontiers)}")
        for frontier in scored_frontiers[:10]:
            print(
                f"  pos={frontier['position']} "
                f"bins={frontier['scores']} "
                f"max={frontier['max_score']} "
                f"default={frontier['default']}"
            )
        self.done = True


def main() -> None:
    rclpy.init()
    node = InspectScoredNavGraph()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
