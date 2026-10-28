#!/usr/bin/env python3

import copy

import cv2
import numpy as np
import rclpy
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


def decode_layer(message: GridMap, layer_name: str) -> np.ndarray:
    index = message.layers.index(layer_name)
    array = message.data[index]
    values = np.asarray(array.data, dtype=np.float32)
    dims = array.layout.dim
    if len(dims) < 2:
        raise ValueError(f"Layer {layer_name!r} does not have two dimensions")
    first, second = dims[0], dims[1]
    if first.label == "column_index" and second.label == "row_index":
        return values.reshape((second.size, first.size), order="F")
    return values.reshape((first.size, second.size), order="C")


class ElevationRelay(Node):
    def __init__(self):
        super().__init__("elevation_relay")
        self.declare_parameter(
            "input_topic", "/elevation_mapping_node/elevation_map"
        )
        self.declare_parameter("occupancy_topic", "/spot1/local_map")
        self.declare_parameter("traversability_topic", "/spot1/traversability_map")
        self.declare_parameter(
            "navigation_cost_topic", "/spot1/navigation_cost_map"
        )
        self.declare_parameter("traversability_threshold", 0.7)

        input_topic = self.get_parameter("input_topic").value
        occupancy_topic = self.get_parameter("occupancy_topic").value
        traversability_topic = self.get_parameter("traversability_topic").value
        navigation_cost_topic = self.get_parameter(
            "navigation_cost_topic"
        ).value
        self.threshold = self.get_parameter("traversability_threshold").value

        self.occupancy_publisher = self.create_publisher(
            OccupancyGrid, occupancy_topic, 10
        )
        self.traversability_publisher = self.create_publisher(
            GridMap, traversability_topic, 10
        )
        self.navigation_cost_publisher = self.create_publisher(
            GridMap, navigation_cost_topic, 10
        )
        self.subscription = self.create_subscription(
            GridMap, input_topic, self.callback, 10
        )

    def callback(self, grid_map: GridMap) -> None:
        traversability = decode_layer(grid_map, "traversability")
        # is_valid is the only honest "observed" mask. Where the robot has no
        # returns (blind zone under/around the base, freshly initialized map)
        # elevation_mapping_cupy leaves is_valid = 0, variance = 1000 and fills
        # traversability with a small default. Treating those finite fill
        # values as "blocked" walls the robot in with its own blind spot.
        # Unknown must stay unknown: graphnav_mapper does the same thing
        # ("unknown is too close to assess traversability, leave unchanged").
        if "is_valid" in grid_map.layers:
            observed = decode_layer(grid_map, "is_valid") > 0.5
        else:
            observed = np.ones_like(traversability, dtype=bool)
        valid = observed & np.isfinite(traversability)

        rows, cols = traversability.shape
        occupancy = OccupancyGrid()
        occupancy.header = grid_map.header
        occupancy.info.resolution = float(grid_map.info.resolution)
        occupancy.info.width = cols
        occupancy.info.height = rows
        occupancy.info.origin = copy.deepcopy(grid_map.info.pose)
        occupancy.info.origin.position.x -= 0.5 * cols * occupancy.info.resolution
        occupancy.info.origin.position.y -= 0.5 * rows * occupancy.info.resolution

        occupancy_values = np.full((rows, cols), -1, dtype=np.int8)
        safe = (valid & (traversability >= self.threshold)).astype(np.uint8)
        safe = cv2.morphologyEx(
            safe,
            cv2.MORPH_CLOSE,
            np.ones((3, 3), dtype=np.uint8),
        ).astype(bool)
        occupancy_values[valid & safe] = 0
        occupancy_values[valid & ~safe] = 100
        occupancy.data = occupancy_values.flatten(order="C").tolist()

        traversability_map = GridMap()
        traversability_map.header = grid_map.header
        traversability_map.info = copy.deepcopy(grid_map.info)
        traversability_map.layers = ["traversability", "unknown", "elevation"]
        traversability_map.basic_layers = ["elevation"]

        for layer_name in ("traversability", "elevation"):
            layer_index = grid_map.layers.index(layer_name)
            traversability_map.data.append(grid_map.data[layer_index])

        elevation_index = grid_map.layers.index("elevation")
        unknown = Float32MultiArray()
        unknown.layout = grid_map.data[elevation_index].layout
        unknown.data = (~valid).astype(np.float32).flatten(order="F").tolist()
        traversability_map.data.insert(1, unknown)

        navigation_cost_map = copy.deepcopy(traversability_map)
        navigation_cost_map.data[0].data = (
            (~safe).astype(np.float32).flatten(order="F").tolist()
        )

        self.occupancy_publisher.publish(occupancy)
        self.traversability_publisher.publish(traversability_map)
        self.navigation_cost_publisher.publish(navigation_cost_map)


def main() -> None:
    rclpy.init()
    node = ElevationRelay()
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
