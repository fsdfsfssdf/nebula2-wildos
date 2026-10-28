#!/usr/bin/env python3

import math
import os
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import torch
import torch.nn.functional as F
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener
from torchvision.transforms import ToTensor
from visualization_msgs.msg import Marker

from explorfm import ExploRFMInference


class SemanticGoalNavigator(Node):
    """Convert a natural-language object query into a safe Nav2 goal."""

    def __init__(self) -> None:
        super().__init__("semantic_goal_navigator")

        default_root = os.environ.get(
            "WILDOS_ROOT", str(Path.home() / "cfy" / "nebula2-wildos")
        )
        self.declare_parameter("text_query", "red house")
        self.declare_parameter("image_topic", "/spot1/realsense/front/color/image_raw")
        self.declare_parameter(
            "camera_info_topic", "/spot1/realsense/front/color/camera_info"
        )
        self.declare_parameter("pointcloud_topic", "/spot1/points")
        self.declare_parameter("odometry_topic", "/spot1/odom")
        self.declare_parameter("costmap_topic", "/global_costmap/costmap")
        self.declare_parameter("goal_topic", "/spot1/goal_pose")
        self.declare_parameter("global_frame", "odom")
        self.declare_parameter(
            "model_version", str(Path(default_root) / "ckpts" / "c-radio_v3-b_half.pth.tar")
        )
        self.declare_parameter(
            "adaptor_checkpoint", str(Path(default_root) / "ckpts" / "siglip2")
        )
        self.declare_parameter("adaptor_version", "siglip2")
        self.declare_parameter("mask_threshold", 0.09)
        self.declare_parameter("min_mask_pixels", 40)
        self.declare_parameter("min_lidar_points", 5)
        self.declare_parameter("inference_period", 2.0)
        self.declare_parameter(
            "standoff_distances", [2.3, 2.9, 3.5, 4.5, 5.5, 6.5, 7.5]
        )
        self.declare_parameter("goal_clearance", 0.55)
        self.declare_parameter("minimum_robot_distance", 1.0)
        self.declare_parameter("allow_unknown_space", False)

        self.text_query = str(self.get_parameter("text_query").value)
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.mask_threshold = float(self.get_parameter("mask_threshold").value)
        self.min_mask_pixels = int(self.get_parameter("min_mask_pixels").value)
        self.min_lidar_points = int(self.get_parameter("min_lidar_points").value)
        self.inference_period = float(self.get_parameter("inference_period").value)
        self.standoff_distances = [
            float(value)
            for value in self.get_parameter("standoff_distances").value
        ]
        self.goal_clearance = float(self.get_parameter("goal_clearance").value)
        self.minimum_robot_distance = float(
            self.get_parameter("minimum_robot_distance").value
        )
        self.allow_unknown_space = bool(
            self.get_parameter("allow_unknown_space").value
        )

        self.bridge = CvBridge()
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_untyped = self.tf_buffer

        self.latest_costmap = None
        self.last_inference_time = 0.0
        self.mission_complete = False
        self.goal_repeats_left = 0
        self.safe_goal = None
        self.target_world = None
        self.goal_is_final = False
        self.last_staging_goal = None
        self.to_tensor = ToTensor()

        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("costmap_topic").value),
            self.costmap_callback,
            10,
        )
        self.goal_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("goal_topic").value), 10
        )
        self.final_goal_pub = self.create_publisher(
            PoseStamped, "/spot1/final_visual_goal", 10
        )
        self.mask_pub = self.create_publisher(
            Image, "/spot1/semantic_object_mask", 10
        )
        self.marker_pub = self.create_publisher(
            Marker, "/spot1/semantic_navigation_marker", 10
        )

        self.load_model()
        self.init_synchronized_subscribers()
        self.create_timer(1.0, self.publish_goal_repeatedly)

        self.get_logger().info(
            f"Semantic search ready: query='{self.text_query}', "
            f"model='{self.get_parameter('model_version').value}'"
        )

    def load_model(self) -> None:
        self.model = ExploRFMInference(
            frontier_ckpt=None,
            traversability_ckpt=None,
            model_version=str(self.get_parameter("model_version").value),
            adaptor_version=str(self.get_parameter("adaptor_version").value),
            adaptor_ckpt_path=str(self.get_parameter("adaptor_checkpoint").value),
            use_naclip=True,
            use_summary_for_spatial=True,
            radio_dim=768,
            static_scale_factor=0.75,
            model_precision="FP16",
            device="cuda",
        )
        self.text_features = self.model.forward_on_text([self.text_query])

    def init_synchronized_subscribers(self) -> None:
        image_sub = Subscriber(
            self,
            Image,
            str(self.get_parameter("image_topic").value),
            qos_profile=10,
        )
        info_sub = Subscriber(
            self,
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            qos_profile=10,
        )
        cloud_sub = Subscriber(
            self,
            PointCloud2,
            str(self.get_parameter("pointcloud_topic").value),
            qos_profile=10,
        )
        odom_sub = Subscriber(
            self,
            Odometry,
            str(self.get_parameter("odometry_topic").value),
            qos_profile=10,
        )
        self.synchronizer = ApproximateTimeSynchronizer(
            [image_sub, info_sub, cloud_sub, odom_sub],
            queue_size=5,
            slop=0.5,
        )
        self.synchronizer.registerCallback(self.synchronized_callback)

    def costmap_callback(self, message: OccupancyGrid) -> None:
        self.latest_costmap = message

    def synchronized_callback(
        self,
        image_message: Image,
        info_message: CameraInfo,
        cloud_message: PointCloud2,
        odom_message: Odometry,
    ) -> None:
        if self.mission_complete:
            return

        now_seconds = time.monotonic()
        if now_seconds - self.last_inference_time < self.inference_period:
            return
        self.last_inference_time = now_seconds

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(
                image_message, desired_encoding="rgb8"
            ).copy()
        except Exception as error:
            self.get_logger().error(f"Image conversion failed: {error}")
            return

        if self.target_world is None:
            mask = self.segment_query(rgb_image)
            mask_pixels = int(np.count_nonzero(mask))
            ys, xs = np.nonzero(mask)
            bbox = (
                (int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max()))
                if len(xs)
                else None
            )
            self.get_logger().info(
                f"Query '{self.text_query}' mask pixels: {mask_pixels}, bbox: {bbox}"
            )
            if mask_pixels < self.min_mask_pixels:
                return

            self.publish_mask(rgb_image, mask)
            self.target_world = self.localize_mask(
                info_message, cloud_message, mask
            )
            if self.target_world is None:
                return
            self.get_logger().info(
                f"Stored object target: {self.target_world.tolist()}"
            )

        robot_position = (
            float(odom_message.pose.pose.position.x),
            float(odom_message.pose.pose.position.y),
        )
        safe_goal = self.find_safe_goal(
            robot_position,
            (float(self.target_world[0]), float(self.target_world[1])),
        )
        if safe_goal is not None:
            if (
                self.safe_goal is None
                or not self.goal_is_final
                or math.hypot(
                    safe_goal[0] - self.safe_goal[0],
                    safe_goal[1] - self.safe_goal[1],
                )
                > 0.5
            ):
                self.safe_goal = safe_goal
                self.goal_is_final = True
                self.goal_repeats_left = 1
                self.publish_navigation_marker()
                self.get_logger().info(
                    "FINAL semantic goal: "
                    f"({safe_goal[0]:.2f}, {safe_goal[1]:.2f})"
                )
                self.publish_goal_repeatedly()
            self.mission_complete = True
            return

        staging_goal = self.find_staging_goal(robot_position)
        if staging_goal is None:
            self.get_logger().warning(
                "No final or staging goal is currently available"
            )
            return
        if (
            self.last_staging_goal is None
            or math.hypot(
                staging_goal[0] - self.last_staging_goal[0],
                staging_goal[1] - self.last_staging_goal[1],
            )
            > 0.75
        ):
            self.last_staging_goal = staging_goal
            self.safe_goal = staging_goal
            self.goal_is_final = False
            self.goal_repeats_left = 3
            self.get_logger().info(
                "Publishing observation/staging goal: "
                f"({staging_goal[0]:.2f}, {staging_goal[1]:.2f})"
            )
            self.publish_goal_repeatedly()

    @torch.inference_mode()
    def segment_query(self, rgb_image: np.ndarray) -> np.ndarray:
        image_tensor = self.to_tensor(rgb_image).unsqueeze(0).cuda()
        _, _, spatial_features = self.model.forward(image_tensor)
        if spatial_features.dtype != self.text_features.dtype:
            spatial_features = spatial_features.to(self.text_features.dtype)

        spatial_features = spatial_features / spatial_features.norm(
            dim=1, keepdim=True
        ).clamp_min(1e-6)
        similarity = torch.einsum(
            "qc,bchw->bqhw", self.text_features, spatial_features
        )
        similarity = F.interpolate(
            similarity,
            size=rgb_image.shape[:2],
            mode="nearest",
        )[0, 0]
        return (similarity > self.mask_threshold).cpu().numpy().astype(np.uint8)

    def localize_mask(
        self,
        info_message: CameraInfo,
        cloud_message: PointCloud2,
        mask: np.ndarray,
    ):
        camera_frame = info_message.header.frame_id.lstrip("/")
        lidar_frame = cloud_message.header.frame_id.lstrip("/")
        try:
            lidar_to_camera = self.tf_buffer.lookup_transform(
                camera_frame,
                lidar_frame,
                cloud_message.header.stamp,
                timeout=Duration(seconds=0.2),
            )
            camera_to_world = self.tf_buffer.lookup_transform(
                self.global_frame,
                camera_frame,
                cloud_message.header.stamp,
                timeout=Duration(seconds=0.2),
            )
        except Exception as error:
            self.get_logger().warning(f"TF lookup failed: {error}")
            return None

        points = point_cloud2.read_points(
            cloud_message,
            field_names=("x", "y", "z"),
            skip_nans=True,
        )
        points = np.column_stack(
            (points["x"], points["y"], points["z"])
        ).astype(np.float64)
        points = points[np.isfinite(points).all(axis=1)]
        if len(points) == 0:
            return None

        rotation, translation = self.transform_to_matrix(lidar_to_camera)
        points_camera = points @ rotation.T + translation
        points_camera = points_camera[points_camera[:, 2] > 0.1]

        intrinsic = np.asarray(info_message.k, dtype=np.float64).reshape(3, 3)
        pixels = points_camera @ intrinsic.T
        pixels = pixels[:, :2] / pixels[:, 2:3]
        pixels = np.rint(pixels).astype(np.int64)

        height, width = mask.shape
        inside = (
            (pixels[:, 0] >= 0)
            & (pixels[:, 0] < width)
            & (pixels[:, 1] >= 0)
            & (pixels[:, 1] < height)
        )
        pixels = pixels[inside]
        points_camera = points_camera[inside]
        selected = mask[pixels[:, 1], pixels[:, 0]].astype(bool)
        points_camera = points_camera[selected]
        pixels = pixels[selected]
        self.get_logger().info(
            f"LiDAR points inside semantic mask: {len(points_camera)}"
        )
        if len(points_camera) < self.min_lidar_points:
            return None

        ranges = np.linalg.norm(points_camera, axis=1)
        world_rotation, world_translation = self.transform_to_matrix(camera_to_world)
        all_world = points_camera @ world_rotation.T + world_translation
        percentiles = np.percentile(ranges, [0, 25, 50, 75, 100])
        self.get_logger().info(
            "Mask LiDAR ranges [min, p25, median, p75, max]: "
            + ", ".join(f"{value:.2f}" for value in percentiles)
            + "; world z range: "
            f"{all_world[:, 2].min():.2f} to {all_world[:, 2].max():.2f}; "
            f"pixel box: ({int(pixels[:, 0].min())},{int(pixels[:, 1].min())})"
            f"-({int(pixels[:, 0].max())},{int(pixels[:, 1].max())})"
        )

        order = np.argsort(ranges)
        clusters = []
        current = [int(order[0])]
        for index in order[1:]:
            gap_threshold = max(1.0, float(ranges[index]) * 0.08)
            if ranges[index] - ranges[current[-1]] > gap_threshold:
                clusters.append(current)
                current = []
            current.append(int(index))
        clusters.append(current)

        minimum_cluster_points = min(3, self.min_lidar_points)
        valid_clusters = [
            cluster for cluster in clusters if len(cluster) >= minimum_cluster_points
        ]
        if not valid_clusters:
            return None
        chosen = max(
            valid_clusters,
            key=lambda cluster: float(np.median(ranges[cluster])),
        )
        self.get_logger().info(
            "LiDAR depth clusters: "
            + ", ".join(
                f"{len(cluster)}@"
                f"{float(np.median(ranges[cluster])):.2f}m"
                for cluster in clusters
            )
            + f"; selected farthest valid cluster with {len(chosen)} points"
        )
        points_camera = points_camera[chosen]

        points_world = points_camera @ world_rotation.T + world_translation
        return np.median(points_world, axis=0)

    @staticmethod
    def transform_to_matrix(transform_message):
        transform = transform_message.transform
        q = transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        rotation = np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ],
            dtype=np.float64,
        )
        translation = np.array(
            [
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
            ],
            dtype=np.float64,
        )
        return rotation, translation

    def find_safe_goal(self, robot_xy, object_xy):
        if self.latest_costmap is None:
            self.get_logger().warning("Global costmap is not available yet")
            return None

        object_distance = math.hypot(
            object_xy[0] - robot_xy[0],
            object_xy[1] - robot_xy[1],
        )
        if (
            self.safe_goal is not None
            and not self.goal_is_final
            and math.hypot(
                object_xy[0] - self.safe_goal[0],
                object_xy[1] - self.safe_goal[1],
            )
            <= max(self.standoff_distances)
            and self.costmap_clearance(
                self.safe_goal, allow_unknown=True
            )
            is not None
            and self.segment_clearance(
                self.safe_goal, object_xy, end_margin=1.2
            )
            is not None
        ):
            self.get_logger().info(
                "Promoting the current forward staging goal to the final "
                "visibility stop"
            )
            return self.safe_goal
        if (
            object_distance <= max(self.standoff_distances)
            and self.costmap_clearance(robot_xy) is not None
            and self.segment_clearance(
                robot_xy, object_xy, end_margin=1.2
            )
            is not None
        ):
            self.get_logger().info(
                "Current robot pose is a safe visibility viewpoint "
                f"{object_distance:.2f} m from the target"
            )
            return robot_xy

        preferred_angle = math.atan2(
            robot_xy[1] - object_xy[1],
            robot_xy[0] - object_xy[0],
        )
        candidates = []
        for radius in self.standoff_distances:
            for angle_index in range(36):
                angle = angle_index * (2.0 * math.pi / 36.0)
                candidate = (
                    object_xy[0] + radius * math.cos(angle),
                    object_xy[1] + radius * math.sin(angle),
                )
                if math.hypot(
                    candidate[0] - robot_xy[0],
                    candidate[1] - robot_xy[1],
                ) < self.minimum_robot_distance:
                    continue
                clearance = self.costmap_clearance(candidate)
                if clearance is None:
                    continue
                segment_clearance = self.segment_clearance(robot_xy, candidate)
                if segment_clearance is None:
                    continue
                visibility_clearance = self.segment_clearance(
                    candidate, object_xy, end_margin=1.2
                )
                if visibility_clearance is None:
                    continue
                angle_error = abs(
                    math.atan2(
                        math.sin(angle - preferred_angle),
                        math.cos(angle - preferred_angle),
                    )
                )
                robot_distance = math.hypot(
                    candidate[0] - robot_xy[0],
                    candidate[1] - robot_xy[1],
                )
                score = (
                    1.5 * angle_error
                    + 0.03 * robot_distance
                    + 0.004 * clearance
                    + 0.004 * segment_clearance
                    + 0.004 * visibility_clearance
                )
                candidates.append(
                    (
                        score,
                        clearance,
                        angle_error,
                        robot_distance,
                        candidate,
                    )
                )

        if not candidates:
            return None

        candidates.sort(key=lambda item: item[0])
        best_cost, best_unknown, best_angle_error, best_distance, best = candidates[0]
        self.get_logger().info(
            f"Safe-goal search: {len(candidates)} candidates, "
            f"selected score {best_cost:.2f}, "
            f"angle_error {best_angle_error:.2f} rad, "
            f"unknown_cells {best_unknown}, "
            f"robot_distance {best_distance:.2f} m"
        )
        return best

    def find_staging_goal(self, robot_xy):
        object_xy = (
            float(self.target_world[0]),
            float(self.target_world[1]),
        )
        desired_angle = math.atan2(
            object_xy[1] - robot_xy[1],
            object_xy[0] - robot_xy[0],
        )
        candidates = []
        for distance in [10.0, 8.0, 6.0, 4.0]:
            for offset_degrees in [0, -5, 5, -10, 10, -15, 15, -20, 20, -30, 30]:
                angle = desired_angle + math.radians(offset_degrees)
                candidate = (
                    robot_xy[0] + distance * math.cos(angle),
                    robot_xy[1] + distance * math.sin(angle),
                )
                if self.costmap_clearance(candidate, allow_unknown=True) is None:
                    continue
                if self.segment_clearance(robot_xy, candidate) is None:
                    continue
                candidates.append(
                    (
                        abs(math.radians(offset_degrees)),
                        -distance,
                        candidate,
                    )
                )
        if not candidates:
            return None
        candidates.sort(key=lambda item: (item[0], item[1]))
        return candidates[0][2]

    def segment_clearance(self, start, end, end_margin=0.0):
        message = self.latest_costmap
        info = message.info
        resolution = info.resolution
        full_distance = math.hypot(end[0] - start[0], end[1] - start[1])
        if full_distance <= end_margin:
            return None
        ratio = (full_distance - end_margin) / full_distance
        checked_end = (
            start[0] + ratio * (end[0] - start[0]),
            start[1] + ratio * (end[1] - start[1]),
        )
        distance = math.hypot(
            checked_end[0] - start[0], checked_end[1] - start[1]
        )
        samples = max(2, int(math.ceil(distance / max(resolution, 0.05))))
        unknown_count = 0
        for index in range(samples + 1):
            ratio = index / samples
            x = start[0] + ratio * (checked_end[0] - start[0])
            y = start[1] + ratio * (checked_end[1] - start[1])
            col = int((x - info.origin.position.x) / resolution)
            row = int((y - info.origin.position.y) / resolution)
            if not (0 <= row < info.height and 0 <= col < info.width):
                return None
            value = message.data[row * info.width + col]
            if value >= 50:
                return None
            if value < 0:
                unknown_count += 1
        return unknown_count

    def costmap_clearance(self, point, allow_unknown=None):
        if allow_unknown is None:
            allow_unknown = self.allow_unknown_space
        message = self.latest_costmap
        info = message.info
        resolution = info.resolution
        col = int((point[0] - info.origin.position.x) / resolution)
        row = int((point[1] - info.origin.position.y) / resolution)
        if not (0 <= row < info.height and 0 <= col < info.width):
            return None

        radius_cells = max(1, int(math.ceil(self.goal_clearance / resolution)))
        unknown_count = 0
        for row_offset in range(-radius_cells, radius_cells + 1):
            for col_offset in range(-radius_cells, radius_cells + 1):
                if (
                    row_offset * row_offset + col_offset * col_offset
                    > radius_cells * radius_cells
                ):
                    continue
                target_row = row + row_offset
                target_col = col + col_offset
                if not (
                    0 <= target_row < info.height
                    and 0 <= target_col < info.width
                ):
                    return None
                value = message.data[target_row * info.width + target_col]
                if value >= 50:
                    return None
                if value < 0:
                    if not allow_unknown:
                        return None
                    unknown_count += 1
        return unknown_count

    def publish_mask(self, rgb_image: np.ndarray, mask: np.ndarray) -> None:
        visualization = rgb_image.copy()
        visualization[mask.astype(bool)] = (
            0.55 * visualization[mask.astype(bool)] + 0.45 * np.array([255, 0, 0])
        ).astype(np.uint8)
        cv2.imwrite("/tmp/semantic_source.png", cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR))
        cv2.imwrite("/tmp/semantic_mask.png", cv2.cvtColor(visualization, cv2.COLOR_RGB2BGR))
        self.mask_pub.publish(
            self.bridge.cv2_to_imgmsg(visualization, encoding="rgb8")
        )

    def publish_navigation_marker(self) -> None:
        if self.safe_goal is None or self.target_world is None:
            return
        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "semantic_navigation"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.8
        marker.color.a = 1.0
        start = PoseStamped().pose.position
        start.x, start.y, start.z = (
            float(self.safe_goal[0]),
            float(self.safe_goal[1]),
            0.1,
        )
        end = PoseStamped().pose.position
        end.x, end.y, end.z = (
            float(self.target_world[0]),
            float(self.target_world[1]),
            min(float(self.target_world[2]), 1.5),
        )
        marker.points = [start, end]
        self.marker_pub.publish(marker)

    def publish_goal_repeatedly(self) -> None:
        if self.safe_goal is None or self.goal_repeats_left <= 0:
            return
        goal = PoseStamped()
        goal.header.frame_id = self.global_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(self.safe_goal[0])
        goal.pose.position.y = float(self.safe_goal[1])
        goal.pose.position.z = 0.0
        yaw = math.atan2(
            float(self.target_world[1]) - goal.pose.position.y,
            float(self.target_world[0]) - goal.pose.position.x,
        )
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        if self.goal_is_final:
            self.final_goal_pub.publish(goal)
        self.goal_pub.publish(goal)
        self.goal_repeats_left -= 1
        kind = "final" if self.goal_is_final else "staging"
        self.get_logger().info(
            f"Published {kind} semantic goal: "
            f"x={goal.pose.position.x:.2f}, y={goal.pose.position.y:.2f}, "
            f"yaw={yaw:.2f}"
        )


def main() -> None:
    rclpy.init()
    node = SemanticGoalNavigator()
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
