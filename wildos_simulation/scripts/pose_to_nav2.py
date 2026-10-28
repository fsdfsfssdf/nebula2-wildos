#!/usr/bin/env python3

import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool


class PoseToNav2(Node):
    def __init__(self) -> None:
        super().__init__("pose_to_nav2")
        self.latest_goal = None
        self.active_goal = None
        self.active_goal_is_final = False
        self.final_visual_goal = None
        self.latest_odom = None
        self.last_completed_goal = None
        self.aligning = False
        self.alignment_started = 0.0
        self.action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.create_subscription(
            PoseStamped,
            "/spot1/goal_pose",
            self.goal_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            "/spot1/final_visual_goal",
            self.final_goal_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            "/spot1/odom",
            self.odom_callback,
            10,
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "/spot1/cmd_vel", 10)
        self.create_subscription(
            Bool, "/spot1/goal_reached", self.reached_callback, 10
        )
        self.create_subscription(
            PoseStamped, "/spot1/direct_goal", self.direct_goal_callback, 10
        )
        self.create_timer(0.2, self.send_goal_if_ready)
        self.create_timer(0.05, self.alignment_step)
        self.create_timer(0.5, self.check_orbit)
        self.server_ready_logged = False
        self.ready_since = None
        self.active_goal_handle = None
        self.active_goal_serial = 0
        self.pending_goal = None
        self.pending_since = None
        self.last_preempt_time = None
        self.task_finished = False
        self.odom_history = deque(maxlen=400)
        self.orbit_guard_until = 0.0
        self.direct_until = 0.0
        self.active_goal_since = 0.0
        self.startup_delay = 3.0

    def reached_callback(self, message: Bool) -> None:
        """Finish the run once the perceived object waypoint is reached.

        Upstream this is where ``goal_navigator.py`` prints
        "Reached the last waypoint." and clears its waypoint list; the robot
        side then has nothing left to track. Nav2 is still holding the last
        NavigateToPose here, so the bridge cancels it and latches.
        """
        if not message.data or self.task_finished:
            return
        self.task_finished = True
        self.latest_goal = None
        self.pending_goal = None
        if self.active_goal_handle is not None:
            self.active_goal_handle.cancel_goal_async()
        self.active_goal_handle = None
        self.active_goal = None
        self.active_goal_is_final = False
        self.active_goal_serial += 1
        self.aligning = False
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info(
            "Waypoint reached; cancelling Nav2 and holding still"
        )

    def direct_goal_callback(self, message: PoseStamped) -> None:
        """A direct waypoint wins over the graph follower."""
        if self.task_finished:
            return
        message.header.stamp = self.get_clock().now().to_msg()
        self.direct_until = self.get_clock().now().nanoseconds * 1e-9 + 8.0
        self.latest_goal = message

    def goal_callback(self, message: PoseStamped) -> None:
        if self.task_finished:
            return
        message.header.stamp = self.get_clock().now().to_msg()
        if self.get_clock().now().nanoseconds * 1e-9 < self.direct_until:
            return
        self.latest_goal = message

    def final_goal_callback(self, message: PoseStamped) -> None:
        if self.task_finished:
            return
        self.final_visual_goal = message

    def odom_callback(self, message: Odometry) -> None:
        self.latest_odom = message
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self.odom_history.append(
            (stamp, message.pose.pose.position.x, message.pose.pose.position.y)
        )

    @staticmethod
    def yaw_from_quaternion(quaternion) -> float:
        return math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
        )

    def goals_match(self, first: PoseStamped, second: PoseStamped) -> bool:
        position_error = math.hypot(
            first.pose.position.x - second.pose.position.x,
            first.pose.position.y - second.pose.position.y,
        )
        yaw_error = self.yaw_from_quaternion(
            first.pose.orientation
        ) - self.yaw_from_quaternion(second.pose.orientation)
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
        return position_error <= 0.10 and abs(yaw_error) <= 0.10

    def check_orbit(self) -> None:
        """Stop a robot that drives in circles without getting anywhere.

        DWB happily keeps a constant linear and angular command while orbiting
        a goal it cannot converge on, which is how the rover ended up on the
        shoulder. In a 10 s window a real approach covers metres; an orbit
        stays inside a small circle.
        """
        if self.task_finished:
            return
        if len(self.odom_history) < 20 or self.active_goal is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        recent = [s for s in self.odom_history if now - s[0] <= 10.0]
        if len(recent) < 20:
            return
        xs = [p[1] for p in recent]
        ys = [p[2] for p in recent]
        span = max(
            math.hypot(a - b, c - d)
            for a, c in zip(xs, ys)
            for b, d in zip(xs, ys)
        )
        if span > 2.0:
            return
        travelled = 0.0
        for first, second in zip(recent, recent[1:]):
            travelled += math.hypot(second[1] - first[1], second[2] - first[2])
        if travelled < 3.0:
            return
        self.get_logger().warning(
            "Detected a "
            f"{travelled:.1f} m orbit inside {span:.1f} m in 10 s; "
            "cancelling the goal to stop circling"
        )
        if self.active_goal_handle is not None:
            self.active_goal_handle.cancel_goal_async()
        self.active_goal_handle = None
        self.active_goal = None
        self.active_goal_is_final = False
        self.active_goal_serial += 1
        self.latest_goal = None
        self.pending_goal = None
        self.cmd_vel_pub.publish(Twist())
        self.orbit_guard_until = now + 6.0
        self.odom_history.clear()

    def send_goal_if_ready(self) -> None:
        if self.task_finished:
            return
        if self.get_clock().now().nanoseconds * 1e-9 < self.orbit_guard_until:
            return
        # A goal that never completes used to block every later goal forever:
        # whatever the adapter decided afterwards was silently overwritten.
        if self.active_goal is not None:
            if self.latest_goal is None:
                return
            gap = math.hypot(
                self.latest_goal.pose.position.x - self.active_goal.pose.position.x,
                self.latest_goal.pose.position.y - self.active_goal.pose.position.y,
            )
            if gap <= 1.5:
                self.latest_goal = None
                self.pending_goal = None
                return
            # Two nodes publish goals on this topic; only replace an active goal
            # once the new one has been stable for a while.
            now = self.get_clock().now()
            if (
                self.pending_goal is None
                or math.hypot(
                    self.latest_goal.pose.position.x - self.pending_goal.pose.position.x,
                    self.latest_goal.pose.position.y - self.pending_goal.pose.position.y,
                ) > 0.5
            ):
                self.pending_goal = self.latest_goal
                self.pending_since = now
                return
            if (now - self.pending_since).nanoseconds < 2.0e9:
                return
            if (
                self.last_preempt_time is not None
                and (now - self.last_preempt_time).nanoseconds < 4.0e9
            ):
                return
            self.get_logger().info(
                f"Preempting the active goal; the new goal is {gap:.2f} m away"
            )
            self.last_preempt_time = now
            if self.active_goal_handle is not None:
                self.active_goal_handle.cancel_goal_async()
            self.active_goal_handle = None
            self.active_goal = None
            self.active_goal_is_final = False
        if self.latest_goal is None:
            return
        if not self.action_client.server_is_ready():
            if not self.server_ready_logged:
                self.get_logger().info("Waiting for Nav2 navigate_to_pose action server...")
                self.server_ready_logged = True
            return

        if self.ready_since is None:
            self.ready_since = self.get_clock().now()
            self.get_logger().info("Nav2 is active; waiting for a fresh routed goal")
            return
        if (
            self.get_clock().now() - self.ready_since
        ).nanoseconds < self.startup_delay * 1e9:
            return

        self.server_ready_logged = False
        goal = self.latest_goal
        self.latest_goal = None
        if (
            self.last_completed_goal is not None
            and self.goals_match(goal, self.last_completed_goal)
            and self.latest_odom is not None
        ):
            position = self.latest_odom.pose.pose.position
            if math.hypot(
                position.x - goal.pose.position.x,
                position.y - goal.pose.position.y,
            ) <= 0.5:
                self.get_logger().debug(
                    "Ignoring unchanged goal already reached"
                )
                return
        self.active_goal = goal
        self.active_goal_since = self.get_clock().now().nanoseconds * 1e-9
        self.active_goal_is_final = (
            self.final_visual_goal is not None
            and self.goals_match(goal, self.final_visual_goal)
        )
        self.get_logger().info(
            "Sending Nav2 goal: "
            f"x={goal.pose.position.x:.2f}, "
            f"y={goal.pose.position.y:.2f}, "
            f"frame={goal.header.frame_id}"
        )
        self.active_goal_serial += 1
        serial = self.active_goal_serial
        future = self.action_client.send_goal_async(
            NavigateToPose.Goal(pose=goal)
        )
        future.add_done_callback(
            lambda done, s=serial: self.goal_response_callback(done, s)
        )

    def goal_response_callback(self, future, serial: int) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warning("Nav2 rejected the goal")
            if serial == self.active_goal_serial:
                self.active_goal = None
                self.active_goal_handle = None
            return
        if serial != self.active_goal_serial:
            goal_handle.cancel_goal_async()
            return
        self.active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done, s=serial: self.result_callback(done, s)
        )

    def result_callback(self, future, serial: int) -> None:
        if serial != self.active_goal_serial:
            return
        result = future.result()
        status = result.status if result is not None else None
        self.get_logger().info(f"Nav2 goal finished with status {status}")
        if status == 4:
            self.last_completed_goal = self.active_goal
        if status == 4 and self.active_goal_is_final and self.active_goal is not None:
            self.aligning = True
            self.alignment_started = time.monotonic()
            self.get_logger().info(
                "Final position reached; aligning camera toward the semantic target"
            )
        self.active_goal = None
        self.active_goal_handle = None
        self.active_goal_is_final = False

    def alignment_step(self) -> None:
        if not self.aligning or self.task_finished:
            return
        command = Twist()
        if self.latest_odom is None or self.final_visual_goal is None:
            return

        current_yaw = self.yaw_from_quaternion(
            self.latest_odom.pose.pose.orientation
        )
        desired_yaw = self.yaw_from_quaternion(
            self.final_visual_goal.pose.orientation
        )
        error = math.atan2(
            math.sin(desired_yaw - current_yaw),
            math.cos(desired_yaw - current_yaw),
        )
        if abs(error) <= 0.07 or time.monotonic() - self.alignment_started > 30.0:
            self.cmd_vel_pub.publish(command)
            self.aligning = False
            if abs(error) <= 0.07:
                self.get_logger().info(
                    f"Final visual alignment complete; yaw error {error:.3f} rad"
                )
            else:
                self.get_logger().warning(
                    f"Final visual alignment timed out; yaw error {error:.3f} rad"
                )
            return

        command.angular.z = max(-0.8, min(0.8, 2.0 * error))
        self.cmd_vel_pub.publish(command)


def main() -> None:
    rclpy.init()
    node = PoseToNav2()
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
