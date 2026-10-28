#!/usr/bin/env python3
"""Record one WildOS run's trajectory and the inputs needed to score it.

Each run gets its own directory holding ``trajectory.csv``:

    t,x,y,z,yaw,vx,wz,cmd_v,cmd_w,roll,pitch,range_min

``t`` is the odometry header stamp in seconds (sim time under Gazebo). Columns
whose source never published stay empty, so the file is still usable when a
launch does not start the IMU or the lidar.

All derived numbers (path length, detour ratio, stall events, attitude,
clearance) are computed by ``wildos_metrics.py`` from this file. That keeps a
run re-scorable after the fact and keeps this node cheap enough to sit in the
control loop.
"""

from __future__ import annotations

import csv
import json
import math
import os
import signal
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan

SENSOR_QOS = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)

FIELDS = [
    "t",
    "x",
    "y",
    "z",
    "yaw",
    "vx",
    "wz",
    "cmd_v",
    "cmd_w",
    "roll",
    "pitch",
    "range_min",
]


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def roll_pitch_from_quaternion(x: float, y: float, z: float, w: float):
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    return roll, pitch


class OdomRecorder(Node):
    """Streams odometry to CSV; every other topic is a passive annotation."""

    def __init__(self) -> None:
        super().__init__("odom_recorder")

        self.declare_parameter("odom_topic", "/spot1/odom")
        self.declare_parameter("cmd_topic", "/spot1/cmd_vel")
        self.declare_parameter("imu_topic", "/spot1/imu")
        self.declare_parameter("scan_topic", "/spot1/scan")
        self.declare_parameter("output_dir", "outputs/metrics")
        self.declare_parameter("run_name", "")
        self.declare_parameter("world", "")
        self.declare_parameter("scenario", "")
        self.declare_parameter("flush_every", 200)

        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._cmd_topic = str(self.get_parameter("cmd_topic").value)
        self._imu_topic = str(self.get_parameter("imu_topic").value)
        self._scan_topic = str(self.get_parameter("scan_topic").value)
        output_dir = os.path.expanduser(str(self.get_parameter("output_dir").value))
        run_name = str(self.get_parameter("run_name").value).strip()
        self._flush_every = int(self.get_parameter("flush_every").value)
        self._world = str(self.get_parameter("world").value)
        self._scenario = str(self.get_parameter("scenario").value)

        if not run_name:
            run_name = time.strftime("%Y%m%d_%H%M%S")
        self.run_dir = Path(output_dir).expanduser().resolve() / run_name
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = self.run_dir / "trajectory.csv"

        self._csv_file = self.csv_path.open("w", newline="")
        self._writer = csv.DictWriter(self._csv_file, fieldnames=FIELDS)
        self._writer.writeheader()

        # Latest auxiliary samples; None means "never received".
        self._cmd_v = None
        self._cmd_w = None
        self._roll = None
        self._pitch = None
        self._range_min = None

        self._rows = 0
        self._path_length = 0.0
        self._prev_xy = None
        self._sim_start = None
        self._sim_end = None
        self._wall_start = time.time()
        self._closed = False

        self.create_subscription(Odometry, self._odom_topic, self._on_odom, SENSOR_QOS)
        self.create_subscription(Twist, self._cmd_topic, self._on_cmd, SENSOR_QOS)
        self.create_subscription(Imu, self._imu_topic, self._on_imu, SENSOR_QOS)
        self.create_subscription(LaserScan, self._scan_topic, self._on_scan, SENSOR_QOS)

        self.create_timer(30.0, self._report_progress)
        self._watchdog = self.create_timer(10.0, self._check_first_sample)

        self.get_logger().info(f"Recording trajectory to {self.csv_path}")
        self.get_logger().info(
            f"odom={self._odom_topic} cmd={self._cmd_topic} "
            f"imu={self._imu_topic} scan={self._scan_topic}"
        )

    # ------------------------------------------------------------------ inputs
    def _on_odom(self, msg: Odometry) -> None:
        if self._closed:
            return
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9
        twist = msg.twist.twist

        if self._prev_xy is not None:
            dx = position.x - self._prev_xy[0]
            dy = position.y - self._prev_xy[1]
            self._path_length += math.hypot(dx, dy)
        self._prev_xy = (position.x, position.y)

        self._writer.writerow(
            {
                "t": f"{t:.6f}",
                "x": f"{position.x:.6f}",
                "y": f"{position.y:.6f}",
                "z": f"{position.z:.6f}",
                "yaw": f"{yaw:.6f}",
                "vx": f"{twist.linear.x:.6f}",
                "wz": f"{twist.angular.z:.6f}",
                "cmd_v": self._fmt(self._cmd_v),
                "cmd_w": self._fmt(self._cmd_w),
                "roll": self._fmt(self._roll),
                "pitch": self._fmt(self._pitch),
                "range_min": self._fmt(self._range_min),
            }
        )

        self._rows += 1
        if self._sim_start is None:
            self._sim_start = t
        self._sim_end = t
        if self._rows % self._flush_every == 0:
            self._csv_file.flush()

    def _on_cmd(self, msg: Twist) -> None:
        self._cmd_v = msg.linear.x
        self._cmd_w = msg.angular.z

    def _on_imu(self, msg: Imu) -> None:
        q = msg.orientation
        self._roll, self._pitch = roll_pitch_from_quaternion(q.x, q.y, q.z, q.w)

    def _on_scan(self, msg: LaserScan) -> None:
        best = None
        for r in msg.ranges:
            if math.isfinite(r) and r >= msg.range_min:
                if best is None or r < best:
                    best = r
        if best is not None:
            self._range_min = best

    # ------------------------------------------------------------------ output
    @staticmethod
    def _fmt(value):
        return "" if value is None else f"{value:.6f}"

    def _report_progress(self) -> None:
        if self._rows:
            self.get_logger().info(
                f"{self._rows} samples, {self._path_length:.1f} m travelled, "
                f"file={self.csv_path}"
            )

    def _check_first_sample(self) -> None:
        if self._rows:
            self.destroy_timer(self._watchdog)
            return
        if time.time() - self._wall_start > 20.0:
            self.get_logger().warn(
                f"No odometry on {self._odom_topic} yet; check that the bridge is up."
            )

    def finalize(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self._csv_file.flush()
            self._csv_file.close()
        except Exception:  # pragma: no cover - shutdown best effort
            pass

        meta = {
            "run_name": self.run_dir.name,
            "world": self._world,
            "scenario": self._scenario,
            "topics": {
                "odom": self._odom_topic,
                "cmd": self._cmd_topic,
                "imu": self._imu_topic,
                "scan": self._scan_topic,
            },
            "samples": self._rows,
            "path_length_live_m": round(self._path_length, 3),
            "sim_start_s": self._sim_start,
            "sim_end_s": self._sim_end,
            "wall_seconds": round(time.time() - self._wall_start, 2),
            "csv": str(self.csv_path),
        }
        (self.run_dir / "meta.json").write_text(json.dumps(meta, indent=2) + "\n")

        # print(), not the ROS logger: by now the context is usually tearing
        # down and rosout would only emit "publisher's context is invalid".
        print(
            f"[odom_recorder] recorded {self._rows} samples, "
            f"{self._path_length:.1f} m -> {self.csv_path}",
            flush=True,
        )
        self._write_summary()

    def _write_summary(self) -> None:
        if self._rows < 2:
            self.get_logger().warn("Too few samples to summarise; keeping CSV only.")
            return
        try:
            sys.path.insert(0, str(Path(__file__).resolve().parent))
            from wildos_metrics import analyze_file, render_markdown

            metrics = analyze_file(self.csv_path)
            (self.run_dir / "summary.json").write_text(
                json.dumps(metrics, indent=2) + "\n"
            )
            (self.run_dir / "summary.md").write_text(render_markdown(metrics) + "\n")
            print(
                f"[odom_recorder] summary: {metrics['path_length_m']:.1f} m, "
                f"detour={metrics.get('detour_ratio')}, "
                f"stalls={metrics['stall_events']}",
                flush=True,
            )
        except Exception as exc:  # never let scoring break a run
            print(f"[odom_recorder] could not summarise: {exc}", flush=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # launch sends SIGINT to every child; a second one can land while the
        # nodes are still tearing down. Ignore it so the CSV gets closed and
        # the summary is written.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.finalize()
        try:
            node.destroy_node()
        except BaseException:
            pass
        try:
            rclpy.shutdown()
        except BaseException:
            pass


if __name__ == "__main__":
    main()