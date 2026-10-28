#!/usr/bin/env bash
# kill_wildos.sh -- stop every WildOS simulation process, including orphans
# left behind by earlier runs. A launch parent that dies does NOT take its
# children with it: those orphans keep publishing on the same ROS domain and
# silently corrupt pose graphs, cmd_vel and Nav2 lifecycle services.
set -u

count_nodes() {
  pgrep -fc "wildos_ws/install|/opt/ros/jazzy/lib/nav2_|velocity_smoother|parameter_bridge|ros_gz_bridge" 2>/dev/null || true
}

echo "WildOS / Nav2 processes before cleanup: $(count_nodes)"

# 1) every node installed from this workspace
pkill -9 -f "wildos_ws/install" 2>/dev/null || true
# 2) Nav2 and friends live under /opt/ros, so kill them explicitly
pkill -9 -f "/opt/ros/jazzy/lib/nav2_" 2>/dev/null || true
pkill -9 -f "/opt/ros/jazzy/lib/velocity_smoother" 2>/dev/null || true
pkill -9 -f "/opt/ros/jazzy/lib/robot_state_publisher" 2>/dev/null || true
pkill -9 -f "/opt/ros/jazzy/lib/tf2_ros" 2>/dev/null || true
# 3) gazebo / gz transport / clock bridge
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "gz-sim" 2>/dev/null || true
pkill -9 -f "parameter_bridge" 2>/dev/null || true
pkill -9 -f "ros_gz_bridge" 2>/dev/null || true
# 4) a ros2 daemon caches the old graph and must go too
ros2 daemon stop >/dev/null 2>&1 || true

sleep 2
echo "remaining WildOS / Nav2 processes: $(count_nodes)"
