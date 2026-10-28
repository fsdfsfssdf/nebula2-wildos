# WildOS Minimal Simulation

This package provides the D1H navigation model and Gazebo Harmonic sensor
simulation for developing the WildOS ROS 2 data path before the real robot is
available. The current base uses a planar velocity-control proxy; the official
D1H locomotion controller remains a separate integration step.

## Launch

```bash
source ~/cfy/nebula2-wildos/scripts/wildos_env.sh
ros2 launch wildos_simulation minimal_sim.launch.py
```

## Progressive terrain

Three additional worlds increase ground roughness while preserving the hidden
target and navigation route:

```bash
# level 0 = flat baseline, 1..3 = progressively harder terrain
~/cfy/nebula2-wildos/wildos_simulation/scripts/run_terrain_level.sh 1 \
  headless:=false rviz:=true
```

The wrapper is equivalent to passing
`world_name:=terrain_0N_*.sdf` to `object_search_stage7.launch.py`.

See [TERRAIN.md](TERRAIN.md) for grades, regeneration and validation results.

Disable RViz:

```bash
ros2 launch wildos_simulation minimal_sim.launch.py rviz:=false
```

Enable the local elevation map and traversability relay:

```bash
ros2 launch wildos_simulation minimal_sim.launch.py \
  rviz:=false enable_mapping:=true
```

Enable the sparse navigation graph without semantic vision scoring:

```bash
ros2 launch wildos_simulation minimal_sim.launch.py \
  rviz:=false enable_mapping:=true enable_graph:=true
```

Add the graph planner and path follower:

```bash
ros2 launch wildos_simulation minimal_sim.launch.py \
  rviz:=false enable_mapping:=true enable_graph:=true enable_planner:=true
```

Enable ExploRFM visual graph scoring and make the planner consume the scored
graph:

```bash
ros2 launch wildos_simulation minimal_sim.launch.py \
  rviz:=false enable_mapping:=true enable_graph:=true \
  enable_visual_scoring:=true enable_planner:=true \
  planner_graph_topic:=/spot1/scored_nav_graph
```

Send a fixed goal:

```bash
ros2 topic pub --once /spot1/graph_goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: odom}, pose: {position: {x: 8.0, y: 0.0}, orientation: {w: 1.0}}}"
```

## ROS topics

```text
/clock
/spot1/cmd_vel
/spot1/odom
/spot1/imu
/spot1/points
/spot1/scan
/spot1/realsense/front/color/image_raw
/spot1/realsense/front/color/camera_info
/spot1/realsense/left/color/image_raw
/spot1/realsense/left/color/camera_info
/spot1/realsense/right/color/image_raw
/spot1/realsense/right/color/camera_info
/elevation_mapping_node/elevation_map
/spot1/local_map
/spot1/traversability_map
/spot1/navigation_cost_map
/spot1/pose_graph
/spot1/nav_graph
/spot1/nav_graph_vis
/spot1/graph_goal_pose
/spot1/graphnav_path
/spot1/goal_pose
/spot1/scored_nav_graph
/spot1/model_visualization
/spot1/score_rings
```

Drive the robot:

```bash
ros2 topic pub -r 10 /spot1/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}" --times 20
```

## Robot replacement

The shared robot model is `urdf/wildos_rover.urdf.xacro`.

- Replace the chassis, wheels, sensor links, and fixed-joint calibration with
  the real robot values.
- Keep the simulated sensor topics stable, or update the bridge and
  `config/sim_topics.yaml` together.
- The current fixed sensor transforms are read from the same URDF by
  `scripts/odom_tf.py`.
- Gazebo-only plugins are grouped separately from sensor link definitions.
- `cams_inverted` is false for this robot.

The first version intentionally uses a differential-drive base. The high-level
WildOS interface only requires odometry, TF, camera images/camera info, LiDAR,
IMU, and a velocity command topic.


## Goal hold and driving speed

These two behaviours were tuned after watching `object_search_fork.launch.py`
run end to end.

### Why the robot kept adjusting at the goal

Three separate mechanisms were fighting each other:

1. `object_goal_adapter` recomputed the standoff point from every raw object
   estimate (`standoff = min(3.5, d - 1.0)`), so as the robot got closer the
   standoff moved forward and the hold radius kept being crossed. The last run
   before the fix entered and left the hold state **31 times**.
2. The same node republished its goal every second. Each republish restarts
   the Nav2 behaviour tree and resets `SimpleProgressChecker`, so a robot
   pushing into a bank was never declared stuck; it took ~70 s per failure and
   the recovery then spun it in place at `max_rotational_vel`.
3. `pose_to_nav2` kept a single `active_goal` and ignored every newer goal
   until that one finished. Once a stale goal could not be reached, all later
   decisions from the adapter were silently dropped.

The fix: median filter the target over 9 estimates, latch the standoff point
once per target lock, publish only on real change (20 s keepalive), decide the
hold from the **distance to the target** (`standoff_distance` +/- band), and
publish `/spot1/hold_position` so Nav2 cancels the active goal and stops.
`pose_to_nav2` now preempts an active goal only when a materially different
goal has been stable for 2 s and at most once every 4 s.

Measured on the same scene:

| run | sim duration | path | mean speed | p95 speed | final state |
| --- | --- | --- | --- | --- | --- |
| before | 240.5 s | 33.57 m | 0.136 m/s | 0.463 m/s | stuck on the shoulder, 7.5 m from the house |
| after | 86.6 s | 27.27 m | 0.315 m/s | 0.813 m/s | parked at 3.8 m from the house, commands zero |

### Speed limits

Four layers cap the robot; raising only one has no effect:

| layer | file | before | now |
| --- | --- | --- | --- |
| DWB `max_vel_x` / `max_speed_xy` | `config/nav2_sim.yaml` | 0.45 | 0.80 |
| DWB `acc_lim_x` / `decel_lim_x` | `config/nav2_sim.yaml` | 2.0 | 2.5 |
| velocity smoother `max_velocity` | `config/nav2_sim.yaml` | 0.45 | 0.80 |
| velocity smoother `max_accel` | `config/nav2_sim.yaml` | 1.5 | 2.5 |
| `DiffDrive` `max_linear_velocity` | `urdf/d1h/d1h_navigation.urdf.xacro` | 1.2 | unchanged |

The smoother used to be the tightest constraint (1.5 m/s^2 against the
controller's 2.0), which is why the robot felt sluggish even below the 0.45 m/s
cap.

Wall-clock pace is a separate problem: the perception node runs at ~650 % CPU
and Gazebo's real-time factor sits around 0.2-0.3, so 90 s of simulated driving
takes several minutes. Reducing the camera rate in the URDF is the cheapest
lever if wall-clock speed matters more than perception refresh rate.


### Circling at the goal and the planner stall

A later run locked the target and then circled near it before ending up on the
shoulder. Two independent faults were responsible.

1. **Target ping-pong between neighbouring graph nodes.** After the robot
   reached the chosen node, `minimum_robot_distance` excluded it, and the next
   best node was *behind* the robot, so the adapter sent it back: 15.9 m and
   17.7 m alternated for half a minute with the heading flipping through 180
   degrees. The adapter now refuses any node that does not get at least
   `min_standoff_progress` closer to the target than where the robot already
   stands, and holds instead.
2. **`ComputePathToPose` never returned.** With `allow_unknown: true` NavFn
   expanded the unknown half of the 60x60 m rolling costmap, and the 20 s goal
   keepalive restarted the behaviour tree before the search finished. The robot
   had an empty local costmap (probe: 10000 cells, zero cost around it) and a
   zero velocity command while the BT log showed only `ComputePathToPose`
   RUNNING. `allow_unknown` is now `false` (the raytrace clearing already marks
   the corridor that has been seen) and the keepalive is 60 s.

A safety net was added as well: `pose_to_nav2` watches the odometry and cancels
the goal if the robot travels more than 3 m while staying inside a 2 m circle
for 10 s, which is what an orbit looks like. `max_vel_theta` was reduced from
1.20 to 0.90 so any residual orbit is gentler.

Verified on the fork scene: 102 s of simulated driving, 29.9 m of path, 0.82
m/s at the 95th percentile, no `Failed to make progress`, no orbit, and the
robot parks at 3.5 m from the house with zero commands and a 1.28 m LiDAR
return (the house wall right in front of it).


### Why the rover U-turned halfway

The last run that circled showed this in the log:

```
Sending Nav2 goal: x=16.20, y=2.75          # normal, ahead of the rover
Preempting the active goal; the new goal is 3.97 m away
Sending Nav2 goal: x=12.59, y=1.11          # four metres BEHIND it
```

`path_follower_node` re-anchors the graph path on the nearest graph node every
time the graph is rebuilt; when that node sits behind the rover, the first
waypoint points backwards. `pose_to_nav2` accepted it immediately, the rover
U-turned, the next replan pointed forwards again, and the pair of them produced
the circling. The fix is a no-U-turn rule: a goal whose bearing is more than
120 degrees off the current heading is ignored while the active goal is younger
than 25 s, so the rover finishes the leg it is on.

Two regressions from the same batch of changes are worth remembering:

- `allow_unknown: false` made the very first frontier goal unplannable and the
  rover never left the spawn. Frontier goals always live in unknown space, so
  the flag has to stay `true`; the search cost is bounded by shrinking the
  global costmap to 40x40 m at 0.15 m instead.
- a `Time - float` subtraction in the new U-turn check killed `pose_to_nav2`
  on its first goal, which looked exactly like "the rover stopped for no
  reason". Use `get_clock().now().nanoseconds * 1e-9` for arithmetic.

Verified afterwards: 104 s, 27.1 m of path, **zero** samples with a negative
velocity (no U-turn), no orbit, no progress failures, parked with zero commands
from 61 s on.

## 更正：到达语义已改回仓库实现（2026-10-27）

上面关于 standoff 锁点、`/spot1/hold_position`、no-U-turn 规则的段落描述的是一版
**自建**的停靠逻辑，已经全部移除。它并非论文仓库的做法。

仓库里的到达逻辑在 `visual_navigation/visual_navigation/utils/goal_navigator.py`：

- `goal_reach_radius: 2.0`，waypoint 就是感知到的物体位置本身，没有 standoff；
- 距 waypoint 小于 2 m 时打印 `Reached the last waypoint.`，清空 waypoint 列表；
- `imgfrontier_nav/nav.py` 的 `handle_goal_lock()` 在列表清空后直接 return，
  不再发布目标。

仿真侧现在的对应实现：

| 环节 | 现在的行为 |
| --- | --- |
| `object_goal_adapter.py` | 9 帧中位数滤波得到 waypoint = 物体估计位置；距离 ≤ 2.0 m 打印 `Reached the last waypoint.` 并发布 `/spot1/goal_reached`（Bool），此后不再发目标 |
| `pose_to_nav2.py` | 收到 `goal_reached=True` 后取消活动 NavigateToPose、发零速，并置 `task_finished` 锁存，忽略后续所有 `goal_pose` / `direct_goal` / `final_visual_goal` |
| `task_monitor.py` | 监听 `/spot1/goal_reached`，打印一次 `TASK COMPLETE ...`，并锁存发布 `/spot1/task_state` |
| 已删除 | `hold_position` 协议、standoff 锁点、no-U-turn 规则、绕圈看门狗之外的启发式 |

仍保留、但属于**仿真桥接层**而非论文实现的改动：`max_vel` 0.80 m/s、
全局 costmap 40x40 m @0.15 m、`allow_unknown: true`、目标抢占去抖（2 s 稳定 +
4 s 间隔）。这些是 Nav2/Gazebo 侧的必要参数，跟到达判据无关。

注意"2 m"是相对**物体估计位置**，估计本身有 1–2 m 偏差，所以机器人停下时
离真值房子的距离会更大；`TASK COMPLETE` 里打印的也是到估计值的距离。

### 到达链验证记录（fork_01，2026-10-27）

用 `object_search_fork.launch.py`（`GZ_PARTITION=forkrun23`）跑了一次，到达判定这一段
与仓库语义一致，日志恰好出现一次：

```
[spot1.object_goal_adapter]: Reached the last waypoint.
[pose_to_nav2]: Waypoint reached; cancelling Nav2 and holding still
[task_monitor]: TASK COMPLETE | arrived after 75.5 s | 1.90 m from the estimated target | pose (22.40, 13.20)
```

之后 20 s 内没有新的 `GOAL LOCK`、没有新的 `Sending Nav2 goal`，`/spot1/cmd_vel`
不再发布，odom 只漂移 1 mm，`/spot1/task_state` 锁存为 `COMPLETE`。

注意：这次是把机器人直接置于目标附近来验证"到达之后"的行为，**不是**一次完整的
越野行驶。同一场景里，从出生点到红房子的行驶段仍然会失败：

- 机器人离开岔路 A 的路面，开到草地 `(12.85, 3.94)`；
- 那儿的路面是 0.6 m 厚的板，侧壁垂直，轮子顶在侧壁上，雷达右侧回波 0.3 m；
- 控制器 20 s 内没有位移，Nav2 报 `Failed to make progress` 并 abort。

也就是说：**到达判据本身没有问题了，问题在"从路面开到房前"这一段的可通行性**。
下一步要么给红房子接一条车道（让它不再需要越野），要么给 Nav2 加可通行区域
约束，二选一。

### 雷达避障失效的两个真实原因（2026-10-28）

现象：机器人正面顶在路边石礅上，`cmd_vel` 一直发 0.8 m/s，odom 不动，20 s 后
Nav2 报 `Failed to make progress`。查下来雷达本身没问题，是点云没进代价地图：

1. **高度阈值把低矮障碍滤掉了。** 代价地图用 `min_obstacle_height: 0.10`，
   但这个值是相对 odom 的 z 原点算的，而 odom z=0 在机器人基座上——地面实际在
   odom z ≈ -0.61（草地）到 -0.40（抬高路面）之间。0.24 m 高的石礅顶点只到
   -0.32，实测 124 个点里 **0 个**通过阈值。单改阈值不可行：阈值放到 -0.45 就会
   把路面本身标成障碍。

   解决：新增 `scripts/ground_filter.py`，从点云自身拟合局部地面（按方位/距离
   分箱取低包络，再做带剔除的最小二乘），只放行"高出地面 0.12 m"的点，发布到
   `/spot1/points_obstacles`；两个代价地图改用该话题并把 `min_obstacle_height`
   设成 -10（由节点负责过滤），local 的 `origin_z` 下移到 -1.0、`z_voxels` 28。

2. **QoS 不兼容，点云一个都没送到。** ground_filter 一开始用 best-effort 发布，
   `nav2_costmap_2d` 用 reliable 订阅，日志里是：

   ```
   [ground_filter]: New subscription discovered on topic '/spot1/points_obstacles',
   requesting incompatible QoS. No messages will be sent to it. Last incompatible policy: RELIABILITY
   ```

   改成 reliable 之后，石礅所在的 1.2 m 方格从 `max=0, lethal=0` 变成
   `max=99, lethal=11`（全局代价地图），树也一直是 100。

3. 顺带发现：反复重启会留下多个 `gz sim` 实例（`kill_wildos.sh` 没被调用时），
   同一个 partition 里多个 Gazebo 会让 odom 在几个机器人状态之间跳，表现为
   "走两步退回去"。**每次重启前必须先跑 `scripts/kill_wildos.sh`。**

修完这两处后，机器人能开过原来必卡的石礅区（走到 (15.5, 6.1)），但**仍然会失败**：
目标是路外的红房子，路径要跨过抬高 0.6 m 的路基和散落的石头，机器人在路肩处
再次顶住。也就是说，剩下的是**场景可通行性**问题，不是感知问题——需要给房子接
一条车道，或让规划器只走路面。

## traversability 接入 Nav2 之后的四次翻车/卡死（2026-10-28）

把 `trav_head` / 高程图的可通行性真正接进 Nav2，比预想的曲折。记下每一步的
症状和根因，避免以后重踩。

### 1. unknown 被当成 blocked，机器人被自己的盲区围死

`elevation_relay.py` 原来用 `isfinite(traversability)` 判断"已知区域"。
实测机器人脚下 3x3 m：

```
traversability 0.01-0.36   variance 1000   is_valid 0
```

高程图对未观测区域会填一个默认低值，于是这一圈"未知"被写成 100（blocked），
两个代价地图里机器人周围 1 m 有 91-93% 是 lethal，规划器从 lethal 起点出发，
输出 `Failed to create plan from (0.10, 0.06) to (3.04, 1.75)`。

仓库自己的做法是 unknown 不下结论：`graphnav_mapper/src/mapper.cpp`

```cpp
if (collision_check_line(pos_a, pos_b, map, "unknown_sdf", r)) {
  continue;  // unknown is too close to assess traversability, leave unchanged
}
```

修正：`valid = (is_valid > 0.5) & isfinite(traversability)`，invalid 一律留在
unknown(-1)，不再猜成障碍。这条与实机部署直接相关——真机上 D1H 的雷达/相机
装在高处，脚下同样是盲区。

### 2. 有效区边界上有一圈"伪悬崖"

修完第 1 条，机器人周围变成 82% unknown，但仍然动不了。测点云分布发现
1.0-1.5 m 有一整圈 blocked 点、1.5-3.0 m 一个点都没有——这是高程图在
**有效区域边界**外推坡度造成的假悬崖，方差只有 0.005（说明它"很自信"地判错）。

`trav_obstacle_relay.py` 增加 `unknown_margin`（默认 0.6 m）：先把 unknown 膨胀，
膨胀带里的 blocked 格一律丢弃。等价于 mapper.cpp 的 `unknown_sdf` 检查。

### 3. nav2_sim.yaml 键重复，trav 反而开了 clearing

`trav:` 块里 `clearing / raytrace_range / obstacle_range` 写了两遍，YAML 后键
覆盖前键，最终 trav 是 `clearing: true`、`raytrace_max_range: 30`，而 `points`
什么范围参数都没拿到。表现是一秒五条的

```
[local_costmap.local_costmap]: Sensor origin at (0.00, 0.00 0.00) is out of map
bounds ... The costmap cannot raytrace for it.
```

修正后两个 costmap 分开配置：`points` 负责真实几何（clearing 开、范围 25 m），
`trav` 只 marking（clearing 关、raytrace 0.1 m），告警归零。

### 4. 地形不该进局部代价地图

即使修好 1-3，机器人一旦把车开到被 trav 判为 blocked 的格子上，起点就是
lethal，全局规划必然失败，然后原地转圈恢复，形成死循环。

论文里 traversability 只决定图的节点和边（节点 4 m 间距、0.5 m 余量），
局部避障是 Spot 自己的 autonomy，仓库没有开源这一层。我们复现时的等价做法：

- 局部代价地图**只用雷达**（真实几何）；
- 全局代价地图用 `points + trav`；
- `trav_obstacle_relay` 额外丢弃机器人 2.5 m 半径内的 blocked 格
  （`robot_exclude_radius`）。WildOS 的图永远允许当前节点存在，Nav2 没有这个
  概念，所以要显式补上。

### 5. 近距门限让障碍物"靠近就消失"

机器人最后顶在 `rock_trunk_011`（33 cm 高）上不动，查代价地图该点代价是 **0**。
原因链：`ground_filter.min_range = 0.80` 先丢掉近处点，代价地图又设
`obstacle_min_range = 0.65`，石头一旦进入 0.8 m 就从点云里消失，下一帧
raytrace 把它的格子清空，DWB 于是照着空气全速开过去。

修正：`ground_filter.min_range = 0.45`（车身遮挡的下限）、两个 costmap 的
`obstacle_min_range` 与 `raytrace_min_range` 都改成 0.40。

### 6. 场景侧：路缘、路基、栅栏、散石

- 路面板厚 0.6 m，抬高剖面让每条路边形成约 0.2 m 的**垂直墙**，0.18 m 的轮子
  掉到草地就再也爬不回来。`generate_fork_scene.py` 的 `PROFILE` 改为水平；
  坡度测试交给 `terrain_01..03`。
- 路肩外侧原本是一圈 18 deg 碎石缓坡（`bank_*`），但岔口处三条 ribbon 的侧向
  偏移会互相压到路面，加了 `clears_other_roads()` 过滤。
- 散石原先摆在路缘外 1.1-1.8 m，正好是规划器抄近路的位置；改为 5.0-6.5 m 外，
  只作背景。
- 栅栏从 y=±4.2 移到 ±7.0 并缩短到 8 m，末端不再楔车。

### 7. 替身底盘参数

- `DiffDrive` 补 `max_wheel_torque = 25`、`max_wheel_acceleration = 4`：22 kg、
  轮半径 0.18 m，13 deg 坡每轮需要约 4.3 N·m，默认 5 N·m 正好在失速点。
- 轮距 0.78 -> 1.10 m，车体抬高（离地约 0.25 m），配重压到轴下。
- DWB `max_vel_x = 0.55`（原 0.8 在粗糙路面容易侧翻）。

### 验证结果（forkrun40，2026-10-28）

```
[object_goal_adapter] Reached the last waypoint.
[task_monitor] TASK COMPLETE | arrived after 66.5 s | 1.91 m from the estimated target | pose (20.53, 11.23)
```

- 轨迹 28.17 m，起点到终点直线 23.59 m，**绕路比 1.19**
- 到达后 odom 不再变化，姿态 roll 0.04 deg / pitch 5.3 deg，没有翻车
- 全程没有 `Failed to make progress`，没有 `Failed to create plan`

这组数说明链路是通的，但也要清楚它证明了什么：**语义目标 → 全局路径 → 局部
避障 → 2 m 到达判定**这条链路在理想定位下成立。它不证明 D1H 的底盘能力，也不
等价于论文里 Spot 的实机结果——那一层的地形自主性官方没有开源。
