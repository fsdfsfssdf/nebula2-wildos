# 运行指标记录

每次仿真自动把轨迹落到 `outputs/metrics/<时间戳>/`，用来回答"这次跑得好不好"。

## 为什么记轨迹

成功率、绕路比、卡死、姿态、离障距离都由轨迹算出来。轨迹是原始证据，指标是派生量——
先存轨迹，就能事后改算法重算，不用重跑十分钟仿真。

## 文件结构

```
outputs/metrics/20261027_153012/
├── trajectory.csv   # 30 Hz：t,x,y,yaw,vx,wz,cmd_v,cmd_w,roll,pitch,range_min
├── meta.json        # 世界名、话题、采样数、起止时间
├── summary.json     # 指标（机器可读）
├── summary.md       # 指标（人读）
└── trajectory.png   # 手动 --plot 时生成
```

## 指标口径

| 指标 | 定义 |
| --- | --- |
| 路径长度 | 相邻里程计位姿的平面距离累加 |
| 绕路比 | 路径长度 ÷ 起点到终点直线距离；1.0 = 直线，2.0 = 走了两倍 |
| 卡死 | 3 秒窗口内指令持续下发（覆盖率 >= 90%）、速度 > 0.05，但位移 < 0.10 m 且转向 < 0.05 rad；原地转向和出发前的空转不算 |
| roll / pitch | IMU 欧拉角，度 |
| 最近障碍距离 | 全程 2D 激光最小有效距离；P05 排除偶发噪点 |

## 用法

启动时会打印落盘路径，跑完自动生成 `summary.json` 和 `summary.md`。

事后重算、加目标真值、出图：

```bash
source ~/cfy/nebula2-wildos/scripts/wildos_env.sh

python3 ~/cfy/nebula2-wildos/wildos_simulation/scripts/wildos_metrics.py \
  --csv outputs/metrics/20261027_153012/trajectory.csv \
  --target 28 8 \
  --out  outputs/metrics/20261027_153012 \
  --plot
```

`--target X Y` 是场景里目标物的真实坐标，给出后会额外算"终点到目标物距离"。
`--goal X Y` 是导航目标点，会额外算"对目标的绕路比"。

关闭记录：`ros2 launch ... enable_metrics:=false`。
指定输出目录：`metrics_output_dir:=/path/to/dir`。

## 已知局限

- 仿真里 `/spot1/odom` 是 Gazebo 真值，所以成功率和绕路比是**上界**；要评估定位退化，
  需要另外注入漂移后重跑对照。
- 姿态来自滑板底座，roll/pitch 远小于真实轮足，只能做相对比较。
- 重规划次数尚未采集（需要订阅 `/plan` 比较路径变化），列为下一版。
- 场景变多后单次 10–15 分钟，建议 headless 批量跑，用 `meta.json` 里的 `world` 字段分组。