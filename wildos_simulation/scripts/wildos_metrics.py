#!/usr/bin/env python3
"""Turn a recorded trajectory CSV into WildOS run metrics.

The definitions are intentionally simple and stated here, because these
numbers end up in tables:

* ``path_length_m``  -- sum of consecutive planar steps of the odometry pose.
* ``detour_ratio``   -- ``path_length_m`` divided by the straight-line distance
                        from the first to the last sample. 1.0 means "went
                        straight", 2.0 means "travelled twice the shortest
                        line". Only reported when the robot actually moved.
* ``stall_events``   -- windows where a velocity command was being published
                        but the robot neither moved nor turned (the failure
                        mode we hit on the tri-mesh terrain). In-place turns
                        are excluded on purpose, so they are not counted.
* ``attitude``       -- roll/pitch statistics from the IMU column, in degrees.
* ``clearance``      -- closest lidar return seen during the run, in metres.

Usage:

    python3 wildos_metrics.py --csv run/trajectory.csv
    python3 wildos_metrics.py --csv run/trajectory.csv --out run/ \
        --target 28 8 --plot
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

DEFAULT_STALL_WINDOW_S = 3.0
DEFAULT_STALL_MOVE_M = 0.10
DEFAULT_STALL_CMD = 0.05
DEFAULT_STALL_YAW_RAD = 0.05
# A window only counts if the controller was commanding for (almost) all of
# it; otherwise the idle seconds before the first goal look like a stall.
STALL_CMD_COVERAGE = 0.9


def _num(value):
    """Parse a CSV cell into a float, or None when it is empty/invalid."""
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return None
    try:
        number = float(text)
    except ValueError:
        return None
    return number if math.isfinite(number) else None


def load_rows(csv_path: Path):
    rows = []
    with Path(csv_path).open(newline="") as handle:
        for raw in csv.DictReader(handle):
            t = _num(raw.get("t"))
            x = _num(raw.get("x"))
            y = _num(raw.get("y"))
            if t is None or x is None or y is None:
                continue
            rows.append(
                {
                    "t": t,
                    "x": x,
                    "y": y,
                    "z": _num(raw.get("z")),
                    "yaw": _num(raw.get("yaw")),
                    "vx": _num(raw.get("vx")),
                    "cmd_v": _num(raw.get("cmd_v")),
                    "cmd_w": _num(raw.get("cmd_w")),
                    "roll": _num(raw.get("roll")),
                    "pitch": _num(raw.get("pitch")),
                    "range_min": _num(raw.get("range_min")),
                }
            )
    rows.sort(key=lambda row: row["t"])
    return rows


def _percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = fraction * (len(ordered) - 1)
    low = int(math.floor(position))
    high = min(low + 1, len(ordered) - 1)
    weight = position - low
    return ordered[low] * (1.0 - weight) + ordered[high] * weight


def _merge_stall_flags(rows, flags):
    events = 0
    total_s = 0.0
    longest_s = 0.0
    run_start = None
    for index, flag in enumerate(flags):
        if flag and run_start is None:
            run_start = rows[index]["t"]
        if run_start is not None and (not flag or index == len(flags) - 1):
            end = rows[index]["t"]
            span = max(0.0, end - run_start)
            events += 1
            total_s += span
            longest_s = max(longest_s, span)
            run_start = None
    return events, total_s, longest_s


def _stall_flags(rows, window_s, move_m, cmd_threshold,
                 yaw_threshold=DEFAULT_STALL_YAW_RAD):
    count = len(rows)
    if count < 3:
        return None
    flags = [False] * count
    first_time = rows[0]["t"]
    times = [row["t"] - first_time for row in rows]

    has_cmd = any(row["cmd_v"] is not None for row in rows)
    if not has_cmd:
        return None

    right = 0
    for left in range(count):
        if right < left:
            right = left
        while right < count - 1 and times[right] - times[left] < window_s:
            right += 1
        if times[right] - times[left] < window_s:
            break

        displacement = math.hypot(
            rows[right]["x"] - rows[left]["x"], rows[right]["y"] - rows[left]["y"]
        )
        window = rows[left : right + 1]
        commanded = [
            abs(row["cmd_v"] or 0.0) + abs(row["cmd_w"] or 0.0) for row in window
        ]
        if not commanded:
            continue
        coverage = sum(1 for row in window if row["cmd_v"] is not None) / len(window)
        if coverage < STALL_CMD_COVERAGE:
            continue
        mean_command = sum(commanded) / len(commanded)

        yaw_left = rows[left]["yaw"]
        yaw_right = rows[right]["yaw"]
        if yaw_left is None or yaw_right is None:
            yaw_delta = 0.0
        else:
            yaw_delta = abs(
                math.atan2(
                    math.sin(yaw_right - yaw_left), math.cos(yaw_right - yaw_left)
                )
            )

        if (
            mean_command > cmd_threshold
            and displacement < move_m
            and yaw_delta < yaw_threshold
        ):
            for index in range(left, right + 1):
                flags[index] = True
    return flags


def compute_metrics(rows, goal=None, target=None,
                    stall_window_s=DEFAULT_STALL_WINDOW_S,
                    stall_move_m=DEFAULT_STALL_MOVE_M,
                    stall_cmd=DEFAULT_STALL_CMD,
                    stall_yaw_rad=DEFAULT_STALL_YAW_RAD):
    notes = []
    if len(rows) < 2:
        raise ValueError("need at least two trajectory samples")

    path_length = 0.0
    step_speeds = []
    for previous, current in zip(rows, rows[1:]):
        step = math.hypot(current["x"] - previous["x"], current["y"] - previous["y"])
        path_length += step
        dt = current["t"] - previous["t"]
        if dt > 1e-3:
            step_speeds.append(step / dt)

    duration = rows[-1]["t"] - rows[0]["t"]
    start = rows[0]
    end = rows[-1]
    straight_line = math.hypot(end["x"] - start["x"], end["y"] - start["y"])

    detour_ratio = None
    if straight_line >= 0.5:
        detour_ratio = path_length / straight_line
    else:
        notes.append("net displacement < 0.5 m; detour ratio undefined")

    flags = _stall_flags(
        rows, stall_window_s, stall_move_m, stall_cmd, stall_yaw_rad
    )
    if flags is None:
        stall_events, stall_seconds, longest_stall = 0, 0.0, 0.0
        notes.append("no velocity command recorded; stall detection disabled")
    else:
        stall_events, stall_seconds, longest_stall = _merge_stall_flags(rows, flags)

    zs = [row["z"] for row in rows if row["z"] is not None]
    rolls = [math.degrees(row["roll"]) for row in rows if row["roll"] is not None]
    pitches = [math.degrees(row["pitch"]) for row in rows if row["pitch"] is not None]
    ranges = [row["range_min"] for row in rows if row["range_min"] is not None]

    def _rms(values):
        if not values:
            return None
        return math.sqrt(sum(value * value for value in values) / len(values))

    metrics = {
        "samples": len(rows),
        "duration_s": round(duration, 2),
        "start": [round(start["x"], 3), round(start["y"], 3),
                  round(math.degrees(start["yaw"]), 1) if start["yaw"] is not None else None],
        "end": [round(end["x"], 3), round(end["y"], 3),
                round(math.degrees(end["yaw"]), 1) if end["yaw"] is not None else None],
        "z_min_m": None if not zs else round(min(zs), 3),
        "z_max_m": None if not zs else round(max(zs), 3),
        "path_length_m": round(path_length, 2),
        "net_displacement_m": round(straight_line, 2),
        "detour_ratio": None if detour_ratio is None else round(detour_ratio, 3),
        "avg_speed_mps": round(path_length / duration, 3) if duration > 0 else None,
        "speed_p95_mps": None if not step_speeds else round(_percentile(step_speeds, 0.95), 3),
        "max_speed_mps": None if not step_speeds else round(max(step_speeds), 3),
        "stall_events": stall_events,
        "stall_seconds": round(stall_seconds, 2),
        "longest_stall_s": round(longest_stall, 2),
        "attitude": {
            "roll_rms_deg": None if _rms(rolls) is None else round(_rms(rolls), 2),
            "pitch_rms_deg": None if _rms(pitches) is None else round(_rms(pitches), 2),
            "pitch_min_deg": None if not pitches else round(min(pitches), 2),
            "pitch_max_deg": None if not pitches else round(max(pitches), 2),
        },
        "clearance": {
            "range_min_m": None if not ranges else round(min(ranges), 3),
            "range_p05_m": None if not ranges else round(_percentile(ranges, 0.05), 3),
        },
        "stall_definition": {
            "window_s": stall_window_s,
            "move_threshold_m": stall_move_m,
            "command_threshold": stall_cmd,
            "yaw_threshold_rad": stall_yaw_rad,
            "command_coverage_min": STALL_CMD_COVERAGE,
            "note": "in-place turns and pre-goal idle time are not stalls",
        },
    }

    if goal is not None:
        goal_distance = math.hypot(start["x"] - goal[0], start["y"] - goal[1])
        metrics["goal"] = {
            "xy": [goal[0], goal[1]],
            "start_to_goal_m": round(goal_distance, 2),
            "detour_ratio_vs_goal": (
                round(path_length / goal_distance, 3) if goal_distance >= 0.5 else None
            ),
            "end_to_goal_m": round(
                math.hypot(end["x"] - goal[0], end["y"] - goal[1]), 2
            ),
        }

    if target is not None:
        metrics["target_xy"] = [target[0], target[1]]
        metrics["end_to_target_m"] = round(
            math.hypot(end["x"] - target[0], end["y"] - target[1]), 2
        )

    if not ranges:
        notes.append("no lidar column; clearance not measured")
    if not rolls:
        notes.append("no IMU column; attitude not measured")
    notes.append("attitude from a skid base under-reports real body sway")
    metrics["notes"] = notes
    return metrics


def analyze_file(csv_path, goal=None, target=None, **kwargs):
    metrics = compute_metrics(load_rows(csv_path), goal=goal, target=target, **kwargs)
    metrics["csv"] = str(csv_path)
    return metrics


def _cell(value):
    if value is None:
        return "—"
    if isinstance(value, float):
        return f"{value:g}"
    return str(value)


def render_markdown(metrics, title="运行指标"):
    attitude = metrics.get("attitude", {})
    clearance = metrics.get("clearance", {})
    lines = [
        f"## {title}",
        "",
        "| 指标 | 数值 |",
        "| --- | --- |",
        f"| 时长 | {_cell(metrics.get('duration_s'))} s |",
        f"| 采样点 | {_cell(metrics.get('samples'))} |",
        f"| 高度范围 | {_cell(metrics.get('z_min_m'))} .. {_cell(metrics.get('z_max_m'))} m |",
        f"| 路径长度 | {_cell(metrics.get('path_length_m'))} m |",
        f"| 净位移 | {_cell(metrics.get('net_displacement_m'))} m |",
        f"| 绕路比 | {_cell(metrics.get('detour_ratio'))} |",
        f"| 平均速度 | {_cell(metrics.get('avg_speed_mps'))} m/s |",
        f"| 速度 P95 | {_cell(metrics.get('speed_p95_mps'))} m/s |",
        f"| 卡死次数 | {_cell(metrics.get('stall_events'))} |",
        f"| 卡死总时长 | {_cell(metrics.get('stall_seconds'))} s |",
        f"| 最长卡死 | {_cell(metrics.get('longest_stall_s'))} s |",
        f"| roll RMS | {_cell(attitude.get('roll_rms_deg'))} deg |",
        f"| pitch RMS | {_cell(attitude.get('pitch_rms_deg'))} deg |",
        f"| pitch 范围 | {_cell(attitude.get('pitch_min_deg'))} .. "
        f"{_cell(attitude.get('pitch_max_deg'))} deg |",
        f"| 最近障碍距离 | {_cell(clearance.get('range_min_m'))} m |",
        f"| 障碍距离 P05 | {_cell(clearance.get('range_p05_m'))} m |",
        f"| 起点 | {metrics.get('start')} |",
        f"| 终点 | {metrics.get('end')} |",
    ]
    if "goal" in metrics:
        goal = metrics["goal"]
        lines += [
            f"| 起点到目标 | {_cell(goal.get('start_to_goal_m'))} m |",
            f"| 绕路比（对目标） | {_cell(goal.get('detour_ratio_vs_goal'))} |",
            f"| 终点到目标 | {_cell(goal.get('end_to_goal_m'))} m |",
        ]
    if "end_to_target_m" in metrics:
        lines.append(f"| 终点到目标物 | {_cell(metrics.get('end_to_target_m'))} m |")
    notes = metrics.get("notes") or []
    if notes:
        lines += ["", "备注："] + [f"- {note}" for note in notes]
    return "\n".join(lines)


def plot_trajectory(metrics, rows, output_path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    xs = [row["x"] for row in rows]
    ys = [row["y"] for row in rows]
    figure, axis = plt.subplots(figsize=(7, 7))
    axis.plot(xs, ys, linewidth=1.5, label="trajectory")
    axis.scatter([xs[0]], [ys[0]], marker="o", color="tab:green", label="start")
    axis.scatter([xs[-1]], [ys[-1]], marker="x", color="tab:red", label="end")
    if "target_xy" in metrics:
        axis.scatter(metrics["target_xy"][0:1], metrics["target_xy"][1:2],
                     marker="*", s=180, color="tab:orange", label="target")
    if "goal" in metrics:
        axis.scatter(metrics["goal"]["xy"][0:1], metrics["goal"]["xy"][1:2],
                     marker="P", color="tab:blue", label="goal")
    axis.set_aspect("equal", adjustable="datalim")
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.set_title(
        f"path {metrics['path_length_m']:.1f} m, "
        f"detour {metrics.get('detour_ratio')}, "
        f"stalls {metrics['stall_events']}"
    )
    axis.grid(True, alpha=0.3)
    axis.legend(loc="best")
    figure.tight_layout()
    figure.savefig(output_path, dpi=140)
    plt.close(figure)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", required=True, help="trajectory.csv to score")
    parser.add_argument("--out", default=None, help="directory for summary files")
    parser.add_argument("--goal", nargs=2, type=float, metavar=("X", "Y"))
    parser.add_argument("--target", nargs=2, type=float, metavar=("X", "Y"))
    parser.add_argument("--plot", action="store_true", help="write trajectory.png")
    parser.add_argument("--stall-window", type=float, default=DEFAULT_STALL_WINDOW_S)
    parser.add_argument("--stall-move", type=float, default=DEFAULT_STALL_MOVE_M)
    parser.add_argument("--stall-cmd", type=float, default=DEFAULT_STALL_CMD)
    parser.add_argument("--stall-yaw", type=float, default=DEFAULT_STALL_YAW_RAD)
    args = parser.parse_args(argv)

    csv_path = Path(args.csv).expanduser()
    rows = load_rows(csv_path)
    metrics = compute_metrics(
        rows,
        goal=args.goal,
        target=args.target,
        stall_window_s=args.stall_window,
        stall_move_m=args.stall_move,
        stall_cmd=args.stall_cmd,
        stall_yaw_rad=args.stall_yaw,
    )
    metrics["csv"] = str(csv_path)

    if args.out:
        out_dir = Path(args.out).expanduser()
        out_dir.mkdir(parents=True, exist_ok=True)
        (out_dir / "summary.json").write_text(json.dumps(metrics, indent=2) + "\n")
        (out_dir / "summary.md").write_text(render_markdown(metrics) + "\n")
        if args.plot:
            plot_trajectory(metrics, rows, out_dir / "trajectory.png")
        print(render_markdown(metrics))
    else:
        print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()