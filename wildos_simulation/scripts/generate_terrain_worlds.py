#!/usr/bin/env python3
"""Generate the progressive Gazebo terrain suite used by WildOS."""

from __future__ import annotations

import argparse
import math
import random
from dataclasses import dataclass
from pathlib import Path
from textwrap import indent


WORLD_NAME = "wildos_minimal_field"
ROAD_START_X, ROAD_END_X, SEGMENT_LENGTH = 0.0, 36.0, 0.5


@dataclass(frozen=True)
class Level:
    level: int
    filename: str
    title: str
    description: str
    profile: tuple[tuple[float, float], ...]
    road_half_width: float
    mounds: int
    rocks: int
    rock_height: tuple[float, float]
    boulders: int


LEVELS = (
    Level(1, "terrain_01_gentle.sdf", "Gentle undulation",
          "Low grass mounds and two shallow road undulations.",
          ((0, 0), (4, 0), (7, .06), (10, 0), (14, 0), (17, .08),
           (20, 0), (36, 0)), 2.5, 30, 18, (.10, .28), 0),
    Level(2, "terrain_02_rough.sdf", "Rough shoulders and rolling grade",
          "A 0.25 m rolling grade, rough shoulders and roadside rocks.",
          ((0, 0), (4, 0), (10, .20), (14, .06), (20, .25),
           (26, .05), (36, .05)), 2.2, 52, 38, (.16, .48), 2),
    Level(3, "terrain_03_challenge.sdf", "Narrow rough traverse",
          "A 0.45 m climb, narrowing track, boulders and extra occluders.",
          ((0, 0), (4, 0), (12, .40), (16, .40), (22, .18),
           (30, .45), (36, .45)), 1.8, 76, 60, (.22, .72), 4),
)


def height_at(profile: tuple[tuple[float, float], ...], x: float) -> float:
    if x <= profile[0][0]:
        return profile[0][1]
    for (x0, z0), (x1, z1) in zip(profile, profile[1:]):
        if x <= x1:
            t = (x - x0) / (x1 - x0)
            # Cosine easing keeps the grade continuous at every crest/dip.
            # A piecewise-linear profile creates a sharp ridge that can throw
            # the narrow wheel-legged proxy into a large pitch on contact.
            ease = 0.5 - 0.5 * math.cos(math.pi * t)
            return z0 + ease * (z1 - z0)
    return profile[-1][1]


def mat(name: str, rgb: tuple[float, float, float]) -> str:
    r, g, b = rgb
    return (f'<material><ambient>{r:.3f} {g:.3f} {b:.3f} 1'
            f'</ambient><diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>'
            '<specular>0.04 0.04 0.04 1</specular></material>')


def box(name: str, pose: tuple[float, ...], size: tuple[float, ...],
        rgb: tuple[float, float, float]) -> str:
    x, y, z, roll, pitch, yaw = pose
    sx, sy, sz = size
    geom = f"<box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box>"
    return (f'<model name="{name}"><static>true</static>'
            f'<pose>{x:.4f} {y:.4f} {z:.4f} {roll:.5f} {pitch:.5f} '
            f'{yaw:.5f}</pose><link name="link">'
            f'<collision name="collision"><geometry>{geom}</geometry></collision>'
            f'<visual name="visual"><geometry>{geom}</geometry>'
            f'{mat(name + "_material", rgb)}</visual></link></model>')


def ellipsoid(name: str, x: float, y: float, z: float,
              radii: tuple[float, float, float],
              rgb: tuple[float, float, float], yaw: float = 0) -> str:
    rx, ry, rz = radii
    geom = f"<ellipsoid><radii>{rx:.3f} {ry:.3f} {rz:.3f}</radii></ellipsoid>"
    return (f'<model name="{name}"><static>true</static>'
            f'<pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.5f}</pose>'
            f'<link name="link"><collision name="collision"><geometry>{geom}'
            f'</geometry></collision><visual name="visual"><geometry>{geom}'
            f'</geometry>{mat(name + "_material", rgb)}</visual></link></model>')


def write_road_mesh(level: Level, directory: Path) -> Path:
    """Write one watertight, smooth road mesh.

    Segmenting the road into rotated boxes left small collision lips at every
    height knot.  A single triangle mesh removes those lips and keeps the
    traversability layer representative of the visible surface.
    """
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"{Path(level.filename).stem}_road.obj"
    step = 0.25
    count = int((ROAD_END_X - ROAD_START_X) / step) + 1
    half = level.road_half_width
    lines = ["# generated smooth WildOS terrain road"]
    for i in range(count):
        x = ROAD_START_X + i * step
        z = height_at(level.profile, x)
        lines.append(f"v {x:.5f} {half:.5f} {z:.5f}")
        lines.append(f"v {x:.5f} {-half:.5f} {z:.5f}")
    for i in range(count):
        x0 = ROAD_START_X + max(i - 1, 0) * step
        x1 = ROAD_START_X + min(i + 1, count - 1) * step
        dz = height_at(level.profile, x1) - height_at(level.profile, x0)
        nx, nz = -dz, (x1 - x0)
        norm = math.hypot(nx, nz)
        assert norm > 1e-9
        lines.append(f"vn {nx / norm:.5f} 0 {nz / norm:.5f}")
        lines.append(f"vn {nx / norm:.5f} 0 {nz / norm:.5f}")
    bottom = 2 * count
    for i in range(count):
        x = ROAD_START_X + i * step
        lines.append(f"v {x:.5f} {half:.5f} -0.50")
        lines.append(f"v {x:.5f} {-half:.5f} -0.50")
    for _ in range(count):
        lines.append("vn 0 0 -1")
        lines.append("vn 0 0 -1")
    for i in range(count - 1):
        a, b, c, d = 2 * i + 1, 2 * i + 2, 2 * i + 4, 2 * i + 3
        lines.append(f"f {a}//{a} {b}//{b} {c}//{c} {d}//{d}")
        bl, br = bottom + 2 * i + 1, bottom + 2 * i + 2
        cl, cr = bottom + 2 * i + 4, bottom + 2 * i + 3
        lines.append(f"f {a}//{a} {bl}//{bl} {cl}//{cl} {d}//{d}")
        lines.append(f"f {b}//{b} {c}//{c} {cr}//{cr} {br}//{br}")
        lines.append(f"f {bl}//{bl} {br}//{br} {cr}//{cr} {cl}//{cl}")
    lines.append(f"f 1//1 2//2 {bottom + 2}//{bottom + 2} "
                 f"{bottom + 1}//{bottom + 1}")
    end = 2 * (count - 1) + 1
    lines.append(f"f {end}//{end} {bottom + 2 * count}//{bottom + 2 * count} "
                 f"{bottom + 2 * count - 1}//{bottom + 2 * count - 1} "
                 f"{end + 1}//{end + 1}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def road(level: Level) -> str:
    uri = f"meshes/{Path(level.filename).stem}_road.obj"
    geom = f"<mesh><uri>{uri}</uri></mesh>"
    return ('<model name="terrain_road"><static>true</static>'
            '<link name="link"><collision name="collision"><geometry>'
            f'{geom}</geometry></collision><visual name="visual"><geometry>'
            f'{geom}</geometry>{mat("road", (.31, .29, .24))}'
            '</visual></link></model>')


def shoulders(level: Level) -> list[str]:
    out = []
    for side in (-1.0, 1.0):
        x, index = 1.0, 0
        while x < 35.0:
            x1 = min(x + 1.0, 35.0)
            z0, z1 = height_at(level.profile, x), height_at(level.profile, x1)
            pitch = -math.atan2(z1 - z0, x1 - x)
            z = .5 * (z0 + z1) - .08 * math.cos(pitch)
            y = side * (level.road_half_width + .42)
            out.append(box(f"shoulder_{'l' if side > 0 else 'r'}_{index:03d}",
                           (.5 * (x + x1), y, z, 0, pitch, 0),
                           (1.04, .56, .16), (.43, .38, .27)))
            x, index = x1, index + 1
    return out


def scatter(level: Level) -> list[str]:
    rng = random.Random(7100 + level.level)
    out = []
    for i in range(level.mounds):
        side = -1.0 if i % 2 == 0 else 1.0
        x, y = rng.uniform(1, 35), side * rng.uniform(3.5, 10)
        radius, h = rng.uniform(.45, 1.35), rng.uniform(.05, .13) + .025 * level.level
        out.append(ellipsoid(f"grass_mound_{i:03d}", x, y, h * .2,
                             (radius, radius * rng.uniform(.65, 1), h),
                             (.18 + .02 * level.level, .34, .12),
                             rng.uniform(-math.pi, math.pi)))
    for i in range(level.rocks):
        side = -1.0 if i % 2 == 0 else 1.0
        x = rng.uniform(1, 35)
        y = side * rng.uniform(level.road_half_width + .55, 8.5)
        h = rng.uniform(*level.rock_height)
        radius = h * rng.uniform(.55, 1.15)
        gray = rng.uniform(.30, .52)
        out.append(ellipsoid(f"rock_{i:03d}", x, y, h * .3,
                             (radius, radius * rng.uniform(.7, 1.2), h),
                             (gray, gray * .92, gray * .78),
                             rng.uniform(-math.pi, math.pi)))
    return out


def boulders(level: Level) -> list[str]:
    specs = ((17, 3.25, 1.15, .55, .85), (25, -3.35, 1.25, .60, .95),
             (13.5, -3.15, .95, .50, .75), (30, 3.25, 1.30, .65, 1.0))
    return [ellipsoid(f"boulder_{i:03d}", x, y, .55 * rz,
                      (rx, ry, rz), (.33, .30, .25), .4 + i)
            for i, (x, y, rx, ry, rz) in enumerate(specs[:level.boulders])]


def landmarks(level: Level) -> list[str]:
    base = height_at(level.profile, 10)
    return [
        box("hidden_target_occluder", (10, 6.5, base + 1.7, 0, 0, 0),
            (.5, 9, 5.8), (.45, .45, .45)),
        box("left_fence", (12, 4, .7, 0, 0, 0), (24, .15, 1.4),
            (.75, .55, .25)),
        box("right_fence_a", (7, -4, .7, 0, 0, 0), (10, .15, 1.4),
            (.75, .55, .25)),
        box("right_fence_b", (23, -4, .7, 0, 0, 0), (14, .15, 1.4),
            (.75, .55, .25)),
        box("dead_end_barrier", (22, 7, 1, 0, 0, 0), (.5, 7, 2),
            (.34, .34, .34)),
        box("target_house", (28, 8, 1.5, 0, 0, .4), (4, 4, 3),
            (.85, .06, .04)),
    ]


TREE = """<model name="target_tree"><static>true</static>
<pose>14 -8 2 0 0 0</pose><link name="trunk">
<collision name="collision"><geometry><cylinder><radius>0.3</radius><length>4</length></cylinder></geometry></collision>
<visual name="visual"><geometry><cylinder><radius>0.3</radius><length>4</length></cylinder></geometry>
<material><ambient>0.35 0.20 0.08 1</ambient><diffuse>0.35 0.20 0.08 1</diffuse></material></visual></link>
<link name="crown"><pose>0 0 2.2 0 0 0</pose>
<collision name="collision"><geometry><sphere><radius>1.8</radius></sphere></geometry></collision>
<visual name="visual"><geometry><sphere><radius>1.8</radius></sphere></geometry>
<material><ambient>0.10 0.45 0.10 1</ambient><diffuse>0.10 0.45 0.10 1</diffuse></material></visual></link>
<joint name="crown_joint" type="fixed"><parent>trunk</parent><child>crown</child></joint></model>"""


def render(level: Level) -> str:
    models = [road(level)] + shoulders(level) + scatter(level) + boulders(level)
    models += landmarks(level) + [TREE]
    body = "\n".join(indent(m, "    ") for m in models)
    sample_step = 0.05
    heights = [height_at(level.profile, i * sample_step)
               for i in range(int(36 / sample_step) + 1)]
    grade = max(abs(b - a) / sample_step for a, b in zip(heights, heights[1:]))
    return f"""<?xml version="1.0"?>
<!-- Generated by generate_terrain_worlds.py. Level {level.level}: {level.title}.
     {level.description} Maximum grade: {grade * 100:.2f}%.
     Hidden target and x=0..36 route match minimal_field_hidden.sdf. -->
<sdf version="1.9"><world name="{WORLD_NAME}">
<gravity>0 0 -9.8</gravity>
<physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size>
<real_time_factor>1.0</real_time_factor></physics>
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
<render_engine>ogre2</render_engine></plugin>
<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
<scene><ambient>0.8 0.8 0.8</ambient><background>0.55 0.75 0.95</background>
<grid>true</grid></scene>
<light type="directional" name="sun"><cast_shadows>true</cast_shadows>
<pose>0 0 20 0 0 0</pose><diffuse>1 1 1 1</diffuse>
<specular>0.3 0.3 0.3 1</specular><direction>-0.4 0.2 -0.9</direction></light>
<model name="ground"><static>true</static><pose>0 0 -0.03 0 0 0</pose>
<link name="link"><collision name="collision"><geometry><plane>
<normal>0 0 1</normal><size>200 200</size></plane></geometry></collision>
<visual name="visual"><geometry><plane><normal>0 0 1</normal>
<size>200 200</size></plane></geometry>{mat("ground_grass", (.32, .42, .20))}
</visual></link></model>
{body}
</world></sdf>
"""


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path,
                        default=Path(__file__).resolve().parents[1] / "worlds")
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for level in LEVELS:
        write_road_mesh(level, args.output_dir / "meshes")
        path = args.output_dir / level.filename
        path.write_text(render(level), encoding="utf-8")
        print(f"wrote {path}")


if __name__ == "__main__":
    main()
