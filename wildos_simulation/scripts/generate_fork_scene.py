#!/usr/bin/env python3
r"""Generate the first topology scene: a fork with a blocker and a hidden target.

Layout (top view, +x to the right):

                 fork_01
        target (house, hidden from spawn)
                    *
                   /
        branch A   /   (goes up-left, ends near the house)
                  /
    spawn -------+-----------> x
     (0,0)      fork(13,0)
                  \
                   \  branch B (shorter, blocked by a parked car)
                    x

The trunk and both branches share one elevation profile evaluated as z = f(x),
so every ribbon agrees at the junction and there are no collision lips there.
A building beside the trunk hides the house from the spawn point; the house
becomes visible once the robot reaches the fork. That is what makes the
"target was not visible at start" metric meaningful.

Outputs:
    worlds/fork_01.sdf
    worlds/meshes/fork_01_*.obj
    worlds/fork_01_scenario.json   (ground truth for the metrics scripts)
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from textwrap import indent

from generate_terrain_worlds import LEVELS, box, ellipsoid, height_at, mat

WORLD_NAME = "wildos_fork"
# Flat on purpose. The road is a 0.6 m thick slab on a flat ground plane, so a
# raised profile leaves a ~0.2 m vertical wall along every edge; a 0.18 m wheel
# cannot climb back onto the road and the stand-in rover got trapped beside it.
# Slope handling is covered by the terrain_01..03 worlds.
PROFILE = ((0.0, 0.0), (60.0, 0.0))
WORLD_FILE = "fork_01.sdf"


@dataclass(frozen=True)
class Ribbon:
    name: str
    origin: tuple[float, float]
    heading_deg: float
    length: float
    half_width: float
    drop: float = 0.0  # small offset to avoid z-fighting at the junction

    @property
    def heading(self) -> float:
        return math.radians(self.heading_deg)

    @property
    def direction(self) -> tuple[float, float]:
        return math.cos(self.heading), math.sin(self.heading)

    @property
    def left(self) -> tuple[float, float]:
        return -math.sin(self.heading), math.cos(self.heading)

    def point(self, distance: float, lateral: float = 0.0) -> tuple[float, float]:
        dx, dy = self.direction
        lx, ly = self.left
        return (
            self.origin[0] + dx * distance + lx * lateral,
            self.origin[1] + dy * distance + ly * lateral,
        )

    def end(self) -> tuple[float, float]:
        return self.point(self.length)


TRUNK = Ribbon("trunk", (-2.0, 0.0), 0.0, 17.0, 2.2)
BRANCH_A = Ribbon("branch_a", (13.0, 0.0), 38.0, 17.0, 1.7, drop=0.002)
BRANCH_B = Ribbon("branch_b", (13.0, 0.0), -35.0, 12.0, 1.7, drop=0.002)
RIBBONS = (TRUNK, BRANCH_A, BRANCH_B)

FORK_POINT = (13.0, 0.0)
SPAWN = (0.0, 0.0, 0.0)

SIGHT_BLOCKER = {"name": "sight_blocker", "xy": (9.0, 5.5), "size": (5.0, 5.0, 5.0)}
BACKDROP = {"name": "backdrop_block", "xy": (31.0, 16.0), "size": (6.0, 6.0, 4.5)}
OCCLUDERS = (SIGHT_BLOCKER, BACKDROP)

HOUSE_SIZE = (4.0, 4.0, 3.0)


# --------------------------------------------------------------------- geometry
def segment_hits_box(p0, p1, cx, cy, sx, sy) -> bool:
    """2-D slab test: does the segment p0-p1 cross the axis-aligned box?"""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    t0, t1 = 0.0, 1.0
    for origin, delta, low, high in (
        (p0[0], dx, cx - sx / 2, cx + sx / 2),
        (p0[1], dy, cy - sy / 2, cy + sy / 2),
    ):
        if abs(delta) < 1e-9:
            if origin < low or origin > high:
                return False
            continue
        near, far = (low - origin) / delta, (high - origin) / delta
        if near > far:
            near, far = far, near
        t0, t1 = max(t0, near), min(t1, far)
        if t0 > t1:
            return False
    return True


def visible(a, b, boxes) -> bool:
    return not any(
        segment_hits_box(a, b, item["xy"][0], item["xy"][1],
                         item["size"][0], item["size"][1])
        for item in boxes
    )


def road_clear(point, margin: float = 0.3) -> bool:
    """True when a point is outside every road ribbon (plus a margin)."""
    for ribbon in RIBBONS:
        dx, dy = point[0] - ribbon.origin[0], point[1] - ribbon.origin[1]
        along = dx * ribbon.direction[0] + dy * ribbon.direction[1]
        lateral = dx * ribbon.left[0] + dy * ribbon.left[1]
        if -0.5 <= along <= ribbon.length + 0.5:
            if abs(lateral) <= ribbon.half_width + margin:
                return False
    return True


# ------------------------------------------------------------------------ mesh
def write_ribbon_mesh(ribbon: Ribbon, directory: Path, step: float = 0.5) -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"fork_01_{ribbon.name}.obj"

    stations = int(round(ribbon.length / step)) + 1
    top, bottom = [], []
    for index in range(stations):
        distance = min(index * step, ribbon.length)
        left_xy = ribbon.point(distance, ribbon.half_width)
        right_xy = ribbon.point(distance, -ribbon.half_width)
        zl = height_at(PROFILE, left_xy[0]) - ribbon.drop
        zr = height_at(PROFILE, right_xy[0]) - ribbon.drop
        top.append(((left_xy[0], left_xy[1], zl), (right_xy[0], right_xy[1], zr)))
        bottom.append(
            (
                (left_xy[0], left_xy[1], zl - 0.6),
                (right_xy[0], right_xy[1], zr - 0.6),
            )
        )

    def normal(index: int) -> tuple[float, float, float]:
        prev = top[max(index - 1, 0)]
        nxt = top[min(index + 1, stations - 1)]
        along = tuple(nxt[0][k] - prev[0][k] for k in range(3))
        across = tuple(top[index][1][k] - top[index][0][k] for k in range(3))
        nx = across[1] * along[2] - across[2] * along[1]
        ny = across[2] * along[0] - across[0] * along[2]
        nz = across[0] * along[1] - across[1] * along[0]
        norm = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
        nx, ny, nz = nx / norm, ny / norm, nz / norm
        if nz < 0:
            nx, ny, nz = -nx, -ny, -nz
        return nx, ny, nz

    lines = [f"# generated fork scene ribbon: {ribbon.name}"]
    for index, (left_pt, right_pt) in enumerate(top):
        nx, ny, nz = normal(index)
        lines.append(f"v {left_pt[0]:.5f} {left_pt[1]:.5f} {left_pt[2]:.5f}")
        lines.append(f"v {right_pt[0]:.5f} {right_pt[1]:.5f} {right_pt[2]:.5f}")
        lines.append(f"vn {nx:.5f} {ny:.5f} {nz:.5f}")
        lines.append(f"vn {nx:.5f} {ny:.5f} {nz:.5f}")
    bottom_base = 2 * stations  # 'vn' lines do not create OBJ vertices
    for index, (left_pt, right_pt) in enumerate(bottom):
        lines.append(f"v {left_pt[0]:.5f} {left_pt[1]:.5f} {left_pt[2]:.5f}")
        lines.append(f"v {right_pt[0]:.5f} {right_pt[1]:.5f} {right_pt[2]:.5f}")
        # Bottom faces reuse the vertex id as the normal id. Without a normal
        # per bottom vertex those indices run past the end of the vn list, the
        # OBJ parse fails and the slab never gets a collision shape.
        lines.append("vn 0 0 -1")
        lines.append("vn 0 0 -1")

    for index in range(stations - 1):
        lt, rt = 2 * index + 1, 2 * index + 2
        nl, nr = lt, rt
        lt2, rt2 = 2 * index + 3, 2 * index + 4
        nl2, nr2 = lt2, rt2
        lb, rb = bottom_base + 2 * index + 1, bottom_base + 2 * index + 2
        lb2, rb2 = bottom_base + 2 * index + 3, bottom_base + 2 * index + 4
        # The winding matches generate_terrain_worlds.py exactly: Gazebo only
        # generates contacts for this trimesh when the strip is closed and the
        # faces are ordered the same way. An open shell silently lets the robot
        # fall through until the rising road pins it down.
        lines.append(f"f {lt}//{nl} {rt}//{nr} {rt2}//{nr2} {lt2}//{nl2}")
        lines.append(f"f {lb}//{lb} {rb}//{rb} {rb2}//{rb2} {lb2}//{lb2}")
        lines.append(f"f {lt}//{nl} {lb}//{nl} {lb2}//{nl2} {lt2}//{nl2}")
        lines.append(f"f {rt}//{nr} {rt2}//{nr2} {rb2}//{nr2} {rb}//{nr}")

    # the ribbon ends are open; cap them so the slab looks solid
    first_l, first_r = 1, 2
    last_l = 2 * (stations - 1) + 1
    last_r = 2 * (stations - 1) + 2
    lines.append(f"f {first_l}//{first_l} {first_r}//{first_r} "
                 f"{bottom_base + 2}//{bottom_base + 2} "
                 f"{bottom_base + 1}//{bottom_base + 1}")
    lines.append(f"f {last_l}//{last_l} {last_r}//{last_r} "
                 f"{bottom_base + 2 * stations}//{bottom_base + 2 * stations} "
                 f"{bottom_base + 2 * stations - 1}//{bottom_base + 2 * stations - 1}")

    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def road_model(ribbon: Ribbon, step: float = 0.5, thickness: float = 0.6) -> str:
    """Road slab: mesh for the eye, oriented boxes for the physics engine.

    gz-physics-dartsim logs "Mesh construction from an SDF has not been
    implemented yet for dartsim" and silently skips <mesh> inside <collision>.
    A mesh-only slab therefore has no collider at all: the robot drives under
    the rising road and gets pinned by its side wall. The visual keeps the
    mesh, collision is a strip of slightly overlapping boxes.
    """
    uri = f"meshes/fork_01_{ribbon.name}.obj"
    count = max(1, int(round(ribbon.length / step)))
    boxes = []
    for index in range(count):
        d0 = index * step
        d1 = min((index + 1) * step, ribbon.length)
        if d1 - d0 < 1e-6:
            continue
        p0 = ribbon.point(d0)
        p1 = ribbon.point(d1)
        z0 = height_at(PROFILE, p0[0]) - ribbon.drop
        z1 = height_at(PROFILE, p1[0]) - ribbon.drop
        run = math.hypot(p1[0] - p0[0], p1[1] - p0[1]) or 1e-6
        pitch = -math.atan2(z1 - z0, run)
        cz = 0.5 * (z0 + z1) - thickness / 2.0
        cx = 0.5 * (p0[0] + p1[0])
        cy = 0.5 * (p0[1] + p1[1])
        boxes.append(
            f'<collision name="seg_{index:03d}"><pose>{cx:.4f} {cy:.4f} {cz:.4f} '
            f'0 {pitch:.5f} {ribbon.heading:.5f}</pose><geometry><box>'
            f'<size>{run * 1.04:.4f} {2 * ribbon.half_width:.4f} {thickness:.4f}</size>'
            '</box></geometry></collision>'
        )
    return (f'<model name="road_{ribbon.name}"><static>true</static>'
            '<link name="link">'
            f'<visual name="visual"><geometry><mesh><uri>{uri}</uri></mesh></geometry>'
            f'{mat("road", (.31, .29, .24))}</visual>'
            + "".join(boxes)
            + '</link></model>')


# ------------------------------------------------------------------- furniture
def choose_target():
    """Pick a target spot that is hidden from spawn and visible from the fork."""
    end = BRANCH_A.end()
    lx, ly = BRANCH_A.left
    for offset in (4.5, 5.5, 6.5, 7.5, 8.5):
        for reach in (0.0, 1.5, 3.0):
            candidate = (
                end[0] + lx * offset + BRANCH_A.direction[0] * reach,
                end[1] + ly * offset + BRANCH_A.direction[1] * reach,
            )
            if not road_clear(candidate, margin=2.0):
                continue
            hidden = any(
                segment_hits_box(SPAWN[:2], candidate, item["xy"][0],
                                 item["xy"][1], item["size"][0], item["size"][1])
                for item in OCCLUDERS
            )
            if not hidden:
                continue
            if not visible(FORK_POINT, candidate, OCCLUDERS):
                continue
            return candidate
    raise SystemExit("could not place the target: adjust the occluders")


def clears_other_roads(point, own, margin: float = 1.0) -> bool:
    """True when a shoulder/bank point does not lie on another ribbon.

    Near the fork the side offsets of the trunk cross the branch slabs; without
    this check the trunk apron and bank sat on top of branch A and the rover
    climbed a lip in the middle of the road.
    """
    for ribbon in RIBBONS:
        if ribbon is own:
            continue
        dx, dy = point[0] - ribbon.origin[0], point[1] - ribbon.origin[1]
        along = dx * ribbon.direction[0] + dy * ribbon.direction[1]
        lateral = dx * ribbon.left[0] + dy * ribbon.left[1]
        if -0.5 <= along <= ribbon.length + 0.5:
            if abs(lateral) <= ribbon.half_width + margin:
                return False
    return True


def shoulders() -> list[str]:
    """Gravel apron plus a drivable bank along both edges of every ribbon.

    The ground is a flat plane and each road is a raised slab, so the apron
    used to end in a 0.2 m vertical wall. A wheeled stand-in that drifted to
    the edge rolled down that wall three times at the same shoulder, so the
    apron is now flush with the road and followed by an 18 deg bank.
    """
    models = []
    for ribbon in RIBBONS:
        distance = 0.5
        while distance < ribbon.length:
            z0 = height_at(PROFILE, ribbon.point(distance)[0])
            z1 = height_at(PROFILE, ribbon.point(min(distance + 1.0, ribbon.length))[0])
            pitch = -math.atan2(z1 - z0, 1.0)
            for side in (-1.0, 1.0):
                tag = "l" if side > 0 else "r"
                point = ribbon.point(distance, side * (ribbon.half_width + 0.28))
                if not clears_other_roads(point, ribbon):
                    continue
                models.append(
                    box(
                        f"shoulder_{ribbon.name}_{tag}_{int(distance * 2):03d}",
                        (point[0], point[1], z0 - 0.06, 0.0, pitch, ribbon.heading),
                        (2.02, 0.56, 0.12),
                        (.43, .38, .27),
                    )
                )
                bank_angle = math.radians(18.0)
                bank_w = 1.10
                offset = ribbon.half_width + 0.56 + 0.5 * bank_w * math.cos(bank_angle)
                bp = ribbon.point(distance, side * offset)
                if not clears_other_roads(bp, ribbon):
                    continue
                models.append(
                    box(
                        f"bank_{ribbon.name}_{tag}_{int(distance * 2):03d}",
                        (
                            bp[0],
                            bp[1],
                            z0 - 0.5 * bank_w * math.sin(bank_angle) - 0.06,
                            -side * bank_angle,
                            pitch,
                            ribbon.heading,
                        ),
                        (2.02, bank_w, 0.12),
                        (.36, .33, .24),
                    )
                )
            distance += 1.0
    return models


def scenery() -> list[str]:
    models = [
        box(SIGHT_BLOCKER["name"],
            (SIGHT_BLOCKER["xy"][0], SIGHT_BLOCKER["xy"][1], 2.5, 0, 0, 0.15),
            SIGHT_BLOCKER["size"], (.62, .58, .50)),
        box(BACKDROP["name"],
            (BACKDROP["xy"][0], BACKDROP["xy"][1], 2.2, 0, 0, -0.2),
            BACKDROP["size"], (.58, .55, .48)),
        box("car_blocker", (BRANCH_B.point(9.0)[0], BRANCH_B.point(9.0)[1], 0.95,
                            0, 0, math.radians(55)),
            (1.9, 4.4, 1.5), (.22, .26, .34)),
        box("dead_end_sign", (BRANCH_B.point(10.0, 2.4)[0],
                              BRANCH_B.point(10.0, 2.4)[1], 1.1, 0, 0, 0),
            (0.12, 1.2, 1.6), (.85, .72, .15)),
        box("fence_left", (4.0, 7.0, 0.7, 0, 0, 0), (8.0, 0.14, 1.4),
            (.75, .55, .25)),
        box("fence_right", (4.0, -7.0, 0.7, 0, 0, 0), (8.0, 0.14, 1.4),
            (.75, .55, .25)),
    ]
    return models


def scatter() -> list[str]:
    models = []
    index = 0
    for ribbon in RIBBONS:
        distance = 3.0
        while distance < ribbon.length:
            for side in (-1.0, 1.0):
                # Any boulder within ~6 m of a centreline sits in the strip
                # the planner crosses when it cuts the corner to the target,
                # and the wheeled stand-in grinds to a halt against it. Scatter
                # is therefore kept well outside the drivable corridor and only
                # serves as scenery.
                lateral = side * (ribbon.half_width + 5.0 + 1.5 * ((index * 7) % 5) / 4)
                point = ribbon.point(distance, lateral)
                rock = ellipsoid(
                    f"rock_{ribbon.name}_{index:03d}", point[0], point[1], 0.10,
                    (0.28 + 0.05 * (index % 3), 0.24, 0.14 + 0.03 * (index % 4)),
                    (.42, .39, .33), 0.3 * index,
                )
                models.append(rock)
                index += 1
            distance += 2.5
    return models


def tree(x: float, y: float, name: str) -> str:
    return f"""<model name="{name}"><static>true</static>
<pose>{x:.3f} {y:.3f} 2 0 0 0</pose><link name="trunk">
<collision name="collision"><geometry><cylinder><radius>0.3</radius><length>4</length></cylinder></geometry></collision>
<visual name="visual"><geometry><cylinder><radius>0.3</radius><length>4</length></cylinder></geometry>
<material><ambient>0.35 0.20 0.08 1</ambient><diffuse>0.35 0.20 0.08 1</diffuse></material></visual></link>
<link name="crown"><pose>0 0 2.4 0 0 0</pose>
<collision name="collision"><geometry><sphere><radius>1.9</radius></sphere></geometry></collision>
<visual name="visual"><geometry><sphere><radius>1.9</radius></sphere></geometry>
<material><ambient>0.10 0.45 0.10 1</ambient><diffuse>0.10 0.45 0.10 1</diffuse></material></visual></link>
<joint name="crown_joint" type="fixed"><parent>trunk</parent><child>crown</child></joint></model>"""


def render(target) -> str:
    models = [road_model(ribbon) for ribbon in RIBBONS]
    models += shoulders()
    models += scenery()
    models += scatter()
    models += [tree(4.0, 8.0, "tree_left"), tree(16.0, -9.0, "tree_right"),
               tree(24.0, 17.0, "tree_back")]
    tx, ty = target
    models.append(
        box("target_house", (tx, ty, 1.5, 0, 0, 0.35), HOUSE_SIZE, (.85, .06, .04))
    )
    body = "\n".join(indent(model, "    ") for model in models)
    return f"""<?xml version="1.0"?>
<!-- Generated by generate_fork_scene.py: fork_01.
     Trunk -> fork at (13, 0); branch A leads toward the hidden target,
     branch B is blocked by a parked car. Target at ({tx:.2f}, {ty:.2f}).
     Do not edit by hand; rerun the generator instead. -->
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


def scenario(target) -> dict:
    tx, ty = target
    return {
        "name": "fork_01",
        "world": WORLD_FILE,
        "seed": 11,
        "spawn": {"x": SPAWN[0], "y": SPAWN[1], "yaw": SPAWN[2]},
        "target": {"type": "red_house", "xy": [round(tx, 3), round(ty, 3)],
                   "size": list(HOUSE_SIZE)},
        "hidden_from_spawn": True,
        "visible_from_fork": True,
        "fork_point": list(FORK_POINT),
        "ribbons": [
            {"name": r.name, "origin": list(r.origin),
             "heading_deg": r.heading_deg, "length_m": r.length,
             "half_width_m": r.half_width, "end": [round(v, 3) for v in r.end()]}
            for r in RIBBONS
        ],
        "blocker": {"type": "parked_car",
                    "xy": [round(v, 3) for v in BRANCH_B.point(9.0)]},
        "occluders": [
            {"name": item["name"], "xy": list(item["xy"]), "size": list(item["size"])}
            for item in OCCLUDERS
        ],
        "straight_line_spawn_to_target_m": round(math.hypot(tx, ty), 3),
        "expected_behaviour": (
            "drive to the fork, prefer branch A (target direction), "
            "ignore branch B which ends at a parked car"
        ),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path,
                        default=Path(__file__).resolve().parents[1] / "worlds")
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    target = choose_target()
    for ribbon in RIBBONS:
        write_ribbon_mesh(ribbon, args.output_dir / "meshes")
    world_path = args.output_dir / WORLD_FILE
    world_path.write_text(render(target), encoding="utf-8")
    scenario_path = args.output_dir / "fork_01_scenario.json"
    data = scenario(target)
    scenario_path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")

    print(f"wrote {world_path}")
    print(f"wrote {scenario_path}")
    print(f"target at ({target[0]:.2f}, {target[1]:.2f}), "
          f"hidden from spawn: {not visible(SPAWN[:2], target, OCCLUDERS)}, "
          f"visible from fork: {visible(FORK_POINT, target, OCCLUDERS)}")
    for ribbon in RIBBONS:
        end = ribbon.end()
        print(f"  {ribbon.name}: {ribbon.length:.1f} m, "
              f"end ({end[0]:.2f}, {end[1]:.2f})")


if __name__ == "__main__":
    main()