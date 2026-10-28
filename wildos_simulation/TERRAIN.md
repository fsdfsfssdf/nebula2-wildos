# Progressive terrain suite

The terrain suite keeps the hidden-target route, camera/LiDAR layout and Gazebo
world name (`wildos_minimal_field`) unchanged.  Only the ground profile,
shoulder clutter and obstacle density become progressively harder.

| Level | World | Max grade | Mounds | Rocks | Boulders | Intended use |
|---|---|---:|---:|---:|---:|---|
| 0 | `minimal_field_hidden.sdf` | 0% | 0 | 0 | 0 | Regression baseline |
| 1 | `terrain_01_gentle.sdf` | 4.19% | 30 | 18 | 0 | First terrain smoke test |
| 2 | `terrain_02_rough.sdf` | 5.50% | 52 | 38 | 2 | Rough shoulders and rolling grade |
| 3 | `terrain_03_challenge.sdf` | 7.85% | 76 | 60 | 4 | Narrow traverse and larger climb |

The road is one watertight triangle mesh with a cosine-eased height profile.
This avoids the collision lips and sharp crests produced by joining rotated
boxes; the first implementation pitched the proxy to 82 degrees at a piecewise
linear crest even though the grade was only 3.3%.

## Run

```bash
source ~/cfy/nebula2-wildos/scripts/wildos_env.sh
export GZ_PARTITION=wildos_terrain_l1

ros2 launch wildos_simulation object_search_stage7.launch.py \
  headless:=false rviz:=true world_name:=terrain_01_gentle.sdf
```

Replace the world name with `terrain_02_rough.sdf` or
`terrain_03_challenge.sdf` as the previous level becomes stable.

## Regenerate

```bash
python3 ~/cfy/nebula2-wildos/wildos_simulation/scripts/generate_terrain_worlds.py
cd ~/cfy/wildos_ws
colcon build --symlink-install \
  --base-paths ~/cfy/nebula2-wildos \
  --packages-select wildos_simulation
```

The generator writes the SDF files and `worlds/meshes/*_road.obj`.

## End-to-end results

Every level ran the complete semantic loop on the same route: visual frontier
exploration, LiDAR target lock, safe graph-node selection, Nav2 approach, and
red-house confirmation from the front camera.

| Level | Max grade | Observation node | Robot final (x, y) | Target estimate | Surface error | Front-camera red px | Final pitch |
|---|---:|---|---|---|---:|---:|---:|
| 1 | 4.19% | (19.86, 2.11) | (19.51, 2.09) | (26.62, 6.14) | 2.32 m | 1,466 | -1.5 deg |
| 2 | 5.50% | (19.82, 1.86) | (19.51, 1.73) | (26.37, 6.76) | 2.05 m | 4,459 | -0.8 deg |
| 3 | 7.85% | (21.82, 1.33) | (21.50, 1.21) | (26.31, 6.86) | 2.04 m | 27,128 | +1.0 deg |

The target house is a 4 x 4 x 3 m box centred at (28, 8, 1.5). LiDAR and
multi-view triangulation return the visible *surface*, so a ~2 m offset toward
the robot is geometric, not drift: the estimate lands on the near face of the
house. All runs finished with Nav2 reporting `Goal succeeded` multiple times and
the robot stationary within 0.35 m of its observation node.

## Navigation-proxy fix found during level 2

The proxy originally rested on a single 1.0 x 0.8 m flat skid box. On the level 2
mesh the box snagged on triangle edges: at x = 18.1 m the planner commanded
0.14 rad/s but the measured yaw rate was 1.2e-4 rad/s, so the robot sat still
while Nav2 kept reporting success. Lifting the model 0.25 m freed it instantly,
which confirmed a contact jam rather than a planner failure.

The skid collision is now four 6 cm low-friction spheres at the corners of
`d1h_navigation.urdf.xacro`. The same corner is now traversed normally, and the
proxy keeps its upright posture: final roll/pitch stayed within 1 deg on all
three levels.

## Reproduce one level

```bash
# level 0 = flat baseline, 1..3 = progressive terrain
~/cfy/nebula2-wildos/wildos_simulation/scripts/run_terrain_level.sh 2 \
  headless:=false rviz:=true
```

The wrapper sources `wildos_env.sh`, sets its own `GZ_PARTITION`
(`wildos_terrain_l<level>`) and forwards any extra `ros2 launch` arguments.

## Known limits

- The robot still uses the planar `VelocityControl` navigation proxy, not the
  articulated D1H wheel-leg controller. Terrain enters planning through the
  elevation/traversability maps, not through leg dynamics.
- The terrain is a designed corridor, not learned or scanned outdoor terrain.
  Levels differ by grade, shoulder clutter, rock density and track width.
- The target lies behind a fence and wall; the reachable observation distance is
  about 7-9 m even though the nominal object standoff is 3.5 m.
- One run per level is reported. Multi-seed start poses, repeated trials and the
  geometric-frontier / ImgFrontierNav / LRN baselines are still open.