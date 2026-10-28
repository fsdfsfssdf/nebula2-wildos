#!/usr/bin/env bash
# Launch the WildOS semantic-search stack on one terrain level.
#
#   ./run_terrain_level.sh 2 headless:=false rviz:=true
#
# Level 0 is the flat regression world. Extra arguments are forwarded to
# ros2 launch, so headless/rviz/world_name overrides still work.
set -eo pipefail

level="${1:-1}"
case "${level}" in
  0) world="minimal_field_hidden.sdf" ;;
  1) world="terrain_01_gentle.sdf" ;;
  2) world="terrain_02_rough.sdf" ;;
  3) world="terrain_03_challenge.sdf" ;;
  *)
    echo "usage: $0 [0|1|2|3] [ros2 launch arguments...]" >&2
    exit 2
    ;;
esac
shift || true

source "${HOME}/cfy/nebula2-wildos/scripts/wildos_env.sh"
export GZ_PARTITION="${GZ_PARTITION:-wildos_terrain_l${level}}"

exec ros2 launch wildos_simulation object_search_stage7.launch.py \
  world_name:="${world}" "$@"