#!/usr/bin/env bash

export WILDOS_ROOT="/home/admin1/cfy/nebula2-wildos"
export WILDOS_WS="/home/admin1/cfy/wildos_ws"

source /opt/ros/jazzy/setup.bash
if [ -f /opt/ros/jazzy/share/gz_tools_vendor/local_setup.bash ]; then
    source /opt/ros/jazzy/share/gz_tools_vendor/local_setup.bash
fi
source "${WILDOS_WS}/install/setup.bash"
source "${WILDOS_ROOT}/.venv_ros/bin/activate"

for _wildos_cuda_lib in "${WILDOS_ROOT}"/.venv_ros/lib/python3.12/site-packages/nvidia/*/lib; do
    if [ -d "${_wildos_cuda_lib}" ]; then
        export LD_LIBRARY_PATH="${_wildos_cuda_lib}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
    fi
done
unset _wildos_cuda_lib

case ":${PYTHONPATH:-}:" in
    *":${WILDOS_ROOT}:"*) ;;
    *) export PYTHONPATH="${WILDOS_ROOT}${PYTHONPATH:+:${PYTHONPATH}}" ;;
esac

export HF_HUB_OFFLINE=1
export RUGD_ROOT="/home/admin1/cfy/data/RUGD"
export WILDOS_FRONTIERS_ROOT="/home/admin1/cfy/data/wildos"
