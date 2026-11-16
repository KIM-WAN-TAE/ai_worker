#!/usr/bin/env bash
# Invoked by avatar_bringup after its existing preflight and underlay setup.
set -eo pipefail
selected_backend=${MINI_TELEOP_BACKEND:-official}
if [[ "$selected_backend" == legacy ]]; then
    exec ros2 launch ffw_bringup ffw_lg2_mini_leader_ai.launch.py "$@"
fi
if [[ "$selected_backend" != official ]]; then
    echo "Unsupported MINI_TELEOP_BACKEND: $selected_backend" >&2
    exit 1
fi
if [[ -f /run/robot_type ]] && [[ "$(tr -d '[:space:]' < /run/robot_type)" != sg2 ]]; then
    echo 'Official Mini integration is configured for SG2. Select SG2 or MINI_TELEOP_BACKEND=legacy.' >&2
    exit 1
fi
relative_install=/workspace/official_relative_teleop/install
if [[ ! -f "$relative_install/official_relative_teleop/share/official_relative_teleop/package.xml" ]]; then
    echo 'Official relative overlay is missing. Run docker/build_official_relative.sh first.' >&2
    exit 1
fi
source "$relative_install/local_setup.bash"
exec ros2 launch official_relative_teleop mini_sg2.launch.py "$@"
