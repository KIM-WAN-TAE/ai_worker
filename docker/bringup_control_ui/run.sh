#!/usr/bin/env bash
set -eo pipefail

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
export ZENOH_CONFIG_OVERRIDE="${ZENOH_CONFIG_OVERRIDE:-transport/shared_memory/enabled=false}"
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source "${COLCON_WS:-/root/ros2_ws}/install/setup.bash"

control_host="${BRINGUP_CONTROL_HOST:-0.0.0.0}"
if [[ "${ENABLE_BRINGUP_CONTROL:-false}" =~ ^(1|true|yes)$ ]] && [ -z "${BRINGUP_CONTROL_TOKEN:-}" ]; then
  echo "[bringup_control_ui] Shared control mode: every client that can reach this server may request robot actions." >&2
fi

cd /root/ros2_ws/src/ai_worker/docker/bringup_control_ui
exec python3 -m uvicorn server:app \
  --host "${control_host}" \
  --port "${BRINGUP_CONTROL_PORT:-8780}" \
  --log-level info
