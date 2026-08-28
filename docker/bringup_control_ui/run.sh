#!/usr/bin/env bash
set -eo pipefail

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
export ZENOH_CONFIG_OVERRIDE="${ZENOH_CONFIG_OVERRIDE:-transport/shared_memory/enabled=false}"
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source "${COLCON_WS:-/root/ros2_ws}/install/setup.bash"

# Cyclo owns the inference/recording message definitions. Build those types
# into a small, isolated overlay so this UI can subscribe with official ROS
# type support without changing the Follower/Leader controller install.
cyclo_interfaces_source="${CYCLO_INTERFACES_SOURCE:-/opt/cyclo_interfaces_src}"
cyclo_interfaces_overlay="${CYCLO_INTERFACES_OVERLAY:-/var/lib/ai_worker_cyclo_interfaces}"
cyclo_interfaces_setup="${cyclo_interfaces_overlay}/install/setup.bash"
rebuild_interfaces=false

if [ -f "${cyclo_interfaces_source}/package.xml" ]; then
  if [ ! -f "${cyclo_interfaces_setup}" ]; then
    rebuild_interfaces=true
  elif find "${cyclo_interfaces_source}" -type f -newer "${cyclo_interfaces_setup}" -print -quit | grep -q .; then
    rebuild_interfaces=true
  fi

  if [ "${rebuild_interfaces}" = true ]; then
    echo "[bringup_control_ui] Building Cyclo interfaces safety overlay..."
    mkdir -p \
      "${cyclo_interfaces_overlay}/build" \
      "${cyclo_interfaces_overlay}/install" \
      "${cyclo_interfaces_overlay}/log"
    colcon --log-base "${cyclo_interfaces_overlay}/log" build \
      --base-paths "${cyclo_interfaces_source}" \
      --packages-select interfaces \
      --build-base "${cyclo_interfaces_overlay}/build" \
      --install-base "${cyclo_interfaces_overlay}/install" \
      --symlink-install \
      --event-handlers console_direct+
  fi

  source "${cyclo_interfaces_setup}"
else
  echo "[bringup_control_ui] Cyclo interfaces source is unavailable; intervention remains fail-closed." >&2
fi

control_host="${BRINGUP_CONTROL_HOST:-0.0.0.0}"
if [[ "${ENABLE_BRINGUP_CONTROL:-false}" =~ ^(1|true|yes)$ ]] && [ -z "${BRINGUP_CONTROL_TOKEN:-}" ]; then
  echo "[bringup_control_ui] Shared control mode: every client that can reach this server may request robot actions." >&2
fi

cd /root/ros2_ws/src/ai_worker/docker/bringup_control_ui
exec python3 -m uvicorn server:app \
  --host "${control_host}" \
  --port "${BRINGUP_CONTROL_PORT:-8780}" \
  --log-level info
