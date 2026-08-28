#!/usr/bin/env bash
# ROS setup scripts legitimately read unset variables, so nounset cannot be
# enabled while sourcing the workspace environment.
set -eo pipefail

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
export ZENOH_CONFIG_OVERRIDE="${ZENOH_CONFIG_OVERRIDE:-transport/shared_memory/enabled=false}"
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source /root/ros2_ws/install/setup.bash

cd /root/ros2_ws/src/ai_worker/docker/task_motion_inspector

# This service only subscribes, records, and analyzes. It has no robot command
# publisher or replay endpoint, so the diagnostic UI may be viewed on the robot LAN.
inspector_host="${MOTION_INSPECTOR_HOST:-0.0.0.0}"

exec python3 -m uvicorn server:app \
  --host "${inspector_host}" \
  --port "${MOTION_INSPECTOR_PORT:-8765}"
