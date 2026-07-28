#!/bin/bash
# Reusable ROS2 service run script template
# This script launches ROS2 commands for any service
# Usage:
#   SERVICE_NAME=<name> ROS2_COMMAND="<full command>" /path/to/ros2_service_run.sh
#   If ROS2_COMMAND is not set, defaults to: ros2 launch ffw_bringup ${SERVICE_NAME}.launch.py
# Note: This script is called via /command/with-contenv, so environment is already set up

set -e

# Service name must be provided via environment variable
SERVICE_NAME="${SERVICE_NAME}"
if [ -z "${SERVICE_NAME}" ]; then
    echo "Error: SERVICE_NAME environment variable must be set" >&2
    exit 1
fi

# Set ROS_DOMAIN_ID if not already set (default to 30)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}
export ROS_DISTRO=${ROS_DISTRO:-jazzy}
export COLCON_WS=${COLCON_WS:-/root/ros2_ws}

# Enable s6-overlay debug logging (set S6_VERBOSITY=1 for more verbose output)
export S6_VERBOSITY=${S6_VERBOSITY:-1}

echo "[${SERVICE_NAME}] Starting service..."
echo "[${SERVICE_NAME}] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[${SERVICE_NAME}] ROS_DISTRO=${ROS_DISTRO}"
echo "[${SERVICE_NAME}] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "[${SERVICE_NAME}] COLCON_WS=${COLCON_WS}"
echo "[${SERVICE_NAME}] PID: $$"

# Record process group ID so the finish script can target the whole group
PGID=$(ps -o pgid= -p $$ | tr -d ' ')
echo "[${SERVICE_NAME}] Process group: ${PGID}"
echo "${PGID}" > /run/${SERVICE_NAME}.pgid || true

# Source ROS2 environment
source /opt/ros/${ROS_DISTRO}/setup.bash
source ${COLCON_WS}/install/setup.bash

# Physical robot bringup must never overlap a running policy container.
# Policy containers expose one of these services as soon as they start, even
# while the model session itself is STOPPED.  Fail closed if discovery cannot
# complete, because an unchecked bringup can race an existing command
# publisher and move hardware unexpectedly.
if [ "${SERVICE_NAME}" = "ai_worker_bringup" ]; then
    INFERENCE_SERVICES_FILE=$(mktemp)
    if ! timeout 8s ros2 service list > "${INFERENCE_SERVICES_FILE}"; then
        echo "[${SERVICE_NAME}] SAFETY BLOCK: could not verify that inference containers are stopped." >&2
        rm -f "${INFERENCE_SERVICES_FILE}"
        exit 64
    fi
    ACTIVE_INFERENCE_SERVICES=$(
        grep -E '^/(lerobot|groot)/inference_command$' "${INFERENCE_SERVICES_FILE}" || true
    )
    rm -f "${INFERENCE_SERVICES_FILE}"
    if [ -n "${ACTIVE_INFERENCE_SERVICES}" ]; then
        echo "[${SERVICE_NAME}] SAFETY BLOCK: inference service is still active:" >&2
        echo "${ACTIVE_INFERENCE_SERVICES}" >&2
        echo "[${SERVICE_NAME}] STOP, UNLOAD, and stop all inference containers before bringup." >&2
        exit 64
    fi
    echo "[${SERVICE_NAME}] Inference-container preflight passed."
fi

# Determine the command to execute
# If ROS2_COMMAND is set, use it; otherwise, use the default launch command
if [ -n "${ROS2_COMMAND}" ]; then
    ROS2_CMD="${ROS2_COMMAND}"
    echo "[${SERVICE_NAME}] Executing custom command: ${ROS2_CMD}"
else
    ROS2_CMD="ros2 launch ffw_bringup ${SERVICE_NAME}.launch.py"
    echo "[${SERVICE_NAME}] Executing default command: ${ROS2_CMD}"
fi

# If launch args file exists, append to command (format: key:=value key:=value)
LAUNCH_ARGS_FILE="/run/launch_args/${SERVICE_NAME}"
if [ -f "${LAUNCH_ARGS_FILE}" ]; then
    LAUNCH_ARGS=$(cat "${LAUNCH_ARGS_FILE}")
    ROS2_CMD="${ROS2_CMD} ${LAUNCH_ARGS}"
    echo "[${SERVICE_NAME}] Launch args: ${LAUNCH_ARGS}"
fi

# Execute the ROS2 command
# Using 'exec' ensures the command becomes PID 1 of this service,
# which allows s6 to properly signal it and its children
# Note: stdout/stderr are automatically piped to ${SERVICE_NAME}-log via producer-for/consumer-for
exec bash -i -c "${ROS2_CMD}"
