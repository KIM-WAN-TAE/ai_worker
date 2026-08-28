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

# An intervention starts the Leader while a policy container remains loaded
# but PAUSED. The web supervisor may grant this script one short-lived,
# one-shot exception only after it has independently verified PAUSED state,
# command quiet, Follower feedback/controllers, DXL, and E-stop gates.
INTERVENTION_HANDOFF=false
INTERVENTION_LEASE_FILE="/run/ai_worker_intervention_preflight"
INTERVENTION_ACK_FILE="/run/ai_worker_intervention_preflight.ack"
if [ "${SERVICE_NAME}" = "avatar_bringup" ] && [ -f "${INTERVENTION_LEASE_FILE}" ]; then
    LEASE_VERSION=""
    LEASE_EXPIRES=""
    LEASE_MODEL=""
    LEASE_NONCE=""
    read -r LEASE_VERSION LEASE_EXPIRES LEASE_MODEL LEASE_NONCE < "${INTERVENTION_LEASE_FILE}" || true
    rm -f "${INTERVENTION_LEASE_FILE}"
    NOW_EPOCH=$(date +%s)
    SELECTED_LEADER="a3"
    if [ -f /run/leader_type ]; then
        SELECTED_LEADER=$(tr -d '[:space:]' < /run/leader_type | tr '[:upper:]' '[:lower:]')
    fi
    if [[ "${LEASE_EXPIRES}" =~ ^[0-9]+$ ]] \
        && [ "${LEASE_VERSION}" = "v1" ] \
        && [ "${LEASE_MODEL}" = "${SELECTED_LEADER}" ] \
        && [[ "${LEASE_NONCE}" =~ ^[a-f0-9]{32}$ ]] \
        && [ "${LEASE_EXPIRES}" -ge "${NOW_EPOCH}" ] \
        && [ "${LEASE_EXPIRES}" -le "$((NOW_EPOCH + 20))" ]; then
        INTERVENTION_HANDOFF=true
        INTERVENTION_ACK_TEMP="${INTERVENTION_ACK_FILE}.$$"
        printf '%s\n' "${LEASE_NONCE}" > "${INTERVENTION_ACK_TEMP}"
        chmod 0600 "${INTERVENTION_ACK_TEMP}"
        mv -f "${INTERVENTION_ACK_TEMP}" "${INTERVENTION_ACK_FILE}"
        echo "[${SERVICE_NAME}] Valid one-shot intervention hand-off lease accepted."
    else
        echo "[${SERVICE_NAME}] Invalid or expired intervention hand-off lease; normal safety preflight applies." >&2
    fi
fi

# Physical robot bringup must never overlap a running policy container.
# Policy containers expose one of these services as soon as they start, even
# while the model session itself is STOPPED.  Fail closed if discovery cannot
# complete, because an unchecked bringup can race an existing command
# publisher and move hardware unexpectedly.
if [ "${SERVICE_NAME}" = "ai_worker_bringup" ] || [ "${SERVICE_NAME}" = "avatar_bringup" ]; then
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
    if [ -n "${ACTIVE_INFERENCE_SERVICES}" ] && [ "${INTERVENTION_HANDOFF}" != true ]; then
        echo "[${SERVICE_NAME}] SAFETY BLOCK: inference service is still active:" >&2
        echo "${ACTIVE_INFERENCE_SERVICES}" >&2
        echo "[${SERVICE_NAME}] STOP, UNLOAD, and stop all inference containers before bringup." >&2
        exit 64
    elif [ -n "${ACTIVE_INFERENCE_SERVICES}" ]; then
        echo "[${SERVICE_NAME}] Intervention hand-off: PAUSED inference service connection allowed."
    fi

    # Newer policy runtimes may expose read-only RTC telemetry before (or
    # without) the command service becoming discoverable in this container.
    # A live RTC publisher still proves that a policy process capable of
    # reconnecting to robot command topics exists, so startup remains blocked.
    for RTC_TOPIC in /groot/rtc_status /lerobot/rtc_status; do
        RTC_TOPIC_INFO=$(mktemp)
        if timeout 5s ros2 topic info "${RTC_TOPIC}" > "${RTC_TOPIC_INFO}" 2>/dev/null; then
            RTC_PUBLISHERS=$(sed -n 's/^Publisher count: //p' "${RTC_TOPIC_INFO}" | tr -d '[:space:]')
            if [ -n "${RTC_PUBLISHERS}" ] && [ "${RTC_PUBLISHERS}" -gt 0 ] \
                && [ "${INTERVENTION_HANDOFF}" != true ]; then
                echo "[${SERVICE_NAME}] SAFETY BLOCK: policy runtime is publishing ${RTC_TOPIC}." >&2
                echo "[${SERVICE_NAME}] STOP, UNLOAD, and stop the policy container before hardware startup." >&2
                rm -f "${RTC_TOPIC_INFO}" "${INFERENCE_SERVICES_FILE}"
                exit 64
            elif [ -n "${RTC_PUBLISHERS}" ] && [ "${RTC_PUBLISHERS}" -gt 0 ]; then
                echo "[${SERVICE_NAME}] Intervention hand-off: ${RTC_TOPIC} publisher connection allowed."
            fi
        fi
        rm -f "${RTC_TOPIC_INFO}"
    done

    for COMMAND_TOPIC in \
        /leader/joint_trajectory_command_broadcaster_left/joint_trajectory \
        /leader/joint_trajectory_command_broadcaster_right/joint_trajectory \
        /leader/joystick_controller_left/joint_trajectory \
        /leader/joystick_controller_right/joint_trajectory; do
        COMMAND_TOPIC_INFO=$(mktemp)
        if timeout 5s ros2 topic info "${COMMAND_TOPIC}" > "${COMMAND_TOPIC_INFO}" 2>/dev/null; then
            COMMAND_PUBLISHERS=$(sed -n 's/^Publisher count: //p' "${COMMAND_TOPIC_INFO}" | tr -d '[:space:]')
            if [ -n "${COMMAND_PUBLISHERS}" ] && [ "${COMMAND_PUBLISHERS}" -gt 0 ] \
                && [ "${INTERVENTION_HANDOFF}" != true ]; then
                echo "[${SERVICE_NAME}] SAFETY BLOCK: command publisher is connected to ${COMMAND_TOPIC}." >&2
                echo "[${SERVICE_NAME}] Stop the command source before starting robot hardware." >&2
                rm -f "${COMMAND_TOPIC_INFO}"
                exit 64
            elif [ -n "${COMMAND_PUBLISHERS}" ] && [ "${COMMAND_PUBLISHERS}" -gt 0 ]; then
                echo "[${SERVICE_NAME}] Intervention hand-off: quiet publisher connection allowed on ${COMMAND_TOPIC}."
            fi
        fi
        rm -f "${COMMAND_TOPIC_INFO}"
    done
    echo "[${SERVICE_NAME}] Inference-container preflight passed."
fi

# A Leader is a command source, so never start it before a complete Follower
# control stack is observable.  This protects direct s6 starts as well as the
# web API gate.  The web supervisor additionally checks freshness and DXL
# error values before requesting this service.
if [ "${SERVICE_NAME}" = "avatar_bringup" ]; then
    for FOLLOWER_TOPIC in \
        /joint_states \
        /ffw_follower/dxl_state \
        /arm_l_controller/controller_state \
        /arm_r_controller/controller_state \
        /head_controller/controller_state \
        /lift_controller/controller_state; do
        FOLLOWER_TOPIC_INFO=$(mktemp)
        if ! timeout 5s ros2 topic info "${FOLLOWER_TOPIC}" > "${FOLLOWER_TOPIC_INFO}" 2>/dev/null; then
            echo "[${SERVICE_NAME}] SAFETY BLOCK: cannot verify Follower topic ${FOLLOWER_TOPIC}." >&2
            rm -f "${FOLLOWER_TOPIC_INFO}"
            exit 64
        fi
        FOLLOWER_PUBLISHERS=$(sed -n 's/^Publisher count: //p' "${FOLLOWER_TOPIC_INFO}" | tr -d '[:space:]')
        rm -f "${FOLLOWER_TOPIC_INFO}"
        if [ -z "${FOLLOWER_PUBLISHERS}" ] || [ "${FOLLOWER_PUBLISHERS}" -lt 1 ]; then
            echo "[${SERVICE_NAME}] SAFETY BLOCK: Follower is not ready (${FOLLOWER_TOPIC} has no publisher)." >&2
            echo "[${SERVICE_NAME}] Bring up the Follower and verify controllers, feedback, and DXL first." >&2
            exit 64
        fi
    done
    echo "[${SERVICE_NAME}] Follower-first preflight passed."
fi

# Follower startup is also blocked while a Leader publisher is alive.  A
# newly spawned follower controller subscribes to these streams immediately;
# allowing startup in that state can make the first cached/current command
# look like a large step on real hardware.
if [ "${SERVICE_NAME}" = "ai_worker_bringup" ]; then
    LEADER_TOPIC_INFO=$(mktemp)
    if timeout 5s ros2 topic info /leader/joint_states > "${LEADER_TOPIC_INFO}" 2>/dev/null; then
        LEADER_PUBLISHERS=$(sed -n 's/^Publisher count: //p' "${LEADER_TOPIC_INFO}" | tr -d '[:space:]')
        if [ -n "${LEADER_PUBLISHERS}" ] && [ "${LEADER_PUBLISHERS}" -gt 0 ]; then
            echo "[${SERVICE_NAME}] SAFETY BLOCK: Leader is publishing /leader/joint_states." >&2
            echo "[${SERVICE_NAME}] Stop the Leader and wait for its command publishers to disappear." >&2
            rm -f "${LEADER_TOPIC_INFO}"
            exit 64
        fi
    fi
    rm -f "${LEADER_TOPIC_INFO}"
    echo "[${SERVICE_NAME}] Leader-publisher preflight passed."
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
