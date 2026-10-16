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
ISOLATED_LEADER_START=false
LEADER_START_PURPOSE="normal"
INTERVENTION_LEASE_FILE="/run/ai_worker_intervention_preflight"
INTERVENTION_ACK_FILE="/run/ai_worker_intervention_preflight.ack"
if [ "${SERVICE_NAME}" = "avatar_bringup" ] && [ -f "${INTERVENTION_LEASE_FILE}" ]; then
    LEASE_VERSION=""
    LEASE_EXPIRES=""
    LEASE_PURPOSE=""
    LEASE_MODEL=""
    LEASE_NONCE=""
    LEASE_FIELD_3=""
    LEASE_FIELD_4=""
    LEASE_FIELD_5=""
    read -r LEASE_VERSION LEASE_EXPIRES LEASE_FIELD_3 LEASE_FIELD_4 LEASE_FIELD_5 \
        < "${INTERVENTION_LEASE_FILE}" || true
    rm -f "${INTERVENTION_LEASE_FILE}"
    if [ "${LEASE_VERSION}" = "v1" ]; then
        # Backward-compatible interpretation for a controller being upgraded
        # while this bind-mounted runner is already live.
        LEASE_PURPOSE="intervention"
        LEASE_MODEL="${LEASE_FIELD_3}"
        LEASE_NONCE="${LEASE_FIELD_4}"
    elif [ "${LEASE_VERSION}" = "v2" ]; then
        LEASE_PURPOSE="${LEASE_FIELD_3}"
        LEASE_MODEL="${LEASE_FIELD_4}"
        LEASE_NONCE="${LEASE_FIELD_5}"
    fi
    NOW_EPOCH=$(date +%s)
    SELECTED_LEADER="a3"
    if [ -f /run/leader_type ]; then
        SELECTED_LEADER=$(tr -d '[:space:]' < /run/leader_type | tr '[:upper:]' '[:lower:]')
    fi
    if [[ "${LEASE_EXPIRES}" =~ ^[0-9]+$ ]] \
        && { [ "${LEASE_VERSION}" = "v1" ] || [ "${LEASE_VERSION}" = "v2" ]; } \
        && { [ "${LEASE_PURPOSE}" = "intervention" ] || [ "${LEASE_PURPOSE}" = "standby" ]; } \
        && [ "${LEASE_MODEL}" = "${SELECTED_LEADER}" ] \
        && [[ "${LEASE_NONCE}" =~ ^[a-f0-9]{32}$ ]] \
        && [ "${LEASE_EXPIRES}" -ge "${NOW_EPOCH}" ] \
        && [ "${LEASE_EXPIRES}" -le "$((NOW_EPOCH + 20))" ]; then
        ISOLATED_LEADER_START=true
        LEADER_START_PURPOSE="${LEASE_PURPOSE}"
        INTERVENTION_ACK_TEMP="${INTERVENTION_ACK_FILE}.$$"
        printf '%s\n' "${LEASE_NONCE}" > "${INTERVENTION_ACK_TEMP}"
        chmod 0600 "${INTERVENTION_ACK_TEMP}"
        mv -f "${INTERVENTION_ACK_TEMP}" "${INTERVENTION_ACK_FILE}"
        echo "[${SERVICE_NAME}] Valid one-shot ${LEASE_PURPOSE} lease accepted."
    else
        echo "[${SERVICE_NAME}] Invalid or expired intervention hand-off lease; normal safety preflight applies." >&2
    fi
fi

# Starting a Leader used to invoke a fresh `ros2` CLI process for every safety
# topic. With rmw_zenoh each invocation spends about 0.7-0.8 s rebuilding the
# same graph, which added roughly 10 s to an Intervention hand-off. Inspect the
# complete graph once instead. The required Follower publishers are also used
# as the discovery-ready sentinel, so an incomplete graph still fails closed.
AVATAR_GRAPH_PREFLIGHT=false
if [ "${SERVICE_NAME}" = "avatar_bringup" ]; then
    echo "[${SERVICE_NAME}] Running combined ROS graph preflight..."
    if ! ISOLATED_LEADER_START="${ISOLATED_LEADER_START}" timeout 5s python3 - <<'PY'
import os
import sys
import time

import rclpy
from rclpy.node import Node


FOLLOWER_TOPICS = (
    "/joint_states",
    "/ffw_follower/dxl_state",
    "/arm_l_controller/controller_state",
    "/arm_r_controller/controller_state",
    "/head_controller/controller_state",
    "/lift_controller/controller_state",
)
RTC_TOPICS = ("/groot/rtc_status", "/lerobot/rtc_status")
COMMAND_TOPICS = (
    "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory",
    "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory",
    "/leader/joystick_controller_left/joint_trajectory",
    "/leader/joystick_controller_right/joint_trajectory",
)
INFERENCE_SERVICES = {"/groot/inference_command", "/lerobot/inference_command"}


def publisher_counts(node, topics):
    return {topic: len(node.get_publishers_info_by_topic(topic)) for topic in topics}


isolated_start = os.environ.get("ISOLATED_LEADER_START", "false") == "true"
rclpy.init()
node = Node(f"avatar_bringup_graph_preflight_{os.getpid()}")
try:
    deadline = time.monotonic() + 4.0
    follower_counts = {}
    while time.monotonic() < deadline:
        follower_counts = publisher_counts(node, FOLLOWER_TOPICS)
        if all(count >= 1 for count in follower_counts.values()):
            # One short settle window replaces twelve independent CLI graph
            # discovery windows while still catching endpoints arriving with
            # the already-observed Follower graph.
            time.sleep(0.15)
            follower_counts = publisher_counts(node, FOLLOWER_TOPICS)
            if all(count >= 1 for count in follower_counts.values()):
                break
        time.sleep(0.05)
    else:
        missing = [topic for topic, count in follower_counts.items() if count < 1]
        print(
            "[avatar_bringup] SAFETY BLOCK: Follower publishers were not "
            f"discovered: {', '.join(missing)}",
            file=sys.stderr,
        )
        raise SystemExit(64)

    services = {name for name, _types in node.get_service_names_and_types()}
    active_inference = sorted(INFERENCE_SERVICES.intersection(services))
    rtc_counts = publisher_counts(node, RTC_TOPICS)
    command_counts = publisher_counts(node, COMMAND_TOPICS)

    if not isolated_start:
        connected_rtc = [topic for topic, count in rtc_counts.items() if count > 0]
        connected_commands = [
            topic for topic, count in command_counts.items() if count > 0
        ]
        blockers = active_inference + connected_rtc + connected_commands
        if blockers:
            print(
                "[avatar_bringup] SAFETY BLOCK: policy or command endpoints "
                f"are still connected: {', '.join(blockers)}",
                file=sys.stderr,
            )
            raise SystemExit(64)
    else:
        for service in active_inference:
            print(
                "[avatar_bringup] Intervention hand-off: PAUSED inference "
                f"service connection allowed on {service}."
            )
        for topic, count in {**rtc_counts, **command_counts}.items():
            if count > 0:
                print(
                    "[avatar_bringup] Intervention hand-off: quiet publisher "
                    f"connection allowed on {topic}."
                )
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
    then
        echo "[${SERVICE_NAME}] SAFETY BLOCK: combined ROS graph preflight failed." >&2
        exit 64
    fi
    AVATAR_GRAPH_PREFLIGHT=true
    echo "[${SERVICE_NAME}] Combined ROS graph preflight passed."
fi

# Physical robot bringup must never overlap a running policy container.
# Policy containers expose one of these services as soon as they start, even
# while the model session itself is STOPPED.  Fail closed if discovery cannot
# complete, because an unchecked bringup can race an existing command
# publisher and move hardware unexpectedly.
if [ "${SERVICE_NAME}" = "ai_worker_bringup" ] || \
    { [ "${SERVICE_NAME}" = "avatar_bringup" ] && [ "${AVATAR_GRAPH_PREFLIGHT}" != true ]; }; then
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
    if [ -n "${ACTIVE_INFERENCE_SERVICES}" ] && [ "${ISOLATED_LEADER_START}" != true ]; then
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
                && [ "${ISOLATED_LEADER_START}" != true ]; then
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
                && [ "${ISOLATED_LEADER_START}" != true ]; then
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
if [ "${SERVICE_NAME}" = "avatar_bringup" ] && [ "${AVATAR_GRAPH_PREFLIGHT}" != true ]; then
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

# A valid Intervention lease must also isolate the Leader command publishers from
# the final F2 controller topics. The web controller selects TELEOP only after
# both remapped streams are fresh; without that explicit selection the arbiter
# stays fail-closed in NONE.
if [ "${SERVICE_NAME}" = "avatar_bringup" ] && [ "${ISOLATED_LEADER_START}" = true ]; then
    ROS2_CMD="${ROS2_CMD} left_trajectory_output_topic:=/intervention/teleop/arm_left right_trajectory_output_topic:=/intervention/teleop/arm_right auto_select_teleop:=false"
    if [ "${SELECTED_LEADER}" = "mini" ]; then
        # Mini SensorXEL auxiliary outputs are not arm-arbitrated. Keep them
        # off the F2 command and recording-control topics for the entire
        # standby/intervention lease. Arm enable buttons remain on their
        # private Leader topics and are still required after TELEOP selection.
        ROS2_CMD="${ROS2_CMD} left_joystick_output_topic:=/intervention/teleop/head right_joystick_output_topic:=/intervention/teleop/lift cmd_vel_output_topic:=/intervention/teleop/cmd_vel tact_trigger_output_topic:=/intervention/teleop/tact_trigger"
    fi
    if [ "${SELECTED_LEADER}" = "a3" ] || [ "${SELECTED_LEADER}" = "mini" ]; then
        ROS2_CMD="${ROS2_CMD} teleop_reference_mode:=relative"
    fi
    echo "[${SERVICE_NAME}] ${LEADER_START_PURPOSE}: ${SELECTED_LEADER} outputs isolated behind command-source arbiter in relative mode."
fi

# Execute the ROS2 command
# Using 'exec' ensures the command becomes PID 1 of this service,
# which allows s6 to properly signal it and its children
# Note: stdout/stderr are automatically piped to ${SERVICE_NAME}-log via producer-for/consumer-for
exec bash -i -c "${ROS2_CMD}"
