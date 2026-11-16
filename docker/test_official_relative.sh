#!/usr/bin/env bash
set -euo pipefail
repository=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
docker run --rm --network none --entrypoint bash \
    -e ROS_DOMAIN_ID=91 -e ROS_LOCALHOST_ONLY=1 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -v "$repository/docker/workspace/official_relative_teleop:/workspace/official_relative_teleop" \
    -v "$repository:/local_ai_worker:ro" \
    "${OFFICIAL_RELATIVE_IMAGE:-robotis/ai-worker:1.3.0}" -c '
set -eo pipefail
# The base image predates Mini. Only this disposable test container is updated.
cp -a /local_ai_worker/ffw_description/. /root/ros2_ws/install/ffw_description/share/ffw_description/
source /root/ros2_ws/install/setup.bash
source /workspace/official_relative_teleop/install/local_setup.bash
python3 /local_ai_worker/official_relative_teleop/test/mock_integration.py
'
