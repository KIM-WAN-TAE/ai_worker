#!/usr/bin/env bash
# Build pinned upstream packages in a container without devices or ROS networking.
set -euo pipefail
repository=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
relative_workspace="$repository/docker/workspace/official_relative_teleop"
mkdir -p "$relative_workspace/src"
python3 - "$repository" "$relative_workspace" <<'PY'
import json
from pathlib import Path
import subprocess
import sys
repository, workspace = map(Path,sys.argv[1:])
versions=json.loads((repository/'official_relative_teleop/upstream_versions.json').read_text())
cancel_patch=repository/'official_relative_teleop/patches/cancel_poses.patch'
sparse={
    'ai_worker':['ffw_joint_trajectory_command_broadcaster'],
    'cyclo_control':['cyclo_teleoperation','cyclo_motion_controller_core',
                     'cyclo_motion_controller_models','osqp_eigen_vendor'],
}
for name,revision in versions.items():
    target=workspace/'src'/name
    def git(*args):
        return subprocess.check_output(['git','-C',str(target),*args],text=True).strip()
    if not target.exists():
        subprocess.check_call(['git','clone','--filter=blob:none','--sparse',
                               f'https://github.com/ROBOTIS-GIT/{name}.git',str(target)])
    dirty=git('status','--porcelain')
    if dirty:
        own_patch=(name=='cyclo_control'
            and dirty=='M cyclo_teleoperation/src/core/teleoperation_node.cpp'
            and git('diff')==cancel_patch.read_text().strip())
        if not own_patch:
            raise SystemExit(f'Refusing to replace local changes in {target}')
        git('apply','--reverse',str(cancel_patch))
    if name in sparse:
        git('sparse-checkout','set',*sparse[name])
    else:
        git('sparse-checkout','disable')
    git('fetch','origin',revision)
    git('checkout','--detach',revision)
    if name=='cyclo_control':
        git('apply','--check',str(cancel_patch))
        git('apply',str(cancel_patch))
PY
docker run --rm --network none --entrypoint bash \
    -v "$relative_workspace:/workspace/official_relative_teleop" \
    -v "$repository:/local_ai_worker:ro" \
    "${OFFICIAL_RELATIVE_IMAGE:-robotis/ai-worker:1.3.0}" -c '
set -eo pipefail
source /root/ros2_ws/install/setup.bash
cd /workspace/official_relative_teleop
MAKEFLAGS=-j2 colcon build --executor sequential \
  --base-paths src /local_ai_worker/official_relative_teleop \
  --packages-select robotis_interfaces cyclo_motion_controller_models \
    cyclo_motion_controller_core cyclo_teleoperation \
    ffw_joint_trajectory_command_broadcaster official_relative_teleop \
  --packages-ignore osqp_eigen_vendor \
  --allow-overriding robotis_interfaces cyclo_motion_controller_models \
    cyclo_motion_controller_core ffw_joint_trajectory_command_broadcaster \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
'
