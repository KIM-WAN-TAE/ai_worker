# Official relative pose control for SG2 + Mini Leader

This package integrates ROBOTIS `ai_worker` 2.3.2 and `cyclo_teleoperation` 0.5.3
with this workspace's Mini Leader and command source arbiter. Exact revisions are
in `upstream_versions.json`. Upstream sources are kept in the separate
`docker/workspace/official_relative_teleop` workspace. One small local patch,
`patches/cancel_poses.patch`, exposes the official runtime's existing ownership
reset as `~/cancel_poses` (Trigger). Official FK, relative mapping, QP, and collision
calculations are unchanged. This patch is necessary because the upstream source
switch service rejects requests while a preset is moving, and its arm-disable
command does not cancel presets. The adapter gates outgoing commands immediately,
cancels poses, then resets the source before accepting a new enable command.

The official leader computes a relative end-effector displacement from each
arm's enable-time anchor. Official `AbsolutePoseMode` (control mode 2) converts
that target to joint commands using its QP and collision constraints. Translation
gain is 1.0. The Mini bus remains at 50 Hz; the follower calculation runs at 100 Hz.

## Starting and reverting

The Mini entry in `avatar_bringup` runs `scripts/run_mini.sh`. Existing bringup UI,
preflight, intervention preparation, and per-arm buttons remain the entry points.
Starting the stack selects TELEOP after intervention preparation and fresh
feedback; both arms remain disabled until enabled with their normal buttons.
This integration is configured for **SG2 + Mini only**.

For a manual launch inside the already configured `ai_worker` container:

```bash
source /root/ros2_ws/install/setup.bash
bash /root/ros2_ws/src/ai_worker/official_relative_teleop/scripts/run_mini.sh
```

Use `MINI_TELEOP_BACKEND=legacy` for the previous Mini launch. For the supervised
service, set that variable in its environment before the next start. Do not source
the official overlay into the legacy shell. Existing underlay binaries and the
previous relative implementation are retained.

The install step does not restart any running robot process. A running Mini
session must be stopped and started through the existing bringup controls to
change implementations.

## Compatibility details

- Official trajectories use private `/official_relative/output/*` topics. The
  adapter forwards them to `/intervention/teleop/arm_*`; the existing arbiter is
  the sole publisher of physical arm command topics.
- Policy/waypoint ownership is preserved. During those modes, standby teleop
  samples track measured joints. A new TELEOP selection passes through the
  official external source state to discard old anchors and disable both arms.
- Existing UInt8 buttons keep their meanings: 0 disable, 1 enable, 2 toggle,
  3/4 saved poses. The saved-pose values are a snapshot of the local Mini config.
  Simultaneous preset requests are serialized until the follower acknowledges
  each request. Stopping an arm during a preset cancels the source and stops
  **both arms**; re-enable the desired arm afterwards. This also cancels queued
  presets. Mini's existing falling-edge gripper shortcut owns pose selection;
  the upstream long-hold initial-pose gesture is disabled.
- The Mini spring gripper keeps its clutch behavior: current executed target at
  enable, 0.02 rad deadband, range 0–1. Its fixed FK tool frame excludes trigger
  rotation so squeezing the trigger does not rotate the follower wrist.
- The follower's unused raw-joint input topics are isolated. Otherwise upstream's
  joint callback also updates the gripper and competes with the clutch input.
  Only relative control mode 2 is offered by this integration.
- `/robot/arm_*_leader/joint_states` mirrors arbiter-executed arm targets for
  compatibility with older recording configurations. Authoritative executed
  trajectories remain `/command_source_arbiter/executed/arm_*`.
- The calculation model uses the intersection of official and locally installed
  arm position/velocity limits. Official relative-mode collision constraints are
  enabled. Hardware description and follower controllers are unchanged.

## Rebuilding and testing

From the host repository root:

```bash
bash docker/build_official_relative.sh
bash docker/test_official_relative.sh
```

The build fetches pinned official revisions and compiles six packages in a
device-free container. The test container has no host network or device mounts,
uses ROS domain 91 and mock Mini encoders, and runs the real official controller,
QP runtime, compatibility adapter, and local arbiter. Its follower is a kinematic
command-following mock, not a physics or hardware simulation.

Results and logs are written to
`docker/workspace/official_relative_teleop/test_results/`.
This test does not establish hardware tracking accuracy, contact behavior,
physical collision clearance, camera/recording throughput, or pedal behavior.
The existing preflight and emergency-stop workflow still apply during operation.

`config/mini_hardware.yaml` is a normalized snapshot of the current Mini settings.
If Mini calibration offsets or saved poses are changed in the original config,
update this snapshot and rebuild this package as well.
