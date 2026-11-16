# Inference motion diagnostics

`capture.py` only subscribes. It never starts inference, publishes a robot command,
changes torque/gains, accesses the serial port, or calls a motor service.
Use a new output directory for each run; existing evidence is never overwritten.

Inside the `ai_worker` container with ROS and the workspace sourced:

```bash
python3 /root/ros2_ws/src/ai_worker/docker/control_diagnostics/capture.py \
  --seconds 30 --output /workspace/control_diagnostics/run_01
python3 /root/ros2_ws/src/ai_worker/docker/control_diagnostics/analyze.py \
  /workspace/control_diagnostics/run_01/capture.jsonl \
  --output /workspace/control_diagnostics/run_01/analysis.json
```

The recorder collects final arbiter output, inference preview, controller
reference/feedback, joint states, GPIO current, motor torque/error status, and
arbiter status on a shared receiver clock. Inference preview is the delivered
command preview, **not raw neural-network output**. To examine raw model chunks
and buffer events, use the existing inference/RLT recorder as well.
Arrival timestamps do not prove hardware execution times. Inspect missing
topics, dropped rows and gaps before interpreting results.

Compare these separately with the same pose and load:

1. A settled fixed target, torque on: constant controller reference with moving
   feedback is a downstream candidate (motor, mechanics, contact or load).
2. An existing slow, known trajectory with inference stopped: check reference
   tracking, current, and communication timing.
3. Inference with the same task and load: compare final commands and feedback,
   especially policy switches and queue depletion.

Physical trials require the operator to prepare the robot and work area. These
scripts do not arrange a pose or enable torque. Torque-off data must not be used
to claim stability under load. The analyzer explicitly reports it as inconclusive.
Its 2-second windows exclude the first 2 seconds, require sample coverage and
fresh torque-on telemetry, and report candidates rather than root-cause verdicts.

On 2026-11-10, the current container source was found to publish torque from
`dxl_torque_state_`, a map that is never populated, instead of the driver's
`GetDxlTorqueState()` cache. Until this publisher is corrected and validated,
run the analyzer with `--untrusted-torque-telemetry`. Its false values do not
establish that torque is off. Operator-reported holding state and measured
position/current should be reported separately; previous torque-OFF conclusions
based only on this topic are withdrawn. Even a corrected driver cache does not
replace a fresh hardware register read when independent verification is needed.

For the current Y-series arm configuration, startup logs specify time-based
profiles with `Profile Time=500` and `Profile Acceleration Time=250`. The Y-series
manual's 0.2ms units correspond to 100ms total and 50ms acceleration; verify live
registers before changing them. Keep initial gains and profile values as the
baseline. Do not blindly disable internal profiling or add a second long filter.
The position-interface ROS controller gains are not the motor's internal gains.
Current values retain driver units; saturation cannot be inferred without
verified model-specific current limits.

The optional runtime arm conditioner remains off by default. With
`CYCLO_SG2_ARM_SMOOTHING=true`, `CYCLO_ARM_LOOKAHEAD_S` accepts 0..0.1 seconds
(default 0). It samples already committed TT-RTC motion for arm joints only;
grippers, policy prefixes and queue phase are unchanged. This can reduce the
conditioner's added lag, but changes execution and requires task validation.
It does not compensate a measured motor delay automatically. Run
`cyclo_brain/policy/common/runtime/diagnostics/replay_arm_conditioning.py` in
the intelligence repository for offline candidate comparison before a trial.

Reference: https://emanual.robotis.com/docs/en/dxl/y/ym070-210-r099-rh/#profile-acceleration-time248-profile-time252
