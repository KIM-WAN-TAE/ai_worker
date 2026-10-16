// Copyright 2026 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef JOINT_TRAJECTORY_COMMAND_BROADCASTER__LEADER_ACTION_LATCH_HPP_
#define JOINT_TRAJECTORY_COMMAND_BROADCASTER__LEADER_ACTION_LATCH_HPP_

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace joint_trajectory_command_broadcaster
{

inline bool is_gripper_joint(const std::string & joint_name)
{
  return joint_name.rfind("gripper_", 0) == 0;
}

// Replace the held command atomically at vector granularity. A partial,
// non-finite, or dimensionally inconsistent leader/follower sample must never
// corrupt the command that the broadcaster continues to publish.
inline bool try_store_finite_action(
  const std::vector<double> & candidate, std::vector<double> & held_action)
{
  if (candidate.empty() ||
    (!held_action.empty() && candidate.size() != held_action.size()) ||
    !std::all_of(candidate.begin(), candidate.end(), [](double value) {
      return std::isfinite(value);
    }))
  {
    return false;
  }

  held_action = candidate;
  return true;
}

// Rebase arm joints to measured follower positions without turning a loaded
// gripper's deflected position into its new command. Gripper joints retain the
// most recently executed command so position error, and therefore grasp force,
// is continuous across an IDLE -> TELEOP handoff.
inline bool try_rebase_measured_action_preserving_grippers(
  const std::vector<double> & measured_state,
  const std::vector<double> & commanded_action,
  const std::vector<std::string> & joint_names,
  std::vector<double> & held_action)
{
  if (measured_state.empty() ||
    measured_state.size() != commanded_action.size() ||
    measured_state.size() != joint_names.size() ||
    !std::all_of(measured_state.begin(), measured_state.end(), [](double value) {
      return std::isfinite(value);
    }) ||
    !std::all_of(commanded_action.begin(), commanded_action.end(), [](double value) {
      return std::isfinite(value);
    }))
  {
    return false;
  }

  std::vector<double> candidate = measured_state;
  for (size_t i = 0; i < joint_names.size(); ++i) {
    if (is_gripper_joint(joint_names[i])) {
      candidate[i] = commanded_action[i];
    }
  }
  return try_store_finite_action(candidate, held_action);
}

// Apply a soft clutch to a leader gripper after TELEOP entry. The follower
// keeps the last executed command inside the deadband, then changes
// continuously from that command by the leader's displacement beyond it.
inline bool try_map_gripper_clutch_target(
  const double leader_position,
  const double leader_anchor,
  const double command_anchor,
  const double deadband,
  const double scale,
  double & target)
{
  if (!std::isfinite(leader_position) || !std::isfinite(leader_anchor) ||
    !std::isfinite(command_anchor) || !std::isfinite(deadband) ||
    !std::isfinite(scale) || deadband < 0.0 || scale <= 0.0)
  {
    return false;
  }

  const double displacement = leader_position - leader_anchor;
  const double active_displacement = std::copysign(
    std::max(0.0, std::abs(displacement) - deadband), displacement);
  const double candidate = command_anchor + scale * active_displacement;
  if (!std::isfinite(candidate)) {
    return false;
  }
  target = candidate;
  return true;
}

}  // namespace joint_trajectory_command_broadcaster

#endif  // JOINT_TRAJECTORY_COMMAND_BROADCASTER__LEADER_ACTION_LATCH_HPP_
