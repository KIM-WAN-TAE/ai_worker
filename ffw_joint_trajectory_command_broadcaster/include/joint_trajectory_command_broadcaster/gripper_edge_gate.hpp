// Copyright 2026 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef JOINT_TRAJECTORY_COMMAND_BROADCASTER__GRIPPER_EDGE_GATE_HPP_
#define JOINT_TRAJECTORY_COMMAND_BROADCASTER__GRIPPER_EDGE_GATE_HPP_

#include <cmath>

namespace ffw_gripper_trigger
{

// Suppress the synthetic falling edge caused by starting the node while the
// gripper is already below the threshold. A real trigger is emitted only after
// a valid baseline sample and a subsequent above-to-below transition.
class GripperEdgeGate
{
public:
  bool observe(double position, double threshold)
  {
    if (!std::isfinite(position)) {
      return false;
    }

    const bool below_now = position < threshold;
    if (!initialized_) {
      initialized_ = true;
      below_ = below_now;
      return false;
    }

    const bool falling_edge = below_now && !below_;
    below_ = below_now;
    return falling_edge;
  }

  bool initialized() const
  {
    return initialized_;
  }

private:
  bool initialized_ = false;
  bool below_ = false;
};

enum class GripperGestureAction
{
  NONE,
  SHORT_PRESS,
  LONG_PRESS,
};

// Classify a gripper lever pull when there is no SensorXEL tact switch.
// A short action is emitted on release. A long action is emitted once while
// held, and suppresses the short action on the following release.
class GripperPressGestureGate
{
public:
  GripperGestureAction observe(
    double position, double press_threshold, double release_threshold,
    double now_sec, double long_press_duration)
  {
    if (!std::isfinite(position) || !std::isfinite(press_threshold) ||
      !std::isfinite(release_threshold) || !std::isfinite(now_sec) ||
      !std::isfinite(long_press_duration))
    {
      return GripperGestureAction::NONE;
    }

    if (!initialized_) {
      initialized_ = true;
      if (position < press_threshold) {
        pressed_ = true;
        suppress_until_release_ = true;
      }
      return GripperGestureAction::NONE;
    }

    if (!pressed_) {
      if (position < press_threshold) {
        pressed_ = true;
        press_start_sec_ = now_sec;
        long_press_triggered_ = false;
        suppress_until_release_ = false;
      }
      return GripperGestureAction::NONE;
    }

    if (position > release_threshold) {
      const bool emit_short = !suppress_until_release_ && !long_press_triggered_;
      pressed_ = false;
      long_press_triggered_ = false;
      suppress_until_release_ = false;
      return emit_short ? GripperGestureAction::SHORT_PRESS : GripperGestureAction::NONE;
    }

    if (!suppress_until_release_ && !long_press_triggered_ &&
      now_sec >= press_start_sec_ &&
      now_sec - press_start_sec_ >= long_press_duration)
    {
      long_press_triggered_ = true;
      return GripperGestureAction::LONG_PRESS;
    }

    return GripperGestureAction::NONE;
  }

  bool initialized() const
  {
    return initialized_;
  }

private:
  bool initialized_ = false;
  bool pressed_ = false;
  bool suppress_until_release_ = false;
  bool long_press_triggered_ = false;
  double press_start_sec_ = 0.0;
};

}  // namespace ffw_gripper_trigger

#endif  // JOINT_TRAJECTORY_COMMAND_BROADCASTER__GRIPPER_EDGE_GATE_HPP_
