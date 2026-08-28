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

}  // namespace ffw_gripper_trigger

#endif  // JOINT_TRAJECTORY_COMMAND_BROADCASTER__GRIPPER_EDGE_GATE_HPP_
