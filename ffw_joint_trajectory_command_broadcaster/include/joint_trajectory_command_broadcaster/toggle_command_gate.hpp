// Copyright 2026 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef JOINT_TRAJECTORY_COMMAND_BROADCASTER__TOGGLE_COMMAND_GATE_HPP_
#define JOINT_TRAJECTORY_COMMAND_BROADCASTER__TOGGLE_COMMAND_GATE_HPP_

#include <cmath>

namespace joint_trajectory_command_broadcaster
{

// A toggle is an event, not an absolute state. Suppress a second event arriving
// inside the debounce window so duplicated publications cannot immediately
// undo the first transition. An explicit non-toggle command resets this gate.
class ToggleCommandGate
{
public:
  bool accept(double now_sec, double debounce_duration_sec)
  {
    if (!std::isfinite(now_sec) || !std::isfinite(debounce_duration_sec) ||
      debounce_duration_sec < 0.0)
    {
      return false;
    }

    if (!has_last_accepted_ || now_sec < last_accepted_sec_ ||
      now_sec - last_accepted_sec_ >= debounce_duration_sec)
    {
      has_last_accepted_ = true;
      last_accepted_sec_ = now_sec;
      return true;
    }

    return false;
  }

  void reset()
  {
    has_last_accepted_ = false;
    last_accepted_sec_ = 0.0;
  }

private:
  bool has_last_accepted_ = false;
  double last_accepted_sec_ = 0.0;
};

}  // namespace joint_trajectory_command_broadcaster

#endif  // JOINT_TRAJECTORY_COMMAND_BROADCASTER__TOGGLE_COMMAND_GATE_HPP_
