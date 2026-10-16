// Copyright 2026 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef JOINT_TRAJECTORY_COMMAND_BROADCASTER__TELEOP_REFERENCE_MODE_HPP_
#define JOINT_TRAJECTORY_COMMAND_BROADCASTER__TELEOP_REFERENCE_MODE_HPP_

#include <optional>
#include <string>

namespace joint_trajectory_command_broadcaster
{

enum class TeleopReferenceMode
{
  ABSOLUTE,
  RELATIVE,
};

inline std::optional<TeleopReferenceMode> parse_teleop_reference_mode(
  const std::string & value)
{
  if (value == "absolute") {
    return TeleopReferenceMode::ABSOLUTE;
  }
  if (value == "relative") {
    return TeleopReferenceMode::RELATIVE;
  }
  return std::nullopt;
}

}  // namespace joint_trajectory_command_broadcaster

#endif  // JOINT_TRAJECTORY_COMMAND_BROADCASTER__TELEOP_REFERENCE_MODE_HPP_
