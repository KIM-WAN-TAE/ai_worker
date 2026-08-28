// Copyright 2026 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <limits>

#include "gtest/gtest.h"
#include "joint_trajectory_command_broadcaster/gripper_edge_gate.hpp"

namespace ffw_gripper_trigger
{

TEST(GripperEdgeGateTest, StartupBelowThresholdDoesNotTrigger)
{
  GripperEdgeGate gate;

  EXPECT_FALSE(gate.observe(-3.52, -0.5));
  EXPECT_TRUE(gate.initialized());
  EXPECT_FALSE(gate.observe(-3.0, -0.5));
}

TEST(GripperEdgeGateTest, TriggersOnlyAfterReleaseAndFreshPress)
{
  GripperEdgeGate gate;

  EXPECT_FALSE(gate.observe(-3.52, -0.5));
  EXPECT_FALSE(gate.observe(0.0, -0.5));
  EXPECT_TRUE(gate.observe(-0.6, -0.5));
  EXPECT_FALSE(gate.observe(-0.7, -0.5));
}

TEST(GripperEdgeGateTest, StartupAboveThresholdNeedsRealFallingEdge)
{
  GripperEdgeGate gate;

  EXPECT_FALSE(gate.observe(0.0, -0.5));
  EXPECT_TRUE(gate.observe(-0.6, -0.5));
}

TEST(GripperEdgeGateTest, NonFiniteSampleDoesNotInitializeOrTrigger)
{
  GripperEdgeGate gate;

  EXPECT_FALSE(gate.observe(std::numeric_limits<double>::quiet_NaN(), -0.5));
  EXPECT_FALSE(gate.initialized());
  EXPECT_FALSE(gate.observe(0.0, -0.5));
  EXPECT_TRUE(gate.initialized());
}

}  // namespace ffw_gripper_trigger
