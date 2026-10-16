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

TEST(GripperPressGestureGateTest, ShortPressIsEmittedOnRelease)
{
  GripperPressGestureGate gate;

  EXPECT_EQ(gate.observe(0.0, -0.7, -0.3, 0.0, 2.0), GripperGestureAction::NONE);
  EXPECT_EQ(gate.observe(-0.8, -0.7, -0.3, 1.0, 2.0), GripperGestureAction::NONE);
  EXPECT_EQ(gate.observe(-0.5, -0.7, -0.3, 1.5, 2.0), GripperGestureAction::NONE);
  EXPECT_EQ(
    gate.observe(0.0, -0.7, -0.3, 1.6, 2.0), GripperGestureAction::SHORT_PRESS);
}

TEST(GripperPressGestureGateTest, LongPressIsEmittedOnceAndSuppressesShortPress)
{
  GripperPressGestureGate gate;

  EXPECT_EQ(gate.observe(0.0, -0.7, -0.3, 0.0, 2.0), GripperGestureAction::NONE);
  EXPECT_EQ(gate.observe(-0.8, -0.7, -0.3, 1.0, 2.0), GripperGestureAction::NONE);
  EXPECT_EQ(gate.observe(-0.8, -0.7, -0.3, 2.9, 2.0), GripperGestureAction::NONE);
  EXPECT_EQ(
    gate.observe(-0.8, -0.7, -0.3, 3.0, 2.0), GripperGestureAction::LONG_PRESS);
  EXPECT_EQ(gate.observe(-0.8, -0.7, -0.3, 4.0, 2.0), GripperGestureAction::NONE);
  EXPECT_EQ(gate.observe(0.0, -0.7, -0.3, 4.1, 2.0), GripperGestureAction::NONE);
}

TEST(GripperPressGestureGateTest, StartupBelowThresholdIsSuppressedUntilRelease)
{
  GripperPressGestureGate gate;

  EXPECT_EQ(gate.observe(-1.0, -0.7, -0.3, 0.0, 2.0), GripperGestureAction::NONE);
  EXPECT_EQ(gate.observe(-1.0, -0.7, -0.3, 5.0, 2.0), GripperGestureAction::NONE);
  EXPECT_EQ(gate.observe(0.0, -0.7, -0.3, 5.1, 2.0), GripperGestureAction::NONE);
  EXPECT_EQ(gate.observe(-1.0, -0.7, -0.3, 6.0, 2.0), GripperGestureAction::NONE);
  EXPECT_EQ(
    gate.observe(0.0, -0.7, -0.3, 6.2, 2.0), GripperGestureAction::SHORT_PRESS);
}

TEST(GripperPressGestureGateTest, HysteresisDoesNotCreateDuplicatePresses)
{
  GripperPressGestureGate gate;

  EXPECT_EQ(gate.observe(0.0, -0.7, -0.3, 0.0, 2.0), GripperGestureAction::NONE);
  EXPECT_EQ(gate.observe(-0.8, -0.7, -0.3, 1.0, 2.0), GripperGestureAction::NONE);
  EXPECT_EQ(gate.observe(-0.6, -0.7, -0.3, 1.2, 2.0), GripperGestureAction::NONE);
  EXPECT_EQ(gate.observe(-0.8, -0.7, -0.3, 1.4, 2.0), GripperGestureAction::NONE);
  EXPECT_EQ(
    gate.observe(0.0, -0.7, -0.3, 1.6, 2.0), GripperGestureAction::SHORT_PRESS);
}

TEST(GripperPressGestureGateTest, RepeatedNoisyPressesEmitExactlyOnceEach)
{
  GripperPressGestureGate gate;
  size_t short_press_count = 0;

  EXPECT_EQ(gate.observe(0.0, -0.7, -0.3, 0.0, 2.0), GripperGestureAction::NONE);
  for (size_t cycle = 0; cycle < 1000; ++cycle) {
    const double start = static_cast<double>(cycle) * 0.1 + 1.0;
    EXPECT_EQ(
      gate.observe(-0.8, -0.7, -0.3, start, 2.0), GripperGestureAction::NONE);
    EXPECT_EQ(
      gate.observe(-0.5, -0.7, -0.3, start + 0.01, 2.0), GripperGestureAction::NONE);
    EXPECT_EQ(
      gate.observe(-0.8, -0.7, -0.3, start + 0.02, 2.0), GripperGestureAction::NONE);
    EXPECT_EQ(
      gate.observe(
        std::numeric_limits<double>::quiet_NaN(), -0.7, -0.3, start + 0.03, 2.0),
      GripperGestureAction::NONE);
    if (gate.observe(0.0, -0.7, -0.3, start + 0.04, 2.0) ==
      GripperGestureAction::SHORT_PRESS)
    {
      ++short_press_count;
    }
  }

  EXPECT_EQ(short_press_count, 1000U);
}

}  // namespace ffw_gripper_trigger
