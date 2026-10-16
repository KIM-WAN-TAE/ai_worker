// Copyright 2026 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "gtest/gtest.h"
#include "joystick_controller/tact_switch_gate.hpp"

namespace joystick_controller
{

TEST(TactSwitchGateTest, InvalidSampleCannotCreateFalseRelease)
{
  TactSwitchGate gate;

  EXPECT_EQ(gate.observe(true, false, 0.0, 2.0), TactSwitchAction::NONE);
  EXPECT_EQ(gate.observe(true, true, 1.0, 2.0), TactSwitchAction::NONE);
  EXPECT_EQ(gate.observe(false, false, 1.1, 2.0), TactSwitchAction::NONE);
  EXPECT_EQ(gate.observe(true, true, 1.2, 2.0), TactSwitchAction::NONE);
  EXPECT_EQ(gate.observe(true, false, 1.3, 2.0), TactSwitchAction::SHORT_PRESS);
}

TEST(TactSwitchGateTest, LongPressEmitsOnceAndSuppressesRelease)
{
  TactSwitchGate gate;

  EXPECT_EQ(gate.observe(true, false, 0.0, 2.0), TactSwitchAction::NONE);
  EXPECT_EQ(gate.observe(true, true, 1.0, 2.0), TactSwitchAction::NONE);
  EXPECT_EQ(gate.observe(true, true, 3.0, 2.0), TactSwitchAction::LONG_PRESS);
  EXPECT_EQ(gate.observe(true, true, 3.1, 2.0), TactSwitchAction::NONE);
  EXPECT_EQ(gate.observe(true, false, 3.2, 2.0), TactSwitchAction::NONE);
}

TEST(TactSwitchGateTest, StartupPressedIsSuppressedUntilFreshPress)
{
  TactSwitchGate gate;

  EXPECT_EQ(gate.observe(true, true, 0.0, 2.0), TactSwitchAction::NONE);
  EXPECT_EQ(gate.observe(true, true, 3.0, 2.0), TactSwitchAction::NONE);
  EXPECT_EQ(gate.observe(true, false, 3.1, 2.0), TactSwitchAction::NONE);
  EXPECT_EQ(gate.observe(true, true, 4.0, 2.0), TactSwitchAction::NONE);
  EXPECT_EQ(gate.observe(true, false, 4.1, 2.0), TactSwitchAction::SHORT_PRESS);
}

TEST(TactSwitchGateTest, TwoGatesReleaseIndependently)
{
  TactSwitchGate left;
  TactSwitchGate right;

  EXPECT_EQ(left.observe(true, false, 0.0, 2.0), TactSwitchAction::NONE);
  EXPECT_EQ(right.observe(true, false, 0.0, 2.0), TactSwitchAction::NONE);
  EXPECT_EQ(left.observe(true, true, 1.0, 2.0), TactSwitchAction::NONE);
  EXPECT_EQ(right.observe(true, false, 1.0, 2.0), TactSwitchAction::NONE);
  EXPECT_EQ(left.observe(true, true, 1.1, 2.0), TactSwitchAction::NONE);
  EXPECT_EQ(right.observe(true, true, 1.1, 2.0), TactSwitchAction::NONE);
  EXPECT_EQ(left.observe(true, false, 1.2, 2.0), TactSwitchAction::SHORT_PRESS);
  EXPECT_EQ(right.observe(true, true, 1.2, 2.0), TactSwitchAction::NONE);
  EXPECT_EQ(left.observe(true, false, 1.3, 2.0), TactSwitchAction::NONE);
  EXPECT_EQ(right.observe(true, false, 1.3, 2.0), TactSwitchAction::SHORT_PRESS);
}

TEST(TactSwitchGateTest, RepeatedPressesWithInvalidSamplesEmitExactlyOnceEach)
{
  TactSwitchGate gate;
  size_t short_press_count = 0;

  EXPECT_EQ(gate.observe(true, false, 0.0, 2.0), TactSwitchAction::NONE);
  for (size_t cycle = 0; cycle < 1000; ++cycle) {
    const double start = static_cast<double>(cycle) * 0.1 + 1.0;
    EXPECT_EQ(gate.observe(true, true, start, 2.0), TactSwitchAction::NONE);
    EXPECT_EQ(gate.observe(false, false, start + 0.01, 2.0), TactSwitchAction::NONE);
    EXPECT_EQ(gate.observe(true, true, start + 0.02, 2.0), TactSwitchAction::NONE);
    if (gate.observe(true, false, start + 0.03, 2.0) == TactSwitchAction::SHORT_PRESS) {
      ++short_press_count;
    }
  }

  EXPECT_EQ(short_press_count, 1000U);
}

TEST(TactSwitchGateTest, RepeatedLongPressesNeverAlsoEmitShortPress)
{
  TactSwitchGate gate;
  size_t long_press_count = 0;
  size_t short_press_count = 0;

  EXPECT_EQ(gate.observe(true, false, 0.0, 2.0), TactSwitchAction::NONE);
  for (size_t cycle = 0; cycle < 1000; ++cycle) {
    const double start = static_cast<double>(cycle) * 3.0 + 1.0;
    EXPECT_EQ(gate.observe(true, true, start, 2.0), TactSwitchAction::NONE);
    if (gate.observe(true, true, start + 2.0, 2.0) == TactSwitchAction::LONG_PRESS) {
      ++long_press_count;
    }
    EXPECT_EQ(gate.observe(true, true, start + 2.1, 2.0), TactSwitchAction::NONE);
    if (gate.observe(true, false, start + 2.2, 2.0) == TactSwitchAction::SHORT_PRESS) {
      ++short_press_count;
    }
  }

  EXPECT_EQ(long_press_count, 1000U);
  EXPECT_EQ(short_press_count, 0U);
}

}  // namespace joystick_controller
