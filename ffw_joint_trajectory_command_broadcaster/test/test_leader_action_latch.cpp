// Copyright 2026 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "joint_trajectory_command_broadcaster/leader_action_latch.hpp"
#include "joint_trajectory_command_broadcaster/toggle_command_gate.hpp"

namespace joint_trajectory_command_broadcaster
{

TEST(LeaderActionLatchTest, IdleFollowerFeedbackReplacesStaleTeleopAction)
{
  std::vector<double> held_action{0.1, -0.2, 0.3};
  const std::vector<double> follower_feedback{0.8, -0.7, 0.6};

  EXPECT_TRUE(try_store_finite_action(follower_feedback, held_action));
  EXPECT_EQ(held_action, follower_feedback);
}

TEST(LeaderActionLatchTest, IdleRebaseKeepsCommandedGripperInsteadOfMeasuredDeflection)
{
  std::vector<double> held_action{0.1, -0.2, 0.8};
  const std::vector<double> follower_feedback{0.4, -0.5, 0.2};
  const std::vector<double> last_executed_action{0.3, -0.4, 0.8};
  const std::vector<std::string> joint_names{
    "arm_l_joint1", "arm_l_joint2", "gripper_l_joint1"};

  EXPECT_TRUE(try_rebase_measured_action_preserving_grippers(
      follower_feedback, last_executed_action, joint_names, held_action));
  EXPECT_EQ(held_action, (std::vector<double>{0.4, -0.5, 0.8}));
}

TEST(LeaderActionLatchTest, InvalidExecutedActionCannotCorruptIdleHold)
{
  const std::vector<double> original{0.1, -0.2, 0.8};
  std::vector<double> held_action = original;
  const std::vector<std::string> joint_names{
    "arm_l_joint1", "arm_l_joint2", "gripper_l_joint1"};

  EXPECT_FALSE(try_rebase_measured_action_preserving_grippers(
      {0.4, -0.5, 0.2}, {0.3, -0.4}, joint_names, held_action));
  EXPECT_EQ(held_action, original);
  EXPECT_FALSE(try_rebase_measured_action_preserving_grippers(
      {0.4, -0.5, 0.2},
      {0.3, -0.4, std::numeric_limits<double>::quiet_NaN()},
      joint_names, held_action));
  EXPECT_EQ(held_action, original);
}

TEST(LeaderActionLatchTest, CompleteFiniteLeaderActionReplacesHeldAction)
{
  std::vector<double> held_action{0.1, -0.2, 0.3};
  const std::vector<double> leader_action{0.4, -0.5, 0.6};

  EXPECT_TRUE(try_store_finite_action(leader_action, held_action));
  EXPECT_EQ(held_action, leader_action);
}

TEST(LeaderActionLatchTest, NonFiniteLeaderActionCannotCorruptHeldAction)
{
  const std::vector<double> original{0.1, -0.2, 0.3};
  std::vector<double> held_action = original;

  EXPECT_FALSE(try_store_finite_action(
      {0.4, std::numeric_limits<double>::quiet_NaN(), 0.6}, held_action));
  EXPECT_EQ(held_action, original);
  EXPECT_FALSE(try_store_finite_action(
      {0.4, std::numeric_limits<double>::infinity(), 0.6}, held_action));
  EXPECT_EQ(held_action, original);
}

TEST(LeaderActionLatchTest, WrongSizedLeaderActionCannotCorruptHeldAction)
{
  const std::vector<double> original{0.1, -0.2, 0.3};
  std::vector<double> held_action = original;

  EXPECT_FALSE(try_store_finite_action({0.4, -0.5}, held_action));
  EXPECT_EQ(held_action, original);
  EXPECT_FALSE(try_store_finite_action({}, held_action));
  EXPECT_EQ(held_action, original);
}

TEST(LeaderActionLatchTest, GripperClutchKeepsExecutedActionInsideDeadband)
{
  double target = std::numeric_limits<double>::quiet_NaN();

  EXPECT_TRUE(try_map_gripper_clutch_target(0.31, 0.30, 0.80, 0.02, 1.0, target));
  EXPECT_DOUBLE_EQ(target, 0.80);
}

TEST(LeaderActionLatchTest, GripperClutchChangesContinuouslyBeyondDeadband)
{
  double positive_target = 0.0;
  double negative_target = 0.0;

  EXPECT_TRUE(try_map_gripper_clutch_target(
      0.35, 0.30, 0.80, 0.02, 1.0, positive_target));
  EXPECT_NEAR(positive_target, 0.83, 1.0e-12);
  EXPECT_TRUE(try_map_gripper_clutch_target(
      0.25, 0.30, 0.80, 0.02, 2.0, negative_target));
  EXPECT_NEAR(negative_target, 0.74, 1.0e-12);
}

TEST(LeaderActionLatchTest, InvalidGripperClutchInputLeavesTargetUnchanged)
{
  double target = 0.75;

  EXPECT_FALSE(try_map_gripper_clutch_target(
      std::numeric_limits<double>::quiet_NaN(), 0.30, 0.80, 0.02, 1.0, target));
  EXPECT_DOUBLE_EQ(target, 0.75);
  EXPECT_FALSE(try_map_gripper_clutch_target(0.30, 0.30, 0.80, -0.01, 1.0, target));
  EXPECT_DOUBLE_EQ(target, 0.75);
}

TEST(ToggleCommandGateTest, TwoAccumulatedTogglesProduceOneAcceptedEvent)
{
  ToggleCommandGate gate;

  EXPECT_TRUE(gate.accept(1.0, 0.75));
  EXPECT_FALSE(gate.accept(1.1, 0.75));
}

TEST(ToggleCommandGateTest, LaterIntentionalToggleIsAccepted)
{
  ToggleCommandGate gate;

  EXPECT_TRUE(gate.accept(1.0, 0.75));
  EXPECT_FALSE(gate.accept(1.749, 0.75));
  EXPECT_TRUE(gate.accept(1.75, 0.75));
}

TEST(ToggleCommandGateTest, ExplicitStateCommandResetAllowsImmediateToggle)
{
  ToggleCommandGate gate;

  EXPECT_TRUE(gate.accept(1.0, 0.75));
  gate.reset();
  EXPECT_TRUE(gate.accept(1.1, 0.75));
}

TEST(ToggleCommandGateTest, RepeatedDuplicatePairsNeverCancelEachOther)
{
  ToggleCommandGate gate;
  size_t accepted_count = 0;

  for (size_t cycle = 0; cycle < 1000; ++cycle) {
    const double start = static_cast<double>(cycle);
    if (gate.accept(start, 0.75)) {
      ++accepted_count;
    }
    if (gate.accept(start + 0.1, 0.75)) {
      ++accepted_count;
    }
  }

  EXPECT_EQ(accepted_count, 1000U);
}

}  // namespace joint_trajectory_command_broadcaster
