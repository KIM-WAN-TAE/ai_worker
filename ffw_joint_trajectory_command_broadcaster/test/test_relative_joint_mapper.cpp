// Copyright 2026 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <cmath>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "gtest/gtest.h"
#include "joint_trajectory_command_broadcaster/relative_pose_mapper.hpp"
#include "joint_trajectory_command_broadcaster/teleop_reference_mode.hpp"

namespace joint_trajectory_command_broadcaster
{

TEST(RelativePoseMapperTest, ParsesOnlySupportedReferenceModes)
{
  EXPECT_EQ(parse_teleop_reference_mode("absolute"), TeleopReferenceMode::ABSOLUTE);
  EXPECT_EQ(parse_teleop_reference_mode("relative"), TeleopReferenceMode::RELATIVE);
  EXPECT_FALSE(parse_teleop_reference_mode("RELATIVE").has_value());
}

TEST(RelativePoseMapperTest, ZeroLeaderMotionKeepsFollowerAnchor)
{
  RelativePoseMapper mapper;
  Eigen::Isometry3d leader = Eigen::Isometry3d::Identity();
  leader.translate(Eigen::Vector3d(0.2, -0.4, 0.6));
  leader.rotate(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()));
  Eigen::Isometry3d follower = Eigen::Isometry3d::Identity();
  follower.translate(Eigen::Vector3d(1.0, -1.0, 0.5));
  follower.rotate(Eigen::AngleAxisd(-0.4, Eigen::Vector3d::UnitX()));

  ASSERT_TRUE(mapper.capture(leader, follower));
  Eigen::Isometry3d goal;
  ASSERT_TRUE(mapper.map(leader, goal));
  EXPECT_TRUE(goal.matrix().isApprox(follower.matrix(), 1.0e-12));
}

TEST(RelativePoseMapperTest, AppliesTranslationInSharedBaseAxes)
{
  RelativePoseMapper mapper;
  Eigen::Isometry3d leader_anchor = Eigen::Isometry3d::Identity();
  leader_anchor.translate(Eigen::Vector3d(0.1, 0.2, 0.3));
  leader_anchor.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
  Eigen::Isometry3d follower_anchor = Eigen::Isometry3d::Identity();
  follower_anchor.translate(Eigen::Vector3d(1.0, 2.0, 3.0));
  follower_anchor.rotate(Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX()));
  Eigen::Isometry3d leader_current = leader_anchor;
  leader_current.translation() += Eigen::Vector3d(0.4, -0.1, 0.2);

  ASSERT_TRUE(mapper.capture(leader_anchor, follower_anchor));
  Eigen::Isometry3d goal;
  ASSERT_TRUE(mapper.map(leader_current, goal));
  EXPECT_TRUE(goal.translation().isApprox(Eigen::Vector3d(1.4, 1.9, 3.2), 1.0e-12));
  EXPECT_TRUE(goal.linear().isApprox(follower_anchor.linear(), 1.0e-12));
}

TEST(RelativePoseMapperTest, AppliesRotationDeltaOnFollowerAnchorFromTheLeft)
{
  RelativePoseMapper mapper;
  Eigen::Isometry3d leader_anchor = Eigen::Isometry3d::Identity();
  leader_anchor.rotate(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX()));
  Eigen::Isometry3d follower_anchor = Eigen::Isometry3d::Identity();
  follower_anchor.rotate(Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY()));
  const Eigen::Matrix3d delta =
    Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Isometry3d leader_current = leader_anchor;
  leader_current.linear() = delta * leader_anchor.linear();

  ASSERT_TRUE(mapper.capture(leader_anchor, follower_anchor));
  Eigen::Isometry3d goal;
  ASSERT_TRUE(mapper.map(leader_current, goal));
  EXPECT_TRUE(goal.linear().isApprox(delta * follower_anchor.linear(), 1.0e-12));
}

TEST(RelativePoseMapperTest, ResetAndRecaptureUseFreshAnchors)
{
  RelativePoseMapper mapper;
  const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
  ASSERT_TRUE(mapper.capture(identity, identity));
  mapper.reset();
  Eigen::Isometry3d goal;
  EXPECT_FALSE(mapper.map(identity, goal));

  Eigen::Isometry3d follower = Eigen::Isometry3d::Identity();
  follower.translation() = Eigen::Vector3d(-0.5, 0.2, 0.7);
  ASSERT_TRUE(mapper.capture(identity, follower));
  ASSERT_TRUE(mapper.map(identity, goal));
  EXPECT_TRUE(goal.matrix().isApprox(follower.matrix(), 1.0e-12));
}

TEST(RelativePoseMapperTest, InvalidPoseIsRejected)
{
  RelativePoseMapper mapper;
  Eigen::Isometry3d invalid = Eigen::Isometry3d::Identity();
  invalid.translation().x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(mapper.capture(invalid, Eigen::Isometry3d::Identity()));
  EXPECT_FALSE(mapper.anchored());
}

TEST(RelativePoseMapperTest, OrientationErrorUsesShortestRotation)
{
  const Eigen::Matrix3d current = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d goal =
    Eigen::AngleAxisd(3.5, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d error = shortest_orientation_error(goal, current);
  EXPECT_NEAR(error.norm(), 2.0 * M_PI - 3.5, 1.0e-12);
  EXPECT_LT(error.z(), 0.0);
}

TEST(RelativePoseMapperTest, WeightedDampedVelocityMatchesClosedFormForIdentityJacobian)
{
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, 7);
  jacobian.leftCols(6).setIdentity();
  Eigen::Matrix<double, 6, 1> desired_velocity;
  desired_velocity << 1.0, -2.0, 3.0, -4.0, 5.0, -6.0;
  Eigen::Matrix<double, 6, 1> weights;
  weights << 10.0, 10.0, 10.0, 1.0, 1.0, 1.0;
  Eigen::VectorXd joint_velocity;

  ASSERT_TRUE(solve_weighted_damped_velocity(
      jacobian, desired_velocity, weights, 0.1, joint_velocity));
  ASSERT_EQ(joint_velocity.size(), 7);
  for (Eigen::Index i = 0; i < 6; ++i) {
    EXPECT_NEAR(
      joint_velocity[i],
      desired_velocity[i] * weights[i] / (weights[i] + 0.1), 1.0e-12);
  }
  EXPECT_DOUBLE_EQ(joint_velocity[6], 0.0);
}

TEST(RelativePoseMapperTest, WeightedDampedVelocityRejectsInvalidInputs)
{
  Eigen::VectorXd output = Eigen::VectorXd::Constant(2, 3.0);
  EXPECT_FALSE(solve_weighted_damped_velocity(
      Eigen::MatrixXd::Zero(5, 2),
      Eigen::Matrix<double, 6, 1>::Zero(),
      Eigen::Matrix<double, 6, 1>::Ones(), 0.1, output));
  EXPECT_TRUE(output.isApprox(Eigen::VectorXd::Constant(2, 3.0), 0.0));
}

}  // namespace joint_trajectory_command_broadcaster
