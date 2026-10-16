// Copyright 2026 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef JOINT_TRAJECTORY_COMMAND_BROADCASTER__RELATIVE_POSE_MAPPER_HPP_
#define JOINT_TRAJECTORY_COMMAND_BROADCASTER__RELATIVE_POSE_MAPPER_HPP_

#include <cmath>
#include <utility>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace joint_trajectory_command_broadcaster
{

// This is the same base-axis relative-pose mapping used by Cyclo
// RelativePoseMode. Translation and rotation are intentionally handled
// separately; this is not follower_anchor * leader_anchor^-1 * leader_current.
class RelativePoseMapper
{
public:
  bool capture(
    const Eigen::Isometry3d & leader_anchor,
    const Eigen::Isometry3d & follower_anchor)
  {
    if (!leader_anchor.matrix().allFinite() || !follower_anchor.matrix().allFinite()) {
      reset();
      return false;
    }
    leader_anchor_ = leader_anchor;
    follower_anchor_ = follower_anchor;
    anchored_ = true;
    return true;
  }

  bool map(const Eigen::Isometry3d & leader_current, Eigen::Isometry3d & goal) const
  {
    if (!anchored_ || !leader_current.matrix().allFinite()) {
      return false;
    }

    goal = follower_anchor_;
    goal.translation() += leader_current.translation() - leader_anchor_.translation();
    const Eigen::Matrix3d base_frame_rotation_delta =
      leader_current.linear() * leader_anchor_.linear().transpose();
    goal.linear() = base_frame_rotation_delta * follower_anchor_.linear();
    return goal.matrix().allFinite();
  }

  void reset()
  {
    anchored_ = false;
    leader_anchor_.setIdentity();
    follower_anchor_.setIdentity();
  }

  bool anchored() const {return anchored_;}

private:
  bool anchored_ = false;
  Eigen::Isometry3d leader_anchor_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d follower_anchor_ = Eigen::Isometry3d::Identity();
};

inline Eigen::Quaterniond normalized_quaternion(const Eigen::Matrix3d & rotation)
{
  Eigen::Quaterniond quaternion(rotation);
  if (!std::isfinite(quaternion.norm()) || quaternion.norm() < 1.0e-12) {
    return Eigen::Quaterniond::Identity();
  }
  quaternion.normalize();
  if (quaternion.w() < 0.0) {
    quaternion.coeffs() *= -1.0;
  }
  return quaternion;
}

inline Eigen::Vector3d shortest_orientation_error(
  const Eigen::Matrix3d & goal_rotation,
  const Eigen::Matrix3d & current_rotation)
{
  Eigen::Quaterniond error =
    normalized_quaternion(goal_rotation) * normalized_quaternion(current_rotation).conjugate();
  if (!std::isfinite(error.norm()) || error.norm() < 1.0e-12) {
    return Eigen::Vector3d::Zero();
  }
  error.normalize();
  if (error.w() < 0.0) {
    error.coeffs() *= -1.0;
  }
  const Eigen::AngleAxisd angle_axis(error);
  if (!std::isfinite(angle_axis.angle()) || angle_axis.angle() <= 1.0e-9) {
    return Eigen::Vector3d::Zero();
  }
  return angle_axis.axis() * angle_axis.angle();
}

inline bool solve_weighted_damped_velocity(
  const Eigen::MatrixXd & jacobian,
  const Eigen::Matrix<double, 6, 1> & desired_velocity,
  const Eigen::Matrix<double, 6, 1> & weights,
  const double damping_weight,
  Eigen::VectorXd & joint_velocity)
{
  if (jacobian.rows() != 6 || jacobian.cols() == 0 || !jacobian.allFinite() ||
    !desired_velocity.allFinite() || !weights.allFinite() || weights.minCoeff() <= 0.0 ||
    !std::isfinite(damping_weight) || damping_weight <= 0.0)
  {
    return false;
  }
  Eigen::MatrixXd hessian = jacobian.transpose() * weights.asDiagonal() * jacobian;
  hessian.diagonal().array() += damping_weight;
  const Eigen::VectorXd gradient =
    jacobian.transpose() * weights.asDiagonal() * desired_velocity;
  const Eigen::LDLT<Eigen::MatrixXd> decomposition(hessian);
  if (decomposition.info() != Eigen::Success) {
    return false;
  }
  Eigen::VectorXd candidate = decomposition.solve(gradient);
  if (decomposition.info() != Eigen::Success || !candidate.allFinite()) {
    return false;
  }
  joint_velocity = std::move(candidate);
  return true;
}

}  // namespace joint_trajectory_command_broadcaster

#endif  // JOINT_TRAJECTORY_COMMAND_BROADCASTER__RELATIVE_POSE_MAPPER_HPP_
