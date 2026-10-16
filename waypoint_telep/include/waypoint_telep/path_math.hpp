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

#ifndef WAYPOINT_TELEP__PATH_MATH_HPP_
#define WAYPOINT_TELEP__PATH_MATH_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace waypoint_telep
{

struct Quaternion
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

struct CartesianPose
{
  std::array<double, 3> position{0.0, 0.0, 0.0};
  Quaternion orientation{};
};

inline double clamp_unit(double value)
{
  return std::clamp(value, -1.0, 1.0);
}

inline Quaternion normalize(Quaternion value)
{
  const double norm = std::sqrt(
    value.x * value.x + value.y * value.y + value.z * value.z + value.w * value.w);
  if (!std::isfinite(norm) || norm < 1.0e-12) {
    throw std::invalid_argument("quaternion norm is zero or non-finite");
  }
  value.x /= norm;
  value.y /= norm;
  value.z /= norm;
  value.w /= norm;
  return value;
}

inline double quaternion_dot(const Quaternion & lhs, const Quaternion & rhs)
{
  return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z + lhs.w * rhs.w;
}

inline Quaternion slerp(Quaternion start, Quaternion goal, double progress)
{
  start = normalize(start);
  goal = normalize(goal);
  progress = std::clamp(progress, 0.0, 1.0);

  double dot = quaternion_dot(start, goal);
  if (dot < 0.0) {
    goal.x = -goal.x;
    goal.y = -goal.y;
    goal.z = -goal.z;
    goal.w = -goal.w;
    dot = -dot;
  }

  if (dot > 0.9995) {
    Quaternion result{
      start.x + progress * (goal.x - start.x),
      start.y + progress * (goal.y - start.y),
      start.z + progress * (goal.z - start.z),
      start.w + progress * (goal.w - start.w)};
    return normalize(result);
  }

  const double theta = std::acos(clamp_unit(dot));
  const double sin_theta = std::sin(theta);
  const double start_weight = std::sin((1.0 - progress) * theta) / sin_theta;
  const double goal_weight = std::sin(progress * theta) / sin_theta;
  return normalize(Quaternion{
      start_weight * start.x + goal_weight * goal.x,
      start_weight * start.y + goal_weight * goal.y,
      start_weight * start.z + goal_weight * goal.z,
      start_weight * start.w + goal_weight * goal.w});
}

inline double quintic_progress(double progress)
{
  const double value = std::clamp(progress, 0.0, 1.0);
  const double value2 = value * value;
  const double value3 = value2 * value;
  return 10.0 * value3 - 15.0 * value3 * value + 6.0 * value3 * value2;
}

inline double translation_distance(const CartesianPose & start, const CartesianPose & goal)
{
  const double dx = goal.position[0] - start.position[0];
  const double dy = goal.position[1] - start.position[1];
  const double dz = goal.position[2] - start.position[2];
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

inline double angular_distance(const Quaternion & start, const Quaternion & goal)
{
  const double dot = std::abs(quaternion_dot(normalize(start), normalize(goal)));
  return 2.0 * std::acos(clamp_unit(dot));
}

inline CartesianPose interpolate_pose(
  const CartesianPose & start, const CartesianPose & goal, double progress)
{
  const double blend = quintic_progress(progress);
  CartesianPose result;
  for (std::size_t index = 0; index < result.position.size(); ++index) {
    result.position[index] =
      start.position[index] + blend * (goal.position[index] - start.position[index]);
  }
  result.orientation = slerp(start.orientation, goal.orientation, blend);
  return result;
}

inline double interpolate_scalar(double start, double goal, double progress)
{
  const double blend = quintic_progress(progress);
  return start + blend * (goal - start);
}

inline double tool_path_correction_blend(
  std::size_t waypoint_number,
  std::size_t approach_start_waypoint,
  std::size_t grasp_waypoint)
{
  if (approach_start_waypoint < 1 || grasp_waypoint < approach_start_waypoint) {
    throw std::invalid_argument("invalid tool-path correction waypoint range");
  }
  if (waypoint_number < approach_start_waypoint) {
    return 0.0;
  }
  if (waypoint_number >= grasp_waypoint || grasp_waypoint == approach_start_waypoint) {
    return 1.0;
  }
  return static_cast<double>(waypoint_number - approach_start_waypoint) /
         static_cast<double>(grasp_waypoint - approach_start_waypoint);
}

inline std::array<double, 3> rotate_vector(
  const Quaternion & orientation,
  const std::array<double, 3> & vector)
{
  const Quaternion q = normalize(orientation);
  const double tx = 2.0 * (q.y * vector[2] - q.z * vector[1]);
  const double ty = 2.0 * (q.z * vector[0] - q.x * vector[2]);
  const double tz = 2.0 * (q.x * vector[1] - q.y * vector[0]);
  return {
    vector[0] + q.w * tx + q.y * tz - q.z * ty,
    vector[1] + q.w * ty + q.z * tx - q.x * tz,
    vector[2] + q.w * tz + q.x * ty - q.y * tx,
  };
}

inline CartesianPose apply_local_translation(
  const CartesianPose & pose,
  const std::array<double, 3> & local_translation,
  double scale = 1.0)
{
  if (!std::isfinite(scale)) {
    throw std::invalid_argument("tool-path correction scale must be finite");
  }
  const std::array<double, 3> scaled{
    local_translation[0] * scale,
    local_translation[1] * scale,
    local_translation[2] * scale,
  };
  if (!std::all_of(scaled.begin(), scaled.end(), [](double value) {
      return std::isfinite(value);
    }))
  {
    throw std::invalid_argument("tool-path correction must be finite");
  }
  const auto fixed_translation = rotate_vector(pose.orientation, scaled);
  CartesianPose result = pose;
  for (std::size_t index = 0; index < result.position.size(); ++index) {
    result.position[index] += fixed_translation[index];
  }
  return result;
}

inline double pchip_tangent(
  double previous_slope, double next_slope,
  double previous_duration, double next_duration)
{
  if (!std::isfinite(previous_slope) || !std::isfinite(next_slope) ||
    !std::isfinite(previous_duration) || !std::isfinite(next_duration) ||
    previous_duration <= 0.0 || next_duration <= 0.0)
  {
    throw std::invalid_argument("PCHIP slopes and durations must be finite and positive");
  }
  if (previous_slope == 0.0 || next_slope == 0.0 ||
    std::signbit(previous_slope) != std::signbit(next_slope))
  {
    return 0.0;
  }
  const double previous_weight = 2.0 * next_duration + previous_duration;
  const double next_weight = next_duration + 2.0 * previous_duration;
  return (previous_weight + next_weight) /
         (previous_weight / previous_slope + next_weight / next_slope);
}

inline double interpolate_cubic_hermite(
  double start, double goal, double start_velocity, double goal_velocity,
  double duration, double progress)
{
  if (!std::isfinite(duration) || duration <= 0.0) {
    throw std::invalid_argument("Hermite duration must be finite and positive");
  }
  const double value = std::clamp(progress, 0.0, 1.0);
  const double value2 = value * value;
  const double value3 = value2 * value;
  const double start_position_weight = 2.0 * value3 - 3.0 * value2 + 1.0;
  const double start_velocity_weight = value3 - 2.0 * value2 + value;
  const double goal_position_weight = -2.0 * value3 + 3.0 * value2;
  const double goal_velocity_weight = value3 - value2;
  return start_position_weight * start +
         start_velocity_weight * duration * start_velocity +
         goal_position_weight * goal +
         goal_velocity_weight * duration * goal_velocity;
}

inline double segment_duration(
  const CartesianPose & start,
  const CartesianPose & goal,
  double linear_speed,
  double angular_speed,
  double minimum_duration)
{
  if (linear_speed <= 0.0 || angular_speed <= 0.0 || minimum_duration <= 0.0) {
    throw std::invalid_argument("trajectory speeds and duration must be positive");
  }
  return std::max({
      minimum_duration,
      translation_distance(start, goal) / linear_speed,
      angular_distance(start.orientation, goal.orientation) / angular_speed});
}

}  // namespace waypoint_telep

#endif  // WAYPOINT_TELEP__PATH_MATH_HPP_
