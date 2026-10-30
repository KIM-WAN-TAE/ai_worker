#ifndef WAYPOINT_TELEP__PATH_REFINEMENT_HPP_
#define WAYPOINT_TELEP__PATH_REFINEMENT_HPP_

#include <cmath>
#include <stdexcept>
#include <vector>

#include "waypoint_telep/path_math.hpp"

namespace waypoint_telep
{
// The optional eighth setting selects geometry-based sampling. Seven-value
// messages retain the original fixed-count behavior for saved/legacy clients.
struct PathRefinement
{
  bool enabled{false};
  std::size_t subdivisions{10};
  double smoothing{1.0};
  double position_tolerance{0.001};
  double orientation_tolerance{0.017453292519943295};
  double joint_tolerance{0.01};
  std::size_t max_refinements{5};
  bool automatic_subdivisions{true};
};

inline PathRefinement parse_path_refinement(const std::vector<double> & d)
{
  if (d.size() != 7 && d.size() != 8) {
    throw std::invalid_argument("path refinement requires seven or eight settings");
  }
  for (double x : d) {
    if (!std::isfinite(x)) {
      throw std::invalid_argument("path refinement settings must be finite");
    }
  }
  if ((d[0] != 0.0 && d[0] != 1.0) || d[1] < 1 || d[1] > 256 ||
    std::floor(d[1]) != d[1] || d[2] < 0 || d[2] > 1 ||
    d[3] <= 0 || d[3] > 0.1 || d[4] <= 0 || d[4] > 3.141592653589793 ||
    d[5] <= 0 || d[5] > 1 || d[6] < 0 || d[6] > 8 || std::floor(d[6]) != d[6] ||
    (d.size() == 8 && d[7] != 0.0 && d[7] != 1.0))
  {
    throw std::invalid_argument("path refinement settings are outside their allowed range");
  }
  return {d[0] == 1.0, static_cast<std::size_t>(d[1]), d[2], d[3], d[4], d[5],
    static_cast<std::size_t>(d[6]), d.size() == 8 && d[7] == 1.0};
}

// Seed each segment independently: at most 10 mm translation or 5 degrees of
// rotation/joint travel per interval. Dense FK validation remains authoritative
// and can further refine Cartesian segments. These are sampling, not accuracy,
// limits. Joint travel also covers equal-tip-position changes of configuration.
inline std::size_t automatic_segment_subdivisions(
  const CartesianPose & start, const CartesianPose & goal,
  const std::vector<double> & start_joints, const std::vector<double> & goal_joints)
{
  if (start_joints.empty() || start_joints.size() != goal_joints.size()) {
    throw std::invalid_argument("automatic sampling requires matching joint dimensions");
  }
  constexpr double angle_step = 0.08726646259971647;
  double intervals = std::max(
    translation_distance(start, goal) / 0.01,
    angular_distance(start.orientation, goal.orientation) / angle_step);
  for (std::size_t j = 0; j < start_joints.size(); ++j) {
    const double delta = std::abs(goal_joints[j] - start_joints[j]);
    if (!std::isfinite(delta)) {
      throw std::invalid_argument("automatic sampling requires finite joint values");
    }
    intervals = std::max(intervals, delta / angle_step);
  }
  if (!std::isfinite(intervals) || intervals > 4096) {
    throw std::runtime_error("generation limit: automatic sampling exceeds 4096 subdivisions");
  }
  return std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(intervals - 1.e-12)));
}

inline CartesianPose reference_pose(
  const CartesianPose & start, const CartesianPose & goal, double progress)
{
  CartesianPose result;
  for (std::size_t i = 0; i < 3; ++i) {
    result.position[i] = start.position[i] + progress * (goal.position[i] - start.position[i]);
  }
  result.orientation = slerp(start.orientation, goal.orientation, progress);
  return result;
}

// Compare path geometry independently of its clock. Acceleration near a stop
// changes progress in time without changing the joint-space straight line.
inline double joint_path_progress(
  const std::vector<double> & start, const std::vector<double> & goal,
  const std::vector<double> & actual)
{
  double dot = 0, length = 0;
  for (std::size_t j = 0; j < start.size(); ++j) {
    const double delta = goal[j] - start[j];
    dot += (actual[j] - start[j]) * delta;
    length += delta * delta;
  }
  return length > 1.e-24 ? std::clamp(dot / length, 0.0, 1.0) : 0.0;
}
}  // namespace waypoint_telep
#endif  // WAYPOINT_TELEP__PATH_REFINEMENT_HPP_
