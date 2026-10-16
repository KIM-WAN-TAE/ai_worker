#ifndef WAYPOINT_TELEP__PATH_REFINEMENT_HPP_
#define WAYPOINT_TELEP__PATH_REFINEMENT_HPP_

#include <cmath>
#include <stdexcept>
#include <vector>

#include "waypoint_telep/path_math.hpp"

namespace waypoint_telep
{
// Wire units: enabled, subdivisions, smoothing, metres, radians, radians, retries.
struct PathRefinement
{
  bool enabled{false};
  std::size_t subdivisions{10};
  double smoothing{1.0};
  double position_tolerance{0.001};
  double orientation_tolerance{0.017453292519943295};
  double joint_tolerance{0.01};
  std::size_t max_refinements{5};
};

inline PathRefinement parse_path_refinement(const std::vector<double> & d)
{
  if (d.size() != 7) {
    throw std::invalid_argument("path refinement requires seven settings");
  }
  for (double x : d) {
    if (!std::isfinite(x)) {
      throw std::invalid_argument("path refinement settings must be finite");
    }
  }
  if ((d[0] != 0.0 && d[0] != 1.0) || d[1] < 1 || d[1] > 256 ||
    std::floor(d[1]) != d[1] || d[2] < 0 || d[2] > 1 ||
    d[3] <= 0 || d[3] > 0.1 || d[4] <= 0 || d[4] > 3.141592653589793 ||
    d[5] <= 0 || d[5] > 1 || d[6] < 0 || d[6] > 8 || std::floor(d[6]) != d[6])
  {
    throw std::invalid_argument("path refinement settings are outside their allowed range");
  }
  return {d[0] == 1.0, static_cast<std::size_t>(d[1]), d[2], d[3], d[4], d[5],
    static_cast<std::size_t>(d[6])};
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
