// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0

#ifndef WAYPOINT_TELEP__EQUAL_TIME_HPP_
#define WAYPOINT_TELEP__EQUAL_TIME_HPP_

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace waypoint_telep
{
struct EqualTimeResult
{
  double duration_s;
  double minimum_s;
  double maximum_s;
  std::vector<double> scales;
  double fixed_s{0.0};
};

// Entry zero is the live-pose alignment and is always preserved.
inline EqualTimeResult equal_segment_scales(
  const std::vector<double> & base_seconds, const std::vector<double> & old_scales,
  const std::vector<bool> & excluded, double requested_s, double max_scale)
{
  if (base_seconds.size() != old_scales.size() || excluded.size() != old_scales.size() ||
    !std::isfinite(requested_s) || requested_s < 0.0 ||
    !std::isfinite(max_scale) || max_scale < 1.0)
  {
    throw std::invalid_argument("invalid equal-time inputs");
  }
  EqualTimeResult result{0.0, 0.0, 1.e300, old_scales};
  double sum = 0.0;
  std::size_t count = 0;
  for (std::size_t i = 1; i < old_scales.size(); ++i) {
    if (excluded[i]) {continue;}
    if (!std::isfinite(base_seconds[i]) || base_seconds[i] <= 0.0 ||
      !std::isfinite(old_scales[i]) || old_scales[i] < 0.01 || old_scales[i] > max_scale)
    {
      throw std::invalid_argument("invalid segment duration or speed scale");
    }
    result.minimum_s = std::max(result.minimum_s, base_seconds[i] / max_scale);
    result.maximum_s = std::min(result.maximum_s, base_seconds[i] / 0.01);
    sum += base_seconds[i] / old_scales[i];
    ++count;
  }
  if (count == 0) {throw std::invalid_argument("no segments remain after exclusions");}
  if (result.minimum_s > result.maximum_s) {
    throw std::invalid_argument("no common duration within the speed scale limits");
  }
  result.duration_s = requested_s == 0.0 ?
    std::clamp(sum / count, result.minimum_s, result.maximum_s) : requested_s;
  if (result.duration_s < result.minimum_s || result.duration_s > result.maximum_s) {
    throw std::invalid_argument(
      "duration outside feasible range [" + std::to_string(result.minimum_s) + ", " +
      std::to_string(result.maximum_s) + "] seconds");
  }
  for (std::size_t i = 1; i < old_scales.size(); ++i) {
    if (!excluded[i]) {result.scales[i] = base_seconds[i] / result.duration_s;}
  }
  return result;
}

// Total starts at arrival at waypoint one and includes every dwell, including
// the last waypoint's dwell. Alignment from the measured pose is separate.
inline EqualTimeResult equal_segment_scales_for_total(
  const std::vector<double> & base_seconds, const std::vector<double> & old_scales,
  const std::vector<bool> & excluded, const std::vector<double> & dwells,
  double total_s, double max_scale)
{
  auto feasible = equal_segment_scales(base_seconds, old_scales, excluded, 0.0, max_scale);
  if (dwells.size() != base_seconds.size() || !std::isfinite(total_s) || total_s <= 0.0) {
    throw std::invalid_argument("total duration must be positive with valid dwell data");
  }
  double fixed = 0.0;
  std::size_t count = 0;
  for (std::size_t i = 0; i < dwells.size(); ++i) {
    if (!std::isfinite(dwells[i]) || dwells[i] < 0.0) {
      throw std::invalid_argument("invalid dwell duration");
    }
    fixed += dwells[i];
    if (i == 0) {continue;}
    if (!excluded[i]) {++count; continue;}
    if (!std::isfinite(base_seconds[i]) || base_seconds[i] <= 0.0 ||
      !std::isfinite(old_scales[i]) || old_scales[i] < 0.01 || old_scales[i] > max_scale)
    {
      throw std::invalid_argument("invalid excluded segment duration");
    }
    fixed += base_seconds[i] / old_scales[i];
  }
  const double minimum = fixed + count * feasible.minimum_s;
  const double maximum = fixed + count * feasible.maximum_s;
  if (total_s < minimum || total_s > maximum) {
    throw std::invalid_argument(
      "total duration outside speed-scale range [" + std::to_string(minimum) + ", " +
      std::to_string(maximum) + "] seconds (dwells and exclusions included)");
  }
  const double segment_s = std::clamp(
    (total_s - fixed) / count, feasible.minimum_s, feasible.maximum_s);
  auto result = equal_segment_scales(base_seconds, old_scales, excluded, segment_s, max_scale);
  result.fixed_s = fixed;
  return result;
}
}  // namespace waypoint_telep
#endif  // WAYPOINT_TELEP__EQUAL_TIME_HPP_
