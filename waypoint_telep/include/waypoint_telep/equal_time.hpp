// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0

#ifndef WAYPOINT_TELEP__EQUAL_TIME_HPP_
#define WAYPOINT_TELEP__EQUAL_TIME_HPP_

#include <algorithm>
#include <cmath>
#include <queue>
#include <functional>
#include <utility>
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
// Ignore floating-point noise at an exact control-period boundary.
inline std::size_t control_sample_count(double seconds, double rate, std::size_t minimum = 2)
{
  return std::max(minimum, static_cast<std::size_t>(std::ceil(seconds * rate - 1.e-8)));
}

// Allocate whole control periods, retaining every waypoint and dwell. The
// native planner must additionally verify the generated stream before commit.
inline EqualTimeResult scheduled_segment_scales_for_total(
  const std::vector<double> & base, const std::vector<double> & scales,
  const std::vector<bool> & excluded, const std::vector<double> & dwells,
  double total, double maximum_scale, double rate,
  const std::vector<double> & minimum_seconds = {},
  const std::vector<double> & distances = {})
{
  if (!std::isfinite(rate) || rate <= 0 || !std::isfinite(total) || total <= 0 ||
    total * rate > 250000 || base.size() != dwells.size() ||
    (!minimum_seconds.empty() && minimum_seconds.size() != base.size()) ||
    (!distances.empty() && distances.size() != base.size()))
  {throw std::invalid_argument("invalid scheduled timing budget");}
  auto result = equal_segment_scales(base, scales, excluded, 0.0, maximum_scale);
  const auto budget = static_cast<std::size_t>(std::llround(total * rate));
  if (std::abs(total * rate - budget) > 1.e-6) {
    throw std::invalid_argument("target must be a multiple of the control period: " +
      std::to_string(1.0 / rate) + " seconds");
  }
  std::vector<std::size_t> ticks(base.size(), 0), upper(base.size(), 0);
  using Entry = std::pair<double, std::size_t>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
  auto priority = [&](std::size_t i) {
      const double weight = distances.empty() ? 1.0 : distances[i];
      if (!std::isfinite(weight) || weight < 0) {
        throw std::invalid_argument("invalid segment travel distance");
      }
      // Allocate spare periods to the largest travel-per-time first. Equal
      // durations make sparsely sampled descents much faster than dense paths.
      return ticks[i] / std::max(weight, 1.e-12);
    };
  std::size_t used = 0, adjustable = 0;
  result.fixed_s = 0;
  for (std::size_t i = 0; i < base.size(); ++i) {
    if (!std::isfinite(dwells[i]) || dwells[i] < 0) {
      throw std::invalid_argument("invalid dwell duration");
    }
    const auto hold = dwells[i] > 0 ? control_sample_count(dwells[i], rate, 1) : 0;
    used += hold;
    result.fixed_s += hold / rate;
    if (i == 0) {continue;}
    if (excluded[i]) {
      if (!std::isfinite(base[i]) || base[i] <= 0 || !std::isfinite(scales[i]) ||
        scales[i] < .01 || scales[i] > maximum_scale)
      {throw std::invalid_argument("invalid preserved segment");}
      const auto fixed = control_sample_count(
        base[i] / scales[i], rate, minimum_seconds.empty() ? 2 : 1);
      used += fixed;
      result.fixed_s += fixed / rate;
      continue;
    }
    const double minimum = minimum_seconds.empty() ? 0.0 : minimum_seconds[i];
    if (!std::isfinite(minimum) || minimum < 0) {
      throw std::invalid_argument("invalid segment motion minimum");
    }
    ticks[i] = control_sample_count(
      std::max(base[i] / maximum_scale, minimum), rate, minimum_seconds.empty() ? 2 : 1);
    upper[i] = static_cast<std::size_t>(std::floor(base[i] / .01 * rate + 1.e-8));
    if (ticks[i] > upper[i]) {throw std::invalid_argument("no feasible control-period allocation");}
    used += ticks[i];
    ++adjustable;
    if (ticks[i] < upper[i]) {queue.emplace(priority(i), i);}
  }
  if (used > budget) {
    throw std::invalid_argument("target below sampled minimum: " +
      std::to_string(used / rate) + " seconds (waypoints, dwells and exclusions retained)");
  }
  while (used < budget) {
    if (queue.empty()) {throw std::invalid_argument("target exceeds speed-scale range");}
    const auto i = queue.top().second;
    queue.pop();
    ++ticks[i]; ++used;
    if (ticks[i] < upper[i]) {queue.emplace(priority(i), i);}
  }
  for (std::size_t i = 1; i < base.size(); ++i) {
    if (!excluded[i]) {result.scales[i] = base[i] / (ticks[i] / rate);}
  }
  result.duration_s = (total - result.fixed_s) / adjustable;
  return result;
}
}  // namespace waypoint_telep
#endif  // WAYPOINT_TELEP__EQUAL_TIME_HPP_
