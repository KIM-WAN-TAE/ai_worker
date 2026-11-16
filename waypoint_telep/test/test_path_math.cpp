// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0

#include <cmath>

#include "gtest/gtest.h"
#include "waypoint_telep/path_math.hpp"
#include "waypoint_telep/joint_clock.hpp"
#include "waypoint_telep/equal_time.hpp"
#include "waypoint_telep/path_refinement.hpp"

TEST(PathRefinement, RejectsUnsafeWireSettings)
{
  const std::vector<double> valid{1, 10, 0.6, 0.001, 0.01, 0.01, 5};
  const auto settings = waypoint_telep::parse_path_refinement(valid);
  EXPECT_EQ(settings.subdivisions, 10u);
  EXPECT_FALSE(settings.automatic_subdivisions);
  EXPECT_DOUBLE_EQ(settings.smoothing, 0.6);
  EXPECT_THROW(waypoint_telep::parse_path_refinement({}), std::invalid_argument);
  for (const auto & change : std::vector<std::pair<std::size_t, double>>{
      {0, 2}, {1, 0}, {1, 1.5}, {1, 257}, {2, -0.01}, {2, 1.01},
      {3, 0}, {4, 0}, {5, 0}, {6, 9}, {6, 1.2}, {3, NAN}, {4, INFINITY}})
  {
    auto bad = valid;
    bad[change.first] = change.second;
    EXPECT_THROW(waypoint_telep::parse_path_refinement(bad), std::invalid_argument);
  }
}

TEST(PathRefinement, AutomaticSamplingFollowsDistanceRotationAndJointTravel)
{
  using waypoint_telep::automatic_segment_subdivisions;
  waypoint_telep::CartesianPose start, goal;
  const std::vector<double> joints(8, 0);
  EXPECT_EQ(automatic_segment_subdivisions(start, goal, joints, joints), 1u);
  goal.position[0] = 0.005;
  EXPECT_EQ(automatic_segment_subdivisions(start, goal, joints, joints), 1u);
  goal.position[0] = 0.10;
  EXPECT_EQ(automatic_segment_subdivisions(start, goal, joints, joints), 10u);
  goal.position[0] = 0.35;
  EXPECT_EQ(automatic_segment_subdivisions(start, goal, joints, joints), 35u);
  goal.position[0] = 0;
  goal.orientation = {0, 0, std::sin(M_PI / 4), std::cos(M_PI / 4)};
  EXPECT_EQ(automatic_segment_subdivisions(start, goal, joints, joints), 18u);
  goal = start;
  auto bent = joints;
  bent[3] = M_PI / 2;
  EXPECT_EQ(automatic_segment_subdivisions(start, goal, joints, bent), 18u);
  EXPECT_THROW(automatic_segment_subdivisions(start, goal, joints, {}), std::invalid_argument);
  bent[3] = NAN;
  EXPECT_THROW(automatic_segment_subdivisions(start, goal, joints, bent), std::invalid_argument);
  goal.position[0] = 50;
  EXPECT_THROW(automatic_segment_subdivisions(start, goal, joints, joints), std::runtime_error);
  auto automatic = std::vector<double>{1, 10, 0.6, 0.001, 0.01, 0.01, 5, 1};
  EXPECT_TRUE(waypoint_telep::parse_path_refinement(automatic).automatic_subdivisions);
  automatic[7] = 0.5;
  EXPECT_THROW(waypoint_telep::parse_path_refinement(automatic), std::invalid_argument);
}

TEST(PathRefinement, ReferenceUsesUniformPositionAndShortestOrientation)
{
  waypoint_telep::CartesianPose start, goal;
  goal.position = {1, 2, 3};
  goal.orientation = {0, 0, 1, 0};
  const auto quarter = waypoint_telep::reference_pose(start, goal, 0.25);
  EXPECT_DOUBLE_EQ(quarter.position[0], 0.25);
  EXPECT_NEAR(waypoint_telep::angular_distance(start.orientation, quarter.orientation), M_PI / 4, 1e-12);
  EXPECT_NEAR(waypoint_telep::translation_distance(
      waypoint_telep::reference_pose(start, goal, 1), goal), 0, 1e-12);
}

TEST(EqualTime, UsesInverseDurationAndPreservesAlignmentAndExclusions)
{
  const auto result = waypoint_telep::equal_segment_scales(
    {0.0, 0.5, 2.0, 1.0}, {0.7, 1.0, 2.0, 1.0},
    {true, false, true, false}, 1.25, 3.0);
  EXPECT_DOUBLE_EQ(result.scales[0], 0.7);
  EXPECT_DOUBLE_EQ(result.scales[2], 2.0);
  EXPECT_NEAR(0.5 / result.scales[1], 1.25, 1.e-12);
  EXPECT_NEAR(1.0 / result.scales[3], 1.25, 1.e-12);
}

TEST(EqualTime, AutoDurationUsesMeanAndRespectsCommonFeasibleRange)
{
  const auto result = waypoint_telep::equal_segment_scales(
    {0.0, 0.5, 2.0}, {1.0, 1.0, 2.0}, {true, false, false}, 0.0, 3.0);
  EXPECT_DOUBLE_EQ(result.duration_s, 0.75);
  const auto limited = waypoint_telep::equal_segment_scales(
    {0.0, 0.5, 20.0}, {1.0, 3.0, 3.0}, {true, false, false}, 0.0, 3.0);
  EXPECT_DOUBLE_EQ(limited.duration_s, 20.0 / 3.0);
  EXPECT_THROW(waypoint_telep::equal_segment_scales(
    {0.0, 0.5, 2.0}, {1.0, 1.0, 1.0}, {true, false, false}, 0.1, 3.0),
    std::invalid_argument);
  EXPECT_THROW(waypoint_telep::equal_segment_scales(
    {0.0, 0.5, 200.0}, {1.0, 1.0, 1.0}, {true, false, false}, 0.0, 3.0),
    std::invalid_argument);
  EXPECT_THROW(waypoint_telep::equal_segment_scales(
    {0.0, 0.5}, {1.0, 1.0}, {true, true}, 0.0, 3.0), std::invalid_argument);
}

TEST(EqualTime, TotalBudgetIncludesAllDwellsAndExcludedTravel)
{
  const auto result = waypoint_telep::equal_segment_scales_for_total(
    {0.0, 0.5, 2.0, 1.0}, {0.7, 1.0, 2.0, 1.0},
    {true, false, true, false}, {0.2, 0.3, 0.4, 0.6}, 6.5, 3.0);
  EXPECT_DOUBLE_EQ(result.fixed_s, 2.5);
  EXPECT_DOUBLE_EQ(result.duration_s, 2.0);
  EXPECT_DOUBLE_EQ(result.scales[0], 0.7);
  EXPECT_DOUBLE_EQ(result.scales[2], 2.0);
  EXPECT_NEAR(result.fixed_s + 0.5 / result.scales[1] + 1.0 / result.scales[3], 6.5, 1.e-12);
  EXPECT_THROW(waypoint_telep::equal_segment_scales_for_total(
    {0.0, 0.5, 2.0}, {1.0, 1.0, 2.0}, {true, false, true},
    {0.2, 0.0, 0.3}, 1.0, 3.0), std::invalid_argument);
  EXPECT_THROW(waypoint_telep::equal_segment_scales_for_total(
    {0.0, 0.5}, {1.0, 1.0}, {true, false}, {0.0, 0.0}, 0.0, 3.0),
    std::invalid_argument);
}

TEST(JointClock, ContinuousMonotoneClockWithStoppedEndpoints)
{
  std::vector<waypoint_telep::ClockNode> clock;
  for (int i = 0; i <= 100; ++i) {
    clock.push_back({i * 0.01, 1.0, 0.5});
  }
  waypoint_telep::retime_clock(clock, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(clock.front().rate, 0.0);
  EXPECT_DOUBLE_EQ(clock.back().rate, 0.0);
  double previous = 0.0;
  for (std::size_t i = 1; i < clock.size(); ++i) {
    EXPECT_LE(clock[i].rate, clock[i].cap);
    EXPECT_GT(clock[i].t, clock[i-1].t);
    const double acceleration = (clock[i].rate-clock[i-1].rate) /
      (clock[i].t-clock[i-1].t);
    EXPECT_LE(std::abs(acceleration), 0.5+1.e-9);
    for (int j = 0; j <= 10; ++j) {
      const double s = waypoint_telep::clock_parameter(clock[i-1], clock[i],
        clock[i-1].t+(clock[i].t-clock[i-1].t)*j/10.0);
      EXPECT_GE(s+1.e-12, previous);
      previous = s;
    }
  }
  EXPECT_NEAR(previous, 1.0, 1.e-12);
}

TEST(JointClock, RejectsUnreachablePreservedBoundary)
{
  std::vector<waypoint_telep::ClockNode> clock{{0, 1, 0.1}, {0.01, 1, 0.1}};
  EXPECT_THROW(waypoint_telep::retime_clock(clock, 0.0, 1.0), std::invalid_argument);
}

TEST(JointClock, SmoothingNeverRaisesFeasibleSpeedAndKeepsBoundaryRates)
{
  std::vector<waypoint_telep::ClockNode> clock;
  for (int i = 0; i <= 400; ++i) {
    clock.push_back({i*0.01, i > 180 && i < 220 ? 0.3 : 1.0, 0.5});
  }
  waypoint_telep::retime_clock(clock, 0.0, 1.0);
  const auto original = clock;
  waypoint_telep::smooth_clock_slowdowns(clock, 10);
  EXPECT_DOUBLE_EQ(clock.front().rate, 0.0);
  EXPECT_DOUBLE_EQ(clock.back().rate, 1.0);
  EXPECT_GE(clock.back().t, original.back().t);
  for (std::size_t i = 0; i < clock.size(); ++i) {
    EXPECT_LE(clock[i].rate, original[i].rate+1.e-9);
  }
}

namespace waypoint_telep
{

TEST(PathMath, QuinticProgressKeepsEndpoints)
{
  EXPECT_DOUBLE_EQ(quintic_progress(-1.0), 0.0);
  EXPECT_DOUBLE_EQ(quintic_progress(0.0), 0.0);
  EXPECT_NEAR(quintic_progress(0.5), 0.5, 1.0e-12);
  EXPECT_DOUBLE_EQ(quintic_progress(1.0), 1.0);
  EXPECT_DOUBLE_EQ(quintic_progress(2.0), 1.0);
}

TEST(PathMath, InterpolatesPositionOnStraightLine)
{
  CartesianPose start;
  CartesianPose goal;
  goal.position = {1.0, 2.0, 3.0};

  const CartesianPose middle = interpolate_pose(start, goal, 0.5);
  EXPECT_NEAR(middle.position[0], 0.5, 1.0e-12);
  EXPECT_NEAR(middle.position[1], 1.0, 1.0e-12);
  EXPECT_NEAR(middle.position[2], 1.5, 1.0e-12);
}

TEST(PathMath, InterpolatesJointScalarWithQuinticBlend)
{
  EXPECT_NEAR(interpolate_scalar(-1.0, 1.0, 0.5), 0.0, 1.0e-12);
  EXPECT_NEAR(interpolate_scalar(-1.0, 1.0, 0.0), -1.0, 1.0e-12);
  EXPECT_NEAR(interpolate_scalar(-1.0, 1.0, 1.0), 1.0, 1.0e-12);
}

TEST(PathMath, ToolPathCorrectionBlendsFromApproachIntoFullGraspOffset)
{
  EXPECT_DOUBLE_EQ(tool_path_correction_blend(2, 3, 5), 0.0);
  EXPECT_DOUBLE_EQ(tool_path_correction_blend(3, 3, 5), 0.0);
  EXPECT_DOUBLE_EQ(tool_path_correction_blend(4, 3, 5), 0.5);
  EXPECT_DOUBLE_EQ(tool_path_correction_blend(5, 3, 5), 1.0);
  EXPECT_DOUBLE_EQ(tool_path_correction_blend(9, 3, 5), 1.0);
  EXPECT_DOUBLE_EQ(tool_path_correction_blend(5, 5, 5), 1.0);
  EXPECT_THROW(tool_path_correction_blend(1, 4, 3), std::invalid_argument);
}

TEST(PathMath, ToolPathCorrectionUsesEachWaypointLocalAxes)
{
  CartesianPose pose;
  pose.position = {1.0, 2.0, 3.0};
  const double half_sqrt = std::sqrt(0.5);
  pose.orientation = {0.0, 0.0, half_sqrt, half_sqrt};

  const CartesianPose corrected = apply_local_translation(
    pose, {0.010, 0.0, 0.0}, 0.5);
  EXPECT_NEAR(corrected.position[0], 1.0, 1.0e-12);
  EXPECT_NEAR(corrected.position[1], 2.005, 1.0e-12);
  EXPECT_NEAR(corrected.position[2], 3.0, 1.0e-12);
  EXPECT_NEAR(corrected.orientation.z, pose.orientation.z, 1.0e-12);
  EXPECT_NEAR(corrected.orientation.w, pose.orientation.w, 1.0e-12);
}

TEST(PathMath, PchipTangentPreservesDirectionAndStopsAtReversal)
{
  EXPECT_NEAR(pchip_tangent(1.0, 2.0, 1.0, 1.0), 4.0 / 3.0, 1.0e-12);
  EXPECT_DOUBLE_EQ(pchip_tangent(1.0, -1.0, 1.0, 1.0), 0.0);
  EXPECT_DOUBLE_EQ(pchip_tangent(0.0, 1.0, 1.0, 1.0), 0.0);
}

TEST(PathMath, CubicHermitePassesEndpointsWithRequestedVelocity)
{
  EXPECT_DOUBLE_EQ(interpolate_cubic_hermite(1.0, 3.0, 0.5, 0.25, 2.0, 0.0), 1.0);
  EXPECT_DOUBLE_EQ(interpolate_cubic_hermite(1.0, 3.0, 0.5, 0.25, 2.0, 1.0), 3.0);
  EXPECT_NEAR(
    interpolate_cubic_hermite(0.0, 1.0, 1.0, 1.0, 1.0, 0.5), 0.5, 1.0e-12);
}

TEST(PathMath, CubicHermiteRejectsInvalidDuration)
{
  EXPECT_THROW(
    interpolate_cubic_hermite(0.0, 1.0, 0.0, 0.0, 0.0, 0.5),
    std::invalid_argument);
}

TEST(PathMath, UsesShortestQuaternionPath)
{
  const Quaternion identity{0.0, 0.0, 0.0, 1.0};
  const Quaternion equivalent_identity{0.0, 0.0, 0.0, -1.0};
  const Quaternion result = slerp(identity, equivalent_identity, 0.5);
  EXPECT_NEAR(std::abs(result.w), 1.0, 1.0e-12);
  EXPECT_NEAR(result.x, 0.0, 1.0e-12);
  EXPECT_NEAR(result.y, 0.0, 1.0e-12);
  EXPECT_NEAR(result.z, 0.0, 1.0e-12);
}

TEST(PathMath, SegmentDurationUsesSlowestConstraint)
{
  CartesianPose start;
  CartesianPose goal;
  goal.position[0] = 0.10;

  EXPECT_NEAR(segment_duration(start, goal, 0.02, 1.0, 0.5), 5.0, 1.0e-12);
}

TEST(PathMath, RejectsInvalidQuaternion)
{
  EXPECT_THROW(normalize(Quaternion{0.0, 0.0, 0.0, 0.0}), std::invalid_argument);
}

}  // namespace waypoint_telep

TEST(EqualTime, ScheduledDenseReplayFitsActualControlPeriods)
{
  std::vector<double> base(353, .1), scales(353, 1), dwells(353, 0);
  std::vector<bool> excluded(353, false);
  excluded[0] = true;
  dwells[280] = .2; dwells[304] = .2;
  const auto result = waypoint_telep::scheduled_segment_scales_for_total(
    base, scales, excluded, dwells, 10.0, 90.0, 100.0);
  std::size_t samples = 40;
  for (std::size_t i = 1; i < base.size(); ++i) {
    const auto count = waypoint_telep::control_sample_count(base[i] / result.scales[i], 100.0);
    EXPECT_TRUE(count == 2 || count == 3);
    samples += count;
  }
  EXPECT_EQ(samples, 1000u);
  EXPECT_DOUBLE_EQ(result.scales[0], scales[0]);
  EXPECT_THROW(waypoint_telep::scheduled_segment_scales_for_total(
    base, scales, excluded, dwells, 7.0, 90.0, 100.0), std::invalid_argument);
  EXPECT_THROW(waypoint_telep::scheduled_segment_scales_for_total(
    base, scales, excluded, dwells, 10.001, 90.0, 100.0), std::invalid_argument);
}

TEST(EqualTime, ScheduledTimingRetainsExcludedScaleAndAccountsForShortDwell)
{
  const auto result = waypoint_telep::scheduled_segment_scales_for_total(
    {.1, .1, .1}, {1, 1, 5}, {true, false, true}, {.003, 0, 0}, .08, 90, 100);
  EXPECT_DOUBLE_EQ(result.scales[2], 5);
  EXPECT_NEAR(result.fixed_s, .03, 1.e-12);
  EXPECT_NEAR(.1 / result.scales[1], .05, 1.e-12);
  EXPECT_EQ(waypoint_telep::control_sample_count(.03000000000000001, 100), 3u);
}

TEST(EqualTime, ScheduledMotionMinimaGiveLargeGripperMovesMoreTime)
{
  std::vector<double> base(493, .1), scales(493, 1), dwells(493, 0), minimum(493, .01);
  std::vector<bool> excluded(493, false);
  excluded[0] = true;
  dwells[20] = 1.9;
  base[7] = .8816;
  minimum[7] = 2.0 * .8816 / 5.0;
  const auto result = waypoint_telep::scheduled_segment_scales_for_total(
    base, scales, excluded, dwells, 10, 90, 100, minimum);
  EXPECT_GE(base[7] / result.scales[7], minimum[7]);
  double scheduled = 1.9;
  for (std::size_t i = 1; i < base.size(); ++i) {
    scheduled += waypoint_telep::control_sample_count(base[i] / result.scales[i], 100, 1) / 100.0;
  }
  EXPECT_NEAR(scheduled, 10, 1.e-9);
}

TEST(EqualTime, ScheduledTravelWeightsPreventFastSparseDescent)
{
  const std::vector<double> base{.1, .1, .1, .1};
  const std::vector<double> distance{0, .001, .002, .03};
  const auto result = waypoint_telep::scheduled_segment_scales_for_total(
    base, {1, 1, 1, 1}, {true, false, false, false}, {0, 0, 0, 0},
    .66, 90, 100, {.01, .01, .01, .02}, distance);
  EXPECT_NEAR(base[1] / result.scales[1], .02, 1.e-12);
  EXPECT_NEAR(base[2] / result.scales[2], .04, 1.e-12);
  EXPECT_NEAR(base[3] / result.scales[3], .60, 1.e-12);
  for (std::size_t i = 1; i < base.size(); ++i) {
    EXPECT_NEAR(distance[i] / (base[i] / result.scales[i]), .05, 1.e-10);
  }
}

TEST(EqualTime, ScheduledTravelWeightsKeepMotionMinimumAndZeroTravel)
{
  const auto result = waypoint_telep::scheduled_segment_scales_for_total(
    {.1, .1, .1, .1}, {1, 1, 1, 1}, {true, false, false, false}, {0, 0, 0, 0},
    1, 90, 100, {.01, .2, .01, .3}, {0, .001, .1, 0});
  EXPECT_NEAR(.1 / result.scales[1], .2, 1.e-12);
  EXPECT_NEAR(.1 / result.scales[3], .3, 1.e-12);
  EXPECT_NEAR(.1 / result.scales[2], .5, 1.e-12);
}
