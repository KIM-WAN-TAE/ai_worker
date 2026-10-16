#include "gtest/gtest.h"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainiksolvervel_pinv.hpp"
#include "waypoint_telep/limit_ik.hpp"

namespace
{
struct Fixture
{
  KDL::Chain chain;
  KDL::JntArray lower, upper;
  explicit Fixture(unsigned int count) : lower(count), upper(count)
  {
    for (unsigned int j = 0; j < count; ++j) {
      chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ)));
      lower(j) = -1;
      upper(j) = 1;
    }
  }
  int solve(const std::vector<double> & values, double angle,
    KDL::JntArray & result, waypoint_telep::LimitIkDiagnostics & diagnostics,
    double continuity_limit = std::numeric_limits<double>::infinity())
  {
    KDL::ChainFkSolverPos_recursive fk(chain);
    KDL::ChainIkSolverVel_pinv velocity(chain);
    KDL::ChainIkSolverPos_NR_JL primary(chain, lower, upper, fk, velocity, 200, 1.e-8);
    KDL::JntArray seed(values.size());
    for (unsigned int j = 0; j < seed.rows(); ++j) {seed(j) = values[j];}
    return waypoint_telep::limit_aware_ik(chain, fk, velocity, primary,
      lower, upper, seed, KDL::Frame(KDL::Rotation::RotZ(angle)), result, 200, 1.e-8,
      &diagnostics, continuity_limit);
  }
};
}  // namespace

TEST(LimitIk, FindsAnInteriorRedundantSolutionInsteadOfKeepingTheStop)
{
  Fixture robot(2);
  KDL::JntArray result(2);
  waypoint_telep::LimitIkDiagnostics diagnostics;
  ASSERT_GE(robot.solve({0.999, -0.799}, 0.2, result, diagnostics), 0);
  EXPECT_GT(diagnostics.attempts, 1u);
  EXPECT_GE(diagnostics.minimum_margin, 0.087 - 1.e-8);
  EXPECT_NEAR(result(0) + result(1), 0.2, 2.e-8);
  EXPECT_FALSE(diagnostics.relaxed);
}

TEST(LimitIk, AllowsTheOnlyNearLimitSolutionButLeavesBoundaryClearance)
{
  Fixture robot(1);
  KDL::JntArray result(1);
  waypoint_telep::LimitIkDiagnostics diagnostics;
  ASSERT_GE(robot.solve({0.9}, 0.998, result, diagnostics), 0);
  EXPECT_TRUE(diagnostics.relaxed);
  EXPECT_NEAR(result(0), 0.998, 2.e-8);
  EXPECT_GT(diagnostics.minimum_margin, 0.0001);
  EXPECT_LT(robot.solve({0.9}, 1.0, result, diagnostics), 0);
  EXPECT_LT(robot.solve({0.9}, 1.5, result, diagnostics), 0);
}

TEST(LimitIk, PreservesAnAlreadyInteriorSolutionWithoutReseeding)
{
  Fixture robot(2);
  KDL::JntArray result(2);
  waypoint_telep::LimitIkDiagnostics diagnostics;
  ASSERT_GE(robot.solve({0.1, 0.2}, 0.3, result, diagnostics), 0);
  EXPECT_EQ(diagnostics.attempts, 1u);
  EXPECT_NEAR(result(0), 0.1, 1.e-9);
  EXPECT_NEAR(result(1), 0.2, 1.e-9);
}

TEST(LimitIk, RelaxesInwardMarginToPreserveContinuousSmallMotion)
{
  Fixture robot(2);
  KDL::JntArray result(2);
  waypoint_telep::LimitIkDiagnostics diagnostics;
  ASSERT_GE(robot.solve({0.999, -0.799}, 0.2, result, diagnostics, 0.02), 0);
  EXPECT_LE(std::abs(result(0) - 0.999), 0.02);
  EXPECT_LE(std::abs(result(1) + 0.799), 0.02);
  EXPECT_NEAR(result(0) + result(1), 0.2, 2.e-8);
  EXPECT_GT(diagnostics.minimum_margin, 0.0001);
  EXPECT_TRUE(diagnostics.relaxed);
}
