#ifndef WAYPOINT_TELEP__LIMIT_IK_HPP_
#define WAYPOINT_TELEP__LIMIT_IK_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "kdl/chainiksolverpos_nr_jl.hpp"

namespace waypoint_telep
{
struct LimitIkDiagnostics
{
  unsigned int attempts{0};
  double minimum_margin{0};
  bool relaxed{false};
};

// Search inward before accepting a solution close to a mechanical stop. Every
// candidate must still solve the original pose; clamping the output is unsafe.
inline int limit_aware_ik(
  const KDL::Chain & chain, KDL::ChainFkSolverPos & fk,
  KDL::ChainIkSolverVel & velocity, KDL::ChainIkSolverPos_NR_JL & primary,
  const KDL::JntArray & lower, const KDL::JntArray & upper,
  const KDL::JntArray & seed, const KDL::Frame & pose, KDL::JntArray & output,
  unsigned int iterations, double epsilon, LimitIkDiagnostics * diagnostics = nullptr,
  double continuity_limit = std::numeric_limits<double>::infinity())
{
  LimitIkDiagnostics report;
  const auto fits = [&](const KDL::JntArray & q, double margin) {
      for (unsigned int j = 0; j < q.rows(); ++j) {
        const double inset = std::min(margin, 0.05 * (upper(j) - lower(j)));
        if (!std::isfinite(q(j)) || q(j) < lower(j) + inset - 1.e-10 ||
          q(j) > upper(j) - inset + 1.e-10) {return false;}
      }
      KDL::Frame actual;
      if (fk.JntToCart(q, actual) < 0) {return false;}
      double x, y, z, w;
      (actual.M.Inverse() * pose.M).GetQuaternion(x, y, z, w);
      const double angle = 2 * std::atan2(std::sqrt(x * x + y * y + z * z), std::abs(w));
      return (actual.p - pose.p).Norm() <= 2 * epsilon && angle <= 2 * epsilon;
    };
  const auto finish = [&](const KDL::JntArray & q, bool relaxed) {
      output = q;
      report.minimum_margin = std::numeric_limits<double>::infinity();
      for (unsigned int j = 0; j < q.rows(); ++j) {
        report.minimum_margin = std::min(report.minimum_margin,
          std::min(q(j) - lower(j), upper(j) - q(j)));
      }
      report.relaxed = relaxed;
      if (diagnostics) {*diagnostics = report;}
      return 0;
    };
  constexpr double preferred_margin = 0.08726646259971647;  // Five degrees.
  constexpr double boundary_margin = 0.0001;
  KDL::JntArray original(seed.rows());
  ++report.attempts;
  const int original_code = primary.CartToJnt(seed, pose, original);
  if (original_code >= 0 && fits(original, preferred_margin)) {return finish(original, false);}

  KDL::JntArray preferred(seed.rows());
  bool have_preferred = false;
  bool preferred_relaxed = false;
  for (double margin : {preferred_margin, preferred_margin / 10, boundary_margin}) {
    KDL::JntArray lo(seed.rows()), hi(seed.rows());
    for (unsigned int j = 0; j < seed.rows(); ++j) {
      const double inset = std::min(margin, 0.05 * (upper(j) - lower(j)));
      lo(j) = lower(j) + inset;
      hi(j) = upper(j) - inset;
    }
    KDL::ChainIkSolverPos_NR_JL solver(chain, lo, hi, fk, velocity, iterations, epsilon);
    std::vector<KDL::JntArray> seeds{seed};
    // Bounded and deterministic: nearby redundant configurations, then centre.
    for (unsigned int j = 0; j < seed.rows(); ++j) {
      for (double delta : {-0.35, 0.35}) {
        auto alternative = seed;
        alternative(j) += delta;
        seeds.push_back(alternative);
      }
    }
    auto centre = seed;
    for (unsigned int j = 0; j < seed.rows(); ++j) {centre(j) = 0.5 * (lo(j) + hi(j));}
    seeds.push_back(centre);
    KDL::JntArray best(seed.rows());
    double best_distance = std::numeric_limits<double>::infinity();
    for (auto trial : seeds) {
      for (unsigned int j = 0; j < trial.rows(); ++j) {
        trial(j) = std::clamp(trial(j), lo(j), hi(j));
      }
      KDL::JntArray candidate(seed.rows());
      ++report.attempts;
      if (solver.CartToJnt(trial, pose, candidate) < 0 || !fits(candidate, margin)) {continue;}
      double distance = 0;
      for (unsigned int j = 0; j < trial.rows(); ++j) {
        distance += std::pow(candidate(j) - seed(j), 2);
      }
      if (distance < best_distance) {best = candidate; best_distance = distance;}
      // An inward solve from the previous configuration preserves continuity.
      if (best_distance < 0.01 * 0.01) {break;}
    }
    if (std::isfinite(best_distance)) {
      if (!have_preferred) {
        preferred = best;
        have_preferred = true;
        preferred_relaxed = margin < preferred_margin;
      }
      double largest_change = 0;
      for (unsigned int j = 0; j < best.rows(); ++j) {
        largest_change = std::max(largest_change, std::abs(best(j) - seed(j)));
      }
      // A larger inward margin must not introduce an avoidable branch jump.
      // Try smaller margins before rejecting a previously continuous motion.
      if (largest_change <= continuity_limit) {return finish(best, margin < preferred_margin);}
    }
  }
  if (have_preferred) {return finish(preferred, preferred_relaxed);}
  if (diagnostics) {*diagnostics = report;}
  // A hard-boundary solution must not become an executable command.
  return original_code < 0 ? original_code : -100;
}
}  // namespace waypoint_telep
#endif  // WAYPOINT_TELEP__LIMIT_IK_HPP_
