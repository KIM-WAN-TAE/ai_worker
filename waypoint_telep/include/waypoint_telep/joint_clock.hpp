#ifndef WAYPOINT_TELEP__JOINT_CLOCK_HPP_
#define WAYPOINT_TELEP__JOINT_CLOCK_HPP_

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace waypoint_telep
{
// Reparameterize an existing curve. These are integration nodes, not waypoints.
struct ClockNode
{
  double s;
  double cap;
  double acceleration;
  double rate{0.0};
  double t{0.0};
  double reference_rate{1.0};
};

inline void retime_clock(std::vector<ClockNode> & nodes, double first, double last)
{
  if (nodes.size() < 2) {throw std::invalid_argument("clock needs two nodes");}
  std::vector<double> w;
  for (const auto & n : nodes) {
    if (!std::isfinite(n.cap) || n.cap <= 0.0 || n.acceleration <= 0.0) {
      throw std::invalid_argument("invalid clock envelope");
    }
    w.push_back(n.cap * n.cap);
  }
  if (w.front() + 1.e-8 < first * first || w.back() + 1.e-8 < last * last) {
    throw std::invalid_argument("preserved boundary exceeds the timing envelope");
  }
  w.front() = first * first;
  w.back() = last * last;
  for (std::size_t i = 1; i < nodes.size(); ++i) {
    const double ds = nodes[i].s - nodes[i-1].s;
    if (ds <= 0.0) {throw std::invalid_argument("clock parameters must increase");}
    w[i] = std::min(w[i], w[i-1] +
      2.0 * std::min(nodes[i].acceleration, nodes[i-1].acceleration) * ds);
  }
  for (std::size_t i = nodes.size()-1; i > 0; --i) {
    w[i-1] = std::min(w[i-1], w[i] +
      2.0 * std::min(nodes[i].acceleration, nodes[i-1].acceleration) *
      (nodes[i].s - nodes[i-1].s));
  }
  if (std::abs(w.front()-first*first) > 1.e-7 ||
    std::abs(w.back()-last*last) > 1.e-7)
  {
    throw std::invalid_argument("insufficient distance for preserved boundary speed");
  }
  nodes.front().t = 0.0;
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    nodes[i].rate = std::sqrt(w[i]);
    if (i) {
      const double sum = nodes[i].rate + nodes[i-1].rate;
      if (sum <= 1.e-12) {throw std::invalid_argument("untraversable clock interval");}
      nodes[i].t = nodes[i-1].t + 2.0 * (nodes[i].s-nodes[i-1].s) / sum;
    }
  }
}

// Widen only the constraint-induced slowdowns. Filtering the clock rate itself
// would unnecessarily disturb constant TCP speed on a curved joint trajectory.
inline void smooth_clock_slowdowns(std::vector<ClockNode> & nodes, std::size_t radius)
{
  const std::size_t count = nodes.size();
  const double first = nodes.front().rate, last = nodes.back().rate;
  std::vector<double> factor(count), eroded(count);
  for (std::size_t i = 0; i < count; ++i) {
    factor[i] = std::pow(nodes[i].rate / nodes[i].reference_rate, 2);
  }
  for (std::size_t i = 0; i < count; ++i) {
    const auto begin = i > radius ? i-radius : 0;
    const auto end = std::min(count-1, i+radius);
    eroded[i] = *std::min_element(factor.begin()+begin, factor.begin()+end+1);
  }
  for (std::size_t i = 0; i < count; ++i) {
    double sum = 0.0, weights = 0.0;
    for (int offset = -static_cast<int>(radius); offset <= static_cast<int>(radius);
      ++offset)
    {
      const auto j = static_cast<std::size_t>(std::clamp<long>(
        static_cast<long>(i)+offset, 0, static_cast<long>(count)-1));
      const double weight = static_cast<double>(radius+1)-std::abs(offset);
      sum += eroded[j]*weight;
      weights += weight;
    }
    const double edge = std::min(1.0,
      static_cast<double>(std::min(i, count-1-i))/(4.0*radius));
    const double blend = edge*edge*edge*(10.0-15.0*edge+6.0*edge*edge);
    const double w = (1.0-blend)*nodes[i].rate*nodes[i].rate +
      blend*sum/weights*nodes[i].reference_rate*nodes[i].reference_rate;
    nodes[i].cap = std::min(nodes[i].cap, std::sqrt(std::max(0.0, w)));
    // retime_clock accepts positive caps, even at a stopped endpoint.
    nodes[i].cap = std::max(nodes[i].cap, 1.e-12);
  }
  retime_clock(nodes, first, last);
}

// Constant clock acceleration on each integration interval gives continuous
// clock velocity, including zero-speed endpoints, without overshooting s.
inline double clock_parameter(const ClockNode & a, const ClockNode & b, double t)
{
  const double dt = b.t-a.t;
  const double u = std::clamp(t-a.t, 0.0, dt);
  return std::clamp(a.s+a.rate*u+0.5*(b.rate-a.rate)*u*u/dt, a.s, b.s);
}
}  // namespace waypoint_telep
#endif
