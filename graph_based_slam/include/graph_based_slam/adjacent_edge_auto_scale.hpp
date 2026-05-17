// Copyright 2026 Sasaki
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef GRAPH_BASED_SLAM__ADJACENT_EDGE_AUTO_SCALE_HPP_
#define GRAPH_BASED_SLAM__ADJACENT_EDGE_AUTO_SCALE_HPP_

#include <algorithm>
#include <cstddef>
#include <vector>

namespace graphslam
{
namespace detail
{

// NIS-driven auto-scaling for the scalar `adjacent_edge_info_weight` used on
// adjacent submap edges in graph_based_slam. The idea (Level 1 of the
// covariance-driven roadmap): keep the existing identity-information matrix
// shape, but multiply its overall scale so that the median chi-squared (NIS)
// of adjacent edges after optimisation lands near the SE(3) degrees of freedom
// target (~6). This auto-balances "trust the LIO front-end vs trust the graph
// adjustment" between datasets without manual tuning per dataset.
//
// The maths:
//   chi2 = e^T * (s * I) * e = s * e^T * I * e
// so if the current median chi2 is c and the target is t, the *next* scale
// should be roughly s_next = s * (t / c). An EMA smooths the update against
// chi2 noise; min/max clamps protect against pathological feedback loops.

struct AutoScaleConfig
{
  // Target median chi-squared per adjacent edge. SE(3) edges have 6 DoF, so 6
  // is the natural target; users can lower it (closer to 3) if they want the
  // backend to lean more on the LIO front-end.
  double target_nis {6.0};

  // EMA mixing factor in [0, 1]. 0 = no adaptation, 1 = full jump to the
  // chi-squared-implied scale. 0.3 is a conservative default.
  double ema_alpha {0.3};

  // Clamp range for the resulting scale. Defaults span four orders of
  // magnitude around the historical hand-tuned defaults (100 / 1000).
  double min_scale {1.0};
  double max_scale {1.0e6};
};

// Median of a vector of doubles. Returns 0.0 when the input is empty.
inline double medianChi2(std::vector<double> values)
{
  if (values.empty()) {
    return 0.0;
  }
  const std::size_t mid = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + mid, values.end());
  const double upper = values[mid];
  if (values.size() % 2 == 1) {
    return upper;
  }
  // Even count: also locate the lower middle and average.
  std::nth_element(values.begin(), values.begin() + mid - 1, values.begin() + mid);
  const double lower = values[mid - 1];
  return 0.5 * (lower + upper);
}

// Compute the next scale value from the current scale, the post-optimisation
// median chi-squared across adjacent edges, and the config.
//
// Behaviour:
//   - empty median (no samples) or non-positive median → return current_scale
//     unchanged (preserve historical hand-tuned weight).
//   - target_nis non-positive → return current_scale unchanged.
//   - otherwise mix with EMA and clamp to [min_scale, max_scale].
inline double nextScale(
  double current_scale,
  double median_chi2,
  const AutoScaleConfig & cfg)
{
  if (current_scale <= 0.0) {
    return current_scale;
  }
  if (cfg.target_nis <= 0.0) {
    return current_scale;
  }
  if (median_chi2 <= 0.0) {
    return current_scale;
  }
  const double ratio = cfg.target_nis / median_chi2;
  const double implied = current_scale * ratio;
  const double alpha = std::min(std::max(cfg.ema_alpha, 0.0), 1.0);
  const double mixed = (1.0 - alpha) * current_scale + alpha * implied;
  const double clamped = std::min(std::max(mixed, cfg.min_scale), cfg.max_scale);
  return clamped;
}

}  // namespace detail
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__ADJACENT_EDGE_AUTO_SCALE_HPP_
