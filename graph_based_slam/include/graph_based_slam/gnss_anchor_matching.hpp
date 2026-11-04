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

#ifndef GRAPH_BASED_SLAM__GNSS_ANCHOR_MATCHING_HPP_
#define GRAPH_BASED_SLAM__GNSS_ANCHOR_MATCHING_HPP_

// Pick the GNSS position that anchors one submap. The submap stamp falls
// between two fixes; at walking speed and 10 Hz the nearest fix is up to
// ~5 cm away from where the sensor was, so the fixes on both sides are
// interpolated when they are close enough in time. Without interpolation
// (max_interpolation_gap_sec <= 0) this is the historical nearest-fix match.

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Core>  // NOLINT(build/include_order)

namespace graphslam
{
namespace detail
{

struct GnssSample
{
  double stamp {0.0};
  Eigen::Vector3d position {Eigen::Vector3d::Zero()};
  Eigen::Vector3d info {Eigen::Vector3d::Zero()};
  bool rtk_like {false};
};

struct GnssAnchorMatchConfig
{
  // Nearest-fix fallback window (the historical rule: 1 s).
  double max_nearest_dt_sec {1.0};
  // Interpolate between the fixes either side of the stamp when they are at
  // most this far apart; <= 0 disables interpolation.
  double max_interpolation_gap_sec {0.0};
};

struct GnssAnchorMatch
{
  bool valid {false};
  bool interpolated {false};
  Eigen::Vector3d position {Eigen::Vector3d::Zero()};
  Eigen::Vector3d info {Eigen::Vector3d::Zero()};
  bool rtk_like {false};
};

inline bool gnssSampleStampLess(const GnssSample & a, const GnssSample & b)
{
  return a.stamp < b.stamp;
}

// samples must be sorted by stamp (std::stable_sort with gnssSampleStampLess).
inline GnssAnchorMatch matchGnssAnchor(
  const std::vector<GnssSample> & samples, double stamp, const GnssAnchorMatchConfig & config)
{
  GnssAnchorMatch match;
  if (samples.empty() || !std::isfinite(stamp)) {
    return match;
  }
  GnssSample probe;
  probe.stamp = stamp;
  // First sample with sample.stamp >= stamp.
  const auto after = std::lower_bound(
    samples.begin(), samples.end(), probe, gnssSampleStampLess);

  if (config.max_interpolation_gap_sec > 0.0 && after != samples.end() &&
    after != samples.begin())
  {
    const GnssSample & a = *(after - 1);
    const GnssSample & b = *after;
    const double gap = b.stamp - a.stamp;
    if (gap > 0.0 && gap <= config.max_interpolation_gap_sec) {
      const double w = (stamp - a.stamp) / gap;
      match.valid = true;
      match.interpolated = true;
      match.position = (1.0 - w) * a.position + w * b.position;
      // Conservative: the weaker of the two fixes on each axis.
      match.info = a.info.cwiseMin(b.info);
      match.rtk_like = a.rtk_like && b.rtk_like;
      return match;
    }
  }

  // Nearest fix; on a tie the earlier one wins (the historical scan order).
  const GnssSample * best = nullptr;
  double best_dt = std::numeric_limits<double>::max();
  if (after != samples.begin()) {
    const GnssSample & a = *(after - 1);
    best_dt = std::abs(stamp - a.stamp);
    best = &a;
  }
  if (after != samples.end() && std::abs(after->stamp - stamp) < best_dt) {
    best_dt = std::abs(after->stamp - stamp);
    best = &*after;
  }
  if (best == nullptr || best_dt > config.max_nearest_dt_sec) {
    return match;
  }
  match.valid = true;
  match.position = best->position;
  match.info = best->info;
  match.rtk_like = best->rtk_like;
  return match;
}

}  // namespace detail
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__GNSS_ANCHOR_MATCHING_HPP_
