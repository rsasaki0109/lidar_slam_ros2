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

#ifndef GRAPH_BASED_SLAM__TRAJECTORY_REVISIT_SEGMENTATION_HPP_
#define GRAPH_BASED_SLAM__TRAJECTORY_REVISIT_SEGMENTATION_HPP_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>  // NOLINT(build/include_order)

namespace graphslam
{
namespace map_thickness
{

struct RevisitSegmentationConfig
{
  double match_radius_m {3.0};
  double min_prior_travel_m {20.0};
  double exit_hysteresis_travel_m {5.0};
  double min_epoch_separation_m {10.0};
};

struct RevisitSegmentationResult
{
  std::vector<std::int64_t> revisit_ids;
  std::vector<double> cumulative_travel_m;
  std::vector<std::size_t> epoch_start_indices;
  int revisit_epoch_count {0};
  int matched_scan_count {0};
};

inline RevisitSegmentationResult segmentTrajectoryRevisits(
  const std::vector<Eigen::Vector3d> & positions,
  const RevisitSegmentationConfig & input_config = RevisitSegmentationConfig())
{
  RevisitSegmentationResult result;
  result.revisit_ids.resize(positions.size(), 0);
  result.cumulative_travel_m.resize(positions.size(), 0.0);
  if (positions.empty()) {
    return result;
  }
  result.epoch_start_indices.push_back(0U);

  RevisitSegmentationConfig config = input_config;
  config.match_radius_m = std::max(0.0, config.match_radius_m);
  config.min_prior_travel_m = std::max(0.0, config.min_prior_travel_m);
  config.exit_hysteresis_travel_m = std::max(0.0, config.exit_hysteresis_travel_m);
  config.min_epoch_separation_m = std::max(0.0, config.min_epoch_separation_m);

  for (std::size_t i = 1; i < positions.size(); ++i) {
    result.cumulative_travel_m[i] = result.cumulative_travel_m[i - 1] +
      (positions[i] - positions[i - 1]).norm();
  }

  std::int64_t epoch = 0;
  bool overlap_active = false;
  double last_overlap_travel = 0.0;
  double last_epoch_start_travel = -config.min_epoch_separation_m;
  const double match_radius_squared = config.match_radius_m * config.match_radius_m;
  for (std::size_t i = 0; i < positions.size(); ++i) {
    bool has_prior_match = false;
    for (std::size_t j = 0; j < i; ++j) {
      const double travel_separation =
        result.cumulative_travel_m[i] - result.cumulative_travel_m[j];
      if (travel_separation < config.min_prior_travel_m) {
        continue;
      }
      if ((positions[i] - positions[j]).squaredNorm() <= match_radius_squared) {
        has_prior_match = true;
        break;
      }
    }

    const double travel = result.cumulative_travel_m[i];
    if (has_prior_match) {
      ++result.matched_scan_count;
      if (!overlap_active &&
        travel - last_epoch_start_travel >= config.min_epoch_separation_m)
      {
        ++epoch;
        last_epoch_start_travel = travel;
        result.epoch_start_indices.push_back(i);
      }
      overlap_active = true;
      last_overlap_travel = travel;
    } else {
      if (overlap_active &&
        travel - last_overlap_travel >= config.exit_hysteresis_travel_m)
      {
        overlap_active = false;
      }
    }
    result.revisit_ids[i] = epoch;
  }
  result.revisit_epoch_count = static_cast<int>(epoch + 1);
  return result;
}

}  // namespace map_thickness
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__TRAJECTORY_REVISIT_SEGMENTATION_HPP_
