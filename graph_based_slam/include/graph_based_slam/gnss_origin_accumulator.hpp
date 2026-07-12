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

#ifndef GRAPH_BASED_SLAM__GNSS_ORIGIN_ACCUMULATOR_HPP_
#define GRAPH_BASED_SLAM__GNSS_ORIGIN_ACCUMULATOR_HPP_

#include <algorithm>
#include <cstddef>
#include <vector>

#include "graph_based_slam/gnss_geometry.hpp"

namespace graphslam
{
namespace detail
{

struct GnssOriginUpdate
{
  bool reset_after_jump {false};
  bool restarted_after_inconsistency {false};
  bool initialized {false};
  double deviation_m {0.0};
  std::size_t accepted_samples {0};
  GeodeticOrigin origin;
};

class GnssOriginAccumulator
{
public:
  void configure(int min_samples, double consistency_threshold_m)
  {
    min_samples_ = std::max(1, min_samples);
    consistency_threshold_m_ = consistency_threshold_m;
    candidates_.clear();
  }

  GnssOriginUpdate add(double latitude_deg, double longitude_deg, double altitude_m)
  {
    GnssOriginUpdate update;
    const GeodeticOrigin sample {latitude_deg, longitude_deg, altitude_m};

    if (!candidates_.empty()) {
      const GeodeticOrigin mean = meanOrigin();
      update.deviation_m = approximateGeodeticDistanceMeters(
        mean.latitude_deg, mean.longitude_deg, latitude_deg, longitude_deg);
      if (update.deviation_m > consistency_threshold_m_) {
        candidates_.clear();
        update.reset_after_jump = true;
      }
    }

    candidates_.push_back(sample);
    update.accepted_samples = candidates_.size();
    if (static_cast<int>(candidates_.size()) < min_samples_) {
      return update;
    }

    const GeodeticOrigin mean = meanOrigin();
    double max_deviation_m = 0.0;
    for (const auto & candidate : candidates_) {
      max_deviation_m = std::max(
        max_deviation_m,
        approximateGeodeticDistanceMeters(
          mean.latitude_deg, mean.longitude_deg,
          candidate.latitude_deg, candidate.longitude_deg));
    }
    update.deviation_m = max_deviation_m;

    if (max_deviation_m > consistency_threshold_m_) {
      candidates_.assign(1, sample);
      update.restarted_after_inconsistency = true;
      update.accepted_samples = candidates_.size();
      return update;
    }

    update.initialized = true;
    update.origin = mean;
    update.accepted_samples = candidates_.size();
    candidates_.clear();
    return update;
  }

private:
  GeodeticOrigin meanOrigin() const
  {
    GeodeticOrigin mean;
    for (const auto & candidate : candidates_) {
      mean.latitude_deg += candidate.latitude_deg;
      mean.longitude_deg += candidate.longitude_deg;
      mean.altitude_m += candidate.altitude_m;
    }
    const double count = static_cast<double>(candidates_.size());
    mean.latitude_deg /= count;
    mean.longitude_deg /= count;
    mean.altitude_m /= count;
    return mean;
  }

  int min_samples_ {1};
  double consistency_threshold_m_ {20.0};
  std::vector<GeodeticOrigin> candidates_;
};

}  // namespace detail
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__GNSS_ORIGIN_ACCUMULATOR_HPP_
