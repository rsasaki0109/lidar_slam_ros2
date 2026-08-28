// Copyright 2026 Sasaki
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef SCANMATCHER__REGISTRATION_RUNTIME_HPP_
#define SCANMATCHER__REGISTRATION_RUNTIME_HPP_

#include <cmath>

#include <lidarslam_plugin_interfaces/registration.hpp>

namespace graphslam
{

inline bool registrationRuntimeUsesHostPreparedTarget(
  const lidarslam::plugins::registration::TargetPolicy policy)
{
  return policy == lidarslam::plugins::registration::TargetPolicy::kAcceptHostPrepared;
}

inline bool registrationRuntimeUsesRawTarget(
  const lidarslam::plugins::registration::TargetPolicy policy)
{
  return policy == lidarslam::plugins::registration::TargetPolicy::kRequiresRawTarget;
}

// Convert the cached correspondence contract into the scalar consumed by the
// adaptive threshold policy.  GICP's contract is explicitly the square-root
// fitness proxy; NDT reports its mean correspondence distance directly.
inline bool registrationRuntimeMetricValue(
  const lidarslam::plugins::registration::AlignmentResult & result,
  const lidarslam::plugins::registration::CorrespondenceMetric metric,
  double * value)
{
  if (value == nullptr) {
    return false;
  }
  using lidarslam::plugins::registration::CorrespondenceMetric;
  if (metric == CorrespondenceMetric::kMeanDistance) {
    if (
      !result.diagnostics.mean_correspondence_distance_valid ||
      !std::isfinite(result.diagnostics.mean_correspondence_distance))
    {
      return false;
    }
    *value = result.diagnostics.mean_correspondence_distance;
    return true;
  }
  if (metric == CorrespondenceMetric::kSquareRootFitnessProxy) {
    if (!std::isfinite(result.fitness_score) || result.fitness_score < 0.0) {
      return false;
    }
    *value = std::sqrt(result.fitness_score);
    return std::isfinite(*value);
  }
  return false;
}

}  // namespace graphslam

#endif  // SCANMATCHER__REGISTRATION_RUNTIME_HPP_
