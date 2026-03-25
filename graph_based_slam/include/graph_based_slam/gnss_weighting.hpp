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

#ifndef GRAPH_BASED_SLAM__GNSS_WEIGHTING_HPP_
#define GRAPH_BASED_SLAM__GNSS_WEIGHTING_HPP_

#include <algorithm>
#include <cmath>
#include <limits>

#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace graphslam
{
namespace detail
{

struct GnssWeightingConfig
{
  double base_info_weight {1.0};
  double vertical_weight_scale {0.1};
  bool use_covariance_weighting {true};
  double covariance_min_variance_m2 {0.01};
  double covariance_max_variance_m2 {25.0};
  double rtk_fix_max_horizontal_stddev_m {0.3};
  double rtk_fix_weight_scale {3.0};
  double non_rtk_weight_scale {1.0};
};

struct GnssConstraintWeights
{
  double info_x {1.0};
  double info_y {1.0};
  double info_z {0.1};
  bool covariance_valid {false};
  bool rtk_like {false};
  double horizontal_stddev_m {std::numeric_limits<double>::quiet_NaN()};
};

struct GnssTimestampResolution
{
  double stamp_sec {0.0};
  bool used_fallback {false};
};

inline double clampVariance(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

inline bool hasKnownGnssCovariance(const sensor_msgs::msg::NavSatFix & msg)
{
  if (msg.position_covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
    return false;
  }
  const double var_x = msg.position_covariance[0];
  const double var_y = msg.position_covariance[4];
  const double var_z = msg.position_covariance[8];
  return std::isfinite(var_x) && std::isfinite(var_y) && std::isfinite(var_z) &&
         var_x > 0.0 && var_y > 0.0 && var_z > 0.0;
}

inline GnssConstraintWeights computeGnssConstraintWeights(
  const sensor_msgs::msg::NavSatFix & msg,
  const GnssWeightingConfig & config)
{
  GnssConstraintWeights weights;
  weights.info_x = config.base_info_weight * config.non_rtk_weight_scale;
  weights.info_y = config.base_info_weight * config.non_rtk_weight_scale;
  weights.info_z =
    config.base_info_weight * config.vertical_weight_scale * config.non_rtk_weight_scale;

  if (!config.use_covariance_weighting || !hasKnownGnssCovariance(msg)) {
    return weights;
  }

  const double var_x = clampVariance(
    msg.position_covariance[0],
    config.covariance_min_variance_m2,
    config.covariance_max_variance_m2);
  const double var_y = clampVariance(
    msg.position_covariance[4],
    config.covariance_min_variance_m2,
    config.covariance_max_variance_m2);
  const double var_z = clampVariance(
    msg.position_covariance[8],
    config.covariance_min_variance_m2,
    config.covariance_max_variance_m2);

  weights.covariance_valid = true;
  weights.horizontal_stddev_m = std::sqrt(std::max(var_x, var_y));
  weights.rtk_like = weights.horizontal_stddev_m <= config.rtk_fix_max_horizontal_stddev_m;

  const double class_scale =
    weights.rtk_like ? config.rtk_fix_weight_scale : config.non_rtk_weight_scale;
  weights.info_x = config.base_info_weight * class_scale / var_x;
  weights.info_y = config.base_info_weight * class_scale / var_y;
  weights.info_z = config.base_info_weight * config.vertical_weight_scale * class_scale / var_z;
  return weights;
}

inline GnssTimestampResolution resolveGnssMeasurementStamp(
  double header_stamp_sec,
  double fallback_stamp_sec,
  double max_skew_sec)
{
  GnssTimestampResolution resolved;
  resolved.stamp_sec = header_stamp_sec;
  if (!std::isfinite(header_stamp_sec) || header_stamp_sec <= 0.0) {
    resolved.stamp_sec = fallback_stamp_sec;
    resolved.used_fallback = true;
    return resolved;
  }
  if (
    std::isfinite(fallback_stamp_sec) &&
    max_skew_sec > 0.0 &&
    std::abs(header_stamp_sec - fallback_stamp_sec) > max_skew_sec)
  {
    resolved.stamp_sec = fallback_stamp_sec;
    resolved.used_fallback = true;
  }
  return resolved;
}

}  // namespace detail
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__GNSS_WEIGHTING_HPP_
