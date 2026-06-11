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

#ifndef GRAPH_BASED_SLAM__GNSS_ALIGNMENT_HPP_
#define GRAPH_BASED_SLAM__GNSS_ALIGNMENT_HPP_

// The odometry frame's x axis is the robot's initial heading; GNSS anchors
// live in ENU where x is east. Estimate the planar rigid transform between
// the two from matched (odometry position, ENU position) pairs so the pose
// graph can be moved into the ENU frame before anchors are applied (see
// docs/research/gnss-constraint-first-validation.md for what happens
// without this: the anchors shear the map instead of georeferencing it).

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Core>  // NOLINT(build/include_order)

namespace graphslam
{
namespace gnss_alignment
{

struct PlanarAlignment
{
  bool valid {false};
  // T maps odometry-frame positions into the ENU frame:
  // enu_xy = R(yaw_rad) * odom_xy + translation.
  double yaw_rad {0.0};
  Eigen::Vector2d translation {Eigen::Vector2d::Zero()};
  // Diagnostics: the spread of the odometry-side track that supported the
  // estimate, and the post-fit RMS residual.
  double baseline_m {0.0};
  double rms_residual_m {0.0};
};

// Closed-form 2-D rigid alignment (rotation + translation, no scale):
// the Umeyama/Kabsch solution on the horizontal components. Requires at
// least `min_pairs` matches whose odometry-side spread (max pairwise
// distance from the centroid, doubled) reaches `min_baseline_m`; a short
// baseline cannot observe yaw.
inline PlanarAlignment estimatePlanarAlignment(
  const std::vector<Eigen::Vector2d> & odom_xy,
  const std::vector<Eigen::Vector2d> & enu_xy,
  int min_pairs = 10,
  double min_baseline_m = 5.0)
{
  PlanarAlignment result;
  const int n = static_cast<int>(std::min(odom_xy.size(), enu_xy.size()));
  if (n < std::max(min_pairs, 2)) {
    return result;
  }

  Eigen::Vector2d odom_mean = Eigen::Vector2d::Zero();
  Eigen::Vector2d enu_mean = Eigen::Vector2d::Zero();
  for (int i = 0; i < n; ++i) {
    odom_mean += odom_xy[i];
    enu_mean += enu_xy[i];
  }
  odom_mean /= static_cast<double>(n);
  enu_mean /= static_cast<double>(n);

  double max_dev_sq = 0.0;
  Eigen::Matrix2d cross = Eigen::Matrix2d::Zero();
  for (int i = 0; i < n; ++i) {
    const Eigen::Vector2d od = odom_xy[i] - odom_mean;
    const Eigen::Vector2d ed = enu_xy[i] - enu_mean;
    cross += ed * od.transpose();
    max_dev_sq = std::max(max_dev_sq, od.squaredNorm());
  }
  result.baseline_m = 2.0 * std::sqrt(max_dev_sq);
  if (result.baseline_m < min_baseline_m) {
    return result;
  }

  // Planar Kabsch: yaw from the 2x2 cross-covariance.
  const double s = cross(1, 0) - cross(0, 1);
  const double c = cross(0, 0) + cross(1, 1);
  if (s == 0.0 && c == 0.0) {
    return result;
  }
  result.yaw_rad = std::atan2(s, c);

  const double cy = std::cos(result.yaw_rad);
  const double sy = std::sin(result.yaw_rad);
  Eigen::Matrix2d rot;
  rot << cy, -sy, sy, cy;
  result.translation = enu_mean - rot * odom_mean;

  double residual_sq_sum = 0.0;
  for (int i = 0; i < n; ++i) {
    const Eigen::Vector2d mapped = rot * odom_xy[i] + result.translation;
    residual_sq_sum += (mapped - enu_xy[i]).squaredNorm();
  }
  result.rms_residual_m = std::sqrt(residual_sq_sum / static_cast<double>(n));
  result.valid = true;
  return result;
}

}  // namespace gnss_alignment
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__GNSS_ALIGNMENT_HPP_
