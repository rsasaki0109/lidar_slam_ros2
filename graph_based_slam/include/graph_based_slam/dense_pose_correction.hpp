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

#ifndef GRAPH_BASED_SLAM__DENSE_POSE_CORRECTION_HPP_
#define GRAPH_BASED_SLAM__DENSE_POSE_CORRECTION_HPP_

#include <cmath>
#include <stdexcept>
#include <vector>

#include <Eigen/Geometry>  // NOLINT(build/include_order)

namespace graphslam
{
namespace dense_pose_correction
{

struct TimedPose
{
  double stamp_sec {0.0};
  Eigen::Isometry3d pose {Eigen::Isometry3d::Identity()};
};

struct CorrectionAnchor
{
  double stamp_sec {0.0};
  Eigen::Vector3d translation {Eigen::Vector3d::Zero()};
  Eigen::Quaterniond rotation {Eigen::Quaterniond::Identity()};
};

inline Eigen::Quaterniond canonicalQuaternion(const Eigen::Quaterniond & input)
{
  Eigen::Quaterniond quaternion = input.normalized();
  if (quaternion.w() < 0.0) {
    quaternion.coeffs() *= -1.0;
  }
  return quaternion;
}

inline CorrectionAnchor makeCorrectionAnchor(
  const TimedPose & raw, const TimedPose & corrected)
{
  if (std::abs(raw.stamp_sec - corrected.stamp_sec) > 1.0e-9) {
    throw std::invalid_argument("raw and corrected anchor timestamps differ");
  }
  const Eigen::Isometry3d correction = corrected.pose * raw.pose.inverse();
  CorrectionAnchor anchor;
  anchor.stamp_sec = raw.stamp_sec;
  anchor.translation = correction.translation();
  anchor.rotation = canonicalQuaternion(Eigen::Quaterniond(correction.rotation()));

  const Eigen::AngleAxisd angle_axis(anchor.rotation);
  if (anchor.translation.norm() <= 1.0e-12 && std::abs(angle_axis.angle()) <= 1.0e-12) {
    anchor.translation.setZero();
    anchor.rotation = Eigen::Quaterniond::Identity();
  }
  return anchor;
}

inline std::vector<CorrectionAnchor> buildCorrectionAnchors(
  const std::vector<TimedPose> & raw,
  const std::vector<TimedPose> & corrected)
{
  if (raw.empty() || raw.size() != corrected.size()) {
    throw std::invalid_argument("raw and corrected anchor arrays must have equal non-zero size");
  }
  std::vector<CorrectionAnchor> anchors;
  anchors.reserve(raw.size());
  for (std::size_t i = 0; i < raw.size(); ++i) {
    if (i > 0U && raw[i].stamp_sec <= raw[i - 1U].stamp_sec) {
      throw std::invalid_argument("anchor timestamps must be strictly increasing");
    }
    anchors.push_back(makeCorrectionAnchor(raw[i], corrected[i]));
  }
  return anchors;
}

inline Eigen::Isometry3d correctionAt(
  const std::vector<CorrectionAnchor> & anchors, const double stamp_sec)
{
  if (anchors.empty()) {
    throw std::invalid_argument("correction anchors are empty");
  }
  const CorrectionAnchor * left = &anchors.front();
  const CorrectionAnchor * right = &anchors.front();
  if (stamp_sec >= anchors.back().stamp_sec) {
    left = &anchors.back();
    right = left;
  } else if (stamp_sec > anchors.front().stamp_sec) {
    std::size_t lo = 0U;
    std::size_t hi = anchors.size() - 1U;
    while (hi - lo > 1U) {
      const std::size_t middle = lo + (hi - lo) / 2U;
      if (anchors[middle].stamp_sec <= stamp_sec) {
        lo = middle;
      } else {
        hi = middle;
      }
    }
    left = &anchors[lo];
    right = &anchors[hi];
  }

  double alpha = 0.0;
  const double span = right->stamp_sec - left->stamp_sec;
  if (span > 0.0) {
    alpha = (stamp_sec - left->stamp_sec) / span;
  }
  Eigen::Isometry3d correction = Eigen::Isometry3d::Identity();
  correction.translation() =
    (1.0 - alpha) * left->translation + alpha * right->translation;
  correction.linear() = canonicalQuaternion(left->rotation.slerp(alpha,
        right->rotation)).toRotationMatrix();
  return correction;
}

inline std::vector<TimedPose> applyDenseCorrections(
  const std::vector<TimedPose> & dense_raw,
  const std::vector<CorrectionAnchor> & anchors)
{
  std::vector<TimedPose> result;
  result.reserve(dense_raw.size());
  for (const TimedPose & raw : dense_raw) {
    TimedPose corrected;
    corrected.stamp_sec = raw.stamp_sec;
    corrected.pose = correctionAt(anchors, raw.stamp_sec) * raw.pose;
    result.push_back(corrected);
  }
  return result;
}

}  // namespace dense_pose_correction
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__DENSE_POSE_CORRECTION_HPP_
