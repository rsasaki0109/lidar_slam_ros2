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

#ifndef SCANMATCHER_ODOM_PRIOR_UTILS_HPP_
#define SCANMATCHER_ODOM_PRIOR_UTILS_HPP_

#include <algorithm>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace graphslam
{
namespace odom_prior
{

inline double wrapAngleRad(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

inline Eigen::Matrix3f yawRotation(float yaw_rad)
{
  Eigen::AngleAxisf yaw_axis(yaw_rad, Eigen::Vector3f::UnitZ());
  return yaw_axis.toRotationMatrix();
}

inline Eigen::Matrix4f filterAndBlendDelta(
  const Eigen::Matrix4f & delta,
  const bool planar,
  const bool translation_only,
  const double weight)
{
  const float clamped_weight = static_cast<float>(std::clamp(weight, 0.0, 1.0));
  Eigen::Matrix4f filtered = Eigen::Matrix4f::Identity();
  Eigen::Vector3f translation = delta.block<3, 1>(0, 3);
  Eigen::Matrix3f rotation = delta.block<3, 3>(0, 0);

  if (planar) {
    translation.z() = 0.0f;
    const float yaw_rad = std::atan2(rotation(1, 0), rotation(0, 0));
    rotation = yawRotation(yaw_rad);
  }

  translation *= clamped_weight;

  filtered.block<3, 1>(0, 3) = translation;
  if (translation_only || clamped_weight <= 0.0f) {
    filtered.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();
    return filtered;
  }

  Eigen::Quaternionf target_quat(rotation);
  if (target_quat.norm() < 1e-6f) {
    filtered.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();
    return filtered;
  }
  target_quat.normalize();
  const Eigen::Quaternionf blended =
    Eigen::Quaternionf::Identity().slerp(clamped_weight, target_quat).normalized();
  filtered.block<3, 3>(0, 0) = blended.toRotationMatrix();
  return filtered;
}

}  // namespace odom_prior
}  // namespace graphslam

#endif  // SCANMATCHER_ODOM_PRIOR_UTILS_HPP_
