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

#ifndef GRAPH_BASED_SLAM__NDT_TRAJECTORY_IO_HPP_
#define GRAPH_BASED_SLAM__NDT_TRAJECTORY_IO_HPP_

#include <Eigen/Geometry>

#include <cmath>
#include <cstddef>
#include <iomanip>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace graphslam::ndt_localization
{

struct RegisteredPose
{
  std::size_t pair_index {0U};
  double stamp_sec {0.0};
  bool converged {false};
  Eigen::Vector3d translation {Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation {Eigen::Quaterniond::Identity()};
};

inline std::string registeredPoseTumLine(const RegisteredPose & pose)
{
  Eigen::Quaterniond orientation = pose.orientation;
  if (!std::isfinite(pose.stamp_sec) || !pose.translation.allFinite() ||
    !orientation.coeffs().allFinite() || !(orientation.norm() > 0.0))
  {
    throw std::invalid_argument("registered NDT pose is not finite");
  }
  orientation.normalize();
  if (orientation.w() < 0.0) {
    orientation.coeffs() *= -1.0;
  }
  if (orientation.x() == 0.0) {orientation.x() = 0.0;}
  if (orientation.y() == 0.0) {orientation.y() = 0.0;}
  if (orientation.z() == 0.0) {orientation.z() = 0.0;}
  if (orientation.w() == 0.0) {orientation.w() = 0.0;}
  std::ostringstream line;
  line << std::setprecision(17) << pose.stamp_sec << ' ' << pose.translation.x() << ' ' <<
    pose.translation.y() << ' ' << pose.translation.z() << ' ' << orientation.x() << ' ' <<
    orientation.y() << ' ' << orientation.z() << ' ' << orientation.w();
  return line.str();
}

inline void writeRegisteredPoseTum(
  std::ostream & output, const std::vector<RegisteredPose> & poses)
{
  for (const RegisteredPose & pose : poses) {
    output << registeredPoseTumLine(pose) << '\n';
  }
}

inline Eigen::Affine3d regularizePoseUpdate(
  const Eigen::Affine3d & initial, const Eigen::Matrix4d & optimized,
  const double update_gain)
{
  if (!initial.matrix().allFinite() || !optimized.allFinite() ||
    !std::isfinite(update_gain) || update_gain < 0.0 || update_gain > 1.0)
  {
    throw std::invalid_argument("NDT pose update and gain must be finite and gain in [0, 1]");
  }
  if (update_gain == 0.0) {return initial;}
  if (update_gain == 1.0) {return Eigen::Affine3d(optimized);}
  Eigen::Quaterniond initial_orientation(initial.rotation());
  Eigen::Quaterniond optimized_orientation(optimized.block<3, 3>(0, 0));
  if (!(initial_orientation.norm() > 0.0) || !(optimized_orientation.norm() > 0.0)) {
    throw std::invalid_argument("NDT pose update contains an invalid rotation");
  }
  initial_orientation.normalize();
  optimized_orientation.normalize();
  Eigen::Affine3d guarded = Eigen::Affine3d::Identity();
  guarded.translation() = initial.translation() + update_gain *
    (optimized.block<3, 1>(0, 3) - initial.translation());
  guarded.linear() = initial_orientation.slerp(update_gain,
      optimized_orientation).toRotationMatrix();
  return guarded;
}

}  // namespace graphslam::ndt_localization

#endif  // GRAPH_BASED_SLAM__NDT_TRAJECTORY_IO_HPP_
