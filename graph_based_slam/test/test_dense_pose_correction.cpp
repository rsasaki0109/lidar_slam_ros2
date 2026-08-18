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

#include <gtest/gtest.h>

#include <cstddef>
#include <stdexcept>
#include <vector>

#include "graph_based_slam/dense_pose_correction.hpp"

namespace
{

using graphslam::dense_pose_correction::TimedPose;
using graphslam::dense_pose_correction::applyDenseCorrections;
using graphslam::dense_pose_correction::buildCorrectionAnchors;

constexpr double kPi = 3.14159265358979323846;

TimedPose pose(const double stamp, const double x, const double yaw)
{
  TimedPose sample;
  sample.stamp_sec = stamp;
  sample.pose.translation() = Eigen::Vector3d(x, 0.0, 0.0);
  sample.pose.linear() =
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return sample;
}

}  // namespace

TEST(DensePoseCorrection, IdentityAnchorsPreserveDensePosesExactly)
{
  const std::vector<TimedPose> anchors {pose(0.0, 1.0, 0.2), pose(2.0, 3.0, -0.4)};
  const std::vector<TimedPose> dense {pose(-1.0, 0.0, 0.1), pose(1.0, 2.0, 0.3),
    pose(3.0, 4.0, 0.5)};
  const auto corrections = buildCorrectionAnchors(anchors, anchors);
  const std::vector<TimedPose> corrected = applyDenseCorrections(dense, corrections);

  ASSERT_EQ(corrected.size(), dense.size());
  for (std::size_t i = 0; i < dense.size(); ++i) {
    EXPECT_TRUE(corrected[i].pose.matrix().isApprox(dense[i].pose.matrix(), 0.0));
  }
}

TEST(DensePoseCorrection, InterpolatesWorldSideTranslationAndRotation)
{
  const std::vector<TimedPose> raw {pose(0.0, 0.0, 0.0), pose(2.0, 0.0, 0.0)};
  const std::vector<TimedPose> corrected_anchors {
    pose(0.0, 0.0, 0.0), pose(2.0, 2.0, kPi)};
  const std::vector<TimedPose> dense {pose(1.0, 1.0, 0.0)};
  const std::vector<TimedPose> corrected = applyDenseCorrections(
    dense, buildCorrectionAnchors(raw, corrected_anchors));

  ASSERT_EQ(corrected.size(), 1U);
  EXPECT_NEAR(corrected[0].pose.translation().x(), 1.0, 1.0e-12);
  EXPECT_NEAR(corrected[0].pose.translation().y(), 1.0, 1.0e-12);
  EXPECT_NEAR(
    Eigen::AngleAxisd(corrected[0].pose.rotation()).angle(), kPi / 2.0, 1.0e-12);
}

TEST(DensePoseCorrection, ClampsCorrectionsOutsideAnchorRange)
{
  const std::vector<TimedPose> raw {pose(0.0, 0.0, 0.0), pose(2.0, 0.0, 0.0)};
  const std::vector<TimedPose> corrected_anchors {
    pose(0.0, 1.0, 0.0), pose(2.0, 3.0, 0.0)};
  const std::vector<TimedPose> dense {pose(-1.0, 0.0, 0.0), pose(3.0, 0.0, 0.0)};
  const std::vector<TimedPose> corrected = applyDenseCorrections(
    dense, buildCorrectionAnchors(raw, corrected_anchors));

  EXPECT_NEAR(corrected[0].pose.translation().x(), 1.0, 1.0e-12);
  EXPECT_NEAR(corrected[1].pose.translation().x(), 3.0, 1.0e-12);
}

TEST(DensePoseCorrection, RejectsNonMonotonicAnchors)
{
  const std::vector<TimedPose> raw {pose(1.0, 0.0, 0.0), pose(1.0, 1.0, 0.0)};
  EXPECT_THROW(buildCorrectionAnchors(raw, raw), std::invalid_argument);
}
