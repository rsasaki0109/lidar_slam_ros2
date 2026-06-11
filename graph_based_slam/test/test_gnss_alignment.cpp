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

#include <cmath>
#include <vector>

#include <Eigen/Core>  // NOLINT(build/include_order)

#include "graph_based_slam/gnss_alignment.hpp"

namespace graphslam
{
namespace
{

using gnss_alignment::estimatePlanarAlignment;

std::vector<Eigen::Vector2d> makeTrack()
{
  // An L-shaped 30 m track: enough baseline in both axes to observe yaw.
  std::vector<Eigen::Vector2d> track;
  for (int i = 0; i <= 15; ++i) {
    track.emplace_back(static_cast<double>(i), 0.0);
  }
  for (int i = 1; i <= 15; ++i) {
    track.emplace_back(15.0, static_cast<double>(i));
  }
  return track;
}

std::vector<Eigen::Vector2d> transformTrack(
  const std::vector<Eigen::Vector2d> & track, double yaw_rad, const Eigen::Vector2d & t)
{
  const double c = std::cos(yaw_rad);
  const double s = std::sin(yaw_rad);
  Eigen::Matrix2d rot;
  rot << c, -s, s, c;
  std::vector<Eigen::Vector2d> out;
  out.reserve(track.size());
  for (const auto & p : track) {
    out.push_back(rot * p + t);
  }
  return out;
}

TEST(GnssAlignment, RecoversKnownRotationAndTranslationExactly)
{
  const auto odom = makeTrack();
  const double yaw = 137.0 * M_PI / 180.0;
  const Eigen::Vector2d t(42.5, -17.25);
  const auto enu = transformTrack(odom, yaw, t);

  const auto a = estimatePlanarAlignment(odom, enu);
  ASSERT_TRUE(a.valid);
  EXPECT_NEAR(a.yaw_rad, yaw, 1e-12);
  EXPECT_NEAR(a.translation.x(), t.x(), 1e-9);
  EXPECT_NEAR(a.translation.y(), t.y(), 1e-9);
  EXPECT_NEAR(a.rms_residual_m, 0.0, 1e-9);
}

TEST(GnssAlignment, RecoversNegativeYaw)
{
  const auto odom = makeTrack();
  const double yaw = -92.0 * M_PI / 180.0;
  const auto enu = transformTrack(odom, yaw, Eigen::Vector2d(3.0, 4.0));

  const auto a = estimatePlanarAlignment(odom, enu);
  ASSERT_TRUE(a.valid);
  EXPECT_NEAR(a.yaw_rad, yaw, 1e-12);
}

TEST(GnssAlignment, RejectsTooFewPairs)
{
  std::vector<Eigen::Vector2d> odom = {{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}};
  const auto enu = transformTrack(odom, 0.5, Eigen::Vector2d(1.0, 1.0));
  const auto a = estimatePlanarAlignment(odom, enu, /*min_pairs=*/ 10);
  EXPECT_FALSE(a.valid);
}

TEST(GnssAlignment, RejectsShortBaseline)
{
  // 30 pairs but everything within ~1 m: yaw is unobservable.
  std::vector<Eigen::Vector2d> odom;
  for (int i = 0; i < 30; ++i) {
    odom.emplace_back(0.01 * i, 0.005 * i);
  }
  const auto enu = transformTrack(odom, 1.0, Eigen::Vector2d(5.0, 5.0));
  const auto a = estimatePlanarAlignment(odom, enu, 10, /*min_baseline_m=*/ 5.0);
  EXPECT_FALSE(a.valid);
  EXPECT_LT(a.baseline_m, 5.0);
}

TEST(GnssAlignment, NoisyTrackReportsResidualButStaysAccurate)
{
  const auto odom = makeTrack();
  const double yaw = 0.7;
  auto enu = transformTrack(odom, yaw, Eigen::Vector2d(-8.0, 2.0));
  // Deterministic +/-5 cm zig-zag noise on the ENU side.
  for (size_t i = 0; i < enu.size(); ++i) {
    enu[i].x() += (i % 2 == 0) ? 0.05 : -0.05;
    enu[i].y() += (i % 3 == 0) ? 0.05 : -0.05;
  }

  const auto a = estimatePlanarAlignment(odom, enu);
  ASSERT_TRUE(a.valid);
  EXPECT_NEAR(a.yaw_rad, yaw, 0.01);
  EXPECT_GT(a.rms_residual_m, 0.01);
  EXPECT_LT(a.rms_residual_m, 0.2);
}

}  // namespace
}  // namespace graphslam
