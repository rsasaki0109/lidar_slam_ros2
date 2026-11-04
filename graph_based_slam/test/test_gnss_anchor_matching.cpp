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

#include <vector>

#include <Eigen/Core>  // NOLINT(build/include_order)

#include "graph_based_slam/gnss_anchor_matching.hpp"

namespace graphslam
{
namespace detail
{
namespace
{

// Fixes every 0.1 s moving +1 m/s along x, info 100 except where noted.
std::vector<GnssSample> makeTrack()
{
  std::vector<GnssSample> samples;
  for (int k = 0; k <= 10; ++k) {
    GnssSample s;
    s.stamp = 100.0 + 0.1 * k;
    s.position = Eigen::Vector3d(0.1 * k, 0.0, 0.0);
    s.info = Eigen::Vector3d(100.0, 100.0, 10.0);
    s.rtk_like = true;
    samples.push_back(s);
  }
  return samples;
}

TEST(GnssAnchorMatching, NearestFixWithoutInterpolation)
{
  const auto samples = makeTrack();
  GnssAnchorMatchConfig config;  // interpolation off: the historical rule
  const auto m = matchGnssAnchor(samples, 100.34, config);
  ASSERT_TRUE(m.valid);
  EXPECT_FALSE(m.interpolated);
  EXPECT_NEAR(m.position.x(), 0.3, 1e-9);
}

TEST(GnssAnchorMatching, InterpolatesBetweenCloseFixes)
{
  auto samples = makeTrack();
  samples[4].info = Eigen::Vector3d(50.0, 80.0, 10.0);
  samples[4].rtk_like = false;
  GnssAnchorMatchConfig config;
  config.max_interpolation_gap_sec = 0.5;
  const auto m = matchGnssAnchor(samples, 100.34, config);
  ASSERT_TRUE(m.valid);
  EXPECT_TRUE(m.interpolated);
  EXPECT_NEAR(m.position.x(), 0.34, 1e-9);
  // The weaker fix sets the information, and both must be RTK-like.
  EXPECT_DOUBLE_EQ(m.info.x(), 50.0);
  EXPECT_DOUBLE_EQ(m.info.y(), 80.0);
  EXPECT_FALSE(m.rtk_like);
}

TEST(GnssAnchorMatching, FallsBackToNearestAcrossAGap)
{
  auto samples = makeTrack();
  samples.erase(samples.begin() + 3, samples.begin() + 8);  // 100.2 -> 100.8
  GnssAnchorMatchConfig config;
  config.max_interpolation_gap_sec = 0.5;
  const auto m = matchGnssAnchor(samples, 100.3, config);
  ASSERT_TRUE(m.valid);
  EXPECT_FALSE(m.interpolated);
  EXPECT_NEAR(m.position.x(), 0.2, 1e-9);
}

TEST(GnssAnchorMatching, NoFixWithinTheWindow)
{
  const auto samples = makeTrack();
  GnssAnchorMatchConfig config;
  config.max_interpolation_gap_sec = 0.5;
  EXPECT_FALSE(matchGnssAnchor(samples, 98.5, config).valid);
  EXPECT_FALSE(matchGnssAnchor(samples, 102.5, config).valid);
  EXPECT_TRUE(matchGnssAnchor(samples, 101.9, config).valid);  // 0.9 s after the last fix
  EXPECT_FALSE(matchGnssAnchor({}, 100.0, config).valid);
}

TEST(GnssAnchorMatching, ExactStampUsesThatFix)
{
  const auto samples = makeTrack();
  GnssAnchorMatchConfig config;
  config.max_interpolation_gap_sec = 0.5;
  const auto m = matchGnssAnchor(samples, 100.5, config);
  ASSERT_TRUE(m.valid);
  EXPECT_NEAR(m.position.x(), 0.5, 1e-9);
}

}  // namespace
}  // namespace detail
}  // namespace graphslam
