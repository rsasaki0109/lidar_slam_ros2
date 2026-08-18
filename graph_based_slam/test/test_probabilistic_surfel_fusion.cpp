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

#include <algorithm>
#include <cstring>
#include <vector>

#include "graph_based_slam/probabilistic_surfel_fusion.hpp"

namespace
{

graphslam::SurfelObservation observation(
  const std::uint64_t scan_id, const double x, const double y, const double z,
  const double translation_variance = 0.0)
{
  graphslam::SurfelObservation value;
  value.scan_id = scan_id;
  value.position = Eigen::Vector3d(x, y, z);
  value.sensor_origin = Eigen::Vector3d(0.0, 0.0, -2.0);
  value.pose_translation_variance_m2 = translation_variance;
  return value;
}

std::vector<graphslam::SurfelObservation> planarObservations()
{
  std::vector<graphslam::SurfelObservation> observations;
  for (std::uint64_t scan = 0; scan < 6; ++scan) {
    const double x = (scan % 3U == 0U) ? -0.2 : ((scan % 3U == 1U) ? 0.0 : 0.2);
    const double y = scan < 3U ? -0.15 : 0.15;
    observations.push_back(observation(scan, x, y, 0.002 * static_cast<double>(scan)));
  }
  return observations;
}

}  // namespace

TEST(ProbabilisticSurfelFusion, CollapsesDensityWithinEachScan)
{
  auto observations = planarObservations();
  for (int duplicate = 0; duplicate < 100; ++duplicate) {
    observations.push_back(observation(99U, 0.0, 0.0, 0.05));
  }
  const auto surfel = graphslam::fuseProbabilisticSurfel(observations);

  ASSERT_TRUE(surfel.valid);
  EXPECT_EQ(surfel.distinct_scans, 7U);
  EXPECT_LT(surfel.mean.z(), 0.02);
}

TEST(ProbabilisticSurfelFusion, DownweightsUncertainNormalObservation)
{
  auto observations = planarObservations();
  observations.push_back(observation(42U, 0.0, 0.0, 0.10, 0.25));
  const auto surfel = graphslam::fuseProbabilisticSurfel(observations);

  ASSERT_TRUE(surfel.valid);
  EXPECT_LT(std::abs(surfel.mean.z()), 0.02);
  EXPECT_LT(surfel.fused_normal_sigma_m, 0.02);
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(surfel.covariance);
  ASSERT_EQ(solver.info(), Eigen::Success);
  EXPECT_GE(solver.eigenvalues().minCoeff(), -1.0e-15);
}

TEST(ProbabilisticSurfelFusion, IsBitwiseInvariantToInputOrder)
{
  auto observations = planarObservations();
  const auto first = graphslam::fuseProbabilisticSurfel(observations);
  std::reverse(observations.begin(), observations.end());
  const auto second = graphslam::fuseProbabilisticSurfel(observations);

  ASSERT_TRUE(first.valid);
  ASSERT_TRUE(second.valid);
  EXPECT_EQ(0, std::memcmp(first.mean.data(), second.mean.data(), sizeof(double) * 3U));
  EXPECT_EQ(0, std::memcmp(first.normal.data(), second.normal.data(), sizeof(double) * 3U));
  EXPECT_EQ(
    0, std::memcmp(
      first.covariance.data(), second.covariance.data(), sizeof(double) * 9U));
}

TEST(ProbabilisticSurfelFusion, RejectsInsufficientScanSupport)
{
  std::vector<graphslam::SurfelObservation> observations{
    observation(1U, 0.0, 0.0, 0.0), observation(1U, 0.2, 0.0, 0.0),
    observation(2U, 0.0, 0.2, 0.0)};
  const auto surfel = graphslam::fuseProbabilisticSurfel(observations);

  EXPECT_FALSE(surfel.valid);
  EXPECT_EQ(surfel.distinct_scans, 2U);
}

TEST(ProbabilisticSurfelFusion, RejectsLineLikeSupport)
{
  std::vector<graphslam::SurfelObservation> observations;
  for (std::uint64_t scan = 0; scan < 6U; ++scan) {
    observations.push_back(observation(
        scan, 0.05 * static_cast<double>(scan), 0.0, 0.0));
  }

  const auto surfel = graphslam::fuseProbabilisticSurfel(observations);

  EXPECT_FALSE(surfel.valid);
}
