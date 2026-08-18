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
#include <cstdint>
#include <vector>

#include "graph_based_slam/trajectory_revisit_segmentation.hpp"

namespace
{

using graphslam::map_thickness::RevisitSegmentationConfig;
using graphslam::map_thickness::RevisitSegmentationResult;
using graphslam::map_thickness::segmentTrajectoryRevisits;

std::vector<Eigen::Vector3d> outAndBackTrajectory()
{
  std::vector<Eigen::Vector3d> positions;
  for (int x = 0; x <= 12; ++x) {
    positions.emplace_back(static_cast<double>(x), 0.0, 0.0);
  }
  for (int x = 11; x >= 0; --x) {
    positions.emplace_back(static_cast<double>(x), 0.0, 0.0);
  }
  return positions;
}

}  // namespace

TEST(TrajectoryRevisitSegmentation, OutAndBackStartsOneNewEpoch)
{
  RevisitSegmentationConfig config;
  config.match_radius_m = 0.1;
  config.min_prior_travel_m = 10.0;
  config.exit_hysteresis_travel_m = 2.0;
  config.min_epoch_separation_m = 5.0;
  const RevisitSegmentationResult result =
    segmentTrajectoryRevisits(outAndBackTrajectory(), config);

  ASSERT_EQ(result.revisit_ids.size(), 25U);
  EXPECT_EQ(result.revisit_ids[16], 0);
  EXPECT_EQ(result.revisit_ids[17], 1);
  EXPECT_EQ(result.revisit_ids.back(), 1);
  EXPECT_EQ(result.revisit_epoch_count, 2);
  EXPECT_EQ(result.matched_scan_count, 8);
  EXPECT_EQ(result.epoch_start_indices, (std::vector<std::size_t> {0U, 17U}));
}

TEST(TrajectoryRevisitSegmentation, NonRevisitingLineStaysInEpochZero)
{
  std::vector<Eigen::Vector3d> positions;
  for (int x = 0; x < 40; ++x) {
    positions.emplace_back(static_cast<double>(x), 0.0, 0.0);
  }
  const RevisitSegmentationResult result = segmentTrajectoryRevisits(positions);

  EXPECT_EQ(result.revisit_epoch_count, 1);
  EXPECT_EQ(result.matched_scan_count, 0);
  for (const std::int64_t id : result.revisit_ids) {
    EXPECT_EQ(id, 0);
  }
}

TEST(TrajectoryRevisitSegmentation, EmptyInputHasNoEpoch)
{
  const RevisitSegmentationResult result = segmentTrajectoryRevisits({});
  EXPECT_TRUE(result.revisit_ids.empty());
  EXPECT_EQ(result.revisit_epoch_count, 0);
  EXPECT_TRUE(result.epoch_start_indices.empty());
}

TEST(TrajectoryRevisitSegmentation, SameInputIsByteDeterministic)
{
  const RevisitSegmentationResult first = segmentTrajectoryRevisits(outAndBackTrajectory());
  const RevisitSegmentationResult second = segmentTrajectoryRevisits(outAndBackTrajectory());
  EXPECT_EQ(first.revisit_ids, second.revisit_ids);
  EXPECT_EQ(first.cumulative_travel_m, second.cumulative_travel_m);
}
