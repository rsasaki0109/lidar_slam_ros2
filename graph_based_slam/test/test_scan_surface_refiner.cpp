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

#include <cstring>
#include <vector>

#include "graph_based_slam/scan_surface_refiner.hpp"

namespace
{

std::vector<graphslam::scan_surface_refinement::ScanSurfaceRefinerScan> makePlaneScans()
{
  std::vector<graphslam::scan_surface_refinement::ScanSurfaceRefinerScan> scans;
  for (std::size_t scan_index = 0; scan_index < 7U; ++scan_index) {
    graphslam::scan_surface_refinement::ScanSurfaceRefinerScan scan;
    scan.scan_id = scan_index;
    const double offset = scan_index % 2U == 0U ? 0.012 : -0.012;
    scan.world_pose(0, 3) = 0.08 * static_cast<double>(scan_index);
    scan.world_pose(1, 3) = 0.08 * static_cast<double>(scan_index % 3U);
    scan.world_pose(2, 3) = offset;
    for (int x = -4; x <= 4; ++x) {
      for (int y = -4; y <= 4; ++y) {
        scan.local_points.push_back(Eigen::Vector3d(
            0.5 + 0.1 * x, 0.5 + 0.1 * y, -1.5));
      }
    }
    scans.push_back(scan);
  }
  return scans;
}

graphslam::scan_surface_refinement::ScanSurfaceRefinerConfig testConfig()
{
  graphslam::scan_surface_refinement::ScanSurfaceRefinerConfig config;
  config.scan_downsample_voxel_size_m = 0.0;
  config.support_voxel_size_m = 2.0;
  config.min_surface_observations_per_scan = 1U;
  config.absolute_translation_prior_sigma_m = 0.05;
  config.temporal_smoothness_sigma_m = 0.10;
  config.max_total_translation_correction_m = 0.02;
  return config;
}

}  // namespace

TEST(ScanSurfaceRefiner, ReducesScanToSurfaceResidual)
{
  const auto scans = makePlaneScans();
  const auto result = graphslam::scan_surface_refinement::refineScanSurfaceTranslations(
    scans, testConfig());

  ASSERT_TRUE(result.accepted) <<
    "surfels=" << result.valid_support_surfels <<
    " constrained=" << result.constrained_scans <<
    " observations=" << result.surface_observations <<
    " initial=" << result.initial_objective <<
    " final=" << result.final_objective;
  EXPECT_EQ(result.corrected_poses.size(), scans.size());
  EXPECT_EQ(result.translation_corrections.size(), scans.size());
  EXPECT_EQ(result.constrained_scans, scans.size());
  EXPECT_LT(result.final_surface_rms_m, result.initial_surface_rms_m);
  EXPECT_LT(result.final_objective, result.initial_objective);
  EXPECT_GT(result.correction_rms_m, 0.0);
}

TEST(ScanSurfaceRefiner, EnforcesTotalCorrectionCap)
{
  const auto scans = makePlaneScans();
  auto config = testConfig();
  config.max_total_translation_correction_m = 0.0005;
  const auto result = graphslam::scan_surface_refinement::refineScanSurfaceTranslations(
    scans, config);

  ASSERT_TRUE(result.accepted) <<
    "surfels=" << result.valid_support_surfels <<
    " constrained=" << result.constrained_scans <<
    " initial=" << result.initial_objective <<
    " final=" << result.final_objective;
  EXPECT_LE(result.correction_max_m, config.max_total_translation_correction_m + 1.0e-15);
}

TEST(ScanSurfaceRefiner, CrossFitUsesOnlyOppositeParitySurface)
{
  const auto scans = makePlaneScans();
  auto config = testConfig();
  config.cross_fit_scan_parity = true;
  const auto result = graphslam::scan_surface_refinement::refineScanSurfaceTranslations(
    scans, config);

  ASSERT_TRUE(result.accepted) <<
    "surfels=" << result.valid_support_surfels <<
    " constrained=" << result.constrained_scans <<
    " observations=" << result.surface_observations;
  EXPECT_EQ(result.constrained_scans, scans.size());
  EXPECT_GT(result.initial_surface_rms_m, 0.02);
  EXPECT_LT(result.final_surface_rms_m, result.initial_surface_rms_m);
}

TEST(ScanSurfaceRefiner, DisabledCrossFitMatchesDefaultBitwise)
{
  const auto scans = makePlaneScans();
  auto explicit_disabled = testConfig();
  explicit_disabled.cross_fit_scan_parity = false;
  const auto default_result = graphslam::scan_surface_refinement::refineScanSurfaceTranslations(
    scans, testConfig());
  const auto disabled_result = graphslam::scan_surface_refinement::refineScanSurfaceTranslations(
    scans, explicit_disabled);

  ASSERT_EQ(default_result.corrected_poses.size(), disabled_result.corrected_poses.size());
  for (std::size_t i = 0; i < default_result.corrected_poses.size(); ++i) {
    EXPECT_EQ(
      0, std::memcmp(
        default_result.corrected_poses[i].data(), disabled_result.corrected_poses[i].data(),
        16U * sizeof(double)));
  }
}

TEST(ScanSurfaceRefiner, FeaturelessInputFallsBackBitwise)
{
  auto scans = makePlaneScans();
  for (auto & scan : scans) {
    scan.local_points.clear();
  }
  const auto result = graphslam::scan_surface_refinement::refineScanSurfaceTranslations(
    scans, testConfig());

  EXPECT_FALSE(result.accepted);
  ASSERT_EQ(result.corrected_poses.size(), scans.size());
  for (std::size_t i = 0; i < scans.size(); ++i) {
    EXPECT_EQ(
      0, std::memcmp(
        result.corrected_poses[i].data(), scans[i].world_pose.data(), 16U * sizeof(double)));
  }
}

TEST(ScanSurfaceRefiner, IsBitwiseDeterministic)
{
  const auto scans = makePlaneScans();
  const auto first = graphslam::scan_surface_refinement::refineScanSurfaceTranslations(
    scans, testConfig());
  const auto second = graphslam::scan_surface_refinement::refineScanSurfaceTranslations(
    scans, testConfig());

  ASSERT_EQ(first.corrected_poses.size(), second.corrected_poses.size());
  for (std::size_t i = 0; i < first.corrected_poses.size(); ++i) {
    EXPECT_EQ(
      0, std::memcmp(
        first.corrected_poses[i].data(), second.corrected_poses[i].data(),
        16U * sizeof(double)));
  }
}
