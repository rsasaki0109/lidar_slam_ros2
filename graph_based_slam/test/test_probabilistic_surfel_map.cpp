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
#include <array>
#include <cmath>
#include <cstring>
#include <vector>

#include "graph_based_slam/probabilistic_surfel_map.hpp"

namespace
{

graphslam::ProbabilisticSurfelMapScan scan(
  const std::uint64_t id, const double z, const double variance = 0.0)
{
  graphslam::ProbabilisticSurfelMapScan value;
  value.scan_id = id;
  value.sensor_origin = Eigen::Vector3d(0.0, 0.0, -2.0);
  value.pose_translation_variance_m2 = variance;
  const double center_x = id % 2U == 0U ? 0.02 : 0.07;
  const double center_y = id % 4U < 2U ? 0.02 : 0.07;
  value.world_points = {
    Eigen::Vector3d(center_x - 0.005, center_y - 0.005, z),
    Eigen::Vector3d(center_x + 0.005, center_y - 0.005, z),
    Eigen::Vector3d(center_x - 0.005, center_y + 0.005, z)};
  return value;
}

std::vector<graphslam::ProbabilisticSurfelMapScan> connectedPlaneScans()
{
  std::vector<graphslam::ProbabilisticSurfelMapScan> scans(4U);
  const std::array<Eigen::Vector2d, 4> offsets{
    Eigen::Vector2d(-0.08, -0.08), Eigen::Vector2d(0.08, -0.08),
    Eigen::Vector2d(-0.08, 0.08), Eigen::Vector2d(0.08, 0.08)};
  const std::array<double, 4> cell_x{0.25, 0.75, 1.25, 2.25};
  const std::array<double, 4> cell_z{0.0, 0.025, -0.005, 0.005};
  for (std::size_t scan_index = 0; scan_index < scans.size(); ++scan_index) {
    scans[scan_index].scan_id = scan_index;
    scans[scan_index].sensor_origin = Eigen::Vector3d(0.0, 0.0, -2.0);
    for (std::size_t cell = 0; cell < cell_z.size(); ++cell) {
      scans[scan_index].world_points.push_back(Eigen::Vector3d(
          cell_x[cell] + offsets[scan_index].x(),
          0.25 + offsets[scan_index].y(), cell_z[cell]));
    }
  }
  // One unsupported fine voxel next to the last valid support plane exercises
  // bounded surface extension without creating another valid surfel.
  scans.front().world_points.push_back(Eigen::Vector3d(1.75, 0.25, 0.03));
  return scans;
}

}  // namespace

TEST(ProbabilisticSurfelMap, PreservesEveryOccupiedVoxel)
{
  std::vector<graphslam::ProbabilisticSurfelMapScan> scans{
    scan(0U, 0.011), scan(1U, 0.012), scan(2U, 0.013)};
  scans[0].world_points.push_back(Eigen::Vector3d(1.01, 0.01, 0.01));
  graphslam::ProbabilisticSurfelMapConfig config;
  config.build_support_partition_maps = true;
  const auto result = graphslam::buildProbabilisticSurfelMap(scans, config);

  EXPECT_EQ(result.stats.occupied_voxels, 2U);
  EXPECT_EQ(result.baseline_centroids.size(), result.stats.occupied_voxels);
  EXPECT_EQ(result.fused_points.size(), result.stats.occupied_voxels);
  EXPECT_EQ(
    result.stats.fused_surfel_voxels + result.stats.fallback_centroid_voxels,
    result.stats.occupied_voxels);
  EXPECT_EQ(
    result.supported_partition_points.size() + result.fallback_partition_points.size(),
    result.stats.occupied_voxels);
}

TEST(ProbabilisticSurfelMap, UncertainSurfaceDoesNotPullFusedPoint)
{
  std::vector<graphslam::ProbabilisticSurfelMapScan> scans{
    scan(0U, 0.01), scan(1U, 0.01), scan(2U, 0.01), scan(3U, 0.01),
    scan(4U, 0.03, 1.0)};
  const auto result = graphslam::buildProbabilisticSurfelMap(scans);

  ASSERT_EQ(result.fused_points.size(), 1U);
  ASSERT_EQ(result.stats.fused_surfel_voxels, 1U);
  EXPECT_LT(result.fused_points.front().z(), result.baseline_centroids.front().z());
  EXPECT_LT(result.fused_points.front().z(), 0.012);
}

TEST(ProbabilisticSurfelMap, FallsBackForInsufficientSupport)
{
  std::vector<graphslam::ProbabilisticSurfelMapScan> scans{scan(0U, 0.02)};
  const auto result = graphslam::buildProbabilisticSurfelMap(scans);

  ASSERT_EQ(result.fused_points.size(), 1U);
  EXPECT_EQ(result.stats.fused_surfel_voxels, 0U);
  EXPECT_EQ(result.stats.fallback_centroid_voxels, 1U);
  EXPECT_EQ(
    0, std::memcmp(
      result.fused_points.front().data(), result.baseline_centroids.front().data(),
      sizeof(double) * 3U));
}

TEST(ProbabilisticSurfelMap, IsBitwiseInvariantToScanAndPointOrder)
{
  std::vector<graphslam::ProbabilisticSurfelMapScan> scans{
    scan(0U, 0.011), scan(1U, 0.012), scan(2U, 0.013)};
  const auto first = graphslam::buildProbabilisticSurfelMap(scans);
  std::reverse(scans.begin(), scans.end());
  for (auto & item : scans) {
    std::reverse(item.world_points.begin(), item.world_points.end());
  }
  const auto second = graphslam::buildProbabilisticSurfelMap(scans);

  ASSERT_EQ(first.fused_points.size(), second.fused_points.size());
  for (std::size_t i = 0; i < first.fused_points.size(); ++i) {
    EXPECT_EQ(
      0, std::memcmp(
        first.fused_points[i].data(), second.fused_points[i].data(), sizeof(double) * 3U));
  }
}

TEST(ProbabilisticSurfelMap, ClampSurvivesPointXyzFloatRoundTrip)
{
  using graphslam::probabilistic_surfel_map_detail::VoxelKey;
  const VoxelKey key(999, -1001, 7);
  const double voxel_size = 0.1;
  const Eigen::Vector3d outside(1000.0, -1000.0, 1000.0);
  const Eigen::Vector3d clamped =
    graphslam::probabilistic_surfel_map_detail::clampToVoxel(outside, key, voxel_size);
  const Eigen::Vector3d float_round_trip(
    static_cast<float>(clamped.x()), static_cast<float>(clamped.y()),
    static_cast<float>(clamped.z()));

  EXPECT_EQ(
    graphslam::probabilistic_surfel_map_detail::voxelKey(float_round_trip, voxel_size), key);
}

TEST(ProbabilisticSurfelMap, PersistenceFilterRemovesShortLivedNearVoxel)
{
  graphslam::ProbabilisticSurfelMapConfig config;
  config.build_persistence_filtered_map = true;
  config.persistence_min_distinct_scans = 3U;
  config.persistence_min_scan_span = 3U;
  std::vector<graphslam::ProbabilisticSurfelMapScan> scans{
    scan(0U, 0.01), scan(1U, 0.01), scan(2U, 0.01)};

  const auto result = graphslam::buildProbabilisticSurfelMap(scans, config);

  EXPECT_TRUE(result.persistence_filtered_points.empty());
  EXPECT_EQ(result.stats.persistence_removed_voxels, 1U);
}

TEST(ProbabilisticSurfelMap, PersistenceFilterKeepsSeparatedOrFarObservation)
{
  graphslam::ProbabilisticSurfelMapConfig config;
  config.build_persistence_filtered_map = true;
  config.persistence_min_distinct_scans = 3U;
  config.persistence_min_scan_span = 3U;
  std::vector<graphslam::ProbabilisticSurfelMapScan> scans{
    scan(0U, 0.01), scan(2U, 0.01), scan(4U, 0.01)};
  auto far_scan = scan(10U, 0.01);
  far_scan.sensor_origin = Eigen::Vector3d(0.0, 0.0, -100.0);
  for (Eigen::Vector3d & point : far_scan.world_points) {
    point.x() += 1.0;
  }
  scans.push_back(far_scan);

  const auto result = graphslam::buildProbabilisticSurfelMap(scans, config);

  EXPECT_EQ(result.persistence_filtered_points.size(), 2U);
  EXPECT_EQ(result.stats.persistence_kept_voxels, 1U);
  EXPECT_EQ(result.stats.persistence_far_range_keep_voxels, 1U);
}

TEST(ProbabilisticSurfelMap, BlendedPhasesAreBitwiseOrderInvariant)
{
  graphslam::ProbabilisticSurfelMapConfig config;
  config.surfel_support_voxel_size_m = 0.3;
  config.support_grid_phases = 8;
  config.blend_support_phases = true;
  std::vector<graphslam::ProbabilisticSurfelMapScan> scans{
    scan(0U, 0.011), scan(1U, 0.012), scan(2U, 0.013), scan(3U, 0.014)};
  const auto first = graphslam::buildProbabilisticSurfelMap(scans, config);
  std::reverse(scans.begin(), scans.end());
  for (auto & item : scans) {
    std::reverse(item.world_points.begin(), item.world_points.end());
  }
  const auto second = graphslam::buildProbabilisticSurfelMap(scans, config);

  ASSERT_EQ(first.fused_points.size(), second.fused_points.size());
  for (std::size_t i = 0; i < first.fused_points.size(); ++i) {
    EXPECT_EQ(
      0, std::memcmp(
        first.fused_points[i].data(), second.fused_points[i].data(), sizeof(double) * 3U));
  }
}

TEST(ProbabilisticSurfelMap, FallbackPhasesFillOnlyUnsupportedBoundaryVoxels)
{
  graphslam::ProbabilisticSurfelMapConfig config;
  config.voxel_size_m = 0.1;
  config.surfel_support_voxel_size_m = 0.5;
  config.support_grid_phases = 4;
  config.support_phases_fallback_only = true;
  std::vector<graphslam::ProbabilisticSurfelMapScan> scans(4U);
  const std::array<Eigen::Vector2d, 4> positions{
    Eigen::Vector2d(0.46, 0.46), Eigen::Vector2d(0.54, 0.46),
    Eigen::Vector2d(0.46, 0.54), Eigen::Vector2d(0.54, 0.54)};
  for (std::size_t i = 0; i < scans.size(); ++i) {
    scans[i].scan_id = i;
    scans[i].sensor_origin = Eigen::Vector3d(0.0, 0.0, -2.0);
    scans[i].world_points.push_back(Eigen::Vector3d(positions[i].x(), positions[i].y(), 0.01));
  }

  const auto result = graphslam::buildProbabilisticSurfelMap(scans, config);

  EXPECT_EQ(result.stats.occupied_voxels, 4U);
  EXPECT_EQ(result.stats.fused_surfel_voxels, 4U);
  EXPECT_EQ(result.stats.fallback_centroid_voxels, 0U);
  EXPECT_EQ(result.stats.shifted_phase_fused_voxels, 4U);
}

TEST(ProbabilisticSurfelMap, FallbackPhasesNeverOverwritePhaseZero)
{
  std::vector<graphslam::ProbabilisticSurfelMapScan> scans{
    scan(0U, 0.011), scan(1U, 0.012), scan(2U, 0.013), scan(3U, 0.014)};
  graphslam::ProbabilisticSurfelMapConfig phase_zero_config;
  phase_zero_config.surfel_support_voxel_size_m = 0.5;
  const auto phase_zero = graphslam::buildProbabilisticSurfelMap(scans, phase_zero_config);
  auto fallback_config = phase_zero_config;
  fallback_config.support_grid_phases = 8;
  fallback_config.support_phases_fallback_only = true;
  const auto fallback = graphslam::buildProbabilisticSurfelMap(scans, fallback_config);

  ASSERT_EQ(phase_zero.fused_points.size(), fallback.fused_points.size());
  EXPECT_EQ(fallback.stats.shifted_phase_fused_voxels, 0U);
  for (std::size_t i = 0; i < phase_zero.fused_points.size(); ++i) {
    EXPECT_EQ(
      0, std::memcmp(
        phase_zero.fused_points[i].data(), fallback.fused_points[i].data(),
        sizeof(double) * 3U));
  }
}

TEST(ProbabilisticSurfelMap, CoarseToFineScalesAreBitwiseOrderInvariant)
{
  graphslam::ProbabilisticSurfelMapConfig config;
  config.surfel_support_voxel_size_m = 0.5;
  const std::vector<graphslam::ProbabilisticSurfelMapScan> original_scans{
    scan(0U, 0.011), scan(1U, 0.012), scan(2U, 0.013), scan(3U, 0.014)};
  const auto coarse_only = graphslam::buildProbabilisticSurfelMap(original_scans, config);
  config.secondary_support_voxel_size_m = 0.3;
  auto scans = original_scans;
  const auto first = graphslam::buildProbabilisticSurfelMap(scans, config);
  config.tertiary_support_voxel_size_m = 0.7;
  const auto tertiary = graphslam::buildProbabilisticSurfelMap(scans, config);
  std::reverse(scans.begin(), scans.end());
  for (auto & item : scans) {
    std::reverse(item.world_points.begin(), item.world_points.end());
  }
  const auto second = graphslam::buildProbabilisticSurfelMap(scans, config);

  ASSERT_EQ(first.fused_points.size(), second.fused_points.size());
  ASSERT_EQ(first.fused_points.size(), coarse_only.fused_points.size());
  ASSERT_EQ(first.fused_points.size(), tertiary.fused_points.size());
  ASSERT_GT(first.stats.fused_surfel_voxels, 0U);
  for (std::size_t i = 0; i < first.fused_points.size(); ++i) {
    EXPECT_EQ(
      0, std::memcmp(
        first.fused_points[i].data(), second.fused_points[i].data(), sizeof(double) * 3U));
    EXPECT_EQ(
      0, std::memcmp(
        first.fused_points[i].data(), coarse_only.fused_points[i].data(), sizeof(double) * 3U));
    EXPECT_EQ(
      0, std::memcmp(
        first.fused_points[i].data(), tertiary.fused_points[i].data(), sizeof(double) * 3U));
  }
}

TEST(ProbabilisticSurfelMap, ConnectedSurfacePreservesCoverageAndOrder)
{
  graphslam::ProbabilisticSurfelMapConfig config;
  config.voxel_size_m = 0.1;
  config.surfel_support_voxel_size_m = 0.5;
  config.build_connected_surface_map = true;
  config.connected_surface_min_support_cells = 2U;
  auto scans = connectedPlaneScans();
  const auto first = graphslam::buildProbabilisticSurfelMap(scans, config);
  std::reverse(scans.begin(), scans.end());
  for (auto & item : scans) {
    std::reverse(item.world_points.begin(), item.world_points.end());
  }
  const auto second = graphslam::buildProbabilisticSurfelMap(scans, config);

  ASSERT_EQ(first.connected_surface_points.size(), first.fused_points.size());
  ASSERT_EQ(first.connected_surface_points.size(), second.connected_surface_points.size());
  EXPECT_EQ(first.stats.connected_surface_support_cells, 4U);
  EXPECT_GT(first.stats.connected_surface_merged_cells, 0U);
  EXPECT_GT(first.stats.connected_surface_projected_voxels, 0U);
  EXPECT_GT(first.stats.connected_surface_extended_fallback_voxels, 0U);
  for (std::size_t i = 0; i < first.connected_surface_points.size(); ++i) {
    EXPECT_EQ(
      0, std::memcmp(
        first.connected_surface_points[i].data(), second.connected_surface_points[i].data(),
        sizeof(double) * 3U));
  }
}

TEST(ProbabilisticSurfelMap, ConnectedNeighborhoodDoesNotCrossPerpendicularCorner)
{
  using graphslam::probabilistic_surfel_map_detail::SupportSurfelCell;
  graphslam::ProbabilisticSurfel horizontal;
  horizontal.valid = true;
  horizontal.normal = Eigen::Vector3d::UnitZ();
  horizontal.covariance = Eigen::Vector3d(0.1, 0.1, 0.001).asDiagonal();
  horizontal.distinct_scans = 3U;
  horizontal.input_observations = 3U;
  graphslam::ProbabilisticSurfel parallel = horizontal;
  parallel.mean = Eigen::Vector3d(1.0, 0.0, 0.01);
  graphslam::ProbabilisticSurfel perpendicular = horizontal;
  perpendicular.mean = Eigen::Vector3d(0.0, 1.0, 0.0);
  perpendicular.normal = Eigen::Vector3d::UnitX();
  perpendicular.covariance = Eigen::Vector3d(0.001, 0.1, 0.1).asDiagonal();
  std::vector<SupportSurfelCell> compatible{
    SupportSurfelCell{{0, 0, 0}, horizontal}, SupportSurfelCell{{1, 0, 0}, parallel},
    SupportSurfelCell{{0, 1, 0}, perpendicular}};
  std::sort(
    compatible.begin(), compatible.end(),
    graphslam::probabilistic_surfel_map_detail::supportSurfelCellLess);
  const auto center = std::lower_bound(
    compatible.begin(), compatible.end(), SupportSurfelCell{{0, 0, 0}, {}},
    graphslam::probabilistic_surfel_map_detail::supportSurfelCellLess);
  ASSERT_NE(center, compatible.end());
  const auto merged =
    graphslam::probabilistic_surfel_map_detail::mergeConnectedSupportNeighborhood(
    compatible, static_cast<std::size_t>(std::distance(compatible.begin(), center)),
    std::cos(8.0 * std::acos(-1.0) / 180.0), 0.04, 2U);

  ASSERT_TRUE(merged.valid);
  EXPECT_EQ(merged.input_observations, 6U);
  EXPECT_GT(std::abs(merged.normal.dot(Eigen::Vector3d::UnitZ())), 0.99);
}

TEST(ProbabilisticSurfelMap, SurfaceConsolidationMergesProjectedLayersDeterministically)
{
  graphslam::ProbabilisticSurfelMapConfig config;
  config.voxel_size_m = 0.1;
  config.surfel_support_voxel_size_m = 0.5;
  config.build_surface_consolidated_map = true;
  std::vector<graphslam::ProbabilisticSurfelMapScan> scans(4U);
  const std::array<Eigen::Vector2d, 4> positions{
    Eigen::Vector2d(0.12, 0.12), Eigen::Vector2d(0.32, 0.12),
    Eigen::Vector2d(0.12, 0.32), Eigen::Vector2d(0.32, 0.32)};
  for (std::size_t i = 0; i < scans.size(); ++i) {
    scans[i].scan_id = i;
    scans[i].sensor_origin = Eigen::Vector3d(0.0, 0.0, -2.0);
    scans[i].world_points = {
      Eigen::Vector3d(positions[i].x(), positions[i].y(), 0.01),
      Eigen::Vector3d(positions[i].x(), positions[i].y(), 0.11)};
  }
  const auto first = graphslam::buildProbabilisticSurfelMap(scans, config);
  std::reverse(scans.begin(), scans.end());
  for (auto & item : scans) {
    std::reverse(item.world_points.begin(), item.world_points.end());
                                                                                              }
  const auto second = graphslam::buildProbabilisticSurfelMap(scans, config);

  ASSERT_EQ(first.baseline_centroids.size(), 8U);
  ASSERT_EQ(first.fused_points.size(), 8U);
  ASSERT_EQ(first.surface_consolidated_points.size(), 4U);
  ASSERT_EQ(
    first.surface_consolidated_points.size(), second.surface_consolidated_points.size());
  EXPECT_EQ(first.stats.surface_consolidation_merged_points, 4U);
  for (std::size_t i = 0; i < first.surface_consolidated_points.size(); ++i) {
    EXPECT_EQ(
      0, std::memcmp(
        first.surface_consolidated_points[i].data(),
        second.surface_consolidated_points[i].data(), sizeof(double) * 3U));
  }
}

TEST(ProbabilisticSurfelMap, SurfaceConsolidationCanSelectOnlyLargePlaneCorrections)
{
  graphslam::ProbabilisticSurfelMapConfig config;
  config.voxel_size_m = 0.1;
  config.surfel_support_voxel_size_m = 0.5;
  config.build_surface_consolidated_map = true;
  config.surface_consolidation_min_projection_distance_m = 1.0;
  std::vector<graphslam::ProbabilisticSurfelMapScan> scans(4U);
  const std::array<Eigen::Vector2d, 4> positions{
    Eigen::Vector2d(0.12, 0.12), Eigen::Vector2d(0.32, 0.12),
    Eigen::Vector2d(0.12, 0.32), Eigen::Vector2d(0.32, 0.32)};
  for (std::size_t i = 0; i < scans.size(); ++i) {
    scans[i].scan_id = i;
    scans[i].sensor_origin = Eigen::Vector3d(0.0, 0.0, -2.0);
    scans[i].world_points = {
      Eigen::Vector3d(positions[i].x(), positions[i].y(), 0.01),
      Eigen::Vector3d(positions[i].x(), positions[i].y(), 0.11)};
  }

  const auto result = graphslam::buildProbabilisticSurfelMap(scans, config);

  EXPECT_EQ(result.stats.surface_consolidation_selected_points, 0U);
  EXPECT_EQ(result.surface_consolidated_points.size(), result.fused_points.size());
  EXPECT_EQ(result.stats.surface_consolidation_merged_points, 0U);
}

TEST(ProbabilisticSurfelMap, VisibilityFilterRequiresMeasuredFreeSpaceContradictions)
{
  graphslam::ProbabilisticSurfelMapConfig config;
  config.build_visibility_filtered_map = true;
  std::vector<graphslam::ProbabilisticSurfelMapScan> scans(31U);
  for (std::size_t i = 0; i < scans.size(); ++i) {
    scans[i].scan_id = i;
    scans[i].sensor_origin = Eigen::Vector3d::Zero();
  }
  scans[10].world_points.push_back(Eigen::Vector3d(2.0, 0.0, 0.0));
  scans[15].world_points.push_back(Eigen::Vector3d(5.0, 0.0, 0.0));
  scans[25].world_points.push_back(Eigen::Vector3d(5.0, 0.0, 0.0));

  const auto result = graphslam::buildProbabilisticSurfelMap(scans, config);

  ASSERT_EQ(result.fused_points.size(), 2U);
  EXPECT_EQ(result.stats.visibility_candidate_voxels, 1U);
  EXPECT_EQ(result.stats.visibility_tested_voxels, 1U);
  EXPECT_EQ(result.stats.visibility_contradicted_voxels, 1U);
  EXPECT_EQ(result.stats.visibility_removed_voxels, 1U);
  ASSERT_EQ(result.visibility_filtered_points.size(), 1U);
  EXPECT_NEAR(result.visibility_filtered_points.front().x(), 5.0, 1.0e-12);
}

TEST(ProbabilisticSurfelMap, VisibilityFilterTreatsNearReturnAsOcclusion)
{
  graphslam::ProbabilisticSurfelMapConfig config;
  config.build_visibility_filtered_map = true;
  std::vector<graphslam::ProbabilisticSurfelMapScan> scans(31U);
  for (std::size_t i = 0; i < scans.size(); ++i) {
    scans[i].scan_id = i;
    scans[i].sensor_origin = Eigen::Vector3d::Zero();
  }
  scans[10].world_points.push_back(Eigen::Vector3d(2.0, 0.0, 0.0));
  scans[15].world_points.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
  scans[25].world_points.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));

  const auto result = graphslam::buildProbabilisticSurfelMap(scans, config);

  EXPECT_EQ(result.stats.visibility_tested_voxels, 1U);
  EXPECT_EQ(result.stats.visibility_contradicted_voxels, 0U);
  EXPECT_EQ(result.stats.visibility_removed_voxels, 0U);
  EXPECT_EQ(result.visibility_filtered_points.size(), result.fused_points.size());
}

TEST(ProbabilisticSurfelMap, VisibilityFilterIsBitwiseOrderInvariant)
{
  graphslam::ProbabilisticSurfelMapConfig config;
  config.build_visibility_filtered_map = true;
  std::vector<graphslam::ProbabilisticSurfelMapScan> scans(31U);
  for (std::size_t i = 0; i < scans.size(); ++i) {
    scans[i].scan_id = i;
    scans[i].sensor_origin = Eigen::Vector3d::Zero();
  }
  scans[10].world_points.push_back(Eigen::Vector3d(2.0, 0.0, 0.0));
  scans[15].world_points.push_back(Eigen::Vector3d(5.0, 0.0, 0.0));
  scans[25].world_points.push_back(Eigen::Vector3d(5.0, 0.0, 0.0));
  const auto first = graphslam::buildProbabilisticSurfelMap(scans, config);
  std::reverse(scans.begin(), scans.end());
  const auto second = graphslam::buildProbabilisticSurfelMap(scans, config);

  ASSERT_EQ(first.visibility_filtered_points.size(), second.visibility_filtered_points.size());
  for (std::size_t i = 0; i < first.visibility_filtered_points.size(); ++i) {
    EXPECT_EQ(
      0, std::memcmp(
        first.visibility_filtered_points[i].data(),
        second.visibility_filtered_points[i].data(), sizeof(double) * 3U));
  }
}
