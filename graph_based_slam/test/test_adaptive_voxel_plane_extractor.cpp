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
#include <cmath>
#include <cstdint>
#include <cstring>
#include <random>
#include <vector>

#include "graph_based_slam/adaptive_voxel_plane_extractor.hpp"

namespace
{

using graphslam::plane_extraction::PlaneExtractionConfig;
using graphslam::plane_extraction::PlaneExtractionResult;
using graphslam::plane_extraction::PlanarPatch;
using graphslam::plane_extraction::extractPlanarPatches;

std::vector<Eigen::Vector3d> makeFlatWallGrid()
{
  std::vector<Eigen::Vector3d> points;
  points.reserve(100);

  for (int ix = 0; ix < 10; ++ix) {
    for (int iy = 0; iy < 10; ++iy) {
      const double x = 0.08 + 0.08 * static_cast<double>(ix);
      const double y = 0.10 + 0.07 * static_cast<double>(iy);
      // Keep the wall strictly inside the root voxel z in [0, 1): a wall
      // at z = +/-0.0005 would straddle the voxel boundary at z = 0 and
      // split into two root-voxel patches.
      const double z = 0.5 + (((ix + iy) % 2 == 0) ? 0.0005 : -0.0005);
      points.push_back(Eigen::Vector3d(x, y, z));
    }
  }
  return points;
}

std::vector<Eigen::Vector3d> makeFloorGrid(const double z)
{
  std::vector<Eigen::Vector3d> points;
  points.reserve(64);

  for (int ix = 0; ix < 8; ++ix) {
    for (int iy = 0; iy < 8; ++iy) {
      const double x = 0.06 + 0.10 * static_cast<double>(ix);
      const double y = 0.07 + 0.09 * static_cast<double>(iy);
      const double dz = ((ix + 2 * iy) % 3 == 0) ? 0.0004 : -0.0004;
      points.push_back(Eigen::Vector3d(x, y, z + dz));
    }
  }
  return points;
}

std::vector<Eigen::Vector3d> makeTwoParallelFloors()
{
  std::vector<Eigen::Vector3d> points = makeFloorGrid(0.2);
  const std::vector<Eigen::Vector3d> upper = makeFloorGrid(1.3);
  points.insert(points.end(), upper.begin(), upper.end());
  return points;
}

std::vector<Eigen::Vector3d> makeParaboloid()
{
  std::vector<Eigen::Vector3d> points;
  points.reserve(169);

  for (int ix = -6; ix <= 6; ++ix) {
    for (int iy = -6; iy <= 6; ++iy) {
      const double x = 0.5 + 0.07 * static_cast<double>(ix);
      const double y = 0.5 + 0.07 * static_cast<double>(iy);
      const double dx = x - 0.5;
      const double dy = y - 0.5;
      const double z = 0.03 + 0.65 * (dx * dx + dy * dy);
      points.push_back(Eigen::Vector3d(x, y, z));
    }
  }
  return points;
}

std::vector<Eigen::Vector3d> makeVolumetricCloud()
{
  std::vector<Eigen::Vector3d> points;
  points.reserve(160);

  std::mt19937 generator(12345);
  std::uniform_real_distribution<double> distribution(0.05, 0.95);

  for (int i = 0; i < 160; ++i) {
    points.push_back(
      Eigen::Vector3d(
        distribution(generator),
        distribution(generator),
        distribution(generator)));
  }
  return points;
}

PlaneExtractionConfig baseConfig()
{
  PlaneExtractionConfig config;
  config.root_voxel_size = 1.0;
  config.max_octree_depth = 3;
  config.min_points_per_plane = 20;
  config.max_plane_thickness = 0.01;
  config.min_planarity_ratio = 6.0;
  config.enable_quarter_test = true;
  config.quarter_test_tolerance = 2.0;
  return config;
}

void expectPatchesBitIdentical(
  const PlaneExtractionResult & lhs,
  const PlaneExtractionResult & rhs)
{
  ASSERT_EQ(lhs.patches.size(), rhs.patches.size());
  EXPECT_EQ(lhs.total_points, rhs.total_points);
  EXPECT_EQ(lhs.planar_points, rhs.planar_points);
  EXPECT_EQ(lhs.planar_coverage, rhs.planar_coverage);

  for (std::size_t i = 0; i < lhs.patches.size(); ++i) {
    const PlanarPatch & a = lhs.patches[i];
    const PlanarPatch & b = rhs.patches[i];

    EXPECT_EQ(0, std::memcmp(a.centroid.data(), b.centroid.data(), 3 * sizeof(double)));
    EXPECT_EQ(0, std::memcmp(a.normal.data(), b.normal.data(), 3 * sizeof(double)));
    EXPECT_EQ(a.lambda_min, b.lambda_min);
    EXPECT_EQ(a.lambda_mid, b.lambda_mid);
    EXPECT_EQ(a.lambda_max, b.lambda_max);
    EXPECT_EQ(a.thickness_rms, b.thickness_rms);
    EXPECT_EQ(a.point_count, b.point_count);
    EXPECT_EQ(a.depth, b.depth);
  }
}

}  // namespace

TEST(AdaptiveVoxelPlaneExtractor, FlatWallGridAcceptsOnePatch)
{
  const std::vector<Eigen::Vector3d> points = makeFlatWallGrid();
  const PlaneExtractionConfig config = baseConfig();

  const PlaneExtractionResult result = extractPlanarPatches(points, config);

  ASSERT_EQ(1u, result.patches.size());
  EXPECT_EQ(static_cast<std::int64_t>(points.size()), result.total_points);
  EXPECT_EQ(static_cast<std::int64_t>(points.size()), result.planar_points);
  EXPECT_NEAR(1.0, result.planar_coverage, 1.0e-12);

  const PlanarPatch & patch = result.patches[0];
  EXPECT_NEAR(0.0, patch.normal.x(), 1.0e-9);
  EXPECT_NEAR(0.0, patch.normal.y(), 1.0e-9);
  EXPECT_NEAR(1.0, patch.normal.z(), 1.0e-9);
  EXPECT_LT(patch.thickness_rms, 0.001);
  EXPECT_EQ(0, patch.depth);
}

TEST(AdaptiveVoxelPlaneExtractor, TwoParallelFloorsProduceTwoOrderedPatches)
{
  const std::vector<Eigen::Vector3d> points = makeTwoParallelFloors();
  const PlaneExtractionConfig config = baseConfig();

  const PlaneExtractionResult result = extractPlanarPatches(points, config);

  ASSERT_EQ(2u, result.patches.size());
  EXPECT_EQ(static_cast<std::int64_t>(points.size()), result.total_points);
  EXPECT_EQ(static_cast<std::int64_t>(points.size()), result.planar_points);
  EXPECT_NEAR(1.0, result.planar_coverage, 1.0e-12);

  EXPECT_LT(result.patches[0].centroid.z(), result.patches[1].centroid.z());
  EXPECT_NEAR(0.2, result.patches[0].centroid.z(), 0.001);
  EXPECT_NEAR(1.3, result.patches[1].centroid.z(), 0.001);
  EXPECT_NEAR(1.0, result.patches[0].normal.z(), 1.0e-9);
  EXPECT_NEAR(1.0, result.patches[1].normal.z(), 1.0e-9);
}

TEST(AdaptiveVoxelPlaneExtractor, CurvedSurfaceDoesNotReturnThickPatches)
{
  const std::vector<Eigen::Vector3d> points = makeParaboloid();
  PlaneExtractionConfig config = baseConfig();
  config.max_plane_thickness = 0.02;
  config.max_octree_depth = 3;

  const PlaneExtractionResult result = extractPlanarPatches(points, config);

  for (std::size_t i = 0; i < result.patches.size(); ++i) {
    EXPECT_LE(result.patches[i].thickness_rms, config.max_plane_thickness);
  }
  EXPECT_EQ(static_cast<std::int64_t>(points.size()), result.total_points);
}

TEST(AdaptiveVoxelPlaneExtractor, VolumetricCloudRejectsPlanarity)
{
  const std::vector<Eigen::Vector3d> points = makeVolumetricCloud();
  PlaneExtractionConfig config = baseConfig();
  config.max_octree_depth = 0;
  config.max_plane_thickness = 1.0;

  const PlaneExtractionResult result = extractPlanarPatches(points, config);

  EXPECT_TRUE((result.patches.empty()));
  EXPECT_EQ(static_cast<std::int64_t>(points.size()), result.total_points);
  EXPECT_EQ(0, result.planar_points);
  EXPECT_EQ(0.0, result.planar_coverage);
}

TEST(AdaptiveVoxelPlaneExtractor, SameInputProducesBitIdenticalResults)
{
  std::vector<Eigen::Vector3d> points = makeFlatWallGrid();
  const std::vector<Eigen::Vector3d> floors = makeTwoParallelFloors();
  points.insert(points.end(), floors.begin(), floors.end());

  std::mt19937 generator(24680);
  std::shuffle(points.begin(), points.end(), generator);

  const PlaneExtractionConfig config = baseConfig();

  const PlaneExtractionResult first = extractPlanarPatches(points, config);
  const PlaneExtractionResult second = extractPlanarPatches(points, config);

  expectPatchesBitIdentical(first, second);
}

TEST(AdaptiveVoxelPlaneExtractor, SparseCloudBelowMinimumIsRejected)
{
  std::vector<Eigen::Vector3d> points;
  points.push_back(Eigen::Vector3d(0.1, 0.1, 0.0));
  points.push_back(Eigen::Vector3d(0.2, 0.1, 0.0));
  points.push_back(Eigen::Vector3d(0.1, 0.2, 0.0));
  points.push_back(Eigen::Vector3d(0.2, 0.2, 0.0));

  PlaneExtractionConfig config = baseConfig();
  config.min_points_per_plane = 5;

  const PlaneExtractionResult result = extractPlanarPatches(points, config);

  EXPECT_TRUE((result.patches.empty()));
  EXPECT_EQ(static_cast<std::int64_t>(points.size()), result.total_points);
  EXPECT_EQ(0, result.planar_points);
  EXPECT_EQ(0.0, result.planar_coverage);
}
