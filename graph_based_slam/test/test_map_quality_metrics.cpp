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
#include <random>
#include <string>
#include <vector>

#include "graph_based_slam/map_quality_metrics.hpp"

namespace
{

using graphslam::map_quality::MapQualityConfig;
using graphslam::map_quality::MapQualityReport;

std::vector<Eigen::Vector3d> makeNoisyFloor(double noise_sigma, int half_extent)
{
  // Deterministic grid floor on z=0 with fixed-seed gaussian z-noise.
  std::mt19937 rng(12345);
  std::normal_distribution<double> noise(0.0, noise_sigma);
  std::vector<Eigen::Vector3d> points;
  for (int x = -half_extent; x < half_extent; ++x) {
    for (int y = -half_extent; y < half_extent; ++y) {
      points.push_back(
        Eigen::Vector3d(0.05 * x, 0.05 * y, noise(rng)));
    }
  }
  return points;
}

std::vector<Eigen::Vector3d> makeVolumetricCloud(int count)
{
  std::mt19937 rng(6789);
  std::uniform_real_distribution<double> uniform(-2.0, 2.0);
  std::vector<Eigen::Vector3d> points;
  points.reserve(count);
  for (int i = 0; i < count; ++i) {
    const double x = uniform(rng);
    const double y = uniform(rng);
    const double z = uniform(rng);
    points.push_back(Eigen::Vector3d(x, y, z));
  }
  return points;
}

MapQualityConfig defaultConfig()
{
  MapQualityConfig config;
  config.mme_radius = 0.5;
  config.mme_min_neighbors = 8;
  return config;
}

TEST(MapQualityMetrics, NoisyFloorIsMeaningfulAndThicknessTracksNoise) {
  const auto points = makeNoisyFloor(0.01, 30);  // 3m x 3m, sigma 1cm
  const MapQualityReport report =
    graphslam::map_quality::computeMapQuality(points, defaultConfig());
  EXPECT_TRUE(report.plane_metrics_meaningful);
  EXPECT_GT(report.plane_patch_count, 0);
  EXPECT_GT(report.planar_coverage, 0.9);
  // The weighted thickness should track the injected sigma (loose band).
  EXPECT_GT(report.plane_thickness_rms_mean_m, 0.004);
  EXPECT_LT(report.plane_thickness_rms_mean_m, 0.02);
  EXPECT_EQ(report.input_points, static_cast<std::int64_t>(points.size()));
}

TEST(MapQualityMetrics, VolumetricCloudIsNotMeaningful) {
  const auto points = makeVolumetricCloud(4000);
  const MapQualityReport report =
    graphslam::map_quality::computeMapQuality(points, defaultConfig());
  EXPECT_FALSE(report.plane_metrics_meaningful);
  EXPECT_LT(report.planar_coverage, 0.10);
}

TEST(MapQualityMetrics, CrisperMapHasLowerEntropy) {
  const auto crisp = makeNoisyFloor(0.005, 30);
  const auto blurry = makeNoisyFloor(0.05, 30);
  const auto config = defaultConfig();
  const MapQualityReport crisp_report =
    graphslam::map_quality::computeMapQuality(crisp, config);
  const MapQualityReport blurry_report =
    graphslam::map_quality::computeMapQuality(blurry, config);
  EXPECT_GT(crisp_report.mme_valid_points, 0);
  EXPECT_GT(blurry_report.mme_valid_points, 0);
  EXPECT_LT(crisp_report.mean_map_entropy, blurry_report.mean_map_entropy);
}

TEST(MapQualityMetrics, MmeValidFractionReportsSupport) {
  // A dense floor plus far isolated points: the isolated points must be
  // excluded from the MME and visible in the valid fraction.
  auto points = makeNoisyFloor(0.01, 20);
  const size_t dense_count = points.size();
  points.push_back(Eigen::Vector3d(100.0, 100.0, 100.0));
  points.push_back(Eigen::Vector3d(-100.0, 50.0, -3.0));
  const MapQualityReport report =
    graphslam::map_quality::computeMapQuality(points, defaultConfig());
  EXPECT_GT(report.mme_valid_points, 0);
  EXPECT_LE(report.mme_valid_points, static_cast<std::int64_t>(dense_count));
  EXPECT_LT(report.mme_valid_fraction, 1.0);
}

TEST(MapQualityMetrics, DownsampleCollapsesVoxelsToCentroids) {
  std::vector<Eigen::Vector3d> points;
  points.push_back(Eigen::Vector3d(0.1, 0.1, 0.1));
  points.push_back(Eigen::Vector3d(0.3, 0.3, 0.3));   // same 0.5m voxel
  points.push_back(Eigen::Vector3d(10.0, 10.0, 10.0));
  const auto result = graphslam::map_quality::downsampleByVoxelCentroid(points, 0.5);
  ASSERT_EQ(result.size(), 2u);
  EXPECT_DOUBLE_EQ(result[0].x(), 0.2);
  EXPECT_DOUBLE_EQ(result[0].y(), 0.2);
  EXPECT_DOUBLE_EQ(result[0].z(), 0.2);
}

TEST(MapQualityMetrics, DownsampleOffReturnsInputUnchanged) {
  const auto points = makeVolumetricCloud(50);
  const auto result = graphslam::map_quality::downsampleByVoxelCentroid(points, 0.0);
  ASSERT_EQ(result.size(), points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    EXPECT_EQ(0, std::memcmp(points[i].data(), result[i].data(), 3 * sizeof(double)));
  }
}

TEST(MapQualityMetrics, ReportBytesAreDeterministic) {
  const auto points = makeNoisyFloor(0.01, 25);
  const auto config = defaultConfig();
  const MapQualityReport first =
    graphslam::map_quality::computeMapQuality(points, config);
  const MapQualityReport second =
    graphslam::map_quality::computeMapQuality(points, config);
  const auto first_lines = graphslam::map_quality::reportYamlLines(first, config);
  const auto second_lines = graphslam::map_quality::reportYamlLines(second, config);
  ASSERT_EQ(first_lines.size(), second_lines.size());
  for (size_t i = 0; i < first_lines.size(); ++i) {
    EXPECT_EQ(first_lines[i], second_lines[i]);
  }
  // Bitwise determinism on the raw doubles, not just the formatted text.
  EXPECT_EQ(
    0,
    std::memcmp(&first.mean_map_entropy, &second.mean_map_entropy, sizeof(double)));
  EXPECT_EQ(
    0,
    std::memcmp(
      &first.plane_thickness_rms_mean_m, &second.plane_thickness_rms_mean_m,
      sizeof(double)));
}

TEST(MapQualityMetrics, EmptyInputProducesZeroReport) {
  const std::vector<Eigen::Vector3d> points;
  const MapQualityReport report =
    graphslam::map_quality::computeMapQuality(points, defaultConfig());
  EXPECT_EQ(report.input_points, 0);
  EXPECT_EQ(report.evaluated_points, 0);
  EXPECT_FALSE(report.plane_metrics_meaningful);
  const auto lines =
    graphslam::map_quality::reportYamlLines(report, defaultConfig());
  EXPECT_FALSE(lines.empty());
}

TEST(MapQualityMetrics, YamlHasFixedKeyOrderAndMeaningfulFlag) {
  const auto points = makeNoisyFloor(0.01, 20);
  const auto config = defaultConfig();
  const MapQualityReport report =
    graphslam::map_quality::computeMapQuality(points, config);
  const auto lines = graphslam::map_quality::reportYamlLines(report, config);
  ASSERT_FALSE(lines.empty());
  EXPECT_EQ(lines[0], "map_quality_report:");
  bool found_meaningful = false;
  for (size_t i = 0; i < lines.size(); ++i) {
    if (lines[i] == "    meaningful: true") {found_meaningful = true;}
  }
  EXPECT_TRUE(found_meaningful);
}

}  // namespace
