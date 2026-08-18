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
#include <cstring>
#include <memory>

#include "graph_based_slam/planar_map_consolidation.hpp"

namespace
{

void addPoint(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
  double x, double y, double z, float intensity = 1.0F)
{
  pcl::PointXYZI point;
  point.x = static_cast<float>(x);
  point.y = static_cast<float>(y);
  point.z = static_cast<float>(z);
  point.intensity = intensity;
  cloud->push_back(point);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr makeThickPlane()
{
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (int ix = 0; ix < 8; ++ix) {
    for (int iy = 0; iy < 8; ++iy) {
      const double z = ((ix + iy) % 2 == 0) ? -0.02 : 0.02;
      addPoint(cloud, 0.1 + 0.1 * ix, 0.1 + 0.1 * iy, z, static_cast<float>(ix));
    }
  }
  return cloud;
}

double zRms(const pcl::PointCloud<pcl::PointXYZI> & cloud)
{
  double squared_sum = 0.0;
  for (const auto & point : cloud.points) {
    squared_sum += static_cast<double>(point.z) * point.z;
  }
  return std::sqrt(squared_sum / static_cast<double>(cloud.size()));
}

graphslam::PlanarMapConsolidationConfig testConfig()
{
  graphslam::PlanarMapConsolidationConfig config;
  config.voxel_size = 1.0;
  config.min_neighbors = 20;
  config.max_small_eigenvalue_ratio = 0.02;
  config.min_middle_eigenvalue_ratio = 0.05;
  config.max_plane_distance_m = 0.05;
  config.projection_gain = 1.0;
  config.max_displacement_m = 0.05;
  config.min_supported_ratio = 0.9;
  return config;
}

}  // namespace

TEST(PlanarMapConsolidation, ReducesThicknessWithoutDeletingPoints)
{
  const auto cloud = makeThickPlane();
  const auto result = graphslam::buildPlanarMapConsolidatedMap(cloud, testConfig());

  ASSERT_FALSE(result.stats.fallback_to_input);
  EXPECT_EQ(result.cloud->size(), cloud->size());
  EXPECT_EQ(result.stats.projected_points, cloud->size());
  EXPECT_LT(zRms(*result.cloud), 0.1 * zRms(*cloud));
  for (std::size_t i = 0; i < cloud->size(); ++i) {
    EXPECT_FLOAT_EQ(result.cloud->points[i].intensity, cloud->points[i].intensity);
  }
}

TEST(PlanarMapConsolidation, BoundsEveryDisplacement)
{
  const auto cloud = makeThickPlane();
  auto config = testConfig();
  config.max_displacement_m = 0.005;
  const auto result = graphslam::buildPlanarMapConsolidatedMap(cloud, config);

  ASSERT_FALSE(result.stats.fallback_to_input);
  EXPECT_LE(result.stats.max_displacement_m, 0.005);
  for (std::size_t i = 0; i < cloud->size(); ++i) {
    const Eigen::Vector3f before = cloud->points[i].getVector3fMap();
    const Eigen::Vector3f after = result.cloud->points[i].getVector3fMap();
    EXPECT_LE((after - before).norm(), 0.005001F);
  }
}

TEST(PlanarMapConsolidation, FallsBackWhenPlanarSupportIsInsufficient)
{
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (int i = 0; i < 30; ++i) {
    addPoint(cloud, 0.13 * i, 0.17 * (i % 7), 0.19 * (i % 5));
  }
  auto config = testConfig();
  config.min_supported_ratio = 0.8;
  const auto result = graphslam::buildPlanarMapConsolidatedMap(cloud, config);

  EXPECT_TRUE(result.stats.fallback_to_input);
  ASSERT_EQ(result.cloud->size(), cloud->size());
  EXPECT_EQ(
    0, std::memcmp(
      result.cloud->points.data(), cloud->points.data(),
      cloud->size() * sizeof(pcl::PointXYZI)));
}

TEST(PlanarMapConsolidation, SameInputProducesBitwiseIdenticalCloud)
{
  const auto cloud = makeThickPlane();
  const auto first = graphslam::buildPlanarMapConsolidatedMap(cloud, testConfig());
  const auto second = graphslam::buildPlanarMapConsolidatedMap(cloud, testConfig());

  ASSERT_EQ(first.cloud->size(), second.cloud->size());
  EXPECT_EQ(
    0, std::memcmp(
      first.cloud->points.data(), second.cloud->points.data(),
      first.cloud->size() * sizeof(pcl::PointXYZI)));
}
