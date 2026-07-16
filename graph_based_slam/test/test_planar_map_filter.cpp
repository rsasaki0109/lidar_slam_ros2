// Copyright 2026 Sasaki
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)

#include <gtest/gtest.h>

#include <memory>

#include "graph_based_slam/planar_map_filter.hpp"

namespace
{

void addPoint(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
  double x, double y, double z)
{
  pcl::PointXYZI point;
  point.x = static_cast<float>(x);
  point.y = static_cast<float>(y);
  point.z = static_cast<float>(z);
  point.intensity = 1.0F;
  cloud->push_back(point);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr makePlaneAndVolumeCloud()
{
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (int ix = 0; ix < 4; ++ix) {
    for (int iy = 0; iy < 4; ++iy) {
      addPoint(cloud, 0.1 + 0.2 * ix, 0.1 + 0.2 * iy, 0.0);
    }
  }
  for (int ix = 0; ix < 2; ++ix) {
    for (int iy = 0; iy < 2; ++iy) {
      for (int iz = 0; iz < 2; ++iz) {
        addPoint(cloud, 5.1 + 0.6 * ix, 0.1 + 0.6 * iy, 0.1 + 0.6 * iz);
      }
    }
  }
  return cloud;
}

}  // namespace

TEST(PlanarMapFilter, RetainsPlanarSupportAndRemovesVolumetricCluster)
{
  const auto cloud = makePlaneAndVolumeCloud();
  graphslam::PlanarMapFilterConfig config;
  config.voxel_size = 1.0;
  config.min_neighbors = 8;
  config.max_small_eigenvalue_ratio = 0.03;
  config.min_middle_eigenvalue_ratio = 0.05;
  config.min_retained_ratio = 0.5;

  const auto result = graphslam::buildPlanarMapFilteredMap(cloud, config);

  EXPECT_FALSE(result.stats.fallback_to_input);
  EXPECT_EQ(result.stats.input_points, 24u);
  EXPECT_EQ(result.stats.supported_points, 16u);
  EXPECT_EQ(result.stats.output_points, 16u);
  for (const auto & point : result.cloud->points) {
    EXPECT_LT(point.x, 2.0F);
  }
}

TEST(PlanarMapFilter, FallsBackWhenPlanarSupportWouldDeleteTooMuch)
{
  const auto cloud = makePlaneAndVolumeCloud();
  graphslam::PlanarMapFilterConfig config;
  config.voxel_size = 1.0;
  config.min_neighbors = 8;
  config.min_retained_ratio = 0.75;

  const auto result = graphslam::buildPlanarMapFilteredMap(cloud, config);

  EXPECT_TRUE(result.stats.fallback_to_input);
  EXPECT_EQ(result.stats.supported_points, 16u);
  EXPECT_EQ(result.stats.output_points, cloud->size());
}

TEST(PlanarMapFilter, InvalidConfigurationPreservesInput)
{
  const auto cloud = makePlaneAndVolumeCloud();
  graphslam::PlanarMapFilterConfig config;
  config.voxel_size = 0.0;

  const auto result = graphslam::buildPlanarMapFilteredMap(cloud, config);

  EXPECT_TRUE(result.stats.fallback_to_input);
  EXPECT_EQ(result.stats.output_points, cloud->size());
}
