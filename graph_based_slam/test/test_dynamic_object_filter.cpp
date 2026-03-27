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

#include <memory>
#include <vector>

#include "graph_based_slam/dynamic_object_filter.hpp"

namespace
{

pcl::PointCloud<pcl::PointXYZI>::ConstPtr makeCloud(
  const std::vector<Eigen::Vector3d> & points)
{
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (const auto & point : points) {
    pcl::PointXYZI pcl_point;
    pcl_point.x = static_cast<float>(point.x());
    pcl_point.y = static_cast<float>(point.y());
    pcl_point.z = static_cast<float>(point.z());
    pcl_point.intensity = 1.0F;
    cloud->push_back(pcl_point);
  }
  return cloud;
}

bool containsPointNear(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  const Eigen::Vector3d & target,
  double tolerance)
{
  for (const auto & point : cloud.points) {
    const Eigen::Vector3d candidate(point.x, point.y, point.z);
    if ((candidate - target).norm() <= tolerance) {
      return true;
    }
  }
  return false;
}

}  // namespace

TEST(DynamicObjectFilter, TemporalConsistencyAcceptsNearbyRepeatedObservations)
{
  EXPECT_TRUE(graphslam::hasTemporalConsistency({0, 1, 3}, 3, 2));
  EXPECT_FALSE(graphslam::hasTemporalConsistency({0, 5}, 2, 2));
}

TEST(DynamicObjectFilter, RemovesSingleObservationCandidateVoxel)
{
  graphslam::DynamicObjectFilterConfig config;
  config.voxel_size = 0.5;
  config.min_observations = 2;
  config.temporal_window = 2;
  config.max_range_from_sensor_m = 20.0;

  std::vector<graphslam::TimedSubmapCloud> submaps;
  submaps.push_back(
    graphslam::TimedSubmapCloud{
    0, Eigen::Vector3d::Zero(), makeCloud({Eigen::Vector3d(1.0, 0.0, 0.0)})});

  const auto result = graphslam::buildDynamicObjectFilteredMap(submaps, config);
  EXPECT_TRUE(result.cloud->empty());
  EXPECT_EQ(result.stats.candidate_voxels, 1u);
  EXPECT_EQ(result.stats.removed_candidate_voxels, 1u);
}

TEST(DynamicObjectFilter, KeepsRepeatedStaticVoxel)
{
  graphslam::DynamicObjectFilterConfig config;
  config.voxel_size = 0.5;
  config.min_observations = 2;
  config.temporal_window = 2;
  config.max_range_from_sensor_m = 20.0;

  std::vector<graphslam::TimedSubmapCloud> submaps;
  submaps.push_back(
    graphslam::TimedSubmapCloud{
    0, Eigen::Vector3d::Zero(), makeCloud({Eigen::Vector3d(5.0, 0.0, 0.0)})});
  submaps.push_back(
    graphslam::TimedSubmapCloud{
    1, Eigen::Vector3d(1.0, 0.0, 0.0), makeCloud({Eigen::Vector3d(5.1, 0.0, 0.0)})});

  const auto result = graphslam::buildDynamicObjectFilteredMap(submaps, config);
  ASSERT_EQ(result.stats.kept_candidate_voxels, 1u);
  EXPECT_TRUE(containsPointNear(*result.cloud, Eigen::Vector3d(5.05, 0.0, 0.0), 0.2));
}

TEST(DynamicObjectFilter, KeepsFarSingleObservationOutsideFilterRange)
{
  graphslam::DynamicObjectFilterConfig config;
  config.voxel_size = 0.5;
  config.min_observations = 2;
  config.temporal_window = 2;
  config.max_range_from_sensor_m = 10.0;

  std::vector<graphslam::TimedSubmapCloud> submaps;
  submaps.push_back(
    graphslam::TimedSubmapCloud{
    0, Eigen::Vector3d::Zero(), makeCloud({Eigen::Vector3d(15.0, 0.0, 0.0)})});

  const auto result = graphslam::buildDynamicObjectFilteredMap(submaps, config);
  ASSERT_EQ(result.stats.always_keep_voxels, 1u);
  EXPECT_TRUE(containsPointNear(*result.cloud, Eigen::Vector3d(15.0, 0.0, 0.0), 0.1));
}
