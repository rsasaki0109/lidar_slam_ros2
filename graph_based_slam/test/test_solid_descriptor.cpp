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

#include "graph_based_slam/solid_descriptor.hpp"

namespace
{

pcl::PointCloud<pcl::PointXYZI>::Ptr makeForwardFacingCloud()
{
  auto cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  for (int i = 0; i < 40; ++i) {
    pcl::PointXYZI point;
    point.x = static_cast<float>(8.0 + 0.3 * i);
    point.y = static_cast<float>(-3.0 + 0.15 * i);
    point.z = static_cast<float>(-0.8 + 0.04 * i);
    point.intensity = 1.0f;
    cloud->push_back(point);
  }
  for (int i = 0; i < 20; ++i) {
    pcl::PointXYZI point;
    point.x = static_cast<float>(12.0 + 0.2 * i);
    point.y = static_cast<float>(4.0 + 0.1 * i);
    point.z = 1.5f;
    point.intensity = 1.0f;
    cloud->push_back(point);
  }
  return cloud;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr rotateCloud(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud,
  double yaw_rad)
{
  auto rotated = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  const float c = static_cast<float>(std::cos(yaw_rad));
  const float s = static_cast<float>(std::sin(yaw_rad));
  rotated->reserve(cloud->size());
  for (const auto & point : cloud->points) {
    pcl::PointXYZI rotated_point = point;
    rotated_point.x = c * point.x - s * point.y;
    rotated_point.y = s * point.x + c * point.y;
    rotated->push_back(rotated_point);
  }
  return rotated;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr makeDifferentCloud()
{
  auto cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  for (int i = 0; i < 36; ++i) {
    const double angle = static_cast<double>(i) * 2.0 * M_PI / 36.0;
    pcl::PointXYZI point;
    point.x = static_cast<float>(15.0 * std::cos(angle));
    point.y = static_cast<float>(15.0 * std::sin(angle));
    point.z = 0.3f;
    point.intensity = 1.0f;
    cloud->push_back(point);
  }
  return cloud;
}

}  // namespace

TEST(SolidDescriptor, SimilarityStaysHighForRotatedCloud)
{
  const auto base_cloud = makeForwardFacingCloud();
  const auto rotated_cloud = rotateCloud(base_cloud, M_PI_2);

  const auto base_descriptor = graphslam::SolidDescriptor::computeDescriptor(base_cloud);
  const auto rotated_descriptor = graphslam::SolidDescriptor::computeDescriptor(rotated_cloud);

  const double similarity = graphslam::SolidDescriptor::loopSimilarity(
    base_descriptor, rotated_descriptor);
  const double yaw_rad = graphslam::SolidDescriptor::poseYawRad(
    base_descriptor, rotated_descriptor);

  EXPECT_GT(similarity, 0.9);
  EXPECT_GT(std::abs(yaw_rad), 0.1);
}

TEST(SolidDescriptor, DatabaseReturnsMatchingSubmapId)
{
  graphslam::SolidDescriptor::Database db;
  db.add(7, graphslam::SolidDescriptor::computeDescriptor(makeForwardFacingCloud()));
  db.add(13, graphslam::SolidDescriptor::computeDescriptor(makeDifferentCloud()));
  db.add(
    21,
    graphslam::SolidDescriptor::computeDescriptor(
      rotateCloud(makeForwardFacingCloud(), M_PI_2)));

  const auto match = db.query(
    graphslam::SolidDescriptor::computeDescriptor(makeForwardFacingCloud()),
    /*num_candidates=*/ 3,
    /*exclude_recent=*/ 1,
    /*min_similarity=*/ 0.5);

  EXPECT_EQ(match.first, 7);
  EXPECT_GT(match.second, 0.5);
}
