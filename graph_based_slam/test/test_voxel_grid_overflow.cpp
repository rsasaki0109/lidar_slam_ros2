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
//    copyright notice, this list of conditions and the following disclaimer
//    in the documentation and/or other materials provided with the
//    distribution.
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

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>

#include "graph_based_slam/voxel_grid_overflow.hpp"

namespace
{

// Two corners spanning `extent` plus a pair of points 1 mm apart near the origin, so that a
// VoxelGrid that really filters merges the pair and returns fewer points than it got.
pcl::PointCloud<pcl::PointXYZI> boxCloud(float ex, float ey, float ez)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  for (const auto & xyz : {Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(ex, ey, ez),
      Eigen::Vector3f(0.0501f, 0.0501f, 0.0501f), Eigen::Vector3f(0.0511f, 0.0501f, 0.0501f)})
  {
    pcl::PointXYZI p;
    p.x = xyz.x();
    p.y = xyz.y();
    p.z = xyz.z();
    p.intensity = 1.0f;
    cloud.push_back(p);
  }
  return cloud;
}

bool voxelGridCopiesInput(const pcl::PointCloud<pcl::PointXYZI> & cloud, float leaf)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr input(new pcl::PointCloud<pcl::PointXYZI>(cloud));
  pcl::PointCloud<pcl::PointXYZI> output;
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setInputCloud(input);
  vg.setLeafSize(leaf, leaf, leaf);
  vg.filter(output);
  return output.size() == cloud.size();
}

}  // namespace

TEST(VoxelGridOverflow, MatchesPclOnBothSidesOfTheIndexLimit)
{
  const float leaf = 0.1f;
  // dx * dy * dz around 2^31 - 1 = 2147483647 at a 0.1 m leaf (5001 * 5001 * 85 below, * 86 above).
  for (const float ez : {1.0f, 5.0f, 8.4f, 8.45f, 8.49f, 8.5f, 8.51f, 9.0f, 20.0f}) {
    const auto cloud = boxCloud(500.0f, 500.0f, ez);
    EXPECT_EQ(
      graphslam::map_saver::voxelGridWouldCopyInput(cloud, leaf),
      voxelGridCopiesInput(cloud, leaf)) << "extent z " << ez;
  }
}

TEST(VoxelGridOverflow, SmallMapsAreFilteredAndHugeMapsAreCopied)
{
  using graphslam::map_saver::voxelGridWouldCopyInput;
  EXPECT_FALSE(voxelGridWouldCopyInput(boxCloud(100.0f, 100.0f, 5.0f), 0.1f));
  EXPECT_FALSE(voxelGridCopiesInput(boxCloud(100.0f, 100.0f, 5.0f), 0.1f));
  EXPECT_TRUE(voxelGridWouldCopyInput(boxCloud(1000.0f, 1000.0f, 50.0f), 0.1f));
  EXPECT_TRUE(voxelGridCopiesInput(boxCloud(1000.0f, 1000.0f, 50.0f), 0.1f));
  EXPECT_FALSE(voxelGridWouldCopyInput(pcl::PointCloud<pcl::PointXYZI>(), 0.1f));
}
