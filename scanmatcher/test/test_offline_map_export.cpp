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
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>

#include <cmath>

#include <lidarslam_msgs/msg/map_array.hpp>
#include <pcl_conversions/pcl_conversions.h>  // NOLINT(build/include_order)

#include "scanmatcher/offline_map_export.hpp"

namespace
{

lidarslam_msgs::msg::SubMap makeSubmap(
  float x, float y, double tx, double ty)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.push_back(pcl::PointXYZI{x, y, 0.0F, 1.0F});
  cloud.push_back(pcl::PointXYZI{x + 1.0F, y, 0.0F, 2.0F});

  lidarslam_msgs::msg::SubMap submap;
  pcl::toROSMsg(cloud, submap.cloud);
  submap.pose.position.x = tx;
  submap.pose.position.y = ty;
  submap.pose.orientation.w = 1.0;
  return submap;
}

}  // namespace

TEST(OfflineMapExport, MergesSubmapsInMessageOrderAndAppliesWorldPose)
{
  lidarslam_msgs::msg::MapArray map_array;
  map_array.submaps.push_back(makeSubmap(1.0F, 2.0F, 10.0, 20.0));
  map_array.submaps.push_back(makeSubmap(3.0F, 4.0F, -5.0, 7.0));

  const auto map = graphslam::offline_map_export::mergeSubmaps(map_array);

  ASSERT_EQ(map.size(), 4U);
  EXPECT_FLOAT_EQ(map.points[0].x, 11.0F);
  EXPECT_FLOAT_EQ(map.points[0].y, 22.0F);
  EXPECT_FLOAT_EQ(map.points[1].x, 12.0F);
  EXPECT_FLOAT_EQ(map.points[1].y, 22.0F);
  EXPECT_FLOAT_EQ(map.points[2].x, -2.0F);
  EXPECT_FLOAT_EQ(map.points[2].y, 11.0F);
  EXPECT_FLOAT_EQ(map.points[3].x, -1.0F);
  EXPECT_FLOAT_EQ(map.points[3].y, 11.0F);
}

TEST(OfflineMapExport, PreservesPointFieldsAndEmptyMap)
{
  lidarslam_msgs::msg::MapArray empty;
  EXPECT_TRUE(graphslam::offline_map_export::mergeSubmaps(empty).empty());

  lidarslam_msgs::msg::MapArray map_array;
  map_array.submaps.push_back(makeSubmap(0.0F, 0.0F, 0.0, 0.0));
  const auto map = graphslam::offline_map_export::mergeSubmaps(map_array);

  ASSERT_EQ(map.size(), 2U);
  EXPECT_FLOAT_EQ(map.points[0].intensity, 1.0F);
  EXPECT_FLOAT_EQ(map.points[1].intensity, 2.0F);
  EXPECT_TRUE(std::isfinite(map.points[0].z));
}
