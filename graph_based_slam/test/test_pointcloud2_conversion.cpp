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
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the
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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "graph_based_slam/pointcloud2_conversion.hpp"

namespace
{

sensor_msgs::msg::PointCloud2 makeXyzMessage()
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = 2;
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.points.emplace_back(1.0F, -2.0F, 3.5F);
  cloud.points.emplace_back(-4.0F, 5.0F, -6.5F);

  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(cloud, message);
  return message;
}

sensor_msgs::msg::PointCloud2 makeXyziMessage()
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.width = 2;
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.points.emplace_back(1.0F, -2.0F, 3.5F, 7.25F);
  cloud.points.emplace_back(-4.0F, 5.0F, -6.5F, 8.5F);

  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(cloud, message);
  return message;
}

TEST(PointCloud2Conversion, FillsMissingIntensityWithZeroWithoutChangingGeometry)
{
  const auto message = makeXyzMessage();
  ASSERT_FALSE(graphslam::pointcloud2_conversion::hasField(message, "intensity"));

  pcl::PointCloud<pcl::PointXYZI> output;
  graphslam::pointcloud2_conversion::fromRosMsgPointXYZI(message, output);

  ASSERT_EQ(output.width, 2U);
  ASSERT_EQ(output.height, 1U);
  ASSERT_EQ(output.points.size(), 2U);
  EXPECT_FLOAT_EQ(output.points[0].x, 1.0F);
  EXPECT_FLOAT_EQ(output.points[0].y, -2.0F);
  EXPECT_FLOAT_EQ(output.points[0].z, 3.5F);
  EXPECT_FLOAT_EQ(output.points[0].intensity, 0.0F);
  EXPECT_FLOAT_EQ(output.points[1].x, -4.0F);
  EXPECT_FLOAT_EQ(output.points[1].y, 5.0F);
  EXPECT_FLOAT_EQ(output.points[1].z, -6.5F);
  EXPECT_FLOAT_EQ(output.points[1].intensity, 0.0F);
}

TEST(PointCloud2Conversion, KeepsExistingIntensityConversion)
{
  const auto message = makeXyziMessage();
  ASSERT_TRUE(graphslam::pointcloud2_conversion::hasField(message, "intensity"));

  pcl::PointCloud<pcl::PointXYZI> output;
  graphslam::pointcloud2_conversion::fromRosMsgPointXYZI(message, output);

  ASSERT_EQ(output.points.size(), 2U);
  EXPECT_FLOAT_EQ(output.points[0].x, 1.0F);
  EXPECT_FLOAT_EQ(output.points[0].y, -2.0F);
  EXPECT_FLOAT_EQ(output.points[0].z, 3.5F);
  EXPECT_FLOAT_EQ(output.points[0].intensity, 7.25F);
  EXPECT_FLOAT_EQ(output.points[1].x, -4.0F);
  EXPECT_FLOAT_EQ(output.points[1].y, 5.0F);
  EXPECT_FLOAT_EQ(output.points[1].z, -6.5F);
  EXPECT_FLOAT_EQ(output.points[1].intensity, 8.5F);
}

TEST(PointCloud2Conversion, ContentRevisionChangesWhenPayloadChanges)
{
  auto message = makeXyzMessage();
  const auto first = graphslam::pointcloud2_conversion::contentRevision(message);
  ASSERT_NE(first, 0U);
  message.data.back() ^= 0x01U;
  const auto second = graphslam::pointcloud2_conversion::contentRevision(message);
  EXPECT_NE(first, second);
}

TEST(PointCloud2Conversion, EmptyPayloadIsNotCacheable)
{
  sensor_msgs::msg::PointCloud2 message;
  EXPECT_EQ(graphslam::pointcloud2_conversion::contentRevision(message), 0U);
}

}  // namespace
