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

#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>  // NOLINT(build/include_order)
#include <pcl/point_types.h>  // NOLINT(build/include_order)
#include <pcl_conversions/pcl_conversions.h>  // NOLINT(build/include_order)
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "graph_based_slam/pointcloud_conversion.hpp"

TEST(PointcloudConversion, PreservesIntensityWhenAvailable)
{
  pcl::PointCloud<pcl::PointXYZI> source;
  source.width = 2;
  source.height = 1;
  source.is_dense = true;
  source.points.resize(2);
  source.points[0].x = 1.0F;
  source.points[0].y = 2.0F;
  source.points[0].z = 3.0F;
  source.points[0].intensity = 17.0F;
  source.points[1].x = -1.0F;
  source.points[1].y = -2.0F;
  source.points[1].z = -3.0F;
  source.points[1].intensity = 42.0F;

  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(source, message);
  pcl::PointCloud<pcl::PointXYZI> converted;

  EXPECT_TRUE(
    graphslam::pointcloud_conversion::fromRosMsgWithOptionalIntensity(
      message, converted));
  ASSERT_EQ(converted.points.size(), source.points.size());
  EXPECT_FLOAT_EQ(converted.points[0].intensity, 17.0F);
  EXPECT_FLOAT_EQ(converted.points[1].intensity, 42.0F);
  EXPECT_FLOAT_EQ(converted.points[1].z, -3.0F);
}

TEST(PointcloudConversion, QuietlyDefaultsMissingIntensityToZero)
{
  pcl::PointCloud<pcl::PointXYZ> source;
  source.width = 2;
  source.height = 1;
  source.is_dense = true;
  source.points.resize(2);
  source.points[0].x = 1.0F;
  source.points[0].y = 2.0F;
  source.points[0].z = 3.0F;
  source.points[1].x = -1.0F;
  source.points[1].y = -2.0F;
  source.points[1].z = -3.0F;

  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(source, message);
  ASSERT_FALSE(graphslam::pointcloud_conversion::hasField(message, "intensity"));
  pcl::PointCloud<pcl::PointXYZI> converted;

  EXPECT_FALSE(
    graphslam::pointcloud_conversion::fromRosMsgWithOptionalIntensity(
      message, converted));
  ASSERT_EQ(converted.points.size(), source.points.size());
  EXPECT_EQ(converted.width, source.width);
  EXPECT_EQ(converted.height, source.height);
  EXPECT_EQ(converted.is_dense, source.is_dense);
  EXPECT_FLOAT_EQ(converted.points[0].intensity, 0.0F);
  EXPECT_FLOAT_EQ(converted.points[1].intensity, 0.0F);
  EXPECT_FLOAT_EQ(converted.points[0].x, 1.0F);
  EXPECT_FLOAT_EQ(converted.points[1].z, -3.0F);
}

namespace
{

sensor_msgs::msg::PointField floatField(const std::string & name, std::uint32_t offset)
{
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;
  return field;
}

// A width x height message with the given fields at the given point and row steps; point i has
// the value 0.25 * i + 0.125 * k in its FLOAT32 field k (and NaN in field 0 of every fifth point).
sensor_msgs::msg::PointCloud2 makeMessage(
  const std::vector<sensor_msgs::msg::PointField> & fields, std::uint32_t point_step,
  std::uint32_t width, std::uint32_t height, std::uint32_t row_step)
{
  sensor_msgs::msg::PointCloud2 message;
  message.header.frame_id = "base_link";
  message.header.stamp.sec = 12;
  message.header.stamp.nanosec = 345678000;
  message.fields = fields;
  message.point_step = point_step;
  message.width = width;
  message.height = height;
  message.row_step = row_step;
  message.is_dense = false;
  message.data.assign(static_cast<std::size_t>(row_step) * height, 0xAB);
  for (std::uint32_t row = 0; row < height; ++row) {
    for (std::uint32_t col = 0; col < width; ++col) {
      const std::uint32_t i = row * width + col;
      for (std::size_t k = 0; k < fields.size(); ++k) {
        if (fields[k].datatype != sensor_msgs::msg::PointField::FLOAT32) {
          continue;
        }
        float value = 0.25F * static_cast<float>(i) + 0.125F * static_cast<float>(k);
        if (k == 0 && i % 5 == 0) {
          value = std::numeric_limits<float>::quiet_NaN();
        }
        std::memcpy(
          message.data.data() + row * row_step + col * point_step + fields[k].offset, &value,
          sizeof(float));
      }
    }
  }
  return message;
}

// The converted clouds hold the same values (bit for bit, NaN included).
void expectSameConversion(
  const sensor_msgs::msg::PointCloud2 & a, const sensor_msgs::msg::PointCloud2 & b)
{
  pcl::PointCloud<pcl::PointXYZI> cloud_a;
  pcl::PointCloud<pcl::PointXYZI> cloud_b;
  EXPECT_EQ(
    graphslam::pointcloud_conversion::fromRosMsgWithOptionalIntensity(a, cloud_a),
    graphslam::pointcloud_conversion::fromRosMsgWithOptionalIntensity(b, cloud_b));
  EXPECT_EQ(cloud_a.header.stamp, cloud_b.header.stamp);
  EXPECT_EQ(cloud_a.header.frame_id, cloud_b.header.frame_id);
  EXPECT_EQ(cloud_a.width, cloud_b.width);
  EXPECT_EQ(cloud_a.height, cloud_b.height);
  EXPECT_EQ(cloud_a.is_dense, cloud_b.is_dense);
  ASSERT_EQ(cloud_a.size(), cloud_b.size());
  for (std::size_t i = 0; i < cloud_a.size(); ++i) {
    const float values_a[4] = {cloud_a[i].x, cloud_a[i].y, cloud_a[i].z, cloud_a[i].intensity};
    const float values_b[4] = {cloud_b[i].x, cloud_b[i].y, cloud_b[i].z, cloud_b[i].intensity};
    EXPECT_EQ(std::memcmp(values_a, values_b, sizeof(values_a)), 0) << "point " << i;
  }
}

}  // namespace

TEST(PointcloudConversion, PacksAnXyzScanToTwelveBytesAPoint)
{
  // RKO-LIO's deskewed scan: x, y, z and 4 bytes of padding.
  const sensor_msgs::msg::PointCloud2 scan = makeMessage(
    {floatField("x", 0), floatField("y", 4), floatField("z", 8)}, 16, 7, 1, 7 * 16);
  sensor_msgs::msg::PointCloud2 packed = scan;

  ASSERT_TRUE(graphslam::pointcloud_conversion::packForPointXYZI(packed));
  EXPECT_EQ(packed.point_step, 12U);
  EXPECT_EQ(packed.row_step, 7U * 12U);
  EXPECT_EQ(packed.data.size(), 7U * 12U);
  EXPECT_EQ(packed.fields.size(), 3U);
  expectSameConversion(scan, packed);

  // A packed message is left as it is.
  sensor_msgs::msg::PointCloud2 again = packed;
  ASSERT_TRUE(graphslam::pointcloud_conversion::packForPointXYZI(again));
  EXPECT_EQ(again, packed);
}

TEST(PointcloudConversion, PacksIntensityAndDropsTheFieldsTheConversionDoesNotRead)
{
  // PointXYZI as pcl::toROSMsg writes it (32 bytes a point).
  pcl::PointCloud<pcl::PointXYZI> source;
  for (int i = 0; i < 6; ++i) {
    pcl::PointXYZI point;
    point.x = 0.5F * static_cast<float>(i);
    point.y = -1.0F * static_cast<float>(i);
    point.z = 2.0F;
    point.intensity = 10.0F + static_cast<float>(i);
    source.push_back(point);
  }
  sensor_msgs::msg::PointCloud2 xyzi;
  pcl::toROSMsg(source, xyzi);
  sensor_msgs::msg::PointCloud2 packed = xyzi;
  ASSERT_TRUE(graphslam::pointcloud_conversion::packForPointXYZI(packed));
  EXPECT_EQ(packed.point_step, 16U);
  expectSameConversion(xyzi, packed);

  // An organized cloud with padded rows and a FLOAT64 time field ahead of the others.
  sensor_msgs::msg::PointField time;
  time.name = "time";
  time.offset = 0;
  time.datatype = sensor_msgs::msg::PointField::FLOAT64;
  time.count = 1;
  const sensor_msgs::msg::PointCloud2 organized = makeMessage(
    {time, floatField("intensity", 8), floatField("z", 12), floatField("y", 16),
      floatField("x", 20)}, 24, 4, 3, 4 * 24 + 8);
  packed = organized;
  ASSERT_TRUE(graphslam::pointcloud_conversion::packForPointXYZI(packed));
  EXPECT_EQ(packed.point_step, 16U);
  EXPECT_EQ(packed.data.size(), 12U * 16U);
  EXPECT_EQ(packed.height, 3U);
  expectSameConversion(organized, packed);
}

TEST(PointcloudConversion, LeavesAMessageItCannotPackAsItIs)
{
  sensor_msgs::msg::PointField intensity;
  intensity.name = "intensity";
  intensity.offset = 12;
  intensity.datatype = sensor_msgs::msg::PointField::UINT16;
  intensity.count = 1;
  const sensor_msgs::msg::PointCloud2 uint16_intensity = makeMessage(
    {floatField("x", 0), floatField("y", 4), floatField("z", 8), intensity}, 16, 5, 1, 5 * 16);
  const sensor_msgs::msg::PointCloud2 no_z = makeMessage(
    {floatField("x", 0), floatField("y", 4)}, 16, 5, 1, 5 * 16);
  const sensor_msgs::msg::PointCloud2 two_x = makeMessage(
    {floatField("x", 0), floatField("y", 4), floatField("z", 8), floatField("x", 12)}, 16, 5, 1,
    5 * 16);
  sensor_msgs::msg::PointCloud2 short_data = makeMessage(
    {floatField("x", 0), floatField("y", 4), floatField("z", 8)}, 16, 5, 1, 5 * 16);
  short_data.data.resize(4 * 16);
  for (const auto & message : {uint16_intensity, no_z, two_x, short_data}) {
    sensor_msgs::msg::PointCloud2 copy = message;
    EXPECT_FALSE(graphslam::pointcloud_conversion::packForPointXYZI(copy));
    EXPECT_EQ(copy, message);
  }
}
