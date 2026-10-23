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

#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "graph_based_slam/cloud_input_extrinsic.hpp"

using graphslam::cloud_input::LidarExtrinsic;
using graphslam::cloud_input::parseLidarExtrinsic;
using graphslam::cloud_input::scanIsInBaseFrame;
using graphslam::cloud_input::transformCloudToBase;

namespace
{
// HILTI 2022 PandarXT-32: configs/hilti2022/rko_lio_hilti2022_pandar_competitive_v2.yaml
const std::vector<double> kHiltiPandar {
  0.7071068, -0.7071068, 0.0, 0.0, -0.001, -0.00855, 0.055};

sensor_msgs::msg::PointField field(
  const std::string & name, std::uint32_t offset, std::uint8_t type)
{
  sensor_msgs::msg::PointField result;
  result.name = name;
  result.offset = offset;
  result.datatype = type;
  result.count = 1;
  return result;
}

// x, y, z, intensity and 4 bytes of padding per point, two rows of two.
sensor_msgs::msg::PointCloud2 cloud(const std::vector<float> & values)
{
  sensor_msgs::msg::PointCloud2 result;
  result.height = 2;
  result.width = 2;
  result.point_step = 20;
  result.row_step = 40;
  using sensor_msgs::msg::PointField;
  result.fields = {
    field("x", 0, PointField::FLOAT32), field("y", 4, PointField::FLOAT32),
    field("z", 8, PointField::FLOAT32), field("intensity", 12, PointField::FLOAT32)};
  result.data.assign(80, 0xAB);
  for (std::size_t point = 0; point < 4; ++point) {
    std::memcpy(result.data.data() + point * 20, &values[point * 4], 4 * sizeof(float));
  }
  return result;
}

float at(const sensor_msgs::msg::PointCloud2 & cloud, std::size_t point, std::size_t component)
{
  float value = 0.0F;
  std::memcpy(&value, cloud.data.data() + point * cloud.point_step + component * 4, sizeof(float));
  return value;
}
}  // namespace

TEST(CloudInputExtrinsic, IdentityIsDetectedExactly)
{
  LidarExtrinsic extrinsic;
  ASSERT_TRUE(parseLidarExtrinsic({0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0}, &extrinsic, nullptr));
  EXPECT_TRUE(extrinsic.identity);
  ASSERT_TRUE(parseLidarExtrinsic({0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1e-9}, &extrinsic, nullptr));
  EXPECT_FALSE(extrinsic.identity);
}

TEST(CloudInputExtrinsic, ParsesTheRkoLioQuaternionXyzwTranslationLayout)
{
  LidarExtrinsic extrinsic;
  ASSERT_TRUE(parseLidarExtrinsic(kHiltiPandar, &extrinsic, nullptr));
  EXPECT_FALSE(extrinsic.identity);
  // A half turn about (1, -1, 0) / sqrt(2) maps (x, y, z) to (-y, -x, -z).
  const Eigen::Vector3d moved = extrinsic.base_from_lidar * Eigen::Vector3d(1.0, 2.0, 3.0);
  EXPECT_NEAR(moved.x(), -2.0 - 0.001, 1e-6);
  EXPECT_NEAR(moved.y(), -1.0 - 0.00855, 1e-6);
  EXPECT_NEAR(moved.z(), -3.0 + 0.055, 1e-6);
}

TEST(CloudInputExtrinsic, RejectsMalformedValues)
{
  LidarExtrinsic extrinsic;
  std::string error;
  EXPECT_FALSE(parseLidarExtrinsic({0.0, 0.0, 0.0, 1.0, 0.0, 0.0}, &extrinsic, &error));
  EXPECT_FALSE(error.empty());
  EXPECT_FALSE(
    parseLidarExtrinsic(
      {0.0, 0.0, 0.0, std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0}, &extrinsic,
      &error));
  EXPECT_FALSE(parseLidarExtrinsic({0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0}, &extrinsic, &error));
}

TEST(CloudInputExtrinsic, MovesOnlyTheCoordinatesOfFinitePoints)
{
  LidarExtrinsic extrinsic;
  ASSERT_TRUE(parseLidarExtrinsic(kHiltiPandar, &extrinsic, nullptr));
  const float nan = std::numeric_limits<float>::quiet_NaN();
  auto input = cloud({1.0F, 2.0F, 3.0F, 7.0F, -4.0F, 0.5F, 2.0F, 8.0F, nan, 1.0F, 1.0F, 9.0F,
      0.0F, 0.0F, 0.0F, 10.0F});
  const auto original = input;

  ASSERT_TRUE(transformCloudToBase(extrinsic, input, nullptr));
  for (std::size_t point = 0; point < 4; ++point) {
    const Eigen::Vector3d p(at(original, point, 0), at(original, point, 1), at(original, point, 2));
    if (point == 2) {
      EXPECT_TRUE(std::isnan(at(input, point, 0)));
    } else {
      const Eigen::Vector3d expected = extrinsic.base_from_lidar * p;
      for (std::size_t axis = 0; axis < 3; ++axis) {
        EXPECT_FLOAT_EQ(at(input, point, axis), static_cast<float>(expected[axis]));
      }
    }
    EXPECT_EQ(at(input, point, 3), at(original, point, 3));
    EXPECT_EQ(
      0,
      std::memcmp(input.data.data() + point * 20 + 16, original.data.data() + point * 20 + 16, 4));
  }
}

TEST(CloudInputExtrinsic, IdentityLeavesTheCloudBytesUnchanged)
{
  LidarExtrinsic extrinsic;
  auto input = cloud({1.0F, 2.0F, 3.0F, 7.0F, -4.0F, 0.5F, 2.0F, 8.0F, 0.1F, 1.0F, 1.0F, 9.0F,
      0.0F, 0.0F, 0.0F, 10.0F});
  const auto original = input;

  ASSERT_TRUE(transformCloudToBase(extrinsic, input, nullptr));
  EXPECT_EQ(input.data, original.data);
}

TEST(CloudInputExtrinsic, RejectsCloudsWithoutSingleFloatCoordinates)
{
  LidarExtrinsic extrinsic;
  ASSERT_TRUE(parseLidarExtrinsic(kHiltiPandar, &extrinsic, nullptr));
  auto input = cloud(std::vector<float>(16, 1.0F));
  input.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT64;
  std::string error;
  EXPECT_FALSE(transformCloudToBase(extrinsic, input, &error));
  EXPECT_FALSE(error.empty());

  input = cloud(std::vector<float>(16, 1.0F));
  input.fields.pop_back();
  input.fields.erase(input.fields.begin() + 2);
  EXPECT_FALSE(transformCloudToBase(extrinsic, input, &error));

  input = cloud(std::vector<float>(16, 1.0F));
  input.data.resize(60);
  EXPECT_FALSE(transformCloudToBase(extrinsic, input, &error));
}

TEST(CloudInputExtrinsic, OnlyAScanStampedWithTheBaseFrameIsAlreadyInIt)
{
  // Official RKO-LIO >= 0.4.1 stamps the deskewed scan with base_frame; the
  // fork and earlier releases stamp it with the LiDAR frame.
  EXPECT_TRUE(scanIsInBaseFrame("base_link", "base_link"));
  EXPECT_FALSE(scanIsInBaseFrame("livox_frame", "base_link"));
  EXPECT_FALSE(scanIsInBaseFrame("", "base_link"));
  EXPECT_FALSE(scanIsInBaseFrame("", ""));
}
