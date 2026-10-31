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
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "graph_based_slam/cloud_compression.hpp"

namespace
{

// A packed cloud (x, y, z and, with `intensity`, intensity; FLOAT32 without padding) of
// width x height points along a wall, with NaN, infinity and -0 among them.
sensor_msgs::msg::PointCloud2 packedCloud(std::uint32_t width, std::uint32_t height, bool intensity)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "base_link";
  cloud.header.stamp.sec = 7;
  cloud.width = width;
  cloud.height = height;
  cloud.is_dense = false;
  std::vector<std::string> names = {"x", "y", "z"};
  if (intensity) {
    names.push_back("intensity");
  }
  for (std::size_t k = 0; k < names.size(); ++k) {
    sensor_msgs::msg::PointField field;
    field.name = names[k];
    field.offset = static_cast<std::uint32_t>(4 * k);
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    cloud.fields.push_back(field);
  }
  cloud.point_step = static_cast<std::uint32_t>(4 * names.size());
  cloud.row_step = width * cloud.point_step;
  const std::size_t points = static_cast<std::size_t>(width) * height;
  std::vector<float> values;
  for (std::size_t i = 0; i < points; ++i) {
    const float angle = 0.01F * static_cast<float>(i);
    values.push_back(10.0F * std::cos(angle));
    values.push_back(10.0F * std::sin(angle));
    values.push_back(0.001F * static_cast<float>(i % 97));
    if (intensity) {
      values.push_back(static_cast<float>(i % 255));
    }
  }
  if (points > 0) {
    values[0] = std::numeric_limits<float>::quiet_NaN();
    values[1] = std::numeric_limits<float>::infinity();
    values[2] = -0.0F;
  }
  cloud.data.resize(values.size() * sizeof(float));
  std::memcpy(cloud.data.data(), values.data(), cloud.data.size());
  return cloud;
}

}  // namespace

TEST(CloudCompression, RestoresAPackedCloudBitForBit)
{
  for (const bool intensity : {false, true}) {
    const sensor_msgs::msg::PointCloud2 original = packedCloud(500, 4, intensity);
    const graphslam::cloud_compression::StoredCloud stored =
      graphslam::cloud_compression::store(original);
    if (graphslam::cloud_compression::available()) {
      EXPECT_TRUE(stored.message.data.empty());
      ASSERT_FALSE(stored.compressed.empty());
      EXPECT_LT(stored.compressed.size(), original.data.size());
      EXPECT_EQ(graphslam::cloud_compression::storedBytes(stored), stored.compressed.size());
    } else {
      EXPECT_TRUE(stored.compressed.empty());
    }
    EXPECT_EQ(stored.message.width, original.width);
    EXPECT_EQ(stored.message.height, original.height);
    EXPECT_EQ(graphslam::cloud_compression::restore(stored), original);
  }
}

TEST(CloudCompression, KeepsOtherCloudsAsTheyAre)
{
  // Padded to 16 bytes a point, as RKO-LIO sends its scan.
  sensor_msgs::msg::PointCloud2 padded = packedCloud(100, 1, false);
  std::vector<std::uint8_t> data(100 * 16, 0);
  for (std::size_t i = 0; i < 100; ++i) {
    std::memcpy(data.data() + 16 * i, padded.data.data() + 12 * i, 12);
  }
  padded.data = data;
  padded.point_step = 16;
  padded.row_step = 100 * 16;
  // Packed, but without points.
  const sensor_msgs::msg::PointCloud2 empty = packedCloud(0, 1, false);
  for (const auto & message : {padded, empty}) {
    const graphslam::cloud_compression::StoredCloud stored =
      graphslam::cloud_compression::store(message);
    EXPECT_TRUE(stored.compressed.empty());
    EXPECT_EQ(stored.message, message);
    EXPECT_EQ(graphslam::cloud_compression::storedBytes(stored), message.data.size());
    EXPECT_EQ(graphslam::cloud_compression::restore(stored), message);
  }
}
