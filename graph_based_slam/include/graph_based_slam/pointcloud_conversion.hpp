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

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace graphslam
{
namespace pointcloud_conversion
{

inline bool hasField(
  const sensor_msgs::msg::PointCloud2 & message,
  const std::string & name)
{
  return std::any_of(
    message.fields.cbegin(), message.fields.cend(),
    [&name](const sensor_msgs::msg::PointField & field) {
      return field.name == name;
    });
}

// PCL prints "Failed to find match for field 'intensity'" every time an XYZ-only
// PointCloud2 is converted directly to PointXYZI. RKO-LIO's deskewed geometry
// stream intentionally contains XYZ only, so use a quiet, explicit zero default.
// The return value tells callers whether real intensity values were preserved.
inline bool fromRosMsgWithOptionalIntensity(
  const sensor_msgs::msg::PointCloud2 & message,
  pcl::PointCloud<pcl::PointXYZI> & output)
{
  if (hasField(message, "intensity")) {
    pcl::fromROSMsg(message, output);
    return true;
  }

  pcl::PointCloud<pcl::PointXYZ> xyz;
  pcl::fromROSMsg(message, xyz);

  output.header = xyz.header;
  output.width = xyz.width;
  output.height = xyz.height;
  output.is_dense = xyz.is_dense;
  output.sensor_origin_ = xyz.sensor_origin_;
  output.sensor_orientation_ = xyz.sensor_orientation_;
  output.points.resize(xyz.points.size());
  for (std::size_t index = 0; index < xyz.points.size(); ++index) {
    output.points[index].x = xyz.points[index].x;
    output.points[index].y = xyz.points[index].y;
    output.points[index].z = xyz.points[index].z;
    output.points[index].intensity = 0.0F;
  }
  return false;
}

// Whether the message has the layout packForPointXYZI leaves: x, y, z and, if there is one,
// intensity, FLOAT32 at offsets 0, 4, 8 and 12, in that order, without padding.
inline bool isPackedForPointXYZI(const sensor_msgs::msg::PointCloud2 & message)
{
  const char * const names[4] = {"x", "y", "z", "intensity"};
  const std::size_t count = message.fields.size();
  if (count != 3 && count != 4) {
    return false;
  }
  for (std::size_t k = 0; k < count; ++k) {
    const auto & field = message.fields[k];
    if (field.name != names[k] || field.offset != sizeof(float) * k ||
      field.datatype != sensor_msgs::msg::PointField::FLOAT32 || field.count != 1)
    {
      return false;
    }
  }
  const std::size_t point_step = sizeof(float) * count;
  return message.point_step == point_step && message.row_step == message.width * point_step &&
         message.data.size() == static_cast<std::size_t>(message.height) * message.row_step;
}

// Keeps only what fromRosMsgWithOptionalIntensity reads: x, y, z and, if the message has one,
// intensity, each FLOAT32, packed without padding (12 or 16 bytes a point; RKO-LIO's scan has 16
// with x, y, z). Converting the packed message gives the same cloud. Returns false and leaves the
// message as it is when a field the conversion reads is missing, of another type or named twice,
// or the data is shorter than its layout says.
inline bool packForPointXYZI(sensor_msgs::msg::PointCloud2 & message)
{
  using sensor_msgs::msg::PointField;
  const char * const names[4] = {"x", "y", "z", "intensity"};
  const std::size_t count = hasField(message, "intensity") ? 4 : 3;
  std::uint32_t offsets[4] = {0, 0, 0, 0};
  for (std::size_t k = 0; k < count; ++k) {
    std::size_t found = 0;
    for (const auto & field : message.fields) {
      if (field.name != names[k]) {
        continue;
      }
      ++found;
      if (field.datatype != PointField::FLOAT32 || field.count != 1 ||
        static_cast<std::size_t>(field.offset) + sizeof(float) > message.point_step)
      {
        return false;
      }
      offsets[k] = field.offset;
    }
    if (found != 1) {
      return false;
    }
  }
  if (isPackedForPointXYZI(message)) {
    return true;
  }
  const std::size_t width = message.width;
  const std::size_t height = message.height;
  const std::size_t point_step = sizeof(float) * count;
  if (width * height > 0 &&
    (message.row_step < width * message.point_step ||
    message.data.size() < (height - 1) * message.row_step + width * message.point_step))
  {
    return false;
  }
  std::vector<std::uint8_t> data(width * height * point_step);
  std::uint8_t * out = data.data();
  for (std::size_t row = 0; row < height; ++row) {
    const std::uint8_t * row_data = message.data.data() + row * message.row_step;
    for (std::size_t col = 0; col < width; ++col) {
      const std::uint8_t * point = row_data + col * message.point_step;
      for (std::size_t k = 0; k < count; ++k, out += sizeof(float)) {
        std::memcpy(out, point + offsets[k], sizeof(float));
      }
    }
  }
  message.fields.resize(count);
  for (std::size_t k = 0; k < count; ++k) {
    message.fields[k].name = names[k];
    message.fields[k].offset = static_cast<std::uint32_t>(sizeof(float) * k);
    message.fields[k].datatype = PointField::FLOAT32;
    message.fields[k].count = 1;
  }
  message.point_step = static_cast<std::uint32_t>(point_step);
  message.row_step = static_cast<std::uint32_t>(width * point_step);
  message.data = std::move(data);
  return true;
}

}  // namespace pointcloud_conversion
}  // namespace graphslam
