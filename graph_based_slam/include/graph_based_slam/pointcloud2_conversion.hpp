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

#ifndef GRAPH_BASED_SLAM__POINTCLOUD2_CONVERSION_HPP_
#define GRAPH_BASED_SLAM__POINTCLOUD2_CONVERSION_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cstddef>
#include <cstdint>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace graphslam
{
namespace pointcloud2_conversion
{

inline bool hasField(
  const sensor_msgs::msg::PointCloud2 & message,
  const char * const name)
{
  for (const auto & field : message.fields) {
    if (field.name == name) {
      return true;
    }
  }
  return false;
}

// Stable content revision for a PointCloud2 snapshot.  The complete field
// schema and payload are included; stamp/shape-only identities are unsafe
// because a producer can replace a cloud without changing its dimensions.
// Empty payloads return zero so a PCD/message provider with unknown content
// remains on the historical uncached path.
inline std::uint64_t contentRevision(
  const sensor_msgs::msg::PointCloud2 & message)
{
  if (message.data.empty() || message.width == 0U || message.height == 0U) {
    return 0U;
  }
  std::uint64_t hash = 1469598103934665603ULL;
  const auto mix = [&hash](const void * data, const std::size_t size) {
      const auto * bytes = static_cast<const std::uint8_t *>(data);
      for (std::size_t i = 0; i < size; ++i) {
        hash ^= bytes[i];
        hash *= 1099511628211ULL;
      }
    };
  mix(&message.header.stamp.sec, sizeof(message.header.stamp.sec));
  mix(&message.header.stamp.nanosec, sizeof(message.header.stamp.nanosec));
  mix(message.header.frame_id.data(), message.header.frame_id.size());
  mix(&message.height, sizeof(message.height));
  mix(&message.width, sizeof(message.width));
  mix(&message.is_bigendian, sizeof(message.is_bigendian));
  mix(&message.point_step, sizeof(message.point_step));
  mix(&message.row_step, sizeof(message.row_step));
  mix(&message.is_dense, sizeof(message.is_dense));
  for (const auto & field : message.fields) {
    mix(field.name.data(), field.name.size());
    mix(&field.offset, sizeof(field.offset));
    mix(&field.datatype, sizeof(field.datatype));
    mix(&field.count, sizeof(field.count));
  }
  mix(message.data.data(), message.data.size());
  return hash == 0U ? 1U : hash;
}

// Convert a PointCloud2 to PointXYZI without asking PCL to map an absent
// intensity field. PCL emits one warning for every missing destination field;
// the recorded backend clouds legitimately contain XYZ-only messages. Keeping
// the normal PointXYZI conversion when intensity exists preserves the legacy
// byte-level path, while the XYZ-only path makes the documented zero-fill
// policy explicit and warning-free.
inline void fromRosMsgPointXYZI(
  const sensor_msgs::msg::PointCloud2 & message,
  pcl::PointCloud<pcl::PointXYZI> & output)
{
  if (hasField(message, "intensity")) {
    pcl::fromROSMsg(message, output);
    return;
  }

  // PointXYZ and PointXYZI have the same x/y/z offsets and point stride in
  // PCL. Build the mapping for PointXYZ (which has no intensity tag), then
  // apply it directly to the zero-initialized PointXYZI output. This avoids an
  // intermediate cloud and keeps the fallback to one binary copy pass.
  pcl::PCLPointCloud2 pcl_cloud;
  pcl_conversions::copyPointCloud2MetaData(message, pcl_cloud);
  pcl::MsgFieldMap field_map;
  pcl::createMapping<pcl::PointXYZ>(pcl_cloud.fields, field_map);
  pcl::fromPCLPointCloud2(pcl_cloud, output, field_map, message.data.data());
}

}  // namespace pointcloud2_conversion
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__POINTCLOUD2_CONVERSION_HPP_
