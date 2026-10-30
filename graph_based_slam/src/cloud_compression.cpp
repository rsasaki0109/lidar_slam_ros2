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

#include "graph_based_slam/cloud_compression.hpp"

#include <cstddef>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#ifdef GRAPH_BASED_SLAM_HAVE_ZSTD
#include <zstd.h>
#endif

#include "graph_based_slam/pointcloud_conversion.hpp"

namespace graphslam
{
namespace cloud_compression
{

namespace
{

constexpr int kZstdLevel = 1;

// Value v = p * points + i (field p of point i) puts its byte k at k * values + v. Whole values are
// read and written, which is faster than a loop over bytes; the round trip is exact either way.
void shuffle(
  const std::vector<std::uint8_t> & data, std::size_t points, std::size_t fields,
  std::vector<std::uint8_t> & shuffled)
{
  const std::size_t values = points * fields;
  shuffled.resize(data.size());
  for (std::size_t p = 0; p < fields; ++p) {
    std::uint8_t * const b0 = shuffled.data() + p * points;
    std::uint8_t * const b1 = b0 + values;
    std::uint8_t * const b2 = b1 + values;
    std::uint8_t * const b3 = b2 + values;
    for (std::size_t i = 0; i < points; ++i) {
      std::uint32_t value;
      std::memcpy(&value, data.data() + (i * fields + p) * sizeof(value), sizeof(value));
      b0[i] = static_cast<std::uint8_t>(value);
      b1[i] = static_cast<std::uint8_t>(value >> 8);
      b2[i] = static_cast<std::uint8_t>(value >> 16);
      b3[i] = static_cast<std::uint8_t>(value >> 24);
    }
  }
}

void unshuffle(
  const std::vector<std::uint8_t> & shuffled, std::size_t points, std::size_t fields,
  std::vector<std::uint8_t> & data)
{
  const std::size_t values = points * fields;
  data.resize(shuffled.size());
  for (std::size_t p = 0; p < fields; ++p) {
    const std::uint8_t * const b0 = shuffled.data() + p * points;
    const std::uint8_t * const b1 = b0 + values;
    const std::uint8_t * const b2 = b1 + values;
    const std::uint8_t * const b3 = b2 + values;
    for (std::size_t i = 0; i < points; ++i) {
      const std::uint32_t value = static_cast<std::uint32_t>(b0[i]) |
        (static_cast<std::uint32_t>(b1[i]) << 8) | (static_cast<std::uint32_t>(b2[i]) << 16) |
        (static_cast<std::uint32_t>(b3[i]) << 24);
      std::memcpy(data.data() + (i * fields + p) * sizeof(value), &value, sizeof(value));
    }
  }
}

}  // namespace

bool available()
{
#ifdef GRAPH_BASED_SLAM_HAVE_ZSTD
  return true;
#else
  return false;
#endif
}

StoredCloud store(sensor_msgs::msg::PointCloud2 message)
{
  StoredCloud cloud;
#ifdef GRAPH_BASED_SLAM_HAVE_ZSTD
  if (!message.data.empty() && pointcloud_conversion::isPackedForPointXYZI(message)) {
    std::vector<std::uint8_t> shuffled;
    shuffle(
      message.data, static_cast<std::size_t>(message.width) * message.height,
      message.fields.size(), shuffled);
    std::vector<std::uint8_t> buffer(ZSTD_compressBound(shuffled.size()));
    const std::size_t size = ZSTD_compress(
      buffer.data(), buffer.size(), shuffled.data(), shuffled.size(), kZstdLevel);
    if (!ZSTD_isError(size)) {
      cloud.compressed.assign(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(size));
      std::vector<std::uint8_t>().swap(message.data);
    }
  }
#endif
  cloud.message = std::move(message);
  return cloud;
}

sensor_msgs::msg::PointCloud2 restore(const StoredCloud & cloud)
{
  sensor_msgs::msg::PointCloud2 message = cloud.message;
  if (cloud.compressed.empty()) {
    return message;
  }
#ifdef GRAPH_BASED_SLAM_HAVE_ZSTD
  const std::size_t points = static_cast<std::size_t>(message.width) * message.height;
  std::vector<std::uint8_t> shuffled(points * message.point_step);
  const std::size_t size = ZSTD_decompress(
    shuffled.data(), shuffled.size(), cloud.compressed.data(), cloud.compressed.size());
  if (ZSTD_isError(size) || size != shuffled.size()) {
    throw std::runtime_error(
      "cloud_compression: a stored cloud does not decompress to its " +
      std::to_string(shuffled.size()) + " bytes");
  }
  unshuffle(shuffled, points, message.fields.size(), message.data);
  return message;
#else
  throw std::runtime_error("cloud_compression: built without zstd");
#endif
}

std::size_t storedBytes(const StoredCloud & cloud)
{
  return cloud.compressed.empty() ? cloud.message.data.size() : cloud.compressed.size();
}

}  // namespace cloud_compression
}  // namespace graphslam
