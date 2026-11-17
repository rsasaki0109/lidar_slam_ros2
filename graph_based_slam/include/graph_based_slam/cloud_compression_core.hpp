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

#ifndef GRAPH_BASED_SLAM__CLOUD_COMPRESSION_CORE_HPP_
#define GRAPH_BASED_SLAM__CLOUD_COMPRESSION_CORE_HPP_

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "graph_based_slam/pointcloud_conversion.hpp"

namespace graphslam
{
namespace cloud_compression
{

// A cloud as the backend keeps it: the message, with its data either in it or compressed. Cloud is
// sensor_msgs::msg::PointCloud2 (StoredCloud, cloud_compression.hpp) or
// graphslam::msg::PointCloud2.
template<typename Cloud>
struct BasicStoredCloud
{
  Cloud message;  // its data is empty while `compressed` holds it
  std::vector<std::uint8_t> compressed;
  // 0: `compressed` is lossless. Otherwise x, y and z were stored as multiples of this step
  // (compressQuantized).
  double quantization_step = 0.0;
};

// Whether graph_based_slam was built with zstd; without it, nothing is compressed.
bool available();

namespace detail
{
// The values of each field in a plane, the bytes of all values shuffled (the first byte of every
// value, then the second ...), then zstd level 1. False, leaving `compressed` empty, without zstd
// or when zstd fails.
bool compressPacked(
  const std::vector<std::uint8_t> & data, std::size_t points, std::size_t fields,
  std::vector<std::uint8_t> & compressed);
// The inverse of compressPacked; throws std::runtime_error on a mismatch or without zstd.
void decompressPacked(
  const std::vector<std::uint8_t> & compressed, std::size_t points, std::size_t fields,
  std::size_t point_step, std::vector<std::uint8_t> & data);
// Packed x, y, z (and intensity) with x, y and z rounded to multiples of `step`: each coordinate
// as an int32 count of steps, the difference to the previous point's, zigzag-coded; intensity's
// bits unchanged; then compressPacked. False, leaving `compressed` empty, when a coordinate is not
// finite or 2^30 steps or more from 0, or as compressPacked.
bool compressQuantized(
  const std::vector<std::uint8_t> & data, std::size_t points, std::size_t fields, double step,
  std::vector<std::uint8_t> & compressed);
// The inverse of compressQuantized: x, y and z within step / 2 (plus float rounding).
void decompressQuantized(
  const std::vector<std::uint8_t> & compressed, std::size_t points, std::size_t fields,
  std::size_t point_step, double step, std::vector<std::uint8_t> & data);
// The step for a maximum error: 2 x max_error, less 0.1 % so that float rounding of a restored
// coordinate stays inside the bound.
inline double quantizationStep(double max_error)
{
  return 2.0 * max_error * 0.999;
}
}  // namespace detail

// Keeps the message, its data compressed when the message is packed (isPackedForPointXYZI) and has
// points. Lossless.
template<typename Cloud>
BasicStoredCloud<Cloud> store(Cloud message)
{
  BasicStoredCloud<Cloud> cloud;
  if (!message.data.empty() && pointcloud_conversion::isPackedForPointXYZI(message) &&
    detail::compressPacked(
      message.data, static_cast<std::size_t>(message.width) * message.height,
      message.fields.size(), cloud.compressed))
  {
    std::vector<std::uint8_t>().swap(message.data);
  }
  cloud.message = std::move(message);
  return cloud;
}

// Keeps the message like store(), but with max_error > 0 its x, y and z within max_error metres
// (submap_cloud_max_error_m): a much smaller store, no longer lossless. A message that cannot be
// quantized (a non-finite or very large coordinate) is stored losslessly.
template<typename Cloud>
BasicStoredCloud<Cloud> store(Cloud message, double max_error)
{
  if (!(max_error > 0.0) || message.data.empty() ||
    !pointcloud_conversion::isPackedForPointXYZI(message))
  {
    return store(std::move(message));
  }
  BasicStoredCloud<Cloud> cloud;
  const double step = detail::quantizationStep(max_error);
  if (!detail::compressQuantized(
      message.data, static_cast<std::size_t>(message.width) * message.height,
      message.fields.size(), step, cloud.compressed))
  {
    return store(std::move(message));
  }
  cloud.quantization_step = step;
  std::vector<std::uint8_t>().swap(message.data);
  cloud.message = std::move(message);
  return cloud;
}

// The message as it was stored, its data restored.
template<typename Cloud>
Cloud restore(const BasicStoredCloud<Cloud> & cloud)
{
  Cloud message = cloud.message;
  if (cloud.compressed.empty()) {
    return message;
  }
  if (cloud.quantization_step > 0.0) {
    detail::decompressQuantized(
      cloud.compressed, static_cast<std::size_t>(message.width) * message.height,
      message.fields.size(), message.point_step, cloud.quantization_step, message.data);
    return message;
  }
  detail::decompressPacked(
    cloud.compressed, static_cast<std::size_t>(message.width) * message.height,
    message.fields.size(), message.point_step, message.data);
  return message;
}

// The bytes the cloud's points take: compressed, or the message's data.
template<typename Cloud>
std::size_t storedBytes(const BasicStoredCloud<Cloud> & cloud)
{
  return cloud.compressed.empty() ? cloud.message.data.size() : cloud.compressed.size();
}

}  // namespace cloud_compression
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__CLOUD_COMPRESSION_CORE_HPP_
