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

#ifndef GRAPH_BASED_SLAM__CLOUD_COMPRESSION_HPP_
#define GRAPH_BASED_SLAM__CLOUD_COMPRESSION_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace graphslam
{
namespace cloud_compression
{

// A cloud as the backend keeps it: the message, with its data either in it or compressed.
struct StoredCloud
{
  sensor_msgs::msg::PointCloud2 message;  // its data is empty while `compressed` holds it
  std::vector<std::uint8_t> compressed;
};

// Whether graph_based_slam was built with zstd; without it, nothing is compressed.
bool available();

// Keeps the message, its data compressed when the message is packed (isPackedForPointXYZI) and has
// points: the values of each field in a plane, the bytes of all values shuffled (the first byte of
// every value, then the second ...), then zstd level 1. Lossless.
StoredCloud store(sensor_msgs::msg::PointCloud2 message);

// The message as it was stored, its data restored.
sensor_msgs::msg::PointCloud2 restore(const StoredCloud & cloud);

// The bytes the cloud's points take: compressed, or the message's data.
std::size_t storedBytes(const StoredCloud & cloud);

}  // namespace cloud_compression
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__CLOUD_COMPRESSION_HPP_
