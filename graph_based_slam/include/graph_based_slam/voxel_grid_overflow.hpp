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

#ifndef GRAPH_BASED_SLAM__VOXEL_GRID_OVERFLOW_HPP_
#define GRAPH_BASED_SLAM__VOXEL_GRID_OVERFLOW_HPP_

#include <cstdint>
#include <limits>

#include <Eigen/Core>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>

namespace graphslam
{
namespace map_saver
{

// True when pcl::VoxelGrid with this leaf size would not downsample `cloud`: its grid has more
// voxels than a 32-bit index holds, and VoxelGrid::applyFilter then copies the input to the
// output unchanged. This is the same test, with the same float arithmetic, as in PCL 1.14
// (no filter field, every point used). A caller can then use the cloud itself instead of a copy.
template<typename PointT>
inline bool voxelGridWouldCopyInput(const pcl::PointCloud<PointT> & cloud, float leaf_size)
{
  if (cloud.empty()) {
    return false;
  }
  Eigen::Vector4f min_p, max_p;
  pcl::getMinMax3D<PointT>(cloud, min_p, max_p);
  const Eigen::Array4f leaf(leaf_size, leaf_size, leaf_size, 1.0f);
  const Eigen::Array4f inverse_leaf = Eigen::Array4f::Ones() / leaf;
  const std::int64_t dx = static_cast<std::int64_t>((max_p[0] - min_p[0]) * inverse_leaf[0]) + 1;
  const std::int64_t dy = static_cast<std::int64_t>((max_p[1] - min_p[1]) * inverse_leaf[1]) + 1;
  const std::int64_t dz = static_cast<std::int64_t>((max_p[2] - min_p[2]) * inverse_leaf[2]) + 1;
  return dx * dy * dz > static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max());
}

}  // namespace map_saver
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__VOXEL_GRID_OVERFLOW_HPP_
