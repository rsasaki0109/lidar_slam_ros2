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

// PCL precompiles its filters for its own point types only; this unit instantiates
// pcl::VoxelGrid<MapPoint> from PCL's implementation headers.
#define PCL_NO_PRECOMPILE
#include <pcl/filters/voxel_grid.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "graph_based_slam/map_point.hpp"
#include "graph_based_slam/voxel_grid_overflow.hpp"

namespace graphslam
{
namespace map_saver
{

void voxelGridFilter(const MapCloud::ConstPtr & input, float leaf_size, MapCloud & output)
{
  pcl::VoxelGrid<MapPoint> vg;
  vg.setInputCloud(input);
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.filter(output);
}

void voxelGridFilterInParts(const MapCloud & input, float leaf_size, MapCloud & output)
{
  // As pcl::VoxelGrid::setLeafSize computes it; a point's voxel along an axis is
  // floor(coordinate * inverse_leaf).
  const float inverse_leaf = 1.0f / leaf_size;
  const auto voxel_of = [inverse_leaf](float coordinate) {
      return static_cast<int>(std::floor(coordinate * inverse_leaf));
    };

  std::vector<std::vector<std::size_t>> pending(1);
  pending.front().reserve(input.size());
  for (std::size_t i = 0; i < input.size(); ++i) {
    const MapPoint & p = input[i];
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
      pending.front().push_back(i);
    }
  }

  MapCloud result;
  while (!pending.empty()) {
    std::vector<std::size_t> indices = std::move(pending.back());
    pending.pop_back();
    if (indices.empty()) {
      continue;
    }
    Eigen::Vector4f min_p = Eigen::Vector4f::Constant(std::numeric_limits<float>::max());
    Eigen::Vector4f max_p = Eigen::Vector4f::Constant(std::numeric_limits<float>::lowest());
    for (const std::size_t i : indices) {
      for (int axis = 0; axis < 3; ++axis) {
        min_p[axis] = std::min(min_p[axis], input[i].data[axis]);
        max_p[axis] = std::max(max_p[axis], input[i].data[axis]);
      }
    }
    if (!voxelGridIndexOverflows(min_p, max_p, leaf_size)) {
      MapCloud::Ptr part(new MapCloud);
      part->reserve(indices.size());
      for (const std::size_t i : indices) {
        part->push_back(input[i]);
      }
      std::vector<std::size_t>().swap(indices);
      MapCloud filtered;
      voxelGridFilter(part, leaf_size, filtered);
      result.insert(result.end(), filtered.begin(), filtered.end());
      continue;
    }
    // An overflowing part spans many voxels along its longest axis, so both halves are
    // non-empty: the lowest point lies below the split voxel and the highest one at or above it.
    const Eigen::Vector4f extent = max_p - min_p;
    int axis = 0;
    for (int candidate = 1; candidate < 3; ++candidate) {
      if (extent[candidate] > extent[axis]) {
        axis = candidate;
      }
    }
    const int low = voxel_of(min_p[axis]);
    const int split = low + (voxel_of(max_p[axis]) - low + 1) / 2;
    std::vector<std::size_t> lower;
    std::vector<std::size_t> upper;
    for (const std::size_t i : indices) {
      (voxel_of(input[i].data[axis]) < split ? lower : upper).push_back(i);
    }
    std::vector<std::size_t>().swap(indices);
    pending.push_back(std::move(upper));
    pending.push_back(std::move(lower));
  }

  // The cloud properties pcl::Filter::filter and VoxelGrid::applyFilter give their output.
  result.header = input.header;
  result.sensor_origin_ = input.sensor_origin_;
  result.sensor_orientation_ = input.sensor_orientation_;
  result.width = static_cast<std::uint32_t>(result.size());
  result.height = 1;
  result.is_dense = true;
  output = std::move(result);
}

}  // namespace map_saver
}  // namespace graphslam
