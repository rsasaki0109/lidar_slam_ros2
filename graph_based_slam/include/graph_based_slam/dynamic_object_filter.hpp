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

#ifndef GRAPH_BASED_SLAM__DYNAMIC_OBJECT_FILTER_HPP_
#define GRAPH_BASED_SLAM__DYNAMIC_OBJECT_FILTER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

namespace graphslam
{

struct DynamicObjectFilterConfig
{
  double voxel_size {0.3};
  int min_observations {2};
  int temporal_window {5};
  double max_range_from_sensor_m {30.0};
};

struct DynamicObjectFilterStats
{
  std::size_t input_points {0};
  std::size_t candidate_voxels {0};
  std::size_t kept_candidate_voxels {0};
  std::size_t removed_candidate_voxels {0};
  std::size_t always_keep_voxels {0};
  std::size_t output_points {0};
};

struct TimedSubmapCloud
{
  int submap_index {0};
  Eigen::Vector3d sensor_position {Eigen::Vector3d::Zero()};
  pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud;
};

struct DynamicObjectFilterResult
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZI>()};
  DynamicObjectFilterStats stats;
};

struct VoxelKey
{
  int x {0};
  int y {0};
  int z {0};

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    const std::uint64_t hx = static_cast<std::uint64_t>(static_cast<std::uint32_t>(key.x));
    const std::uint64_t hy = static_cast<std::uint64_t>(static_cast<std::uint32_t>(key.y));
    const std::uint64_t hz = static_cast<std::uint64_t>(static_cast<std::uint32_t>(key.z));
    std::uint64_t seed = hx * 0x9E3779B185EBCA87ULL;
    seed ^= hy + 0x9E3779B97F4A7C15ULL + (seed << 6U) + (seed >> 2U);
    seed ^= hz + 0xC2B2AE3D27D4EB4FULL + (seed << 6U) + (seed >> 2U);
    return static_cast<std::size_t>(seed);
  }
};

inline VoxelKey makeVoxelKey(const pcl::PointXYZI & point, double voxel_size)
{
  return VoxelKey{
    static_cast<int>(std::floor(point.x / voxel_size)),
    static_cast<int>(std::floor(point.y / voxel_size)),
    static_cast<int>(std::floor(point.z / voxel_size))};
}

inline bool hasTemporalConsistency(
  const std::vector<int> & submap_indices,
  int temporal_window,
  int min_observations)
{
  if (submap_indices.empty()) {
    return false;
  }
  if (min_observations <= 1) {
    return true;
  }

  std::size_t begin = 0;
  for (std::size_t end = 0; end < submap_indices.size(); ++end) {
    while (begin < end && (submap_indices[end] - submap_indices[begin]) > temporal_window) {
      ++begin;
    }
    if (static_cast<int>(end - begin + 1) >= min_observations) {
      return true;
    }
  }
  return false;
}

namespace detail
{

struct LocalVoxelAccumulator
{
  double x_sum {0.0};
  double y_sum {0.0};
  double z_sum {0.0};
  double intensity_sum {0.0};
  int count {0};

  void add(const pcl::PointXYZI & point)
  {
    x_sum += point.x;
    y_sum += point.y;
    z_sum += point.z;
    intensity_sum += point.intensity;
    ++count;
  }

  pcl::PointXYZI centroid() const
  {
    const double inv = 1.0 / static_cast<double>(count);
    pcl::PointXYZI point;
    point.x = static_cast<float>(x_sum * inv);
    point.y = static_cast<float>(y_sum * inv);
    point.z = static_cast<float>(z_sum * inv);
    point.intensity = static_cast<float>(intensity_sum * inv);
    return point;
  }
};

struct GlobalVoxelAccumulator
{
  double x_sum {0.0};
  double y_sum {0.0};
  double z_sum {0.0};
  double intensity_sum {0.0};
  int sample_count {0};
  int last_submap_index {std::numeric_limits<int>::min()};
  std::vector<int> observed_submaps;

  void add(const pcl::PointXYZI & point, int submap_index)
  {
    x_sum += point.x;
    y_sum += point.y;
    z_sum += point.z;
    intensity_sum += point.intensity;
    ++sample_count;
    if (last_submap_index != submap_index) {
      observed_submaps.push_back(submap_index);
      last_submap_index = submap_index;
    }
  }

  pcl::PointXYZI centroid() const
  {
    const double inv = 1.0 / static_cast<double>(sample_count);
    pcl::PointXYZI point;
    point.x = static_cast<float>(x_sum * inv);
    point.y = static_cast<float>(y_sum * inv);
    point.z = static_cast<float>(z_sum * inv);
    point.intensity = static_cast<float>(intensity_sum * inv);
    return point;
  }
};

}  // namespace detail

inline DynamicObjectFilterResult buildDynamicObjectFilteredMap(
  const std::vector<TimedSubmapCloud> & submap_clouds,
  const DynamicObjectFilterConfig & config)
{
  DynamicObjectFilterResult result;
  if (submap_clouds.empty()) {
    return result;
  }

  if (config.voxel_size <= 0.0) {
    return result;
  }

  std::unordered_map<VoxelKey, detail::GlobalVoxelAccumulator, VoxelKeyHash> candidate_voxels;
  std::unordered_map<VoxelKey, detail::GlobalVoxelAccumulator, VoxelKeyHash> always_keep_voxels;

  for (const auto & submap : submap_clouds) {
    if (!submap.cloud || submap.cloud->empty()) {
      continue;
    }
    result.stats.input_points += submap.cloud->size();

    std::unordered_map<
      VoxelKey,
      detail::LocalVoxelAccumulator,
      VoxelKeyHash> local_candidate_voxels;
    std::unordered_map<
      VoxelKey,
      detail::LocalVoxelAccumulator,
      VoxelKeyHash> local_keep_voxels;

    for (const auto & point : submap.cloud->points) {
      const Eigen::Vector3d point_vec(point.x, point.y, point.z);
      const double point_range = (point_vec - submap.sensor_position).norm();
      const bool always_keep =
        config.max_range_from_sensor_m > 0.0 && point_range > config.max_range_from_sensor_m;
      const VoxelKey key = makeVoxelKey(point, config.voxel_size);
      if (always_keep) {
        local_keep_voxels[key].add(point);
      } else {
        local_candidate_voxels[key].add(point);
      }
    }

    for (const auto & entry : local_keep_voxels) {
      always_keep_voxels[entry.first].add(entry.second.centroid(), submap.submap_index);
    }
    for (const auto & entry : local_candidate_voxels) {
      candidate_voxels[entry.first].add(entry.second.centroid(), submap.submap_index);
    }
  }

  result.stats.candidate_voxels = candidate_voxels.size();
  result.stats.always_keep_voxels = always_keep_voxels.size();

  for (const auto & entry : always_keep_voxels) {
    result.cloud->push_back(entry.second.centroid());
  }
  for (const auto & entry : candidate_voxels) {
    if (
      hasTemporalConsistency(
        entry.second.observed_submaps,
        config.temporal_window,
        config.min_observations))
    {
      result.cloud->push_back(entry.second.centroid());
      ++result.stats.kept_candidate_voxels;
    } else {
      ++result.stats.removed_candidate_voxels;
    }
  }

  result.stats.output_points = result.cloud->size();
  return result;
}

}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__DYNAMIC_OBJECT_FILTER_HPP_
