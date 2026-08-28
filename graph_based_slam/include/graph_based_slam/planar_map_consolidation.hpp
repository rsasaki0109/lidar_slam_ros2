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

#ifndef GRAPH_BASED_SLAM__PLANAR_MAP_CONSOLIDATION_HPP_
#define GRAPH_BASED_SLAM__PLANAR_MAP_CONSOLIDATION_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <unordered_map>

#include "graph_based_slam/planar_map_filter.hpp"

namespace graphslam
{

struct PlanarMapConsolidationConfig
{
  double voxel_size {0.3};
  int min_neighbors {12};
  double max_small_eigenvalue_ratio {0.05};
  double min_middle_eigenvalue_ratio {0.05};
  double max_plane_distance_m {0.10};
  double projection_gain {0.5};
  double max_displacement_m {0.02};
  double min_supported_ratio {0.10};
};

struct PlanarMapConsolidationStats
{
  std::size_t input_points {0};
  std::size_t finite_points {0};
  std::size_t voxel_count {0};
  std::size_t planar_voxels {0};
  std::size_t supported_points {0};
  std::size_t projected_points {0};
  std::size_t output_points {0};
  double mean_displacement_m {0.0};
  double max_displacement_m {0.0};
  bool fallback_to_input {false};
};

struct PlanarMapConsolidationResult
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZI>()};
  PlanarMapConsolidationStats stats;
};

namespace planar_map_consolidation_detail
{

struct PlaneModel
{
  Eigen::Vector3d mean {Eigen::Vector3d::Zero()};
  Eigen::Vector3d normal {Eigen::Vector3d::UnitZ()};
};

}  // namespace planar_map_consolidation_detail

// Consolidate already-export-sized map points toward well-supported local
// planes without deleting points. Projection is bounded twice: points too far
// from the fitted plane are left untouched, and accepted displacements are
// clamped. This makes the operation a surface-noise reducer rather than a
// geometry replacement step.
inline PlanarMapConsolidationResult buildPlanarMapConsolidatedMap(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & input,
  const PlanarMapConsolidationConfig & config)
{
  using planar_map_consolidation_detail::PlaneModel;
  using planar_map_filter_detail::Moments;
  using planar_map_filter_detail::VoxelKey;
  using planar_map_filter_detail::VoxelKeyHash;

  PlanarMapConsolidationResult result;
  if (!input) {
    return result;
  }
  result.stats.input_points = input->size();
  *result.cloud = *input;
  result.stats.output_points = input->size();
  if (input->empty() || config.voxel_size <= 0.0 || config.min_neighbors < 3 ||
    config.max_plane_distance_m <= 0.0 || config.max_displacement_m < 0.0)
  {
    result.stats.fallback_to_input = true;
    return result;
  }

  std::unordered_map<VoxelKey, Moments, VoxelKeyHash> voxels;
  voxels.reserve(input->size() / 4U + 1U);
  for (const auto & point : input->points) {
    if (!planar_map_filter_detail::isFinite(point)) {
      continue;
    }
    ++result.stats.finite_points;
    voxels[planar_map_filter_detail::voxelKey(point, config.voxel_size)].add(point);
  }
  result.stats.voxel_count = voxels.size();

  std::unordered_map<VoxelKey, PlaneModel, VoxelKeyHash> planes;
  planes.reserve(voxels.size());
  for (const auto & entry : voxels) {
    Moments neighborhood;
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          const VoxelKey neighbor{entry.first.x + dx, entry.first.y + dy, entry.first.z + dz};
          const auto found = voxels.find(neighbor);
          if (found != voxels.end()) {
            neighborhood.add(found->second);
          }
        }
      }
    }
    if (neighborhood.count < static_cast<std::size_t>(config.min_neighbors)) {
      continue;
    }

    const double count = static_cast<double>(neighborhood.count);
    const Eigen::Vector3d mean = neighborhood.sum / count;
    Eigen::Matrix3d covariance = neighborhood.outer_sum / count - mean * mean.transpose();
    covariance = 0.5 * (covariance + covariance.transpose());
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success) {
      continue;
    }
    const Eigen::Vector3d eigenvalues = solver.eigenvalues().cwiseMax(0.0);
    const double sum = eigenvalues.sum();
    if (!(sum > std::numeric_limits<double>::epsilon())) {
      continue;
    }
    if (eigenvalues.x() / sum > config.max_small_eigenvalue_ratio ||
      eigenvalues.y() / sum < config.min_middle_eigenvalue_ratio)
    {
      continue;
    }
    PlaneModel model;
    model.mean = mean;
    model.normal = solver.eigenvectors().col(0).normalized();
    planes.emplace(entry.first, model);
    ++result.stats.planar_voxels;
  }

  for (const auto & point : input->points) {
    if (!planar_map_filter_detail::isFinite(point)) {
      continue;
    }
    const auto found = planes.find(
      planar_map_filter_detail::voxelKey(point, config.voxel_size));
    if (found == planes.end()) {
      continue;
    }
    const Eigen::Vector3d position(point.x, point.y, point.z);
    const double distance = found->second.normal.dot(position - found->second.mean);
    if (std::abs(distance) <= config.max_plane_distance_m) {
      ++result.stats.supported_points;
    }
  }

  const double supported_ratio = input->empty() ? 0.0 :
    static_cast<double>(result.stats.supported_points) / static_cast<double>(input->size());
  const double min_supported_ratio = std::max(0.0, std::min(1.0, config.min_supported_ratio));
  if (supported_ratio < min_supported_ratio) {
    result.stats.fallback_to_input = true;
    return result;
  }

  const double gain = std::max(0.0, std::min(1.0, config.projection_gain));
  double displacement_sum = 0.0;
  for (auto & point : result.cloud->points) {
    if (!planar_map_filter_detail::isFinite(point)) {
      continue;
    }
    const auto found = planes.find(
      planar_map_filter_detail::voxelKey(point, config.voxel_size));
    if (found == planes.end()) {
      continue;
    }
    Eigen::Vector3d position(point.x, point.y, point.z);
    const double distance = found->second.normal.dot(position - found->second.mean);
    if (std::abs(distance) > config.max_plane_distance_m) {
      continue;
    }
    const double displacement = std::min(
      config.max_displacement_m, gain * std::abs(distance));
    if (!(displacement > 0.0)) {
      continue;
    }
    position -= std::copysign(displacement, distance) * found->second.normal;
    point.x = static_cast<float>(position.x());
    point.y = static_cast<float>(position.y());
    point.z = static_cast<float>(position.z());
    ++result.stats.projected_points;
    displacement_sum += displacement;
    result.stats.max_displacement_m = std::max(
      result.stats.max_displacement_m, displacement);
  }
  if (result.stats.projected_points > 0U) {
    result.stats.mean_displacement_m = displacement_sum /
      static_cast<double>(result.stats.projected_points);
  }
  return result;
}

}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__PLANAR_MAP_CONSOLIDATION_HPP_
