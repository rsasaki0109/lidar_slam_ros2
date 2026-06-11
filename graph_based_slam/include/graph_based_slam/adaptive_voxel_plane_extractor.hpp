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

#ifndef GRAPH_BASED_SLAM__ADAPTIVE_VOXEL_PLANE_EXTRACTOR_HPP_
#define GRAPH_BASED_SLAM__ADAPTIVE_VOXEL_PLANE_EXTRACTOR_HPP_

#include <Eigen/Core>  // NOLINT(build/include_order)
#include <Eigen/Eigenvalues>  // NOLINT(build/include_order)

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <vector>

namespace graphslam
{
namespace plane_extraction
{

/// Shared deterministic plane extractor for the v0.7 map-quality metrics
/// (Phase 1) and plane BA (Phase 2), docs/roadmap/v0.7.md.
struct PlaneExtractionConfig
{
  double root_voxel_size {1.0};
  int max_octree_depth {3};
  int min_points_per_plane {20};
  double max_plane_thickness {0.06};
  double min_planarity_ratio {6.0};
  bool enable_quarter_test {true};
  double quarter_test_tolerance {2.0};
};

struct PlanarPatch
{
  Eigen::Vector3d centroid;
  Eigen::Vector3d normal;
  double lambda_min;
  double lambda_mid;
  double lambda_max;
  double thickness_rms;
  int point_count;
  int depth;
};

struct PlaneExtractionResult
{
  std::vector<PlanarPatch> patches;
  std::int64_t total_points {0};
  std::int64_t planar_points {0};
  double planar_coverage {0.0};
};

namespace detail
{

struct VoxelKey
{
  std::int64_t x;
  std::int64_t y;
  std::int64_t z;
};

inline bool operator<(const VoxelKey & lhs, const VoxelKey & rhs)
{
  if (lhs.x != rhs.x) {
    return lhs.x < rhs.x;
  }
  if (lhs.y != rhs.y) {
    return lhs.y < rhs.y;
  }
  return lhs.z < rhs.z;
}

struct NodeBounds
{
  Eigen::Vector3d min_corner;
  double size;
};

struct NodeStats
{
  Eigen::Vector3d centroid;
  Eigen::Matrix3d covariance;
  Eigen::Vector3d eigenvalues;
  Eigen::Matrix3d eigenvectors;
};

inline std::int64_t floorToLongLong(const double value)
{
  return static_cast<std::int64_t>(std::floor(value));
}

inline VoxelKey makeRootKey(
  const Eigen::Vector3d & point,
  const double root_voxel_size)
{
  VoxelKey key;
  key.x = floorToLongLong(point.x() / root_voxel_size);
  key.y = floorToLongLong(point.y() / root_voxel_size);
  key.z = floorToLongLong(point.z() / root_voxel_size);
  return key;
}

inline Eigen::Vector3d makeMinCorner(
  const VoxelKey & key,
  const double root_voxel_size)
{
  return Eigen::Vector3d(
    static_cast<double>(key.x) * root_voxel_size,
    static_cast<double>(key.y) * root_voxel_size,
    static_cast<double>(key.z) * root_voxel_size);
}

inline double clampSmallEigenvalue(const double value)
{
  if (value < 0.0 && value > -1.0e-12) {
    return 0.0;
  }
  return value;
}

inline NodeStats computeNodeStats(
  const std::vector<Eigen::Vector3d> & points,
  const std::vector<int> & indices)
{
  NodeStats stats;
  stats.centroid = Eigen::Vector3d::Zero();
  stats.covariance = Eigen::Matrix3d::Zero();

  for (std::size_t i = 0; i < indices.size(); ++i) {
    stats.centroid += points[indices[i]];
  }
  stats.centroid /= static_cast<double>(indices.size());

  for (std::size_t i = 0; i < indices.size(); ++i) {
    const Eigen::Vector3d delta = points[indices[i]] - stats.centroid;
    stats.covariance += delta * delta.transpose();
  }
  stats.covariance /= static_cast<double>(indices.size());

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(stats.covariance);
  stats.eigenvalues = solver.eigenvalues();
  stats.eigenvectors = solver.eigenvectors();

  stats.eigenvalues.x() = clampSmallEigenvalue(stats.eigenvalues.x());
  stats.eigenvalues.y() = clampSmallEigenvalue(stats.eigenvalues.y());
  stats.eigenvalues.z() = clampSmallEigenvalue(stats.eigenvalues.z());

  return stats;
}

inline int largestAbsComponentIndex(const Eigen::Vector3d & vector)
{
  const double ax = std::abs(vector.x());
  const double ay = std::abs(vector.y());
  const double az = std::abs(vector.z());

  if (ax >= ay && ax >= az) {
    return 0;
  }
  if (ay >= az) {
    return 1;
  }
  return 2;
}

inline Eigen::Vector3d canonicalizeNormal(const Eigen::Vector3d & normal)
{
  Eigen::Vector3d canonical = normal.normalized();
  const int largest_index = largestAbsComponentIndex(canonical);

  if (canonical(largest_index) < 0.0) {
    canonical = -canonical;
  }
  return canonical;
}

inline int octantForPoint(
  const Eigen::Vector3d & point,
  const Eigen::Vector3d & center)
{
  int octant = 0;
  if (point.x() >= center.x()) {
    octant |= 1;
  }
  if (point.y() >= center.y()) {
    octant |= 2;
  }
  if (point.z() >= center.z()) {
    octant |= 4;
  }
  return octant;
}

inline NodeBounds childBounds(
  const NodeBounds & parent,
  const int octant)
{
  NodeBounds child;
  child.size = parent.size * 0.5;
  child.min_corner = parent.min_corner;

  if ((octant & 1) != 0) {
    child.min_corner.x() += child.size;
  }
  if ((octant & 2) != 0) {
    child.min_corner.y() += child.size;
  }
  if ((octant & 4) != 0) {
    child.min_corner.z() += child.size;
  }
  return child;
}

inline bool passesBasicPlaneTest(
  const NodeStats & stats,
  const std::size_t point_count,
  const PlaneExtractionConfig & config)
{
  if (point_count < static_cast<std::size_t>(config.min_points_per_plane)) {
    return false;
  }

  const double lambda_min = stats.eigenvalues.x();
  const double lambda_mid = stats.eigenvalues.y();
  const double thickness_rms = std::sqrt(std::max(0.0, lambda_min));

  if (thickness_rms > config.max_plane_thickness) {
    return false;
  }
  if (lambda_mid < config.min_planarity_ratio * lambda_min) {
    return false;
  }
  return true;
}

inline bool passesQuarterTest(
  const std::vector<Eigen::Vector3d> & points,
  const std::vector<int> & indices,
  const NodeBounds & bounds,
  const NodeStats & stats,
  const PlaneExtractionConfig & config)
{
  if (!config.enable_quarter_test) {
    return true;
  }

  std::vector<int> octant_indices[8];
  const Eigen::Vector3d center = bounds.min_corner + Eigen::Vector3d::Constant(bounds.size * 0.5);

  for (std::size_t i = 0; i < indices.size(); ++i) {
    const int octant = octantForPoint(points[indices[i]], center);
    octant_indices[octant].push_back(indices[i]);
  }

  const int min_quarter_points = std::max(config.min_points_per_plane / 4, 5);
  int populated_octants = 0;

  for (int octant = 0; octant < 8; ++octant) {
    if (octant_indices[octant].size() >= static_cast<std::size_t>(min_quarter_points)) {
      ++populated_octants;
    }
  }

  if (populated_octants < 2) {
    return true;
  }

  const double lambda_limit = config.quarter_test_tolerance * stats.eigenvalues.x() + 1.0e-12;
  for (int octant = 0; octant < 8; ++octant) {
    if (octant_indices[octant].size() < static_cast<std::size_t>(min_quarter_points)) {
      continue;
    }

    const NodeStats quarter_stats = computeNodeStats(points, octant_indices[octant]);
    if (quarter_stats.eigenvalues.x() > lambda_limit) {
      return false;
    }
  }
  return true;
}

inline PlanarPatch makePatch(
  const NodeStats & stats,
  const std::size_t point_count,
  const int depth)
{
  PlanarPatch patch;
  patch.centroid = stats.centroid;
  patch.normal = canonicalizeNormal(stats.eigenvectors.col(0));
  patch.lambda_min = stats.eigenvalues.x();
  patch.lambda_mid = stats.eigenvalues.y();
  patch.lambda_max = stats.eigenvalues.z();
  patch.thickness_rms = std::sqrt(std::max(0.0, patch.lambda_min));
  patch.point_count = static_cast<int>(point_count);
  patch.depth = depth;
  return patch;
}

inline void extractRecursive(
  const std::vector<Eigen::Vector3d> & points,
  const std::vector<int> & indices,
  const NodeBounds & bounds,
  const int depth,
  const PlaneExtractionConfig & config,
  std::vector<PlanarPatch> * patches)
{
  if (indices.empty()) {
    return;
  }

  const NodeStats stats = computeNodeStats(points, indices);
  const bool accepted =
    passesBasicPlaneTest(stats, indices.size(), config) &&
    passesQuarterTest(points, indices, bounds, stats, config);

  if (accepted) {
    patches->push_back(makePatch(stats, indices.size(), depth));
    return;
  }

  if (depth >= config.max_octree_depth) {
    return;
  }
  if (indices.size() < static_cast<std::size_t>(2 * config.min_points_per_plane)) {
    return;
  }

  std::vector<int> child_indices[8];
  const Eigen::Vector3d center = bounds.min_corner + Eigen::Vector3d::Constant(bounds.size * 0.5);

  for (std::size_t i = 0; i < indices.size(); ++i) {
    const int octant = octantForPoint(points[indices[i]], center);
    child_indices[octant].push_back(indices[i]);
  }

  for (int octant = 0; octant < 8; ++octant) {
    if (child_indices[octant].empty()) {
      continue;
    }

    const NodeBounds child = childBounds(bounds, octant);
    extractRecursive(points, child_indices[octant], child, depth + 1, config, patches);
  }
}

inline PlaneExtractionConfig sanitizeConfig(const PlaneExtractionConfig & config)
{
  PlaneExtractionConfig sanitized = config;
  if (sanitized.root_voxel_size <= 0.0) {
    sanitized.root_voxel_size = 1.0;
  }
  if (sanitized.max_octree_depth < 0) {
    sanitized.max_octree_depth = 0;
  }
  if (sanitized.min_points_per_plane < 1) {
    sanitized.min_points_per_plane = 1;
  }
  if (sanitized.max_plane_thickness < 0.0) {
    sanitized.max_plane_thickness = 0.0;
  }
  if (sanitized.min_planarity_ratio < 0.0) {
    sanitized.min_planarity_ratio = 0.0;
  }
  if (sanitized.quarter_test_tolerance < 0.0) {
    sanitized.quarter_test_tolerance = 0.0;
  }
  return sanitized;
}

}  // namespace detail

inline PlaneExtractionResult extractPlanarPatches(
  const std::vector<Eigen::Vector3d> & points,
  const PlaneExtractionConfig & config)
{
  PlaneExtractionResult result;
  result.total_points = static_cast<std::int64_t>(points.size());

  if (points.empty()) {
    return result;
  }

  const PlaneExtractionConfig sanitized = detail::sanitizeConfig(config);
  std::map<detail::VoxelKey, std::vector<int>> voxel_indices;

  for (std::size_t i = 0; i < points.size(); ++i) {
    const detail::VoxelKey key = detail::makeRootKey(points[i], sanitized.root_voxel_size);
    voxel_indices[key].push_back(static_cast<int>(i));
  }

  for (std::map<detail::VoxelKey, std::vector<int>>::const_iterator iter = voxel_indices.begin();
    iter != voxel_indices.end(); ++iter)
  {
    detail::NodeBounds bounds;
    bounds.min_corner = detail::makeMinCorner(iter->first, sanitized.root_voxel_size);
    bounds.size = sanitized.root_voxel_size;

    detail::extractRecursive(
      points,
      iter->second,
      bounds,
      0,
      sanitized,
      &result.patches);
  }

  for (std::size_t i = 0; i < result.patches.size(); ++i) {
    result.planar_points += result.patches[i].point_count;
  }
  if (result.total_points > 0) {
    result.planar_coverage =
      static_cast<double>(result.planar_points) / static_cast<double>(result.total_points);
  }
  return result;
}

}  // namespace plane_extraction
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__ADAPTIVE_VOXEL_PLANE_EXTRACTOR_HPP_
