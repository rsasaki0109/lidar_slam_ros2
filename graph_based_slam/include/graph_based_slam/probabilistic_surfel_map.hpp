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

#ifndef GRAPH_BASED_SLAM__PROBABILISTIC_SURFEL_MAP_HPP_
#define GRAPH_BASED_SLAM__PROBABILISTIC_SURFEL_MAP_HPP_

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <tuple>
#include <utility>
#include <vector>

#include "graph_based_slam/probabilistic_surfel_fusion.hpp"

namespace graphslam
{

struct ProbabilisticSurfelMapScan
{
  std::uint64_t scan_id {0};
  Eigen::Vector3d sensor_origin {Eigen::Vector3d::Zero()};
  double pose_translation_variance_m2 {0.0};
  double pose_rotation_variance_rad2 {0.0};
  std::vector<Eigen::Vector3d> world_points;
};

struct ProbabilisticSurfelMapConfig
{
  // Fine output occupancy and coarser plane-estimation support are separate.
  // The former preserves coverage; the latter gives PCA enough spatial extent.
  double voxel_size_m {0.10};
  double surfel_support_voxel_size_m {0.30};
  double secondary_support_voxel_size_m {0.0};
  double tertiary_support_voxel_size_m {0.0};
  int support_grid_phases {1};
  bool blend_support_phases {false};
  bool support_phases_fallback_only {false};
  bool build_surface_consolidated_map {false};
  double surface_consolidation_min_projection_distance_m {0.0};
  bool build_support_partition_maps {false};
  bool build_connected_surface_map {false};
  double connected_surface_max_normal_angle_deg {8.0};
  double connected_surface_max_plane_distance_m {0.04};
  std::size_t connected_surface_min_support_cells {3};
  bool connected_surface_extend_fallback {true};
  double connected_surface_max_extension_distance_m {0.04};
  std::size_t connected_surface_min_extension_support_cells {2};
  ProbabilisticSurfelFusionConfig fusion {};
  bool build_persistence_filtered_map {false};
  std::size_t persistence_min_distinct_scans {3};
  std::uint64_t persistence_min_scan_span {3};
  double persistence_max_filter_range_m {30.0};
  bool build_visibility_filtered_map {false};
  std::size_t visibility_max_distinct_scans {2};
  std::uint64_t visibility_max_scan_span {2};
  std::size_t visibility_near_scan_offset {5};
  std::size_t visibility_far_scan_offset {15};
  std::size_t visibility_min_free_space_votes {2};
  double visibility_angular_resolution_rad {0.004363323129985824};  // 0.25 degree
  double visibility_free_space_margin_m {0.50};
  double visibility_max_range_m {30.0};
  double visibility_max_origin_displacement_m {3.0};
};

struct ProbabilisticSurfelMapStats
{
  std::size_t input_scans {0};
  std::size_t input_points {0};
  std::size_t finite_points {0};
  std::size_t occupied_voxels {0};
  std::size_t occupied_support_voxels {0};
  std::size_t valid_support_surfels {0};
  std::size_t fused_surfel_voxels {0};
  std::size_t fallback_centroid_voxels {0};
  std::size_t shifted_phase_fused_voxels {0};
  std::size_t surface_consolidation_input_points {0};
  std::size_t surface_consolidation_selected_points {0};
  std::size_t surface_consolidation_output_points {0};
  std::size_t surface_consolidation_merged_points {0};
  std::size_t connected_surface_support_cells {0};
  std::size_t connected_surface_merged_cells {0};
  std::size_t connected_surface_projected_voxels {0};
  std::size_t connected_surface_extended_fallback_voxels {0};
  double mean_raw_normal_rms_m {0.0};
  double mean_fused_normal_sigma_m {0.0};
  std::size_t persistence_candidate_voxels {0};
  std::size_t persistence_kept_voxels {0};
  std::size_t persistence_removed_voxels {0};
  std::size_t persistence_far_range_keep_voxels {0};
  std::size_t visibility_candidate_voxels {0};
  std::size_t visibility_tested_voxels {0};
  std::size_t visibility_contradicted_voxels {0};
  std::size_t visibility_removed_voxels {0};
  std::size_t visibility_kept_voxels {0};
};

struct ProbabilisticSurfelMapResult
{
  // Both arrays have the same size and key order. The baseline makes it
  // impossible to win a thickness comparison merely by deleting voxels.
  std::vector<Eigen::Vector3d> baseline_centroids;
  std::vector<Eigen::Vector3d> fused_points;
  std::vector<Eigen::Vector3d> surface_consolidated_points;
  std::vector<Eigen::Vector3d> supported_partition_points;
  std::vector<Eigen::Vector3d> fallback_partition_points;
  std::vector<Eigen::Vector3d> connected_surface_points;
  std::vector<Eigen::Vector3d> persistence_filtered_points;
  std::vector<Eigen::Vector3d> visibility_filtered_points;
  ProbabilisticSurfelMapStats stats;
};

namespace probabilistic_surfel_map_detail
{

using VoxelKey = std::tuple<std::int64_t, std::int64_t, std::int64_t>;

inline VoxelKey voxelKey(const Eigen::Vector3d & point, const double voxel_size)
{
  return VoxelKey(
    static_cast<std::int64_t>(std::floor(point.x() / voxel_size)),
    static_cast<std::int64_t>(std::floor(point.y() / voxel_size)),
    static_cast<std::int64_t>(std::floor(point.z() / voxel_size)));
}

inline Eigen::Vector3d deterministicCentroid(std::vector<SurfelObservation> observations)
{
  std::sort(
    observations.begin(), observations.end(),
    probabilistic_surfel_detail::observationLess);
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  for (const SurfelObservation & observation : observations) {
    sum += observation.position;
  }
  return sum / static_cast<double>(observations.size());
}

inline Eigen::Vector3d clampToVoxel(
  const Eigen::Vector3d & point, const VoxelKey & key, const double voxel_size)
{
  Eigen::Vector3d clamped = point;
  const std::array<std::int64_t, 3> indices{
    std::get<0>(key), std::get<1>(key), std::get<2>(key)};
  // PCD PointXYZ stores float. Keep a small interior margin so converting to
  // float cannot round a clamped upper face into the adjacent output voxel.
  const double margin = voxel_size * 1.0e-3;
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    const double lower = static_cast<double>(indices[axis]) * voxel_size;
    const double upper = lower + voxel_size;
    clamped[axis] = std::max(lower + margin, std::min(clamped[axis], upper - margin));
  }
  return clamped;
}

inline bool hasPersistence(
  std::vector<SurfelObservation> observations, const std::size_t min_distinct_scans,
  const std::uint64_t min_scan_span)
{
  std::sort(observations.begin(), observations.end(),
    probabilistic_surfel_detail::observationLess);
  std::size_t distinct_scans = 0U;
  std::uint64_t first_scan = 0U;
  std::uint64_t last_scan = 0U;
  for (std::size_t i = 0; i < observations.size(); ++i) {
    if (i == 0U || observations[i].scan_id != observations[i - 1U].scan_id) {
      if (distinct_scans == 0U) {first_scan = observations[i].scan_id;}
      last_scan = observations[i].scan_id;
      ++distinct_scans;
    }
  }
  return distinct_scans >= min_distinct_scans &&
         last_scan - first_scan >= min_scan_span;
}

struct KeyedPoint
{
  VoxelKey key;
  Eigen::Vector3d position {Eigen::Vector3d::Zero()};
  std::uint64_t scan_id {0};
  std::uint32_t scan_index {0};
  std::uint32_t output_index {0};
};

struct KeyedOutputPoint
{
  VoxelKey key;
  Eigen::Vector3d position {Eigen::Vector3d::Zero()};
  std::size_t original_index {0};
};

inline bool keyedOutputPointLess(const KeyedOutputPoint & lhs, const KeyedOutputPoint & rhs)
{
  if (lhs.key != rhs.key) {return lhs.key < rhs.key;}
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    if (lhs.position[axis] != rhs.position[axis]) {
      return lhs.position[axis] < rhs.position[axis];
    }
  }
  return lhs.original_index < rhs.original_index;
}

inline std::vector<Eigen::Vector3d> deterministicVoxelCentroids(
  const std::vector<Eigen::Vector3d> & input, const double voxel_size_m)
{
  std::vector<KeyedOutputPoint> keyed;
  keyed.reserve(input.size());
  for (std::size_t i = 0; i < input.size(); ++i) {
    if (!input[i].allFinite()) {continue;}
    keyed.push_back(KeyedOutputPoint{voxelKey(input[i], voxel_size_m), input[i], i});
  }
  std::sort(keyed.begin(), keyed.end(), keyedOutputPointLess);
  std::vector<Eigen::Vector3d> output;
  output.reserve(keyed.size());
  for (std::size_t first = 0U; first < keyed.size(); ) {
    std::size_t last = first + 1U;
    while (last < keyed.size() && keyed[last].key == keyed[first].key) {++last;}
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (std::size_t i = first; i < last; ++i) {
      centroid += keyed[i].position;
    }
    output.push_back(centroid / static_cast<double>(last - first));
    first = last;
  }
  return output;
}

inline bool keyedPointLess(const KeyedPoint & lhs, const KeyedPoint & rhs)
{
  if (lhs.key != rhs.key) {return lhs.key < rhs.key;}
  if (lhs.scan_id != rhs.scan_id) {return lhs.scan_id < rhs.scan_id;}
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    if (lhs.position[axis] != rhs.position[axis]) {
      return lhs.position[axis] < rhs.position[axis];
    }
  }
  if (lhs.scan_index != rhs.scan_index) {return lhs.scan_index < rhs.scan_index;}
  return lhs.output_index < rhs.output_index;
}

struct SupportOutputRef
{
  VoxelKey key;
  std::size_t output_index {0};
};

struct SupportSurfelCell
{
  VoxelKey key;
  ProbabilisticSurfel surfel;
};

inline bool supportSurfelCellLess(
  const SupportSurfelCell & lhs, const SupportSurfelCell & rhs)
{
  return lhs.key < rhs.key;
}

inline VoxelKey offsetVoxelKey(
  const VoxelKey & key, const std::int64_t dx, const std::int64_t dy,
  const std::int64_t dz)
{
  return VoxelKey(
    std::get<0>(key) + dx, std::get<1>(key) + dy, std::get<2>(key) + dz);
}

inline bool compatibleSupportSurfels(
  const ProbabilisticSurfel & reference, const ProbabilisticSurfel & candidate,
  const double minimum_normal_dot, const double max_plane_distance_m)
{
  const Eigen::Vector3d delta = candidate.mean - reference.mean;
  return std::abs(reference.normal.dot(candidate.normal)) >= minimum_normal_dot &&
         std::abs(reference.normal.dot(delta)) <= max_plane_distance_m &&
         std::abs(candidate.normal.dot(delta)) <= max_plane_distance_m;
}

inline ProbabilisticSurfel mergeConnectedSupportNeighborhood(
  const std::vector<SupportSurfelCell> & cells, const std::size_t center_index,
  const double minimum_normal_dot, const double max_plane_distance_m,
  const std::size_t min_support_cells)
{
  ProbabilisticSurfel merged;
  if (center_index >= cells.size()) {return merged;}
  const SupportSurfelCell & center = cells[center_index];
  std::vector<const ProbabilisticSurfel *> members;
  members.push_back(&center.surfel);
  for (std::int64_t dx = -1; dx <= 1; ++dx) {
    for (std::int64_t dy = -1; dy <= 1; ++dy) {
      for (std::int64_t dz = -1; dz <= 1; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) {continue;}
        const VoxelKey neighbor_key = offsetVoxelKey(center.key, dx, dy, dz);
        const auto found = std::lower_bound(
          cells.begin(), cells.end(), SupportSurfelCell{neighbor_key, {}},
          supportSurfelCellLess);
        if (found == cells.end() || found->key != neighbor_key ||
          !compatibleSupportSurfels(
            center.surfel, found->surfel, minimum_normal_dot, max_plane_distance_m))
        {
          continue;
        }
        members.push_back(&found->surfel);
      }
    }
  }
  if (members.size() < min_support_cells) {return merged;}

  double weight_sum = 0.0;
  for (const ProbabilisticSurfel * surfel : members) {
    const double weight = static_cast<double>(std::max<std::size_t>(1U, surfel->distinct_scans));
    merged.mean += weight * surfel->mean;
    weight_sum += weight;
  }
  merged.mean /= weight_sum;
  for (const ProbabilisticSurfel * surfel : members) {
    const double weight = static_cast<double>(std::max<std::size_t>(1U, surfel->distinct_scans));
    const Eigen::Vector3d delta = surfel->mean - merged.mean;
    merged.covariance.noalias() += weight *
      (surfel->covariance + delta * delta.transpose());
    merged.input_observations += surfel->input_observations;
    merged.distinct_scans += surfel->distinct_scans;
    merged.raw_normal_rms_m += weight * surfel->raw_normal_rms_m;
    merged.fused_normal_sigma_m += weight * surfel->fused_normal_sigma_m;
  }
  merged.covariance /= weight_sum;
  merged.raw_normal_rms_m /= weight_sum;
  merged.fused_normal_sigma_m /= weight_sum;
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(merged.covariance);
  if (solver.info() != Eigen::Success) {return ProbabilisticSurfel{};}
  merged.normal = solver.eigenvectors().col(0).normalized();
  probabilistic_surfel_detail::orientNormalDeterministically(merged.normal);
  if (std::abs(merged.normal.dot(center.surfel.normal)) < minimum_normal_dot) {
    return ProbabilisticSurfel{};
  }
  merged.valid = merged.mean.allFinite() && merged.normal.allFinite() &&
    merged.covariance.allFinite();
  return merged;
}

struct VoxelObservationSpan
{
  std::uint32_t first_scan_rank {0};
  std::uint32_t last_scan_rank {0};
  std::uint32_t first_scan_index {0};
  std::uint32_t last_scan_index {0};
  std::uint32_t distinct_scans {0};
  bool all_far_range {false};
};

struct AngularReturn
{
  std::int64_t key {0};
  double range_m {0.0};
};

inline bool angularReturnLess(const AngularReturn & lhs, const AngularReturn & rhs)
{
  if (lhs.key != rhs.key) {return lhs.key < rhs.key;}
  return lhs.range_m < rhs.range_m;
}

inline std::int64_t angularBinKey(
  const Eigen::Vector3d & ray, const double angular_resolution_rad)
{
  const double horizontal = std::hypot(ray.x(), ray.y());
  const double azimuth = std::atan2(ray.y(), ray.x());
  const double elevation = std::atan2(ray.z(), horizontal);
  const std::int32_t azimuth_bin = static_cast<std::int32_t>(
    std::floor(azimuth / angular_resolution_rad));
  const std::int32_t elevation_bin = static_cast<std::int32_t>(
    std::floor(elevation / angular_resolution_rad));
  const std::uint64_t packed =
    (static_cast<std::uint64_t>(static_cast<std::uint32_t>(azimuth_bin)) << 32) |
    static_cast<std::uint32_t>(elevation_bin);
  return static_cast<std::int64_t>(packed);
}

inline bool supportOutputRefLess(const SupportOutputRef & lhs, const SupportOutputRef & rhs)
{
  if (lhs.key != rhs.key) {return lhs.key < rhs.key;}
  return lhs.output_index < rhs.output_index;
}

inline VoxelKey phasedSupportVoxelKey(
  const Eigen::Vector3d & point, const double voxel_size, const int phase)
{
  Eigen::Vector3d shifted = point;
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    if ((phase & (1 << axis)) != 0) {shifted[axis] += 0.5 * voxel_size;}
  }
  return voxelKey(shifted, voxel_size);
}

}  // namespace probabilistic_surfel_map_detail

inline ProbabilisticSurfelMapResult buildProbabilisticSurfelMap(
  const std::vector<ProbabilisticSurfelMapScan> & scans,
  const ProbabilisticSurfelMapConfig & config = {})
{
  ProbabilisticSurfelMapResult result;
  result.stats.input_scans = scans.size();
  if (!(config.voxel_size_m > 0.0) || !std::isfinite(config.voxel_size_m) ||
    !(config.surfel_support_voxel_size_m >= config.voxel_size_m) ||
    !std::isfinite(config.surfel_support_voxel_size_m) ||
    (config.secondary_support_voxel_size_m != 0.0 &&
    config.secondary_support_voxel_size_m < config.voxel_size_m) ||
    !std::isfinite(config.secondary_support_voxel_size_m) ||
    (config.tertiary_support_voxel_size_m != 0.0 &&
    config.tertiary_support_voxel_size_m < config.voxel_size_m) ||
    !std::isfinite(config.tertiary_support_voxel_size_m) ||
    config.support_grid_phases < 1 || config.support_grid_phases > 8 ||
    !(config.surface_consolidation_min_projection_distance_m >= 0.0) ||
    !std::isfinite(config.surface_consolidation_min_projection_distance_m) ||
    (config.support_phases_fallback_only && config.blend_support_phases) ||
    (config.build_connected_surface_map &&
    (config.support_grid_phases != 1 || config.blend_support_phases ||
    !(config.connected_surface_max_normal_angle_deg > 0.0) ||
    !(config.connected_surface_max_normal_angle_deg < 90.0) ||
    !std::isfinite(config.connected_surface_max_normal_angle_deg) ||
    !(config.connected_surface_max_plane_distance_m > 0.0) ||
    !std::isfinite(config.connected_surface_max_plane_distance_m) ||
    config.connected_surface_min_support_cells < 2U ||
    config.connected_surface_min_support_cells > 27U ||
    !(config.connected_surface_max_extension_distance_m > 0.0) ||
    !std::isfinite(config.connected_surface_max_extension_distance_m) ||
    config.connected_surface_min_extension_support_cells < 2U ||
    config.connected_surface_min_extension_support_cells > 27U)) ||
    (config.build_visibility_filtered_map &&
    (config.visibility_max_distinct_scans == 0U ||
    config.visibility_near_scan_offset == 0U ||
    config.visibility_far_scan_offset < config.visibility_near_scan_offset ||
    config.visibility_min_free_space_votes == 0U ||
    config.visibility_min_free_space_votes > std::numeric_limits<std::uint8_t>::max() ||
    !(config.visibility_angular_resolution_rad > 0.0) ||
    !std::isfinite(config.visibility_angular_resolution_rad) ||
    !(config.visibility_free_space_margin_m > 0.0) ||
    !std::isfinite(config.visibility_free_space_margin_m) ||
    !(config.visibility_max_range_m > 0.0) ||
    !std::isfinite(config.visibility_max_range_m) ||
    !(config.visibility_max_origin_displacement_m >= 0.0) ||
    !std::isfinite(config.visibility_max_origin_displacement_m))))
  {
    return result;
  }

  using probabilistic_surfel_map_detail::KeyedPoint;
  using probabilistic_surfel_map_detail::SupportOutputRef;
  using probabilistic_surfel_map_detail::SupportSurfelCell;
  using probabilistic_surfel_map_detail::VoxelKey;
  using probabilistic_surfel_map_detail::VoxelObservationSpan;
  std::vector<std::uint32_t> scan_indices_by_rank(scans.size());
  for (std::size_t i = 0; i < scans.size(); ++i) {
    scan_indices_by_rank[i] = static_cast<std::uint32_t>(i);
  }
  std::sort(
    scan_indices_by_rank.begin(), scan_indices_by_rank.end(),
    [&](const std::uint32_t lhs, const std::uint32_t rhs) {
      if (scans[lhs].scan_id != scans[rhs].scan_id) {
        return scans[lhs].scan_id < scans[rhs].scan_id;
      }
      return lhs < rhs;
    });
  std::vector<std::uint32_t> scan_rank_by_index(scans.size(), 0U);
  for (std::size_t rank = 0; rank < scan_indices_by_rank.size(); ++rank) {
    scan_rank_by_index[scan_indices_by_rank[rank]] = static_cast<std::uint32_t>(rank);
  }
  std::size_t input_point_count = 0U;
  for (const ProbabilisticSurfelMapScan & scan : scans) {
    input_point_count += scan.world_points.size();
  }
  std::vector<KeyedPoint> points;
  points.reserve(input_point_count);
  for (std::size_t scan_index = 0; scan_index < scans.size(); ++scan_index) {
    const ProbabilisticSurfelMapScan & scan = scans[scan_index];
    result.stats.input_points += scan.world_points.size();
    if (!scan.sensor_origin.allFinite() ||
      !std::isfinite(scan.pose_translation_variance_m2) ||
      scan.pose_translation_variance_m2 < 0.0 ||
      !std::isfinite(scan.pose_rotation_variance_rad2) ||
      scan.pose_rotation_variance_rad2 < 0.0)
    {
      continue;
    }
    for (const Eigen::Vector3d & point : scan.world_points) {
      if (!point.allFinite()) {continue;}
      ++result.stats.finite_points;
      KeyedPoint keyed;
      keyed.key = probabilistic_surfel_map_detail::voxelKey(point, config.voxel_size_m);
      keyed.position = point;
      keyed.scan_id = scan.scan_id;
      keyed.scan_index = static_cast<std::uint32_t>(scan_index);
      points.push_back(keyed);
    }
  }
  std::sort(
    points.begin(), points.end(), probabilistic_surfel_map_detail::keyedPointLess);

  // A flat sorted array avoids one allocator object and one tree node per
  // occupied fine voxel. This is the long-sequence memory contract.
  result.baseline_centroids.reserve(points.size() / 2U + 1U);
  result.fused_points.reserve(points.size() / 2U + 1U);
  std::vector<VoxelKey> output_keys;
  output_keys.reserve(points.size() / 2U + 1U);
  std::vector<bool> persistence_keep;
  persistence_keep.reserve(points.size() / 2U + 1U);
  std::vector<VoxelObservationSpan> observation_spans;
  if (config.build_visibility_filtered_map) {
    observation_spans.reserve(points.size() / 2U + 1U);
  }
  for (std::size_t first = 0U; first < points.size(); ) {
    std::size_t last = first + 1U;
    while (last < points.size() && points[last].key == points[first].key) {++last;}
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (std::size_t i = first; i < last; ++i) {
      centroid += points[i].position;
    }
    centroid /= static_cast<double>(last - first);
    const std::size_t output_index = result.baseline_centroids.size();
    result.baseline_centroids.push_back(centroid);
    result.fused_points.push_back(centroid);
    output_keys.push_back(points[first].key);
    for (std::size_t i = first; i < last; ++i) {
      points[i].output_index = static_cast<std::uint32_t>(output_index);
    }
    if (config.build_persistence_filtered_map || config.build_visibility_filtered_map) {
      bool persistence_all_far_range = config.persistence_max_filter_range_m > 0.0;
      bool visibility_all_far_range = config.visibility_max_range_m > 0.0;
      std::size_t distinct_scans = 0U;
      std::uint64_t first_scan = 0U;
      std::uint64_t last_scan = 0U;
      std::uint32_t first_scan_index = points[first].scan_index;
      std::uint32_t last_scan_index = points[first].scan_index;
      for (std::size_t i = first; i < last; ++i) {
        const ProbabilisticSurfelMapScan & scan = scans[points[i].scan_index];
        const double range = (points[i].position - scan.sensor_origin).norm();
        if (!(range > config.persistence_max_filter_range_m)) {
          persistence_all_far_range = false;
        }
        if (!(range > config.visibility_max_range_m)) {
          visibility_all_far_range = false;
        }
        if (i == first || points[i].scan_id != points[i - 1U].scan_id) {
          if (distinct_scans == 0U) {
            first_scan = points[i].scan_id;
            first_scan_index = points[i].scan_index;
          }
          last_scan = points[i].scan_id;
          last_scan_index = points[i].scan_index;
          ++distinct_scans;
        }
      }
      if (config.build_persistence_filtered_map) {
        const bool persistent = distinct_scans >= config.persistence_min_distinct_scans &&
          last_scan - first_scan >= config.persistence_min_scan_span;
        persistence_keep.push_back(persistence_all_far_range || persistent);
        if (persistence_all_far_range) {
          ++result.stats.persistence_far_range_keep_voxels;
        } else {
          ++result.stats.persistence_candidate_voxels;
          if (persistent) {
            ++result.stats.persistence_kept_voxels;
          } else {
            ++result.stats.persistence_removed_voxels;
          }
        }
      }
      if (config.build_visibility_filtered_map) {
        VoxelObservationSpan span;
        span.first_scan_rank = scan_rank_by_index[first_scan_index];
        span.last_scan_rank = scan_rank_by_index[last_scan_index];
        span.first_scan_index = first_scan_index;
        span.last_scan_index = last_scan_index;
        span.distinct_scans = static_cast<std::uint32_t>(distinct_scans);
        span.all_far_range = visibility_all_far_range;
        observation_spans.push_back(span);
      }
    }
    first = last;
  }
  result.stats.occupied_voxels = result.baseline_centroids.size();
  std::vector<Eigen::Vector3f> surface_projection_points;
  if (config.build_surface_consolidated_map) {
    // Keep the support-stage projection buffer in the same float precision as
    // the eventual PointXYZ artifact. Deferring the Vector3d output allocation
    // avoids carrying a third full-size double map through all support phases.
    surface_projection_points.reserve(result.baseline_centroids.size());
    for (const Eigen::Vector3d & centroid : result.baseline_centroids) {
      surface_projection_points.push_back(centroid.cast<float>());
    }
  }

  double raw_normal_rms_sum = 0.0;
  double fused_normal_sigma_sum = 0.0;
  std::vector<bool> ever_fused(result.baseline_centroids.size(), false);
  const std::uint32_t no_support_cell = std::numeric_limits<std::uint32_t>::max();
  std::vector<std::uint32_t> primary_support_cell_by_output;
  std::vector<SupportSurfelCell> primary_support_cells;
  std::vector<bool> connected_extended_fallback;
  if (config.build_connected_surface_map) {
    primary_support_cell_by_output.assign(result.baseline_centroids.size(), no_support_cell);
    connected_extended_fallback.assign(result.baseline_centroids.size(), false);
  }
  const auto apply_support_scale = [
    &](const double support_voxel_size_m, const bool only_unfused_outputs,
    const bool collect_connected_support) {
      std::vector<double> best_normal_rms(
        result.baseline_centroids.size(), std::numeric_limits<double>::infinity());
      std::vector<Eigen::Vector3d> blended_projection_sums(
        result.baseline_centroids.size(), Eigen::Vector3d::Zero());
      std::vector<std::uint8_t> blended_projection_counts(
        result.baseline_centroids.size(), 0U);
      std::vector<bool> scale_fused(result.baseline_centroids.size(), false);
      for (int phase = 0; phase < config.support_grid_phases; ++phase) {
        std::vector<SupportOutputRef> support_outputs;
        support_outputs.reserve(result.baseline_centroids.size());
        for (std::size_t output_index = 0; output_index < result.baseline_centroids.size();
          ++output_index)
        {
          support_outputs.push_back(SupportOutputRef{
            probabilistic_surfel_map_detail::phasedSupportVoxelKey(
              result.fused_points[output_index],
              support_voxel_size_m, phase), output_index});
        }
        for (KeyedPoint & point : points) {
          point.key = probabilistic_surfel_map_detail::phasedSupportVoxelKey(
          result.fused_points[point.output_index],
          support_voxel_size_m, phase);
        }
        std::sort(points.begin(), points.end(), probabilistic_surfel_map_detail::keyedPointLess);
        std::sort(
        support_outputs.begin(), support_outputs.end(),
        probabilistic_surfel_map_detail::supportOutputRefLess);

        std::size_t output_cursor = 0U;
        for (std::size_t first = 0U; first < points.size(); ) {
          std::size_t last = first + 1U;
          while (last < points.size() && points[last].key == points[first].key) {++last;}
          ++result.stats.occupied_support_voxels;
          std::vector<SurfelObservation> observations;
          observations.reserve(last - first);
          for (std::size_t i = first; i < last; ++i) {
            const ProbabilisticSurfelMapScan & scan = scans[points[i].scan_index];
            SurfelObservation observation;
            observation.position = points[i].position;
            observation.sensor_origin = scan.sensor_origin;
            observation.scan_id = points[i].scan_id;
            observation.pose_translation_variance_m2 = scan.pose_translation_variance_m2;
            observation.pose_rotation_variance_rad2 = scan.pose_rotation_variance_rad2;
            observations.push_back(observation);
          }
          const ProbabilisticSurfel surfel = fuseProbabilisticSurfel(observations, config.fusion);
          while (output_cursor < support_outputs.size() &&
            support_outputs[output_cursor].key < points[first].key)
          {
            ++output_cursor;
          }
          std::size_t output_last = output_cursor;
          while (output_last < support_outputs.size() &&
            support_outputs[output_last].key == points[first].key)
          {
            ++output_last;
          }
          if (surfel.valid) {
            ++result.stats.valid_support_surfels;
            raw_normal_rms_sum += surfel.raw_normal_rms_m;
            fused_normal_sigma_sum += surfel.fused_normal_sigma_m;
            std::uint32_t connected_support_cell = no_support_cell;
            if (collect_connected_support) {
              connected_support_cell = static_cast<std::uint32_t>(primary_support_cells.size());
              primary_support_cells.push_back(SupportSurfelCell{points[first].key, surfel});
            }
            for (std::size_t i = output_cursor; i < output_last; ++i) {
              const std::size_t output_index = support_outputs[i].output_index;
              if (only_unfused_outputs && ever_fused[output_index]) {continue;}
              if (config.support_phases_fallback_only && phase > 0 &&
                scale_fused[output_index])
              {
                continue;
              }
              if (collect_connected_support) {
                primary_support_cell_by_output[output_index] = connected_support_cell;
              }
              const Eigen::Vector3d & centroid = result.fused_points[output_index];
              const Eigen::Vector3d projected = centroid + surfel.normal *
                surfel.normal.dot(surfel.mean - centroid);
              if (config.blend_support_phases) {
                blended_projection_sums[output_index] += projected;
                ++blended_projection_counts[output_index];
                best_normal_rms[output_index] = std::min(
                best_normal_rms[output_index], surfel.raw_normal_rms_m);
              } else if (surfel.raw_normal_rms_m < best_normal_rms[output_index]) {
                if (config.build_surface_consolidated_map) {
                  surface_projection_points[output_index] = projected.cast<float>();
                }
                result.fused_points[output_index] =
                  probabilistic_surfel_map_detail::clampToVoxel(
                projected, output_keys[output_index], config.voxel_size_m);
                best_normal_rms[output_index] = surfel.raw_normal_rms_m;
                if (config.support_phases_fallback_only && phase > 0 &&
                  !scale_fused[output_index])
                {
                  ++result.stats.shifted_phase_fused_voxels;
                }
                scale_fused[output_index] = true;
              }
            }
          }
          output_cursor = output_last;
          first = last;
        }
      }
      if (config.blend_support_phases) {
        for (std::size_t i = 0; i < result.fused_points.size(); ++i) {
          if (blended_projection_counts[i] == 0U) {continue;}
          const Eigen::Vector3d blended = blended_projection_sums[i] /
            static_cast<double>(blended_projection_counts[i]);
          if (config.build_surface_consolidated_map) {
            surface_projection_points[i] = blended.cast<float>();
          }
          result.fused_points[i] = probabilistic_surfel_map_detail::clampToVoxel(
          blended, output_keys[i], config.voxel_size_m);
        }
      }
      for (std::size_t i = 0; i < best_normal_rms.size(); ++i) {
        ever_fused[i] = ever_fused[i] || std::isfinite(best_normal_rms[i]);
      }
    };
  apply_support_scale(
    config.surfel_support_voxel_size_m, false, config.build_connected_surface_map);
  const std::vector<bool> primary_fused = ever_fused;
  if (config.build_connected_surface_map) {
    result.connected_surface_points = result.fused_points;
    result.stats.connected_surface_support_cells = primary_support_cells.size();
    const double minimum_normal_dot = std::cos(
      config.connected_surface_max_normal_angle_deg * std::acos(-1.0) / 180.0);
    std::vector<ProbabilisticSurfel> connected_planes(primary_support_cells.size());
    for (std::size_t i = 0; i < primary_support_cells.size(); ++i) {
      connected_planes[i] =
        probabilistic_surfel_map_detail::mergeConnectedSupportNeighborhood(
        primary_support_cells, i, minimum_normal_dot,
        config.connected_surface_max_plane_distance_m,
        config.connected_surface_min_support_cells);
      if (connected_planes[i].valid) {++result.stats.connected_surface_merged_cells;}
    }
    for (std::size_t i = 0; i < result.connected_surface_points.size(); ++i) {
      const std::uint32_t support_index = primary_support_cell_by_output[i];
      if (support_index == no_support_cell || !connected_planes[support_index].valid) {continue;}
      const ProbabilisticSurfel & plane = connected_planes[support_index];
      const Eigen::Vector3d & centroid = result.baseline_centroids[i];
      const Eigen::Vector3d projected = centroid + plane.normal *
        plane.normal.dot(plane.mean - centroid);
      result.connected_surface_points[i] =
        probabilistic_surfel_map_detail::clampToVoxel(
        projected, output_keys[i], config.voxel_size_m);
      ++result.stats.connected_surface_projected_voxels;
    }
    if (config.connected_surface_extend_fallback) {
      for (std::size_t i = 0; i < result.connected_surface_points.size(); ++i) {
        if (primary_support_cell_by_output[i] != no_support_cell) {continue;}
        const Eigen::Vector3d & centroid = result.baseline_centroids[i];
        const VoxelKey support_key = probabilistic_surfel_map_detail::phasedSupportVoxelKey(
          centroid, config.surfel_support_voxel_size_m, 0);
        std::vector<std::pair<const ProbabilisticSurfel *, double>> extension_candidates;
        for (std::int64_t dx = -1; dx <= 1; ++dx) {
          for (std::int64_t dy = -1; dy <= 1; ++dy) {
            for (std::int64_t dz = -1; dz <= 1; ++dz) {
              const VoxelKey neighbor_key = probabilistic_surfel_map_detail::offsetVoxelKey(
                support_key, dx, dy, dz);
              const auto found = std::lower_bound(
                primary_support_cells.begin(), primary_support_cells.end(),
                SupportSurfelCell{neighbor_key, {}},
                probabilistic_surfel_map_detail::supportSurfelCellLess);
              if (found == primary_support_cells.end() || found->key != neighbor_key) {continue;}
              const double distance = std::abs(
                found->surfel.normal.dot(centroid - found->surfel.mean));
              if (distance <= config.connected_surface_max_extension_distance_m) {
                extension_candidates.emplace_back(&found->surfel, distance);
              }
            }
          }
        }
        const ProbabilisticSurfel * best_surfel = nullptr;
        std::size_t best_support_count = 0U;
        double best_distance = std::numeric_limits<double>::infinity();
        for (const auto & candidate : extension_candidates) {
          std::size_t support_count = 0U;
          for (const auto & other : extension_candidates) {
            if (probabilistic_surfel_map_detail::compatibleSupportSurfels(
                *candidate.first, *other.first, minimum_normal_dot,
                config.connected_surface_max_plane_distance_m))
            {
              ++support_count;
            }
          }
          if (support_count < config.connected_surface_min_extension_support_cells) {continue;}
          if (support_count > best_support_count ||
            (support_count == best_support_count &&
            (candidate.second < best_distance ||
            (candidate.second == best_distance && best_surfel != nullptr &&
            candidate.first->raw_normal_rms_m < best_surfel->raw_normal_rms_m))))
          {
            best_surfel = candidate.first;
            best_support_count = support_count;
            best_distance = candidate.second;
          }
        }
        if (best_surfel == nullptr) {continue;}
        const Eigen::Vector3d projected = centroid + best_surfel->normal *
          best_surfel->normal.dot(best_surfel->mean - centroid);
        result.connected_surface_points[i] =
          probabilistic_surfel_map_detail::clampToVoxel(
          projected, output_keys[i], config.voxel_size_m);
        connected_extended_fallback[i] = true;
        ++result.stats.connected_surface_projected_voxels;
        ++result.stats.connected_surface_extended_fallback_voxels;
      }
    }
  }
  if (config.secondary_support_voxel_size_m > 0.0) {
    // A smaller support is noisier. It may fill a coarse-scale fallback, but
    // must not replace a point already supported by the stronger coarse plane.
    apply_support_scale(config.secondary_support_voxel_size_m, true, false);
  }
  if (config.tertiary_support_voxel_size_m > 0.0) {
    // The tertiary scale is also fallback-only. It is intended for sparse
    // planar cells that neither the primary nor secondary support could fit.
    apply_support_scale(config.tertiary_support_voxel_size_m, true, false);
  }
  // Support fitting is complete. Release the largest keyed observation buffer
  // before materializing or re-voxelizing any optional output map.
  std::vector<KeyedPoint>().swap(points);
  if (config.build_connected_surface_map) {
    // Preserve the hierarchical fallback contract: outputs without primary
    // connected support inherit the secondary-scale projection byte-for-byte.
    for (std::size_t i = 0; i < result.connected_surface_points.size(); ++i) {
      if (!primary_fused[i] && !connected_extended_fallback[i]) {
        result.connected_surface_points[i] = result.fused_points[i];
      }
    }
  }
  if (config.build_surface_consolidated_map) {
    // Start from the occupancy-preserving fused map and release the fine-voxel
    // clamp only for plane corrections large enough to be thickness outliers.
    // A zero threshold preserves the original all-supported consolidation.
    result.surface_consolidated_points.resize(result.fused_points.size());
    for (std::size_t i = 0; i < result.surface_consolidated_points.size(); ++i) {
      const Eigen::Vector3d projected = surface_projection_points[i].cast<double>();
      const double projection_distance =
        (projected - result.baseline_centroids[i]).norm();
      if (ever_fused[i] &&
        projection_distance >= config.surface_consolidation_min_projection_distance_m)
      {
        result.surface_consolidated_points[i] = projected;
        ++result.stats.surface_consolidation_selected_points;
      } else {
        result.surface_consolidated_points[i] = result.fused_points[i];
      }
    }
    std::vector<Eigen::Vector3f>().swap(surface_projection_points);
    result.stats.surface_consolidation_input_points =
      result.surface_consolidated_points.size();
    result.surface_consolidated_points =
      probabilistic_surfel_map_detail::deterministicVoxelCentroids(
      result.surface_consolidated_points, config.voxel_size_m);
    result.stats.surface_consolidation_output_points =
      result.surface_consolidated_points.size();
    result.stats.surface_consolidation_merged_points =
      result.stats.surface_consolidation_input_points -
      result.stats.surface_consolidation_output_points;
  }
  result.stats.fused_surfel_voxels = static_cast<std::size_t>(std::count_if(
      ever_fused.begin(), ever_fused.end(),
      [](const bool value) {return value;}));
  result.stats.fallback_centroid_voxels =
    result.stats.occupied_voxels - result.stats.fused_surfel_voxels;
  if (config.build_support_partition_maps) {
    result.supported_partition_points.reserve(result.stats.fused_surfel_voxels);
    result.fallback_partition_points.reserve(result.stats.fallback_centroid_voxels);
    for (std::size_t i = 0; i < result.fused_points.size(); ++i) {
      if (ever_fused[i]) {
        result.supported_partition_points.push_back(result.fused_points[i]);
      } else {
        result.fallback_partition_points.push_back(result.fused_points[i]);
      }
    }
  }
  if (result.stats.valid_support_surfels > 0U) {
    const double count = static_cast<double>(result.stats.valid_support_surfels);
    result.stats.mean_raw_normal_rms_m = raw_normal_rms_sum / count;
    result.stats.mean_fused_normal_sigma_m = fused_normal_sigma_sum / count;
  }
  if (config.build_persistence_filtered_map) {
    result.persistence_filtered_points.reserve(
      result.stats.persistence_kept_voxels +
      result.stats.persistence_far_range_keep_voxels);
    for (std::size_t i = 0; i < result.fused_points.size(); ++i) {
      if (persistence_keep[i]) {
        result.persistence_filtered_points.push_back(result.fused_points[i]);
      }
    }
  }
  if (config.build_visibility_filtered_map) {
    std::vector<bool> visibility_candidate(result.fused_points.size(), false);
    std::vector<bool> visibility_tested(result.fused_points.size(), false);
    std::vector<std::uint8_t> free_space_votes(result.fused_points.size(), 0U);
    result.visibility_filtered_points.reserve(result.fused_points.size());
    for (std::size_t i = 0; i < observation_spans.size(); ++i) {
      const VoxelObservationSpan & span = observation_spans[i];
      const std::uint64_t rank_span =
        static_cast<std::uint64_t>(span.last_scan_rank) - span.first_scan_rank;
      visibility_candidate[i] = !span.all_far_range &&
        span.distinct_scans <= config.visibility_max_distinct_scans &&
        rank_span <= config.visibility_max_scan_span;
      if (visibility_candidate[i]) {++result.stats.visibility_candidate_voxels;}
    }

    const auto apply_visibility_queries = [&](const std::size_t scan_offset, const bool before) {
        const std::size_t scan_count = scans.size();
        std::vector<std::size_t> query_counts(scan_count, 0U);
        const auto target_scan_rank = [&](const std::size_t output_index) {
            const VoxelObservationSpan & span = observation_spans[output_index];
            if (before) {
              if (span.first_scan_rank < scan_offset) {return scan_count;}
              return static_cast<std::size_t>(span.first_scan_rank) - scan_offset;
            }
            const std::size_t target =
              static_cast<std::size_t>(span.last_scan_rank) + scan_offset;
            return target < scan_count ? target : scan_count;
          };
        const auto query_is_eligible = [&](const std::size_t output_index,
          const std::size_t target_rank) {
            if (!visibility_candidate[output_index] || target_rank >= scan_count) {return false;}
            const VoxelObservationSpan & span = observation_spans[output_index];
            const std::uint32_t reference_index =
              before ? span.first_scan_index : span.last_scan_index;
            const std::uint32_t target_index = scan_indices_by_rank[target_rank];
            const double origin_displacement =
              (scans[target_index].sensor_origin - scans[reference_index].sensor_origin).norm();
            const double candidate_range =
              (result.fused_points[output_index] - scans[target_index].sensor_origin).norm();
            return std::isfinite(origin_displacement) &&
                   origin_displacement <= config.visibility_max_origin_displacement_m &&
                   candidate_range > 0.0 && candidate_range <= config.visibility_max_range_m;
          };

        for (std::size_t i = 0; i < result.fused_points.size(); ++i) {
          const std::size_t target_rank = target_scan_rank(i);
          if (query_is_eligible(i, target_rank)) {++query_counts[target_rank];}
        }
        std::vector<std::size_t> query_offsets(scan_count + 1U, 0U);
        for (std::size_t rank = 0; rank < scan_count; ++rank) {
          query_offsets[rank + 1U] = query_offsets[rank] + query_counts[rank];
        }
        std::vector<std::uint32_t> query_outputs(query_offsets.back(), 0U);
        std::vector<std::size_t> write_offsets = query_offsets;
        for (std::size_t i = 0; i < result.fused_points.size(); ++i) {
          const std::size_t target_rank = target_scan_rank(i);
          if (query_is_eligible(i, target_rank)) {
            query_outputs[write_offsets[target_rank]++] = static_cast<std::uint32_t>(i);
          }
        }

        for (std::size_t rank = 0; rank < scan_count; ++rank) {
          if (query_offsets[rank] == query_offsets[rank + 1U]) {continue;}
          const ProbabilisticSurfelMapScan & scan = scans[scan_indices_by_rank[rank]];
          std::vector<probabilistic_surfel_map_detail::AngularReturn> returns;
          returns.reserve(scan.world_points.size());
          for (const Eigen::Vector3d & point : scan.world_points) {
            const Eigen::Vector3d ray = point - scan.sensor_origin;
            const double range = ray.norm();
            if (!ray.allFinite() || !(range > 0.0) || !std::isfinite(range)) {continue;}
            returns.push_back(probabilistic_surfel_map_detail::AngularReturn{
              probabilistic_surfel_map_detail::angularBinKey(
                  ray, config.visibility_angular_resolution_rad), range});
          }
          std::sort(
            returns.begin(), returns.end(),
            probabilistic_surfel_map_detail::angularReturnLess);
          for (std::size_t query = query_offsets[rank];
            query < query_offsets[rank + 1U]; ++query)
          {
            const std::size_t output_index = query_outputs[query];
            const Eigen::Vector3d ray = result.fused_points[output_index] - scan.sensor_origin;
            const double candidate_range = ray.norm();
            const std::int64_t key = probabilistic_surfel_map_detail::angularBinKey(
              ray, config.visibility_angular_resolution_rad);
            const auto found = std::lower_bound(
              returns.begin(), returns.end(), key,
              [](const probabilistic_surfel_map_detail::AngularReturn & value,
              const std::int64_t expected_key) {return value.key < expected_key;});
            if (found == returns.end() || found->key != key) {continue;}
            visibility_tested[output_index] = true;
            if (found->range_m > candidate_range + config.visibility_free_space_margin_m &&
              free_space_votes[output_index] < std::numeric_limits<std::uint8_t>::max())
            {
              ++free_space_votes[output_index];
            }
          }
        }
      };

    apply_visibility_queries(config.visibility_near_scan_offset, true);
    if (config.visibility_far_scan_offset != config.visibility_near_scan_offset) {
      apply_visibility_queries(config.visibility_far_scan_offset, true);
    }
    apply_visibility_queries(config.visibility_near_scan_offset, false);
    if (config.visibility_far_scan_offset != config.visibility_near_scan_offset) {
      apply_visibility_queries(config.visibility_far_scan_offset, false);
    }

    for (std::size_t i = 0; i < result.fused_points.size(); ++i) {
      if (visibility_tested[i]) {++result.stats.visibility_tested_voxels;}
      if (free_space_votes[i] > 0U) {++result.stats.visibility_contradicted_voxels;}
      const bool remove = visibility_candidate[i] &&
        free_space_votes[i] >= config.visibility_min_free_space_votes;
      if (remove) {
        ++result.stats.visibility_removed_voxels;
      } else {
        ++result.stats.visibility_kept_voxels;
        result.visibility_filtered_points.push_back(result.fused_points[i]);
      }
    }
  }
  return result;
}

}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__PROBABILISTIC_SURFEL_MAP_HPP_
