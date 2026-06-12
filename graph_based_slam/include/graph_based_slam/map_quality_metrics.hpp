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

#ifndef GRAPH_BASED_SLAM__MAP_QUALITY_METRICS_HPP_
#define GRAPH_BASED_SLAM__MAP_QUALITY_METRICS_HPP_

// Map-quality metrics for the v0.7 quality gates (docs/roadmap/v0.7.md,
// Phase 1): Mean Map Entropy, plane-thickness statistics over the patches
// found by adaptive_voxel_plane_extractor.hpp, and density statistics.
// Every metric carries its support (eligible/valid fractions, planar
// coverage) so a better score can never silently mean "ignored hard
// geometry", and scenes below a planarity-coverage floor report an
// explicit not-meaningful state instead of a number. Deterministic by
// construction: voxel-hash neighborhoods iterated in sorted key order,
// fixed-order double accumulation, no wall clock, no randomness — the
// release gate requires byte-identical reports across runs.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>  // NOLINT(build/include_order)
#include <Eigen/Eigenvalues>  // NOLINT(build/include_order)

#include "graph_based_slam/adaptive_voxel_plane_extractor.hpp"

namespace graphslam
{
namespace map_quality
{

struct MapQualityConfig
{
  // Mean Map Entropy: per-point Gaussian entropy of the neighborhood
  // within mme_radius; points with fewer than mme_min_neighbors
  // neighbors (self included) are excluded and reported as such.
  double mme_radius {0.5};
  int mme_min_neighbors {8};
  // Optional deterministic voxel-centroid downsample applied before any
  // metric (0.0 = off). Keeps the metric cost bounded on dense maps.
  double downsample_voxel_size {0.0};
  // Plane metrics share the Phase 2 extractor, but with the FROZEN
  // metric extraction profile below (Phase 1 calibration on the five
  // gate substrates, docs/research/map-quality-baseline.md). Changing
  // these values invalidates every recorded baseline; Phase 2 tuning
  // must not touch them (the optimizer may not adjust its own judge).
  plane_extraction::PlaneExtractionConfig plane_config;
  // Below this planar coverage the plane metrics are reported as
  // not meaningful (expected on vegetation-heavy outdoor maps).
  double min_meaningful_planar_coverage {0.05};

  MapQualityConfig()
  {
    plane_config.max_plane_thickness = 0.15;
    plane_config.min_planarity_ratio = 4.0;
    plane_config.min_points_per_plane = 10;
    plane_config.max_octree_depth = 4;
  }
};

struct MapQualityReport
{
  std::int64_t input_points {0};
  std::int64_t evaluated_points {0};  // after optional downsample
  // Mean Map Entropy (nats); lower = crisper.
  double mean_map_entropy {0.0};
  std::int64_t mme_valid_points {0};
  double mme_valid_fraction {0.0};
  // Plane metrics (support-reported).
  int plane_patch_count {0};
  double plane_thickness_rms_mean_m {0.0};  // point-count weighted
  double plane_thickness_rms_p95_m {0.0};
  double planar_coverage {0.0};
  bool plane_metrics_meaningful {false};
  // Density over occupied root voxels.
  std::int64_t occupied_root_voxels {0};
  double density_mean_points_per_voxel {0.0};
  double density_stddev_points_per_voxel {0.0};
};

namespace detail
{

struct VoxelKey
{
  std::int64_t x;
  std::int64_t y;
  std::int64_t z;

  bool operator<(const VoxelKey & other) const
  {
    if (x != other.x) {return x < other.x;}
    if (y != other.y) {return y < other.y;}
    return z < other.z;
  }
};

inline VoxelKey keyOf(const Eigen::Vector3d & p, double voxel_size)
{
  VoxelKey key;
  key.x = static_cast<std::int64_t>(std::floor(p.x() / voxel_size));
  key.y = static_cast<std::int64_t>(std::floor(p.y() / voxel_size));
  key.z = static_cast<std::int64_t>(std::floor(p.z() / voxel_size));
  return key;
}

// std::map keeps the bucket iteration order sorted; point indices are
// appended in ascending input order, so every downstream accumulation
// has a fixed order.
inline std::map<VoxelKey, std::vector<int>> bucketize(
  const std::vector<Eigen::Vector3d> & points, double voxel_size)
{
  std::map<VoxelKey, std::vector<int>> buckets;
  for (int i = 0; i < static_cast<int>(points.size()); ++i) {
    buckets[keyOf(points[i], voxel_size)].push_back(i);
  }
  return buckets;
}

}  // namespace detail

// Deterministic voxel-centroid downsample (sorted voxel order, ascending
// index accumulation). Replaces pcl::VoxelGrid here so the metric input
// is a pure function of the point list.
inline std::vector<Eigen::Vector3d> downsampleByVoxelCentroid(
  const std::vector<Eigen::Vector3d> & points, double voxel_size)
{
  if (voxel_size <= 0.0) {return points;}
  const auto buckets = detail::bucketize(points, voxel_size);
  std::vector<Eigen::Vector3d> result;
  result.reserve(buckets.size());
  for (auto it = buckets.begin(); it != buckets.end(); ++it) {
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (size_t j = 0; j < it->second.size(); ++j) {
      sum += points[it->second[j]];
    }
    result.push_back(sum / static_cast<double>(it->second.size()));
  }
  return result;
}

struct MeanMapEntropyResult
{
  double mean_entropy {0.0};
  std::int64_t valid_points {0};
  std::int64_t total_points {0};
};

// Mean Map Entropy: h(p) = 0.5 * ln((2*pi*e)^3 * det(Sigma)) over the
// neighborhood of p (self included); the mean is taken over points with
// at least min_neighbors neighbors and a positive-definite covariance.
inline MeanMapEntropyResult computeMeanMapEntropy(
  const std::vector<Eigen::Vector3d> & points, double radius, int min_neighbors)
{
  MeanMapEntropyResult result;
  result.total_points = static_cast<std::int64_t>(points.size());
  if (points.empty() || radius <= 0.0) {return result;}

  const auto buckets = detail::bucketize(points, radius);
  const double radius_sq = radius * radius;
  // ln((2*pi*e)^3) = 3 * ln(2*pi*e)
  const double log_two_pi_e_cubed = 3.0 * std::log(2.0 * M_PI * M_E);

  double entropy_sum = 0.0;
  std::vector<int> neighbors;
  for (int i = 0; i < static_cast<int>(points.size()); ++i) {
    neighbors.clear();
    const detail::VoxelKey center = detail::keyOf(points[i], radius);
    for (std::int64_t dx = -1; dx <= 1; ++dx) {
      for (std::int64_t dy = -1; dy <= 1; ++dy) {
        for (std::int64_t dz = -1; dz <= 1; ++dz) {
          detail::VoxelKey key;
          key.x = center.x + dx;
          key.y = center.y + dy;
          key.z = center.z + dz;
          const auto it = buckets.find(key);
          if (it == buckets.end()) {continue;}
          for (size_t j = 0; j < it->second.size(); ++j) {
            const int idx = it->second[j];
            if ((points[idx] - points[i]).squaredNorm() <= radius_sq) {
              neighbors.push_back(idx);
            }
          }
        }
      }
    }
    if (static_cast<int>(neighbors.size()) < min_neighbors) {continue;}
    // Ascending index order keeps the float accumulation fixed.
    std::sort(neighbors.begin(), neighbors.end());
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (size_t j = 0; j < neighbors.size(); ++j) {
      mean += points[neighbors[j]];
    }
    mean /= static_cast<double>(neighbors.size());
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (size_t j = 0; j < neighbors.size(); ++j) {
      const Eigen::Vector3d d = points[neighbors[j]] - mean;
      covariance += d * d.transpose();
    }
    covariance /= static_cast<double>(neighbors.size());
    const double det = covariance.determinant();
    if (!(det > 0.0)) {continue;}
    entropy_sum += 0.5 * (log_two_pi_e_cubed + std::log(det));
    ++result.valid_points;
  }
  if (result.valid_points > 0) {
    result.mean_entropy = entropy_sum / static_cast<double>(result.valid_points);
  }
  return result;
}

inline MapQualityReport computeMapQuality(
  const std::vector<Eigen::Vector3d> & input_points, const MapQualityConfig & config)
{
  MapQualityReport report;
  report.input_points = static_cast<std::int64_t>(input_points.size());

  const std::vector<Eigen::Vector3d> points =
    downsampleByVoxelCentroid(input_points, config.downsample_voxel_size);
  report.evaluated_points = static_cast<std::int64_t>(points.size());
  if (points.empty()) {return report;}

  const MeanMapEntropyResult mme =
    computeMeanMapEntropy(points, config.mme_radius, config.mme_min_neighbors);
  report.mean_map_entropy = mme.mean_entropy;
  report.mme_valid_points = mme.valid_points;
  report.mme_valid_fraction =
    static_cast<double>(mme.valid_points) / static_cast<double>(points.size());

  const plane_extraction::PlaneExtractionResult planes =
    plane_extraction::extractPlanarPatches(points, config.plane_config);
  report.plane_patch_count = static_cast<int>(planes.patches.size());
  report.planar_coverage = planes.planar_coverage;
  if (!planes.patches.empty()) {
    double weighted_sum = 0.0;
    double weight = 0.0;
    std::vector<double> thicknesses;
    thicknesses.reserve(planes.patches.size());
    for (size_t i = 0; i < planes.patches.size(); ++i) {
      const auto & patch = planes.patches[i];
      weighted_sum += patch.thickness_rms * static_cast<double>(patch.point_count);
      weight += static_cast<double>(patch.point_count);
      thicknesses.push_back(patch.thickness_rms);
    }
    report.plane_thickness_rms_mean_m = weighted_sum / weight;
    std::sort(thicknesses.begin(), thicknesses.end());
    const size_t p95_index = std::min(
      thicknesses.size() - 1,
      static_cast<size_t>(std::ceil(0.95 * static_cast<double>(thicknesses.size())) - 1.0));
    report.plane_thickness_rms_p95_m = thicknesses[p95_index];
  }
  report.plane_metrics_meaningful =
    report.planar_coverage >= config.min_meaningful_planar_coverage &&
    report.plane_patch_count > 0;

  // Density over occupied root voxels (same root size as the extractor).
  const auto buckets = detail::bucketize(points, config.plane_config.root_voxel_size);
  report.occupied_root_voxels = static_cast<std::int64_t>(buckets.size());
  double count_sum = 0.0;
  for (auto it = buckets.begin(); it != buckets.end(); ++it) {
    count_sum += static_cast<double>(it->second.size());
  }
  const double count_mean = count_sum / static_cast<double>(buckets.size());
  double variance_sum = 0.0;
  for (auto it = buckets.begin(); it != buckets.end(); ++it) {
    const double d = static_cast<double>(it->second.size()) - count_mean;
    variance_sum += d * d;
  }
  report.density_mean_points_per_voxel = count_mean;
  report.density_stddev_points_per_voxel =
    std::sqrt(variance_sum / static_cast<double>(buckets.size()));
  return report;
}

namespace detail
{

inline std::string formatDouble(double value)
{
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.9f", value);
  return std::string(buffer);
}

}  // namespace detail

// YAML content for map_quality_report.yaml. Fixed key order and fixed
// formatting: the release gate compares these bytes across runs.
inline std::vector<std::string> reportYamlLines(
  const MapQualityReport & report, const MapQualityConfig & config)
{
  std::vector<std::string> lines;
  lines.push_back("map_quality_report:");
  lines.push_back("  input_points: " + std::to_string(report.input_points));
  lines.push_back("  evaluated_points: " + std::to_string(report.evaluated_points));
  lines.push_back(
    "  downsample_voxel_size_m: " + detail::formatDouble(config.downsample_voxel_size));
  lines.push_back("  mean_map_entropy:");
  lines.push_back("    value_nats: " + detail::formatDouble(report.mean_map_entropy));
  lines.push_back("    radius_m: " + detail::formatDouble(config.mme_radius));
  lines.push_back("    valid_points: " + std::to_string(report.mme_valid_points));
  lines.push_back("    valid_fraction: " + detail::formatDouble(report.mme_valid_fraction));
  lines.push_back("  plane_metrics:");
  lines.push_back(
    "    meaningful: " + std::string(report.plane_metrics_meaningful ? "true" : "false"));
  lines.push_back("    patch_count: " + std::to_string(report.plane_patch_count));
  lines.push_back(
    "    thickness_rms_mean_m: " + detail::formatDouble(report.plane_thickness_rms_mean_m));
  lines.push_back(
    "    thickness_rms_p95_m: " + detail::formatDouble(report.plane_thickness_rms_p95_m));
  lines.push_back("    planar_coverage: " + detail::formatDouble(report.planar_coverage));
  lines.push_back(
    "    min_meaningful_planar_coverage: " +
    detail::formatDouble(config.min_meaningful_planar_coverage));
  lines.push_back("  density:");
  lines.push_back("    occupied_root_voxels: " + std::to_string(report.occupied_root_voxels));
  lines.push_back(
    "    mean_points_per_voxel: " +
    detail::formatDouble(report.density_mean_points_per_voxel));
  lines.push_back(
    "    stddev_points_per_voxel: " +
    detail::formatDouble(report.density_stddev_points_per_voxel));
  return lines;
}

}  // namespace map_quality
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__MAP_QUALITY_METRICS_HPP_
