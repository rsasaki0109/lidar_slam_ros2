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

#ifndef GRAPH_BASED_SLAM__MAP_THICKNESS_ATTRIBUTION_HPP_
#define GRAPH_BASED_SLAM__MAP_THICKNESS_ATTRIBUTION_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Core>  // NOLINT(build/include_order)

#include "graph_based_slam/adaptive_voxel_plane_extractor.hpp"

namespace graphslam
{
namespace map_thickness
{

struct AttributedPoint
{
  Eigen::Vector3d position {Eigen::Vector3d::Zero()};
  std::int64_t scan_id {0};
  std::int64_t submap_id {0};
  std::int64_t revisit_id {0};
};

struct AttributionConfig
{
  plane_extraction::PlaneExtractionConfig plane_config;

  AttributionConfig()
  {
    plane_config.max_plane_thickness = 0.15;
    plane_config.min_planarity_ratio = 4.0;
    plane_config.min_points_per_plane = 10;
    plane_config.max_octree_depth = 4;
    plane_config.collect_point_indices = true;
  }
};

struct AttributionReport
{
  std::int64_t input_points {0};
  std::int64_t planar_points {0};
  int plane_patch_count {0};
  std::int64_t distinct_scans {0};
  std::int64_t distinct_submaps {0};
  std::int64_t distinct_revisits {0};
  double planar_coverage {0.0};
  double total_sse {0.0};
  double within_scan_sse {0.0};
  double between_scan_sse {0.0};
  double between_submap_sse {0.0};
  double between_revisit_sse {0.0};
  double closure_error {0.0};
  double total_rms_m {0.0};
  double within_scan_rms_m {0.0};
  double between_scan_rms_m {0.0};
  double between_submap_rms_m {0.0};
  double between_revisit_rms_m {0.0};
  double within_scan_fraction {0.0};
  double between_scan_fraction {0.0};
  double between_submap_fraction {0.0};
  double between_revisit_fraction {0.0};
  bool meaningful {false};
};

namespace detail
{

struct ScalarMoments
{
  std::int64_t count {0};
  double sum {0.0};

  void add(const double value)
  {
    ++count;
    sum += value;
  }

  void add(const ScalarMoments & other)
  {
    count += other.count;
    sum += other.sum;
  }

  double mean() const
  {
    return count > 0 ? sum / static_cast<double>(count) : 0.0;
  }
};

using ScanKey = std::tuple<std::int64_t, std::int64_t, std::int64_t>;
using SubmapKey = std::pair<std::int64_t, std::int64_t>;

inline std::string formatDouble(const double value)
{
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.9f", value);
  return std::string(buffer);
}

}  // namespace detail

inline AttributionReport computeAttribution(
  const std::vector<AttributedPoint> & attributed_points,
  const AttributionConfig & config = AttributionConfig())
{
  AttributionReport report;
  report.input_points = static_cast<std::int64_t>(attributed_points.size());
  if (attributed_points.empty()) {
    return report;
  }

  std::vector<Eigen::Vector3d> positions;
  positions.reserve(attributed_points.size());
  std::set<detail::ScanKey> all_scans;
  std::set<detail::SubmapKey> all_submaps;
  std::set<std::int64_t> all_revisits;
  for (const auto & point : attributed_points) {
    positions.push_back(point.position);
    all_scans.emplace(point.revisit_id, point.submap_id, point.scan_id);
    all_submaps.emplace(point.revisit_id, point.submap_id);
    all_revisits.insert(point.revisit_id);
  }
  report.distinct_scans = static_cast<std::int64_t>(all_scans.size());
  report.distinct_submaps = static_cast<std::int64_t>(all_submaps.size());
  report.distinct_revisits = static_cast<std::int64_t>(all_revisits.size());

  plane_extraction::PlaneExtractionConfig plane_config = config.plane_config;
  plane_config.collect_point_indices = true;
  const plane_extraction::PlaneExtractionResult extraction =
    plane_extraction::extractPlanarPatches(positions, plane_config);
  report.planar_points = extraction.planar_points;
  report.plane_patch_count = static_cast<int>(extraction.patches.size());
  report.planar_coverage = extraction.planar_coverage;

  for (std::size_t patch_index = 0; patch_index < extraction.patches.size(); ++patch_index) {
    const plane_extraction::PlanarPatch & patch = extraction.patches[patch_index];
    const std::vector<int> & indices = extraction.patch_point_indices[patch_index];
    std::map<detail::ScanKey, detail::ScalarMoments> scans;
    std::map<detail::SubmapKey, detail::ScalarMoments> submaps;
    std::map<std::int64_t, detail::ScalarMoments> revisits;
    std::vector<double> residuals;
    residuals.reserve(indices.size());
    detail::ScalarMoments total;

    for (const int point_index : indices) {
      const AttributedPoint & point = attributed_points[static_cast<std::size_t>(point_index)];
      const double residual = patch.normal.dot(point.position - patch.centroid);
      residuals.push_back(residual);
      const detail::ScanKey scan_key(point.revisit_id, point.submap_id, point.scan_id);
      const detail::SubmapKey submap_key(point.revisit_id, point.submap_id);
      scans[scan_key].add(residual);
      submaps[submap_key].add(residual);
      revisits[point.revisit_id].add(residual);
      total.add(residual);
    }

    const double total_mean = total.mean();
    for (const double residual : residuals) {
      const double delta = residual - total_mean;
      report.total_sse += delta * delta;
    }

    for (std::size_t i = 0; i < indices.size(); ++i) {
      const AttributedPoint & point = attributed_points[static_cast<std::size_t>(indices[i])];
      const detail::ScanKey scan_key(point.revisit_id, point.submap_id, point.scan_id);
      const double delta = residuals[i] - scans[scan_key].mean();
      report.within_scan_sse += delta * delta;
    }
    for (const auto & scan : scans) {
      const detail::SubmapKey submap_key(std::get<0>(scan.first), std::get<1>(scan.first));
      const double delta = scan.second.mean() - submaps[submap_key].mean();
      report.between_scan_sse += static_cast<double>(scan.second.count) * delta * delta;
    }
    for (const auto & submap : submaps) {
      const std::int64_t revisit_id = submap.first.first;
      const double delta = submap.second.mean() - revisits[revisit_id].mean();
      report.between_submap_sse += static_cast<double>(submap.second.count) * delta * delta;
    }
    for (const auto & revisit : revisits) {
      const double delta = revisit.second.mean() - total_mean;
      report.between_revisit_sse += static_cast<double>(revisit.second.count) * delta * delta;
    }
  }

  const double component_sum = report.within_scan_sse + report.between_scan_sse +
    report.between_submap_sse + report.between_revisit_sse;
  report.closure_error = std::abs(report.total_sse - component_sum);
  if (report.planar_points <= 0) {
    return report;
  }
  const double count = static_cast<double>(report.planar_points);
  report.total_rms_m = std::sqrt(std::max(0.0, report.total_sse / count));
  report.within_scan_rms_m = std::sqrt(std::max(0.0, report.within_scan_sse / count));
  report.between_scan_rms_m = std::sqrt(std::max(0.0, report.between_scan_sse / count));
  report.between_submap_rms_m = std::sqrt(std::max(0.0, report.between_submap_sse / count));
  report.between_revisit_rms_m = std::sqrt(std::max(0.0, report.between_revisit_sse / count));
  if (report.total_sse > 0.0) {
    report.within_scan_fraction = report.within_scan_sse / report.total_sse;
    report.between_scan_fraction = report.between_scan_sse / report.total_sse;
    report.between_submap_fraction = report.between_submap_sse / report.total_sse;
    report.between_revisit_fraction = report.between_revisit_sse / report.total_sse;
  }
  const double closure_tolerance = 1.0e-9 * std::max(1.0, report.total_sse);
  report.meaningful = report.total_sse > 0.0 && report.closure_error <= closure_tolerance;
  return report;
}

inline std::vector<std::string> reportYamlLines(
  const AttributionReport & report,
  const AttributionConfig & config = AttributionConfig())
{
  std::vector<std::string> lines;
  lines.push_back("map_thickness_attribution:");
  lines.push_back("  schema_version: 1");
  lines.push_back("  input_points: " + std::to_string(report.input_points));
  lines.push_back("  planar_points: " + std::to_string(report.planar_points));
  lines.push_back("  plane_patch_count: " + std::to_string(report.plane_patch_count));
  lines.push_back("  distinct_scans: " + std::to_string(report.distinct_scans));
  lines.push_back("  distinct_submaps: " + std::to_string(report.distinct_submaps));
  lines.push_back("  distinct_revisits: " + std::to_string(report.distinct_revisits));
  lines.push_back("  planar_coverage: " + detail::formatDouble(report.planar_coverage));
  lines.push_back("  total_rms_m: " + detail::formatDouble(report.total_rms_m));
  lines.push_back("  components:");
  lines.push_back("    within_scan_rms_m: " + detail::formatDouble(report.within_scan_rms_m));
  lines.push_back("    between_scan_rms_m: " + detail::formatDouble(report.between_scan_rms_m));
  lines.push_back("    between_submap_rms_m: " + detail::formatDouble(
      report.between_submap_rms_m));
  lines.push_back("    between_revisit_rms_m: " + detail::formatDouble(
      report.between_revisit_rms_m));
  lines.push_back("    within_scan_fraction: " + detail::formatDouble(
      report.within_scan_fraction));
  lines.push_back("    between_scan_fraction: " + detail::formatDouble(
      report.between_scan_fraction));
  lines.push_back("    between_submap_fraction: " + detail::formatDouble(
      report.between_submap_fraction));
  lines.push_back("    between_revisit_fraction: " + detail::formatDouble(
      report.between_revisit_fraction));
  lines.push_back("  closure_error: " + detail::formatDouble(report.closure_error));
  lines.push_back(std::string("  meaningful: ") + (report.meaningful ? "true" : "false"));
  lines.push_back("  extraction:");
  lines.push_back("    root_voxel_size_m: " + detail::formatDouble(
      config.plane_config.root_voxel_size));
  lines.push_back("    max_octree_depth: " + std::to_string(
      config.plane_config.max_octree_depth));
  lines.push_back("    min_points_per_plane: " + std::to_string(
      config.plane_config.min_points_per_plane));
  lines.push_back("    max_plane_thickness_m: " + detail::formatDouble(
      config.plane_config.max_plane_thickness));
  lines.push_back("    min_planarity_ratio: " + detail::formatDouble(
      config.plane_config.min_planarity_ratio));
  return lines;
}

}  // namespace map_thickness
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__MAP_THICKNESS_ATTRIBUTION_HPP_
