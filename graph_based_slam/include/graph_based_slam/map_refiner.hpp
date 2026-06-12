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

#ifndef GRAPH_BASED_SLAM__MAP_REFINER_HPP_
#define GRAPH_BASED_SLAM__MAP_REFINER_HPP_

// Public entry point of the v0.7 offline map refinement
// (docs/roadmap/v0.7.md Phase 2): deterministic per-submap downsample ->
// hierarchical plane BA over the pose sequence -> conservative
// acceptance. When the refinement does not improve anything the input
// poses are returned bitwise-unchanged and the report says why — the
// reject-with-report fallback that bounds the blast radius of the
// offline runner's --refine stage. Same inputs ⇒ same refined poses and
// same report bytes.

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include <Eigen/Core>  // NOLINT(build/include_order)

#include "graph_based_slam/hba_pyramid.hpp"
#include "graph_based_slam/map_quality_metrics.hpp"

namespace graphslam
{
namespace map_refinement
{

struct MapRefinerConfig
{
  // Deterministic voxel-centroid downsample applied per submap cloud
  // before the BA (0 = off). Bounds the point-cluster build cost.
  double cloud_downsample_voxel {0.10};
  HbaPyramidConfig pyramid;
};

struct MapRefinerResult
{
  std::vector<Eigen::Matrix4d> poses;  // refined, or the input when !accepted
  bool accepted {false};
  std::string status;                  // "refined" | "no_improvement" | "empty_input"
  std::int64_t input_points {0};
  std::int64_t downsampled_points {0};
  HbaPyramidResult pyramid_result;
};

inline MapRefinerResult refineSubmapPoses(
  const std::vector<std::vector<Eigen::Vector3d>> & local_clouds,
  const std::vector<Eigen::Matrix4d> & initial_poses,
  const MapRefinerConfig & config)
{
  MapRefinerResult result;
  result.poses = initial_poses;
  if (initial_poses.empty() || local_clouds.size() != initial_poses.size()) {
    result.status = "empty_input";
    return result;
  }

  std::vector<std::vector<Eigen::Vector3d>> downsampled;
  downsampled.reserve(local_clouds.size());
  for (size_t i = 0; i < local_clouds.size(); ++i) {
    result.input_points += static_cast<std::int64_t>(local_clouds[i].size());
    downsampled.push_back(
      map_quality::downsampleByVoxelCentroid(local_clouds[i], config.cloud_downsample_voxel));
    result.downsampled_points += static_cast<std::int64_t>(downsampled.back().size());
  }

  result.pyramid_result =
    refinePosesHierarchically(downsampled, initial_poses, config.pyramid);

  const bool improved =
    result.pyramid_result.any_window_improved ||
    (result.pyramid_result.global_pass_ran && result.pyramid_result.global_report.improved);
  if (!improved) {
    // Conservative fallback: the gates can never get worse than the
    // pose-graph solution.
    result.status = "no_improvement";
    return result;
  }
  result.poses = result.pyramid_result.poses;
  result.accepted = true;
  result.status = "refined";
  return result;
}

namespace detail
{

inline std::string refinerDouble(double value)
{
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.9f", value);
  return std::string(buffer);
}

}  // namespace detail

// Fixed-format report: the offline determinism gate compares these bytes.
inline std::vector<std::string> refinerReportYamlLines(
  const MapRefinerResult & result, const MapRefinerConfig & config)
{
  std::vector<std::string> lines;
  lines.push_back("map_refinement_report:");
  lines.push_back("  status: " + result.status);
  lines.push_back(
    "  accepted: " + std::string(result.accepted ? "true" : "false"));
  lines.push_back("  input_points: " + std::to_string(result.input_points));
  lines.push_back(
    "  downsampled_points: " + std::to_string(result.downsampled_points));
  lines.push_back(
    "  cloud_downsample_voxel_m: " +
    detail::refinerDouble(config.cloud_downsample_voxel));
  lines.push_back(
    "  windows: " + std::to_string(result.pyramid_result.windows.size()));
  int improved_windows = 0;
  int no_feature_windows = 0;
  for (size_t i = 0; i < result.pyramid_result.windows.size(); ++i) {
    if (result.pyramid_result.windows[i].improved) {
      ++improved_windows;
    }
    if (result.pyramid_result.windows[i].termination == "no_valid_features") {
      ++no_feature_windows;
    }
  }
  lines.push_back("  improved_windows: " + std::to_string(improved_windows));
  lines.push_back("  no_feature_windows: " + std::to_string(no_feature_windows));
  lines.push_back(
    "  global_pass_ran: " +
    std::string(result.pyramid_result.global_pass_ran ? "true" : "false"));
  if (result.pyramid_result.global_pass_ran) {
    lines.push_back(
      "  global_initial_cost: " +
      detail::refinerDouble(result.pyramid_result.global_report.initial_cost));
    lines.push_back(
      "  global_final_cost: " +
      detail::refinerDouble(result.pyramid_result.global_report.final_cost));
  }
  return lines;
}

}  // namespace map_refinement
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__MAP_REFINER_HPP_
