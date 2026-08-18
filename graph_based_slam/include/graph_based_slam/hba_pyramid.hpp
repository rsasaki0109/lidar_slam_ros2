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

#ifndef GRAPH_BASED_SLAM__HBA_PYRAMID_HPP_
#define GRAPH_BASED_SLAM__HBA_PYRAMID_HPP_

#include <algorithm>
#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Dense>  // NOLINT(build/include_order)

#include "graph_based_slam/plane_ba.hpp"
#include "graph_based_slam/plane_feature_association.hpp"
#include "graph_based_slam/se3_lie.hpp"

namespace graphslam
{
namespace map_refinement
{

/// Hierarchical (HBA-style) plane-BA refinement over a full pose sequence.
///
/// This is the v0.7 Phase 2 orchestration layer from docs/roadmap/v0.7.md and
/// the design note: overlapping bottom-layer windows are refined by local plane
/// BA, corrections are propagated through the pose chain, and an optional global
/// polish pass removes residual long-wavelength error for small sequences.
///
/// Determinism is intentional: windows are processed in ascending start index,
/// the supplied configs are fixed, and no clocks or randomness are used. In an
/// overlap, later windows overwrite earlier estimates and the last write wins.
/// This deliberately avoids averaging overlapping SE(3) poses. Upper hierarchy
/// layers and graph fusion are left for later; large sequences skip the global
/// polish pass when they exceed global_pass_max_poses.
struct HbaPyramidConfig
{
  AssociationConfig association;
  PlaneBaConfig window_ba;
  int window_size {16};
  int window_stride {8};
  bool run_global_pass {true};
  int global_pass_max_poses {64};
  PlaneBaConfig global_ba;
};

struct WindowReport
{
  int start_index {0};
  int pose_count {0};
  int features {0};
  bool improved {false};
  std::string termination;
  double initial_cost {0.0};
  double final_cost {0.0};
};

struct HbaPyramidResult
{
  std::vector<Eigen::Matrix4d> poses;
  std::vector<WindowReport> windows;
  bool any_window_improved {false};
  bool global_pass_ran {false};
  WindowReport global_report;
};

namespace detail
{

inline std::vector<int> makeWindowStarts(
  const int pose_count,
  const int requested_window_size,
  const int requested_stride)
{
  std::vector<int> starts;
  if (pose_count <= 0) {
    return starts;
  }

  const int window_size = std::max(1, requested_window_size);
  const int stride = std::max(1, requested_stride);
  int next_start = 0;

  while (next_start < pose_count) {
    int start_index = next_start;
    if (start_index + window_size >= pose_count) {
      start_index = std::max(0, pose_count - window_size);
    }

    if (starts.empty() || starts.back() != start_index) {
      starts.push_back(start_index);
    }

    if (start_index + window_size >= pose_count) {
      break;
    }
    next_start += stride;
  }

  return starts;
}

inline std::vector<std::vector<Eigen::Vector3d>> copyCloudWindow(
  const std::vector<std::vector<Eigen::Vector3d>> & local_clouds,
  const int start_index,
  const int pose_count)
{
  const auto first = local_clouds.begin() + start_index;
  const auto last = first + pose_count;
  return std::vector<std::vector<Eigen::Vector3d>>(first, last);
}

inline std::vector<Eigen::Matrix4d> copyPoseWindow(
  const std::vector<Eigen::Matrix4d> & poses,
  const int start_index,
  const int pose_count)
{
  const auto first = poses.begin() + start_index;
  const auto last = first + pose_count;
  return std::vector<Eigen::Matrix4d>(first, last);
}

inline void fillReportFromBa(
  const PlaneBaResult & ba_result,
  WindowReport * report)
{
  report->improved = ba_result.improved;
  report->termination = ba_result.termination;
  report->initial_cost = ba_result.initial_cost;
  report->final_cost = ba_result.final_cost;
}

}  // namespace detail

inline HbaPyramidResult refinePosesHierarchically(
  const std::vector<std::vector<Eigen::Vector3d>> & local_clouds,
  const std::vector<Eigen::Matrix4d> & initial_poses,
  const HbaPyramidConfig & config)
{
  HbaPyramidResult result;
  result.poses = initial_poses;

  if (initial_poses.empty() || local_clouds.size() != initial_poses.size()) {
    return result;
  }

  std::vector<Eigen::Matrix4d> poses = initial_poses;
  const int pose_count = static_cast<int>(poses.size());
  const int window_size = std::max(1, config.window_size);
  const std::vector<int> starts = detail::makeWindowStarts(
    pose_count,
    window_size,
    config.window_stride);

  for (std::size_t start_i = 0; start_i < starts.size(); ++start_i) {
    const int start_index = starts[start_i];
    const int end_index = std::min(pose_count, start_index + window_size);
    const int window_pose_count = end_index - start_index;

    WindowReport report;
    report.start_index = start_index;
    report.pose_count = window_pose_count;

    const std::vector<std::vector<Eigen::Vector3d>> window_clouds =
      detail::copyCloudWindow(
      local_clouds,
      start_index,
      window_pose_count);
    std::vector<Eigen::Matrix4d> window_poses = detail::copyPoseWindow(
      poses,
      start_index,
      window_pose_count);
    const std::vector<Eigen::Matrix4d> window_prior_poses = detail::copyPoseWindow(
      initial_poses,
      start_index,
      window_pose_count);

    const AssociationResult association = associatePlaneFeatures(
      window_clouds,
      window_poses,
      config.association);
    report.features = static_cast<int>(association.features.size());

    if (association.features.empty()) {
      report.termination = "no_valid_features";
      result.windows.push_back(report);
      continue;
    }

    PlaneBaConfig window_config = config.window_ba;
    window_config.fix_first_pose = true;

    const PlaneBaResult ba_result = solvePlaneBa(
      association.features,
      window_poses,
      window_config,
      window_prior_poses);
    detail::fillReportFromBa(ba_result, &report);
    result.any_window_improved = result.any_window_improved ||
      ba_result.improved;

    const std::size_t write_count = std::min(
      ba_result.poses.size(),
      window_poses.size());
    for (std::size_t pose_i = 1; pose_i < write_count; ++pose_i) {
      poses[start_index + static_cast<int>(pose_i)] = ba_result.poses[pose_i];
    }

    poses.front() = initial_poses.front();
    result.windows.push_back(report);
  }

  if (config.run_global_pass && pose_count <= config.global_pass_max_poses) {
    result.global_pass_ran = true;

    WindowReport report;
    report.start_index = 0;
    report.pose_count = pose_count;

    const AssociationResult association = associatePlaneFeatures(
      local_clouds,
      poses,
      config.association);
    report.features = static_cast<int>(association.features.size());

    if (association.features.empty()) {
      report.termination = "no_valid_features";
    } else {
      PlaneBaConfig global_config = config.global_ba;
      global_config.fix_first_pose = true;

      const PlaneBaResult ba_result = solvePlaneBa(
        association.features,
        poses,
        global_config,
        initial_poses);
      detail::fillReportFromBa(ba_result, &report);

      const std::size_t write_count = std::min(
        ba_result.poses.size(),
        poses.size());
      for (std::size_t pose_i = 0; pose_i < write_count; ++pose_i) {
        poses[pose_i] = ba_result.poses[pose_i];
      }
      poses.front() = initial_poses.front();
    }

    result.global_report = report;
  }

  poses.front() = initial_poses.front();
  result.poses = poses;
  return result;
}

}  // namespace map_refinement
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__HBA_PYRAMID_HPP_
