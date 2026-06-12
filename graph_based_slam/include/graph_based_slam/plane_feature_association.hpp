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

#ifndef GRAPH_BASED_SLAM__PLANE_FEATURE_ASSOCIATION_HPP_
#define GRAPH_BASED_SLAM__PLANE_FEATURE_ASSOCIATION_HPP_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>  // NOLINT(build/include_order)

#include "graph_based_slam/adaptive_voxel_plane_extractor.hpp"
#include "graph_based_slam/plane_ba.hpp"

namespace graphslam
{
namespace map_refinement
{

/// Converts world-frame planar patches into the per-(feature, pose) local point
/// clusters consumed by plane BA (docs/roadmap/v0.7.md Phase 2). Owns the raw
/// point-to-feature assignment for one BA problem. Deterministic: pose-major
/// input concatenation, ascending index iteration.
struct AssociationResult
{
  std::vector<PlaneFeature> features;
  int patches_total {0};
  int patches_used {0};
  std::int64_t points_used {0};
};

struct AssociationConfig
{
  plane_extraction::PlaneExtractionConfig extraction;
  int min_observing_poses {2};
  int min_points_per_observation {5};
};

namespace detail
{

inline Eigen::Vector3d transformPoint(
  const Eigen::Matrix4d & pose,
  const Eigen::Vector3d & point)
{
  const Eigen::Vector4d homogeneous(
    point.x(),
    point.y(),
    point.z(),
    1.0);
  return (pose * homogeneous).head<3>();
}

inline int sanitizePositiveInt(
  const int value,
  const int fallback)
{
  if (value < 1) {
    return fallback;
  }
  return value;
}

}  // namespace detail

inline AssociationResult associatePlaneFeatures(
  const std::vector<std::vector<Eigen::Vector3d>> & local_clouds,
  const std::vector<Eigen::Matrix4d> & poses,
  const AssociationConfig & config)
{
  AssociationResult result;
  if (local_clouds.size() != poses.size()) {
    return result;
  }

  plane_extraction::PlaneExtractionConfig extraction_config = config.extraction;
  extraction_config.collect_point_indices = true;

  const int min_observing_poses =
    detail::sanitizePositiveInt(config.min_observing_poses, 1);
  const int min_points_per_observation =
    detail::sanitizePositiveInt(config.min_points_per_observation, 1);

  std::vector<std::size_t> pose_offsets;
  pose_offsets.reserve(local_clouds.size() + 1);
  pose_offsets.push_back(0U);

  for (std::size_t pose_index = 0; pose_index < local_clouds.size(); ++pose_index) {
    pose_offsets.push_back(pose_offsets.back() + local_clouds[pose_index].size());
  }

  std::vector<Eigen::Vector3d> world_cloud;
  world_cloud.reserve(pose_offsets.back());

  for (std::size_t pose_index = 0; pose_index < local_clouds.size(); ++pose_index) {
    for (std::size_t point_index = 0; point_index < local_clouds[pose_index].size();
      ++point_index)
    {
      world_cloud.push_back(
        detail::transformPoint(
          poses[pose_index],
          local_clouds[pose_index][point_index]));
    }
  }

  const plane_extraction::PlaneExtractionResult extraction =
    plane_extraction::extractPlanarPatches(world_cloud, extraction_config);
  result.patches_total = static_cast<int>(extraction.patches.size());

  for (std::size_t patch_index = 0; patch_index < extraction.patch_point_indices.size();
    ++patch_index)
  {
    const std::vector<int> & patch_indices = extraction.patch_point_indices[patch_index];
    std::vector<std::vector<int>> local_indices_by_pose(local_clouds.size());

    std::size_t pose_index = 0;
    for (std::size_t i = 0; i < patch_indices.size(); ++i) {
      const int signed_world_index = patch_indices[i];
      if (signed_world_index < 0) {
        continue;
      }

      const std::size_t world_index = static_cast<std::size_t>(signed_world_index);
      while (pose_index + 1U < pose_offsets.size() &&
        pose_offsets[pose_index + 1U] <= world_index)
      {
        ++pose_index;
      }

      if (pose_index >= local_clouds.size()) {
        continue;
      }

      const std::size_t local_index = world_index - pose_offsets[pose_index];
      local_indices_by_pose[pose_index].push_back(static_cast<int>(local_index));
    }

    PlaneFeature feature;
    std::int64_t feature_points = 0;
    for (std::size_t observation_pose = 0; observation_pose < local_indices_by_pose.size();
      ++observation_pose)
    {
      const std::vector<int> & local_indices = local_indices_by_pose[observation_pose];
      if (local_indices.size() < static_cast<std::size_t>(min_points_per_observation)) {
        continue;
      }

      PlaneFeatureObservation observation;
      observation.pose_index = static_cast<int>(observation_pose);
      for (std::size_t local_index_index = 0; local_index_index < local_indices.size();
        ++local_index_index)
      {
        const int local_index = local_indices[local_index_index];
        observation.local_cluster.add(
          local_clouds[observation_pose][static_cast<std::size_t>(local_index)]);
      }

      feature_points += static_cast<std::int64_t>(local_indices.size());
      feature.observations.push_back(observation);
    }

    if (feature.observations.size() >= static_cast<std::size_t>(min_observing_poses)) {
      result.features.push_back(feature);
      ++result.patches_used;
      result.points_used += feature_points;
    }
  }

  return result;
}

}  // namespace map_refinement
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__PLANE_FEATURE_ASSOCIATION_HPP_
