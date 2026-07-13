// Copyright 2026 Sasaki
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef GRAPH_BASED_SLAM__PLANE_REVISIT_CONSTRAINTS_HPP_
#define GRAPH_BASED_SLAM__PLANE_REVISIT_CONSTRAINTS_HPP_

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Core>  // NOLINT(build/include_order)
#include <Eigen/Eigenvalues>  // NOLINT(build/include_order)
#include <Eigen/Geometry>  // NOLINT(build/include_order)

#include "graph_based_slam/plane_ba.hpp"
#include "graph_based_slam/pose_graph_optimization.hpp"

namespace graphslam
{
namespace pose_graph
{

struct PlaneRevisitBuilderConfig
{
  int min_support_points {20};
  int min_pose_separation {5};
  int max_constraints_per_feature {4};
  double max_planarity_rmse_m {0.08};
  double normal_info_weight {10.0};
  double offset_info_weight {10.0};
  double robust_kernel_delta {1.0};
};

struct PlaneRevisitBuilderResult
{
  std::vector<PlaneRevisitConstraint> constraints;
  int features_seen {0};
  int features_with_constraints {0};
  int observations_seen {0};
  int observations_rejected {0};
  int max_pose_separation {0};
};

struct PlaneRevisitGateResult
{
  std::vector<PlaneRevisitConstraint> constraints;
  int rejected {0};
  double accepted_max_normal_error_deg {0.0};
  double accepted_max_offset_error_m {0.0};
};

namespace detail
{

inline bool fitLocalPlane(
  const map_refinement::PointCluster & cluster,
  const PlaneRevisitBuilderConfig & config,
  LocalPlaneObservation * plane)
{
  if (cluster.count < config.min_support_points || cluster.n() <= 0.0) {
    return false;
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cluster.covariance());
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) {
    return false;
  }
  const double smallest = std::max(0.0, solver.eigenvalues()(0));
  if (std::sqrt(smallest) > config.max_planarity_rmse_m) {
          return false;
  }
  if (solver.eigenvalues()(1) <= smallest + 1e-12) {
          return false;
  }
  plane->normal = solver.eigenvectors().col(0);
  Eigen::Index largest = 0;
  plane->normal.cwiseAbs().maxCoeff(&largest);
  if (plane->normal(largest) < 0.0) {
    plane->normal = -plane->normal;
  }
  plane->offset = -plane->normal.dot(cluster.centroid());
  plane->support_points = static_cast<int>(cluster.count);
  return plane->normal.allFinite() && std::isfinite(plane->offset);
}

}  // namespace detail

/// Convert multi-keyframe plane associations into sparse pose-graph factors.
/// Each feature uses its earliest valid observation as an anchor and connects
/// only temporally separated observations, avoiding a dense all-pairs graph.
inline PlaneRevisitBuilderResult buildPlaneRevisitConstraints(
  const std::vector<map_refinement::PlaneFeature> & features,
  const PlaneRevisitBuilderConfig & config)
{
  PlaneRevisitBuilderResult result;
  result.features_seen = static_cast<int>(features.size());
  for (const auto & feature : features) {
    struct FittedObservation
    {
      int pose_index;
      LocalPlaneObservation plane;
    };
    std::vector<FittedObservation> fitted;
    fitted.reserve(feature.observations.size());
    for (const auto & observation : feature.observations) {
      ++result.observations_seen;
      LocalPlaneObservation plane;
      if (observation.pose_index < 0 ||
        !detail::fitLocalPlane(observation.local_cluster, config, &plane))
      {
        ++result.observations_rejected;
        continue;
      }
      fitted.push_back({observation.pose_index, plane});
    }
    if (fitted.size() < 2U) {
      continue;
    }
    std::sort(
      fitted.begin(), fitted.end(),
      [](const FittedObservation & a, const FittedObservation & b) {
        return a.pose_index < b.pose_index;
      });
    const FittedObservation & anchor = fitted.front();
    int emitted = 0;
    for (std::size_t i = 1; i < fitted.size(); ++i) {
      const int pose_separation = fitted[i].pose_index - anchor.pose_index;
      result.max_pose_separation = std::max(result.max_pose_separation, pose_separation);
      if (pose_separation < config.min_pose_separation) {
        continue;
      }
      PlaneRevisitConstraint constraint;
      constraint.from = anchor.pose_index;
      constraint.to = fitted[i].pose_index;
      constraint.measurement.from = anchor.plane;
      constraint.measurement.to = fitted[i].plane;
      constraint.normal_info_weight = config.normal_info_weight;
      constraint.offset_info_weight = config.offset_info_weight;
      constraint.robust_kernel_delta = config.robust_kernel_delta;
      result.constraints.push_back(constraint);
      ++emitted;
      if (emitted == 1) {
        ++result.features_with_constraints;
      }
      if (emitted >= config.max_constraints_per_feature) {
        break;
      }
    }
  }
  return result;
}

/// Reject an association that is incompatible with the poses used to create
/// the world-frame plane patches. Robust kernels remain useful for small
/// residuals, but should not be the first line of defence against a different
/// parallel wall or a mixed patch.
inline PlaneRevisitGateResult gatePlaneRevisitConstraintsByInitialResidual(
  const std::vector<PlaneRevisitConstraint> & constraints,
  const std::vector<Eigen::Isometry3d> & poses,
  const double max_normal_error_deg,
  const double max_offset_error_m)
{
  PlaneRevisitGateResult result;
  result.constraints.reserve(constraints.size());
  const double radians_to_degrees = 180.0 / std::acos(-1.0);
  for (const auto & constraint : constraints) {
    if (constraint.from < 0 || constraint.to < 0 ||
      static_cast<std::size_t>(constraint.from) >= poses.size() ||
      static_cast<std::size_t>(constraint.to) >= poses.size())
    {
      ++result.rejected;
      continue;
    }
    const LocalPlaneObservation from_world = transformPlaneToWorld(
      constraint.measurement.from, poses[static_cast<std::size_t>(constraint.from)]);
    LocalPlaneObservation to_world = transformPlaneToWorld(
      constraint.measurement.to, poses[static_cast<std::size_t>(constraint.to)]);
    double normal_dot = from_world.normal.dot(to_world.normal);
    if (normal_dot < 0.0) {
      normal_dot = -normal_dot;
      to_world.normal = -to_world.normal;
      to_world.offset = -to_world.offset;
    }
    normal_dot = std::max(-1.0, std::min(1.0, normal_dot));
    const double normal_error_deg = std::acos(normal_dot) * radians_to_degrees;
    const double offset_error_m = std::abs(from_world.offset - to_world.offset);
    if (!std::isfinite(normal_error_deg) || !std::isfinite(offset_error_m) ||
      normal_error_deg > max_normal_error_deg || offset_error_m > max_offset_error_m)
    {
      ++result.rejected;
      continue;
    }
    result.accepted_max_normal_error_deg = std::max(
      result.accepted_max_normal_error_deg, normal_error_deg);
    result.accepted_max_offset_error_m = std::max(
      result.accepted_max_offset_error_m, offset_error_m);
    result.constraints.push_back(constraint);
  }
  return result;
}

}  // namespace pose_graph
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__PLANE_REVISIT_CONSTRAINTS_HPP_
