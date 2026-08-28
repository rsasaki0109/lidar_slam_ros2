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

#ifndef GRAPH_BASED_SLAM__SCAN_SURFACE_REFINER_HPP_
#define GRAPH_BASED_SLAM__SCAN_SURFACE_REFINER_HPP_

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <tuple>
#include <vector>

#include "graph_based_slam/map_quality_metrics.hpp"
#include "graph_based_slam/probabilistic_surfel_fusion.hpp"

namespace graphslam
{
namespace scan_surface_refinement
{

struct ScanSurfaceRefinerScan
{
  std::uint64_t scan_id {0};
  double stamp_sec {0.0};
  Eigen::Matrix4d world_pose {Eigen::Matrix4d::Identity()};
  double pose_translation_variance_m2 {0.0};
  double pose_rotation_variance_rad2 {0.0};
  std::vector<Eigen::Vector3d> local_points;
  const std::vector<Eigen::Vector3d> * local_points_view {nullptr};

  const std::vector<Eigen::Vector3d> & points() const
  {
    return local_points_view == nullptr ? local_points : *local_points_view;
  }
};

struct ScanSurfaceRefinerConfig
{
  double scan_downsample_voxel_size_m {0.20};
  double support_voxel_size_m {0.50};
  bool cross_fit_scan_parity {false};
  ProbabilisticSurfelFusionConfig fusion {};
  std::size_t min_surface_observations_per_scan {20};
  double residual_huber_delta_m {0.05};
  double measurement_sigma_floor_m {0.01};
  double absolute_translation_prior_sigma_m {0.02};
  double temporal_smoothness_sigma_m {0.01};
  double max_total_translation_correction_m {0.01};
  double min_relative_objective_decrease {1.0e-6};
};

struct ScanSurfaceRefinerResult
{
  std::vector<Eigen::Matrix4d> corrected_poses;
  std::vector<Eigen::Vector3d> translation_corrections;
  bool accepted {false};
  std::size_t input_points {0};
  std::size_t downsampled_points {0};
  std::size_t occupied_support_voxels {0};
  std::size_t valid_support_surfels {0};
  std::size_t surface_observations {0};
  std::size_t constrained_scans {0};
  double initial_objective {0.0};
  double final_objective {0.0};
  double initial_surface_rms_m {0.0};
  double final_surface_rms_m {0.0};
  double correction_rms_m {0.0};
  double correction_max_m {0.0};
};

namespace detail
{

using VoxelKey = std::tuple<std::int64_t, std::int64_t, std::int64_t>;

inline VoxelKey voxelKey(const Eigen::Vector3d & point, const double voxel_size_m)
{
  return VoxelKey(
    static_cast<std::int64_t>(std::floor(point.x() / voxel_size_m)),
    static_cast<std::int64_t>(std::floor(point.y() / voxel_size_m)),
    static_cast<std::int64_t>(std::floor(point.z() / voxel_size_m)));
}

struct KeyedScanPoint
{
  VoxelKey key;
  Eigen::Vector3d world_point {Eigen::Vector3d::Zero()};
  std::uint64_t scan_id {0};
  std::uint32_t scan_index {0};
};

inline bool keyedScanPointLess(const KeyedScanPoint & lhs, const KeyedScanPoint & rhs)
{
  if (lhs.key != rhs.key) {return lhs.key < rhs.key;}
  if (lhs.scan_id != rhs.scan_id) {return lhs.scan_id < rhs.scan_id;}
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    if (lhs.world_point[axis] != rhs.world_point[axis]) {
      return lhs.world_point[axis] < rhs.world_point[axis];
    }
  }
  return lhs.scan_index < rhs.scan_index;
}

struct ScanNormalEquations
{
  Eigen::Matrix3d hessian {Eigen::Matrix3d::Zero()};
  Eigen::Vector3d gradient {Eigen::Vector3d::Zero()};
  double constant {0.0};
  Eigen::Matrix3d unweighted_hessian {Eigen::Matrix3d::Zero()};
  Eigen::Vector3d unweighted_gradient {Eigen::Vector3d::Zero()};
  double unweighted_constant {0.0};
  std::size_t observations {0};
};

inline bool validConfig(const ScanSurfaceRefinerConfig & config)
{
  return config.scan_downsample_voxel_size_m >= 0.0 &&
         std::isfinite(config.scan_downsample_voxel_size_m) &&
         config.support_voxel_size_m > 0.0 &&
         std::isfinite(config.support_voxel_size_m) &&
         config.min_surface_observations_per_scan > 0U &&
         config.residual_huber_delta_m > 0.0 &&
         std::isfinite(config.residual_huber_delta_m) &&
         config.measurement_sigma_floor_m > 0.0 &&
         std::isfinite(config.measurement_sigma_floor_m) &&
         config.absolute_translation_prior_sigma_m > 0.0 &&
         std::isfinite(config.absolute_translation_prior_sigma_m) &&
         config.temporal_smoothness_sigma_m > 0.0 &&
         std::isfinite(config.temporal_smoothness_sigma_m) &&
         config.max_total_translation_correction_m > 0.0 &&
         std::isfinite(config.max_total_translation_correction_m) &&
         config.min_relative_objective_decrease >= 0.0 &&
         std::isfinite(config.min_relative_objective_decrease);
}

}  // namespace detail

inline ScanSurfaceRefinerResult refineScanSurfaceTranslations(
  const std::vector<ScanSurfaceRefinerScan> & scans,
  const ScanSurfaceRefinerConfig & config = {})
{
  ScanSurfaceRefinerResult result;
  result.corrected_poses.reserve(scans.size());
  for (const ScanSurfaceRefinerScan & scan : scans) {
    result.corrected_poses.push_back(scan.world_pose);
    result.input_points += scan.points().size();
  }
  result.translation_corrections.assign(scans.size(), Eigen::Vector3d::Zero());
  if (scans.empty() || !detail::validConfig(config)) {return result;}

  std::vector<std::vector<Eigen::Vector3d>> downsampled_scans(scans.size());
  for (std::size_t scan_index = 0; scan_index < scans.size(); ++scan_index) {
    const ScanSurfaceRefinerScan & scan = scans[scan_index];
    if (!scan.world_pose.allFinite() ||
      !std::isfinite(scan.pose_translation_variance_m2) ||
      scan.pose_translation_variance_m2 < 0.0 ||
      !std::isfinite(scan.pose_rotation_variance_rad2) ||
      scan.pose_rotation_variance_rad2 < 0.0)
    {
      continue;
    }
    downsampled_scans[scan_index] = map_quality::downsampleByVoxelCentroid(
      scan.points(), config.scan_downsample_voxel_size_m);
    result.downsampled_points += downsampled_scans[scan_index].size();
  }
  std::vector<detail::KeyedScanPoint> points;
  points.reserve(result.downsampled_points);
  for (std::size_t scan_index = 0; scan_index < scans.size(); ++scan_index) {
    const ScanSurfaceRefinerScan & scan = scans[scan_index];
    if (downsampled_scans[scan_index].empty()) {continue;}
    const Eigen::Matrix3d rotation = scan.world_pose.block<3, 3>(0, 0);
    const Eigen::Vector3d translation = scan.world_pose.block<3, 1>(0, 3);
    for (const Eigen::Vector3d & local_point : downsampled_scans[scan_index]) {
      if (!local_point.allFinite()) {continue;}
      const Eigen::Vector3d world_point = rotation * local_point + translation;
      points.push_back(detail::KeyedScanPoint{
            detail::voxelKey(world_point, config.support_voxel_size_m),
            world_point, scan.scan_id, static_cast<std::uint32_t>(scan_index)});
    }
  }
  std::sort(points.begin(), points.end(), detail::keyedScanPointLess);

  std::vector<detail::ScanNormalEquations> equations(scans.size());
  for (std::size_t first = 0U; first < points.size(); ) {
    std::size_t last = first + 1U;
    while (last < points.size() && points[last].key == points[first].key) {++last;}
    ++result.occupied_support_voxels;
    std::vector<SurfelObservation> observations;
    observations.reserve(last - first);
    for (std::size_t i = first; i < last; ++i) {
      const ScanSurfaceRefinerScan & scan = scans[points[i].scan_index];
      SurfelObservation observation;
      observation.position = points[i].world_point;
      observation.sensor_origin = scan.world_pose.block<3, 1>(0, 3);
      observation.scan_id = scan.scan_id;
      observation.pose_translation_variance_m2 = scan.pose_translation_variance_m2;
      observation.pose_rotation_variance_rad2 = scan.pose_rotation_variance_rad2;
      observations.push_back(observation);
    }
    std::vector<SurfelObservation> parity_observations[2];
    ProbabilisticSurfel parity_surfels[2];
    ProbabilisticSurfel shared_surfel;
    if (config.cross_fit_scan_parity) {
      for (const SurfelObservation & observation : observations) {
        parity_observations[observation.scan_id % 2U].push_back(observation);
      }
      for (std::size_t parity = 0U; parity < 2U; ++parity) {
        parity_surfels[parity] = fuseProbabilisticSurfel(
          parity_observations[parity], config.fusion);
        if (parity_surfels[parity].valid) {++result.valid_support_surfels;}
      }
    } else {
      shared_surfel = fuseProbabilisticSurfel(observations, config.fusion);
      if (shared_surfel.valid) {++result.valid_support_surfels;}
    }
    for (std::size_t scan_first = first; scan_first < last; ) {
      std::size_t scan_last = scan_first + 1U;
      while (scan_last < last && points[scan_last].scan_id == points[scan_first].scan_id) {
        ++scan_last;
      }
      const ProbabilisticSurfel & surfel = config.cross_fit_scan_parity ?
        parity_surfels[1U - (points[scan_first].scan_id % 2U)] : shared_surfel;
      if (surfel.valid) {
        const double variance = surfel.raw_normal_rms_m * surfel.raw_normal_rms_m +
          config.measurement_sigma_floor_m * config.measurement_sigma_floor_m;
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        for (std::size_t i = scan_first; i < scan_last; ++i) {
          centroid += points[i].world_point;
        }
        centroid /= static_cast<double>(scan_last - scan_first);
        const double residual = surfel.normal.dot(centroid - surfel.mean);
        const double huber_weight = std::min(
          1.0, config.residual_huber_delta_m / std::max(std::abs(residual), 1.0e-12));
        const double information = huber_weight / variance;
        detail::ScanNormalEquations & equation = equations[points[scan_first].scan_index];
        const Eigen::Matrix3d normal_outer = surfel.normal * surfel.normal.transpose();
        equation.hessian.noalias() += information * normal_outer;
        equation.gradient.noalias() += information * surfel.normal * residual;
        equation.constant += information * residual * residual;
        equation.unweighted_hessian.noalias() += normal_outer;
        equation.unweighted_gradient.noalias() += surfel.normal * residual;
        equation.unweighted_constant += residual * residual;
        ++equation.observations;
        ++result.surface_observations;
      }
      scan_first = scan_last;
    }
    first = last;
  }

  const std::size_t scan_count = scans.size();
  const double prior_information = 1.0 /
    (config.absolute_translation_prior_sigma_m *
    config.absolute_translation_prior_sigma_m);
  const double smoothness_information = 1.0 /
    (config.temporal_smoothness_sigma_m * config.temporal_smoothness_sigma_m);
  std::vector<Eigen::Matrix3d> diagonal(scan_count, Eigen::Matrix3d::Zero());
  std::vector<Eigen::Vector3d> right_hand_side(scan_count, Eigen::Vector3d::Zero());
  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  for (std::size_t i = 0; i < scan_count; ++i) {
    detail::ScanNormalEquations & equation = equations[i];
    if (equation.observations >= config.min_surface_observations_per_scan) {
      const double count = static_cast<double>(equation.observations);
      equation.hessian /= count;
      equation.gradient /= count;
      equation.constant /= count;
      equation.unweighted_hessian /= count;
      equation.unweighted_gradient /= count;
      equation.unweighted_constant /= count;
      diagonal[i] = equation.hessian;
      right_hand_side[i] = -equation.gradient;
      ++result.constrained_scans;
    }
    diagonal[i] += prior_information * identity;
    if (i > 0U) {diagonal[i] += smoothness_information * identity;}
    if (i + 1U < scan_count) {diagonal[i] += smoothness_information * identity;}
  }
  if (result.valid_support_surfels == 0U || result.constrained_scans == 0U) {return result;}

  const Eigen::Matrix3d off_diagonal = -smoothness_information * identity;
  std::vector<Eigen::Matrix3d> reduced_diagonal = diagonal;
  std::vector<Eigen::Vector3d> reduced_rhs = right_hand_side;
  for (std::size_t i = 1U; i < scan_count; ++i) {
    const Eigen::LDLT<Eigen::Matrix3d> previous_solver(reduced_diagonal[i - 1U]);
    if (previous_solver.info() != Eigen::Success) {return result;}
    const Eigen::Matrix3d inverse_times_off = previous_solver.solve(off_diagonal);
    reduced_diagonal[i].noalias() -= off_diagonal * inverse_times_off;
    reduced_rhs[i].noalias() -= off_diagonal * previous_solver.solve(reduced_rhs[i - 1U]);
  }
  const Eigen::LDLT<Eigen::Matrix3d> last_solver(reduced_diagonal.back());
  if (last_solver.info() != Eigen::Success) {return result;}
  result.translation_corrections.back() = last_solver.solve(reduced_rhs.back());
  for (std::size_t reverse = scan_count - 1U; reverse > 0U; --reverse) {
    const std::size_t i = reverse - 1U;
    const Eigen::LDLT<Eigen::Matrix3d> solver(reduced_diagonal[i]);
    if (solver.info() != Eigen::Success) {
      result.translation_corrections.assign(scan_count, Eigen::Vector3d::Zero());
      return result;
    }
    result.translation_corrections[i] = solver.solve(
      reduced_rhs[i] - off_diagonal * result.translation_corrections[i + 1U]);
  }

  for (Eigen::Vector3d & correction : result.translation_corrections) {
    if (!correction.allFinite()) {
      result.translation_corrections.assign(scan_count, Eigen::Vector3d::Zero());
      return result;
    }
    const double norm = correction.norm();
    if (norm > config.max_total_translation_correction_m) {
      correction *= config.max_total_translation_correction_m / norm;
    }
  }

  double initial_unweighted_sum = 0.0;
  double final_unweighted_sum = 0.0;
  std::size_t rms_observations = 0U;
  for (std::size_t i = 0; i < scan_count; ++i) {
    const detail::ScanNormalEquations & equation = equations[i];
    const Eigen::Vector3d & correction = result.translation_corrections[i];
    if (equation.observations >= config.min_surface_observations_per_scan) {
      result.initial_objective += equation.constant;
      result.final_objective += equation.constant +
        2.0 * equation.gradient.dot(correction) +
        correction.dot(equation.hessian * correction);
      initial_unweighted_sum += equation.unweighted_constant;
      final_unweighted_sum += equation.unweighted_constant +
        2.0 * equation.unweighted_gradient.dot(correction) +
        correction.dot(equation.unweighted_hessian * correction);
      rms_observations += equation.observations;
    }
    result.final_objective += prior_information * correction.squaredNorm();
    if (i > 0U) {
      result.final_objective += smoothness_information *
        (correction - result.translation_corrections[i - 1U]).squaredNorm();
    }
  }
  const double relative_decrease =
    (result.initial_objective - result.final_objective) /
    std::max(std::abs(result.initial_objective), 1.0e-300);
  result.accepted = std::isfinite(result.final_objective) &&
    result.final_objective < result.initial_objective &&
    relative_decrease >= config.min_relative_objective_decrease;
  if (!result.accepted) {
    result.translation_corrections.assign(scan_count, Eigen::Vector3d::Zero());
    result.final_objective = result.initial_objective;
    final_unweighted_sum = initial_unweighted_sum;
  }
  if (rms_observations > 0U) {
    result.initial_surface_rms_m = std::sqrt(
      initial_unweighted_sum / static_cast<double>(rms_observations));
    result.final_surface_rms_m = std::sqrt(
      std::max(0.0, final_unweighted_sum) / static_cast<double>(rms_observations));
  }
  double correction_squared_sum = 0.0;
  for (std::size_t i = 0; i < scan_count; ++i) {
    const Eigen::Vector3d & correction = result.translation_corrections[i];
    correction_squared_sum += correction.squaredNorm();
    result.correction_max_m = std::max(result.correction_max_m, correction.norm());
    result.corrected_poses[i].block<3, 1>(0, 3) += correction;
  }
  result.correction_rms_m = std::sqrt(
    correction_squared_sum / static_cast<double>(scan_count));
  return result;
}

}  // namespace scan_surface_refinement
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__SCAN_SURFACE_REFINER_HPP_
