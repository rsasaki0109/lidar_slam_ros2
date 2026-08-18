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

#ifndef GRAPH_BASED_SLAM__PROBABILISTIC_SURFEL_FUSION_HPP_
#define GRAPH_BASED_SLAM__PROBABILISTIC_SURFEL_FUSION_HPP_

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace graphslam
{

struct SurfelObservation
{
  Eigen::Vector3d position {Eigen::Vector3d::Zero()};
  Eigen::Vector3d sensor_origin {Eigen::Vector3d::Zero()};
  std::uint64_t scan_id {0};
  double pose_translation_variance_m2 {0.0};
  double pose_rotation_variance_rad2 {0.0};
};

struct ProbabilisticSurfelFusionConfig
{
  std::size_t min_distinct_scans {3};
  double max_small_eigenvalue_ratio {0.10};
  double min_middle_eigenvalue_ratio {0.05};
  double base_range_sigma_m {0.008};
  double range_sigma_per_meter {0.001};
  double tangential_sigma_m {0.015};
  double huber_sigma {2.5};
  double min_variance_m2 {1.0e-8};
};

struct ProbabilisticSurfel
{
  Eigen::Vector3d mean {Eigen::Vector3d::Zero()};
  Eigen::Vector3d normal {Eigen::Vector3d::UnitZ()};
  Eigen::Matrix3d covariance {Eigen::Matrix3d::Zero()};
  std::size_t input_observations {0};
  std::size_t distinct_scans {0};
  double raw_normal_rms_m {0.0};
  double fused_normal_sigma_m {0.0};
  bool valid {false};
};

namespace probabilistic_surfel_detail
{

inline bool finiteObservation(const SurfelObservation & observation)
{
  return observation.position.allFinite() && observation.sensor_origin.allFinite() &&
         std::isfinite(observation.pose_translation_variance_m2) &&
         observation.pose_translation_variance_m2 >= 0.0 &&
         std::isfinite(observation.pose_rotation_variance_rad2) &&
         observation.pose_rotation_variance_rad2 >= 0.0;
}

inline bool observationLess(const SurfelObservation & lhs, const SurfelObservation & rhs)
{
  if (lhs.scan_id != rhs.scan_id) {return lhs.scan_id < rhs.scan_id;}
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    if (lhs.position[axis] != rhs.position[axis]) {
      return lhs.position[axis] < rhs.position[axis];
    }
  }
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    if (lhs.sensor_origin[axis] != rhs.sensor_origin[axis]) {
      return lhs.sensor_origin[axis] < rhs.sensor_origin[axis];
    }
  }
  if (lhs.pose_translation_variance_m2 != rhs.pose_translation_variance_m2) {
    return lhs.pose_translation_variance_m2 < rhs.pose_translation_variance_m2;
  }
  return lhs.pose_rotation_variance_rad2 < rhs.pose_rotation_variance_rad2;
}

inline std::vector<SurfelObservation> collapsePerScan(
  std::vector<SurfelObservation> observations)
{
  std::sort(observations.begin(), observations.end(), observationLess);
  std::vector<SurfelObservation> collapsed;
  for (std::size_t first = 0; first < observations.size(); ) {
    std::size_t last = first + 1U;
    while (last < observations.size() &&
      observations[last].scan_id == observations[first].scan_id)
    {
      ++last;
    }
    SurfelObservation aggregate;
    aggregate.scan_id = observations[first].scan_id;
    for (std::size_t index = first; index < last; ++index) {
      aggregate.position += observations[index].position;
      aggregate.sensor_origin += observations[index].sensor_origin;
      aggregate.pose_translation_variance_m2 +=
        observations[index].pose_translation_variance_m2;
      aggregate.pose_rotation_variance_rad2 +=
        observations[index].pose_rotation_variance_rad2;
    }
    const double count = static_cast<double>(last - first);
    aggregate.position /= count;
    aggregate.sensor_origin /= count;
    aggregate.pose_translation_variance_m2 /= count;
    aggregate.pose_rotation_variance_rad2 /= count;
    collapsed.push_back(aggregate);
    first = last;
  }
  return collapsed;
}

inline void orientNormalDeterministically(Eigen::Vector3d & normal)
{
  Eigen::Index dominant_axis = 0;
  normal.cwiseAbs().maxCoeff(&dominant_axis);
  if (normal[dominant_axis] < 0.0) {
    normal = -normal;
  }
}

}  // namespace probabilistic_surfel_detail

inline ProbabilisticSurfel fuseProbabilisticSurfel(
  const std::vector<SurfelObservation> & input,
  const ProbabilisticSurfelFusionConfig & config = {})
{
  ProbabilisticSurfel result;
  result.input_observations = input.size();
  if (config.min_distinct_scans < 2U || config.min_variance_m2 <= 0.0 ||
    config.base_range_sigma_m < 0.0 || config.range_sigma_per_meter < 0.0 ||
    config.tangential_sigma_m < 0.0 || config.huber_sigma <= 0.0)
  {
    return result;
  }
  std::vector<SurfelObservation> finite;
  finite.reserve(input.size());
  std::copy_if(
    input.begin(), input.end(), std::back_inserter(finite),
    probabilistic_surfel_detail::finiteObservation);
  const std::vector<SurfelObservation> observations =
    probabilistic_surfel_detail::collapsePerScan(std::move(finite));
  result.distinct_scans = observations.size();
  if (observations.size() < config.min_distinct_scans) {
    return result;
  }

  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const SurfelObservation & observation : observations) {
    mean += observation.position;
  }
  mean /= static_cast<double>(observations.size());
  Eigen::Matrix3d sample_covariance = Eigen::Matrix3d::Zero();
  for (const SurfelObservation & observation : observations) {
    const Eigen::Vector3d centered = observation.position - mean;
    sample_covariance.noalias() += centered * centered.transpose();
  }
  sample_covariance /= static_cast<double>(observations.size());
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(sample_covariance);
  if (solver.info() != Eigen::Success) {
    return result;
  }
  const Eigen::Vector3d eigenvalues = solver.eigenvalues().cwiseMax(0.0);
  const double eigenvalue_sum = eigenvalues.sum();
  if (!(eigenvalue_sum > std::numeric_limits<double>::epsilon()) ||
    eigenvalues.x() / eigenvalue_sum > config.max_small_eigenvalue_ratio ||
    eigenvalues.y() / eigenvalue_sum < config.min_middle_eigenvalue_ratio)
  {
    return result;
  }
  result.normal = solver.eigenvectors().col(0).normalized();
  probabilistic_surfel_detail::orientNormalDeterministically(result.normal);

  double raw_squared_sum = 0.0;
  double information_sum = 0.0;
  double information_offset_sum = 0.0;
  for (const SurfelObservation & observation : observations) {
    const Eigen::Vector3d ray = observation.position - observation.sensor_origin;
    const double range = ray.norm();
    const Eigen::Vector3d direction = range > 1.0e-12 ?
      ray / range : result.normal;
    const double radial_sigma =
      config.base_range_sigma_m + config.range_sigma_per_meter * range;
    const double incidence = result.normal.dot(direction);
    double normal_variance =
      config.tangential_sigma_m * config.tangential_sigma_m +
      (radial_sigma * radial_sigma -
      config.tangential_sigma_m * config.tangential_sigma_m) *
      incidence * incidence;
    normal_variance += observation.pose_translation_variance_m2 +
      range * range * observation.pose_rotation_variance_rad2;
    normal_variance = std::max(config.min_variance_m2, normal_variance);
    const double residual = result.normal.dot(observation.position - mean);
    raw_squared_sum += residual * residual;
    const double residual_sigma = std::sqrt(normal_variance + eigenvalues.x());
    const double robust_weight = std::min(
      1.0, config.huber_sigma * residual_sigma /
      std::max(std::abs(residual), 1.0e-15));
    const double information = robust_weight / normal_variance;
    information_sum += information;
    information_offset_sum += information * result.normal.dot(observation.position);
  }
  if (!(information_sum > 0.0) || !std::isfinite(information_sum)) {
    return result;
  }
  const double fused_offset = information_offset_sum / information_sum;
  result.mean = mean + (fused_offset - result.normal.dot(mean)) * result.normal;
  Eigen::Vector3d fused_eigenvalues = eigenvalues;
  fused_eigenvalues.x() = 1.0 / information_sum;
  result.covariance = solver.eigenvectors() * fused_eigenvalues.asDiagonal() *
    solver.eigenvectors().transpose();
  result.covariance = 0.5 * (result.covariance + result.covariance.transpose());
  result.raw_normal_rms_m = std::sqrt(
    raw_squared_sum / static_cast<double>(observations.size()));
  result.fused_normal_sigma_m = std::sqrt(1.0 / information_sum);
  result.valid = result.mean.allFinite() && result.covariance.allFinite();
  return result;
}

}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__PROBABILISTIC_SURFEL_FUSION_HPP_
