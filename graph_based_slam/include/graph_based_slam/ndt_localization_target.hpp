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

#ifndef GRAPH_BASED_SLAM__NDT_LOCALIZATION_TARGET_HPP_
#define GRAPH_BASED_SLAM__NDT_LOCALIZATION_TARGET_HPP_

#include <Eigen/Eigenvalues>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <tuple>
#include <vector>

namespace graphslam::ndt_localization
{

struct TangentSamplingConfig
{
  double voxel_size_m {1.0};
  double radius_m {0.5};
  double inner_radius_m {0.0};
  bool add_diagonals {true};
  bool add_angular_midpoints {false};
  std::size_t angular_midpoint_pairs {0U};
  std::size_t min_points_per_voxel {6U};
  double max_small_eigenvalue_ratio {0.10};
  double min_middle_eigenvalue_ratio {0.05};
};

struct TangentSamplingResult
{
  std::vector<Eigen::Vector3d> points;
  std::size_t input_points {0U};
  std::size_t planar_input_points {0U};
  std::size_t sampled_points {0U};
  std::size_t planar_voxels {0U};
};

namespace detail
{

using VoxelKey = std::tuple<std::int64_t, std::int64_t, std::int64_t>;

struct VoxelMoments
{
  std::size_t count {0U};
  Eigen::Vector3d sum {Eigen::Vector3d::Zero()};
  Eigen::Matrix3d second {Eigen::Matrix3d::Zero()};
};

inline VoxelKey voxelKey(const Eigen::Vector3d & point, const double voxel_size)
{
  return {
    static_cast<std::int64_t>(std::floor(point.x() / voxel_size)),
    static_cast<std::int64_t>(std::floor(point.y() / voxel_size)),
    static_cast<std::int64_t>(std::floor(point.z() / voxel_size))};
}

}  // namespace detail

inline TangentSamplingResult buildTangentSampledTarget(
  const std::vector<Eigen::Vector3d> & input, const TangentSamplingConfig & config = {})
{
  TangentSamplingResult result;
  result.input_points = input.size();
  if (!(config.voxel_size_m > 0.0) || !std::isfinite(config.voxel_size_m) ||
    !(config.radius_m > 0.0) || !std::isfinite(config.radius_m) ||
    config.radius_m > 0.5 * config.voxel_size_m || config.min_points_per_voxel < 3U ||
    !(config.inner_radius_m >= 0.0) || !std::isfinite(config.inner_radius_m) ||
    config.inner_radius_m >= config.radius_m ||
    config.angular_midpoint_pairs > 4U ||
    !(config.max_small_eigenvalue_ratio >= 0.0) ||
    !(config.max_small_eigenvalue_ratio < 1.0) ||
    !(config.min_middle_eigenvalue_ratio >= 0.0) ||
    !(config.min_middle_eigenvalue_ratio < 1.0))
  {
    return result;
  }

  std::map<detail::VoxelKey, detail::VoxelMoments> moments;
  for (const Eigen::Vector3d & point : input) {
    if (!point.allFinite()) {continue;}
    detail::VoxelMoments & cell = moments[detail::voxelKey(point, config.voxel_size_m)];
    ++cell.count;
    cell.sum += point;
    cell.second += point * point.transpose();
  }

  std::map<detail::VoxelKey, std::array<Eigen::Vector3d, 2>> tangents;
  for (const auto & item : moments) {
    const detail::VoxelMoments & cell = item.second;
    if (cell.count < config.min_points_per_voxel) {continue;}
    const Eigen::Vector3d mean = cell.sum / static_cast<double>(cell.count);
    const Eigen::Matrix3d covariance =
      cell.second / static_cast<double>(cell.count) - mean * mean.transpose();
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(covariance);
    if (eigen.info() != Eigen::Success) {continue;}
    const Eigen::Vector3d values = eigen.eigenvalues().cwiseMax(0.0);
    if (!(values[2] > 0.0) || values[0] / values[2] > config.max_small_eigenvalue_ratio ||
      values[1] / values[2] < config.min_middle_eigenvalue_ratio)
    {
      continue;
    }
    const Eigen::Vector3d normal = eigen.eigenvectors().col(0).normalized();
    const Eigen::Vector3d tangent_a = eigen.eigenvectors().col(2).normalized();
    const Eigen::Vector3d tangent_b = normal.cross(tangent_a).normalized();
    if (tangent_a.allFinite() && tangent_b.allFinite()) {
      tangents[item.first] = {tangent_a, tangent_b};
    }
  }
  result.planar_voxels = tangents.size();

  const std::size_t samples_per_ring = config.add_diagonals ? 8U : 4U;
  const std::size_t ring_count = config.inner_radius_m > 0.0 ? 2U : 1U;
  const std::size_t angular_midpoint_samples = config.angular_midpoint_pairs > 0U ?
    2U * config.angular_midpoint_pairs : (config.add_angular_midpoints ? 8U : 0U);
  const std::size_t samples_per_planar_point =
    samples_per_ring * ring_count + angular_midpoint_samples;
  result.points.reserve(input.size() * (1U + samples_per_planar_point));
  for (const Eigen::Vector3d & point : input) {
    if (!point.allFinite()) {continue;}
    result.points.push_back(point);
    const auto found = tangents.find(detail::voxelKey(point, config.voxel_size_m));
    if (found == tangents.end()) {continue;}
    ++result.planar_input_points;
    const std::array<double, 2> radii {config.radius_m, config.inner_radius_m};
    for (std::size_t ring_index = 0U; ring_index < ring_count; ++ring_index) {
      const double radius = radii[ring_index];
      for (const Eigen::Vector3d & tangent : found->second) {
        result.points.push_back(point - radius * tangent);
        result.points.push_back(point + radius * tangent);
      }
      if (config.add_diagonals) {
        const double diagonal_radius = radius / std::sqrt(2.0);
        for (const double sign_a : {-1.0, 1.0}) {
          for (const double sign_b : {-1.0, 1.0}) {
            result.points.push_back(
              point + diagonal_radius *
              (sign_a * found->second[0] + sign_b * found->second[1]));
          }
        }
      }
    }
    if (config.angular_midpoint_pairs > 0U) {
      constexpr double pi = 3.14159265358979323846;
      const std::array<double, 4> pair_angles {
        pi / 8.0, 5.0 * pi / 8.0, 3.0 * pi / 8.0, 7.0 * pi / 8.0};
      for (std::size_t index = 0U; index < config.angular_midpoint_pairs; ++index) {
        const Eigen::Vector3d offset = config.radius_m *
          (std::cos(pair_angles[index]) * found->second[0] +
          std::sin(pair_angles[index]) * found->second[1]);
        result.points.push_back(point + offset);
        result.points.push_back(point - offset);
      }
    } else if (config.add_angular_midpoints) {
      constexpr double pi = 3.14159265358979323846;
      for (std::size_t index = 0U; index < 8U; ++index) {
        const double angle = pi / 8.0 + static_cast<double>(index) * pi / 4.0;
        result.points.push_back(
          point + config.radius_m *
          (std::cos(angle) * found->second[0] + std::sin(angle) * found->second[1]));
      }
    }
  }
  result.sampled_points = result.planar_input_points * samples_per_planar_point;
  return result;
}

}  // namespace graphslam::ndt_localization

#endif  // GRAPH_BASED_SLAM__NDT_LOCALIZATION_TARGET_HPP_
