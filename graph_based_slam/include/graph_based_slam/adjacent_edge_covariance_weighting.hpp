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

#ifndef GRAPH_BASED_SLAM__ADJACENT_EDGE_COVARIANCE_WEIGHTING_HPP_
#define GRAPH_BASED_SLAM__ADJACENT_EDGE_COVARIANCE_WEIGHTING_HPP_

#include <algorithm>
#include <array>
#include <cmath>

#include <Eigen/Core>  // NOLINT(build/include_order)

#include "graph_based_slam/odometry_covariance_localizability.hpp"

namespace graphslam
{
namespace degeneracy
{

struct AdjacentEdgeCovarianceWeightingConfig
{
  bool enabled {false};
  double degenerate_information_scale {0.25};
  double non_observable_information_scale {0.05};
  LocalizabilityThresholds thresholds {};
};

struct AdjacentEdgeCovarianceWeightingResult
{
  Matrix6d information {Matrix6d::Identity()};
  bool diagnostics_available {false};
  bool applied {false};
};

// Convert the frontend's scale-bearing Hessian covariance into a bounded
// directional modifier for one adjacent pose-graph edge. The caller supplies
// the historical base information matrix, so disabling this path or receiving
// an odometry source without diagnostics preserves the old matrix exactly.
//
// The normalized direction scales deliberately use the Phase-1 categories,
// rather than the absolute Hessian eigenvalues: correspondence count and scene
// density must not silently change the overall backend/frontend balance.
inline AdjacentEdgeCovarianceWeightingResult weightAdjacentEdgeFromCovariance(
  const Matrix6d & base_information,
  const std::array<double, 36> & covariance,
  const AdjacentEdgeCovarianceWeightingConfig & config)
{
  AdjacentEdgeCovarianceWeightingResult result;
  result.information = base_information;
  if (!config.enabled) {
    return result;
  }

  const CovarianceLocalizabilityResult localizability =
    analyzeOdometryCovariance(covariance, config.thresholds);
  result.diagnostics_available = localizability.diagnostics_available;
  if (!localizability.diagnostics_available) {
    return result;
  }

  const double degenerate_scale = std::max(
    0.0, std::min(1.0, config.degenerate_information_scale));
  const double non_observable_scale = std::max(
    0.0, std::min(degenerate_scale, config.non_observable_information_scale));

  Matrix6d directional_scale = Matrix6d::Zero();
  bool has_weak_direction = false;
  for (const DirectionResult & direction : localizability.report.directions) {
    double scale = 1.0;
    if (direction.category == LocalizabilityCategory::DEGENERATE) {
      scale = degenerate_scale;
      has_weak_direction = true;
    } else if (direction.category == LocalizabilityCategory::NON_OBSERVABLE) {
      scale = non_observable_scale;
      has_weak_direction = true;
    }
    directional_scale += scale * direction.eigenvector * direction.eigenvector.transpose();
  }
  if (!has_weak_direction) {
    return result;
  }

  // Congruence with sqrt(base) preserves the caller's legacy translation /
  // rotation balance while applying a full 6-DoF directional modifier.
  Eigen::SelfAdjointEigenSolver<Matrix6d> base_solver(
    0.5 * (base_information + base_information.transpose()));
  if (base_solver.info() != Eigen::Success || base_solver.eigenvalues().minCoeff() < 0.0) {
    return result;
  }
  const Matrix6d base_sqrt = base_solver.eigenvectors() *
    base_solver.eigenvalues().cwiseSqrt().asDiagonal() *
    base_solver.eigenvectors().transpose();
  result.information = base_sqrt * directional_scale * base_sqrt;
  result.information = 0.5 * (result.information + result.information.transpose());
  result.applied = result.information.allFinite();
  if (!result.applied) {
    result.information = base_information;
  }
  return result;
}

}  // namespace degeneracy
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__ADJACENT_EDGE_COVARIANCE_WEIGHTING_HPP_
