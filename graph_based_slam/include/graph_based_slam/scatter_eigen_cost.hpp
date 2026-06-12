// Copyright 2026 Sasaki
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
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
//

#ifndef GRAPH_BASED_SLAM__SCATTER_EIGEN_COST_HPP_
#define GRAPH_BASED_SLAM__SCATTER_EIGEN_COST_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>  // NOLINT(build/include_order)
#include <Eigen/Eigenvalues>  // NOLINT(build/include_order)

#include "graph_based_slam/point_cluster.hpp"
#include "graph_based_slam/se3_lie.hpp"

namespace graphslam
{
namespace map_refinement
{

struct PlaneFeatureObservation
{
  int pose_index {-1};            // index into the pose vector of the BA problem
  PointCluster local_cluster;     // points of this feature in that pose's local frame
};

struct PlaneCostConfig
{
  double eigen_gap_min {1e-9};    // require mu_1 - mu_0 > this, else feature invalid
  double min_total_points {6.0};  // require n >= this
};

struct PlaneCostResult
{
  bool valid {false};
  double cost {0.0};                       // mu_0 of the combined scatter
  double normalized_cost {0.0};            // mu_0 / n
  Eigen::VectorXd gradient;                // size 6 * pose_count (the BA problem's full size)
  Eigen::MatrixXd hessian;                 // (6 pc) x (6 pc), symmetric
  Eigen::Vector3d normal {Eigen::Vector3d::Zero()};  // u_0, sign-canonicalized
  double eigen_gap {0.0};
};

namespace detail
{

struct TransformedObservation
{
  int pose_index {-1};
  Eigen::Matrix4d h {Eigen::Matrix4d::Zero()};
};

struct PlaneDerivativeCache
{
  int pose_index {-1};
  int axis {0};
  Eigen::Matrix3d a {Eigen::Matrix3d::Zero()};
  Eigen::Vector3d s {Eigen::Vector3d::Zero()};
};

inline Eigen::Matrix3d extractQ(const Eigen::Matrix4d & h)
{
  return h.block<3, 3>(0, 0);
}

inline Eigen::Vector3d extractS(const Eigen::Matrix4d & h)
{
  return h.block<3, 1>(0, 3);
}

inline double extractN(const Eigen::Matrix4d & h)
{
  return h(3, 3);
}

inline Eigen::Matrix3d scatterMatrix(
  const Eigen::Matrix3d & q,
  const Eigen::Vector3d & s,
  double n)
{
  return q - s * s.transpose() / n;
}

inline Eigen::Vector3d canonicalNormal(const Eigen::Matrix3d & eigenvectors)
{
  Eigen::Vector3d normal = eigenvectors.col(0);
  int largest_index = 0;
  double largest_abs = std::abs(normal(0));

  for (int i = 1; i < 3; ++i) {
    const double component_abs = std::abs(normal(i));
    if (component_abs > largest_abs) {
      largest_abs = component_abs;
      largest_index = i;
    }
  }

  if (normal(largest_index) < 0.0) {
    normal = -normal;
  }

  return normal;
}

inline double quadraticForm(
  const Eigen::Matrix3d & matrix,
  const Eigen::Vector3d & vector)
{
  return vector.dot(matrix * vector);
}

inline Eigen::Matrix3d firstScatterDerivative(
  const Eigen::Matrix4d & h_derivative,
  const Eigen::Vector3d & s,
  double n,
  Eigen::Vector3d * s_derivative)
{
  const Eigen::Matrix3d q_derivative = extractQ(h_derivative);
  *s_derivative = extractS(h_derivative);
  return q_derivative -
         (*s_derivative * s.transpose() + s * s_derivative->transpose()) / n;
}

inline Eigen::Matrix3d secondScatterDerivative(
  const Eigen::Matrix4d & h_derivative,
  const Eigen::Vector3d & s,
  double n,
  const Eigen::Vector3d & s_a,
  const Eigen::Vector3d & s_b)
{
  const Eigen::Matrix3d q_derivative = extractQ(h_derivative);
  const Eigen::Vector3d s_derivative = extractS(h_derivative);
  return q_derivative -
         (s_derivative * s.transpose() + s * s_derivative.transpose() +
         s_a * s_b.transpose() + s_b * s_a.transpose()) / n;
}

inline Eigen::Matrix3d mixedPoseScatterDerivative(
  const Eigen::Vector3d & s_a,
  const Eigen::Vector3d & s_b,
  double n)
{
  return -(s_a * s_b.transpose() + s_b * s_a.transpose()) / n;
}

inline double eigenvectorCurvatureTerm(
  const Eigen::Matrix3d & a,
  const Eigen::Matrix3d & b,
  const Eigen::Matrix3d & eigenvectors,
  const Eigen::Vector3d & eigenvalues)
{
  const Eigen::Vector3d u0 = eigenvectors.col(0);
  double term = 0.0;

  for (int m = 1; m < 3; ++m) {
    const Eigen::Vector3d um = eigenvectors.col(m);
    const double a_projection = um.dot(a * u0);
    const double b_projection = um.dot(b * u0);
    term += 2.0 * a_projection * b_projection / (eigenvalues(0) - eigenvalues(m));
  }

  return term;
}

}  // namespace detail

/// Exact gradient/Hessian of the eliminated plane cost lambda_min(scatter) for
/// one plane feature observed by multiple poses, in point-cluster coordinates.
/// This is the v0.7 Phase 2 plane-BA cost layer described by
/// docs/roadmap/v0.7.md and docs/research/map-refinement-clean-room-design.md.
/// Accumulation is deterministic: combined clusters are summed in observation
/// order, then derivatives are assembled in fixed ascending (pose, axis) order.
/// No robust loss is applied here; rho(mu_0 / n) belongs to the problem layer.
inline PlaneCostResult evaluatePlaneCost(
  const std::vector<PlaneFeatureObservation> & observations,
  const std::vector<Eigen::Matrix4d> & poses,
  int pose_count,
  const PlaneCostConfig & config)
{
  PlaneCostResult result;
  const int safe_pose_count = std::max(0, pose_count);
  const int variable_count = 6 * safe_pose_count;
  result.gradient = Eigen::VectorXd::Zero(variable_count);
  result.hessian = Eigen::MatrixXd::Zero(variable_count, variable_count);

  if (pose_count < 0) {
    return result;
  }

  Eigen::Matrix4d combined_h = Eigen::Matrix4d::Zero();
  std::vector<detail::TransformedObservation> transformed_observations;
  transformed_observations.reserve(observations.size());
  std::vector<char> pose_observed(static_cast<std::size_t>(pose_count), 0);

  for (std::size_t i = 0; i < observations.size(); ++i) {
    const int pose_index = observations[i].pose_index;
    if (pose_index < 0 || pose_index >= pose_count) {
      return result;
    }

    const std::size_t pose_offset = static_cast<std::size_t>(pose_index);
    if (pose_offset >= poses.size()) {
      return result;
    }

    const PointCluster transformed_cluster =
      observations[i].local_cluster.transformed(poses[pose_offset]);

    detail::TransformedObservation transformed_observation;
    transformed_observation.pose_index = pose_index;
    transformed_observation.h = transformed_cluster.h;
    transformed_observations.push_back(transformed_observation);

    combined_h += transformed_observation.h;
    pose_observed[pose_offset] = 1;
  }

  const double n = detail::extractN(combined_h);
  if (n <= 0.0) {
    return result;
  }

  const Eigen::Matrix3d q = detail::extractQ(combined_h);
  const Eigen::Vector3d s = detail::extractS(combined_h);
  const Eigen::Matrix3d scatter = detail::scatterMatrix(q, s, n);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(scatter);
  if (solver.info() != Eigen::Success) {
    return result;
  }

  const Eigen::Vector3d eigenvalues = solver.eigenvalues();
  const Eigen::Matrix3d eigenvectors = solver.eigenvectors();
  const Eigen::Vector3d u0 = eigenvectors.col(0);

  result.cost = eigenvalues(0);
  result.normalized_cost = result.cost / n;
  result.normal = detail::canonicalNormal(eigenvectors);
  result.eigen_gap = eigenvalues(1) - eigenvalues(0);

  if (n < config.min_total_points || result.eigen_gap <= config.eigen_gap_min) {
    return result;
  }

  result.valid = true;

  std::vector<int> observed_pose_indices;
  observed_pose_indices.reserve(static_cast<std::size_t>(pose_count));
  for (int pose_index = 0; pose_index < pose_count; ++pose_index) {
    if (pose_observed[static_cast<std::size_t>(pose_index)] != 0) {
      observed_pose_indices.push_back(pose_index);
    }
  }

  std::vector<detail::PlaneDerivativeCache> derivatives;
  derivatives.reserve(observed_pose_indices.size() * 6U);

  for (std::size_t i = 0; i < observed_pose_indices.size(); ++i) {
    const int pose_index = observed_pose_indices[i];

    for (int axis = 0; axis < 6; ++axis) {
      Eigen::Matrix4d h_derivative = Eigen::Matrix4d::Zero();

      for (std::size_t j = 0; j < transformed_observations.size(); ++j) {
        if (transformed_observations[j].pose_index == pose_index) {
          h_derivative += clusterFirstDerivative(transformed_observations[j].h, axis);
        }
      }

      detail::PlaneDerivativeCache derivative;
      derivative.pose_index = pose_index;
      derivative.axis = axis;
      derivative.a = detail::firstScatterDerivative(h_derivative, s, n, &derivative.s);

      const int variable_index = 6 * pose_index + axis;
      result.gradient(variable_index) = detail::quadraticForm(derivative.a, u0);
      derivatives.push_back(derivative);
    }
  }

  for (std::size_t i = 0; i < derivatives.size(); ++i) {
    const detail::PlaneDerivativeCache & row_derivative = derivatives[i];

    for (std::size_t j = i; j < derivatives.size(); ++j) {
      const detail::PlaneDerivativeCache & column_derivative = derivatives[j];
      Eigen::Matrix3d second_derivative = Eigen::Matrix3d::Zero();

      if (row_derivative.pose_index == column_derivative.pose_index) {
        Eigen::Matrix4d h_second_derivative = Eigen::Matrix4d::Zero();

        for (std::size_t k = 0; k < transformed_observations.size(); ++k) {
          if (transformed_observations[k].pose_index == row_derivative.pose_index) {
            h_second_derivative += clusterSecondDerivative(
              transformed_observations[k].h,
              row_derivative.axis,
              column_derivative.axis);
          }
        }

        second_derivative = detail::secondScatterDerivative(
          h_second_derivative,
          s,
          n,
          row_derivative.s,
          column_derivative.s);
      } else {
        second_derivative = detail::mixedPoseScatterDerivative(
          row_derivative.s,
          column_derivative.s,
          n);
      }

      const double hessian_value =
        detail::quadraticForm(second_derivative, u0) +
        detail::eigenvectorCurvatureTerm(
        row_derivative.a,
        column_derivative.a,
        eigenvectors,
        eigenvalues);

      const int row = 6 * row_derivative.pose_index + row_derivative.axis;
      const int column = 6 * column_derivative.pose_index + column_derivative.axis;
      result.hessian(row, column) = hessian_value;
      result.hessian(column, row) = hessian_value;
    }
  }

  return result;
}

}  // namespace map_refinement
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__SCATTER_EIGEN_COST_HPP_
