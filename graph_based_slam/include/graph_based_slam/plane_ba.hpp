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

#ifndef GRAPH_BASED_SLAM__PLANE_BA_HPP_
#define GRAPH_BASED_SLAM__PLANE_BA_HPP_

// Plane bundle-adjustment problem and deterministic damped-Newton (LM)
// solver over a window of poses (docs/roadmap/v0.7.md Phase 2; design:
// docs/research/map-refinement-clean-room-design.md). The optimizer-level
// degeneracy safeguards are first-class v0.7 scope: gauge fixing of the
// first pose, soft priors of every pose to its input pose (which hold the
// unconstrained directions of rank-deficient scenes), deterministic
// whole-vector step limiting, and a reject-with-report fallback when no
// plane feature is valid. No clocks, no randomness, fixed evaluation
// order: the same inputs produce bitwise-identical refined poses.

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Cholesky>  // NOLINT(build/include_order)
#include <Eigen/Core>  // NOLINT(build/include_order)

#include "graph_based_slam/scatter_eigen_cost.hpp"
#include "graph_based_slam/se3_lie.hpp"

namespace graphslam
{
namespace map_refinement
{

struct PlaneFeature
{
  // Observations are expected in ascending pose_index order.
  std::vector<PlaneFeatureObservation> observations;
};

struct PlaneBaConfig
{
  PlaneCostConfig cost;
  int max_iterations {30};
  double initial_lambda {1e-4};
  double lambda_up_factor {10.0};
  double lambda_down_factor {0.3};
  double max_lambda {1e12};
  double min_relative_cost_decrease {1e-9};
  // Per-pose limits; the WHOLE delta is scaled by the worst ratio so the
  // step direction is preserved deterministically.
  double max_step_translation {0.25};
  double max_step_rotation_rad {0.052};
  // Gauge: hard-fix pose 0 of the window (solve the reduced system).
  bool fix_first_pose {true};
  // Soft prior of every pose to its input pose; <= 0 disables priors.
  // Gauss-Newton with a unit twist Jacobian (exact as the residual -> 0).
  double prior_translation_sigma {0.30};
  double prior_rotation_sigma_rad {0.035};
};

struct PlaneBaResult
{
  std::vector<Eigen::Matrix4d> poses;
  bool converged {false};
  bool improved {false};
  int iterations {0};
  int accepted_steps {0};
  double initial_cost {0.0};
  double final_cost {0.0};
  int valid_features {0};
  int invalid_features {0};
  std::string termination;
};

namespace detail
{

inline bool priorsEnabled(const PlaneBaConfig & config)
{
  return config.prior_translation_sigma > 0.0 && config.prior_rotation_sigma_rad > 0.0;
}

inline double priorWeight(const PlaneBaConfig & config, int axis)
{
  const double sigma =
    axis < 3 ? config.prior_translation_sigma : config.prior_rotation_sigma_rad;
  return 1.0 / (sigma * sigma);
}

// Prior residual twist of pose i against its input pose.
inline Vector6d priorResidual(
  const Eigen::Matrix4d & pose, const Eigen::Matrix4d & input_pose)
{
  Eigen::Matrix4d input_inverse = Eigen::Matrix4d::Identity();
  const Eigen::Matrix3d rotation = input_pose.block<3, 3>(0, 0);
  input_inverse.block<3, 3>(0, 0) = rotation.transpose();
  input_inverse.block<3, 1>(0, 3) = -rotation.transpose() * input_pose.block<3, 1>(0, 3);
  return logSe3(pose * input_inverse);
}

struct CostEvaluation
{
  double total_cost {0.0};
  int valid_features {0};
  int invalid_features {0};
};

inline CostEvaluation evaluateTotalCost(
  const std::vector<PlaneFeature> & features,
  const std::vector<Eigen::Matrix4d> & poses,
  const std::vector<Eigen::Matrix4d> & input_poses,
  const PlaneBaConfig & config)
{
  CostEvaluation evaluation;
  const int pose_count = static_cast<int>(poses.size());
  for (size_t f = 0; f < features.size(); ++f) {
    const PlaneCostResult result =
      evaluatePlaneCost(features[f].observations, poses, pose_count, config.cost);
    if (!result.valid) {
      ++evaluation.invalid_features;
      continue;
    }
    ++evaluation.valid_features;
    evaluation.total_cost += result.cost;
  }
  if (priorsEnabled(config)) {
    const int first = config.fix_first_pose ? 1 : 0;
    for (int i = first; i < pose_count; ++i) {
      const Vector6d residual = priorResidual(poses[i], input_poses[i]);
      for (int axis = 0; axis < 6; ++axis) {
        evaluation.total_cost +=
          priorWeight(config, axis) * residual(axis) * residual(axis);
      }
    }
  }
  return evaluation;
}

// Accumulate gradient and Hessian of the full problem at the given poses.
// Returns false when no plane feature is valid.
inline bool assembleSystem(
  const std::vector<PlaneFeature> & features,
  const std::vector<Eigen::Matrix4d> & poses,
  const std::vector<Eigen::Matrix4d> & input_poses,
  const PlaneBaConfig & config,
  Eigen::VectorXd * gradient,
  Eigen::MatrixXd * hessian)
{
  const int pose_count = static_cast<int>(poses.size());
  const int size = 6 * pose_count;
  gradient->setZero(size);
  hessian->setZero(size, size);
  bool any_valid = false;
  for (size_t f = 0; f < features.size(); ++f) {
    const PlaneCostResult result =
      evaluatePlaneCost(features[f].observations, poses, pose_count, config.cost);
    if (!result.valid) {
      continue;
    }
    any_valid = true;
    *gradient += result.gradient;
    *hessian += result.hessian;
  }
  if (!any_valid) {
    return false;
  }
  if (priorsEnabled(config)) {
    const int first = config.fix_first_pose ? 1 : 0;
    for (int i = first; i < pose_count; ++i) {
      const Vector6d residual = priorResidual(poses[i], input_poses[i]);
      for (int axis = 0; axis < 6; ++axis) {
        const double weight = priorWeight(config, axis);
        (*gradient)(6 * i + axis) += 2.0 * weight * residual(axis);
        (*hessian)(6 * i + axis, 6 * i + axis) += 2.0 * weight;
      }
    }
  }
  return true;
}

// Copy the free-variable blocks (everything except pose 0 when fixed)
// into a reduced system, in ascending variable order.
inline void reduceSystem(
  const Eigen::VectorXd & gradient,
  const Eigen::MatrixXd & hessian,
  int first_free_variable,
  Eigen::VectorXd * reduced_gradient,
  Eigen::MatrixXd * reduced_hessian)
{
  const int reduced_size = static_cast<int>(gradient.size()) - first_free_variable;
  *reduced_gradient = gradient.tail(reduced_size);
  *reduced_hessian =
    hessian.block(first_free_variable, first_free_variable, reduced_size, reduced_size);
}

// Scale the whole step so no pose exceeds the per-pose limits.
inline void limitStep(
  const PlaneBaConfig & config, int free_pose_count, Eigen::VectorXd * delta)
{
  double worst_ratio = 1.0;
  for (int p = 0; p < free_pose_count; ++p) {
    const double translation_norm = delta->segment<3>(6 * p).norm();
    const double rotation_norm = delta->segment<3>(6 * p + 3).norm();
    if (translation_norm > config.max_step_translation) {
      worst_ratio = std::max(worst_ratio, translation_norm / config.max_step_translation);
    }
    if (rotation_norm > config.max_step_rotation_rad) {
      worst_ratio = std::max(worst_ratio, rotation_norm / config.max_step_rotation_rad);
    }
  }
  if (worst_ratio > 1.0) {
    *delta /= worst_ratio;
  }
}

}  // namespace detail

inline PlaneBaResult solvePlaneBa(
  const std::vector<PlaneFeature> & features,
  const std::vector<Eigen::Matrix4d> & initial_poses,
  const PlaneBaConfig & config)
{
  PlaneBaResult result;
  result.poses = initial_poses;
  const int pose_count = static_cast<int>(initial_poses.size());
  if (pose_count == 0) {
    result.termination = "no_valid_features";
    return result;
  }

  const detail::CostEvaluation initial_evaluation =
    detail::evaluateTotalCost(features, result.poses, initial_poses, config);
  result.initial_cost = initial_evaluation.total_cost;
  result.final_cost = initial_evaluation.total_cost;
  result.valid_features = initial_evaluation.valid_features;
  result.invalid_features = initial_evaluation.invalid_features;
  if (initial_evaluation.valid_features == 0) {
    result.termination = "no_valid_features";
    return result;
  }

  const int first_free_variable = config.fix_first_pose ? 6 : 0;
  const int free_pose_count = config.fix_first_pose ? pose_count - 1 : pose_count;
  if (free_pose_count <= 0) {
    result.termination = "converged";
    result.converged = true;
    return result;
  }

  double lambda = config.initial_lambda;
  double current_cost = initial_evaluation.total_cost;
  result.termination = "max_iterations";

  for (int iteration = 0; iteration < config.max_iterations; ++iteration) {
    result.iterations = iteration + 1;

    Eigen::VectorXd gradient;
    Eigen::MatrixXd hessian;
    if (!detail::assembleSystem(
        features, result.poses, initial_poses, config, &gradient, &hessian))
    {
      result.termination = "no_valid_features";
      break;
    }
    Eigen::VectorXd reduced_gradient;
    Eigen::MatrixXd reduced_hessian;
    detail::reduceSystem(
      gradient, hessian, first_free_variable, &reduced_gradient, &reduced_hessian);

    bool accepted = false;
    while (lambda <= config.max_lambda) {
      Eigen::MatrixXd damped = reduced_hessian;
      for (int i = 0; i < damped.rows(); ++i) {
        damped(i, i) += lambda * std::max(std::abs(reduced_hessian(i, i)), 1e-12);
      }
      Eigen::VectorXd delta = Eigen::LDLT<Eigen::MatrixXd>(damped).solve(-reduced_gradient);
      if (!delta.allFinite()) {
        lambda *= config.lambda_up_factor;
        continue;
      }
      detail::limitStep(config, free_pose_count, &delta);

      std::vector<Eigen::Matrix4d> candidate = result.poses;
      for (int p = 0; p < free_pose_count; ++p) {
        const Vector6d twist = delta.segment<6>(6 * p);
        const int pose_index = p + (config.fix_first_pose ? 1 : 0);
        candidate[pose_index] = leftPerturb(twist, candidate[pose_index]);
      }
      const detail::CostEvaluation candidate_evaluation =
        detail::evaluateTotalCost(features, candidate, initial_poses, config);
      bool candidate_finite = std::isfinite(candidate_evaluation.total_cost);
      for (size_t p = 0; p < candidate.size() && candidate_finite; ++p) {
        candidate_finite = candidate[p].allFinite();
      }
      if (candidate_finite && candidate_evaluation.total_cost < current_cost) {
        const double decrease = current_cost - candidate_evaluation.total_cost;
        const double relative_decrease =
          decrease / std::max(std::abs(current_cost), 1e-300);
        result.poses = candidate;
        current_cost = candidate_evaluation.total_cost;
        result.valid_features = candidate_evaluation.valid_features;
        result.invalid_features = candidate_evaluation.invalid_features;
        ++result.accepted_steps;
        lambda = std::max(lambda * config.lambda_down_factor, 1e-12);
        accepted = true;
        if (relative_decrease < config.min_relative_cost_decrease) {
          result.termination = "converged";
          result.converged = true;
        }
        break;
      }
      lambda *= config.lambda_up_factor;
    }
    if (!accepted) {
      result.termination = "max_lambda";
      break;
    }
    if (result.converged) {
      break;
    }
  }

  result.final_cost = current_cost;
  result.improved = result.final_cost < result.initial_cost;
  return result;
}

}  // namespace map_refinement
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__PLANE_BA_HPP_
