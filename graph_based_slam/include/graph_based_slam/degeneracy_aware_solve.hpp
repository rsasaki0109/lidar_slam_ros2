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
//    copyright notice, this list of conditions and the following disclaimer
//    in the documentation and/or other materials provided with the distribution.
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

#ifndef GRAPH_BASED_SLAM__DEGENERACY_AWARE_SOLVE_HPP_
#define GRAPH_BASED_SLAM__DEGENERACY_AWARE_SOLVE_HPP_

// Pure, ROS-free Phase 2 solution remapping. The six eigendirections are
// classified by localizability_analysis.hpp. Well-conditioned directions
// retain the Gauss-Newton solution, isolated degenerate directions blend
// toward a motion-prior correction, and a repeated non-observable subspace
// is frozen. Defaults make the prior authoritative only where geometry is
// weak; callers must still opt into using this solve instead of their legacy
// linear solver.

#include <algorithm>
#include <cmath>

#include "graph_based_slam/localizability_analysis.hpp"
#include "graph_based_slam/persistent_weak_direction.hpp"

namespace graphslam
{
namespace degeneracy
{

struct DegeneracyAwareSolveConfig
{
  DegeneracyAwareSolveConfig()
  {
    // Intervention is deliberately more selective than the Phase 1
    // diagnostic classifier: only the near-null tail is remapped.
    thresholds.well_conditioned_ratio = 1.0e-6;
  }

  double degenerate_prior_weight {0.25};
  LocalizabilityThresholds thresholds {};
  bool require_persistent_direction {false};
  double persistent_direction_min_absolute_cosine {0.98};
  PersistentWeakDirectionState persistent_direction {};
};

struct DegeneracyAwareSolveResult
{
  Vector6d update {Vector6d::Zero()};
  LocalizabilityReport localizability {};
  bool used_prior {false};
  bool intervention_applied {false};
  bool valid {false};
};

inline DegeneracyAwareSolveResult solveDegeneracyAware(
  const Matrix6d & h,
  const Vector6d & b,
  const Vector6d & prior_update,
  const DegeneracyAwareSolveConfig & config = DegeneracyAwareSolveConfig())
{
  DegeneracyAwareSolveResult result;
  if (!h.allFinite() || !b.allFinite() || !prior_update.allFinite()) {
    return result;
  }
  result.localizability = analyzeLocalizability(h, b, config.thresholds);

  const double prior_weight = std::max(0.0, std::min(1.0, config.degenerate_prior_weight));
  constexpr double kEigenvalueFloor = 1.0e-12;

  if (config.require_persistent_direction) {
    // Preserve the legacy solve exactly until a weak world-frame axis has
    // persisted for the configured number of scans. Once confirmed, replace
    // only that isolated weak component; all other directions, including a
    // repeated NON_OBSERVABLE subspace, retain the legacy geometric update.
    // This is the fail-safe policy motivated by the MID-360 Phase 2 failure.
    result.update = h.ldlt().solve(-b);
    if (!result.update.allFinite()) {
      result.update.setZero();
      return result;
    }

    if (config.persistent_direction.confirmed) {
      const double min_cosine = std::max(
        0.0, std::min(1.0, config.persistent_direction_min_absolute_cosine));
      for (const DirectionResult & direction : result.localizability.directions) {
        if (direction.category != LocalizabilityCategory::DEGENERATE) {
          continue;
        }
        const double match = std::abs(
          direction.eigenvector.dot(config.persistent_direction.axis));
        if (match < min_cosine) {
          continue;
        }

        const Vector6d & axis = direction.eigenvector;
        const double legacy_component = axis.dot(result.update);
        double geometric_component = 0.0;
        if (direction.eigenvalue > kEigenvalueFloor) {
          geometric_component = -axis.dot(b) / direction.eigenvalue;
        }
        const double blended_component =
          (1.0 - prior_weight) * geometric_component + prior_weight * axis.dot(prior_update);
        result.update += (blended_component - legacy_component) * axis;
        result.used_prior = prior_weight > 0.0;
        result.intervention_applied = true;
        break;
      }
    }

    result.valid = result.update.allFinite();
    if (!result.valid) {
      result.update.setZero();
      result.used_prior = false;
      result.intervention_applied = false;
    }
    return result;
  }

  for (const DirectionResult & direction : result.localizability.directions) {
    const Vector6d & axis = direction.eigenvector;
    double component = 0.0;
    if (
      direction.category == LocalizabilityCategory::WELL_CONDITIONED &&
      direction.eigenvalue > kEigenvalueFloor)
    {
      component = -axis.dot(b) / direction.eigenvalue;
    } else if (direction.category == LocalizabilityCategory::DEGENERATE) {
      double geometric_component = 0.0;
      if (direction.eigenvalue > kEigenvalueFloor) {
        geometric_component = -axis.dot(b) / direction.eigenvalue;
      }
      component =
        (1.0 - prior_weight) * geometric_component + prior_weight * axis.dot(prior_update);
      result.used_prior = result.used_prior || prior_weight > 0.0;
      result.intervention_applied = true;
    }
    // A NON_OBSERVABLE eigenspace has no stable individual axes. Its
    // update remains zero instead of injecting an arbitrary basis-dependent
    // component from either the geometric solve or the prior.
    result.update += component * axis;
  }

  result.valid = result.update.allFinite();
  if (!result.valid) {
    result.update.setZero();
    result.used_prior = false;
    result.intervention_applied = false;
  }
  return result;
}

}  // namespace degeneracy
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__DEGENERACY_AWARE_SOLVE_HPP_
