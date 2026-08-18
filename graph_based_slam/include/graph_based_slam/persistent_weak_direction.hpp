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

#ifndef GRAPH_BASED_SLAM__PERSISTENT_WEAK_DIRECTION_HPP_
#define GRAPH_BASED_SLAM__PERSISTENT_WEAK_DIRECTION_HPP_

// ROS-free, stateful gate for the v0.8 degeneracy-aware solve. A single weak
// Hessian direction is not enough evidence to alter an ICP update: narrow-FOV
// sensors can produce short-lived weak eigendirections that rotate from scan
// to scan. This tracker confirms only an isolated, translation-dominant weak
// axis that remains aligned in the world-frame tangent space for consecutive
// scans. Repeated NON_OBSERVABLE eigenspaces are deliberately rejected here;
// their individual eigenvectors are basis-dependent and the Phase 2 evidence
// identified freezing that subspace as the dominant MID-360 failure mode.

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "graph_based_slam/localizability_analysis.hpp"

namespace graphslam
{
namespace degeneracy
{

struct PersistentWeakDirectionConfig
{
  std::size_t min_consecutive_scans {3};
  double min_absolute_cosine {0.98};
  double min_translation_fraction {0.99};
};

struct PersistentWeakDirectionState
{
  Vector6d axis {Vector6d::Zero()};
  std::size_t consecutive_scans {0};
  double matched_absolute_cosine {0.0};
  bool candidate_available {false};
  bool confirmed {false};
};

class PersistentWeakDirectionTracker
{
public:
  explicit PersistentWeakDirectionTracker(
    const PersistentWeakDirectionConfig & config = PersistentWeakDirectionConfig())
  : config_(sanitized(config))
  {
  }

  PersistentWeakDirectionState observe(const LocalizabilityReport & report)
  {
    const DirectionResult * candidate = selectCandidate(report);
    if (candidate == nullptr) {
      reset();
      return state_;
    }

    Vector6d axis = candidate->eigenvector.normalized();
    double matched_cosine = 0.0;
    if (state_.candidate_available) {
      matched_cosine = std::abs(state_.axis.dot(axis));
    }

    if (state_.candidate_available && matched_cosine >= config_.min_absolute_cosine) {
      // Eigenvector sign has no physical meaning. Keep the tracked sign stable
      // so downstream diagnostics and comparisons remain deterministic.
      if (state_.axis.dot(axis) < 0.0) {
        axis = -axis;
      }
      ++state_.consecutive_scans;
      state_.matched_absolute_cosine = matched_cosine;
    } else {
      state_.consecutive_scans = 1;
      state_.matched_absolute_cosine = 0.0;
    }

    state_.axis = axis;
    state_.candidate_available = true;
    state_.confirmed = state_.consecutive_scans >= config_.min_consecutive_scans;
    return state_;
  }

  void reset()
  {
    state_ = PersistentWeakDirectionState();
  }

  const PersistentWeakDirectionState & state() const
  {
    return state_;
  }

private:
  static PersistentWeakDirectionConfig sanitized(PersistentWeakDirectionConfig config)
  {
    config.min_consecutive_scans = std::max<std::size_t>(1, config.min_consecutive_scans);
    config.min_absolute_cosine = std::max(0.0, std::min(1.0, config.min_absolute_cosine));
    config.min_translation_fraction =
      std::max(0.0, std::min(1.0, config.min_translation_fraction));
    return config;
  }

  const DirectionResult * selectCandidate(const LocalizabilityReport & report) const
  {
    const DirectionResult * selected = nullptr;
    double selected_match = -1.0;
    for (const DirectionResult & direction : report.directions) {
      if (direction.category != LocalizabilityCategory::DEGENERATE) {
        continue;
      }
      const double squared_norm = direction.eigenvector.squaredNorm();
      if (!(squared_norm > 0.0) || !std::isfinite(squared_norm)) {
        continue;
      }
      const double translation_fraction =
        direction.eigenvector.head<3>().squaredNorm() / squared_norm;
      if (translation_fraction < config_.min_translation_fraction) {
        continue;
      }

      // When more than one isolated candidate exists, continue the axis that
      // best matches the existing track. With no prior track, ascending
      // eigenvalue order makes the first candidate deterministic.
      const double match = state_.candidate_available ?
        std::abs(state_.axis.dot(direction.eigenvector.normalized())) : 0.0;
      if (selected == nullptr || match > selected_match) {
        selected = &direction;
        selected_match = match;
      }
    }
    return selected;
  }

  PersistentWeakDirectionConfig config_ {};
  PersistentWeakDirectionState state_ {};
};

}  // namespace degeneracy
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__PERSISTENT_WEAK_DIRECTION_HPP_
