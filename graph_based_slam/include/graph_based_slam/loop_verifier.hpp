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

#ifndef GRAPH_BASED_SLAM__LOOP_VERIFIER_HPP_
#define GRAPH_BASED_SLAM__LOOP_VERIFIER_HPP_

// Pure decision logic for the per-candidate verification stage of
// searchLoopForLatest: the registration initial guess, the correction
// magnitudes, the acceptance gates and the best-candidate selection.
// The heavy I/O (submap aggregation, voxel filtering, NDT/GICP, 3D-BBS)
// stays in the ROS component; everything that decides what those results
// mean lives here so the offline BackendCore can reuse it unchanged
// (docs/roadmap/v0.6.md, Phase 2).

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Core>  // NOLINT(build/include_order)
#include <Eigen/Geometry>  // NOLINT(build/include_order)

namespace graphslam
{
namespace loop_verifier
{

struct LoopCandidate
{
  enum class Source
  {
    DISTANCE,
    SCAN_CONTEXT,
    BEV_DESCRIPTOR,
    SOLID_DESCRIPTOR,
    TRIANGLE_DESCRIPTOR
  };

  int index {-1};
  double selection_metric {std::numeric_limits<double>::max()};
  Source source {Source::DISTANCE};
  double yaw_rad {0.0};
  // Recovered SE(3) from the descriptor that proposed this candidate
  // (currently only triangle). Identity unless populated. Used as the NDT
  // initial guess instead of the pose-derived guess when source matches.
  Eigen::Matrix4f relative_transform {Eigen::Matrix4f::Identity()};
  bool has_relative_transform {false};
};

struct LoopCandidateResult
{
  bool valid {false};
  int index {-1};
  double selection_metric {std::numeric_limits<double>::max()};
  double fitness_score {std::numeric_limits<double>::max()};
  double travel_distance {0.0};
  double euclidean_distance {0.0};
  double translation_delta_m {0.0};
  double rotation_delta_deg {0.0};
  double overlap_ratio {0.0};
  double reverse_overlap_ratio {0.0};
  double mutual_overlap_ratio {0.0};
  double support_rmse_m {0.0};
  double support_p90_m {0.0};
  LoopCandidate::Source source {LoopCandidate::Source::DISTANCE};
  bool used_3d_bbs {false};
  double three_d_bbs_score_percentage {0.0};
  double three_d_bbs_elapsed_msec {0.0};
  Eigen::Matrix4f final_transformation {Eigen::Matrix4f::Identity()};
};

inline const char * sourceName(LoopCandidate::Source source)
{
  switch (source) {
    case LoopCandidate::Source::SCAN_CONTEXT:
      return "scan_context";
    case LoopCandidate::Source::BEV_DESCRIPTOR:
      return "bev_descriptor";
    case LoopCandidate::Source::SOLID_DESCRIPTOR:
      return "solid_descriptor";
    case LoopCandidate::Source::TRIANGLE_DESCRIPTOR:
      return "triangle_descriptor";
    case LoopCandidate::Source::DISTANCE:
    default:
      return "distance";
  }
}

struct RegistrationDelta
{
  double translation_m {0.0};
  double rotation_deg {0.0};
};

// Magnitude of the correction a converged registration applied on top of
// the stored poses. The rotation angle comes from the trace identity with
// the cosine clamped so float round-off near identity cannot produce NaN.
inline RegistrationDelta computeRegistrationDelta(const Eigen::Matrix4f & final_transformation)
{
  RegistrationDelta delta;
  const Eigen::Vector3f translation = final_transformation.block<3, 1>(0, 3);
  delta.translation_m = translation.cast<double>().norm();
  const Eigen::Matrix3f rotation = final_transformation.block<3, 3>(0, 0);
  const double trace = static_cast<double>(rotation.trace());
  const double cos_theta = std::max(-1.0, std::min(1.0, 0.5 * (trace - 1.0)));
  delta.rotation_deg = std::acos(cos_theta) * 180.0 / M_PI;
  return delta;
}

// The initial guess maps latest-submap world points into the candidate's
// neighborhood: candidate * latest^-1, optionally refined by a descriptor
// yaw hint. A triangle-recovered SE(3) maps latest-submap-local points to
// candidate-submap-local points, so it is chained in between the two poses
// to recover the world-frame guess.
inline Eigen::Matrix4f computeInitialGuess(
  const Eigen::Affine3d & candidate_affine,
  const Eigen::Affine3d & latest_affine,
  const LoopCandidate & candidate)
{
  if (
    candidate.source == LoopCandidate::Source::TRIANGLE_DESCRIPTOR &&
    candidate.has_relative_transform)
  {
    return (candidate_affine.matrix() *
           candidate.relative_transform.cast<double>() *
           latest_affine.inverse().matrix()).cast<float>();
  }
  if (std::abs(candidate.yaw_rad) > 1e-6) {
    Eigen::Affine3d yaw_correction = Eigen::Affine3d::Identity();
    yaw_correction.rotate(Eigen::AngleAxisd(candidate.yaw_rad, Eigen::Vector3d::UnitZ()));
    return (candidate_affine.matrix() * yaw_correction.matrix() *
           latest_affine.inverse().matrix()).cast<float>();
  }
  return (candidate_affine.matrix() * latest_affine.inverse().matrix()).cast<float>();
}

// DISTANCE candidates historically align without a guess (their stored
// poses are already close); every descriptor-sourced candidate, and any
// candidate re-localized by 3D-BBS, supplies one.
inline bool shouldUseInitialGuess(LoopCandidate::Source source, bool used_3d_bbs)
{
  return source != LoopCandidate::Source::DISTANCE || used_3d_bbs;
}

struct GateConfig
{
  double generic_score_threshold {0.0};
  // ScanContext candidates may use their own fitness threshold; a value
  // <= 0 falls back to the generic one.
  double scan_context_score_threshold {0.0};
  double max_translation_m {0.0};
  double max_rotation_deg {0.0};
  // Descriptor-sourced candidates (TRIANGLE / SCAN_CONTEXT / BEV / SOLID)
  // already passed a place-recognition gate, so they can accept a larger
  // correction when the operator opts in (> 0). DISTANCE candidates (close
  // in stored pose) always keep the strict generic cap.
  double max_translation_descriptor_m {0.0};
  double max_rotation_descriptor_deg {0.0};
  // Fraction of aligned source points that must have a target neighbor
  // within overlap_max_distance_m. A non-positive ratio disables the gate.
  double min_overlap_ratio {0.0};
  // Optional relaxed source-overlap threshold for registrations applying a
  // large translational correction (e.g. a narrow-FOV sensor closing drift).
  double min_overlap_ratio_large_correction {0.0};
  double overlap_large_correction_translation_m {0.0};
  double overlap_max_distance_m {0.5};
};

enum class GateRejection
{
  NONE,
  FITNESS,
  TRANSLATION,
  ROTATION,
  OVERLAP
};

struct GateResult
{
  GateRejection rejection {GateRejection::NONE};
  // Effective thresholds after the per-source fallbacks, for operator logs.
  double score_threshold {0.0};
  double translation_cap_m {0.0};
  double rotation_cap_deg {0.0};
  double min_overlap_ratio {0.0};
  double overlap_ratio {0.0};
};

// Acceptance gates in their historical order: fitness (>= rejects, so a
// score exactly at the threshold is rejected), then the translation and
// rotation caps (> rejects, so a correction exactly at the cap passes).
inline GateResult evaluateGates(
  LoopCandidate::Source source,
  double fitness_score,
  const RegistrationDelta & delta,
  const GateConfig & config,
  double overlap_ratio = 1.0)
{
  GateResult result;
  result.min_overlap_ratio = config.min_overlap_ratio;
  result.overlap_ratio = overlap_ratio;
  if (
    config.overlap_large_correction_translation_m > 0.0 &&
    delta.translation_m >= config.overlap_large_correction_translation_m &&
    config.min_overlap_ratio_large_correction > 0.0)
  {
    result.min_overlap_ratio = config.min_overlap_ratio_large_correction;
  }
  result.score_threshold =
    (source == LoopCandidate::Source::SCAN_CONTEXT &&
    config.scan_context_score_threshold > 0.0) ?
    config.scan_context_score_threshold : config.generic_score_threshold;
  const bool is_descriptor_source = source != LoopCandidate::Source::DISTANCE;
  result.translation_cap_m =
    (is_descriptor_source && config.max_translation_descriptor_m > 0.0) ?
    config.max_translation_descriptor_m : config.max_translation_m;
  result.rotation_cap_deg =
    (is_descriptor_source && config.max_rotation_descriptor_deg > 0.0) ?
    config.max_rotation_descriptor_deg : config.max_rotation_deg;

  if (fitness_score >= result.score_threshold) {
    result.rejection = GateRejection::FITNESS;
    return result;
  }
  if (delta.translation_m > result.translation_cap_m) {
    result.rejection = GateRejection::TRANSLATION;
    return result;
  }
  if (delta.rotation_deg > result.rotation_cap_deg) {
    result.rejection = GateRejection::ROTATION;
    return result;
  }
  if (result.min_overlap_ratio > 0.0 && overlap_ratio < result.min_overlap_ratio) {
    result.rejection = GateRejection::OVERLAP;
    return result;
  }
  return result;
}

// The historical three-track selection. Every converged registration feeds
// considerConverged (diagnostics: best_attempt survives even when all gates
// reject); gate-passing results (valid must already be set) feed
// considerValid, which also tracks the best ScanContext-sourced result so
// the operator can prefer it. Ties keep the first arrival (strict <), which
// together with the deterministic candidate order keeps selection
// deterministic.
struct SelectionState
{
  LoopCandidateResult best_attempt;
  LoopCandidateResult best_valid;
  LoopCandidateResult best_scan_context;

  void considerConverged(const LoopCandidateResult & result)
  {
    if (best_attempt.index < 0 || result.fitness_score < best_attempt.fitness_score) {
      best_attempt = result;
    }
  }

  void considerValid(const LoopCandidateResult & result)
  {
    if (!best_valid.valid || result.fitness_score < best_valid.fitness_score) {
      best_valid = result;
    }
    if (
      result.source == LoopCandidate::Source::SCAN_CONTEXT &&
      (!best_scan_context.valid ||
      result.fitness_score < best_scan_context.fitness_score))
    {
      best_scan_context = result;
    }
  }

  // A valid ScanContext result overrides the global best when preferred,
  // even when its fitness is worse. Returns a result with valid == false
  // when no candidate passed the gates.
  LoopCandidateResult select(bool prefer_scan_context) const
  {
    if (prefer_scan_context && best_scan_context.valid) {
      return best_scan_context;
    }
    return best_valid;
  }
};

// The accepted loop edge stores candidate -> latest expressed in the
// candidate frame: from^-1 * (final_transformation * latest), exactly as
// the optimizer consumed it historically.
inline Eigen::Isometry3d composeLoopRelativePose(
  const Eigen::Affine3d & candidate_affine,
  const Eigen::Affine3d & latest_affine,
  const Eigen::Matrix4f & final_transformation)
{
  Eigen::Isometry3d from = Eigen::Isometry3d(candidate_affine.matrix());
  Eigen::Isometry3d to = Eigen::Isometry3d(
    final_transformation.cast<double>() * latest_affine.matrix());
  return Eigen::Isometry3d(from.inverse() * to);
}

}  // namespace loop_verifier
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__LOOP_VERIFIER_HPP_
