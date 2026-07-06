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

#ifndef GRAPH_BASED_SLAM__LOCALIZABILITY_ANALYSIS_HPP_
#define GRAPH_BASED_SLAM__LOCALIZABILITY_ANALYSIS_HPP_

// Pure, ROS-free localizability/degeneracy detector for v0.8 Phase 1
// (docs/roadmap/v0.8.md §3 "Target architecture", §5 "Phase 1 -- detection,
// report-only"). Consumes any 6x6 Gauss-Newton normal-equations pair
// `(H[, b])` -- from RKO-LIO's `build_icp_linear_system()`, scanmatcher's
// NDT/GICP, or the Phase 0 synthetic fixtures
// (`synthetic_degeneracy_fixtures.hpp`) -- and classifies each of the six
// SE(3) pose-update directions (twist order `[translation, rotation]`, same
// convention as `se3_lie.hpp` / `plane_ba.hpp` / the synthetic fixtures).
// Report-only: this header never mutates `H`/`b` and never changes a pose;
// Phase 2 (solution remapping / direction-wise blending) is a separate,
// opt-in, default-off change layered on top of this detector's output.
//
// Clean-room: derived only from the published mathematics of
//   - Zhang, Kaess, Singh, "On Degeneracy of Optimization-based State
//     Estimation Problems", ICRA 2016: eigen-decompose the symmetric
//     Gauss-Newton Hessian `H`; small eigenvalues along an eigenvector
//     indicate the optimization is under-constrained along that direction
//     of the pose update.
//   - Tuna, Nubert, Pfreundschuh, Cadena, Khattak, Hutter, "X-ICP:
//     Localizability-Aware LiDAR Registration for Robust Localization in
//     Extreme Environments", arXiv:2306.08258 / IEEE T-RO 2024: classify
//     each direction into well-conditioned / degenerate / non-observable
//     from a *normalized* per-direction contribution test, rather than a
//     single scale-dependent eigenvalue threshold (X-ICP's own test is
//     computed per point correspondence; see the adaptation note below for
//     why this file computes an aggregate analogue instead).
// No GPL reference implementation of either paper was read for this file;
// no upstream RKO-LIO source was consulted.
//
// -- Adaptation note: an aggregate, H-only normalized contribution test --
// X-ICP's normalized test operates per point correspondence: each
// correspondence's Jacobian row is projected onto an eigenvector and
// normalized by that correspondence's own Jacobian norm, which cancels the
// unit mismatch between the translation block (unit surface normals, O(1))
// and the rotation block (moment arms `p x n`, scaling with the sensor's
// distance to the surface -- tens of meters in a long corridor) *before*
// the correspondences are summed into `H`. This file's API is intentionally
// `(H, b)`-only (docs/roadmap/v0.8.md §3: "consumes any (H, b) Gauss-Newton
// pair"), so it cannot see individual correspondences and instead computes
// the aggregate analogue: `contribution_i = lambda_i / trace(H)`, the
// fraction of the Hessian's total trace ("information") carried by
// eigenvector `i`. This is exactly X-ICP's normalization goal (a
// scale-portable ratio rather than a raw eigenvalue in `H`'s native units)
// applied at the whole-Hessian level instead of the per-point level, at the
// cost of not fully separating out the translation/rotation unit mismatch
// the way the per-point version does -- e.g. a corridor's rotation-block
// eigenvalues are inflated by the corridor length itself (see
// `synthetic_degeneracy_fixtures.hpp`'s derivation), so `contribution_i` for
// a perfectly well-observed translational direction can still be a small
// *fraction* of the trace when a much longer lever arm dominates elsewhere
// in the spectrum. Empirically (this file's own test fixtures) the
// well-observed/unobserved gap remains many orders of magnitude regardless
// of this effect, but the exact threshold is data-dependent by
// construction, hence not fixed to a paper value below (see
// `LocalizabilityThresholds`).
//
// -- Degenerate vs. non-observable: why they are not just two threshold
//    tiers of the same test --
// Both categories can present as an eigenvalue at (or numerically
// indistinguishable from) exact zero -- a single ill-constrained direction
// and a multi-dimensional unobservable subspace both zero out the relevant
// eigenvalue(s). What differs is *multiplicity*: for a symmetric matrix,
// eigenvectors belonging to a repeated (or near-repeated) eigenvalue are
// only defined up to an arbitrary orthonormal rotation within that
// eigenspace -- a basic linear-algebra fact, not a modeling choice. A
// simple (non-repeated) near-zero eigenvalue names one specific, physically
// meaningful ill-constrained direction (Zhang & Singh's "degenerate"); a
// cluster of two or more mutually near-equal weak eigenvalues names a
// *subspace* in which no individual reported eigenvector is uniquely
// meaningful -- the stronger "non-observable" category. This file clusters
// adjacent (post-sort) weak directions whose normalized contributions are
// within `multiplicity_relative_gap` of each other and escalates any
// cluster of size >= 2 from degenerate to non-observable. This is the
// mechanism that gives the single-plane fixture's tx/ty/yaw trio
// "non-observable" (a genuine rank-3 null space) while the corridor
// fixture's lone along-axis direction stays "degenerate" (an isolated
// simple root), matching docs/roadmap/v0.8.md §4.3's expected fixture
// outcomes.
//
// Determinism: eigenvalues are returned ascending, eigenvectors sign-
// canonicalized by largest-magnitude component (identical convention to
// `synthetic_degeneracy_fixtures.hpp::computeEigenSignature` and
// `scatter_eigen_cost.hpp::canonicalNormal`), and every reduction below is a
// fixed-order loop over exactly six directions -- same input `H`/`b` always
// produces a bitwise-identical `LocalizabilityReport`.

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include <Eigen/Core>  // NOLINT(build/include_order)
#include <Eigen/Eigenvalues>  // NOLINT(build/include_order)

namespace graphslam
{
namespace degeneracy
{

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

/// Per-direction localizability category (X-ICP, arXiv:2306.08258).
enum class LocalizabilityCategory
{
  WELL_CONDITIONED,
  DEGENERATE,
  NON_OBSERVABLE,
};

/// Classification thresholds. Every value here is a dimensionless ratio
/// (invariant to `H -> c*H` for any `c > 0`, the "H scale invariance"
/// property this file's tests check), so none of them are raw eigenvalue
/// floors in `H`'s native physical units.
///
/// PROVISIONAL PHASE 1 DEFAULTS. Per docs/roadmap/v0.8.md §5 Phase 1 /
/// §9 item 3, these are *not* borrowed from Zhang & Singh or X-ICP (their
/// published values were tuned to their own sensors/feature scales and do
/// not port directly) and are *not* the final answer -- they are only
/// calibrated well enough to pass on the Phase 0 synthetic fixtures
/// (corridor / box / single-plane). The Phase 1 gate requires recalibrating
/// both ratios from real Phase 0 HILTI exp01/exp04/exp07 `(H, b)`
/// telemetry before they are used to judge real substrates.
struct LocalizabilityThresholds
{
  /// A direction is WELL_CONDITIONED when its normalized contribution
  /// (`lambda_i / trace(H)`) is at least this fraction of the Hessian's
  /// total information. Directions below this are "weak" candidates for
  /// DEGENERATE or NON_OBSERVABLE (see the multiplicity rule above).
  double well_conditioned_ratio {1.0e-4};

  /// Two adjacent (post-sort) weak directions are considered part of the
  /// same near-repeated eigenspace -- and therefore escalated together to
  /// NON_OBSERVABLE -- when their normalized contributions differ by no
  /// more than this amount. Deliberately far smaller than
  /// `well_conditioned_ratio`: this only merges directions that are
  /// mutually indistinguishable from each other (e.g. simultaneously
  /// algebraically zero), not merely "both weak".
  double multiplicity_relative_gap {1.0e-6};
};

/// Result for a single one of the six SE(3) update directions, in ascending
/// eigenvalue order (index 0 = smallest / most ill-constrained).
struct DirectionResult
{
  /// Raw eigenvalue of (the symmetrized) `H`, in `H`'s native units.
  double eigenvalue {0.0};

  /// Sign-canonicalized eigenvector (largest-magnitude component >= 0).
  /// Twist order `[translation, rotation]`.
  Vector6d eigenvector {Vector6d::Zero()};

  /// `eigenvalue / trace(H)`; 0 when `trace(H) <= 0`. The aggregate,
  /// H-only analogue of X-ICP's per-point normalized contribution (see
  /// file header).
  double normalized_contribution {0.0};

  LocalizabilityCategory category {LocalizabilityCategory::WELL_CONDITIONED};
};

/// Full localizability report for a 6x6 Gauss-Newton system: per-direction
/// categories plus the scalar summary fields the map-bundle degeneracy
/// report / offline diagnostics CSV consume (docs/roadmap/v0.8.md §5
/// Phase 1).
struct LocalizabilityReport
{
  /// Ascending eigenvalue order (index 0 = smallest).
  std::array<DirectionResult, 6> directions {};

  double min_eigenvalue {0.0};
  double max_eigenvalue {0.0};

  /// `max_eigenvalue / min_eigenvalue`; `+inf` when `min_eigenvalue` is at
  /// (or numerically indistinguishable from) zero, i.e. `H` is singular.
  double condition_number {0.0};

  int well_conditioned_count {0};
  int degenerate_count {0};
  int non_observable_count {0};

  /// Diagnostic-only naive Gauss-Newton update `H.ldlt().solve(-b)`.
  /// Deliberately *not* used by classification above, and only populated
  /// (raw_update_valid = true) when every direction is WELL_CONDITIONED --
  /// with any degenerate/non-observable direction present, `H` is singular
  /// or near-singular along that direction and the naive solve is not
  /// trustworthy without Phase 2's solution remapping / direction-wise
  /// blending (docs/roadmap/v0.8.md §2 item 3, §5 Phase 2), which this
  /// report-only Phase 1 detector does not perform.
  Vector6d raw_update {Vector6d::Zero()};
  bool raw_update_valid {false};
};

namespace detail
{

// Sign-canonicalize each eigenvector so its largest-magnitude component is
// non-negative -- identical convention to
// synthetic_degeneracy_fixtures.hpp::computeEigenSignature and
// scatter_eigen_cost.hpp::canonicalNormal, required for the "same H twice ->
// bitwise identical report" determinism contract (SelfAdjointEigenSolver
// does not itself guarantee a canonical sign for a given eigenvalue).
inline void canonicalizeEigenvectorSigns(Matrix6d * vectors)
{
  for (int col = 0; col < 6; ++col) {
    int largest_index = 0;
    double largest_abs = std::abs((*vectors)(0, col));
    for (int row = 1; row < 6; ++row) {
      const double component_abs = std::abs((*vectors)(row, col));
      if (component_abs > largest_abs) {
        largest_abs = component_abs;
        largest_index = row;
      }
    }
    if ((*vectors)(largest_index, col) < 0.0) {
      vectors->col(col) = -vectors->col(col);
    }
  }
}

}  // namespace detail

/// Classify a 6x6 Gauss-Newton system's six SE(3) update directions.
///
/// `h` need not be pre-symmetrized (it is symmetrized defensively, matching
/// `synthetic_degeneracy_fixtures.hpp::computeEigenSignature`); `b` is
/// optional and only feeds the diagnostic `raw_update` field, never the
/// classification itself (Zhang & Singh's detector, and X-ICP's category
/// test, are both purely a function of `H`'s eigenstructure).
inline LocalizabilityReport analyzeLocalizability(
  const Matrix6d & h,
  const Vector6d & b = Vector6d::Zero(),
  const LocalizabilityThresholds & thresholds = LocalizabilityThresholds())
{
  LocalizabilityReport report;

  const Matrix6d symmetric = 0.5 * (h + h.transpose());
  Eigen::SelfAdjointEigenSolver<Matrix6d> solver(symmetric);
  Vector6d eigenvalues = solver.eigenvalues();
  Matrix6d eigenvectors = solver.eigenvectors();
  detail::canonicalizeEigenvectorSigns(&eigenvectors);

  report.min_eigenvalue = eigenvalues(0);
  report.max_eigenvalue = eigenvalues(5);

  // Condition number: +inf when H is singular (or effectively so) rather
  // than a raw division producing NaN/-inf from floating-point noise around
  // zero.
  constexpr double kSingularFloor = 1.0e-12;
  if (report.min_eigenvalue <= kSingularFloor || report.max_eigenvalue <= 0.0) {
    report.condition_number = std::numeric_limits<double>::infinity();
  } else {
    report.condition_number = report.max_eigenvalue / report.min_eigenvalue;
  }

  const double trace = eigenvalues.sum();
  const bool have_information = trace > kSingularFloor && report.max_eigenvalue > kSingularFloor;

  std::array<double, 6> contribution {};
  std::array<bool, 6> is_weak {};
  for (int i = 0; i < 6; ++i) {
    contribution[i] = have_information ? (eigenvalues(i) / trace) : 0.0;
    is_weak[i] = !have_information || (contribution[i] < thresholds.well_conditioned_ratio);
  }

  // Cluster adjacent (ascending-sorted) weak directions whose normalized
  // contributions are mutually indistinguishable -- see the file header's
  // "degenerate vs. non-observable" note. A well-conditioned direction
  // never merges into a weak cluster (the merge test below requires both
  // sides weak), so cluster membership among weak indices is exactly the
  // repeated/near-repeated eigenspace the merge rule targets.
  std::array<int, 6> cluster_id {};
  cluster_id[0] = 0;
  for (int i = 1; i < 6; ++i) {
    const bool merge = is_weak[i] && is_weak[i - 1] &&
      (contribution[i] - contribution[i - 1]) <= thresholds.multiplicity_relative_gap;
    cluster_id[i] = merge ? cluster_id[i - 1] : cluster_id[i - 1] + 1;
  }
  std::array<int, 6> cluster_size {};
  for (int i = 0; i < 6; ++i) {
    ++cluster_size[cluster_id[i]];
  }

  for (int i = 0; i < 6; ++i) {
    DirectionResult & direction = report.directions[i];
    direction.eigenvalue = eigenvalues(i);
    direction.eigenvector = eigenvectors.col(i);
    direction.normalized_contribution = contribution[i];

    if (!is_weak[i]) {
      direction.category = LocalizabilityCategory::WELL_CONDITIONED;
      ++report.well_conditioned_count;
    } else if (cluster_size[cluster_id[i]] >= 2) {
      direction.category = LocalizabilityCategory::NON_OBSERVABLE;
      ++report.non_observable_count;
    } else {
      direction.category = LocalizabilityCategory::DEGENERATE;
      ++report.degenerate_count;
    }
  }

  // Diagnostic-only naive solve; only trusted when nothing is degenerate.
  report.raw_update_valid = (report.degenerate_count == 0 && report.non_observable_count == 0);
  if (report.raw_update_valid) {
    Eigen::LDLT<Matrix6d> ldlt(symmetric);
    if (ldlt.info() == Eigen::Success) {
      report.raw_update = ldlt.solve(-b);
      if (!report.raw_update.allFinite()) {
        report.raw_update_valid = false;
        report.raw_update.setZero();
      }
    } else {
      report.raw_update_valid = false;
    }
  }

  return report;
}

}  // namespace degeneracy
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__LOCALIZABILITY_ANALYSIS_HPP_
