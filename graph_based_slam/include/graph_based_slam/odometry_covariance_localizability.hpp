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

#ifndef GRAPH_BASED_SLAM__ODOMETRY_COVARIANCE_LOCALIZABILITY_HPP_
#define GRAPH_BASED_SLAM__ODOMETRY_COVARIANCE_LOCALIZABILITY_HPP_

// Pure, ROS-free adapter that lets `localizability_analysis.hpp` classify a
// `nav_msgs/Odometry.pose.covariance` field instead of a raw Gauss-Newton
// `H` (docs/roadmap/v0.8.md §5 Phase 1 "Wire the score into ..."). Motivation
// (docs/research/rko-lio-diagnostic-patch-characterization.md): the
// `Thirdparty/rko_lio` fork already fills `pose.covariance` anisotropically
// as `Cov = V * diag(1 / max(floor, lambda_i)) * V^T`, where `V`/`lambda_i`
// are the final ICP iteration's Gauss-Newton `H` eigenvectors/eigenvalues
// (Zhang & Singh ICRA 2016) and `floor = max(1e-9, 1e-6 * lambda_max)`. That
// is an already-inverted, already-floored view of the same eigenstructure
// `localizability_analysis.hpp` classifies, so this header inverts it back
// (`H' = Cov^-1`) rather than re-deriving a second, parallel classifier.
//
// Why inversion is a valid substitute for the original `H` (not merely a
// convenient shortcut), for classification purposes specifically:
//   - `Cov` and `H` are simultaneously diagonalizable (same eigenvectors
//     `V`), so `H' = Cov^-1 = V * diag(max(floor, lambda_i)) * V^T` has
//     *exactly* the true eigenvalue `lambda_i` on every well-conditioned
//     direction (`lambda_i >> floor`), and `floor` itself in place of a true
//     near-zero/zero `lambda_i` on any degenerate/non-observable direction.
//   - `analyzeLocalizability` classifies by `lambda_i / trace(H)`, a ratio.
//     `floor <= 1e-6 * lambda_max`, so a floored direction's reconstructed
//     contribution is bounded by `1e-6 * lambda_max / trace(H')`, which is
//     far below any reasonable `well_conditioned_ratio` (Phase 1 default
//     `1e-4`, see `localizability_analysis.hpp`) whenever at least one
//     direction is well-conditioned (i.e. `trace(H')` is not itself tiny) --
//     so the floor never flips a genuinely degenerate/non-observable
//     direction back to WELL_CONDITIONED. This equivalence is checked
//     directly (not just asserted) by
//     `test/test_odometry_covariance_localizability.cpp`, which replays the
//     Phase 0 corridor/box/single_plane fixtures through the fork's own
//     `Cov = V diag(1/max(floor,lambda)) V^T` transform and confirms the
//     recovered category matches `analyzeLocalizability(H)` on every
//     direction.
//   - The one thing this adapter cannot recover is the *sign* structure of
//     `b` (the fork's diagnostic patch does not publish `b` on the wire, only
//     the pose and the covariance derived from `H`), so
//     `CovarianceLocalizabilityResult::report.raw_update`/`raw_update_valid`
//     are not meaningful here and are not read by any caller of this header
//     (`analyzeLocalizability` is always invoked with its default `b = 0`).
//
// `nav_msgs/Odometry.pose.covariance` (and the underlying
// `geometry_msgs/PoseWithCovariance.covariance`) is a flat, row-major 6x6
// array in `[x, y, z, rot_x, rot_y, rot_z]` order -- the same
// `[translation, rotation]` twist order `localizability_analysis.hpp` and
// `se3_lie.hpp` already use, so no axis reordering is required.
//
// A scan can carry no usable diagnostics at all: the fork's `node.cpp` falls
// back to an isotropic `1e6` diagonal covariance whenever `State` has no
// `icp_diagnostics` for that frame (first frame, dropped scan, kidnap
// recovery, local reset -- see the characterization doc), and any odometry
// source that is not this project's `rko_lio` fork simply leaves
// `pose.covariance` at its ROS default (all zero, i.e. "unknown"). Both cases
// must be recognized and reported as "no diagnostics" rather than silently
// misclassified as WELL_CONDITIONED or DEGENERATE -- see
// `looksLikeNoDiagnosticsFallback` below.
//
// Clean-room: this header only consumes the already-clean-room
// `localizability_analysis.hpp` API and the wire format documented in
// `docs/research/rko-lio-diagnostic-patch-characterization.md` (itself
// written from the fork's own diff, not from any upstream/GPL source); no
// GPL reference implementation was read.

#include <algorithm>
#include <array>
#include <cmath>

#include <Eigen/Cholesky>  // NOLINT(build/include_order)
#include <Eigen/Core>  // NOLINT(build/include_order)

#include "graph_based_slam/localizability_analysis.hpp"

namespace graphslam
{
namespace degeneracy
{

/// Build a symmetrized `Matrix6d` from a flat row-major 6x6 array -- the
/// wire layout of `geometry_msgs/PoseWithCovariance.covariance` /
/// `nav_msgs/Odometry.pose.covariance`. Defensive symmetrization mirrors
/// `analyzeLocalizability`'s own `0.5 * (h + h.transpose())` so a
/// not-quite-symmetric input (floating-point noise on the wire) never
/// produces a different result than an equivalent, exactly-symmetric one.
inline Matrix6d matrix6dFromRowMajorCovariance(const std::array<double, 36> & covariance)
{
  Matrix6d m;
  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col < 6; ++col) {
      m(row, col) = covariance[static_cast<size_t>(row * 6 + col)];
    }
  }
  return 0.5 * (m + m.transpose());
}

/// Heuristically recognize the "no diagnostics for this scan" fallback
/// shape: a covariance that is (numerically) a positive scalar multiple of
/// the identity -- every diagonal entry equal, every off-diagonal entry
/// zero. This is deliberately *not* keyed to the fork's specific `1e6`
/// constant (which is an implementation detail, not part of the wire
/// contract): a genuine anisotropic per-direction covariance derived from
/// real point-to-plane correspondences essentially never lands on an exact
/// scaled identity (`analyzeLocalizability`'s own fixtures span eigenvalue
/// ratios from 1 (box) to +inf (corridor/single_plane), never a uniform 1),
/// so this check is a safe, constant-free proxy for "not a real per-scan
/// covariance" -- including both the fork's isotropic fallback and an
/// all-zero (ROS-default / non-RKO-LIO source) covariance, which is exactly
/// zero times any scale and trivially satisfies "every diagonal entry
/// equal, every off-diagonal entry zero".
inline bool looksLikeNoDiagnosticsFallback(const Matrix6d & covariance)
{
  const double first_diag = covariance(0, 0);
  if (!(first_diag >= 0.0)) {
    return true;  // NaN or negative: not a usable covariance either way.
  }
  const double scale = std::max(first_diag, 1.0);
  constexpr double kRelativeTolerance = 1.0e-9;
  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col < 6; ++col) {
      const double expected = (row == col) ? first_diag : 0.0;
      if (std::abs(covariance(row, col) - expected) > kRelativeTolerance * scale) {
        return false;
      }
    }
  }
  return true;
}

/// Result of classifying an odometry message's `pose.covariance` field.
struct CovarianceLocalizabilityResult
{
  /// False when `covariance` is the fork's isotropic no-diagnostics
  /// fallback, an all-zero (unpopulated) covariance, or otherwise not
  /// safely invertible (defensive; should not occur for real diagnostics
  /// data given the fork's documented eigenvalue floor). `report` is
  /// default-constructed (all WELL_CONDITIONED-by-default-initialization,
  /// zeroed) and must not be read as a classification when this is false.
  bool diagnostics_available {false};

  /// Localizability classification of `H' = covariance^-1`. Only
  /// `report.raw_update`/`raw_update_valid` are not meaningful (see file
  /// header: `b` never crosses the wire) -- every other field is the same
  /// eigenstructure classification `analyzeLocalizability` would produce
  /// from the original `H`, up to the fork's documented eigenvalue floor.
  LocalizabilityReport report;
};

/// Classify a `nav_msgs/Odometry.pose.covariance`-shaped array. Report-only,
/// pure function of its input (docs/roadmap/v0.8.md §5 Phase 1): never
/// mutates anything, never touches a pose.
inline CovarianceLocalizabilityResult analyzeOdometryCovariance(
  const std::array<double, 36> & covariance,
  const LocalizabilityThresholds & thresholds = LocalizabilityThresholds())
{
  CovarianceLocalizabilityResult result;

  const Matrix6d cov = matrix6dFromRowMajorCovariance(covariance);

  // A real per-scan covariance's trace is bounded below by six times the
  // fork's own floor's reciprocal ceiling in the well-conditioned case and
  // unbounded above in the degenerate case, but it is never (numerically)
  // zero; an exactly-zero trace only happens for an unpopulated ROS default.
  constexpr double kMinTrace = 1.0e-12;
  if (!(cov.trace() > kMinTrace)) {
    return result;
  }
  if (looksLikeNoDiagnosticsFallback(cov)) {
    return result;
  }

  Eigen::LDLT<Matrix6d> ldlt(cov);
  if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
    return result;
  }
  const Matrix6d h_prime = ldlt.solve(Matrix6d::Identity());
  if (!h_prime.allFinite()) {
    return result;
  }

  result.diagnostics_available = true;
  result.report = analyzeLocalizability(h_prime, Vector6d::Zero(), thresholds);
  return result;
}

}  // namespace degeneracy
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__ODOMETRY_COVARIANCE_LOCALIZABILITY_HPP_
