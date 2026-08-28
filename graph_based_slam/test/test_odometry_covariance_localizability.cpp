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

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>

#include "graph_based_slam/adjacent_edge_covariance_weighting.hpp"
#include "graph_based_slam/localizability_analysis.hpp"
#include "graph_based_slam/odometry_covariance_localizability.hpp"
#include "graph_based_slam/synthetic_degeneracy_fixtures.hpp"

namespace
{

using graphslam::degeneracy::analyzeLocalizability;
using graphslam::degeneracy::analyzeOdometryCovariance;
using graphslam::degeneracy::AdjacentEdgeCovarianceWeightingConfig;
using graphslam::degeneracy::BoxFixtureConfig;
using graphslam::degeneracy::buildGaussNewtonSystem;
using graphslam::degeneracy::CorridorFixtureConfig;
using graphslam::degeneracy::CovarianceLocalizabilityResult;
using graphslam::degeneracy::GaussNewtonSystem;
using graphslam::degeneracy::LocalizabilityCategory;
using graphslam::degeneracy::looksLikeNoDiagnosticsFallback;
using graphslam::degeneracy::makeBoxFixture;
using graphslam::degeneracy::makeCorridorFixture;
using graphslam::degeneracy::makeSinglePlaneFixture;
using graphslam::degeneracy::matrix6dFromRowMajorCovariance;
using graphslam::degeneracy::Matrix6d;
using graphslam::degeneracy::SinglePlaneFixtureConfig;
using graphslam::degeneracy::weightAdjacentEdgeFromCovariance;

// -----------------------------------------------------------------------
// Test-only replica of the Thirdparty/rko_lio fork's own
// `Cov = V * diag(1 / max(floor, lambda_i)) * V^T` wire transform
// (docs/research/rko-lio-diagnostic-patch-characterization.md), used here
// only to build a realistic input for `analyzeOdometryCovariance` from the
// Phase 0 synthetic (H, b) fixtures -- this is the "transform is valid,
// checked by a test" requirement, not a second production implementation.
// -----------------------------------------------------------------------

Matrix6d covarianceFromHessianLikeFork(const Matrix6d & h)
{
  const Matrix6d symmetric = 0.5 * (h + h.transpose());
  Eigen::SelfAdjointEigenSolver<Matrix6d> solver(symmetric);
  const Eigen::Matrix<double, 6, 1> eigenvalues = solver.eigenvalues();
  const Matrix6d eigenvectors = solver.eigenvectors();

  const double lambda_max = eigenvalues(5);
  const double floor_value = std::max(1.0e-9, 1.0e-6 * lambda_max);

  Eigen::Matrix<double, 6, 1> inv_eigenvalues;
  for (int i = 0; i < 6; ++i) {
    inv_eigenvalues(i) = 1.0 / std::max(floor_value, eigenvalues(i));
  }
  return eigenvectors * inv_eigenvalues.asDiagonal() * eigenvectors.transpose();
}

std::array<double, 36> toRowMajorArray(const Matrix6d & m)
{
  std::array<double, 36> out {};
  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col < 6; ++col) {
      out[static_cast<size_t>(row * 6 + col)] = m(row, col);
    }
  }
  return out;
}

// -----------------------------------------------------------------------
// Round-trip: Phase 0 fixture H -> fork-style Cov -> analyzeOdometryCovariance
// must reproduce the same per-direction categories as analyzeLocalizability(H)
// directly (docs/roadmap/v0.8.md task note: "分類は covariance の固有値でも
// 等価にできる...変換の妥当性をテストで担保すること").
// -----------------------------------------------------------------------

TEST(OdometryCovarianceLocalizability, CorridorRoundTripMatchesDirectClassification)
{
  const auto fixture = makeCorridorFixture(CorridorFixtureConfig());
  const GaussNewtonSystem system = buildGaussNewtonSystem(fixture.correspondences);
  const auto direct = analyzeLocalizability(system.h, system.b);

  const Matrix6d cov = covarianceFromHessianLikeFork(system.h);
  const CovarianceLocalizabilityResult recovered =
    analyzeOdometryCovariance(toRowMajorArray(cov));

  ASSERT_TRUE(recovered.diagnostics_available);
  EXPECT_EQ(recovered.report.degenerate_count, direct.degenerate_count);
  EXPECT_EQ(recovered.report.non_observable_count, direct.non_observable_count);
  EXPECT_EQ(recovered.report.well_conditioned_count, direct.well_conditioned_count);
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(recovered.report.directions[i].category, direct.directions[i].category) << i;
  }
  // Direction 0 (along-corridor tx) is the exact-zero eigenvalue in H; after
  // the round trip it must still land far below well_conditioned_ratio, not
  // be inflated back to a well-conditioned reading by the floor.
  EXPECT_EQ(recovered.report.directions[0].category, LocalizabilityCategory::DEGENERATE);
}

TEST(OdometryCovarianceLocalizability, BoxRoundTripMatchesDirectClassificationAndEigenvalues)
{
  const auto fixture = makeBoxFixture(BoxFixtureConfig());
  const GaussNewtonSystem system = buildGaussNewtonSystem(fixture.correspondences);
  const auto direct = analyzeLocalizability(system.h, system.b);

  const Matrix6d cov = covarianceFromHessianLikeFork(system.h);
  const CovarianceLocalizabilityResult recovered =
    analyzeOdometryCovariance(toRowMajorArray(cov));

  ASSERT_TRUE(recovered.diagnostics_available);
  EXPECT_EQ(recovered.report.well_conditioned_count, 6);
  EXPECT_EQ(recovered.report.degenerate_count, 0);
  EXPECT_EQ(recovered.report.non_observable_count, 0);
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(recovered.report.directions[i].category, direct.directions[i].category) << i;
    // No floor was applied anywhere (no near-zero eigenvalue in the box
    // fixture), so the round trip should recover the eigenvalues almost
    // exactly, not just the category.
    EXPECT_NEAR(
      recovered.report.directions[i].eigenvalue, direct.directions[i].eigenvalue,
      1e-6 * direct.directions[i].eigenvalue) << i;
  }
}

TEST(
  OdometryCovarianceLocalizability,
  SinglePlaneRoundTripMatchesDirectNonObservableClassification)
{
  const auto fixture = makeSinglePlaneFixture(SinglePlaneFixtureConfig());
  const GaussNewtonSystem system = buildGaussNewtonSystem(fixture.correspondences);
  const auto direct = analyzeLocalizability(system.h, system.b);

  const Matrix6d cov = covarianceFromHessianLikeFork(system.h);
  const CovarianceLocalizabilityResult recovered =
    analyzeOdometryCovariance(toRowMajorArray(cov));

  ASSERT_TRUE(recovered.diagnostics_available);
  EXPECT_EQ(recovered.report.non_observable_count, 3);
  EXPECT_EQ(recovered.report.degenerate_count, 0);
  EXPECT_EQ(recovered.report.well_conditioned_count, 3);
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(recovered.report.directions[i].category, direct.directions[i].category) << i;
  }
}

// -----------------------------------------------------------------------
// "No diagnostics available" fallback detection.
// -----------------------------------------------------------------------

TEST(OdometryCovarianceLocalizability, AllZeroCovarianceIsNotAvailable)
{
  std::array<double, 36> covariance {};  // ROS default: every field zero.
  const CovarianceLocalizabilityResult result = analyzeOdometryCovariance(covariance);
  EXPECT_FALSE(result.diagnostics_available);
}

TEST(OdometryCovarianceLocalizability, IsotropicForkFallbackCovarianceIsNotAvailable)
{
  Matrix6d cov = Matrix6d::Identity() * 1.0e6;
  const CovarianceLocalizabilityResult result =
    analyzeOdometryCovariance(toRowMajorArray(cov));
  EXPECT_FALSE(result.diagnostics_available);
  EXPECT_TRUE(looksLikeNoDiagnosticsFallback(cov));
}

TEST(OdometryCovarianceLocalizability, AnyIsotropicScaleIsRecognizedNotJustOneMillion)
{
  // The fallback check is deliberately not keyed to the fork's specific 1e6
  // constant (an implementation detail, not a wire contract): any exact
  // scaled identity must be recognized.
  Matrix6d cov = Matrix6d::Identity() * 42.0;
  EXPECT_TRUE(looksLikeNoDiagnosticsFallback(cov));
  const CovarianceLocalizabilityResult result =
    analyzeOdometryCovariance(toRowMajorArray(cov));
  EXPECT_FALSE(result.diagnostics_available);
}

TEST(OdometryCovarianceLocalizability, RealAnisotropicCovarianceIsNotMistakenForFallback)
{
  const auto fixture = makeBoxFixture(BoxFixtureConfig());
  const GaussNewtonSystem system = buildGaussNewtonSystem(fixture.correspondences);
  const Matrix6d cov = covarianceFromHessianLikeFork(system.h);
  EXPECT_FALSE(looksLikeNoDiagnosticsFallback(cov));
}

TEST(AdjacentEdgeCovarianceWeighting, DisabledPreservesLegacyMatrixExactly)
{
  const auto fixture = makeCorridorFixture(CorridorFixtureConfig());
  const GaussNewtonSystem system = buildGaussNewtonSystem(fixture.correspondences);
  const auto covariance = toRowMajorArray(covarianceFromHessianLikeFork(system.h));
  const Matrix6d base = Matrix6d::Identity() * 1000.0;

  const auto result = weightAdjacentEdgeFromCovariance(
    base, covariance, AdjacentEdgeCovarianceWeightingConfig());

  EXPECT_FALSE(result.applied);
  EXPECT_TRUE(result.information == base);
}

TEST(AdjacentEdgeCovarianceWeighting, MissingDiagnosticsPreservesLegacyMatrix)
{
  std::array<double, 36> covariance {};
  const Matrix6d base = Matrix6d::Identity() * 1000.0;
  AdjacentEdgeCovarianceWeightingConfig config;
  config.enabled = true;

  const auto result = weightAdjacentEdgeFromCovariance(base, covariance, config);

  EXPECT_FALSE(result.diagnostics_available);
  EXPECT_FALSE(result.applied);
  EXPECT_TRUE(result.information == base);
}

TEST(AdjacentEdgeCovarianceWeighting, CorridorDownweightsOnlyWeakAxis)
{
  const auto fixture = makeCorridorFixture(CorridorFixtureConfig());
  const GaussNewtonSystem system = buildGaussNewtonSystem(fixture.correspondences);
  const auto covariance = toRowMajorArray(covarianceFromHessianLikeFork(system.h));
  const Matrix6d base = Matrix6d::Identity() * 1000.0;
  AdjacentEdgeCovarianceWeightingConfig config;
  config.enabled = true;
  config.degenerate_information_scale = 0.25;
  config.non_observable_information_scale = 0.05;

  const auto result = weightAdjacentEdgeFromCovariance(base, covariance, config);

  ASSERT_TRUE(result.diagnostics_available);
  ASSERT_TRUE(result.applied);
  EXPECT_NEAR(result.information(0, 0), 250.0, 1.0e-6);
  EXPECT_NEAR(result.information(1, 1), 1000.0, 1.0e-6);
  EXPECT_NEAR(result.information(2, 2), 1000.0, 1.0e-6);
}

TEST(AdjacentEdgeCovarianceWeighting, ObservableBoxPreservesLegacyMatrix)
{
  const auto fixture = makeBoxFixture(BoxFixtureConfig());
  const GaussNewtonSystem system = buildGaussNewtonSystem(fixture.correspondences);
  const auto covariance = toRowMajorArray(covarianceFromHessianLikeFork(system.h));
  const Matrix6d base = Matrix6d::Identity() * 1000.0;
  AdjacentEdgeCovarianceWeightingConfig config;
  config.enabled = true;

  const auto result = weightAdjacentEdgeFromCovariance(base, covariance, config);

  ASSERT_TRUE(result.diagnostics_available);
  EXPECT_FALSE(result.applied);
  EXPECT_TRUE(result.information.isApprox(base, 1.0e-12));
}

TEST(OdometryCovarianceLocalizability, NegativeDiagonalIsRejectedDefensively)
{
  std::array<double, 36> covariance {};
  covariance[0] = -1.0;  // Malformed input; must not crash or misclassify.
  const CovarianceLocalizabilityResult result = analyzeOdometryCovariance(covariance);
  EXPECT_FALSE(result.diagnostics_available);
}

TEST(OdometryCovarianceLocalizability, NanDiagonalIsRejectedDefensively)
{
  std::array<double, 36> covariance {};
  covariance[0] = std::numeric_limits<double>::quiet_NaN();
  const CovarianceLocalizabilityResult result = analyzeOdometryCovariance(covariance);
  EXPECT_FALSE(result.diagnostics_available);
}

// -----------------------------------------------------------------------
// Row-major layout + symmetrization + determinism.
// -----------------------------------------------------------------------

TEST(OdometryCovarianceLocalizability, RowMajorLayoutMapsToExpectedEntries)
{
  std::array<double, 36> covariance {};
  // Row 2, column 4 (0-indexed) -> flat index 2*6+4 = 16.
  covariance[16] = 7.5;
  const Matrix6d m = matrix6dFromRowMajorCovariance(covariance);
  // Symmetrized: (7.5 + 0) / 2 on both (2,4) and (4,2).
  EXPECT_DOUBLE_EQ(m(2, 4), 3.75);
  EXPECT_DOUBLE_EQ(m(4, 2), 3.75);
}

TEST(OdometryCovarianceLocalizability, SameCovarianceTwiceProducesIdenticalResult)
{
  const auto fixture = makeCorridorFixture(CorridorFixtureConfig());
  const GaussNewtonSystem system = buildGaussNewtonSystem(fixture.correspondences);
  const Matrix6d cov = covarianceFromHessianLikeFork(system.h);
  const std::array<double, 36> covariance = toRowMajorArray(cov);

  const CovarianceLocalizabilityResult first = analyzeOdometryCovariance(covariance);
  const CovarianceLocalizabilityResult second = analyzeOdometryCovariance(covariance);

  ASSERT_EQ(first.diagnostics_available, second.diagnostics_available);
  for (int i = 0; i < 6; ++i) {
    EXPECT_DOUBLE_EQ(
      first.report.directions[i].eigenvalue, second.report.directions[i].eigenvalue) << i;
    EXPECT_EQ(first.report.directions[i].category, second.report.directions[i].category) << i;
  }
}

}  // namespace
