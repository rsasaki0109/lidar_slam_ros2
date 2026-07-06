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

#include <cmath>
#include <limits>

#include "graph_based_slam/localizability_analysis.hpp"
#include "graph_based_slam/synthetic_degeneracy_fixtures.hpp"

namespace
{

using graphslam::degeneracy::analyzeLocalizability;
using graphslam::degeneracy::BoxFixtureConfig;
using graphslam::degeneracy::buildGaussNewtonSystem;
using graphslam::degeneracy::CorridorFixtureConfig;
using graphslam::degeneracy::DegeneracyFixture;
using graphslam::degeneracy::GaussNewtonSystem;
using graphslam::degeneracy::LocalizabilityCategory;
using graphslam::degeneracy::LocalizabilityReport;
using graphslam::degeneracy::LocalizabilityThresholds;
using graphslam::degeneracy::makeBoxFixture;
using graphslam::degeneracy::makeCorridorFixture;
using graphslam::degeneracy::makeSinglePlaneFixture;
using graphslam::degeneracy::Matrix6d;
using graphslam::degeneracy::SinglePlaneFixtureConfig;
using graphslam::degeneracy::Vector6d;

// -----------------------------------------------------------------------
// Phase 0 synthetic fixtures as the Phase 1 detector oracle
// (docs/roadmap/v0.8.md §4.3, §5 Phase 1 gate).
// -----------------------------------------------------------------------

TEST(LocalizabilityAnalysis, CorridorHasExactlyOneDegenerateAlongAxisDirection)
{
  const DegeneracyFixture fixture = makeCorridorFixture(CorridorFixtureConfig());
  const GaussNewtonSystem system = buildGaussNewtonSystem(fixture.correspondences);
  const LocalizabilityReport report = analyzeLocalizability(system.h, system.b);

  EXPECT_EQ(report.degenerate_count, 1);
  EXPECT_EQ(report.non_observable_count, 0);
  EXPECT_EQ(report.well_conditioned_count, 5);

  // Direction 0 (smallest eigenvalue, ascending order) is the along-corridor
  // (tx) direction and must be DEGENERATE, not NON_OBSERVABLE: it is an
  // isolated simple root (see file header), and must not be mistaken for a
  // multi-directional unobservable subspace.
  EXPECT_EQ(report.directions[0].category, LocalizabilityCategory::DEGENERATE);
  EXPECT_NEAR(report.directions[0].eigenvalue, 0.0, 1e-6);

  Vector6d expected_axis = Vector6d::Zero();
  expected_axis(0) = 1.0;
  EXPECT_NEAR(std::abs(report.directions[0].eigenvector.dot(expected_axis)), 1.0, 1e-9);

  for (int i = 1; i < 6; ++i) {
    EXPECT_EQ(report.directions[i].category, LocalizabilityCategory::WELL_CONDITIONED) << i;
  }

  EXPECT_TRUE(std::isinf(report.condition_number));
  EXPECT_FALSE(report.raw_update_valid);
  EXPECT_TRUE(report.raw_update.isZero());
}

TEST(LocalizabilityAnalysis, BoxIsWellConditionedInAllSixDirections)
{
  const DegeneracyFixture fixture = makeBoxFixture(BoxFixtureConfig());
  const GaussNewtonSystem system = buildGaussNewtonSystem(fixture.correspondences);
  const LocalizabilityReport report = analyzeLocalizability(system.h, system.b);

  EXPECT_EQ(report.well_conditioned_count, 6);
  EXPECT_EQ(report.degenerate_count, 0);
  EXPECT_EQ(report.non_observable_count, 0);
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(report.directions[i].category, LocalizabilityCategory::WELL_CONDITIONED) << i;
  }

  EXPECT_FALSE(std::isinf(report.condition_number));
  EXPECT_GT(report.condition_number, 1.0);

  // All six directions well-conditioned -> the naive Gauss-Newton solve is
  // trusted (diagnostic-only; not used by classification itself).
  EXPECT_TRUE(report.raw_update_valid);
}

TEST(LocalizabilityAnalysis, SinglePlaneIsNonObservableInThreeDirectionsNotFalselyWellConditioned)
{
  const DegeneracyFixture fixture = makeSinglePlaneFixture(SinglePlaneFixtureConfig());
  const GaussNewtonSystem system = buildGaussNewtonSystem(fixture.correspondences);
  const LocalizabilityReport report = analyzeLocalizability(system.h, system.b);

  // The hazard this fixture exists to catch (docs/roadmap/v0.8.md §4.3):
  // three simultaneously unobservable directions must not collapse into a
  // false WELL_CONDITIONED reading, and must be reported as NON_OBSERVABLE
  // (a genuine rank-3 null space), not merely DEGENERATE.
  EXPECT_EQ(report.non_observable_count, 3);
  EXPECT_EQ(report.degenerate_count, 0);
  EXPECT_EQ(report.well_conditioned_count, 3);

  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(report.directions[i].category, LocalizabilityCategory::NON_OBSERVABLE) << i;
    EXPECT_NEAR(report.directions[i].eigenvalue, 0.0, 1e-6);
  }
  for (int i = 3; i < 6; ++i) {
    EXPECT_EQ(report.directions[i].category, LocalizabilityCategory::WELL_CONDITIONED) << i;
  }

  EXPECT_TRUE(std::isinf(report.condition_number));
  EXPECT_FALSE(report.raw_update_valid);
}

// -----------------------------------------------------------------------
// Determinism: same (H, b) twice -> bitwise identical report.
// -----------------------------------------------------------------------

TEST(LocalizabilityAnalysis, SameInputTwiceProducesBitwiseIdenticalReport)
{
  const DegeneracyFixture fixture = makeCorridorFixture(CorridorFixtureConfig());
  const GaussNewtonSystem system = buildGaussNewtonSystem(fixture.correspondences);

  const LocalizabilityReport first = analyzeLocalizability(system.h, system.b);
  const LocalizabilityReport second = analyzeLocalizability(system.h, system.b);

  EXPECT_DOUBLE_EQ(first.min_eigenvalue, second.min_eigenvalue);
  EXPECT_DOUBLE_EQ(first.max_eigenvalue, second.max_eigenvalue);
  EXPECT_DOUBLE_EQ(first.condition_number, second.condition_number);
  EXPECT_EQ(first.well_conditioned_count, second.well_conditioned_count);
  EXPECT_EQ(first.degenerate_count, second.degenerate_count);
  EXPECT_EQ(first.non_observable_count, second.non_observable_count);
  EXPECT_EQ(first.raw_update_valid, second.raw_update_valid);
  EXPECT_TRUE(first.raw_update.isApprox(second.raw_update, 0.0));

  for (int i = 0; i < 6; ++i) {
    EXPECT_DOUBLE_EQ(first.directions[i].eigenvalue, second.directions[i].eigenvalue) << i;
    EXPECT_DOUBLE_EQ(
      first.directions[i].normalized_contribution,
      second.directions[i].normalized_contribution) << i;
    EXPECT_EQ(first.directions[i].category, second.directions[i].category) << i;
    EXPECT_TRUE(
      first.directions[i].eigenvector.isApprox(second.directions[i].eigenvector, 0.0)) << i;
  }
}

TEST(LocalizabilityAnalysis, EigenvectorSignsAreCanonicalized)
{
  const DegeneracyFixture fixture = makeSinglePlaneFixture(SinglePlaneFixtureConfig());
  const GaussNewtonSystem system = buildGaussNewtonSystem(fixture.correspondences);
  const LocalizabilityReport report = analyzeLocalizability(system.h, system.b);

  for (int col = 0; col < 6; ++col) {
    const Vector6d & v = report.directions[col].eigenvector;
    int largest_index = 0;
    double largest_abs = std::abs(v(0));
    for (int row = 1; row < 6; ++row) {
      const double component_abs = std::abs(v(row));
      if (component_abs > largest_abs) {
        largest_abs = component_abs;
        largest_index = row;
      }
    }
    EXPECT_GE(v(largest_index), 0.0) << col;
  }
}

// -----------------------------------------------------------------------
// H scale invariance: multiplying (H, b) by a positive constant must not
// change any category or normalized_contribution (X-ICP's normalization
// design goal), only the raw eigenvalue magnitudes and raw_update.
// -----------------------------------------------------------------------

TEST(LocalizabilityAnalysis, ScalingHPreservesCategoriesAndNormalizedContributions)
{
  const DegeneracyFixture fixture = makeCorridorFixture(CorridorFixtureConfig());
  const GaussNewtonSystem system = buildGaussNewtonSystem(fixture.correspondences);

  constexpr double kScale = 1000.0;
  const Matrix6d scaled_h = kScale * system.h;
  const Vector6d scaled_b = kScale * system.b;

  const LocalizabilityReport base = analyzeLocalizability(system.h, system.b);
  const LocalizabilityReport scaled = analyzeLocalizability(scaled_h, scaled_b);

  EXPECT_NEAR(scaled.min_eigenvalue, kScale * base.min_eigenvalue, 1e-6);
  EXPECT_NEAR(scaled.max_eigenvalue, kScale * base.max_eigenvalue, 1e-3);
  EXPECT_DOUBLE_EQ(scaled.condition_number, base.condition_number);

  EXPECT_EQ(scaled.well_conditioned_count, base.well_conditioned_count);
  EXPECT_EQ(scaled.degenerate_count, base.degenerate_count);
  EXPECT_EQ(scaled.non_observable_count, base.non_observable_count);

  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(scaled.directions[i].category, base.directions[i].category) << i;
    EXPECT_NEAR(
      scaled.directions[i].normalized_contribution,
      base.directions[i].normalized_contribution,
      1e-9) << i;
  }

  // b scales, H scales -> the diagnostic dx = H^-1(-b) solve is itself
  // scale-invariant (both numerator and denominator scale by kScale), even
  // though this direction set is not all-well-conditioned so raw_update is
  // marked invalid either way.
  EXPECT_EQ(scaled.raw_update_valid, base.raw_update_valid);
}

TEST(LocalizabilityAnalysis, ScalingWellConditionedSystemPreservesRawUpdate)
{
  const DegeneracyFixture fixture = makeBoxFixture(BoxFixtureConfig());
  const GaussNewtonSystem system = buildGaussNewtonSystem(fixture.correspondences);

  constexpr double kScale = 250.0;
  const Matrix6d scaled_h = kScale * system.h;
  const Vector6d scaled_b = kScale * system.b;

  const LocalizabilityReport base = analyzeLocalizability(system.h, system.b);
  const LocalizabilityReport scaled = analyzeLocalizability(scaled_h, scaled_b);

  ASSERT_TRUE(base.raw_update_valid);
  ASSERT_TRUE(scaled.raw_update_valid);
  EXPECT_TRUE(scaled.raw_update.isApprox(base.raw_update, 1e-6));
}

// -----------------------------------------------------------------------
// Category-boundary threshold sensitivity: a synthetic, hand-built H whose
// smallest eigenvalue sits close to well_conditioned_ratio flips category
// as the threshold crosses it.
// -----------------------------------------------------------------------

TEST(LocalizabilityAnalysis, WellConditionedRatioThresholdControlsBoundaryDirection)
{
  // Diagonal H (eigenvectors are the coordinate axes, eigenvalues the
  // diagonal entries themselves): five directions carry 0.199 of the total
  // trace each (0.995 total) and one carries 0.005 -- a boundary direction
  // whose normalized contribution (0.005) sits between two candidate
  // well_conditioned_ratio thresholds (0.01 and 0.001).
  Matrix6d h = Matrix6d::Zero();
  h(0, 0) = 0.5;      // 0.005 of the trace (100.5 total)
  for (int i = 1; i < 6; ++i) {
    h(i, i) = 20.0;   // 0.199 of the trace each
  }

  LocalizabilityThresholds strict;
  strict.well_conditioned_ratio = 0.01;   // 0.005 < 0.01 -> weak
  const LocalizabilityReport strict_report = analyzeLocalizability(h, Vector6d::Zero(), strict);
  EXPECT_EQ(strict_report.directions[0].category, LocalizabilityCategory::DEGENERATE);
  EXPECT_EQ(strict_report.degenerate_count, 1);
  EXPECT_EQ(strict_report.well_conditioned_count, 5);

  LocalizabilityThresholds lenient;
  lenient.well_conditioned_ratio = 0.001;  // 0.005 >= 0.001 -> well-conditioned
  const LocalizabilityReport lenient_report = analyzeLocalizability(h, Vector6d::Zero(), lenient);
  EXPECT_EQ(lenient_report.directions[0].category, LocalizabilityCategory::WELL_CONDITIONED);
  EXPECT_EQ(lenient_report.degenerate_count, 0);
  EXPECT_EQ(lenient_report.well_conditioned_count, 6);
}

// -----------------------------------------------------------------------
// Multiplicity rule: isolated weak directions stay DEGENERATE individually;
// only a mutually near-equal cluster of weak directions escalates to
// NON_OBSERVABLE. This is the mechanism, exercised directly, that lets the
// corridor and single-plane fixtures reach different categories despite
// both having (numerically) zero eigenvalues.
// -----------------------------------------------------------------------

TEST(LocalizabilityAnalysis, TwoDistinctWeakEigenvaluesStayIndividuallyDegenerate)
{
  // Two weak-but-mutually-distinguishable eigenvalues (ratio gap far larger
  // than multiplicity_relative_gap) must not be lumped into one
  // NON_OBSERVABLE cluster -- each is its own isolated degenerate direction.
  Matrix6d h = Matrix6d::Zero();
  h(0, 0) = 0.0;      // exactly zero
  h(1, 1) = 0.05;     // weak (contribution ~1.25e-5, below the 1.5e-5
                      // default well_conditioned_ratio) but far outside
                      // multiplicity_relative_gap (1e-8) from direction 0's
                      // contribution of exactly 0 -- a distinct, isolated
                      // weak direction, not part of direction 0's cluster.
  for (int i = 2; i < 6; ++i) {
    h(i, i) = 1000.0;
  }

  const LocalizabilityReport report = analyzeLocalizability(h);
  EXPECT_EQ(report.directions[0].category, LocalizabilityCategory::DEGENERATE);
  EXPECT_EQ(report.directions[1].category, LocalizabilityCategory::DEGENERATE);
  EXPECT_EQ(report.degenerate_count, 2);
  EXPECT_EQ(report.non_observable_count, 0);
}

TEST(LocalizabilityAnalysis, ClusterOfMutuallyEqualWeakEigenvaluesIsNonObservable)
{
  // Three simultaneously (and mutually equal) near-zero eigenvalues form a
  // genuine repeated eigenspace -> escalated together to NON_OBSERVABLE.
  Matrix6d h = Matrix6d::Zero();
  h(0, 0) = 0.0;
  h(1, 1) = 0.0;
  h(2, 2) = 0.0;
  for (int i = 3; i < 6; ++i) {
    h(i, i) = 1000.0;
  }

  const LocalizabilityReport report = analyzeLocalizability(h);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(report.directions[i].category, LocalizabilityCategory::NON_OBSERVABLE) << i;
  }
  EXPECT_EQ(report.non_observable_count, 3);
  EXPECT_EQ(report.degenerate_count, 0);
}

// -----------------------------------------------------------------------
// Degenerate edge case: H == 0 (no information at all in any direction).
// -----------------------------------------------------------------------

TEST(LocalizabilityAnalysis, ZeroHessianIsNonObservableInAllSixDirections)
{
  const LocalizabilityReport report = analyzeLocalizability(Matrix6d::Zero());
  EXPECT_EQ(report.non_observable_count, 6);
  EXPECT_EQ(report.well_conditioned_count, 0);
  EXPECT_EQ(report.degenerate_count, 0);
  EXPECT_TRUE(std::isinf(report.condition_number));
  EXPECT_FALSE(report.raw_update_valid);
}

}  // namespace
