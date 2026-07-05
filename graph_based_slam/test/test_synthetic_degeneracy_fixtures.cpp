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

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "graph_based_slam/synthetic_degeneracy_fixtures.hpp"

namespace
{

using graphslam::degeneracy::BoxFixtureConfig;
using graphslam::degeneracy::buildGaussNewtonSystem;
using graphslam::degeneracy::computeEigenSignature;
using graphslam::degeneracy::CorridorFixtureConfig;
using graphslam::degeneracy::DegeneracyFixture;
using graphslam::degeneracy::EigenSignature;
using graphslam::degeneracy::GaussNewtonSystem;
using graphslam::degeneracy::makeBoxFixture;
using graphslam::degeneracy::makeCorridorFixture;
using graphslam::degeneracy::makeSinglePlaneFixture;
using graphslam::degeneracy::SinglePlaneFixtureConfig;

constexpr double kZeroTol = 1e-6;
constexpr double kWellConditionedMargin = 1.0;

TEST(SyntheticDegeneracyFixtures, CorridorGenerationIsDeterministic)
{
  const CorridorFixtureConfig config;
  const DegeneracyFixture a = makeCorridorFixture(config);
  const DegeneracyFixture b = makeCorridorFixture(config);

  ASSERT_EQ(a.correspondences.size(), b.correspondences.size());
  for (size_t i = 0; i < a.correspondences.size(); ++i) {
    EXPECT_EQ(a.correspondences[i].point, b.correspondences[i].point) << i;
    EXPECT_EQ(a.correspondences[i].normal, b.correspondences[i].normal) << i;
    EXPECT_EQ(a.correspondences[i].planar_offset, b.correspondences[i].planar_offset) << i;
  }

  const GaussNewtonSystem system_a = buildGaussNewtonSystem(a.correspondences);
  const GaussNewtonSystem system_b = buildGaussNewtonSystem(b.correspondences);
  EXPECT_TRUE(system_a.h.isApprox(system_b.h, 0.0));
  EXPECT_TRUE(system_a.b.isApprox(system_b.b, 0.0));
}

TEST(SyntheticDegeneracyFixtures, CorridorHasExactAlongAxisDegeneracy)
{
  const CorridorFixtureConfig config;
  const DegeneracyFixture fixture = makeCorridorFixture(config);
  ASSERT_EQ(fixture.correspondences.size(), 902u);

  const GaussNewtonSystem system = buildGaussNewtonSystem(fixture.correspondences);

  // No correspondence's Jacobian has a tx (along-corridor translation)
  // entry, so H's tx column/row is exactly zero -- an algebraic, not
  // merely numeric, degeneracy.
  EXPECT_DOUBLE_EQ(system.h.col(0).norm(), 0.0);
  for (int c = 1; c < 6; ++c) {
    EXPECT_GT(system.h.col(c).norm(), 0.0) << "column " << c;
  }

  const EigenSignature signature = computeEigenSignature(system.h);
  ASSERT_TRUE(std::is_sorted(signature.values.data(), signature.values.data() + 6));

  // Exactly one near-zero eigenvalue (the along-corridor direction); the
  // remaining five are well separated from zero.
  EXPECT_NEAR(signature.values(0), 0.0, kZeroTol);
  for (int i = 1; i < 6; ++i) {
    EXPECT_GT(signature.values(i), kWellConditionedMargin) << "eigenvalue " << i;
  }

  // The degenerate eigenvalue is a simple root (not repeated) whose
  // eigenvector is exactly the along-corridor translation axis, matching
  // the roadmap's "one axis carries substantially more error" evidence
  // (docs/roadmap/v0.8.md §0) -- not a rotational (yaw) degeneracy.
  Eigen::Matrix<double, 6, 1> expected_axis = Eigen::Matrix<double, 6, 1>::Zero();
  expected_axis(0) = 1.0;
  EXPECT_NEAR(std::abs(signature.vectors.col(0).dot(expected_axis)), 1.0, 1e-9);
}

TEST(SyntheticDegeneracyFixtures, BoxIsFullyObservable)
{
  const BoxFixtureConfig config;
  const DegeneracyFixture fixture = makeBoxFixture(config);
  ASSERT_EQ(fixture.correspondences.size(), 962u);

  const GaussNewtonSystem system = buildGaussNewtonSystem(fixture.correspondences);
  for (int c = 0; c < 6; ++c) {
    EXPECT_GT(system.h.col(c).norm(), 0.0) << "column " << c;
  }

  const EigenSignature signature = computeEigenSignature(system.h);
  ASSERT_TRUE(std::is_sorted(signature.values.data(), signature.values.data() + 6));

  // All six directions well-conditioned: even the smallest eigenvalue sits
  // well above the corridor's near-zero floor.
  for (int i = 0; i < 6; ++i) {
    EXPECT_GT(signature.values(i), kWellConditionedMargin) << "eigenvalue " << i;
  }
}

TEST(SyntheticDegeneracyFixtures, SinglePlaneIsNonObservableInThreeDirections)
{
  const SinglePlaneFixtureConfig config;
  const DegeneracyFixture fixture = makeSinglePlaneFixture(config);
  ASSERT_EQ(fixture.correspondences.size(), 205u);

  const GaussNewtonSystem system = buildGaussNewtonSystem(fixture.correspondences);

  // tx, ty, and rz (yaw about the plane normal) never appear in any
  // correspondence's Jacobian for a single horizontal plane.
  EXPECT_DOUBLE_EQ(system.h.col(0).norm(), 0.0);  // tx
  EXPECT_DOUBLE_EQ(system.h.col(1).norm(), 0.0);  // ty
  EXPECT_DOUBLE_EQ(system.h.col(5).norm(), 0.0);  // rz (yaw)
  EXPECT_GT(system.h.col(2).norm(), 0.0);         // tz
  EXPECT_GT(system.h.col(3).norm(), 0.0);         // rx
  EXPECT_GT(system.h.col(4).norm(), 0.0);         // ry

  const EigenSignature signature = computeEigenSignature(system.h);
  ASSERT_TRUE(std::is_sorted(signature.values.data(), signature.values.data() + 6));

  // Three simultaneously non-observable directions, not a single one --
  // this must not collapse to a "well-conditioned" false reading (the
  // exact failure mode the Phase 1 detector must avoid, per
  // docs/roadmap/v0.8.md §4.3).
  EXPECT_NEAR(signature.values(0), 0.0, kZeroTol);
  EXPECT_NEAR(signature.values(1), 0.0, kZeroTol);
  EXPECT_NEAR(signature.values(2), 0.0, kZeroTol);
  for (int i = 3; i < 6; ++i) {
    EXPECT_GT(signature.values(i), kWellConditionedMargin) << "eigenvalue " << i;
  }
}

TEST(SyntheticDegeneracyFixtures, AlongNormalNoiseChangesResidualNotHessian)
{
  CorridorFixtureConfig noiseless_config;
  CorridorFixtureConfig noisy_config;
  noisy_config.wall_noise_sigma = 0.01;

  const DegeneracyFixture noiseless = makeCorridorFixture(noiseless_config);
  const DegeneracyFixture noisy = makeCorridorFixture(noisy_config);

  const GaussNewtonSystem noiseless_system = buildGaussNewtonSystem(noiseless.correspondences);
  const GaussNewtonSystem noisy_system = buildGaussNewtonSystem(noisy.correspondences);

  // Along-normal noise only ever adds a multiple of `normal` to `point`;
  // p x n is unaffected because n x n = 0, so H (and its eigenvalue
  // signature) must be bitwise unaffected while b picks up the residual.
  EXPECT_TRUE(noiseless_system.h.isApprox(noisy_system.h, 0.0));
  EXPECT_DOUBLE_EQ(noiseless_system.b.norm(), 0.0);
  EXPECT_GT(noisy_system.b.norm(), 0.0);

  const EigenSignature noiseless_signature = computeEigenSignature(noiseless_system.h);
  const EigenSignature noisy_signature = computeEigenSignature(noisy_system.h);
  EXPECT_TRUE(noiseless_signature.values.isApprox(noisy_signature.values, 0.0));
}

TEST(SyntheticDegeneracyFixtures, EigenSignatureCanonicalizationIsDeterministic)
{
  const CorridorFixtureConfig config;
  const DegeneracyFixture fixture = makeCorridorFixture(config);
  const GaussNewtonSystem system = buildGaussNewtonSystem(fixture.correspondences);

  const EigenSignature first = computeEigenSignature(system.h);
  const EigenSignature second = computeEigenSignature(system.h);

  EXPECT_TRUE(first.values.isApprox(second.values, 0.0));
  EXPECT_TRUE(first.vectors.isApprox(second.vectors, 0.0));

  // Canonicalization convention: each eigenvector's largest-magnitude
  // component is non-negative.
  for (int col = 0; col < 6; ++col) {
    int largest_index = 0;
    double largest_abs = std::abs(first.vectors(0, col));
    for (int row = 1; row < 6; ++row) {
      const double component_abs = std::abs(first.vectors(row, col));
      if (component_abs > largest_abs) {
        largest_abs = component_abs;
        largest_index = row;
      }
    }
    EXPECT_GE(first.vectors(largest_index, col), 0.0) << "column " << col;
  }
}

}  // namespace
