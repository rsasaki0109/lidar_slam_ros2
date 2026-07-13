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

#include <gtest/gtest.h>

#include <limits>

#include "graph_based_slam/degeneracy_aware_solve.hpp"
#include "graph_based_slam/synthetic_degeneracy_fixtures.hpp"
#include "rko_lio/core/degeneracy_aware_solve.hpp"

namespace
{

using graphslam::degeneracy::BoxFixtureConfig;
using graphslam::degeneracy::buildGaussNewtonSystem;
using graphslam::degeneracy::CorridorFixtureConfig;
using graphslam::degeneracy::DegeneracyAwareSolveConfig;
using graphslam::degeneracy::GaussNewtonSystem;
using graphslam::degeneracy::makeBoxFixture;
using graphslam::degeneracy::makeCorridorFixture;
using graphslam::degeneracy::makeSinglePlaneFixture;
using graphslam::degeneracy::Matrix6d;
using graphslam::degeneracy::solveDegeneracyAware;
using graphslam::degeneracy::SinglePlaneFixtureConfig;
using graphslam::degeneracy::Vector6d;

TEST(DegeneracyAwareSolve, FullyObservableBoxMatchesGaussNewtonSolution)
{
  const GaussNewtonSystem system =
    buildGaussNewtonSystem(makeBoxFixture(BoxFixtureConfig()).correspondences);
  Vector6d expected;
  expected << 0.12, -0.08, 0.03, 0.01, -0.02, 0.04;
  const Vector6d b = -system.h * expected;
  const Vector6d prior = Vector6d::Constant(0.5);

  const auto result = solveDegeneracyAware(system.h, b, prior);

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.used_prior);
  EXPECT_EQ(result.localizability.well_conditioned_count, 6);
  EXPECT_TRUE(result.update.isApprox(expected, 1.0e-10));
}

TEST(DegeneracyAwareSolve, CorridorUsesFrozenPriorFraction)
{
  const GaussNewtonSystem system =
    buildGaussNewtonSystem(makeCorridorFixture(CorridorFixtureConfig()).correspondences);
  Vector6d observable_update;
  observable_update << 0.0, -0.08, 0.03, 0.01, -0.02, 0.04;
  const Vector6d b = -system.h * observable_update;
  Vector6d prior = Vector6d::Zero();
  prior(0) = 0.25;

  const auto result = solveDegeneracyAware(system.h, b, prior);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.used_prior);
  EXPECT_EQ(result.localizability.degenerate_count, 1);
  EXPECT_NEAR(result.update(0), 0.25 * prior(0), 1.0e-12);
  EXPECT_TRUE(result.update.tail<5>().isApprox(observable_update.tail<5>(), 1.0e-10));
}

TEST(DegeneracyAwareSolve, PriorWeightControlsRecoveryFraction)
{
  const GaussNewtonSystem system =
    buildGaussNewtonSystem(makeCorridorFixture(CorridorFixtureConfig()).correspondences);
  Vector6d prior = Vector6d::Zero();
  prior(0) = 0.4;
  DegeneracyAwareSolveConfig config;
  config.degenerate_prior_weight = 0.75;

  const auto result = solveDegeneracyAware(system.h, Vector6d::Zero(), prior, config);

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.update(0), 0.3, 1.0e-12);
}

TEST(DegeneracyAwareSolve, NonObservableSubspaceDoesNotMove)
{
  const GaussNewtonSystem system =
    buildGaussNewtonSystem(makeSinglePlaneFixture(SinglePlaneFixtureConfig()).correspondences);
  Vector6d prior = Vector6d::Zero();
  prior(0) = 0.2;
  prior(1) = -0.3;
  prior(5) = 0.1;

  const auto result = solveDegeneracyAware(system.h, Vector6d::Zero(), prior);

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.used_prior);
  EXPECT_EQ(result.localizability.non_observable_count, 3);
  EXPECT_DOUBLE_EQ(result.update.norm(), 0.0);
}

TEST(DegeneracyAwareSolve, InvalidInputFailsClosed)
{
  Matrix6d h = Matrix6d::Identity();
  h(0, 0) = std::numeric_limits<double>::quiet_NaN();

  const auto result = solveDegeneracyAware(h, Vector6d::Zero(), Vector6d::Zero());

  EXPECT_FALSE(result.valid);
  EXPECT_DOUBLE_EQ(result.update.norm(), 0.0);
}

TEST(DegeneracyAwareSolve, SameInputsAreBitwiseDeterministic)
{
  const GaussNewtonSystem system =
    buildGaussNewtonSystem(makeCorridorFixture(CorridorFixtureConfig()).correspondences);
  Vector6d prior = Vector6d::Zero();
  prior(0) = 0.25;

  const auto first = solveDegeneracyAware(system.h, system.b, prior);
  const auto second = solveDegeneracyAware(system.h, system.b, prior);

  EXPECT_EQ(first.update, second.update);
  EXPECT_EQ(first.used_prior, second.used_prior);
  EXPECT_EQ(first.valid, second.valid);
}

TEST(DegeneracyAwareSolve, RkoLioIntegrationMatchesSharedReference)
{
  const GaussNewtonSystem system =
    buildGaussNewtonSystem(makeCorridorFixture(CorridorFixtureConfig()).correspondences);
  Vector6d observable_update;
  observable_update << 0.0, -0.08, 0.03, 0.01, -0.02, 0.04;
  const Vector6d b = -system.h * observable_update;
  Vector6d prior = Vector6d::Zero();
  prior(0) = 0.25;

  const auto reference = solveDegeneracyAware(system.h, b, prior);
  const auto integrated = rko_lio::core::solve_degeneracy_aware(system.h, b, prior);

  ASSERT_TRUE(reference.valid);
  ASSERT_TRUE(integrated.valid);
  EXPECT_EQ(integrated.degenerate_count, reference.localizability.degenerate_count);
  EXPECT_EQ(integrated.non_observable_count, reference.localizability.non_observable_count);
  EXPECT_EQ(integrated.used_prior, reference.used_prior);
  EXPECT_TRUE(integrated.update.isApprox(reference.update, 1.0e-12));
}

}  // namespace
