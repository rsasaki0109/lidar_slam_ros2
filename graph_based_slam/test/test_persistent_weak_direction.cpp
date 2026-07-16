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

#include "graph_based_slam/persistent_weak_direction.hpp"
#include "graph_based_slam/synthetic_degeneracy_fixtures.hpp"

namespace
{

using graphslam::degeneracy::analyzeLocalizability;
using graphslam::degeneracy::BoxFixtureConfig;
using graphslam::degeneracy::buildGaussNewtonSystem;
using graphslam::degeneracy::CorridorFixtureConfig;
using graphslam::degeneracy::LocalizabilityCategory;
using graphslam::degeneracy::LocalizabilityReport;
using graphslam::degeneracy::makeBoxFixture;
using graphslam::degeneracy::makeCorridorFixture;
using graphslam::degeneracy::makeSinglePlaneFixture;
using graphslam::degeneracy::PersistentWeakDirectionConfig;
using graphslam::degeneracy::PersistentWeakDirectionTracker;
using graphslam::degeneracy::SinglePlaneFixtureConfig;
using graphslam::degeneracy::Vector6d;

LocalizabilityReport corridorReport()
{
  const auto fixture = makeCorridorFixture(CorridorFixtureConfig());
  const auto system = buildGaussNewtonSystem(fixture.correspondences);
  return analyzeLocalizability(system.h, system.b);
}

TEST(PersistentWeakDirection, ConfirmsSameCorridorAxisAfterRequiredScans)
{
  PersistentWeakDirectionTracker tracker;
  const LocalizabilityReport report = corridorReport();

  EXPECT_FALSE(tracker.observe(report).confirmed);
  EXPECT_FALSE(tracker.observe(report).confirmed);
  const auto state = tracker.observe(report);

  EXPECT_TRUE(state.confirmed);
  EXPECT_TRUE(state.candidate_available);
  EXPECT_EQ(state.consecutive_scans, 3U);
  EXPECT_NEAR(state.matched_absolute_cosine, 1.0, 1.0e-12);
  EXPECT_NEAR(std::abs(state.axis(0)), 1.0, 1.0e-12);
}

TEST(PersistentWeakDirection, EigenvectorSignDoesNotBreakAStreak)
{
  PersistentWeakDirectionTracker tracker;
  LocalizabilityReport report = corridorReport();

  tracker.observe(report);
  for (auto & direction : report.directions) {
    if (direction.category == LocalizabilityCategory::DEGENERATE) {
      direction.eigenvector = -direction.eigenvector;
    }
  }
  const auto second = tracker.observe(report);

  EXPECT_EQ(second.consecutive_scans, 2U);
  EXPECT_NEAR(second.matched_absolute_cosine, 1.0, 1.0e-12);
  EXPECT_GT(second.axis(0), 0.0);
}

TEST(PersistentWeakDirection, AxisJumpStartsANewStreak)
{
  PersistentWeakDirectionTracker tracker;
  LocalizabilityReport report = corridorReport();
  tracker.observe(report);
  tracker.observe(report);

  for (auto & direction : report.directions) {
    if (direction.category == LocalizabilityCategory::DEGENERATE) {
      direction.eigenvector = Vector6d::Zero();
      direction.eigenvector(1) = 1.0;
    }
  }
  const auto state = tracker.observe(report);

  EXPECT_FALSE(state.confirmed);
  EXPECT_EQ(state.consecutive_scans, 1U);
  EXPECT_DOUBLE_EQ(state.matched_absolute_cosine, 0.0);
  EXPECT_DOUBLE_EQ(state.axis(1), 1.0);
}

TEST(PersistentWeakDirection, WellConditionedScanResetsTheTrack)
{
  PersistentWeakDirectionTracker tracker;
  const LocalizabilityReport corridor = corridorReport();
  tracker.observe(corridor);
  tracker.observe(corridor);

  const auto fixture = makeBoxFixture(BoxFixtureConfig());
  const auto system = buildGaussNewtonSystem(fixture.correspondences);
  const auto state = tracker.observe(analyzeLocalizability(system.h, system.b));

  EXPECT_FALSE(state.candidate_available);
  EXPECT_FALSE(state.confirmed);
  EXPECT_EQ(state.consecutive_scans, 0U);
}

TEST(PersistentWeakDirection, NonObservableSubspaceIsNeverTracked)
{
  PersistentWeakDirectionTracker tracker;
  const auto fixture = makeSinglePlaneFixture(SinglePlaneFixtureConfig());
  const auto system = buildGaussNewtonSystem(fixture.correspondences);
  const LocalizabilityReport report = analyzeLocalizability(system.h, system.b);

  EXPECT_GT(report.non_observable_count, 0);
  const auto state = tracker.observe(report);

  EXPECT_FALSE(state.candidate_available);
  EXPECT_EQ(state.consecutive_scans, 0U);
}

TEST(PersistentWeakDirection, RotationDominantCandidateIsRejected)
{
  PersistentWeakDirectionTracker tracker;
  LocalizabilityReport report = corridorReport();
  for (auto & direction : report.directions) {
    if (direction.category == LocalizabilityCategory::DEGENERATE) {
      direction.eigenvector = Vector6d::Zero();
      direction.eigenvector(5) = 1.0;
    }
  }

  const auto state = tracker.observe(report);

  EXPECT_FALSE(state.candidate_available);
  EXPECT_EQ(state.consecutive_scans, 0U);
}

TEST(PersistentWeakDirection, ConfigurationIsClampedToSafeRanges)
{
  PersistentWeakDirectionConfig config;
  config.min_consecutive_scans = 0;
  config.min_absolute_cosine = 2.0;
  config.min_translation_fraction = -1.0;
  PersistentWeakDirectionTracker tracker(config);

  const auto state = tracker.observe(corridorReport());

  EXPECT_TRUE(state.confirmed);
  EXPECT_EQ(state.consecutive_scans, 1U);
}

}  // namespace
