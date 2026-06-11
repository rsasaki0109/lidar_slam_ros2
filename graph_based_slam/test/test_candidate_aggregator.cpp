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

// Characterization tests for the candidate-generation logic extracted from
// searchLoopForLatest (distance + ScanContext sources and the shared
// upsert). These pin the historical semantics: gate boundaries, the
// first-surviving-match rule, yaw normalization, and the exact operator
// log lines.

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Core>  // NOLINT(build/include_order)

#include "graph_based_slam/candidate_aggregator.hpp"

namespace graphslam
{
namespace
{

using candidate_aggregator::Config;
using candidate_aggregator::LogLine;
using loop_verifier::LoopCandidate;

using Source = LoopCandidate::Source;

Config makeConfig()
{
  Config config;
  config.debug = false;
  config.max_loop_candidate_count = 3;
  config.distance_loop_closure = 5.0;
  config.range_of_searching_loop_closure = 10.0;
  config.scan_context_threshold = 0.5;
  return config;
}

// A descriptor whose ring key is invariant to the sector offset, so two
// offsets of the same pattern are KNN neighbors with a pure yaw shift.
ScanContext::Descriptor makePatternDescriptor(int sector_offset)
{
  ScanContext::Descriptor desc =
    ScanContext::Descriptor::Zero(ScanContext::NUM_RINGS, ScanContext::NUM_SECTORS);
  desc(2, sector_offset % ScanContext::NUM_SECTORS) = 1.0;
  desc(5, (sector_offset + 3) % ScanContext::NUM_SECTORS) = 2.0;
  return desc;
}

// A filler descriptor far from the pattern in both ring key and cosine
// distance, so it never enters the match list.
ScanContext::Descriptor makeFillerDescriptor(int seed)
{
  ScanContext::Descriptor desc =
    ScanContext::Descriptor::Zero(ScanContext::NUM_RINGS, ScanContext::NUM_SECTORS);
  desc(8, seed % ScanContext::NUM_SECTORS) = 9.0;
  desc(12, (seed + 17) % ScanContext::NUM_SECTORS) = 4.0;
  return desc;
}

TEST(CandidateAggregatorUpsert, KeepsMinimumMetricAndAdoptsLatestHints)
{
  std::vector<LoopCandidate> candidates;
  candidate_aggregator::upsertCandidate(candidates, 4, 2.0, Source::DISTANCE, 0.1);
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_DOUBLE_EQ(candidates[0].selection_metric, 2.0);

  // A worse metric does not raise the stored one, but the yaw hint is
  // always the latest.
  candidate_aggregator::upsertCandidate(candidates, 4, 3.0, Source::DISTANCE, 0.2);
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_DOUBLE_EQ(candidates[0].selection_metric, 2.0);
  EXPECT_DOUBLE_EQ(candidates[0].yaw_rad, 0.2);

  // A relative transform is adopted on update.
  Eigen::Matrix4f relative = Eigen::Matrix4f::Identity();
  relative(0, 3) = 1.5F;
  candidate_aggregator::upsertCandidate(candidates, 4, 1.0, Source::DISTANCE, 0.3, &relative);
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_DOUBLE_EQ(candidates[0].selection_metric, 1.0);
  EXPECT_TRUE(candidates[0].has_relative_transform);
  EXPECT_FLOAT_EQ(candidates[0].relative_transform(0, 3), 1.5F);

  // Negative indices are dropped.
  candidate_aggregator::upsertCandidate(candidates, -1, 0.0, Source::DISTANCE);
  EXPECT_EQ(candidates.size(), 1U);
}

TEST(CandidateAggregatorUpsert, SameIndexDifferentSourcesStaySeparate)
{
  std::vector<LoopCandidate> candidates;
  candidate_aggregator::upsertCandidate(candidates, 4, 2.0, Source::DISTANCE);
  candidate_aggregator::upsertCandidate(candidates, 4, 0.3, Source::SCAN_CONTEXT);
  ASSERT_EQ(candidates.size(), 2U);
  EXPECT_EQ(candidates[0].source, Source::DISTANCE);
  EXPECT_EQ(candidates[1].source, Source::SCAN_CONTEXT);
  EXPECT_DOUBLE_EQ(candidates[0].selection_metric, 2.0);
  EXPECT_DOUBLE_EQ(candidates[1].selection_metric, 0.3);
}

TEST(CandidateAggregatorDistance, GatesByTravelDistanceAndRange)
{
  const auto config = makeConfig();
  // latest at index 4, position origin, travelled 100 m.
  std::vector<Eigen::Vector3d> positions = {
    {3.0, 0.0, 0.0},   // in range, travelled long ago -> candidate
    {3.0, 0.0, 0.0},   // in range but only 3 m of travel separation -> gated
    {15.0, 0.0, 0.0},  // out of range -> gated
    {1.0, 0.0, 0.0},   // in range -> candidate
    {0.0, 0.0, 0.0},
  };
  std::vector<double> travels = {50.0, 97.0, 50.0, 80.0, 100.0};

  const auto distance_candidates =
    candidate_aggregator::collectDistanceCandidates(positions, travels, 4, config);
  ASSERT_EQ(distance_candidates.size(), 2U);
  // Sorted by distance ascending.
  EXPECT_EQ(distance_candidates[0].second, 3);
  EXPECT_DOUBLE_EQ(distance_candidates[0].first, 1.0);
  EXPECT_EQ(distance_candidates[1].second, 0);
  EXPECT_DOUBLE_EQ(distance_candidates[1].first, 3.0);
}

TEST(CandidateAggregatorDistance, EqualDistancesTieBreakOnLowerIndex)
{
  const auto config = makeConfig();
  std::vector<Eigen::Vector3d> positions = {
    {2.0, 0.0, 0.0},
    {-2.0, 0.0, 0.0},
    {0.0, 0.0, 0.0},
  };
  std::vector<double> travels = {0.0, 10.0, 100.0};

  const auto distance_candidates =
    candidate_aggregator::collectDistanceCandidates(positions, travels, 2, config);
  ASSERT_EQ(distance_candidates.size(), 2U);
  EXPECT_EQ(distance_candidates[0].second, 0);
  EXPECT_EQ(distance_candidates[1].second, 1);
}

TEST(CandidateAggregatorDistance, AppendTopCandidatesCapsAtMaxCount)
{
  auto config = makeConfig();
  config.max_loop_candidate_count = 2;
  const std::vector<std::pair<double, int>> distance_candidates = {
    {1.0, 7}, {2.0, 3}, {3.0, 9},
  };
  std::vector<LoopCandidate> candidates;
  candidate_aggregator::appendTopDistanceCandidates(distance_candidates, config, candidates);
  ASSERT_EQ(candidates.size(), 2U);
  EXPECT_EQ(candidates[0].index, 7);
  EXPECT_DOUBLE_EQ(candidates[0].selection_metric, 1.0);
  EXPECT_EQ(candidates[0].source, Source::DISTANCE);
  EXPECT_EQ(candidates[1].index, 3);
}

class CandidateAggregatorScanContext : public ::testing::Test
{
protected:
  // A database whose searchable head (EXCLUDE_RECENT excluded) holds the
  // pattern at ids 0 and 1, query (= back()) holds the same pattern. The
  // latest submap is id 51 with 100 m travelled.
  void buildDatabase(int id0_offset, int id1_offset)
  {
    db_.add(0, makePatternDescriptor(id0_offset));
    db_.add(1, makePatternDescriptor(id1_offset));
    for (int id = 2; id < 51; ++id) {
      db_.add(id, makeFillerDescriptor(id));
    }
    db_.add(51, makePatternDescriptor(0));
    travels_.assign(52, 0.0);
    travels_[51] = 100.0;
  }

  ScanContext::Database db_;
  std::vector<double> travels_;
  std::vector<LoopCandidate> candidates_;
  std::vector<LogLine> logs_;
};

TEST_F(CandidateAggregatorScanContext, FirstSurvivingMatchWinsAndGatedMatchLogsWhenDebug)
{
  buildDatabase(0, 0);
  // id 0 ties with id 1 at descriptor distance 0; the (distance, id) order
  // puts id 0 first, but its travel separation of 3 m gates it out.
  travels_[0] = 97.0;
  auto config = makeConfig();
  config.debug = true;

  candidate_aggregator::collectScanContextCandidate(
    db_, travels_, 51, config, candidates_, logs_);

  ASSERT_EQ(candidates_.size(), 1U);
  EXPECT_EQ(candidates_[0].index, 1);
  EXPECT_EQ(candidates_[0].source, Source::SCAN_CONTEXT);
  EXPECT_NEAR(candidates_[0].selection_metric, 0.0, 1e-9);

  ASSERT_EQ(logs_.size(), 2U);
  EXPECT_TRUE(logs_[0].via_logger);
  EXPECT_EQ(
    logs_[0].text,
    "Skip ScanContext candidate 0 because travel distance 3.000 m is below 5.000 m");
  EXPECT_FALSE(logs_[1].via_logger);
  EXPECT_EQ(logs_[1].text.rfind("ScanContext loop candidate: id=1", 0), 0U);
}

TEST_F(CandidateAggregatorScanContext, GatedSkipIsSilentWithoutDebug)
{
  buildDatabase(0, 0);
  travels_[0] = 97.0;
  const auto config = makeConfig();

  candidate_aggregator::collectScanContextCandidate(
    db_, travels_, 51, config, candidates_, logs_);

  ASSERT_EQ(candidates_.size(), 1U);
  EXPECT_EQ(candidates_[0].index, 1);
  // Only the unconditional acceptance line remains.
  ASSERT_EQ(logs_.size(), 1U);
  EXPECT_FALSE(logs_[0].via_logger);
}

TEST_F(CandidateAggregatorScanContext, SectorShiftBecomesNormalizedYawHint)
{
  // Candidate pattern shifted by 45 sectors: the raw hint of
  // -45 * 6 deg = -270 deg must normalize to +90 deg.
  buildDatabase(45, 45);
  const auto config = makeConfig();

  candidate_aggregator::collectScanContextCandidate(
    db_, travels_, 51, config, candidates_, logs_);

  ASSERT_EQ(candidates_.size(), 1U);
  EXPECT_EQ(candidates_[0].index, 0);
  EXPECT_NEAR(candidates_[0].yaw_rad, M_PI / 2.0, 1e-9);
  EXPECT_GE(candidates_[0].yaw_rad, -M_PI);
  EXPECT_LE(candidates_[0].yaw_rad, M_PI);
}

TEST_F(CandidateAggregatorScanContext, NoMatchEmitsDiagnosticOnlyWhenDebug)
{
  // No pattern in the searchable head: every candidate is a filler beyond
  // the threshold, so the match list is empty.
  db_.add(0, makeFillerDescriptor(100));
  db_.add(1, makeFillerDescriptor(101));
  for (int id = 2; id < 51; ++id) {
    db_.add(id, makeFillerDescriptor(id));
  }
  db_.add(51, makePatternDescriptor(0));
  travels_.assign(52, 0.0);
  travels_[51] = 100.0;

  auto config = makeConfig();
  candidate_aggregator::collectScanContextCandidate(
    db_, travels_, 51, config, candidates_, logs_);
  EXPECT_TRUE(candidates_.empty());
  EXPECT_TRUE(logs_.empty());

  config.debug = true;
  candidate_aggregator::collectScanContextCandidate(
    db_, travels_, 51, config, candidates_, logs_);
  EXPECT_TRUE(candidates_.empty());
  ASSERT_EQ(logs_.size(), 1U);
  EXPECT_FALSE(logs_[0].via_logger);
  EXPECT_EQ(logs_[0].text.rfind("ScanContext no match: best_sc_dist=", 0), 0U);
}

TEST_F(CandidateAggregatorScanContext, DatabaseWithinExcludeRecentWindowIsSkipped)
{
  for (int id = 0; id < ScanContext::EXCLUDE_RECENT; ++id) {
    db_.add(id, makePatternDescriptor(0));
  }
  travels_.assign(ScanContext::EXCLUDE_RECENT, 0.0);
  auto config = makeConfig();
  config.debug = true;

  candidate_aggregator::collectScanContextCandidate(
    db_, travels_, ScanContext::EXCLUDE_RECENT - 1, config, candidates_, logs_);
  EXPECT_TRUE(candidates_.empty());
  EXPECT_TRUE(logs_.empty());
}

}  // namespace
}  // namespace graphslam
