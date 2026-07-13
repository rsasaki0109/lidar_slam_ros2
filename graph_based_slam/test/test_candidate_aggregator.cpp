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
#include <Eigen/Geometry>  // NOLINT(build/include_order)

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

TEST_F(CandidateAggregatorScanContext, QueryStrideSkipsNonSelectedSubmapsDeterministically)
{
  buildDatabase(0, 0);
  auto config = makeConfig();
  config.debug = true;
  config.scan_context_query_stride = 4;

  candidate_aggregator::collectScanContextCandidate(
    db_, travels_, 51, config, candidates_, logs_);

  EXPECT_TRUE(candidates_.empty());
  ASSERT_EQ(logs_.size(), 1U);
  EXPECT_EQ(
    logs_[0].text,
    "Skip ScanContext query at submap 51 because query stride is 4");
}

TEST_F(CandidateAggregatorScanContext, ConfigurableRecentExclusionSupportsShortSequences)
{
  db_.add(0, makePatternDescriptor(0));
  for (int id = 1; id < 43; ++id) {
    db_.add(id, makeFillerDescriptor(id));
  }
  db_.add(43, makePatternDescriptor(0));
  travels_.assign(44, 0.0);
  travels_[43] = 100.0;
  auto config = makeConfig();
  config.scan_context_exclude_recent = 20;

  candidate_aggregator::collectScanContextCandidate(
    db_, travels_, 43, config, candidates_, logs_);

  ASSERT_EQ(candidates_.size(), 1U);
  EXPECT_EQ(candidates_[0].index, 0);
  EXPECT_EQ(candidates_[0].source, Source::SCAN_CONTEXT);
}

Eigen::Affine3d makeTestPose(double x, double y)
{
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  pose.translate(Eigen::Vector3d(x, y, 0.0));
  return pose;
}

// A BEV descriptor whose flattened cosine distance is 0 against the same
// spike cell and 1 against a different one.
SubmapBEVDescriptor::Descriptor makeBevDescriptor(int spike_cell)
{
  SubmapBEVDescriptor::Descriptor desc;
  desc.occupancy = Eigen::MatrixXf::Zero(4, 4);
  desc.density = Eigen::MatrixXf::Zero(4, 4);
  desc.max_height = Eigen::MatrixXf::Zero(4, 4);
  desc.occupancy(spike_cell / 4, spike_cell % 4) = 1.0F;
  desc.coarse_key = Eigen::VectorXf::Zero(1);
  return desc;
}

Config makeBevConfig()
{
  Config config;
  config.max_loop_candidate_count = 2;
  config.bev_descriptor_yaw_bins = 1;  // evaluate yaw 0 only: distances stay exact
  config.bev_descriptor_threshold = 0.5;
  config.bev_descriptor_sequence_threshold = 0.5;
  config.bev_descriptor_rerank_weight_m = 100.0;
  return config;
}

TEST(CandidateAggregatorBev, MatchingDescriptorBoostsFartherCandidateAheadOfNearer)
{
  // Submap 0 (5 m away) matches the latest descriptor, submap 1 (3 m away)
  // does not: the hint shifts 0 ahead of 1 in the reranked order and both
  // end up as DISTANCE candidates, the boosted one carrying the hint yaw.
  SubmapBEVDescriptor::Database db;
  db.add(0, makeBevDescriptor(0));
  db.add(1, makeBevDescriptor(5));
  db.add(2, makeBevDescriptor(0));
  const std::vector<Eigen::Affine3d> poses = {
    makeTestPose(5.0, 0.0), makeTestPose(3.0, 0.0), makeTestPose(0.0, 0.0)};
  std::vector<std::pair<double, int>> distance_candidates = {{3.0, 1}, {5.0, 0}};
  std::vector<LoopCandidate> candidates;
  std::vector<LogLine> logs;

  candidate_aggregator::rerankDistanceCandidatesWithBev(
    db, poses, 2, makeBevConfig(), distance_candidates, candidates, logs);

  ASSERT_EQ(distance_candidates.size(), 2U);
  EXPECT_EQ(distance_candidates[0].second, 0);
  EXPECT_EQ(distance_candidates[1].second, 1);

  ASSERT_EQ(candidates.size(), 2U);
  EXPECT_EQ(candidates[0].index, 0);
  EXPECT_EQ(candidates[0].source, Source::DISTANCE);
  // adjusted = 5.0 + 100 * (0 - 0.5)
  EXPECT_NEAR(candidates[0].selection_metric, -45.0, 1e-9);
  EXPECT_EQ(candidates[1].index, 1);
  EXPECT_DOUBLE_EQ(candidates[1].selection_metric, 3.0);

  ASSERT_EQ(logs.size(), 2U);
  EXPECT_EQ(logs[0].text.rfind("BEV rerank hint: id=0", 0), 0U);
  EXPECT_EQ(logs[1].text.rfind("Distance candidate reranked by BEV: id=0", 0), 0U);
}

TEST(CandidateAggregatorBev, EuclideanGateSkipsAndReportsNoCandidateWhenDebug)
{
  SubmapBEVDescriptor::Database db;
  db.add(0, makeBevDescriptor(0));
  db.add(1, makeBevDescriptor(0));
  const std::vector<Eigen::Affine3d> poses = {
    makeTestPose(5.0, 0.0), makeTestPose(0.0, 0.0)};
  std::vector<std::pair<double, int>> distance_candidates = {{5.0, 0}};
  std::vector<LoopCandidate> candidates;
  std::vector<LogLine> logs;
  auto config = makeBevConfig();
  config.debug = true;
  config.bev_descriptor_max_euclidean_distance_m = 4.0;

  candidate_aggregator::rerankDistanceCandidatesWithBev(
    db, poses, 1, config, distance_candidates, candidates, logs);

  // No hint: the candidate still appears via the plain top-N add.
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_DOUBLE_EQ(candidates[0].selection_metric, 5.0);
  EXPECT_DOUBLE_EQ(candidates[0].yaw_rad, 0.0);

  ASSERT_EQ(logs.size(), 2U);
  EXPECT_TRUE(logs[0].via_logger);
  EXPECT_EQ(
    logs[0].text,
    "Skip BEV candidate 0 because euclidean distance 5.000 m exceeds 4.000 m");
  EXPECT_FALSE(logs[1].via_logger);
  EXPECT_EQ(logs[1].text.rfind("BEV rerank no candidate: best_idx=-1", 0), 0U);
}

TEST(CandidateAggregatorBev, SequenceWindowAveragesAndGatesAtThreshold)
{
  // Candidate 2 matches at the head (distance 0) but its window-1
  // predecessor mismatches (distance 1): the averaged metric 0.5 hits the
  // >= 0.5 gate exactly and the hint is rejected.
  SubmapBEVDescriptor::Database db;
  db.add(0, makeBevDescriptor(0));
  db.add(1, makeBevDescriptor(5));
  db.add(2, makeBevDescriptor(0));
  db.add(3, makeBevDescriptor(0));
  const std::vector<Eigen::Affine3d> poses = {
    makeTestPose(9.0, 0.0), makeTestPose(10.0, 0.0),
    makeTestPose(10.0, 1.0), makeTestPose(0.0, 0.0)};
  std::vector<std::pair<double, int>> distance_candidates = {{10.0, 2}};
  std::vector<LoopCandidate> candidates;
  std::vector<LogLine> logs;
  auto config = makeBevConfig();
  config.debug = true;
  config.bev_descriptor_sequence_window = 1;

  candidate_aggregator::rerankDistanceCandidatesWithBev(
    db, poses, 3, config, distance_candidates, candidates, logs);

  ASSERT_GE(logs.size(), 1U);
  EXPECT_EQ(
    logs[0].text,
    "Skip BEV candidate 2 because sequence metric 0.500 exceeds 0.500");
}

TEST(CandidateAggregatorBev, PoseConsistencyGateComparesTrajectoryDeltas)
{
  // Descriptors match through the window, but the relative motion to the
  // window-1 predecessor differs between the query and candidate tracks,
  // so the pose-consistency gate rejects the hint.
  SubmapBEVDescriptor::Database db;
  for (int i = 0; i < 4; ++i) {
    db.add(i, makeBevDescriptor(0));
  }
  const std::vector<Eigen::Affine3d> poses = {
    makeTestPose(9.0, 0.0), makeTestPose(9.0, 0.0),
    makeTestPose(10.0, 0.0), makeTestPose(0.0, 0.0)};
  std::vector<std::pair<double, int>> distance_candidates = {{10.0, 2}};
  std::vector<LoopCandidate> candidates;
  std::vector<LogLine> logs;
  auto config = makeBevConfig();
  config.debug = true;
  config.bev_descriptor_sequence_window = 1;
  config.bev_descriptor_pose_consistency_threshold_m = 1.0;

  candidate_aggregator::rerankDistanceCandidatesWithBev(
    db, poses, 3, config, distance_candidates, candidates, logs);

  // query delta (latest -> poses[2]) is (10, 0); candidate delta
  // (candidate 2 -> poses[1]) is (-1, 0): 2-D mismatch 11 m >= 1 m.
  ASSERT_GE(logs.size(), 1U);
  EXPECT_EQ(
    logs[0].text,
    "Skip BEV candidate 2 because pose consistency 11.000 m exceeds 1.000 m");
}

SolidDescriptor::Descriptor makeSolidDescriptor(double range0, double range1)
{
  SolidDescriptor::Descriptor desc;
  desc.range = Eigen::VectorXd::Zero(4);
  desc.range(0) = range0;
  desc.range(1) = range1;
  desc.angle = Eigen::VectorXd::Zero(4);
  desc.angle(0) = 1.0;
  desc.solid = Eigen::VectorXd::Zero(8);
  return desc;
}

class CandidateAggregatorSolid : public ::testing::Test
{
protected:
  // 52 entries: ids 0 and 1 hold the given patterns, fillers are
  // orthogonal to the query, the query (id 51) is (1, 0).
  void buildDatabase(double id0_range0, double id0_range1)
  {
    db_.add(0, makeSolidDescriptor(id0_range0, id0_range1));
    db_.add(1, makeSolidDescriptor(0.0, 1.0));
    for (int id = 2; id < 51; ++id) {
      db_.add(id, makeSolidDescriptor(0.0, 1.0));
    }
    db_.add(51, makeSolidDescriptor(1.0, 0.0));
    poses_.assign(52, Eigen::Affine3d::Identity());
  }

  SolidDescriptor::Database db_;
  std::vector<Eigen::Affine3d> poses_;
  std::vector<LoopCandidate> candidates_;
  std::vector<LogLine> logs_;
};

TEST_F(CandidateAggregatorSolid, HighSimilarityCandidateIsAddedWithSequenceMetric)
{
  buildDatabase(1.0, 0.0);  // identical to the query: similarity 1
  auto config = makeConfig();
  config.solid_descriptor_min_similarity = 0.7;
  const std::vector<std::pair<double, int>> distance_candidates = {{1.0, 0}};

  candidate_aggregator::collectSolidCandidates(
    db_, poses_, distance_candidates, 51, config, candidates_, logs_);

  ASSERT_EQ(candidates_.size(), 1U);
  EXPECT_EQ(candidates_[0].index, 0);
  EXPECT_EQ(candidates_[0].source, Source::SOLID_DESCRIPTOR);
  // selection metric is 1 - sequence similarity (window 0 -> plain
  // similarity 1).
  EXPECT_NEAR(candidates_[0].selection_metric, 0.0, 1e-12);
  ASSERT_EQ(logs_.size(), 1U);
  EXPECT_EQ(logs_[0].text.rfind("SOLiD rerank candidate: id=0", 0), 0U);
}

TEST_F(CandidateAggregatorSolid, LowSimilarityIsGatedAndReportedWhenDebug)
{
  buildDatabase(0.0, 1.0);  // orthogonal to the query: similarity 0
  auto config = makeConfig();
  config.debug = true;
  config.solid_descriptor_min_similarity = 0.7;
  const std::vector<std::pair<double, int>> distance_candidates = {{1.0, 0}};

  candidate_aggregator::collectSolidCandidates(
    db_, poses_, distance_candidates, 51, config, candidates_, logs_);

  EXPECT_TRUE(candidates_.empty());
  ASSERT_EQ(logs_.size(), 2U);
  EXPECT_TRUE(logs_[0].via_logger);
  EXPECT_EQ(
    logs_[0].text,
    "Skip SOLiD candidate 0 because similarity 0.000 is below 0.700");
  EXPECT_FALSE(logs_[1].via_logger);
  EXPECT_EQ(logs_[1].text.rfind("SOLiD rerank no candidate: best_idx=0", 0), 0U);
}

TEST_F(CandidateAggregatorSolid, DatabaseWithinExcludeRecentWindowIsSkipped)
{
  for (int id = 0; id < SolidDescriptor::DEFAULT_EXCLUDE_RECENT; ++id) {
    db_.add(id, makeSolidDescriptor(1.0, 0.0));
  }
  poses_.assign(SolidDescriptor::DEFAULT_EXCLUDE_RECENT, Eigen::Affine3d::Identity());
  auto config = makeConfig();
  config.debug = true;
  const std::vector<std::pair<double, int>> distance_candidates = {{1.0, 0}};

  candidate_aggregator::collectSolidCandidates(
    db_, poses_, distance_candidates,
    SolidDescriptor::DEFAULT_EXCLUDE_RECENT - 1, config, candidates_, logs_);
  EXPECT_TRUE(candidates_.empty());
  EXPECT_TRUE(logs_.empty());
}

// A 4x4 grid plus an off-axis marker: distinct pairwise distances give
// unambiguous triangles and a unique SE(3) (same shape as the
// triangle-database tests). `spacing` changes the shape entirely, so two
// different spacings never share hash votes.
std::vector<graphslam::triangle::Keypoint> makeTriangleKeypoints(float spacing)
{
  std::vector<graphslam::triangle::Keypoint> kps;
  for (int iy = 0; iy < 4; ++iy) {
    for (int ix = 0; ix < 4; ++ix) {
      graphslam::triangle::Keypoint k;
      k.position = Eigen::Vector3f(
        static_cast<float>(ix) * spacing,
        static_cast<float>(iy) * spacing,
        0.0F);
      k.salience = 1.0F;
      kps.push_back(k);
    }
  }
  graphslam::triangle::Keypoint marker;
  marker.position = Eigen::Vector3f(4.5F * spacing, 2.0F * spacing, 0.0F);
  marker.salience = 1.0F;
  kps.push_back(marker);
  return kps;
}

class CandidateAggregatorTriangle : public ::testing::Test
{
protected:
  using Features = candidate_aggregator::TriangleSubmapFeatures;

  void SetUp() override
  {
    config_ = makeConfig();
    config_.triangle_descriptor_exclude_recent = 1;
    config_.triangle_descriptor_edge_bin_m = 0.2;
    config_.triangle_descriptor_quad_feature_bin_m = 0.2;
    config_.triangle_descriptor_inlier_translation_m = 0.3;
    config_.triangle_descriptor_inlier_rotation_deg = 5.0;
    config_.triangle_descriptor_min_inliers = 4;
    config_.triangle_descriptor_max_pairs = 300;
    config_.triangle_descriptor_fourth_point_max_distance_m = 1.0;
    config_.triangle_descriptor_min_votes = 1;

    hash_cfg_.edge_bin_m = 0.2F;
    hash_cfg_.quad_feature_bin_m = 0.2F;

    pattern_.keypoints = makeTriangleKeypoints(3.0F);
    pattern_.triangles = graphslam::triangle::buildTriangles(
      pattern_.keypoints, graphslam::triangle::TriangleBuildConfig{});
    filler_.keypoints = makeTriangleKeypoints(2.3F);
    filler_.triangles = graphslam::triangle::buildTriangles(
      filler_.keypoints, graphslam::triangle::TriangleBuildConfig{});

    travels_ = {0.0, 50.0, 100.0};
  }

  // db submaps 0 and 1, query = per_submap[2].
  void buildDatabase(const Features & id0, const Features & id1, const Features & query)
  {
    per_submap_ = {id0, id1, query};
    db_.addSubmap(0, id0.keypoints, id0.triangles, hash_cfg_);
    db_.addSubmap(1, id1.keypoints, id1.triangles, hash_cfg_);
  }

  Config config_;
  graphslam::triangle::HashConfig hash_cfg_;
  Features pattern_;
  Features filler_;
  graphslam::triangle::TriangleDatabase db_;
  std::vector<Features> per_submap_;
  SubmapBEVDescriptor::Database bev_db_;
  std::vector<double> travels_;
  std::vector<LoopCandidate> candidates_;
  std::vector<LogLine> logs_;
};

TEST_F(CandidateAggregatorTriangle, AcceptedCandidateCarriesRecoveredSe3)
{
  buildDatabase(pattern_, filler_, pattern_);

  candidate_aggregator::collectTriangleCandidate(
    db_, per_submap_, bev_db_, false, travels_, 2, config_, candidates_, logs_);

  ASSERT_EQ(candidates_.size(), 1U);
  EXPECT_EQ(candidates_[0].index, 0);
  EXPECT_EQ(candidates_[0].source, Source::TRIANGLE_DESCRIPTOR);
  ASSERT_TRUE(candidates_[0].has_relative_transform);
  // Identical keypoint sets: the recovered SE(3) is the identity.
  EXPECT_TRUE(candidates_[0].relative_transform.isApprox(Eigen::Matrix4f::Identity(), 1e-3F));
  EXPECT_LT(candidates_[0].selection_metric, 1.0);
  ASSERT_EQ(logs_.size(), 1U);
  EXPECT_FALSE(logs_[0].via_logger);
  EXPECT_EQ(logs_[0].text.rfind("Triangle loop candidate: id=0", 0), 0U);
}

TEST_F(CandidateAggregatorTriangle, TravelGateRejectsAndLogsWhenDebug)
{
  buildDatabase(pattern_, filler_, pattern_);
  travels_[0] = 98.0;  // 2 m of travel separation <= 5 m gate
  config_.debug = true;

  candidate_aggregator::collectTriangleCandidate(
    db_, per_submap_, bev_db_, false, travels_, 2, config_, candidates_, logs_);

  EXPECT_TRUE(candidates_.empty());
  ASSERT_EQ(logs_.size(), 1U);
  EXPECT_TRUE(logs_[0].via_logger);
  EXPECT_EQ(logs_[0].text, "Skip Triangle candidate 0 (travel 2.000 m <= 5.000 m)");
}

TEST_F(CandidateAggregatorTriangle, MinVotesGateReportsTopVoteWhenDebug)
{
  buildDatabase(pattern_, filler_, pattern_);
  config_.triangle_descriptor_min_votes = 999999;
  config_.debug = true;

  candidate_aggregator::collectTriangleCandidate(
    db_, per_submap_, bev_db_, false, travels_, 2, config_, candidates_, logs_);

  EXPECT_TRUE(candidates_.empty());
  ASSERT_EQ(logs_.size(), 1U);
  EXPECT_TRUE(logs_[0].via_logger);
  EXPECT_EQ(logs_[0].text.rfind("Triangle top vote 0 only ", 0), 0U);
}

TEST_F(CandidateAggregatorTriangle, ExcludeRecentMasksTheMatchingNeighbor)
{
  // The pattern sits at id 2, inside the widened exclusion window
  // (latest 3 - id 2 < 2): its votes are masked and the non-matching
  // fillers got none, so the debug diagnostic reports the excluded top
  // vote and nothing is added.
  Features filler2;
  filler2.keypoints = makeTriangleKeypoints(1.7F);
  filler2.triangles = graphslam::triangle::buildTriangles(
    filler2.keypoints, graphslam::triangle::TriangleBuildConfig{});
  per_submap_ = {filler_, filler2, pattern_, pattern_};
  db_.addSubmap(0, filler_.keypoints, filler_.triangles, hash_cfg_);
  db_.addSubmap(1, filler2.keypoints, filler2.triangles, hash_cfg_);
  db_.addSubmap(2, pattern_.keypoints, pattern_.triangles, hash_cfg_);
  travels_ = {0.0, 30.0, 60.0, 100.0};
  config_.triangle_descriptor_exclude_recent = 2;
  config_.debug = true;

  candidate_aggregator::collectTriangleCandidate(
    db_, per_submap_, bev_db_, false, travels_, 3, config_, candidates_, logs_);

  EXPECT_TRUE(candidates_.empty());
  ASSERT_EQ(logs_.size(), 1U);
  EXPECT_EQ(logs_[0].text.rfind("Triangle top vote 2 only ", 0), 0U);
}

}  // namespace
}  // namespace graphslam
