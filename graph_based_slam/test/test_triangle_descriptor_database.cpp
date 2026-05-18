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
#include <array>
#include <cmath>
#include <random>
#include <vector>

#include "graph_based_slam/triangle_descriptor_database.hpp"

using graphslam::triangle::DatabaseEntry;
using graphslam::triangle::HashConfig;
using graphslam::triangle::Keypoint;
using graphslam::triangle::TriangleBuildConfig;
using graphslam::triangle::TriangleDatabase;
using graphslam::triangle::TriangleDescriptor;
using graphslam::triangle::TriangleHash;
using graphslam::triangle::VerificationConfig;
using graphslam::triangle::VoteConfig;
using graphslam::triangle::accumulateVotes;
using graphslam::triangle::buildTriangles;
using graphslam::triangle::estimateRigidFromTriangle;
using graphslam::triangle::findLoopCandidate;
using graphslam::triangle::packHash;
using graphslam::triangle::quantizeEdges;

namespace
{

std::vector<Keypoint> makeKeypointGrid(int n_x, int n_y, float spacing)
{
  std::vector<Keypoint> kps;
  kps.reserve(static_cast<std::size_t>(n_x * n_y));
  for (int iy = 0; iy < n_y; ++iy) {
    for (int ix = 0; ix < n_x; ++ix) {
      Keypoint k;
      k.position = Eigen::Vector3f(
        static_cast<float>(ix) * spacing,
        static_cast<float>(iy) * spacing,
        0.0f);
      k.salience = 1.0f;
      kps.push_back(k);
    }
  }
  return kps;
}

// A regular grid has 180° rotational symmetry, so two transforms (T_gt and
// T_gt * R180) are both valid loop-edge solutions. Break the symmetry by
// adding an off-axis marker so SE(3) recovery has a unique answer.
std::vector<Keypoint> makeAsymmetricKeypointSet()
{
  auto kps = makeKeypointGrid(4, 4, 3.0f);
  Keypoint marker;
  marker.position = Eigen::Vector3f(13.5f, 6.0f, 0.0f);
  marker.salience = 1.0f;
  kps.push_back(marker);
  return kps;
}

std::vector<Keypoint> transformKeypoints(
  const std::vector<Keypoint> & src, const Eigen::Matrix4f & T)
{
  std::vector<Keypoint> out;
  out.reserve(src.size());
  for (const auto & k : src) {
    Keypoint kt = k;
    const Eigen::Vector4f hp(k.position.x(), k.position.y(), k.position.z(), 1.0f);
    const Eigen::Vector4f tp = T * hp;
    kt.position = tp.head<3>();
    out.push_back(kt);
  }
  return out;
}

TriangleDescriptor makeTriangle(
  const std::vector<Keypoint> & kps, int i, int j, int k)
{
  const float l_ij = (kps[i].position - kps[j].position).norm();
  const float l_jk = (kps[j].position - kps[k].position).norm();
  const float l_ik = (kps[i].position - kps[k].position).norm();
  struct EdgeRef
  {
    float length;
    int opposite_kp;
  };
  std::array<EdgeRef, 3> e = {{
    {l_jk, i}, {l_ik, j}, {l_ij, k},
  }};
  std::sort(
    e.begin(), e.end(),
    [](const EdgeRef & a, const EdgeRef & b) {return a.length < b.length;});
  TriangleDescriptor t;
  t.edges = {{e[0].length, e[1].length, e[2].length}};
  t.keypoint_ids = {{e[0].opposite_kp, e[1].opposite_kp, e[2].opposite_kp}};
  return t;
}

}  // namespace

TEST(TriangleHash, QuantizesEdgesIntoExpectedBins)
{
  TriangleDescriptor t;
  t.edges = {{3.4f, 5.0f, 7.9f}};
  HashConfig cfg;
  cfg.edge_bin_m = 1.0f;
  cfg.max_bin = 255;
  const auto h = quantizeEdges(t, cfg);
  EXPECT_EQ(h.le, 3);
  EXPECT_EQ(h.me, 5);
  EXPECT_EQ(h.ge, 7);
}

TEST(TriangleHash, IsDeterministic)
{
  TriangleDescriptor t;
  t.edges = {{4.1f, 6.2f, 8.3f}};
  HashConfig cfg;
  cfg.edge_bin_m = 1.0f;
  EXPECT_EQ(packHash(quantizeEdges(t, cfg)), packHash(quantizeEdges(t, cfg)));
}

TEST(TriangleHash, RobustToSmallEdgeNoise)
{
  TriangleDescriptor t1;
  t1.edges = {{3.2f, 5.4f, 7.7f}};
  TriangleDescriptor t2;
  t2.edges = {{3.8f, 5.1f, 7.2f}};
  HashConfig cfg;
  cfg.edge_bin_m = 1.0f;
  EXPECT_EQ(quantizeEdges(t1, cfg), quantizeEdges(t2, cfg));
}

TEST(TriangleHash, ClipsAtMaxBin)
{
  TriangleDescriptor t;
  t.edges = {{500.0f, 600.0f, 700.0f}};
  HashConfig cfg;
  cfg.edge_bin_m = 1.0f;
  cfg.max_bin = 255;
  const auto h = quantizeEdges(t, cfg);
  EXPECT_EQ(h.le, 255);
  EXPECT_EQ(h.me, 255);
  EXPECT_EQ(h.ge, 255);
}

TEST(TriangleDatabase, AddSubmapStoresAllValidTriangles)
{
  const auto kps = makeKeypointGrid(4, 4, 3.0f);
  TriangleBuildConfig build_cfg;
  const auto tris = buildTriangles(kps, build_cfg);
  ASSERT_FALSE(tris.empty());

  TriangleDatabase db;
  HashConfig cfg;
  db.addSubmap(7, kps, tris, cfg);
  EXPECT_EQ(db.submapCount(), 1U);
  EXPECT_EQ(db.triangleCount(), tris.size());
}

TEST(TriangleDatabase, AddSubmapSkipsInvalidKeypointIds)
{
  const auto kps = makeKeypointGrid(3, 3, 2.0f);
  TriangleDescriptor bad;
  bad.edges = {{2.0f, 2.0f, 2.83f}};
  bad.keypoint_ids = {{0, 1, 99}};  // out of range
  TriangleDescriptor good;
  good.edges = {{2.0f, 2.0f, 2.83f}};
  good.keypoint_ids = {{0, 1, 3}};

  TriangleDatabase db;
  HashConfig cfg;
  db.addSubmap(1, kps, {bad, good}, cfg);
  EXPECT_EQ(db.triangleCount(), 1U);
}

TEST(TriangleDatabase, LookupReturnsAddedEntries)
{
  const auto kps = makeKeypointGrid(3, 3, 2.0f);
  TriangleDescriptor t = makeTriangle(kps, 0, 1, 3);

  TriangleDatabase db;
  HashConfig cfg;
  db.addSubmap(42, kps, {t}, cfg);

  const auto & bucket = db.lookup(quantizeEdges(t, cfg));
  ASSERT_EQ(bucket.size(), 1U);
  EXPECT_EQ(bucket[0].submap_id, 42);
}

TEST(TriangleVotes, ReturnsHighestForExactMatch)
{
  const auto kps = makeKeypointGrid(4, 4, 3.0f);
  TriangleBuildConfig build_cfg;
  const auto tris = buildTriangles(kps, build_cfg);
  ASSERT_GE(tris.size(), 10U);

  TriangleDatabase db;
  HashConfig cfg;
  db.addSubmap(3, kps, tris, cfg);

  VoteConfig vote_cfg;
  const auto votes = accumulateVotes(db, tris, cfg, vote_cfg);
  ASSERT_FALSE(votes.empty());
  EXPECT_EQ(votes.front().submap_id, 3);
  EXPECT_GT(votes.front().votes, 0);
}

TEST(TriangleVotes, ExcludesRequestedSubmap)
{
  const auto kps = makeKeypointGrid(4, 4, 3.0f);
  const auto tris = buildTriangles(kps, TriangleBuildConfig{});

  TriangleDatabase db;
  HashConfig cfg;
  db.addSubmap(5, kps, tris, cfg);

  VoteConfig vote_cfg;
  vote_cfg.exclude_submap_id = 5;
  const auto votes = accumulateVotes(db, tris, cfg, vote_cfg);
  for (const auto & v : votes) {
    EXPECT_NE(v.submap_id, 5);
  }
}

TEST(TriangleVotes, MultiSubmapPicksMostSimilar)
{
  const auto kps_a = makeKeypointGrid(4, 4, 3.0f);
  TriangleBuildConfig build_cfg;
  const auto tris_a = buildTriangles(kps_a, build_cfg);

  // Submap B: same shape (so it should win) at a different absolute position.
  Eigen::Matrix4f T_b = Eigen::Matrix4f::Identity();
  T_b.block<3, 1>(0, 3) = Eigen::Vector3f(50.0f, 0.0f, 0.0f);
  const auto kps_b = transformKeypoints(kps_a, T_b);
  const auto tris_b = buildTriangles(kps_b, build_cfg);

  // Submap C: completely different geometry (sparse 3-point setup).
  std::vector<Keypoint> kps_c;
  kps_c.push_back({Eigen::Vector3f(0.0f, 0.0f, 0.0f), 1.0f});
  kps_c.push_back({Eigen::Vector3f(40.0f, 0.0f, 0.0f), 1.0f});
  kps_c.push_back({Eigen::Vector3f(40.0f, 40.0f, 0.0f), 1.0f});
  kps_c.push_back({Eigen::Vector3f(0.0f, 40.0f, 0.0f), 1.0f});
  const auto tris_c = buildTriangles(kps_c, build_cfg);

  TriangleDatabase db;
  HashConfig cfg;
  db.addSubmap(101, kps_b, tris_b, cfg);
  db.addSubmap(202, kps_c, tris_c, cfg);

  VoteConfig vote_cfg;
  const auto votes = accumulateVotes(db, tris_a, cfg, vote_cfg);
  ASSERT_FALSE(votes.empty());
  EXPECT_EQ(votes.front().submap_id, 101);
}

TEST(TriangleVotes, HoldsUpUnderRotation)
{
  const auto kps_a = makeKeypointGrid(4, 4, 3.0f);
  const auto tris_a = buildTriangles(kps_a, TriangleBuildConfig{});

  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  const float yaw = static_cast<float>(M_PI) * 0.37f;
  T.block<3, 3>(0, 0) = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  T.block<3, 1>(0, 3) = Eigen::Vector3f(12.0f, -7.0f, 0.0f);
  const auto kps_b = transformKeypoints(kps_a, T);
  const auto tris_b = buildTriangles(kps_b, TriangleBuildConfig{});

  TriangleDatabase db;
  HashConfig cfg;
  db.addSubmap(9, kps_b, tris_b, cfg);

  VoteConfig vote_cfg;
  const auto votes = accumulateVotes(db, tris_a, cfg, vote_cfg);
  ASSERT_FALSE(votes.empty());
  EXPECT_EQ(votes.front().submap_id, 9);
}

TEST(TriangleLoopCandidate, RecoversIdentity)
{
  const auto kps = makeKeypointGrid(4, 4, 3.0f);
  const auto tris = buildTriangles(kps, TriangleBuildConfig{});

  TriangleDatabase db;
  HashConfig cfg;
  db.addSubmap(11, kps, tris, cfg);

  VoteConfig vote_cfg;
  VerificationConfig verify_cfg;
  const auto candidate = findLoopCandidate(db, kps, tris, cfg, vote_cfg, verify_cfg);
  ASSERT_TRUE(candidate.accepted);
  EXPECT_EQ(candidate.submap_id, 11);
  EXPECT_GT(candidate.inliers, verify_cfg.min_inliers);
  const Eigen::Matrix4f T = candidate.transform;
  const Eigen::Matrix3f R = T.block<3, 3>(0, 0);
  const Eigen::Vector3f t = T.block<3, 1>(0, 3);
  EXPECT_LT((R - Eigen::Matrix3f::Identity()).norm(), 1e-3f);
  EXPECT_LT(t.norm(), 1e-3f);
}

TEST(TriangleLoopCandidate, RecoversYawAndTranslation)
{
  const auto kps_a = makeAsymmetricKeypointSet();
  const auto tris_a = buildTriangles(kps_a, TriangleBuildConfig{});

  Eigen::Matrix4f T_gt = Eigen::Matrix4f::Identity();
  const float yaw = static_cast<float>(M_PI) * 0.21f;
  T_gt.block<3, 3>(0, 0) = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  T_gt.block<3, 1>(0, 3) = Eigen::Vector3f(8.0f, 4.5f, 0.0f);
  const auto kps_b = transformKeypoints(kps_a, T_gt);
  const auto tris_b = buildTriangles(kps_b, TriangleBuildConfig{});

  TriangleDatabase db;
  HashConfig cfg;
  db.addSubmap(77, kps_b, tris_b, cfg);

  VoteConfig vote_cfg;
  VerificationConfig verify_cfg;
  const auto candidate = findLoopCandidate(db, kps_a, tris_a, cfg, vote_cfg, verify_cfg);
  ASSERT_TRUE(candidate.accepted);
  EXPECT_EQ(candidate.submap_id, 77);

  const Eigen::Matrix4f T = candidate.transform;
  const Eigen::Matrix4f delta = T_gt.inverse() * T;
  const Eigen::Vector3f dt = delta.block<3, 1>(0, 3);
  EXPECT_LT(dt.norm(), 0.2f);
  const Eigen::Matrix3f R_delta = delta.block<3, 3>(0, 0);
  const float trace = R_delta.trace();
  const float arg = std::max(-1.0f, std::min(1.0f, (trace - 1.0f) * 0.5f));
  const float angle_deg = std::acos(arg) * 180.0f / static_cast<float>(M_PI);
  EXPECT_LT(angle_deg, 1.0f);
}

TEST(TriangleLoopCandidate, RejectsWhenDatabaseEmpty)
{
  TriangleDatabase db;
  HashConfig cfg;
  const auto candidate = findLoopCandidate(
    db, std::vector<Keypoint>{}, std::vector<TriangleDescriptor>{},
    cfg, VoteConfig{}, VerificationConfig{});
  EXPECT_FALSE(candidate.accepted);
  EXPECT_EQ(candidate.submap_id, -1);
}

TEST(TriangleLoopCandidate, RejectsUnrelatedQueries)
{
  // DB: a regular grid.
  const auto kps_db = makeKeypointGrid(4, 4, 3.0f);
  const auto tris_db = buildTriangles(kps_db, TriangleBuildConfig{});

  TriangleDatabase db;
  HashConfig cfg;
  db.addSubmap(50, kps_db, tris_db, cfg);

  // Query: very different geometry (1.0f spacing -> all edges shrunk; should
  // produce no overlap in hash buckets at edge_bin_m=1.0).
  const auto kps_q = makeKeypointGrid(4, 4, 0.8f);
  const auto tris_q = buildTriangles(kps_q, TriangleBuildConfig{});

  VoteConfig vote_cfg;
  VerificationConfig verify_cfg;
  const auto candidate = findLoopCandidate(db, kps_q, tris_q, cfg, vote_cfg, verify_cfg);
  EXPECT_FALSE(candidate.accepted);
}

TEST(TriangleLoopCandidate, InlierRatioGateRejectsLowRatio)
{
  // Reuse the identity-recovery setup: every triangle pair will produce the
  // same SE(3), so inliers / eval_n approaches 1.0. A ratio threshold above
  // 1.0 cannot be met, so the candidate must be rejected.
  const auto kps = makeKeypointGrid(4, 4, 3.0f);
  const auto tris = buildTriangles(kps, TriangleBuildConfig{});

  TriangleDatabase db;
  HashConfig cfg;
  db.addSubmap(11, kps, tris, cfg);

  VoteConfig vote_cfg;
  VerificationConfig verify_cfg;
  verify_cfg.min_inlier_ratio = 1.5f;  // impossible: ratio is bounded by 1.0
  const auto candidate = findLoopCandidate(db, kps, tris, cfg, vote_cfg, verify_cfg);
  EXPECT_FALSE(candidate.accepted);
  EXPECT_GT(candidate.inliers, 0);  // RANSAC still found inliers, just not enough
}

TEST(TriangleLoopCandidate, InlierRatioGateAcceptsAtZeroDisabled)
{
  const auto kps = makeKeypointGrid(4, 4, 3.0f);
  const auto tris = buildTriangles(kps, TriangleBuildConfig{});

  TriangleDatabase db;
  HashConfig cfg;
  db.addSubmap(13, kps, tris, cfg);

  VoteConfig vote_cfg;
  VerificationConfig verify_cfg;
  verify_cfg.min_inlier_ratio = 0.0f;  // default; disabled
  const auto candidate = findLoopCandidate(db, kps, tris, cfg, vote_cfg, verify_cfg);
  EXPECT_TRUE(candidate.accepted);
}

TEST(TriangleLoopCandidate, MaxPairsLimitsEvaluatedSet)
{
  // Cap max_pairs to 3 -- best-case inliers must not exceed that cap.
  const auto kps = makeKeypointGrid(4, 4, 3.0f);
  const auto tris = buildTriangles(kps, TriangleBuildConfig{});

  TriangleDatabase db;
  HashConfig cfg;
  db.addSubmap(17, kps, tris, cfg);

  VoteConfig vote_cfg;
  VerificationConfig verify_cfg;
  verify_cfg.max_pairs = 3;
  verify_cfg.min_inliers = 1;
  const auto candidate = findLoopCandidate(db, kps, tris, cfg, vote_cfg, verify_cfg);
  EXPECT_LE(candidate.inliers, 3);
  EXPECT_GE(candidate.inliers, 0);
}
