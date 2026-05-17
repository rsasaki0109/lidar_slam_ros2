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

// Triangle descriptor hashing, in-memory database, and vote-based loop
// candidate lookup. Implemented from scratch under BSD-2 so the default
// workflow can include it without GPL contamination.
//
// Pipeline contract:
//   1. quantizeEdges: sorted triangle edges (m) -> integer (le, me, ge) key
//      that places nearby triangles into the same bucket.
//   2. TriangleDatabase: stores per-submap (TriangleDescriptor, vertex
//      positions) indexed by the hash key. Vertices are kept so geometric
//      verification can run without going back to the original keypoints.
//   3. accumulateVotes: hash-lookup each query triangle and count votes
//      per candidate submap (one vote per matching triangle, capped).
//   4. findLoopCandidate: pick the top-voted submap, then verify by
//      enumerating matching triangle pairs and looking for a consensus
//      SE(3) via RANSAC over estimateRigidFromTriangle output.

#ifndef GRAPH_BASED_SLAM__TRIANGLE_DESCRIPTOR_DATABASE_HPP_
#define GRAPH_BASED_SLAM__TRIANGLE_DESCRIPTOR_DATABASE_HPP_

#include "graph_based_slam/triangle_descriptor.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace graphslam
{
namespace triangle
{

struct HashConfig
{
  // Edge bin width (m). Edges within +-edge_bin_m / 2 fall in the same bin.
  float edge_bin_m {1.0f};
  // Hard upper bound on the quantized bin index (clipped at this value).
  // Default keeps the packed key small (uint64) for triangles up to ~80 m.
  int max_bin {255};
};

// (le, me, ge) integer bin tuple, le <= me <= ge.
struct TriangleHash
{
  uint16_t le {0};
  uint16_t me {0};
  uint16_t ge {0};

  bool operator==(const TriangleHash & other) const
  {
    return le == other.le && me == other.me && ge == other.ge;
  }
};

inline TriangleHash quantizeEdges(const TriangleDescriptor & t, const HashConfig & cfg)
{
  TriangleHash h;
  const float bin = std::max(1e-3f, cfg.edge_bin_m);
  auto q = [&](float v) -> uint16_t {
      const int idx = static_cast<int>(std::floor(v / bin));
      const int clipped = std::max(0, std::min(cfg.max_bin, idx));
      return static_cast<uint16_t>(clipped);
    };
  h.le = q(t.edges[0]);
  h.me = q(t.edges[1]);
  h.ge = q(t.edges[2]);
  return h;
}

inline uint64_t packHash(const TriangleHash & h)
{
  return (static_cast<uint64_t>(h.le) << 32) |
         (static_cast<uint64_t>(h.me) << 16) |
         static_cast<uint64_t>(h.ge);
}

struct DatabaseEntry
{
  int submap_id {-1};
  // Vertex positions in submap-local frame, in the order that matches
  // TriangleDescriptor::keypoint_ids (vertex opposite edges[k] is index k).
  std::array<Eigen::Vector3f, 3> vertices {{
    Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero()
  }};
};

class TriangleDatabase
{
public:
  // Stash all triangles from a submap. ``keypoints`` is the keypoint vector
  // that was passed to buildTriangles; the function looks up each
  // ``keypoint_ids`` entry to copy the position into the database.
  void addSubmap(
    int submap_id,
    const std::vector<Keypoint> & keypoints,
    const std::vector<TriangleDescriptor> & triangles,
    const HashConfig & cfg)
  {
    for (const auto & t : triangles) {
      DatabaseEntry e;
      e.submap_id = submap_id;
      bool ok = true;
      for (int k = 0; k < 3; ++k) {
        const int idx = t.keypoint_ids[k];
        if (idx < 0 || idx >= static_cast<int>(keypoints.size())) {
          ok = false;
          break;
        }
        e.vertices[k] = keypoints[idx].position;
      }
      if (!ok) {continue;}
      const uint64_t key = packHash(quantizeEdges(t, cfg));
      buckets_[key].push_back(e);
      ++triangle_count_;
    }
    submap_ids_.insert(submap_id);
  }

  // Returns the bucket for a hash key (empty vector if none).
  const std::vector<DatabaseEntry> & lookup(const TriangleHash & h) const
  {
    static const std::vector<DatabaseEntry> empty;
    auto it = buckets_.find(packHash(h));
    if (it == buckets_.end()) {return empty;}
    return it->second;
  }

  std::size_t triangleCount() const {return triangle_count_;}
  std::size_t submapCount() const {return submap_ids_.size();}
  bool empty() const {return triangle_count_ == 0;}

private:
  std::unordered_map<uint64_t, std::vector<DatabaseEntry>> buckets_;
  std::size_t triangle_count_ {0};
  std::unordered_set<int> submap_ids_;
};

// One submap's vote count after hash-lookup.
struct SubmapVote
{
  int submap_id {-1};
  int votes {0};
};

struct VoteConfig
{
  // Cap on votes contributed by a single query triangle (avoids dominance
  // when many database triangles share an unusual bucket).
  int max_votes_per_query {3};
  // Submap id to exclude from voting (typically the query submap itself).
  int exclude_submap_id {-1};
};

inline std::vector<SubmapVote> accumulateVotes(
  const TriangleDatabase & db,
  const std::vector<TriangleDescriptor> & query_triangles,
  const HashConfig & cfg,
  const VoteConfig & vote_cfg)
{
  std::unordered_map<int, int> counts;
  for (const auto & t : query_triangles) {
    const auto & bucket = db.lookup(quantizeEdges(t, cfg));
    if (bucket.empty()) {continue;}
    // Per-query cap: dedupe by submap id and cap the contribution count.
    std::unordered_map<int, int> per_query;
    for (const auto & e : bucket) {
      if (e.submap_id == vote_cfg.exclude_submap_id) {continue;}
      ++per_query[e.submap_id];
    }
    for (const auto & kv : per_query) {
      const int contribution = std::min(kv.second, std::max(1, vote_cfg.max_votes_per_query));
      counts[kv.first] += contribution;
    }
  }

  std::vector<SubmapVote> result;
  result.reserve(counts.size());
  for (const auto & kv : counts) {
    result.push_back({kv.first, kv.second});
  }
  std::sort(
    result.begin(), result.end(),
    [](const SubmapVote & a, const SubmapVote & b) {return a.votes > b.votes;});
  return result;
}

struct VerificationConfig
{
  // Triangle pair is treated as an inlier if every other matching pair agrees
  // with the proposed SE(3) up to this translation tolerance (m).
  float inlier_translation_m {2.0f};
  // ...and this rotation tolerance (deg).
  float inlier_rotation_deg {5.0f};
  // Minimum inliers required to accept the candidate.
  int min_inliers {3};
  // Cap on triangle pairs evaluated (top N by edge length descending).
  int max_pairs {64};
};

struct LoopCandidate
{
  int submap_id {-1};
  int votes {0};
  int inliers {0};
  Eigen::Matrix4f transform {Eigen::Matrix4f::Identity()};
  bool accepted {false};
};

namespace detail
{

inline float rotationAngleDeg(const Eigen::Matrix3f & R)
{
  const float trace = R.trace();
  const float arg = std::max(-1.0f, std::min(1.0f, (trace - 1.0f) * 0.5f));
  return std::acos(arg) * 180.0f / static_cast<float>(M_PI);
}

inline bool transformAgrees(
  const Eigen::Matrix4f & a, const Eigen::Matrix4f & b,
  float trans_tol_m, float rot_tol_deg)
{
  const Eigen::Vector3f dt = a.block<3, 1>(0, 3) - b.block<3, 1>(0, 3);
  if (dt.norm() > trans_tol_m) {return false;}
  const Eigen::Matrix3f Ra = a.block<3, 3>(0, 0);
  const Eigen::Matrix3f Rb = b.block<3, 3>(0, 0);
  const Eigen::Matrix3f Rdelta = Ra.transpose() * Rb;
  return rotationAngleDeg(Rdelta) <= rot_tol_deg;
}

// Pull (src, dst) vertex arrays for a single query/db triangle pair.
inline void packTrianglePair(
  const TriangleDescriptor & query_tri,
  const std::vector<Keypoint> & query_keypoints,
  const DatabaseEntry & db_entry,
  std::array<Eigen::Vector3f, 3> & src,
  std::array<Eigen::Vector3f, 3> & dst)
{
  for (int k = 0; k < 3; ++k) {
    const int qid = query_tri.keypoint_ids[k];
    src[k] = (qid >= 0 && qid < static_cast<int>(query_keypoints.size())) ?
      query_keypoints[qid].position : Eigen::Vector3f::Zero();
    dst[k] = db_entry.vertices[k];
  }
}

}  // namespace detail

// Find the best loop candidate for the query (keypoints + triangles) against
// the database. Returns LoopCandidate with accepted=false if no inlier set
// meets the verification threshold.
inline LoopCandidate findLoopCandidate(
  const TriangleDatabase & db,
  const std::vector<Keypoint> & query_keypoints,
  const std::vector<TriangleDescriptor> & query_triangles,
  const HashConfig & cfg,
  const VoteConfig & vote_cfg,
  const VerificationConfig & verify_cfg)
{
  LoopCandidate result;
  const auto votes = accumulateVotes(db, query_triangles, cfg, vote_cfg);
  if (votes.empty()) {return result;}

  result.submap_id = votes.front().submap_id;
  result.votes = votes.front().votes;

  // Collect (query_tri, db_entry) pairs that touch the winning submap, then
  // run RANSAC: each pair proposes a transform, count how many other pairs
  // agree.
  struct Pair
  {
    const TriangleDescriptor * query_tri;
    const DatabaseEntry * db_entry;
    float largest_edge;
  };
  std::vector<Pair> pairs;
  pairs.reserve(query_triangles.size());
  for (const auto & qt : query_triangles) {
    const auto & bucket = db.lookup(quantizeEdges(qt, cfg));
    for (const auto & e : bucket) {
      if (e.submap_id != result.submap_id) {continue;}
      pairs.push_back({&qt, &e, qt.edges[2]});
    }
  }
  if (pairs.empty()) {return result;}

  std::sort(
    pairs.begin(), pairs.end(),
    [](const Pair & a, const Pair & b) {return a.largest_edge > b.largest_edge;});
  const int eval_n = std::min<int>(verify_cfg.max_pairs, static_cast<int>(pairs.size()));

  int best_inliers = 0;
  Eigen::Matrix4f best_T = Eigen::Matrix4f::Identity();
  for (int i = 0; i < eval_n; ++i) {
    std::array<Eigen::Vector3f, 3> src;
    std::array<Eigen::Vector3f, 3> dst;
    detail::packTrianglePair(*pairs[i].query_tri, query_keypoints, *pairs[i].db_entry, src, dst);
    const Eigen::Matrix4f T_i = estimateRigidFromTriangle(src, dst);
    int inliers = 0;
    for (int j = 0; j < eval_n; ++j) {
      std::array<Eigen::Vector3f, 3> sj;
      std::array<Eigen::Vector3f, 3> dj;
      detail::packTrianglePair(*pairs[j].query_tri, query_keypoints, *pairs[j].db_entry, sj, dj);
      const Eigen::Matrix4f T_j = estimateRigidFromTriangle(sj, dj);
      if (detail::transformAgrees(
          T_i, T_j, verify_cfg.inlier_translation_m, verify_cfg.inlier_rotation_deg))
      {
        ++inliers;
      }
    }
    if (inliers > best_inliers) {
      best_inliers = inliers;
      best_T = T_i;
    }
  }

  result.inliers = best_inliers;
  result.transform = best_T;
  result.accepted = best_inliers >= verify_cfg.min_inliers;
  return result;
}

}  // namespace triangle
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__TRIANGLE_DESCRIPTOR_DATABASE_HPP_
