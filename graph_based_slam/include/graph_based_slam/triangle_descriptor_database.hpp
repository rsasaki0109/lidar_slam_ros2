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
  // Quad-key extension (STD/BTC-style 4-point context). When > 0, the
  // hash key gains a 4th rotation-invariant dim: the quantized distance
  // from the triangle centroid to the nearest non-vertex keypoint in the
  // same submap. Two triangles must then agree on all 4 bins to collide,
  // which suppresses the wrong-but-agreeing matches that the 3-edge hash
  // alone admits in repeated geometry (corridor, parking lot rows). The
  // 4th-point lookup needs the keypoint list available at hash time; the
  // legacy 3-edge entry point is preserved as a backward-compat overload
  // and is what every test and yaml hits today. 0 = disabled (3-edge).
  float quad_feature_bin_m {0.0f};
};

// (le, me, ge) integer bin tuple, le <= me <= ge. ``quad`` is the optional
// 4th-point dim (zero when quad hashing is disabled — matches the legacy
// 3-edge packed key bit-for-bit).
struct TriangleHash
{
  uint16_t le {0};
  uint16_t me {0};
  uint16_t ge {0};
  uint16_t quad {0};

  bool operator==(const TriangleHash & other) const
  {
    return le == other.le && me == other.me && ge == other.ge && quad == other.quad;
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

// Quad-aware quantize. When cfg.quad_feature_bin_m <= 0 or keypoints is
// empty, behaves identically to quantizeEdges so the legacy hash key
// stays bit-for-bit identical.
inline TriangleHash quantizeKey(
  const TriangleDescriptor & t,
  const std::vector<Keypoint> & keypoints,
  const HashConfig & cfg)
{
  TriangleHash h = quantizeEdges(t, cfg);
  if (cfg.quad_feature_bin_m <= 0.0f || keypoints.empty()) {
    return h;
  }
  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
  for (int k = 0; k < 3; ++k) {
    const int idx = t.keypoint_ids[k];
    if (idx < 0 || idx >= static_cast<int>(keypoints.size())) {
      return h;
    }
    centroid += keypoints[idx].position;
  }
  centroid /= 3.0f;
  float best = std::numeric_limits<float>::infinity();
  for (int i = 0; i < static_cast<int>(keypoints.size()); ++i) {
    if (i == t.keypoint_ids[0] || i == t.keypoint_ids[1] || i == t.keypoint_ids[2]) {
      continue;
    }
    const float d = (keypoints[i].position - centroid).norm();
    if (d < best) {best = d;}
  }
  if (std::isfinite(best)) {
    const float qbin = std::max(1e-3f, cfg.quad_feature_bin_m);
    const int qidx = static_cast<int>(std::floor(best / qbin));
    h.quad = static_cast<uint16_t>(std::max(0, std::min(cfg.max_bin, qidx)));
  }
  return h;
}

inline uint64_t packHash(const TriangleHash & h)
{
  return (static_cast<uint64_t>(h.quad) << 48) |
         (static_cast<uint64_t>(h.le) << 32) |
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
      // Use the quad-aware key: when cfg.quad_feature_bin_m <= 0 this is
      // bit-for-bit identical to the legacy 3-edge key, so every call site
      // that hasn't enabled quad hashing is unchanged.
      const uint64_t key = packHash(quantizeKey(t, keypoints, cfg));
      buckets_[key].push_back(e);
      ++triangle_count_;
    }
    submap_ids_.insert(submap_id);
    // Hold on to the full keypoint list so the 4-point consensus check has
    // off-triangle keypoints to project under the candidate SE(3).
    submap_keypoints_[submap_id] = keypoints;
  }

  // Returns the bucket for a hash key (empty vector if none).
  const std::vector<DatabaseEntry> & lookup(const TriangleHash & h) const
  {
    static const std::vector<DatabaseEntry> empty;
    auto it = buckets_.find(packHash(h));
    if (it == buckets_.end()) {return empty;}
    return it->second;
  }

  // Returns the full keypoint vector for a submap (empty when unknown).
  const std::vector<Keypoint> & keypoints(int submap_id) const
  {
    static const std::vector<Keypoint> empty;
    auto it = submap_keypoints_.find(submap_id);
    if (it == submap_keypoints_.end()) {return empty;}
    return it->second;
  }

  std::size_t triangleCount() const {return triangle_count_;}
  std::size_t submapCount() const {return submap_ids_.size();}
  bool empty() const {return triangle_count_ == 0;}

private:
  std::unordered_map<uint64_t, std::vector<DatabaseEntry>> buckets_;
  std::size_t triangle_count_ {0};
  std::unordered_set<int> submap_ids_;
  std::unordered_map<int, std::vector<Keypoint>> submap_keypoints_;
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

// Quad-aware overload: pass the query keypoint list so the hash lookup
// agrees with the database when cfg.quad_feature_bin_m > 0. The legacy
// 4-arg overload below forwards an empty vector, which keeps the hash
// equivalent to the original 3-edge key (matching the database that
// addSubmap built under the same cfg).
inline std::vector<SubmapVote> accumulateVotes(
  const TriangleDatabase & db,
  const std::vector<Keypoint> & query_keypoints,
  const std::vector<TriangleDescriptor> & query_triangles,
  const HashConfig & cfg,
  const VoteConfig & vote_cfg)
{
  std::unordered_map<int, int> counts;
  for (const auto & t : query_triangles) {
    const auto & bucket = db.lookup(quantizeKey(t, query_keypoints, cfg));
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

// Backward-compatible overload: no keypoints -> quad bin always 0 ->
// hash matches the legacy 3-edge key. Existing tests and call sites
// that haven't been updated to pass query_keypoints land here.
inline std::vector<SubmapVote> accumulateVotes(
  const TriangleDatabase & db,
  const std::vector<TriangleDescriptor> & query_triangles,
  const HashConfig & cfg,
  const VoteConfig & vote_cfg)
{
  static const std::vector<Keypoint> empty;
  return accumulateVotes(db, empty, query_triangles, cfg, vote_cfg);
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
  // Minimum inlier ratio (inliers / eval_n) required when > 0. Useful to
  // attach a relative-density floor to a low absolute count: with max_pairs
  // 64 a count of 4 is only 6%, but at max_pairs 20 the same 4 is 20%. Set
  // to 0.0 to disable.
  float min_inlier_ratio {0.0f};
  // Cap on triangle pairs evaluated (top N by edge length descending).
  int max_pairs {64};
  // 4-point consensus: after picking the best SE(3) from 3-point RANSAC,
  // transform each non-triangle query keypoint by it and require at least
  // this many to fall within `fourth_point_max_distance_m` of any database
  // keypoint in the chosen submap. Three points uniquely determine SE(3),
  // so the 3-point inlier count alone can be fooled by repeated geometry;
  // a 4-point check brings in an independent constraint. Set to 0 to
  // disable (default keeps the previous behaviour).
  int min_4th_point_agreements {0};
  float fourth_point_max_distance_m {2.0f};
  // After the 3-point RANSAC picks the winning SE(3), optionally pool the
  // 3 * N_inliers point correspondences and re-estimate SE(3) by a single
  // N-point Umeyama (least squares). The translation of one 3-point estimate
  // is noisy because each vertex is a noisy keypoint; pooling reduces that
  // noise by √N and consistently produces a tighter SE(3) that NDT can refine
  // without walking to a wrong basin. Default off so the 3-dataset baselines
  // stay reproducible; flip on for indoor / narrow-FOV scenes where the
  // initial-pose accuracy is the gating factor.
  bool refine_se3_with_all_inliers {false};
};

struct LoopCandidate
{
  int submap_id {-1};
  int votes {0};
  int inliers {0};
  // Triangle pairs actually evaluated by RANSAC (= min(verify_cfg.max_pairs,
  // total bucket hits)). Useful for the operator to compute / verify the
  // inlier ratio that gated this emit; populated even when the candidate
  // is rejected so post-mortem tuning can see the ratio that came up
  // short.
  int eval_n {0};
  // ``inliers / eval_n`` when eval_n > 0, else 0. Precomputed so operators
  // can log the ratio without re-computing it from int fields.
  float inlier_ratio {0.0f};
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
  const auto votes = accumulateVotes(db, query_keypoints, query_triangles, cfg, vote_cfg);
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
    const auto & bucket = db.lookup(quantizeKey(qt, query_keypoints, cfg));
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

  // Optional refinement: pool every inlier triangle's 3 point correspondences
  // and re-estimate SE(3) by a single N-point Umeyama. Reduces translation
  // noise compared to keeping the single winning 3-point hypothesis.
  if (verify_cfg.refine_se3_with_all_inliers && best_inliers >= 2) {
    std::vector<Eigen::Vector3f> all_src;
    std::vector<Eigen::Vector3f> all_dst;
    all_src.reserve(static_cast<std::size_t>(best_inliers) * 3);
    all_dst.reserve(static_cast<std::size_t>(best_inliers) * 3);
    for (int j = 0; j < eval_n; ++j) {
      std::array<Eigen::Vector3f, 3> sj;
      std::array<Eigen::Vector3f, 3> dj;
      detail::packTrianglePair(
        *pairs[j].query_tri, query_keypoints, *pairs[j].db_entry, sj, dj);
      const Eigen::Matrix4f T_j = estimateRigidFromTriangle(sj, dj);
      if (detail::transformAgrees(
          best_T, T_j, verify_cfg.inlier_translation_m, verify_cfg.inlier_rotation_deg))
      {
        for (int k = 0; k < 3; ++k) {
          all_src.push_back(sj[k]);
          all_dst.push_back(dj[k]);
        }
      }
    }
    if (all_src.size() >= 3) {
      best_T = estimateRigidFromCorrespondences(all_src, all_dst);
    }
  }

  result.inliers = best_inliers;
  result.eval_n = eval_n;
  result.inlier_ratio = (eval_n > 0) ?
    static_cast<float>(best_inliers) / static_cast<float>(eval_n) : 0.0f;
  result.transform = best_T;
  bool count_ok = best_inliers >= verify_cfg.min_inliers;
  bool ratio_ok = true;
  if (verify_cfg.min_inlier_ratio > 0.0f && eval_n > 0) {
    ratio_ok = result.inlier_ratio >= verify_cfg.min_inlier_ratio;
  }
  bool fourth_ok = true;
  if (count_ok && ratio_ok && verify_cfg.min_4th_point_agreements > 0) {
    // Cross-check non-triangle keypoints: project each query keypoint by the
    // winning SE(3) and look for a database keypoint within tolerance.
    const auto & db_kps = db.keypoints(result.submap_id);
    int agreements = 0;
    const float thresh = verify_cfg.fourth_point_max_distance_m;
    for (const auto & q_kp : query_keypoints) {
      const Eigen::Vector4f qh(q_kp.position.x(), q_kp.position.y(), q_kp.position.z(), 1.0f);
      const Eigen::Vector4f q_trans = best_T * qh;
      const Eigen::Vector3f q3 = q_trans.head<3>();
      float min_dist = std::numeric_limits<float>::infinity();
      for (const auto & d_kp : db_kps) {
        const float dist = (d_kp.position - q3).norm();
        if (dist < min_dist) {min_dist = dist;}
      }
      if (min_dist <= thresh) {++agreements;}
    }
    fourth_ok = agreements >= verify_cfg.min_4th_point_agreements;
  }
  result.accepted = count_ok && ratio_ok && fourth_ok;
  return result;
}

}  // namespace triangle
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__TRIANGLE_DESCRIPTOR_DATABASE_HPP_
