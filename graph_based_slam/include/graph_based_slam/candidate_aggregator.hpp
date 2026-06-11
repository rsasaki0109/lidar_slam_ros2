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

#ifndef GRAPH_BASED_SLAM__CANDIDATE_AGGREGATOR_HPP_
#define GRAPH_BASED_SLAM__CANDIDATE_AGGREGATOR_HPP_

// Pure candidate-generation logic for searchLoopForLatest: one collector
// per loop-candidate source feeding a shared upsert, with operator log
// lines returned as data so the ROS shell can emit them byte-identically.
// Inputs are plain pose/travel-distance arrays plus the (already pure,
// individually tested) descriptor databases; nothing here may touch ROS,
// the clock or global state so the offline BackendCore can replay it
// deterministically (docs/roadmap/v0.6.md, Phase 2).

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>  // NOLINT(build/include_order)
#include <Eigen/Geometry>  // NOLINT(build/include_order)

#include "graph_based_slam/bev_mutual_visibility.hpp"
#include "graph_based_slam/loop_verifier.hpp"
#include "graph_based_slam/scan_context.hpp"
#include "graph_based_slam/solid_descriptor.hpp"
#include "graph_based_slam/submap_bev_descriptor.hpp"

namespace graphslam
{
namespace candidate_aggregator
{

struct LogLine
{
  // Emit through the rclcpp logger (historically RCLCPP_INFO) instead of
  // std::cout. Debug-gated lines are simply not generated when
  // Config::debug is false, exactly like the historical call sites.
  bool via_logger {false};
  std::string text;
};

struct Config
{
  bool debug {false};
  int max_loop_candidate_count {0};
  // Minimum travelled distance between the latest submap and a candidate;
  // anything closer along the trajectory is still "the same place".
  double distance_loop_closure {0.0};
  // Maximum euclidean distance for the distance source.
  double range_of_searching_loop_closure {0.0};
  double scan_context_threshold {0.0};

  bool bev_use_mutual_visibility {false};
  double bev_mutual_visibility_min_overlap_ratio {0.0};
  double bev_mutual_visibility_occupancy_eps {0.0};
  int bev_descriptor_yaw_bins {0};
  double bev_descriptor_max_euclidean_distance_m {0.0};
  double bev_descriptor_threshold {0.0};
  int bev_descriptor_sequence_window {0};
  double bev_descriptor_sequence_threshold {0.0};
  double bev_descriptor_pose_consistency_threshold_m {0.0};
  double bev_descriptor_rerank_weight_m {0.0};

  double solid_descriptor_max_euclidean_distance_m {0.0};
  double solid_descriptor_min_similarity {0.0};
  int solid_descriptor_sequence_window {0};
  double solid_descriptor_sequence_min_similarity {0.0};
  double solid_descriptor_pose_consistency_threshold_m {0.0};
};

// The shared candidate upsert (historically the add_candidate lambda):
// one entry per (index, source); repeats keep the smallest selection
// metric but always adopt the latest yaw hint / relative transform.
inline void upsertCandidate(
  std::vector<loop_verifier::LoopCandidate> & candidates,
  int index,
  double selection_metric,
  loop_verifier::LoopCandidate::Source source,
  double yaw_rad = 0.0,
  const Eigen::Matrix4f * relative_transform = nullptr)
{
  if (index < 0) {
    return;
  }
  for (auto & candidate : candidates) {
    if (candidate.index != index || candidate.source != source) {
      continue;
    }
    candidate.selection_metric = std::min(candidate.selection_metric, selection_metric);
    candidate.yaw_rad = yaw_rad;
    if (relative_transform != nullptr) {
      candidate.relative_transform = *relative_transform;
      candidate.has_relative_transform = true;
    }
    return;
  }

  loop_verifier::LoopCandidate candidate;
  candidate.index = index;
  candidate.selection_metric = selection_metric;
  candidate.source = source;
  candidate.yaw_rad = yaw_rad;
  if (relative_transform != nullptr) {
    candidate.relative_transform = *relative_transform;
    candidate.has_relative_transform = true;
  }
  candidates.push_back(candidate);
}

// Every submap that already left the trajectory exclusion window and lies
// within search range, as (euclidean distance, submap index) pairs sorted
// ascending — the pair ordering tie-breaks equal distances on the lower
// index, which keeps downstream consumers deterministic.
inline std::vector<std::pair<double, int>> collectDistanceCandidates(
  const std::vector<Eigen::Vector3d> & submap_positions,
  const std::vector<double> & submap_travel_distances,
  int latest_idx,
  const Config & config)
{
  std::vector<std::pair<double, int>> distance_candidates;
  distance_candidates.reserve(submap_positions.size());
  const Eigen::Vector3d latest_submap_pos = submap_positions[latest_idx];
  const double latest_moving_distance = submap_travel_distances[latest_idx];
  for (int i = 0; i < latest_idx; i++) {
    const double dist = (latest_submap_pos - submap_positions[i]).norm();
    if (latest_moving_distance - submap_travel_distances[i] <= config.distance_loop_closure) {
      continue;
    }
    if (dist >= config.range_of_searching_loop_closure) {
      continue;
    }
    distance_candidates.emplace_back(dist, i);
  }
  std::sort(distance_candidates.begin(), distance_candidates.end());
  return distance_candidates;
}

// The non-reranked tail of the distance source: the nearest
// max_loop_candidate_count candidates become DISTANCE candidates
// (historically the else branch taken when the BEV reranker is off).
inline void appendTopDistanceCandidates(
  const std::vector<std::pair<double, int>> & distance_candidates,
  const Config & config,
  std::vector<loop_verifier::LoopCandidate> & candidates)
{
  const int num_distance_candidates =
    std::min(config.max_loop_candidate_count, static_cast<int>(distance_candidates.size()));
  for (int i = 0; i < num_distance_candidates; i++) {
    upsertCandidate(
      candidates,
      distance_candidates[i].second,
      distance_candidates[i].first,
      loop_verifier::LoopCandidate::Source::DISTANCE);
  }
}

// ScanContext source: the first top-K match (matches arrive sorted by
// descriptor distance, ties on the lower submap id) that survives the
// travel-distance gate becomes the single SC candidate, with the sector
// shift converted to a [-pi, pi] yaw hint. The query is the database's
// most recent descriptor, which the caller keeps aligned with latest_idx.
inline void collectScanContextCandidate(
  const ScanContext::Database & scan_context_db,
  const std::vector<double> & submap_travel_distances,
  int latest_idx,
  const Config & config,
  std::vector<loop_verifier::LoopCandidate> & candidates,
  std::vector<LogLine> & logs)
{
  if (scan_context_db.size() <= ScanContext::EXCLUDE_RECENT) {
    return;
  }
  const double latest_moving_distance = submap_travel_distances[latest_idx];
  const auto sc_matches = scan_context_db.queryTopMatchesWithYaw(
    scan_context_db.descriptors.back(),
    config.max_loop_candidate_count,
    ScanContext::NUM_CANDIDATES,
    ScanContext::EXCLUDE_RECENT,
    config.scan_context_threshold);

  if (!sc_matches.empty()) {
    bool added_scan_context_candidate = false;
    for (const auto & sc_match : sc_matches) {
      const int sc_idx = sc_match.submap_id;
      const double sc_dist = sc_match.distance;
      if (sc_idx < 0 || sc_idx >= latest_idx) {
        continue;
      }
      const double sc_travel_distance =
        latest_moving_distance - submap_travel_distances[sc_idx];
      if (sc_travel_distance <= config.distance_loop_closure) {
        if (config.debug) {
          char buffer[256];
          std::snprintf(
            buffer, sizeof(buffer),
            "Skip ScanContext candidate %d because travel distance %.3f m is below %.3f m",
            sc_idx,
            sc_travel_distance,
            config.distance_loop_closure);
          logs.push_back(LogLine{true, std::string(buffer)});
        }
        continue;
      }
      double sc_yaw_rad =
        -static_cast<double>(sc_match.yaw_shift) * 2.0 * M_PI / ScanContext::NUM_SECTORS;
      while (sc_yaw_rad > M_PI) {
        sc_yaw_rad -= 2.0 * M_PI;
      }
      while (sc_yaw_rad < -M_PI) {
        sc_yaw_rad += 2.0 * M_PI;
      }
      upsertCandidate(
        candidates,
        sc_idx,
        sc_dist,
        loop_verifier::LoopCandidate::Source::SCAN_CONTEXT,
        sc_yaw_rad);
      std::ostringstream oss;
      oss << "ScanContext loop candidate: id=" << sc_idx
          << " sc_dist=" << sc_dist
          << " yaw_deg=" << sc_yaw_rad * 180.0 / M_PI;
      logs.push_back(LogLine{false, oss.str()});
      added_scan_context_candidate = true;
      break;
    }
    if (!added_scan_context_candidate && config.debug) {
      logs.push_back(
        LogLine{false, "ScanContext matches exist but none satisfied travel-distance gating"});
    }
  } else if (config.debug) {
    const auto best = scan_context_db.query(
      scan_context_db.descriptors.back(),
      ScanContext::NUM_CANDIDATES,
      ScanContext::EXCLUDE_RECENT,
      std::numeric_limits<double>::max());
    std::ostringstream oss;
    oss << "ScanContext no match: best_sc_dist=" << best.second
        << " threshold=" << config.scan_context_threshold;
    logs.push_back(LogLine{false, oss.str()});
  }
}

// BEV reranker for the distance source: descriptor hints gathered over the
// nearest 4x candidates (euclidean / descriptor / sequence /
// pose-consistency gated, best score per submap) shift the distance
// ordering by rerank_weight * (hint - threshold), then the top
// max_loop_candidate_count entries become DISTANCE candidates carrying the
// hint's yaw. The stable sort tie-breaks on the original distance, keeping
// the result deterministic. Call before the SOLiD collector: it reorders
// distance_candidates in place exactly like the historical block did.
inline void rerankDistanceCandidatesWithBev(
  const SubmapBEVDescriptor::Database & bev_db,
  const std::vector<Eigen::Affine3d> & submap_poses,
  int latest_idx,
  const Config & config,
  std::vector<std::pair<double, int>> & distance_candidates,
  std::vector<loop_verifier::LoopCandidate> & candidates,
  std::vector<LogLine> & logs)
{
  struct DescriptorRerankHint
  {
    double score = std::numeric_limits<double>::max();
    double yaw_rad = 0.0;
  };

  const Eigen::Affine3d & latest_affine = submap_poses[latest_idx];
  const Eigen::Vector3d latest_submap_pos = latest_affine.translation();

  std::unordered_map<int, DescriptorRerankHint> bev_rerank_hints;
  const int bev_rerank_candidates = std::min(
    std::max(config.max_loop_candidate_count * 4, config.max_loop_candidate_count),
    static_cast<int>(distance_candidates.size()));
  bool added_bev_candidate = false;
  double best_bev_dist = std::numeric_limits<double>::max();
  int best_bev_idx = -1;
  for (int i = 0; i < bev_rerank_candidates; ++i) {
    const int bev_idx = distance_candidates[i].second;
    if (bev_idx < 0 || bev_idx >= latest_idx || bev_idx >= bev_db.size()) {
      continue;
    }
    const double bev_euclidean_distance =
      (latest_submap_pos - submap_poses[bev_idx].translation()).norm();
    if (
      config.bev_descriptor_max_euclidean_distance_m > 0.0 &&
      bev_euclidean_distance > config.bev_descriptor_max_euclidean_distance_m)
    {
      if (config.debug) {
        char buffer[256];
        std::snprintf(
          buffer, sizeof(buffer),
          "Skip BEV candidate %d because euclidean distance %.3f m exceeds %.3f m",
          bev_idx,
          bev_euclidean_distance,
          config.bev_descriptor_max_euclidean_distance_m);
        logs.push_back(LogLine{true, std::string(buffer)});
      }
      continue;
    }

    SubmapBEVDescriptor::Match bev_match;
    if (config.bev_use_mutual_visibility) {
      graphslam::bev::MutualVisibilityConfig mv_cfg;
      mv_cfg.min_overlap_ratio = config.bev_mutual_visibility_min_overlap_ratio;
      mv_cfg.occupancy_eps =
        static_cast<float>(config.bev_mutual_visibility_occupancy_eps);
      const auto fov = graphslam::bev::mutualVisibilityWithYawSearch(
        bev_db.descriptors.back(),
        bev_db.descriptors[bev_idx],
        bev_idx,
        config.bev_descriptor_yaw_bins,
        mv_cfg);
      bev_match.submap_id = fov.submap_id;
      bev_match.distance = fov.valid ? fov.distance : 1.0;
      bev_match.yaw_bin = fov.yaw_bin;
      bev_match.yaw_rad = fov.yaw_rad;
    } else {
      bev_match = SubmapBEVDescriptor::distanceWithAlignment(
        bev_db.descriptors.back(),
        bev_db.descriptors[bev_idx],
        bev_idx,
        config.bev_descriptor_yaw_bins);
    }
    if (bev_match.distance < best_bev_dist) {
      best_bev_dist = bev_match.distance;
      best_bev_idx = bev_idx;
    }
    if (bev_match.distance >= config.bev_descriptor_threshold) {
      if (config.debug) {
        char buffer[256];
        std::snprintf(
          buffer, sizeof(buffer),
          "Skip BEV candidate %d because descriptor distance %.3f exceeds %.3f",
          bev_idx,
          bev_match.distance,
          config.bev_descriptor_threshold);
        logs.push_back(LogLine{true, std::string(buffer)});
      }
      continue;
    }

    double bev_yaw_rad = bev_match.yaw_rad;
    while (bev_yaw_rad > M_PI) {
      bev_yaw_rad -= 2.0 * M_PI;
    }
    while (bev_yaw_rad < -M_PI) {
      bev_yaw_rad += 2.0 * M_PI;
    }
    double bev_sequence_metric = bev_match.distance;
    if (config.bev_descriptor_sequence_window > 0) {
      double bev_sequence_distance_sum = bev_match.distance;
      int bev_sequence_count = 1;
      for (int offset = 1; offset <= config.bev_descriptor_sequence_window; ++offset) {
        const int query_idx = latest_idx - offset;
        const int candidate_sequence_idx = bev_idx - offset;
        if (
          query_idx < 0 || candidate_sequence_idx < 0 ||
          query_idx >= bev_db.size() ||
          candidate_sequence_idx >= bev_db.size())
        {
          break;
        }
        const auto rotated_candidate_descriptor = SubmapBEVDescriptor::rotateDescriptor(
          bev_db.descriptors[candidate_sequence_idx],
          bev_yaw_rad);
        double sequence_distance;
        if (config.bev_use_mutual_visibility) {
          graphslam::bev::MutualVisibilityConfig mv_cfg;
          mv_cfg.min_overlap_ratio = config.bev_mutual_visibility_min_overlap_ratio;
          mv_cfg.occupancy_eps =
            static_cast<float>(config.bev_mutual_visibility_occupancy_eps);
          const auto fov = graphslam::bev::mutualVisibilityDistance(
            bev_db.descriptors[query_idx],
            rotated_candidate_descriptor,
            mv_cfg);
          sequence_distance = fov.valid ? fov.distance : 1.0;
        } else {
          sequence_distance = SubmapBEVDescriptor::descriptorDistance(
            bev_db.descriptors[query_idx],
            rotated_candidate_descriptor);
        }
        bev_sequence_distance_sum += sequence_distance;
        ++bev_sequence_count;
      }
      bev_sequence_metric = bev_sequence_distance_sum / static_cast<double>(bev_sequence_count);
    }
    if (bev_sequence_metric >= config.bev_descriptor_sequence_threshold) {
      if (config.debug) {
        char buffer[256];
        std::snprintf(
          buffer, sizeof(buffer),
          "Skip BEV candidate %d because sequence metric %.3f exceeds %.3f",
          bev_idx,
          bev_sequence_metric,
          config.bev_descriptor_sequence_threshold);
        logs.push_back(LogLine{true, std::string(buffer)});
      }
      continue;
    }
    double bev_pose_consistency_metric = -1.0;
    if (
      config.bev_descriptor_pose_consistency_threshold_m > 0.0 &&
      config.bev_descriptor_sequence_window > 0)
    {
      const Eigen::Affine3d & bev_candidate_affine = submap_poses[bev_idx];
      const Eigen::AngleAxisd yaw_correction(bev_yaw_rad, Eigen::Vector3d::UnitZ());
      double bev_pose_consistency_sum = 0.0;
      int bev_pose_consistency_count = 0;
      for (int offset = 1; offset <= config.bev_descriptor_sequence_window; ++offset) {
        const int query_idx = latest_idx - offset;
        const int candidate_sequence_idx = bev_idx - offset;
        if (query_idx < 0 || candidate_sequence_idx < 0) {
          break;
        }

        const Eigen::Affine3d & query_prev_affine = submap_poses[query_idx];
        const Eigen::Affine3d & candidate_prev_affine = submap_poses[candidate_sequence_idx];

        const Eigen::Vector3d query_delta =
          (latest_affine.inverse() * query_prev_affine).translation();
        const Eigen::Vector3d candidate_delta =
          yaw_correction * (bev_candidate_affine.inverse() * candidate_prev_affine).translation();
        bev_pose_consistency_sum +=
          (query_delta.head<2>() - candidate_delta.head<2>()).norm();
        ++bev_pose_consistency_count;
      }
      if (bev_pose_consistency_count > 0) {
        bev_pose_consistency_metric =
          bev_pose_consistency_sum / static_cast<double>(bev_pose_consistency_count);
        if (bev_pose_consistency_metric >= config.bev_descriptor_pose_consistency_threshold_m) {
          if (config.debug) {
            char buffer[256];
            std::snprintf(
              buffer, sizeof(buffer),
              "Skip BEV candidate %d because pose consistency %.3f m exceeds %.3f m",
              bev_idx,
              bev_pose_consistency_metric,
              config.bev_descriptor_pose_consistency_threshold_m);
            logs.push_back(LogLine{true, std::string(buffer)});
          }
          continue;
        }
      }
    }
    auto & bev_hint = bev_rerank_hints[bev_idx];
    if (bev_sequence_metric < bev_hint.score) {
      bev_hint.score = bev_sequence_metric;
      bev_hint.yaw_rad = bev_yaw_rad;
    }
    std::ostringstream oss;
    oss << "BEV rerank hint: id=" << bev_idx
        << " bev_dist=" << bev_match.distance
        << " seq_dist=" << bev_sequence_metric
        << " pose_seq_m=" << bev_pose_consistency_metric
        << " yaw_deg=" << bev_yaw_rad * 180.0 / M_PI;
    logs.push_back(LogLine{false, oss.str()});
    added_bev_candidate = true;
  }
  if (!added_bev_candidate && config.debug) {
    std::ostringstream oss;
    oss << "BEV rerank no candidate: best_idx=" << best_bev_idx
        << " best_bev_dist=" << best_bev_dist
        << " threshold=" << config.bev_descriptor_threshold;
    logs.push_back(LogLine{false, oss.str()});
  }

  auto bev_adjusted_distance =
    [&config, &bev_rerank_hints](const std::pair<double, int> & candidate) {
      const auto bev_hint = bev_rerank_hints.find(candidate.second);
      if (bev_hint == bev_rerank_hints.end()) {
        return candidate.first;
      }
      return candidate.first +
             config.bev_descriptor_rerank_weight_m *
             (bev_hint->second.score - config.bev_descriptor_threshold);
    };

  std::stable_sort(
    distance_candidates.begin(),
    distance_candidates.end(),
    [&bev_adjusted_distance](const auto & lhs, const auto & rhs) {
      const double lhs_adjusted = bev_adjusted_distance(lhs);
      const double rhs_adjusted = bev_adjusted_distance(rhs);
      if (lhs_adjusted != rhs_adjusted) {
        return lhs_adjusted < rhs_adjusted;
      }
      return lhs.first < rhs.first;
    });

  const int num_distance_candidates =
    std::min(config.max_loop_candidate_count, static_cast<int>(distance_candidates.size()));
  for (int i = 0; i < num_distance_candidates; ++i) {
    const int candidate_idx = distance_candidates[i].second;
    const auto bev_hint = bev_rerank_hints.find(candidate_idx);
    const double adjusted_distance = bev_adjusted_distance(distance_candidates[i]);
    if (bev_hint != bev_rerank_hints.end()) {
      upsertCandidate(
        candidates,
        candidate_idx,
        adjusted_distance,
        loop_verifier::LoopCandidate::Source::DISTANCE,
        bev_hint->second.yaw_rad);
      std::ostringstream oss;
      oss << "Distance candidate reranked by BEV: id=" << candidate_idx
          << " dist_m=" << distance_candidates[i].first
          << " bev_score=" << bev_hint->second.score
          << " adjusted_dist_m=" << adjusted_distance
          << " yaw_deg=" << bev_hint->second.yaw_rad * 180.0 / M_PI;
      logs.push_back(LogLine{false, oss.str()});
    } else {
      upsertCandidate(
        candidates,
        candidate_idx,
        adjusted_distance,
        loop_verifier::LoopCandidate::Source::DISTANCE);
    }
  }
}

// SOLiD source: rescore the nearest 4x distance candidates by descriptor
// similarity (euclidean / similarity / sequence / pose-consistency gated)
// and add each survivor as a SOLID_DESCRIPTOR candidate whose selection
// metric is 1 - sequence similarity. Reads distance_candidates after any
// BEV reordering, exactly like the historical block did.
inline void collectSolidCandidates(
  const SolidDescriptor::Database & solid_db,
  const std::vector<Eigen::Affine3d> & submap_poses,
  const std::vector<std::pair<double, int>> & distance_candidates,
  int latest_idx,
  const Config & config,
  std::vector<loop_verifier::LoopCandidate> & candidates,
  std::vector<LogLine> & logs)
{
  if (solid_db.size() <= SolidDescriptor::DEFAULT_EXCLUDE_RECENT) {
    return;
  }
  const Eigen::Affine3d & latest_affine = submap_poses[latest_idx];
  const Eigen::Vector3d latest_submap_pos = latest_affine.translation();

  const int solid_rerank_candidates = std::min(
    std::max(config.max_loop_candidate_count * 4, config.max_loop_candidate_count),
    static_cast<int>(distance_candidates.size()));
  bool added_solid_candidate = false;
  double best_solid_similarity = -1.0;
  int best_solid_idx = -1;
  for (int i = 0; i < solid_rerank_candidates; ++i) {
    const int solid_idx = distance_candidates[i].second;
    if (solid_idx < 0 || solid_idx >= latest_idx || solid_idx >= solid_db.size()) {
      continue;
    }
    const double solid_euclidean_distance =
      (latest_submap_pos - submap_poses[solid_idx].translation()).norm();
    if (
      config.solid_descriptor_max_euclidean_distance_m > 0.0 &&
      solid_euclidean_distance > config.solid_descriptor_max_euclidean_distance_m)
    {
      if (config.debug) {
        char buffer[256];
        std::snprintf(
          buffer, sizeof(buffer),
          "Skip SOLiD candidate %d because euclidean distance %.3f m exceeds %.3f m",
          solid_idx,
          solid_euclidean_distance,
          config.solid_descriptor_max_euclidean_distance_m);
        logs.push_back(LogLine{true, std::string(buffer)});
      }
      continue;
    }

    const double solid_similarity = SolidDescriptor::loopSimilarity(
      solid_db.descriptors.back(),
      solid_db.descriptors[solid_idx]);
    if (solid_similarity > best_solid_similarity) {
      best_solid_similarity = solid_similarity;
      best_solid_idx = solid_idx;
    }
    if (solid_similarity < config.solid_descriptor_min_similarity) {
      if (config.debug) {
        char buffer[256];
        std::snprintf(
          buffer, sizeof(buffer),
          "Skip SOLiD candidate %d because similarity %.3f is below %.3f",
          solid_idx,
          solid_similarity,
          config.solid_descriptor_min_similarity);
        logs.push_back(LogLine{true, std::string(buffer)});
      }
      continue;
    }

    double solid_yaw_rad = SolidDescriptor::poseYawRad(
      solid_db.descriptors.back(),
      solid_db.descriptors[solid_idx]);
    while (solid_yaw_rad > M_PI) {
      solid_yaw_rad -= 2.0 * M_PI;
    }
    while (solid_yaw_rad < -M_PI) {
      solid_yaw_rad += 2.0 * M_PI;
    }

    double solid_sequence_similarity = solid_similarity;
    if (config.solid_descriptor_sequence_window > 0) {
      double solid_sequence_similarity_sum = solid_similarity;
      int solid_sequence_count = 1;
      for (int offset = 1; offset <= config.solid_descriptor_sequence_window; ++offset) {
        const int query_idx = latest_idx - offset;
        const int candidate_sequence_idx = solid_idx - offset;
        if (
          query_idx < 0 || candidate_sequence_idx < 0 ||
          query_idx >= solid_db.size() ||
          candidate_sequence_idx >= solid_db.size())
        {
          break;
        }
        solid_sequence_similarity_sum += SolidDescriptor::loopSimilarity(
          solid_db.descriptors[query_idx],
          solid_db.descriptors[candidate_sequence_idx]);
        ++solid_sequence_count;
      }
      solid_sequence_similarity =
        solid_sequence_similarity_sum / static_cast<double>(solid_sequence_count);
    }
    if (solid_sequence_similarity < config.solid_descriptor_sequence_min_similarity) {
      if (config.debug) {
        char buffer[256];
        std::snprintf(
          buffer, sizeof(buffer),
          "Skip SOLiD candidate %d because sequence similarity %.3f is below %.3f",
          solid_idx,
          solid_sequence_similarity,
          config.solid_descriptor_sequence_min_similarity);
        logs.push_back(LogLine{true, std::string(buffer)});
      }
      continue;
    }

    double solid_pose_consistency_metric = -1.0;
    if (
      config.solid_descriptor_pose_consistency_threshold_m > 0.0 &&
      config.solid_descriptor_sequence_window > 0)
    {
      const Eigen::Affine3d & solid_candidate_affine = submap_poses[solid_idx];
      const Eigen::AngleAxisd yaw_correction(solid_yaw_rad, Eigen::Vector3d::UnitZ());
      double solid_pose_consistency_sum = 0.0;
      int solid_pose_consistency_count = 0;
      for (int offset = 1; offset <= config.solid_descriptor_sequence_window; ++offset) {
        const int query_idx = latest_idx - offset;
        const int candidate_sequence_idx = solid_idx - offset;
        if (query_idx < 0 || candidate_sequence_idx < 0) {
          break;
        }

        const Eigen::Affine3d & query_prev_affine = submap_poses[query_idx];
        const Eigen::Affine3d & candidate_prev_affine = submap_poses[candidate_sequence_idx];

        const Eigen::Vector3d query_delta =
          (latest_affine.inverse() * query_prev_affine).translation();
        const Eigen::Vector3d candidate_delta =
          yaw_correction *
          (solid_candidate_affine.inverse() * candidate_prev_affine).translation();
        solid_pose_consistency_sum +=
          (query_delta.head<2>() - candidate_delta.head<2>()).norm();
        ++solid_pose_consistency_count;
      }
      if (solid_pose_consistency_count > 0) {
        solid_pose_consistency_metric =
          solid_pose_consistency_sum / static_cast<double>(solid_pose_consistency_count);
        if (
          solid_pose_consistency_metric >=
          config.solid_descriptor_pose_consistency_threshold_m)
        {
          if (config.debug) {
            char buffer[256];
            std::snprintf(
              buffer, sizeof(buffer),
              "Skip SOLiD candidate %d because pose consistency %.3f m exceeds %.3f m",
              solid_idx,
              solid_pose_consistency_metric,
              config.solid_descriptor_pose_consistency_threshold_m);
            logs.push_back(LogLine{true, std::string(buffer)});
          }
          continue;
        }
      }
    }

    upsertCandidate(
      candidates,
      solid_idx,
      1.0 - solid_sequence_similarity,
      loop_verifier::LoopCandidate::Source::SOLID_DESCRIPTOR,
      solid_yaw_rad);
    std::ostringstream oss;
    oss << "SOLiD rerank candidate: id=" << solid_idx
        << " solid_sim=" << solid_similarity
        << " seq_sim=" << solid_sequence_similarity
        << " pose_seq_m=" << solid_pose_consistency_metric
        << " yaw_deg=" << solid_yaw_rad * 180.0 / M_PI;
    logs.push_back(LogLine{false, oss.str()});
    added_solid_candidate = true;
  }
  if (!added_solid_candidate && config.debug) {
    std::ostringstream oss;
    oss << "SOLiD rerank no candidate: best_idx=" << best_solid_idx
        << " best_similarity=" << best_solid_similarity
        << " threshold=" << config.solid_descriptor_min_similarity;
    logs.push_back(LogLine{false, oss.str()});
  }
}

}  // namespace candidate_aggregator
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__CANDIDATE_AGGREGATOR_HPP_
