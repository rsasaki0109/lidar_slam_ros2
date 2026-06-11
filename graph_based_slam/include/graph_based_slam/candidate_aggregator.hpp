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
#include <utility>
#include <vector>

#include <Eigen/Core>  // NOLINT(build/include_order)

#include "graph_based_slam/loop_verifier.hpp"
#include "graph_based_slam/scan_context.hpp"

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

}  // namespace candidate_aggregator
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__CANDIDATE_AGGREGATOR_HPP_
