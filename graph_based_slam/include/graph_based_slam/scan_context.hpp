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
// Scan Context: Egocentric Spatial Descriptor for Place Recognition
// Based on: Kim & Kim, "Scan Context", IROS 2018
// Implemented from scratch without referencing GPL code.
#pragma once

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

namespace graphslam
{

class ScanContext
{
public:
  // Default parameters from the paper (Section IV-A).
  static constexpr int NUM_RINGS = 20;
  static constexpr int NUM_SECTORS = 60;
  static constexpr double MAX_RANGE = 80.0;
  static constexpr int NUM_CANDIDATES = 50;  // KNN candidates for ring key search.
  static constexpr int EXCLUDE_RECENT = 50;  // Skip recent N nodes.
  static constexpr double DISTANCE_THRESHOLD = 0.3;  // Lower is stricter.

  using Descriptor = Eigen::MatrixXd;  // NUM_RINGS x NUM_SECTORS.
  using RingKey = Eigen::VectorXd;  // NUM_RINGS.

  struct DistanceAlignment
  {
    double distance {1.0};
    int shift {0};
  };

  struct Match
  {
    int submap_id {-1};
    double distance {1.0};
    int yaw_shift {0};
  };

  template<typename T>
  static T clampValue(const T & value, const T & low, const T & high)
  {
    return std::max(low, std::min(value, high));
  }

  // Compute a Scan Context descriptor from a point cloud.
  static Descriptor computeDescriptor(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud,
    double max_range = MAX_RANGE)
  {
    Descriptor desc = Descriptor::Zero(NUM_RINGS, NUM_SECTORS);

    double ring_gap = max_range / NUM_RINGS;
    double sector_gap = 2.0 * M_PI / NUM_SECTORS;

    for (const auto & p : cloud->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }

      double r = std::sqrt(p.x * p.x + p.y * p.y);
      if (r < 0.01 || r >= max_range) {
        continue;
      }

      // Azimuth angle [0, 2*pi).
      double theta = std::atan2(p.y, p.x);
      if (theta < 0) {
        theta += 2.0 * M_PI;
      }

      int ring_idx = static_cast<int>(r / ring_gap);
      int sector_idx = static_cast<int>(theta / sector_gap);

      ring_idx = clampValue(ring_idx, 0, NUM_RINGS - 1);
      sector_idx = clampValue(sector_idx, 0, NUM_SECTORS - 1);

      // Max height encoding (Eq. 2, 3).
      desc(ring_idx, sector_idx) = std::max(desc(ring_idx, sector_idx), static_cast<double>(p.z));
    }
    return desc;
  }

  // Compute a rotation-invariant occupancy ratio per ring (Eq. 8, 9).
  static RingKey computeRingKey(const Descriptor & desc)
  {
    RingKey key(NUM_RINGS);
    for (int i = 0; i < NUM_RINGS; i++) {
      int nonzero = 0;
      for (int j = 0; j < NUM_SECTORS; j++) {
        if (std::abs(desc(i, j)) > 1e-6) {
          nonzero++;
        }
      }
      key(i) = static_cast<double>(nonzero) / NUM_SECTORS;
    }
    return key;
  }

  // Column-wise cosine distance (Eq. 5).
  static double columnCosineDistance(const Descriptor & a, const Descriptor & b)
  {
    double total = 0.0;
    int valid_cols = 0;

    for (int j = 0; j < NUM_SECTORS; j++) {
      Eigen::VectorXd col_a = a.col(j);
      Eigen::VectorXd col_b = b.col(j);

      double norm_a = col_a.norm();
      double norm_b = col_b.norm();

      if (norm_a < 1e-6 || norm_b < 1e-6) {
        continue;
      }

      double cosine = col_a.dot(col_b) / (norm_a * norm_b);
      cosine = clampValue(cosine, -1.0, 1.0);
      total += 1.0 - cosine;
      valid_cols++;
    }

    return valid_cols > 0 ? total / valid_cols : 1.0;
  }

  // Distance with column shifting for rotation invariance (Eq. 6).
  static DistanceAlignment distanceWithAlignment(
    const Descriptor & query,
    const Descriptor & candidate)
  {
    double min_dist = std::numeric_limits<double>::max();
    int best_shift = 0;

    for (int shift = 0; shift < NUM_SECTORS; shift++) {
      // Circularly shift columns of candidate.
      Descriptor shifted(NUM_RINGS, NUM_SECTORS);
      for (int j = 0; j < NUM_SECTORS; j++) {
        shifted.col(j) = candidate.col((j + shift) % NUM_SECTORS);
      }
      double dist = columnCosineDistance(query, shifted);
      if (dist < min_dist) {
        min_dist = dist;
        best_shift = shift;
      }
    }
    return {min_dist, best_shift};
  }

  static double distance(const Descriptor & query, const Descriptor & candidate)
  {
    return distanceWithAlignment(query, candidate).distance;
  }

  // Ring key L2 distance used for KNN search.
  static double ringKeyDistance(const RingKey & a, const RingKey & b)
  {
    return (a - b).norm();
  }

  // Database for loop detection.
  struct Database
  {
    std::vector<int> submap_ids;
    std::vector<Descriptor> descriptors;
    std::vector<RingKey> ring_keys;

    void add(int submap_id, const Descriptor & desc)
    {
      submap_ids.push_back(submap_id);
      descriptors.push_back(desc);
      ring_keys.push_back(computeRingKey(desc));
    }

    int nextSubmapIndex() const
    {
      return submap_ids.empty() ? 0 : (submap_ids.back() + 1);
    }

    std::vector<std::pair<int, double>> queryTopMatches(
      const Descriptor & query_desc,
      int num_matches,
      int num_candidates = NUM_CANDIDATES,
      int exclude_recent = EXCLUDE_RECENT,
      double threshold = DISTANCE_THRESHOLD) const
    {
      std::vector<std::pair<int, double>> matches;

      int n = static_cast<int>(ring_keys.size());
      int search_end = n - exclude_recent;
      if (search_end <= 0 || num_matches <= 0) {
        return matches;
      }

      RingKey query_key = computeRingKey(query_desc);

      std::vector<std::pair<double, int>> candidates;
      candidates.reserve(search_end);
      for (int i = 0; i < search_end; i++) {
        double d = ringKeyDistance(query_key, ring_keys[i]);
        candidates.emplace_back(d, i);
      }

      int k = std::min(num_candidates, static_cast<int>(candidates.size()));
      std::partial_sort(candidates.begin(), candidates.begin() + k, candidates.end());

      std::vector<std::pair<double, int>> verified;
      verified.reserve(k);
      for (int c = 0; c < k; c++) {
        int idx = candidates[c].second;
        double dist = distance(query_desc, descriptors[idx]);
        verified.emplace_back(dist, submap_ids[idx]);
      }

      std::sort(verified.begin(), verified.end());
      for (const auto & candidate : verified) {
        if (candidate.first >= threshold) {
          continue;
        }
        matches.emplace_back(candidate.second, candidate.first);
        if (static_cast<int>(matches.size()) >= num_matches) {
          break;
        }
      }

      return matches;
    }

    std::vector<Match> queryTopMatchesWithYaw(
      const Descriptor & query_desc,
      int num_matches,
      int num_candidates = NUM_CANDIDATES,
      int exclude_recent = EXCLUDE_RECENT,
      double threshold = DISTANCE_THRESHOLD) const
    {
      std::vector<Match> matches;

      int n = static_cast<int>(ring_keys.size());
      int search_end = n - exclude_recent;
      if (search_end <= 0 || num_matches <= 0) {
        return matches;
      }

      RingKey query_key = computeRingKey(query_desc);

      std::vector<std::pair<double, int>> candidates;
      candidates.reserve(search_end);
      for (int i = 0; i < search_end; i++) {
        double d = ringKeyDistance(query_key, ring_keys[i]);
        candidates.emplace_back(d, i);
      }

      int k = std::min(num_candidates, static_cast<int>(candidates.size()));
      std::partial_sort(candidates.begin(), candidates.begin() + k, candidates.end());

      std::vector<Match> verified;
      verified.reserve(k);
      for (int c = 0; c < k; c++) {
        int idx = candidates[c].second;
        const auto alignment = distanceWithAlignment(query_desc, descriptors[idx]);
        verified.push_back({submap_ids[idx], alignment.distance, alignment.shift});
      }

      std::sort(
        verified.begin(), verified.end(),
        [](const Match & lhs, const Match & rhs) {return lhs.distance < rhs.distance;});
      for (const auto & candidate : verified) {
        if (candidate.distance >= threshold) {
          continue;
        }
        matches.push_back(candidate);
        if (static_cast<int>(matches.size()) >= num_matches) {
          break;
        }
      }

      return matches;
    }

    // Return (best_index, best_distance) or (-1, inf) if no match.
    std::pair<int, double> query(
      const Descriptor & query_desc,
      int num_candidates = NUM_CANDIDATES,
      int exclude_recent = EXCLUDE_RECENT,
      double threshold = DISTANCE_THRESHOLD) const
    {
      const auto matches = queryTopMatches(
        query_desc, 1, num_candidates, exclude_recent, threshold);
      if (matches.empty()) {
        return {-1, std::numeric_limits<double>::max()};
      }
      return matches.front();
    }

    int size() const
    {
      return static_cast<int>(descriptors.size());
    }
  };
};

}  // namespace graphslam
