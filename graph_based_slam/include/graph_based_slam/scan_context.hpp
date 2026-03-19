// Scan Context: Egocentric Spatial Descriptor for Place Recognition
// Based on: Kim & Kim, "Scan Context", IROS 2018
// Implemented from scratch (no GPL code referenced)
#pragma once

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <limits>

namespace graphslam {

class ScanContext {
public:
  // Default parameters from the paper (Section IV-A)
  static constexpr int NUM_RINGS = 20;
  static constexpr int NUM_SECTORS = 60;
  static constexpr double MAX_RANGE = 80.0;
  static constexpr int NUM_CANDIDATES = 50;       // KNN candidates for ring key search
  static constexpr int EXCLUDE_RECENT = 50;        // Skip recent N nodes
  static constexpr double DISTANCE_THRESHOLD = 0.3; // Acceptance threshold (lower = stricter)

  using Descriptor = Eigen::MatrixXd;  // NUM_RINGS x NUM_SECTORS
  using RingKey = Eigen::VectorXd;     // NUM_RINGS

  // Compute Scan Context descriptor from point cloud
  static Descriptor computeDescriptor(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud,
    double max_range = MAX_RANGE)
  {
    Descriptor desc = Descriptor::Zero(NUM_RINGS, NUM_SECTORS);

    double ring_gap = max_range / NUM_RINGS;
    double sector_gap = 2.0 * M_PI / NUM_SECTORS;

    for (const auto& p : cloud->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;

      double r = std::sqrt(p.x * p.x + p.y * p.y);
      if (r < 0.01 || r >= max_range) continue;

      // Azimuth angle [0, 2*pi)
      double theta = std::atan2(p.y, p.x);
      if (theta < 0) theta += 2.0 * M_PI;

      int ring_idx = static_cast<int>(r / ring_gap);
      int sector_idx = static_cast<int>(theta / sector_gap);

      ring_idx = std::clamp(ring_idx, 0, NUM_RINGS - 1);
      sector_idx = std::clamp(sector_idx, 0, NUM_SECTORS - 1);

      // Max height encoding (Eq. 2, 3)
      desc(ring_idx, sector_idx) = std::max(desc(ring_idx, sector_idx), static_cast<double>(p.z));
    }
    return desc;
  }

  // Compute ring key from descriptor (Eq. 8, 9)
  // Rotation-invariant sub-descriptor: occupancy ratio per ring
  static RingKey computeRingKey(const Descriptor& desc)
  {
    RingKey key(NUM_RINGS);
    for (int i = 0; i < NUM_RINGS; i++) {
      int nonzero = 0;
      for (int j = 0; j < NUM_SECTORS; j++) {
        if (std::abs(desc(i, j)) > 1e-6) nonzero++;
      }
      key(i) = static_cast<double>(nonzero) / NUM_SECTORS;
    }
    return key;
  }

  // Column-wise cosine distance (Eq. 5)
  static double columnCosineDistance(const Descriptor& a, const Descriptor& b)
  {
    double total = 0.0;
    int valid_cols = 0;

    for (int j = 0; j < NUM_SECTORS; j++) {
      Eigen::VectorXd col_a = a.col(j);
      Eigen::VectorXd col_b = b.col(j);

      double norm_a = col_a.norm();
      double norm_b = col_b.norm();

      if (norm_a < 1e-6 || norm_b < 1e-6) continue;

      double cosine = col_a.dot(col_b) / (norm_a * norm_b);
      cosine = std::clamp(cosine, -1.0, 1.0);
      total += 1.0 - cosine;
      valid_cols++;
    }

    return valid_cols > 0 ? total / valid_cols : 1.0;
  }

  // Distance with column shifting for rotation invariance (Eq. 6)
  static double distance(const Descriptor& query, const Descriptor& candidate)
  {
    double min_dist = std::numeric_limits<double>::max();

    for (int shift = 0; shift < NUM_SECTORS; shift++) {
      // Circularly shift columns of candidate
      Descriptor shifted(NUM_RINGS, NUM_SECTORS);
      for (int j = 0; j < NUM_SECTORS; j++) {
        shifted.col(j) = candidate.col((j + shift) % NUM_SECTORS);
      }
      double dist = columnCosineDistance(query, shifted);
      min_dist = std::min(min_dist, dist);
    }
    return min_dist;
  }

  // Ring key L2 distance (for KNN search)
  static double ringKeyDistance(const RingKey& a, const RingKey& b)
  {
    return (a - b).norm();
  }

  // Database for loop detection
  struct Database {
    std::vector<Descriptor> descriptors;
    std::vector<RingKey> ring_keys;

    void add(const Descriptor& desc) {
      descriptors.push_back(desc);
      ring_keys.push_back(computeRingKey(desc));
    }

    // Find best loop closure candidate
    // Returns: (best_index, best_distance) or (-1, inf) if no match
    std::pair<int, double> query(
      const Descriptor& query_desc,
      int num_candidates = NUM_CANDIDATES,
      int exclude_recent = EXCLUDE_RECENT,
      double threshold = DISTANCE_THRESHOLD) const
    {
      int n = static_cast<int>(ring_keys.size());
      int search_end = n - exclude_recent;
      if (search_end <= 0) return {-1, std::numeric_limits<double>::max()};

      RingKey query_key = computeRingKey(query_desc);

      // Phase 1: Find top-K candidates by ring key L2 distance
      std::vector<std::pair<double, int>> candidates;
      candidates.reserve(search_end);
      for (int i = 0; i < search_end; i++) {
        double d = ringKeyDistance(query_key, ring_keys[i]);
        candidates.emplace_back(d, i);
      }

      // Partial sort to get top K
      int k = std::min(num_candidates, static_cast<int>(candidates.size()));
      std::partial_sort(candidates.begin(), candidates.begin() + k, candidates.end());

      // Phase 2: Verify with full Scan Context distance
      int best_idx = -1;
      double best_dist = std::numeric_limits<double>::max();

      for (int c = 0; c < k; c++) {
        int idx = candidates[c].second;
        double dist = distance(query_desc, descriptors[idx]);
        if (dist < best_dist) {
          best_dist = dist;
          best_idx = idx;
        }
      }

      if (best_dist < threshold) {
        return {best_idx, best_dist};
      }
      return {-1, best_dist};
    }

    int size() const { return static_cast<int>(descriptors.size()); }
  };
};

}  // namespace graphslam
