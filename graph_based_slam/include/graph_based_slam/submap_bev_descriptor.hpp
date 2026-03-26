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

// A lightweight submap-level BEV descriptor for place recognition.
// This is intended to be more sensor-agnostic than Scan Context because it
// operates on accumulated submaps instead of a single rotating scan.

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

class SubmapBEVDescriptor
{
public:
  static constexpr int DEFAULT_GRID_CELLS = 40;
  static constexpr double DEFAULT_GRID_SIZE_M = 80.0;
  static constexpr int DEFAULT_YAW_BINS = 24;
  static constexpr int DEFAULT_NUM_CANDIDATES = 20;
  static constexpr int DEFAULT_EXCLUDE_RECENT = 50;
  static constexpr double DEFAULT_DISTANCE_THRESHOLD = 0.20;
  static constexpr float MAX_HEIGHT_ABS_M = 5.0f;
  static constexpr int COARSE_KEY_CELLS = 8;

  struct Descriptor
  {
    Eigen::MatrixXf occupancy;
    Eigen::MatrixXf density;
    Eigen::MatrixXf max_height;
    Eigen::VectorXf coarse_key;
  };

  struct Match
  {
    int submap_id {-1};
    double distance {1.0};
    int yaw_bin {0};
    double yaw_rad {0.0};
  };

  static Descriptor computeDescriptor(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud,
    double grid_size_m = DEFAULT_GRID_SIZE_M,
    int grid_cells = DEFAULT_GRID_CELLS)
  {
    Descriptor descriptor;
    descriptor.occupancy = Eigen::MatrixXf::Zero(grid_cells, grid_cells);
    descriptor.density = Eigen::MatrixXf::Zero(grid_cells, grid_cells);
    descriptor.max_height = Eigen::MatrixXf::Zero(grid_cells, grid_cells);

    if (!cloud || cloud->empty() || grid_cells <= 0 || grid_size_m <= 0.0) {
      descriptor.coarse_key = Eigen::VectorXf::Zero(COARSE_KEY_CELLS * COARSE_KEY_CELLS * 3);
      return descriptor;
    }

    const float half_extent = static_cast<float>(grid_size_m * 0.5);
    const float cell_size = static_cast<float>(grid_size_m / static_cast<double>(grid_cells));
    Eigen::MatrixXf count = Eigen::MatrixXf::Zero(grid_cells, grid_cells);
    Eigen::MatrixXf max_height = Eigen::MatrixXf::Constant(
      grid_cells, grid_cells, -std::numeric_limits<float>::infinity());

    for (const auto & point : cloud->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      if (std::abs(point.x) > half_extent || std::abs(point.y) > half_extent) {
        continue;
      }

      const int ix = static_cast<int>(std::floor((point.x + half_extent) / cell_size));
      const int iy = static_cast<int>(std::floor((point.y + half_extent) / cell_size));
      if (ix < 0 || ix >= grid_cells || iy < 0 || iy >= grid_cells) {
        continue;
      }

      descriptor.occupancy(iy, ix) = 1.0f;
      count(iy, ix) += 1.0f;
      max_height(iy, ix) = std::max(max_height(iy, ix), point.z);
    }

    const float max_count = std::max(1.0f, count.maxCoeff());
    for (int row = 0; row < grid_cells; ++row) {
      for (int col = 0; col < grid_cells; ++col) {
        descriptor.density(row, col) = std::log1p(count(row, col)) / std::log1p(max_count);
        if (descriptor.occupancy(row, col) > 0.0f && std::isfinite(max_height(row, col))) {
          const float clipped = std::max(
            -MAX_HEIGHT_ABS_M, std::min(MAX_HEIGHT_ABS_M, max_height(row, col)));
          descriptor.max_height(row, col) =
            (clipped + MAX_HEIGHT_ABS_M) / (2.0f * MAX_HEIGHT_ABS_M);
        }
      }
    }

    descriptor.coarse_key = computeCoarseKey(descriptor);
    return descriptor;
  }

  static Descriptor rotateDescriptor(const Descriptor & descriptor, double yaw_rad)
  {
    Descriptor rotated;
    rotated.occupancy = rotateGrid(descriptor.occupancy, yaw_rad);
    rotated.density = rotateGrid(descriptor.density, yaw_rad);
    rotated.max_height = rotateGrid(descriptor.max_height, yaw_rad);
    rotated.coarse_key = computeCoarseKey(rotated);
    return rotated;
  }

  static double descriptorDistance(const Descriptor & query, const Descriptor & candidate)
  {
    const Eigen::VectorXf query_vec = flattenDescriptor(query);
    const Eigen::VectorXf candidate_vec = flattenDescriptor(candidate);
    return cosineDistance(query_vec, candidate_vec);
  }

  static Match distanceWithAlignment(
    const Descriptor & query,
    const Descriptor & candidate,
    int submap_id,
    int yaw_bins = DEFAULT_YAW_BINS)
  {
    Match best_match;
    best_match.submap_id = submap_id;
    best_match.distance = std::numeric_limits<double>::max();
    if (yaw_bins < 1) {
      yaw_bins = 1;
    }
    for (int yaw_bin = 0; yaw_bin < yaw_bins; ++yaw_bin) {
      const double yaw_rad = static_cast<double>(yaw_bin) * 2.0 * M_PI /
        static_cast<double>(yaw_bins);
      const Descriptor rotated = rotateDescriptor(candidate, yaw_rad);
      const double distance = descriptorDistance(query, rotated);
      if (distance < best_match.distance) {
        best_match.distance = distance;
        best_match.yaw_bin = yaw_bin;
        best_match.yaw_rad = yaw_rad;
      }
    }
    return best_match;
  }

  struct Database
  {
    explicit Database(
      double grid_size_m_in = DEFAULT_GRID_SIZE_M,
      int grid_cells_in = DEFAULT_GRID_CELLS,
      int yaw_bins_in = DEFAULT_YAW_BINS)
    : grid_size_m(grid_size_m_in), grid_cells(grid_cells_in), yaw_bins(yaw_bins_in)
    {
    }

    double grid_size_m {DEFAULT_GRID_SIZE_M};
    int grid_cells {DEFAULT_GRID_CELLS};
    int yaw_bins {DEFAULT_YAW_BINS};
    std::vector<int> submap_ids;
    std::vector<Descriptor> descriptors;

    void clear()
    {
      submap_ids.clear();
      descriptors.clear();
    }

    void configure(double new_grid_size_m, int new_grid_cells, int new_yaw_bins)
    {
      const bool changed = std::abs(new_grid_size_m - grid_size_m) > 1e-6 ||
        new_grid_cells != grid_cells || new_yaw_bins != yaw_bins;
      grid_size_m = new_grid_size_m;
      grid_cells = new_grid_cells;
      yaw_bins = new_yaw_bins;
      if (changed) {
        clear();
      }
    }

    void add(int submap_id, const Descriptor & descriptor)
    {
      submap_ids.push_back(submap_id);
      descriptors.push_back(descriptor);
    }

    int nextSubmapIndex() const
    {
      return submap_ids.empty() ? 0 : (submap_ids.back() + 1);
    }

    int size() const
    {
      return static_cast<int>(descriptors.size());
    }

    std::vector<Match> queryTopMatchesWithYaw(
      const Descriptor & query_descriptor,
      int num_matches,
      int num_candidates = DEFAULT_NUM_CANDIDATES,
      int exclude_recent = DEFAULT_EXCLUDE_RECENT,
      double threshold = DEFAULT_DISTANCE_THRESHOLD) const
    {
      std::vector<Match> matches;
      const int total = static_cast<int>(descriptors.size());
      const int search_end = total - exclude_recent;
      if (search_end <= 0 || num_matches <= 0) {
        return matches;
      }

      std::vector<std::pair<double, int>> coarse_candidates;
      coarse_candidates.reserve(search_end);
      for (int idx = 0; idx < search_end; ++idx) {
        const double distance = cosineDistance(
          query_descriptor.coarse_key, descriptors[idx].coarse_key);
        coarse_candidates.emplace_back(distance, idx);
      }

      const int k = std::min(num_candidates, static_cast<int>(coarse_candidates.size()));
      std::partial_sort(
        coarse_candidates.begin(),
        coarse_candidates.begin() + k,
        coarse_candidates.end());

      std::vector<Match> verified;
      verified.reserve(k);
      for (int idx = 0; idx < k; ++idx) {
        const int descriptor_idx = coarse_candidates[idx].second;
        verified.push_back(
          distanceWithAlignment(
            query_descriptor,
            descriptors[descriptor_idx],
            submap_ids[descriptor_idx],
            yaw_bins));
      }

      std::sort(
        verified.begin(), verified.end(),
        [](const Match & lhs, const Match & rhs) {return lhs.distance < rhs.distance;});
      for (const auto & match : verified) {
        if (match.distance >= threshold) {
          continue;
        }
        matches.push_back(match);
        if (static_cast<int>(matches.size()) >= num_matches) {
          break;
        }
      }
      return matches;
    }

    std::pair<int, double> query(
      const Descriptor & query_descriptor,
      int num_candidates = DEFAULT_NUM_CANDIDATES,
      int exclude_recent = DEFAULT_EXCLUDE_RECENT,
      double threshold = DEFAULT_DISTANCE_THRESHOLD) const
    {
      const auto matches = queryTopMatchesWithYaw(
        query_descriptor, 1, num_candidates, exclude_recent, threshold);
      if (matches.empty()) {
        return {-1, std::numeric_limits<double>::max()};
      }
      return {matches.front().submap_id, matches.front().distance};
    }
  };

private:
  static Eigen::VectorXf flattenDescriptor(const Descriptor & descriptor)
  {
    const int cells = static_cast<int>(descriptor.occupancy.rows() * descriptor.occupancy.cols());
    Eigen::VectorXf flattened(cells * 3);
    Eigen::Map<const Eigen::VectorXf> occupancy_map(
      descriptor.occupancy.data(),
      cells);
    Eigen::Map<const Eigen::VectorXf> density_map(
      descriptor.density.data(),
      cells);
    Eigen::Map<const Eigen::VectorXf> height_map(
      descriptor.max_height.data(),
      cells);
    flattened.segment(0, cells) = occupancy_map;
    flattened.segment(cells, cells) = density_map;
    flattened.segment(2 * cells, cells) = height_map;
    const float norm = flattened.norm();
    if (norm > 1e-6f) {
      flattened /= norm;
    }
    return flattened;
  }

  static double cosineDistance(const Eigen::VectorXf & lhs, const Eigen::VectorXf & rhs)
  {
    if (lhs.size() == 0 || rhs.size() == 0 || lhs.size() != rhs.size()) {
      return 1.0;
    }
    const float lhs_norm = lhs.norm();
    const float rhs_norm = rhs.norm();
    if (lhs_norm < 1e-6f || rhs_norm < 1e-6f) {
      return 1.0;
    }
    const float cosine = std::max(-1.0f, std::min(1.0f, lhs.dot(rhs) / (lhs_norm * rhs_norm)));
    return 1.0 - static_cast<double>(cosine);
  }

  static Eigen::MatrixXf rotateGrid(const Eigen::MatrixXf & grid, double yaw_rad)
  {
    if (grid.size() == 0) {
      return grid;
    }
    const int rows = grid.rows();
    const int cols = grid.cols();
    Eigen::MatrixXf rotated = Eigen::MatrixXf::Zero(rows, cols);
    const float center_x = static_cast<float>(cols - 1) * 0.5f;
    const float center_y = static_cast<float>(rows - 1) * 0.5f;
    const float c = static_cast<float>(std::cos(yaw_rad));
    const float s = static_cast<float>(std::sin(yaw_rad));

    for (int row = 0; row < rows; ++row) {
      for (int col = 0; col < cols; ++col) {
        const float x = static_cast<float>(col) - center_x;
        const float y = static_cast<float>(row) - center_y;
        const float src_x = c * x + s * y + center_x;
        const float src_y = -s * x + c * y + center_y;
        const int src_col = static_cast<int>(std::round(src_x));
        const int src_row = static_cast<int>(std::round(src_y));
        if (src_col >= 0 && src_col < cols && src_row >= 0 && src_row < rows) {
          rotated(row, col) = grid(src_row, src_col);
        }
      }
    }
    return rotated;
  }

  static Eigen::MatrixXf averagePool(const Eigen::MatrixXf & grid, int pooled_cells)
  {
    Eigen::MatrixXf pooled = Eigen::MatrixXf::Zero(pooled_cells, pooled_cells);
    if (grid.size() == 0 || pooled_cells <= 0) {
      return pooled;
    }
    const int rows = grid.rows();
    const int cols = grid.cols();
    for (int py = 0; py < pooled_cells; ++py) {
      const int row_begin = (py * rows) / pooled_cells;
      const int row_end = ((py + 1) * rows) / pooled_cells;
      for (int px = 0; px < pooled_cells; ++px) {
        const int col_begin = (px * cols) / pooled_cells;
        const int col_end = ((px + 1) * cols) / pooled_cells;
        float sum = 0.0f;
        int count = 0;
        for (int row = row_begin; row < row_end; ++row) {
          for (int col = col_begin; col < col_end; ++col) {
            sum += grid(row, col);
            ++count;
          }
        }
        if (count > 0) {
          pooled(py, px) = sum / static_cast<float>(count);
        }
      }
    }
    return pooled;
  }

  static Eigen::VectorXf computeCoarseKey(const Descriptor & descriptor)
  {
    const Eigen::MatrixXf occupancy = averagePool(descriptor.occupancy, COARSE_KEY_CELLS);
    const Eigen::MatrixXf density = averagePool(descriptor.density, COARSE_KEY_CELLS);
    const Eigen::MatrixXf height = averagePool(descriptor.max_height, COARSE_KEY_CELLS);

    const int cells = COARSE_KEY_CELLS * COARSE_KEY_CELLS;
    Eigen::VectorXf key(cells * 3);
    Eigen::Map<const Eigen::VectorXf> occupancy_map(occupancy.data(), cells);
    Eigen::Map<const Eigen::VectorXf> density_map(density.data(), cells);
    Eigen::Map<const Eigen::VectorXf> height_map(height.data(), cells);
    key.segment(0, cells) = occupancy_map;
    key.segment(cells, cells) = density_map;
    key.segment(2 * cells, cells) = height_map;
    const float norm = key.norm();
    if (norm > 1e-6f) {
      key /= norm;
    }
    return key;
  }
};

}  // namespace graphslam
