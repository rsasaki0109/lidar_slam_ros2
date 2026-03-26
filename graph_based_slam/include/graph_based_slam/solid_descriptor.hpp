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
//
// This file contains a small BSD-compatible adaptation of descriptor logic
// from sparolab/SOLiD (BSD-3-Clause License, Copyright (c) 2024, sparolab).

#pragma once

#include <Eigen/Core>
#include <pcl/filters/voxel_grid.h>
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

class SolidDescriptor
{
public:
  static constexpr double DEFAULT_FOV_UP_DEG = 2.0;
  static constexpr double DEFAULT_FOV_DOWN_DEG = -24.8;
  static constexpr int DEFAULT_NUM_ANGLE = 60;
  static constexpr int DEFAULT_NUM_RANGE = 40;
  static constexpr int DEFAULT_NUM_HEIGHT = 32;
  static constexpr double DEFAULT_MIN_DISTANCE_M = 3.0;
  static constexpr double DEFAULT_MAX_DISTANCE_M = 80.0;
  static constexpr double DEFAULT_VOXEL_SIZE_M = 0.4;
  static constexpr int DEFAULT_NUM_CANDIDATES = 20;
  static constexpr int DEFAULT_EXCLUDE_RECENT = 50;
  static constexpr double DEFAULT_MIN_SIMILARITY = 0.70;

  struct Descriptor
  {
    Eigen::VectorXd range;
    Eigen::VectorXd angle;
    Eigen::VectorXd solid;
  };

  struct Match
  {
    int submap_id {-1};
    double similarity {-1.0};
    int yaw_bin {0};
    double yaw_rad {0.0};
  };

  static Descriptor computeDescriptor(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud,
    double min_distance_m = DEFAULT_MIN_DISTANCE_M,
    double max_distance_m = DEFAULT_MAX_DISTANCE_M,
    double voxel_size_m = DEFAULT_VOXEL_SIZE_M,
    int num_angle = DEFAULT_NUM_ANGLE,
    int num_range = DEFAULT_NUM_RANGE,
    int num_height = DEFAULT_NUM_HEIGHT,
    double fov_up_deg = DEFAULT_FOV_UP_DEG,
    double fov_down_deg = DEFAULT_FOV_DOWN_DEG)
  {
    Descriptor descriptor;
    descriptor.range = Eigen::VectorXd::Zero(num_range);
    descriptor.angle = Eigen::VectorXd::Zero(num_angle);
    descriptor.solid = Eigen::VectorXd::Zero(num_range + num_angle);
    if (!cloud || cloud->empty()) {
      return descriptor;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cropped(new pcl::PointCloud<pcl::PointXYZI>);
    cropped->reserve(cloud->size());
    for (const auto & point : cloud->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      const double distance = std::sqrt(
        static_cast<double>(point.x) * point.x +
        static_cast<double>(point.y) * point.y +
        static_cast<double>(point.z) * point.z);
      if (distance < min_distance_m || distance > max_distance_m) {
        continue;
      }
      cropped->push_back(point);
    }
    if (cropped->empty()) {
      return descriptor;
    }

    pcl::VoxelGrid<pcl::PointXYZI> voxelgrid;
    voxelgrid.setLeafSize(voxel_size_m, voxel_size_m, voxel_size_m);
    voxelgrid.setInputCloud(cropped);
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>);
    voxelgrid.filter(*downsampled);
    if (downsampled->empty()) {
      return descriptor;
    }

    Eigen::MatrixXd range_matrix = Eigen::MatrixXd::Zero(num_range, num_height);
    Eigen::MatrixXd angle_matrix = Eigen::MatrixXd::Zero(num_angle, num_height);
    Eigen::VectorXd height_weights = Eigen::VectorXd::Zero(num_height);

    const double gap_angle_deg = 360.0 / static_cast<double>(num_angle);
    const double gap_range_m = max_distance_m / static_cast<double>(num_range);
    const double gap_height_deg = (fov_up_deg - fov_down_deg) / static_cast<double>(num_height);

    for (const auto & point : downsampled->points) {
      double px = point.x;
      double py = point.y;
      if (std::abs(px) < 1e-6) {
        px = (px >= 0.0) ? 1e-6 : -1e-6;
      }
      if (std::abs(py) < 1e-6) {
        py = (py >= 0.0) ? 1e-6 : -1e-6;
      }
      const double theta_deg = positiveAngleDeg(std::atan2(py, px) * 180.0 / M_PI);
      const double dist_xy = std::sqrt(px * px + py * py);
      const double phi_deg = std::atan2(point.z, dist_xy) * 180.0 / M_PI;

      const int range_idx = std::min(
        static_cast<int>(dist_xy / gap_range_m),
        num_range - 1);
      const int angle_idx = std::min(
        static_cast<int>(theta_deg / gap_angle_deg),
        num_angle - 1);
      const int height_idx = clampInt(
        static_cast<int>((phi_deg - fov_down_deg) / gap_height_deg),
        0,
        num_height - 1);

      range_matrix(range_idx, height_idx) += 1.0;
      angle_matrix(angle_idx, height_idx) += 1.0;
    }

    for (int col = 0; col < range_matrix.cols(); ++col) {
      height_weights(col) = range_matrix.col(col).sum();
    }
    const double min_weight = height_weights.minCoeff();
    const double max_weight = height_weights.maxCoeff();
    if (max_weight - min_weight > 1e-9) {
      height_weights =
        (height_weights.array() - min_weight) / (max_weight - min_weight);
    } else if (max_weight > 0.0) {
      height_weights.setOnes();
    } else {
      height_weights.setZero();
    }

    descriptor.range = normalize(range_matrix * height_weights);
    descriptor.angle = normalize(angle_matrix * height_weights);
    descriptor.solid.resize(num_range + num_angle);
    descriptor.solid.head(num_range) = descriptor.range;
    descriptor.solid.tail(num_angle) = descriptor.angle;
    descriptor.solid = normalize(descriptor.solid);
    return descriptor;
  }

  static double loopSimilarity(const Descriptor & query, const Descriptor & candidate)
  {
    return cosineSimilarity(query.range, candidate.range);
  }

  static double poseYawRad(const Descriptor & query, const Descriptor & candidate)
  {
    if (query.angle.size() == 0 || query.angle.size() != candidate.angle.size()) {
      return 0.0;
    }
    const int num_angle = query.angle.size();
    double min_l1_norm = std::numeric_limits<double>::max();
    int min_index = 0;
    for (int shift_index = 0; shift_index < num_angle; ++shift_index) {
      Eigen::VectorXd shifted = Eigen::VectorXd::Zero(num_angle);
      for (int i = 0; i < num_angle; ++i) {
        shifted((i + shift_index) % num_angle) = query.angle(i);
      }
      const double l1_norm = (candidate.angle - shifted).cwiseAbs().sum();
      if (l1_norm < min_l1_norm) {
        min_l1_norm = l1_norm;
        min_index = shift_index;
      }
    }
    double yaw_rad = static_cast<double>(min_index) * 2.0 * M_PI / static_cast<double>(num_angle);
    while (yaw_rad > M_PI) {
      yaw_rad -= 2.0 * M_PI;
    }
    while (yaw_rad < -M_PI) {
      yaw_rad += 2.0 * M_PI;
    }
    return yaw_rad;
  }

  struct Database
  {
    std::vector<int> submap_ids;
    std::vector<Descriptor> descriptors;

    void clear()
    {
      submap_ids.clear();
      descriptors.clear();
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
      double min_similarity = DEFAULT_MIN_SIMILARITY) const
    {
      std::vector<Match> matches;
      const int total = static_cast<int>(descriptors.size());
      const int search_end = total - exclude_recent;
      if (search_end <= 0 || num_matches <= 0) {
        return matches;
      }

      std::vector<std::pair<double, int>> scored_candidates;
      scored_candidates.reserve(search_end);
      for (int idx = 0; idx < search_end; ++idx) {
        scored_candidates.emplace_back(loopSimilarity(query_descriptor, descriptors[idx]), idx);
      }

      const int k = std::min(num_candidates, static_cast<int>(scored_candidates.size()));
      std::partial_sort(
        scored_candidates.begin(),
        scored_candidates.begin() + k,
        scored_candidates.end(),
        [](const auto & lhs, const auto & rhs) {
          return lhs.first > rhs.first;
        });

      for (int idx = 0; idx < k; ++idx) {
        const int descriptor_idx = scored_candidates[idx].second;
        const double similarity = scored_candidates[idx].first;
        if (similarity < min_similarity) {
          continue;
        }
        Match match;
        match.submap_id = submap_ids[descriptor_idx];
        match.similarity = similarity;
        match.yaw_rad = poseYawRad(query_descriptor, descriptors[descriptor_idx]);
        match.yaw_bin = static_cast<int>(std::lround(
            positiveAngleDeg(match.yaw_rad * 180.0 / M_PI) *
            static_cast<double>(DEFAULT_NUM_ANGLE) / 360.0)) % DEFAULT_NUM_ANGLE;
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
      double min_similarity = DEFAULT_MIN_SIMILARITY) const
    {
      const auto matches = queryTopMatchesWithYaw(
        query_descriptor, 1, num_candidates, exclude_recent, min_similarity);
      if (matches.empty()) {
        return {-1, -1.0};
      }
      return {matches.front().submap_id, matches.front().similarity};
    }
  };

private:
  static double positiveAngleDeg(double angle_deg)
  {
    while (angle_deg < 0.0) {
      angle_deg += 360.0;
    }
    while (angle_deg >= 360.0) {
      angle_deg -= 360.0;
    }
    return angle_deg;
  }

  static Eigen::VectorXd normalize(const Eigen::VectorXd & vector)
  {
    if (vector.size() == 0) {
      return vector;
    }
    const double norm = vector.norm();
    if (norm < 1e-9) {
      return vector;
    }
    return vector / norm;
  }

  static double cosineSimilarity(const Eigen::VectorXd & lhs, const Eigen::VectorXd & rhs)
  {
    if (lhs.size() == 0 || lhs.size() != rhs.size()) {
      return -1.0;
    }
    const double lhs_norm = lhs.norm();
    const double rhs_norm = rhs.norm();
    if (lhs_norm < 1e-9 || rhs_norm < 1e-9) {
      return -1.0;
    }
    return clampDouble(lhs.dot(rhs) / (lhs_norm * rhs_norm), -1.0, 1.0);
  }

  static int clampInt(int value, int lower, int upper)
  {
    return std::max(lower, std::min(value, upper));
  }

  static double clampDouble(double value, double lower, double upper)
  {
    return std::max(lower, std::min(value, upper));
  }
};

}  // namespace graphslam
