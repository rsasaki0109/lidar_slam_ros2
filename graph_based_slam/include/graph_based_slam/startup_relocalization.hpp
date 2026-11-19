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
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the
//    distribution.
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

#ifndef GRAPH_BASED_SLAM__STARTUP_RELOCALIZATION_HPP_
#define GRAPH_BASED_SLAM__STARTUP_RELOCALIZATION_HPP_

#include <algorithm>
#include <cmath>
#include <optional>
#include <tuple>
#include <utility>
#include <vector>

#include <small_gicp/points/eigen.hpp>
#include <small_gicp/util/downsampling.hpp>

#include "graph_based_slam/multisession_join.hpp"

namespace graphslam::multisession
{

// Prior-assisted startup registration. The reference keyframes must share one map frame.
// Both the prior and the return value place the query keyframe in that frame.
class StartupRelocalizer
{
public:
  struct Match
  {
    std::size_t reference;
    Eigen::Isometry3d pose;
  };

  explicit StartupRelocalizer(std::vector<Keyframe> reference)
  : reference_(std::move(reference)), prepared_(reference_.size()) {}

  const Eigen::Isometry3d & referencePose(std::size_t index) const
  {
    return reference_.at(index).frame;
  }

  std::optional<Match> match(
    const Keyframe & query, const Eigen::Isometry3d & prior)
  {
    if (query.points.empty()) {
      return std::nullopt;
    }
    const auto source = prepare(query);
    if (source.cloud->empty()) {
      return std::nullopt;
    }
    return matchPrepared(source, prior);
  }

  std::optional<Match> matchNearLaunch(const Keyframe & query)
  {
    if (query.points.empty() || reference_.empty()) {
      return std::nullopt;
    }
    constexpr double radius_squared = 25.0;
    const auto origin = reference_.front().frame.translation();
    if (!launch_descriptors_) {
      launch_descriptors_.emplace();
      for (std::size_t i = 0; i < reference_.size(); ++i) {
        const auto & frame = reference_[i];
        if (!frame.points.empty() &&
          (frame.frame.translation() - origin).squaredNorm() <= radius_squared)
        {
          launch_descriptors_->emplace_back(i, detail::scanContext(frame, config_));
        }
      }
    }
    const auto descriptor = detail::scanContext(query, config_);
    std::vector<std::tuple<double, std::size_t, int>> seeds;
    for (const auto & [i, target] : *launch_descriptors_) {
      const auto ranked = ScanContext::distanceWithAlignment(descriptor, target);
      seeds.emplace_back(ranked.distance, i, ranked.shift);
    }
    const auto end = seeds.begin() + std::min<std::size_t>(8, seeds.size());
    std::partial_sort(seeds.begin(), end, seeds.end());
    const auto source = prepare(query);
    if (source.cloud->empty()) {
      return std::nullopt;
    }
    std::optional<Match> consensus;
    for (auto it = seeds.begin(); it != end; ++it) {
      const auto [distance, i, shift] = *it;
      (void)distance;
      Eigen::Isometry3d initial = reference_[i].frame;
      initial.rotate(Eigen::AngleAxisd(shift * 2.0 * M_PI / ScanContext::NUM_SECTORS,
        Eigen::Vector3d::UnitZ()));
      const auto found = matchPrepared(source, initial);
      if (!found || (found->pose.translation() - origin).squaredNorm() > radius_squared) {
        continue;
      }
      if (consensus && !agrees(consensus->pose, found->pose, 0.5, 5.0)) {
        return std::nullopt;
      }
      if (!consensus) {
        consensus = found;
      }
    }
    return consensus;
  }

private:
  std::optional<Match> matchPrepared(
    const detail::Prepared & source, const Eigen::Isometry3d & prior)
  {
    std::vector<Candidate> candidates;
    for (const auto i : neighbors(prior.translation())) {
      auto candidate = align(i, source, prior);
      if (candidate.overlap >= config_.min_inlier_fraction) {
        candidates.push_back(std::move(candidate));
      }
    }
    std::sort(candidates.begin(), candidates.end(), [](const auto & a, const auto & b) {
      return a.overlap == b.overlap ? a.reference < b.reference : a.overlap > b.overlap;
    });
    const auto eligible = [&prior](const Candidate & candidate) {
        return agrees(prior, candidate.pose, 2.0, 20.0);
      };
    const auto best = std::find_if(candidates.begin(), candidates.end(), eligible);
    if (best == candidates.end()) {
      return std::nullopt;
    }
    std::size_t support = 0;
    for (const auto & candidate : candidates) {
      if (agrees(best->pose, candidate.pose, 0.5, 5.0)) {
        ++support;
      } else if (candidate.overlap >= best->overlap - 0.05 && eligible(candidate)) {
        return std::nullopt;
      }
    }
    if (support >= 2) {
      return Match{best->reference, best->pose};
    }
    // Recenter the confirmation search on the estimated pose. The initial-pose bound applies
    // to that estimate. Further registrations must corroborate the same pose independently.
    for (const auto i : neighbors(best->pose.translation())) {
      if (i == best->reference) {
        continue;
      }
      const auto candidate = align(i, source, best->pose);
      if (candidate.overlap >= config_.min_inlier_fraction &&
        agrees(best->pose, candidate.pose, 0.5, 5.0))
      {
        return Match{best->reference, best->pose};
      }
    }
    return std::nullopt;
  }

  struct Candidate
  {
    std::size_t reference;
    Eigen::Isometry3d pose;
    double overlap;
  };

  detail::Prepared prepare(const Keyframe & keyframe) const
  {
    const auto thinned = small_gicp::voxelgrid_sampling<
      std::vector<Eigen::Vector3f>, small_gicp::PointCloud>(keyframe.points, 0.2);
    const auto prepared = small_gicp::preprocess_points(*thinned, config_.registration_voxel_m,
        10, 1);
    return {prepared.first, prepared.second};
  }

  static bool agrees(
    const Eigen::Isometry3d & a, const Eigen::Isometry3d & b, double meters, double degrees)
  {
    const auto yaw = [](const Eigen::Isometry3d & pose) {
        return std::atan2(pose.linear()(1, 0), pose.linear()(0, 0));
      };
    return (a.translation() - b.translation()).norm() <= meters &&
           std::abs(std::remainder(yaw(a) - yaw(b), 2 * M_PI)) <= degrees * M_PI / 180;
  }

  std::vector<std::size_t> neighbors(const Eigen::Vector3d & position) const
  {
    std::vector<std::pair<double, std::size_t>> nearby;
    for (std::size_t i = 0; i < reference_.size(); ++i) {
      const double distance = (reference_[i].frame.translation() - position).squaredNorm();
      if (distance <= 9.0 && !reference_[i].points.empty()) {
        nearby.emplace_back(distance, i);
      }
    }
    const auto count = std::min<std::size_t>(nearby.size(), 3);
    std::partial_sort(nearby.begin(), nearby.begin() + count, nearby.end());
    std::vector<std::size_t> result;
    for (std::size_t i = 0; i < count; ++i) {
      result.push_back(nearby[i].second);
    }
    return result;
  }

  Candidate align(
    std::size_t index, const detail::Prepared & source, const Eigen::Isometry3d & guess)
  {
    if (!prepared_[index]) {
      prepared_[index] = prepare(reference_[index]);
    }
    const auto & target = *prepared_[index];
    Eigen::Isometry3d relative = reference_[index].frame.inverse() * guess;
    for (const double distance : config_.registration_distances_m) {
      small_gicp::RegistrationSetting settings;
      settings.type = small_gicp::RegistrationSetting::GICP;
      settings.num_threads = 1;
      settings.max_correspondence_distance = distance;
      settings.max_iterations = config_.registration_max_iterations;
      relative = small_gicp::align(*target.cloud, *source.cloud, *target.tree, relative,
          settings).T_target_source;
    }
    return {index, reference_[index].frame * relative,
      detail::overlap(target, source, relative, config_.inlier_distance_m)};
  }

  JoinConfig config_;
  std::vector<Keyframe> reference_;
  std::vector<std::optional<detail::Prepared>> prepared_;
  std::optional<std::vector<std::pair<std::size_t, ScanContext::Descriptor>>> launch_descriptors_;
};

}  // namespace graphslam::multisession

#endif  // GRAPH_BASED_SLAM__STARTUP_RELOCALIZATION_HPP_
