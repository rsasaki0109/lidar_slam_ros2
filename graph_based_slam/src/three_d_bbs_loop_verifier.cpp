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

#include "graph_based_slam/three_d_bbs_loop_verifier.hpp"

#include <pcl/common/point_tests.h>

#include <algorithm>
#include <cmath>
#include <thread>
#include <vector>

#ifdef GRAPH_BASED_SLAM_HAVE_3D_BBS
#include "cpu_bbs3d/bbs3d.hpp"
#endif

namespace graphslam
{
namespace
{

std::vector<Eigen::Vector3d> toEigenPoints(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud)
{
  std::vector<Eigen::Vector3d> points;
  if (!cloud) {
    return points;
  }
  points.reserve(cloud->size());
  for (const auto & point : cloud->points) {
    if (!pcl::isFinite(point)) {
      continue;
    }
    points.emplace_back(point.x, point.y, point.z);
  }
  return points;
}

int resolveNumThreads(int requested_threads)
{
  if (requested_threads > 0) {
    return requested_threads;
  }
  const auto hw_threads = std::thread::hardware_concurrency();
  return std::max(1u, hw_threads);
}

}  // namespace

ThreeDBBSLoopVerification ThreeDBBSLoopVerifier::localize(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & source_local,
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & target_global,
  const Eigen::Isometry3d & source_pose_map,
  const Eigen::Isometry3d & search_center_pose_map,
  const ThreeDBBSLoopVerifierConfig & config) const
{
  ThreeDBBSLoopVerification result;
#ifndef GRAPH_BASED_SLAM_HAVE_3D_BBS
  static_cast<void>(source_local);
  static_cast<void>(target_global);
  static_cast<void>(source_pose_map);
  static_cast<void>(search_center_pose_map);
  static_cast<void>(config);
  return result;
#else
  result.available = true;

  const auto src_points = toEigenPoints(source_local);
  const auto tar_points = toEigenPoints(target_global);
  if (src_points.empty() || tar_points.empty()) {
    return result;
  }

  cpu::BBS3D bbs3d;
  bbs3d.set_tar_points(tar_points, config.min_level_res, config.max_level);

  const Eigen::Vector3d search_center_translation = search_center_pose_map.translation();
  const Eigen::Vector3d search_margin =
    Eigen::Vector3d::Constant(std::max(0.1, config.translation_search_margin_m));
  bbs3d.set_trans_search_range(
    search_center_translation - search_margin,
    search_center_translation + search_margin);

  const Eigen::Vector3d search_center_rpy =
    search_center_pose_map.rotation().eulerAngles(0, 1, 2);
  const double roll_pitch_margin_rad = config.roll_pitch_search_deg * M_PI / 180.0;
  const double yaw_margin_rad = config.yaw_search_deg * M_PI / 180.0;
  bbs3d.set_angular_search_range(
    search_center_rpy -
    Eigen::Vector3d(roll_pitch_margin_rad, roll_pitch_margin_rad, yaw_margin_rad),
    search_center_rpy +
    Eigen::Vector3d(roll_pitch_margin_rad, roll_pitch_margin_rad, yaw_margin_rad));

  bbs3d.set_score_threshold_percentage(config.score_threshold_percentage);
  bbs3d.set_num_threads(resolveNumThreads(config.num_threads));
  if (config.timeout_msec > 0) {
    bbs3d.enable_timeout();
    bbs3d.set_timeout_duration_in_msec(config.timeout_msec);
  } else {
    bbs3d.disable_timeout();
  }

  bbs3d.set_src_points(src_points);
  bbs3d.localize();

  result.timed_out = bbs3d.has_timed_out();
  result.elapsed_msec = bbs3d.get_elapsed_time();
  result.score_percentage = bbs3d.get_best_score_percentage();
  result.localized = bbs3d.has_localized();
  if (!result.localized) {
    return result;
  }

  const Eigen::Matrix4d absolute_pose = bbs3d.get_global_pose();
  result.correction_guess = (absolute_pose * source_pose_map.inverse().matrix()).cast<float>();
  return result;
#endif
}

}  // namespace graphslam
