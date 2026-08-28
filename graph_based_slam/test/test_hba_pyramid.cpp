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

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include <Eigen/Dense>  // NOLINT(build/include_order)

#include "graph_based_slam/hba_pyramid.hpp"
#include "graph_based_slam/se3_lie.hpp"

namespace graphslam
{
namespace map_refinement
{
namespace
{

constexpr int kPoseCount = 12;
constexpr double kSpacing = 0.8;

struct CorridorFixture
{
  std::vector<Eigen::Matrix4d> ground_truth;
  std::vector<Eigen::Matrix4d> initial;
  std::vector<std::vector<Eigen::Vector3d>> local_clouds;
};

std::vector<Eigen::Vector3d> makeCorridorWorld()
{
  std::vector<Eigen::Vector3d> points;
  points.reserve(1500);

  for (int ix = 0; ix <= 44; ++ix) {
    const double x = 0.25 * static_cast<double>(ix);

    for (int iy = 0; iy <= 12; ++iy) {
      const double y = 0.25 * static_cast<double>(iy);
      points.push_back(Eigen::Vector3d(x, y, 0.0));
    }

    for (int iz = 1; iz <= 9; ++iz) {
      const double z = 0.25 * static_cast<double>(iz);
      points.push_back(Eigen::Vector3d(x, 0.0, z));
      points.push_back(Eigen::Vector3d(x, 3.0, z));
    }
  }

  // Transverse doorframe walls every 2.0 m: without them a corridor of
  // floor + two parallel walls leaves translation along x unobservable
  // (the classic degenerate scene) and the priors-off recovery test
  // would have no data to recover x from.
  const double doorframe_x[6] = {0.0, 1.7, 3.9, 6.3, 8.2, 10.6};
  for (int iw = 0; iw <= 5; ++iw) {
    const double x = doorframe_x[iw];
    for (int iy = 0; iy <= 12; ++iy) {
      const double y = 0.25 * static_cast<double>(iy);
      for (int iz = 1; iz <= 9; ++iz) {
        const double z = 0.25 * static_cast<double>(iz);
        if (y > 0.75 && y < 2.25 && z < 2.0) {
          continue;  // door opening
        }
        points.push_back(Eigen::Vector3d(x, y, z));
      }
    }
  }

  return points;
}

Eigen::Matrix4d makeGroundTruthPose(const int index)
{
  Vector6d yaw_delta = Vector6d::Zero();
  yaw_delta(5) = 0.015 * std::sin(0.4 * static_cast<double>(index));

  Eigen::Matrix4d pose = expSe3(yaw_delta);
  pose(0, 3) = kSpacing * static_cast<double>(index);
  pose(1, 3) = 1.5;
  pose(2, 3) = 1.0;
  return pose;
}

Eigen::Matrix4d perturbPose(
  const Eigen::Matrix4d & pose,
  const int index)
{
  if (index == 0) {
    return pose;
  }

  const double even_sign = (index % 2 == 0) ? 1.0 : -1.0;
  const double third_sign = (index % 3 == 0) ? -1.0 : 1.0;

  Vector6d translation_delta = Vector6d::Zero();
  translation_delta(1) = even_sign * (0.024 + 0.001 * index);
  translation_delta(2) = third_sign * 0.018;

  Eigen::Matrix4d moved = leftPerturb(translation_delta, pose);

  Vector6d rotation_delta = Vector6d::Zero();
  rotation_delta(3) = even_sign * 0.004;
  rotation_delta(4) = third_sign * 0.003;
  rotation_delta(5) = -even_sign * 0.006;

  return moved * expSe3(rotation_delta);
}

std::vector<std::vector<Eigen::Vector3d>> makeLocalClouds(
  const std::vector<Eigen::Vector3d> & world_points,
  const std::vector<Eigen::Matrix4d> & ground_truth)
{
  std::vector<std::vector<Eigen::Vector3d>> local_clouds;
  local_clouds.reserve(ground_truth.size());

  for (const Eigen::Matrix4d & pose : ground_truth) {
    std::vector<Eigen::Vector3d> local_points;
    const double pose_x = pose(0, 3);
    const Eigen::Matrix3d rotation = pose.block<3, 3>(0, 0);
    const Eigen::Vector3d translation = pose.block<3, 1>(0, 3);

    for (const Eigen::Vector3d & point : world_points) {
      if (std::fabs(point.x() - pose_x) > 2.5) {
        continue;
      }
      local_points.push_back(rotation.transpose() * (point - translation));
    }

    local_clouds.push_back(local_points);
  }

  return local_clouds;
}

CorridorFixture makeCorridorFixture()
{
  CorridorFixture fixture;
  fixture.ground_truth.reserve(kPoseCount);

  for (int i = 0; i < kPoseCount; ++i) {
    fixture.ground_truth.push_back(makeGroundTruthPose(i));
  }

  fixture.initial = fixture.ground_truth;
  for (std::size_t i = 1; i < fixture.initial.size(); ++i) {
    fixture.initial[i] = perturbPose(fixture.ground_truth[i], static_cast<int>(i));
  }

  const std::vector<Eigen::Vector3d> world_points = makeCorridorWorld();
  fixture.local_clouds = makeLocalClouds(world_points, fixture.ground_truth);
  return fixture;
}

HbaPyramidConfig makeConfig(const bool run_global_pass)
{
  HbaPyramidConfig config;
  config.window_size = 6;
  config.window_stride = 3;
  config.run_global_pass = run_global_pass;
  config.global_pass_max_poses = 64;
  config.association.min_observing_poses = 2;
  config.association.min_points_per_observation = 5;
  config.window_ba.max_iterations = 15;
  // Production defaults keep the soft priors ON: they are what prevents
  // the optimizer from jumping to a repeated-geometry false minimum
  // (the design note's "polish the wrong map" failure).
  config.global_ba = config.window_ba;
  config.global_ba.max_iterations = 15;
  return config;
}

double translationError(
  const Eigen::Matrix4d & pose,
  const Eigen::Matrix4d & ground_truth)
{
  const Eigen::Vector3d pose_t = pose.block<3, 1>(0, 3);
  const Eigen::Vector3d truth_t = ground_truth.block<3, 1>(0, 3);
  return (pose_t - truth_t).norm();
}

double meanTranslationError(
  const std::vector<Eigen::Matrix4d> & poses,
  const std::vector<Eigen::Matrix4d> & ground_truth)
{
  if (poses.empty()) {
    return 0.0;
  }

  double sum = 0.0;
  for (std::size_t i = 0; i < poses.size(); ++i) {
    sum += translationError(poses[i], ground_truth[i]);
  }
  return sum / static_cast<double>(poses.size());
}

bool matricesBitwiseEqual(
  const Eigen::Matrix4d & lhs,
  const Eigen::Matrix4d & rhs)
{
  return std::memcmp(lhs.data(), rhs.data(), 16 * sizeof(double)) == 0;
}

}  // namespace

TEST(HbaPyramidTest, RecoversTrajectoryAndReportsCoverage)
{
  const CorridorFixture fixture = makeCorridorFixture();
  const HbaPyramidConfig config = makeConfig(true);

  const HbaPyramidResult result = refinePosesHierarchically(
    fixture.local_clouds,
    fixture.initial,
    config);

  ASSERT_EQ(fixture.ground_truth.size(), result.poses.size());

  const double input_mean = meanTranslationError(
    fixture.initial,
    fixture.ground_truth);
  const double refined_mean = meanTranslationError(
    result.poses,
    fixture.ground_truth);

  for (std::size_t i = 0; i < result.poses.size(); ++i) {
    const double input_error = translationError(
      fixture.initial[i],
      fixture.ground_truth[i]);
    const double refined_error = translationError(
      result.poses[i],
      fixture.ground_truth[i]);
    // Aggregate improvement is the oracle (mean falls at least 45% below);
    // individual poses may trade a few mm against the gauge anchor and
    // the soft priors.
    EXPECT_LE(refined_error, input_error + 0.01) << "pose " << i;
  }

  EXPECT_LT(refined_mean, 0.55 * input_mean);
  EXPECT_GE(result.windows.size(), static_cast<std::size_t>(3));
  ASSERT_FALSE(result.windows.empty());

  const WindowReport & last_window = result.windows.back();
  const int expected_end = static_cast<int>(fixture.ground_truth.size());
  EXPECT_EQ(expected_end, last_window.start_index + last_window.pose_count);
  EXPECT_TRUE(matricesBitwiseEqual(result.poses.front(), fixture.initial.front()));
  EXPECT_TRUE(result.global_pass_ran);
}

TEST(HbaPyramidTest, IsDeterministic)
{
  const CorridorFixture fixture = makeCorridorFixture();
  const HbaPyramidConfig config = makeConfig(true);

  const HbaPyramidResult first = refinePosesHierarchically(
    fixture.local_clouds,
    fixture.initial,
    config);
  const HbaPyramidResult second = refinePosesHierarchically(
    fixture.local_clouds,
    fixture.initial,
    config);

  ASSERT_EQ(first.poses.size(), second.poses.size());

  for (std::size_t i = 0; i < first.poses.size(); ++i) {
    const bool same = matricesBitwiseEqual(first.poses[i], second.poses[i]);
    EXPECT_TRUE(same) << "pose " << i;
  }
}

TEST(HbaPyramidTest, HandlesZeroFeatureCloudsWithoutChangingPoses)
{
  const CorridorFixture fixture = makeCorridorFixture();
  const HbaPyramidConfig config = makeConfig(true);

  std::vector<std::vector<Eigen::Vector3d>> small_clouds(fixture.initial.size());
  for (std::size_t i = 0; i < small_clouds.size(); ++i) {
    small_clouds[i].push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
    small_clouds[i].push_back(Eigen::Vector3d(0.2, 0.0, 0.1));
    small_clouds[i].push_back(Eigen::Vector3d(0.0, 0.2, 0.1));
  }

  const HbaPyramidResult result = refinePosesHierarchically(
    small_clouds,
    fixture.initial,
    config);

  ASSERT_EQ(fixture.initial.size(), result.poses.size());
  ASSERT_FALSE(result.windows.empty());

  for (const WindowReport & report : result.windows) {
    EXPECT_EQ("no_valid_features", report.termination);
    EXPECT_EQ(0, report.features);
    EXPECT_FALSE(report.improved);
  }

  for (std::size_t i = 0; i < result.poses.size(); ++i) {
    const bool same = matricesBitwiseEqual(result.poses[i], fixture.initial[i]);
    EXPECT_TRUE(same) << "pose " << i;
  }

  EXPECT_FALSE(result.any_window_improved);
  EXPECT_TRUE(result.global_pass_ran);
  EXPECT_EQ("no_valid_features", result.global_report.termination);
}

TEST(HbaPyramidTest, WindowOnlyRunSkipsGlobalPassAndStillImproves)
{
  const CorridorFixture fixture = makeCorridorFixture();
  const HbaPyramidConfig config = makeConfig(false);

  const HbaPyramidResult result = refinePosesHierarchically(
    fixture.local_clouds,
    fixture.initial,
    config);

  const double input_mean = meanTranslationError(
    fixture.initial,
    fixture.ground_truth);
  const double refined_mean = meanTranslationError(
    result.poses,
    fixture.ground_truth);

  EXPECT_FALSE(result.global_pass_ran);
  EXPECT_LT(refined_mean, input_mean);
}

TEST(HbaPyramidTest, OverlappingWindowsKeepPriorsAnchoredToOriginalPoses)
{
  const CorridorFixture fixture = makeCorridorFixture();
  HbaPyramidConfig config = makeConfig(false);
  config.window_size = 6;
  config.window_stride = 1;
  config.window_ba.prior_translation_sigma = 0.001;
  config.window_ba.prior_rotation_sigma_rad = 0.0002;

  const HbaPyramidResult result = refinePosesHierarchically(
    fixture.local_clouds, fixture.initial, config);

  ASSERT_EQ(result.poses.size(), fixture.initial.size());
  double max_translation_correction = 0.0;
  for (std::size_t i = 0; i < result.poses.size(); ++i) {
    max_translation_correction = std::max(
      max_translation_correction,
      (result.poses[i].block<3, 1>(0, 3) -
      fixture.initial[i].block<3, 1>(0, 3)).norm());
  }
  EXPECT_LT(max_translation_correction, 0.002);
}

}  // namespace map_refinement
}  // namespace graphslam
