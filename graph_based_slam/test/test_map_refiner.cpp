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

#include <cstring>
#include <random>
#include <string>
#include <vector>

#include "graph_based_slam/map_refiner.hpp"

namespace
{

using graphslam::map_refinement::expSe3;
using graphslam::map_refinement::leftPerturb;
using graphslam::map_refinement::MapRefinerConfig;
using graphslam::map_refinement::MapRefinerResult;
using graphslam::map_refinement::refineSubmapPoses;
using graphslam::map_refinement::refinerReportYamlLines;
using graphslam::map_refinement::Vector6d;

struct Fixture
{
  std::vector<std::vector<Eigen::Vector3d>> local_clouds;
  std::vector<Eigen::Matrix4d> ground_truth;
  std::vector<Eigen::Matrix4d> initial;
};

Eigen::Matrix4d inversePose(const Eigen::Matrix4d & pose)
{
  Eigen::Matrix4d inverse = Eigen::Matrix4d::Identity();
  const Eigen::Matrix3d rotation = pose.block<3, 3>(0, 0);
  inverse.block<3, 3>(0, 0) = rotation.transpose();
  inverse.block<3, 1>(0, 3) = -rotation.transpose() * pose.block<3, 1>(0, 3);
  return inverse;
}

// A small room: floor + two orthogonal walls, 6 poses, ~mm noise.
Fixture makeRoomFixture()
{
  std::mt19937 rng(31415);
  std::normal_distribution<double> noise(0.0, 0.004);
  std::vector<Eigen::Vector3d> world;
  for (int a = 0; a <= 30; ++a) {
    for (int b = 0; b <= 30; ++b) {
      const double u = 0.12 * a;
      const double v = 0.12 * b;
      world.push_back(Eigen::Vector3d(u, v, noise(rng)));
      world.push_back(Eigen::Vector3d(u, noise(rng), 0.3 + 0.8 * v / 3.6));
      world.push_back(Eigen::Vector3d(noise(rng), u, 0.3 + 0.8 * v / 3.6));
    }
  }

  Fixture fixture;
  for (int i = 0; i < 6; ++i) {
    Vector6d twist;
    twist << 0.45 * i, 0.3 * (i % 2), 0.0, 0.0, 0.0, 0.04 * i;
    fixture.ground_truth.push_back(expSe3(twist));
  }
  fixture.initial = fixture.ground_truth;
  for (size_t i = 1; i < fixture.initial.size(); ++i) {
    Vector6d delta;
    const double sign = (i % 2 == 0) ? 1.0 : -1.0;
    delta << 0.03 * sign, -0.02 * sign, 0.015 * sign, 0.008 * sign,
      -0.006 * sign, 0.01 * sign;
    fixture.initial[i] = leftPerturb(delta, fixture.initial[i]);
  }
  for (size_t p = 0; p < fixture.ground_truth.size(); ++p) {
    const Eigen::Matrix4d world_to_local = inversePose(fixture.ground_truth[p]);
    std::vector<Eigen::Vector3d> local;
    local.reserve(world.size());
    for (size_t i = 0; i < world.size(); ++i) {
      local.push_back(
        world_to_local.block<3, 3>(0, 0) * world[i] +
        world_to_local.block<3, 1>(0, 3));
    }
    fixture.local_clouds.push_back(local);
  }
  return fixture;
}

MapRefinerConfig smallConfig()
{
  MapRefinerConfig config;
  config.cloud_downsample_voxel = 0.05;
  config.pyramid.window_size = 4;
  config.pyramid.window_stride = 2;
  config.pyramid.window_ba.max_iterations = 15;
  config.pyramid.global_ba.max_iterations = 15;
  return config;
}

double meanError(
  const std::vector<Eigen::Matrix4d> & poses,
  const std::vector<Eigen::Matrix4d> & reference)
{
  double sum = 0.0;
  for (size_t i = 0; i < poses.size(); ++i) {
    sum += (poses[i].block<3, 1>(0, 3) - reference[i].block<3, 1>(0, 3)).norm();
  }
  return sum / static_cast<double>(poses.size());
}

TEST(MapRefiner, RefinesPerturbedRoomTowardGroundTruth) {
  const Fixture fixture = makeRoomFixture();
  const MapRefinerResult result =
    refineSubmapPoses(fixture.local_clouds, fixture.initial, smallConfig());
  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(result.status, "refined");
  EXPECT_GT(result.input_points, result.downsampled_points);
  const double input_error = meanError(fixture.initial, fixture.ground_truth);
  const double refined_error = meanError(result.poses, fixture.ground_truth);
  EXPECT_LT(refined_error, input_error);
}

TEST(MapRefiner, EmptyInputFallsBack) {
  const std::vector<std::vector<Eigen::Vector3d>> clouds;
  const std::vector<Eigen::Matrix4d> poses;
  const MapRefinerResult result =
    refineSubmapPoses(clouds, poses, smallConfig());
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.status, "empty_input");
}

TEST(MapRefiner, FeaturelessCloudsReturnInputBitwise) {
  Fixture fixture = makeRoomFixture();
  for (size_t i = 0; i < fixture.local_clouds.size(); ++i) {
    fixture.local_clouds[i].resize(3);
  }
  const MapRefinerResult result =
    refineSubmapPoses(fixture.local_clouds, fixture.initial, smallConfig());
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.status, "no_improvement");
  for (size_t i = 0; i < fixture.initial.size(); ++i) {
    EXPECT_EQ(
      0,
      std::memcmp(
        fixture.initial[i].data(), result.poses[i].data(), 16 * sizeof(double)));
  }
}

TEST(MapRefiner, ReportBytesAreDeterministic) {
  const Fixture fixture = makeRoomFixture();
  const MapRefinerConfig config = smallConfig();
  const MapRefinerResult first =
    refineSubmapPoses(fixture.local_clouds, fixture.initial, config);
  const MapRefinerResult second =
    refineSubmapPoses(fixture.local_clouds, fixture.initial, config);
  const std::vector<std::string> first_lines = refinerReportYamlLines(first, config);
  const std::vector<std::string> second_lines = refinerReportYamlLines(second, config);
  ASSERT_EQ(first_lines.size(), second_lines.size());
  for (size_t i = 0; i < first_lines.size(); ++i) {
    EXPECT_EQ(first_lines[i], second_lines[i]);
  }
  for (size_t p = 0; p < first.poses.size(); ++p) {
    EXPECT_EQ(
      0,
      std::memcmp(first.poses[p].data(), second.poses[p].data(), 16 * sizeof(double)));
  }
  ASSERT_FALSE(first_lines.empty());
  EXPECT_EQ(first_lines[0], "map_refinement_report:");
}

}  // namespace
