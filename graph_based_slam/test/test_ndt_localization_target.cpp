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
#include <cstring>
#include <limits>
#include <sstream>
#include <vector>

#include "graph_based_slam/ndt_localization_target.hpp"
#include "graph_based_slam/ndt_trajectory_io.hpp"

TEST(NdtLocalizationTarget, AddsSymmetricPlanarAxisAndDiagonalSamples)
{
  std::vector<Eigen::Vector3d> input;
  for (int x = 1; x <= 3; ++x) {
    for (int y = 1; y <= 3; ++y) {
      input.emplace_back(0.1 * x, 0.1 * y, 0.0);
    }
  }
  graphslam::ndt_localization::TangentSamplingConfig config;
  config.voxel_size_m = 1.0;
  config.radius_m = 0.5;
  config.add_diagonals = true;

  const auto result = graphslam::ndt_localization::buildTangentSampledTarget(input, config);

  EXPECT_EQ(result.input_points, 9U);
  EXPECT_EQ(result.planar_voxels, 1U);
  EXPECT_EQ(result.planar_input_points, 9U);
  EXPECT_EQ(result.sampled_points, 72U);
  ASSERT_EQ(result.points.size(), 81U);
  Eigen::Vector3d input_sum = Eigen::Vector3d::Zero();
  Eigen::Vector3d output_sum = Eigen::Vector3d::Zero();
  for (const Eigen::Vector3d & point : input) {
    input_sum += point;
                                                                  }
  for (const Eigen::Vector3d & point : result.points) {
    output_sum += point;
    EXPECT_NEAR(point.z(), 0.0, 1.0e-12);
  }
  EXPECT_TRUE(output_sum.isApprox(9.0 * input_sum, 1.0e-12));
}

TEST(NdtLocalizationTarget, RejectsVolumetricCellsWithoutDeletingInput)
{
  std::vector<Eigen::Vector3d> input;
  for (const double x : {0.1, 0.8}) {
    for (const double y : {0.1, 0.8}) {
      for (const double z : {0.1, 0.8}) {
        input.emplace_back(x, y, z);
      }
    }
  }

  const auto result = graphslam::ndt_localization::buildTangentSampledTarget(input);

  EXPECT_EQ(result.planar_voxels, 0U);
  EXPECT_EQ(result.sampled_points, 0U);
  ASSERT_EQ(result.points.size(), input.size());
  for (std::size_t i = 0U; i < input.size(); ++i) {
    EXPECT_EQ(
      0, std::memcmp(result.points[i].data(), input[i].data(), 3U * sizeof(double)));
  }
}

TEST(NdtLocalizationTarget, IsByteDeterministicAndRejectsRadiusBeyondHalfVoxel)
{
  std::vector<Eigen::Vector3d> input;
  for (int x = 1; x <= 3; ++x) {
    for (int y = 1; y <= 3; ++y) {
      input.emplace_back(0.1 * x, 0.1 * y, 0.0);
    }
  }
  const auto first = graphslam::ndt_localization::buildTangentSampledTarget(input);
  const auto second = graphslam::ndt_localization::buildTangentSampledTarget(input);
  ASSERT_EQ(first.points.size(), second.points.size());
  for (std::size_t i = 0U; i < first.points.size(); ++i) {
    EXPECT_EQ(
      0, std::memcmp(
        first.points[i].data(), second.points[i].data(), 3U * sizeof(double)));
  }

  graphslam::ndt_localization::TangentSamplingConfig invalid;
  invalid.radius_m = 0.500001;
  const auto rejected = graphslam::ndt_localization::buildTangentSampledTarget(input, invalid);
  EXPECT_TRUE(rejected.points.empty());
}

TEST(NdtLocalizationTarget, WritesDeterministicCanonicalTumPoses)
{
  graphslam::ndt_localization::RegisteredPose first;
  first.pair_index = 7U;
  first.stamp_sec = 123.25;
  first.converged = true;
  first.translation = Eigen::Vector3d(1.0, -2.0, 3.5);
  first.orientation = Eigen::Quaterniond(-2.0, 0.0, 0.0, 0.0);
  graphslam::ndt_localization::RegisteredPose second = first;
  second.orientation = Eigen::Quaterniond(2.0, 0.0, 0.0, 0.0);

  std::ostringstream first_output;
  std::ostringstream second_output;
  graphslam::ndt_localization::writeRegisteredPoseTum(first_output, {first});
  graphslam::ndt_localization::writeRegisteredPoseTum(second_output, {second});

  EXPECT_EQ(first_output.str(), "123.25 1 -2 3.5 0 0 0 1\n");
  EXPECT_EQ(first_output.str(), second_output.str());
}

TEST(NdtLocalizationTarget, RejectsNonFiniteTumPose)
{
  graphslam::ndt_localization::RegisteredPose pose;
  pose.stamp_sec = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(graphslam::ndt_localization::registeredPoseTumLine(pose), std::invalid_argument);
}

TEST(NdtLocalizationTarget, RegularizesPoseUpdateInSe3)
{
  Eigen::Affine3d initial = Eigen::Affine3d::Identity();
  initial.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  Eigen::Affine3d optimized = Eigen::Affine3d::Identity();
  optimized.translation() = Eigen::Vector3d(3.0, 4.0, 5.0);
  optimized.linear() = Eigen::AngleAxisd(
    M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  const Eigen::Affine3d guarded = graphslam::ndt_localization::regularizePoseUpdate(
    initial, optimized.matrix(), 0.5);

  EXPECT_TRUE(guarded.translation().isApprox(Eigen::Vector3d(2.0, 3.0, 4.0)));
  EXPECT_NEAR(
    Eigen::AngleAxisd(guarded.rotation()).angle(), M_PI / 4.0,
    std::numeric_limits<double>::epsilon() * 16.0);
  EXPECT_TRUE(
    graphslam::ndt_localization::regularizePoseUpdate(
      initial, optimized.matrix(), 0.0).matrix().isApprox(initial.matrix()));
  EXPECT_TRUE(
    graphslam::ndt_localization::regularizePoseUpdate(
      initial, optimized.matrix(), 1.0).matrix().isApprox(optimized.matrix()));
  EXPECT_THROW(
    graphslam::ndt_localization::regularizePoseUpdate(initial, optimized.matrix(), 1.1),
    std::invalid_argument);
}

TEST(NdtLocalizationTarget, AddsASecondSymmetricInnerRing)
{
  std::vector<Eigen::Vector3d> input;
  for (int x = 1; x <= 3; ++x) {
    for (int y = 1; y <= 3; ++y) {
      input.emplace_back(0.1 * x, 0.1 * y, 0.0);
    }
  }
  graphslam::ndt_localization::TangentSamplingConfig config;
  config.inner_radius_m = 0.25;

  const auto result = graphslam::ndt_localization::buildTangentSampledTarget(input, config);

  EXPECT_EQ(result.planar_input_points, 9U);
  EXPECT_EQ(result.sampled_points, 144U);
  ASSERT_EQ(result.points.size(), 153U);
  for (const Eigen::Vector3d & point : result.points) {
    EXPECT_NEAR(point.z(), 0.0, 1.0e-12);
  }
  config.inner_radius_m = config.radius_m;
  EXPECT_TRUE(
    graphslam::ndt_localization::buildTangentSampledTarget(input, config).points.empty());
}

TEST(NdtLocalizationTarget, AddsSymmetricAngularMidpointsWithoutNormalThickness)
{
  std::vector<Eigen::Vector3d> input;
  for (int x = 1; x <= 3; ++x) {
    for (int y = 1; y <= 3; ++y) {
      input.emplace_back(0.1 * x, 0.1 * y, 0.0);
    }
  }
  graphslam::ndt_localization::TangentSamplingConfig config;
  config.add_angular_midpoints = true;

  const auto result = graphslam::ndt_localization::buildTangentSampledTarget(input, config);

  EXPECT_EQ(result.sampled_points, 144U);
  ASSERT_EQ(result.points.size(), 153U);
  Eigen::Vector3d input_sum = Eigen::Vector3d::Zero();
  Eigen::Vector3d output_sum = Eigen::Vector3d::Zero();
  for (const Eigen::Vector3d & point : input) {
    input_sum += point;
                                                                  }
  for (const Eigen::Vector3d & point : result.points) {
    output_sum += point;
    EXPECT_NEAR(point.z(), 0.0, 1.0e-12);
  }
  EXPECT_TRUE(output_sum.isApprox(17.0 * input_sum, 1.0e-12));
}

TEST(NdtLocalizationTarget, AddsTwoOrthogonalAngularMidpointPairs)
{
  std::vector<Eigen::Vector3d> input;
  for (int x = 1; x <= 3; ++x) {
    for (int y = 1; y <= 3; ++y) {
      input.emplace_back(0.1 * x, 0.1 * y, 0.0);
    }
  }
  graphslam::ndt_localization::TangentSamplingConfig config;
  config.inner_radius_m = 0.25;
  config.angular_midpoint_pairs = 2U;

  const auto result = graphslam::ndt_localization::buildTangentSampledTarget(input, config);

  EXPECT_EQ(result.sampled_points, 180U);
  ASSERT_EQ(result.points.size(), 189U);
  Eigen::Vector3d input_sum = Eigen::Vector3d::Zero();
  Eigen::Vector3d output_sum = Eigen::Vector3d::Zero();
  for (const Eigen::Vector3d & point : input) {
    input_sum += point;
                                                                  }
  for (const Eigen::Vector3d & point : result.points) {
    output_sum += point;
    EXPECT_NEAR(point.z(), 0.0, 1.0e-12);
  }
  EXPECT_TRUE(output_sum.isApprox(21.0 * input_sum, 1.0e-12));
}
