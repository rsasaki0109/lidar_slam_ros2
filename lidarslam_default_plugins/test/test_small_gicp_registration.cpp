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
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <string>

#include <Eigen/Geometry>  // NOLINT(build/include_order)
#include <pcl/common/transforms.h>  // NOLINT(build/include_order)
#include <small_gicp/pcl/pcl_registration.hpp>  // NOLINT(build/include_order)
#include <small_gicp/pcl/pcl_registration_impl.hpp>  // NOLINT(build/include_order)

#include "lidarslam_default_plugins/small_gicp_registration.hpp"
#include "lidarslam_default_plugins/small_gicp_registration_impl.ipp"

namespace
{

namespace registration = lidarslam::plugins::registration;
using Cloud = registration::PointCloud;
using PointT = registration::PointT;

Cloud::Ptr makeFixture()
{
  Cloud::Ptr cloud(new Cloud());
  for (int ix = -6; ix <= 6; ++ix) {
    for (int iy = -5; iy <= 5; ++iy) {
      for (int iz = 0; iz <= 3; ++iz) {
        PointT point;
        point.x = static_cast<float>(ix) * 0.37F;
        point.y = static_cast<float>(iy) * 0.41F;
        point.z = static_cast<float>(iz) * 0.29F;
        point.intensity = static_cast<float>((ix + 6) * 3 + (iy + 5));
        cloud->push_back(point);
      }
    }
  }
  return cloud;
}

Cloud::Ptr makeTarget(const Cloud::Ptr & source)
{
  Cloud::Ptr target(new Cloud());
  const Eigen::Affine3f transform = Eigen::Translation3f(0.18F, -0.12F, 0.08F) *
    Eigen::AngleAxisf(0.035F, Eigen::Vector3f::UnitZ()) *
    Eigen::AngleAxisf(-0.021F, Eigen::Vector3f::UnitY());
  pcl::transformPointCloud(*source, *target, transform.matrix());
  return target;
}

registration::ParameterMap makeParameters(const bool voxelized)
{
  registration::ParameterMap parameters;
  parameters.emplace(
    "maximum_correspondence_distance", registration::ParameterValue(5.0));
  parameters.emplace("transformation_epsilon", registration::ParameterValue(1e-6));
  parameters.emplace("maximum_iterations", registration::ParameterValue(std::int64_t{35}));
  parameters.emplace("num_threads", registration::ParameterValue(std::int64_t{1}));
  parameters.emplace(
    "adaptive_correspondence_threshold", registration::ParameterValue(false));
  if (voxelized) {
    parameters.emplace("voxel_resolution", registration::ParameterValue(0.6));
  }
  return parameters;
}

struct DirectResult
{
  Eigen::Matrix4f transform{Eigen::Matrix4f::Identity()};
  Cloud aligned;
  double fitness{0.0};
  bool converged{false};
};

DirectResult runDirect(
  const Cloud::Ptr & source, const Cloud::Ptr & target, const bool voxelized,
  const double maximum_correspondence_distance = 5.0)
{
  using Small = small_gicp::RegistrationPCL<PointT, PointT>;
  Small registration;
  registration.setRegistrationType(voxelized ? "VGICP" : "GICP");
  if (voxelized) {
    registration.setVoxelResolution(0.6);
  }
  registration.setMaxCorrespondenceDistance(maximum_correspondence_distance);
  registration.setTransformationEpsilon(1e-6);
  registration.setMaximumIterations(35);
  registration.setNumThreads(1);
  registration.setInputSource(source);
  registration.setInputTarget(target);
  DirectResult result;
  registration.align(result.aligned, Eigen::Matrix4f::Identity());
  result.transform = registration.getFinalTransformation();
  result.fitness = registration.getFitnessScore();
  result.converged = registration.hasConverged();
  return result;
}

bool bitwiseEqual(const Eigen::Matrix4f & lhs, const Eigen::Matrix4f & rhs)
{
  return std::memcmp(lhs.data(), rhs.data(), sizeof(float) * 16U) == 0;
}

bool bitwiseEqual(const Cloud & lhs, const Cloud & rhs)
{
  if (
    lhs.width != rhs.width || lhs.height != rhs.height || lhs.is_dense != rhs.is_dense ||
    lhs.points.size() != rhs.points.size())
  {
    return false;
  }
  return lhs.points.empty() || std::memcmp(
    lhs.points.data(), rhs.points.data(), lhs.points.size() * sizeof(PointT)) == 0;
}

template<typename Plugin>
void checkFixture(const bool voxelized, const std::string & expected_class)
{
  const Cloud::Ptr source = makeFixture();
  const Cloud::Ptr target = makeTarget(source);
  const DirectResult direct = runDirect(source, target, voxelized);

  Plugin plugin;
  const auto metadata = plugin.metadata();
  EXPECT_EQ(metadata.class_id, expected_class);
  EXPECT_EQ(metadata.license, "BSD-2-Clause");
  EXPECT_TRUE(registration::isApiCompatible(registration::kHostApiVersion, metadata.api_version));
  EXPECT_FALSE(plugin.capabilities().has(registration::Capability::kDeterministic));

  std::string error;
  ASSERT_TRUE(plugin.configure(makeParameters(voxelized), &error)) << error;
  EXPECT_TRUE(plugin.capabilities().has(registration::Capability::kDeterministic));
  ASSERT_TRUE(plugin.setInputTarget(target, &error)) << error;
  registration::AlignmentRequest request;
  request.source = source;
  request.initial_guess_enabled = true;
  request.initial_guess = Eigen::Matrix4f::Identity();
  const auto result = plugin.align(request);
  ASSERT_EQ(result.failure, registration::FailureCode::kNone) << result.diagnostics.detail;
  ASSERT_TRUE(result.aligned_source);
  EXPECT_EQ(result.converged, direct.converged);
  EXPECT_TRUE(bitwiseEqual(result.final_transformation, direct.transform));
  EXPECT_DOUBLE_EQ(result.fitness_score, direct.fitness);
  EXPECT_TRUE(bitwiseEqual(*result.aligned_source, direct.aligned));
  EXPECT_TRUE(result.diagnostics.mean_correspondence_distance_valid);
  EXPECT_DOUBLE_EQ(
    result.diagnostics.mean_correspondence_distance,
    direct.fitness > 0.0 ? std::sqrt(direct.fitness) : 0.0);

  // The capability is only advertised for num_threads=1 after this repeated
  // call has demonstrated byte-stable output for the configured adapter.
  const auto repeated = plugin.align(request);
  ASSERT_EQ(repeated.failure, registration::FailureCode::kNone) << repeated.diagnostics.detail;
  ASSERT_TRUE(repeated.aligned_source);
  EXPECT_TRUE(bitwiseEqual(result.final_transformation, repeated.final_transformation));
  EXPECT_DOUBLE_EQ(result.fitness_score, repeated.fitness_score);
  EXPECT_TRUE(bitwiseEqual(*result.aligned_source, *repeated.aligned_source));
}

TEST(SmallGicpRegistration, GicpMatchesDirectFixture)
{
  checkFixture<lidarslam_default_plugins::SmallGicpRegistration>(
    false, "lidarslam_default_plugins/SmallGicpPcl");
}

TEST(SmallGicpRegistration, VgicpMatchesDirectFixture)
{
  checkFixture<lidarslam_default_plugins::SmallVgicpRegistration>(
    true, "lidarslam_default_plugins/SmallVGicpPcl");
}

TEST(SmallGicpRegistration, DisabledInitialGuessIgnoresNaNGuess)
{
  const Cloud::Ptr source = makeFixture();
  const Cloud::Ptr target = makeTarget(source);
  lidarslam_default_plugins::SmallGicpRegistration plugin;
  std::string error;
  ASSERT_TRUE(plugin.configure(makeParameters(false), &error)) << error;
  ASSERT_TRUE(plugin.setInputTarget(target, &error)) << error;
  registration::AlignmentRequest request;
  request.source = source;
  request.initial_guess_enabled = false;
  request.initial_guess.setConstant(std::numeric_limits<float>::quiet_NaN());
  const auto result = plugin.align(request);
  EXPECT_EQ(result.failure, registration::FailureCode::kNone) << result.diagnostics.detail;
}

template<typename Plugin>
void checkAdaptiveReset(const bool voxelized)
{
  const Cloud::Ptr source = makeFixture();
  const Cloud::Ptr target = makeTarget(source);
  auto parameters = makeParameters(voxelized);
  parameters.at("adaptive_correspondence_threshold") = registration::ParameterValue(true);

  Plugin plugin;
  std::string error;
  ASSERT_TRUE(plugin.configure(parameters, &error)) << error;
  ASSERT_TRUE(plugin.setInputTarget(target, &error)) << error;

  registration::AlignmentRequest request;
  request.source = source;
  request.initial_guess_enabled = true;
  request.initial_guess = Eigen::Matrix4f::Identity();

  // The legacy component starts adaptive calls with the configured threshold,
  // then leaves DBL_MAX in the underlying RegistrationPCL for the next call.
  const DirectResult direct_first = runDirect(source, target, voxelized, 5.0);
  const DirectResult direct_second = runDirect(
    source, target, voxelized, std::numeric_limits<double>::max());
  const auto first = plugin.align(request);
  const auto second = plugin.align(request);
  ASSERT_EQ(first.failure, registration::FailureCode::kNone) << first.diagnostics.detail;
  ASSERT_EQ(second.failure, registration::FailureCode::kNone) << second.diagnostics.detail;
  EXPECT_TRUE(bitwiseEqual(first.final_transformation, direct_first.transform));
  EXPECT_DOUBLE_EQ(first.fitness_score, direct_first.fitness);
  ASSERT_TRUE(first.aligned_source);
  EXPECT_TRUE(bitwiseEqual(*first.aligned_source, direct_first.aligned));
  EXPECT_TRUE(bitwiseEqual(second.final_transformation, direct_second.transform));
  EXPECT_DOUBLE_EQ(second.fitness_score, direct_second.fitness);
  ASSERT_TRUE(second.aligned_source);
  EXPECT_TRUE(bitwiseEqual(*second.aligned_source, direct_second.aligned));
}

TEST(SmallGicpRegistration, AdaptiveGicpResetsDistanceAfterEveryCall)
{
  checkAdaptiveReset<lidarslam_default_plugins::SmallGicpRegistration>(false);
}

TEST(SmallGicpRegistration, AdaptiveVgicpResetsDistanceAfterEveryCall)
{
  checkAdaptiveReset<lidarslam_default_plugins::SmallVgicpRegistration>(true);
}

template<typename Plugin>
void checkDeterminismRequiresOneThread(const bool voxelized)
{
  for (const std::int64_t threads : {std::int64_t{0}, std::int64_t{2}}) {
    Plugin plugin;
    auto parameters = makeParameters(voxelized);
    parameters.at("num_threads") = registration::ParameterValue(threads);
    std::string error;
    ASSERT_TRUE(plugin.configure(parameters, &error)) << error;
    EXPECT_FALSE(plugin.capabilities().has(registration::Capability::kDeterministic));
  }
}

TEST(SmallGicpRegistration, DeterminismIsNotAdvertisedForDefaultOrParallelThreads)
{
  checkDeterminismRequiresOneThread<lidarslam_default_plugins::SmallGicpRegistration>(false);
}

TEST(SmallGicpRegistration, VgicpDeterminismIsNotAdvertisedForDefaultOrParallelThreads)
{
  checkDeterminismRequiresOneThread<lidarslam_default_plugins::SmallVgicpRegistration>(true);
}

TEST(SmallGicpRegistration, RejectsVariantSpecificConfiguration)
{
  lidarslam_default_plugins::SmallGicpRegistration gicp;
  std::string error;
  auto parameters = makeParameters(false);
  parameters.emplace("voxel_resolution", registration::ParameterValue(0.5));
  EXPECT_FALSE(gicp.configure(parameters, &error));
  EXPECT_NE(error.find("voxel_resolution"), std::string::npos);

  lidarslam_default_plugins::SmallVgicpRegistration vgicp;
  error.clear();
  parameters = makeParameters(true);
  parameters.erase("voxel_resolution");
  EXPECT_FALSE(vgicp.configure(parameters, &error));
  EXPECT_NE(error.find("voxel_resolution"), std::string::npos);
}

}  // namespace
