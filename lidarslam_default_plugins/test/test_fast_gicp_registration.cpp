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
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>

#include <Eigen/Geometry>  // NOLINT(build/include_order)
#include <fast_gicp/gicp/fast_gicp.hpp>  // NOLINT(build/include_order)
#include <fast_gicp/gicp/fast_vgicp.hpp>  // NOLINT(build/include_order)
#include <pcl/common/transforms.h>  // NOLINT(build/include_order)

#include "lidarslam_default_plugins/fast_gicp_registration.hpp"

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
  for (int i = 0; i < 12; ++i) {
    PointT point;
    point.x = 2.0F + static_cast<float>(i) * 0.07F;
    point.y = -1.4F + static_cast<float>(i % 4) * 0.13F;
    point.z = 1.1F + static_cast<float>(i / 4) * 0.17F;
    point.intensity = 100.0F + static_cast<float>(i);
    cloud->push_back(point);
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
  parameters.emplace(
    "maximum_iterations", registration::ParameterValue(std::int64_t{35}));
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

DirectResult runDirectFastGicp(const Cloud::Ptr & source, const Cloud::Ptr & target)
{
  fast_gicp::FastGICP<PointT, PointT> registration;
  registration.setMaxCorrespondenceDistance(5.0);
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

DirectResult runDirectFastVgicp(const Cloud::Ptr & source, const Cloud::Ptr & target)
{
  fast_gicp::FastVGICP<PointT, PointT> registration;
  registration.setMaxCorrespondenceDistance(5.0);
  registration.setTransformationEpsilon(1e-6);
  registration.setMaximumIterations(35);
  registration.setResolution(0.6);
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
  // PointXYZI contains alignment padding whose bytes are not part of the point
  // value and are allowed to remain indeterminate.  Comparing the complete
  // object representation therefore gives toolchain-dependent failures (for
  // example, Ubuntu 22.04/Humble) even when every public field is bit-identical.
  for (std::size_t index = 0; index < lhs.points.size(); ++index) {
    const auto & left = lhs.points[index];
    const auto & right = rhs.points[index];
    if (
      std::memcmp(&left.x, &right.x, sizeof(float)) != 0 ||
      std::memcmp(&left.y, &right.y, sizeof(float)) != 0 ||
      std::memcmp(&left.z, &right.z, sizeof(float)) != 0 ||
      std::memcmp(&left.intensity, &right.intensity, sizeof(float)) != 0)
    {
      return false;
    }
  }
  return true;
}

template<typename Plugin>
void checkPlugin(
  Plugin * plugin, const bool voxelized, const std::string & expected_class,
  const DirectResult & direct)
{
  const auto metadata = plugin->metadata();
  EXPECT_EQ(metadata.class_id, expected_class);
  EXPECT_EQ(metadata.license, "BSD-2-Clause");
  EXPECT_TRUE(registration::isApiCompatible(registration::kHostApiVersion, metadata.api_version));
  const auto capabilities = plugin->capabilities();
  EXPECT_TRUE(capabilities.has(registration::Capability::kInitialGuess));
  EXPECT_TRUE(capabilities.has(registration::Capability::kMaximumCorrespondenceDistance));
  EXPECT_TRUE(capabilities.has(registration::Capability::kAlignedSource));
  EXPECT_FALSE(capabilities.has(registration::Capability::kDeterministic));
  EXPECT_EQ(capabilities.targetPolicy(), registration::TargetPolicy::kAcceptHostPrepared);
  EXPECT_EQ(
    capabilities.correspondenceMetric(),
    registration::CorrespondenceMetric::kSquareRootFitnessProxy);

  const Cloud::Ptr source = makeFixture();
  const Cloud::Ptr target = makeTarget(source);
  std::string error;
  ASSERT_TRUE(plugin->configure(makeParameters(voxelized), &error)) << error;
  ASSERT_TRUE(plugin->setInputTarget(target, &error)) << error;
  registration::AlignmentRequest request;
  request.source = source;
  request.initial_guess_enabled = true;
  request.initial_guess = Eigen::Matrix4f::Identity();
  const auto result = plugin->align(request);
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
}

TEST(FastGicpRegistration, LegacyAndTypedGicpFixtureMatch)
{
  const Cloud::Ptr source = makeFixture();
  const Cloud::Ptr target = makeTarget(source);
  const DirectResult direct = runDirectFastGicp(source, target);
  lidarslam_default_plugins::FastGicpRegistration plugin;
  checkPlugin(
    &plugin, false, "lidarslam_default_plugins/FastGicp", direct);
}

TEST(FastGicpRegistration, LegacyAndTypedVgicpFixtureMatch)
{
  const Cloud::Ptr source = makeFixture();
  const Cloud::Ptr target = makeTarget(source);
  const DirectResult direct = runDirectFastVgicp(source, target);
  lidarslam_default_plugins::FastVgicpRegistration plugin;
  checkPlugin(
    &plugin, true, "lidarslam_default_plugins/FastVGicp", direct);
}

TEST(FastGicpRegistration, RejectsCrossVariantConfigurationKeys)
{
  lidarslam_default_plugins::FastGicpRegistration gicp;
  auto parameters = makeParameters(false);
  parameters.emplace("voxel_resolution", registration::ParameterValue(0.6));
  std::string error;
  EXPECT_FALSE(gicp.configure(parameters, &error));
  EXPECT_NE(error.find("unknown key"), std::string::npos);

  lidarslam_default_plugins::FastVgicpRegistration vgicp;
  parameters = makeParameters(true);
  parameters.erase("voxel_resolution");
  EXPECT_FALSE(vgicp.configure(parameters, &error));
  EXPECT_NE(error.find("voxel resolution"), std::string::npos);
}

}  // namespace
