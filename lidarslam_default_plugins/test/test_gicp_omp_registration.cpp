// Copyright 2026 Sasaki
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>  // NOLINT(build/include_order)
#include <pcl/common/transforms.h>  // NOLINT(build/include_order)
#include <pclomp/gicp_omp.h>  // NOLINT(build/include_order)
#include <pclomp/gicp_omp_impl.hpp>  // NOLINT(build/include_order)

#include "lidarslam_default_plugins/gicp_omp_registration.hpp"

namespace
{

namespace registration = lidarslam::plugins::registration;
using PointT = registration::PointT;
using Cloud = registration::PointCloud;

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
  // Break the grid symmetries so covariance estimation and the recovered
  // transform are well-conditioned.
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

registration::ParameterMap makeParameters(
  const double maximum_correspondence_distance = 5.0,
  const bool adaptive_correspondence_threshold = false)
{
  registration::ParameterMap parameters;
  parameters.emplace(
    "maximum_correspondence_distance",
    registration::ParameterValue(maximum_correspondence_distance));
  parameters.emplace("transformation_epsilon", registration::ParameterValue(1e-8));
  parameters.emplace(
    "adaptive_correspondence_threshold",
    registration::ParameterValue(adaptive_correspondence_threshold));
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
  const Cloud::Ptr & source, const Cloud::Ptr & target,
  double maximum_correspondence_distance, bool initial_guess_enabled = true)
{
  pclomp::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
  gicp.setMaxCorrespondenceDistance(maximum_correspondence_distance);
  gicp.setTransformationEpsilon(1e-8);
  gicp.setInputSource(source);
  gicp.setInputTarget(target);
  DirectResult result;
  if (initial_guess_enabled) {
    gicp.align(result.aligned, Eigen::Matrix4f::Identity());
  } else {
    gicp.align(result.aligned);
  }
  result.transform = gicp.getFinalTransformation();
  result.fitness = gicp.getFitnessScore();
  result.converged = gicp.hasConverged();
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

TEST(GicpOmpRegistration, MetadataAndCapabilitiesMatchContract)
{
  lidarslam_default_plugins::GicpOmpRegistration plugin;
  const auto metadata = plugin.metadata();
  EXPECT_EQ(metadata.class_id, "lidarslam_default_plugins/GicpOmp");
  EXPECT_EQ(metadata.license, "BSD-2-Clause");
  EXPECT_TRUE(registration::isApiCompatible(registration::kHostApiVersion, metadata.api_version));

  const auto capabilities = plugin.capabilities();
  EXPECT_TRUE(capabilities.has(registration::Capability::kInitialGuess));
  EXPECT_TRUE(capabilities.has(registration::Capability::kMaximumCorrespondenceDistance));
  EXPECT_TRUE(capabilities.has(registration::Capability::kAlignedSource));
  EXPECT_FALSE(capabilities.has(registration::Capability::kMeanCorrespondenceDistance));
  EXPECT_EQ(capabilities.targetPolicy(), registration::TargetPolicy::kAcceptHostPrepared);
  EXPECT_EQ(
    capabilities.correspondenceMetric(),
    registration::CorrespondenceMetric::kSquareRootFitnessProxy);
  EXPECT_EQ(capabilities.threadModel(), registration::ThreadModel::kSerializedOwner);
}

TEST(GicpOmpRegistration, RejectsInvalidTypedConfiguration)
{
  const std::vector<std::pair<std::string, registration::ParameterValue>> invalid{
    {"maximum_correspondence_distance", registration::ParameterValue(0.0)},
    {"maximum_correspondence_distance", registration::ParameterValue(true)},
    {"transformation_epsilon", registration::ParameterValue(0.01)},
    {"adaptive_correspondence_threshold", registration::ParameterValue(1.0)},
    {"unknown", registration::ParameterValue(true)},
  };
  for (const auto & item : invalid) {
    lidarslam_default_plugins::GicpOmpRegistration plugin;
    auto parameters = makeParameters();
    parameters.erase(item.first);
    parameters.emplace(item.first, item.second);
    std::string error;
    EXPECT_FALSE(plugin.configure(parameters, &error)) << item.first;
    EXPECT_FALSE(error.empty()) << item.first;
  }
}

TEST(GicpOmpRegistration, MatchesDirectPclOmpFixture)
{
  const Cloud::Ptr source = makeFixture();
  const Cloud::Ptr target = makeTarget(source);
  const DirectResult direct = runDirect(source, target, 5.0);

  lidarslam_default_plugins::GicpOmpRegistration plugin;
  std::string error;
  ASSERT_TRUE(plugin.configure(makeParameters(), &error)) << error;
  ASSERT_TRUE(plugin.setInputTarget(target, &error)) << error;
  registration::AlignmentRequest request;
  request.source = source;
  request.initial_guess_enabled = true;
  request.initial_guess = Eigen::Matrix4f::Identity();
  const auto aligned = plugin.align(request);
  ASSERT_EQ(aligned.failure, registration::FailureCode::kNone) << aligned.diagnostics.detail;
  ASSERT_TRUE(aligned.aligned_source);
  EXPECT_EQ(aligned.converged, direct.converged);
  EXPECT_TRUE(bitwiseEqual(aligned.final_transformation, direct.transform));
  EXPECT_DOUBLE_EQ(aligned.fitness_score, direct.fitness);
  EXPECT_TRUE(bitwiseEqual(*aligned.aligned_source, direct.aligned));
  EXPECT_TRUE(aligned.diagnostics.mean_correspondence_distance_valid);
  EXPECT_DOUBLE_EQ(aligned.diagnostics.mean_correspondence_distance, std::sqrt(direct.fitness));
}

TEST(GicpOmpRegistration, AdaptiveResetMatchesLegacyFirstAndSecondCall)
{
  const Cloud::Ptr source = makeFixture();
  const Cloud::Ptr target = makeTarget(source);

  lidarslam_default_plugins::GicpOmpRegistration plugin;
  std::string error;
  ASSERT_TRUE(plugin.configure(makeParameters(5.0, true), &error)) << error;
  ASSERT_TRUE(plugin.setInputTarget(target, &error)) << error;

  registration::AlignmentRequest request;
  request.source = source;
  request.initial_guess_enabled = true;
  request.initial_guess = Eigen::Matrix4f::Identity();
  const auto first = plugin.align(request);
  ASSERT_EQ(first.failure, registration::FailureCode::kNone) << first.diagnostics.detail;
  const auto second = plugin.align(request);
  ASSERT_EQ(second.failure, registration::FailureCode::kNone) << second.diagnostics.detail;

  // The legacy scanmatcher sets DBL_MAX after each adaptive GICP call.  In
  // particular this happens on the first call when the EMA is still zero.
  const DirectResult direct_first = runDirect(source, target, 5.0);
  const DirectResult direct_second = runDirect(
    source, target, std::numeric_limits<double>::max());
  EXPECT_TRUE(bitwiseEqual(first.final_transformation, direct_first.transform));
  EXPECT_TRUE(bitwiseEqual(second.final_transformation, direct_second.transform));
  EXPECT_DOUBLE_EQ(first.fitness_score, direct_first.fitness);
  EXPECT_DOUBLE_EQ(second.fitness_score, direct_second.fitness);
}

TEST(GicpOmpRegistration, DisabledInitialGuessIgnoresNaNGuess)
{
  const Cloud::Ptr source = makeFixture();
  const Cloud::Ptr target = makeTarget(source);
  lidarslam_default_plugins::GicpOmpRegistration plugin;
  std::string error;
  ASSERT_TRUE(plugin.configure(makeParameters(), &error)) << error;
  ASSERT_TRUE(plugin.setInputTarget(target, &error)) << error;
  registration::AlignmentRequest request;
  request.source = source;
  request.initial_guess_enabled = false;
  request.initial_guess.setConstant(std::numeric_limits<float>::quiet_NaN());
  const auto result = plugin.align(request);
  EXPECT_EQ(result.failure, registration::FailureCode::kNone) << result.diagnostics.detail;
}

TEST(GicpOmpRegistration, ResetRemovesTargetAndAllowsFreshConfiguration)
{
  const Cloud::Ptr source = makeFixture();
  const Cloud::Ptr target = makeTarget(source);
  lidarslam_default_plugins::GicpOmpRegistration plugin;
  std::string error;
  ASSERT_TRUE(plugin.configure(makeParameters(), &error)) << error;
  ASSERT_TRUE(plugin.setInputTarget(target, &error)) << error;
  plugin.reset();
  registration::AlignmentRequest request;
  request.source = source;
  EXPECT_EQ(plugin.align(request).failure, registration::FailureCode::kNotConfigured);
  ASSERT_TRUE(plugin.configure(makeParameters(), &error)) << error;
  ASSERT_TRUE(plugin.setInputTarget(target, &error)) << error;
  EXPECT_EQ(plugin.align(request).failure, registration::FailureCode::kNone);
}

}  // namespace
