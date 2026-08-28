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
#include <limits>
#include <memory>
#include <string>

#include <pcl/common/transforms.h>  // NOLINT(build/include_order)

#include "lidarslam_registration_plugin_template/template_registration.hpp"

namespace registration = lidarslam::plugins::registration;
namespace
{

registration::PointCloud::Ptr makeCloud()
{
  registration::PointCloud::Ptr cloud(new registration::PointCloud());
  registration::PointT first;
  first.x = 1.0F;
  first.y = 2.0F;
  first.z = 3.0F;
  first.intensity = 4.0F;
  cloud->push_back(first);
  registration::PointT second;
  second.x = -1.0F;
  second.y = 0.5F;
  second.z = 8.0F;
  second.intensity = 2.0F;
  cloud->push_back(second);
  return cloud;
}

void expectCloudEqual(
  const registration::PointCloud & expected, const registration::PointCloud & actual)
{
  ASSERT_EQ(expected.size(), actual.size());
  for (std::size_t index = 0; index < expected.size(); ++index) {
    EXPECT_FLOAT_EQ(expected[index].x, actual[index].x);
    EXPECT_FLOAT_EQ(expected[index].y, actual[index].y);
    EXPECT_FLOAT_EQ(expected[index].z, actual[index].z);
    EXPECT_FLOAT_EQ(expected[index].intensity, actual[index].intensity);
  }
}

}  // namespace

TEST(RegistrationPluginTemplate, ReportsMetadataAndCapabilities)
{
  lidarslam_registration_plugin_template::IdentityRegistration plugin;
  const registration::PluginMetadata metadata = plugin.metadata();
  EXPECT_EQ(metadata.class_id, "lidarslam_registration_plugin_template/Identity");
  EXPECT_EQ(metadata.implementation_version, "0.1.0");
  EXPECT_EQ(metadata.license, "BSD-2-Clause");
  EXPECT_EQ(metadata.api_version.major, registration::kHostApiVersion.major);
  EXPECT_EQ(metadata.api_version.minor, registration::kHostApiVersion.minor);

  const registration::Capabilities capabilities = plugin.capabilities();
  EXPECT_TRUE(capabilities.has(registration::Capability::kInitialGuess));
  EXPECT_TRUE(capabilities.has(registration::Capability::kAlignedSource));
  EXPECT_TRUE(capabilities.has(registration::Capability::kDeterministic));
  EXPECT_EQ(
    capabilities.targetPolicy(), registration::TargetPolicy::kAcceptHostPrepared);
  EXPECT_EQ(
    capabilities.correspondenceMetric(), registration::CorrespondenceMetric::kMeanDistance);
  EXPECT_EQ(capabilities.threadModel(), registration::ThreadModel::kSerializedOwner);
}

TEST(RegistrationPluginTemplate, ValidatesTypedConfiguration)
{
  lidarslam_registration_plugin_template::IdentityRegistration plugin;
  std::string error;

  registration::ParameterMap wrong_type;
  wrong_type.emplace("mode", registration::ParameterValue(true));
  EXPECT_FALSE(plugin.configure(wrong_type, &error));
  EXPECT_FALSE(error.empty());

  registration::ParameterMap unknown;
  unknown.emplace("unknown", registration::ParameterValue("value"));
  error.clear();
  EXPECT_FALSE(plugin.configure(unknown, &error));
  EXPECT_FALSE(error.empty());

  registration::ParameterMap wrong_value;
  wrong_value.emplace("mode", registration::ParameterValue("not_identity"));
  error.clear();
  EXPECT_FALSE(plugin.configure(wrong_value, &error));
  EXPECT_FALSE(error.empty());

  registration::ParameterMap valid;
  valid.emplace("mode", registration::ParameterValue("identity"));
  error.clear();
  EXPECT_TRUE(plugin.configure(valid, &error)) << error;
}

TEST(RegistrationPluginTemplate, AlignsAndHonorsDisabledInitialGuess)
{
  lidarslam_registration_plugin_template::IdentityRegistration plugin;
  ASSERT_TRUE(plugin.configure({}, nullptr));
  const registration::PointCloud::Ptr target = makeCloud();
  std::string error;
  ASSERT_TRUE(plugin.setInputTarget(target, &error)) << error;

  registration::AlignmentRequest request;
  request.source = makeCloud();
  request.initial_guess_enabled = true;
  request.initial_guess = Eigen::Matrix4f::Identity();
  request.initial_guess(0, 3) = 3.0F;
  request.initial_guess(1, 3) = -2.0F;
  const registration::AlignmentResult guessed = plugin.align(request);
  ASSERT_EQ(guessed.failure, registration::FailureCode::kNone);
  ASSERT_TRUE(guessed.converged);
  EXPECT_TRUE(guessed.final_transformation.isApprox(request.initial_guess, 0.0F));
  ASSERT_TRUE(guessed.aligned_source);
  registration::PointCloud expected_aligned;
  pcl::transformPointCloud(*request.source, expected_aligned, request.initial_guess);
  expectCloudEqual(expected_aligned, *guessed.aligned_source);
  EXPECT_TRUE(guessed.diagnostics.mean_correspondence_distance_valid);

  request.initial_guess_enabled = false;
  request.initial_guess.setConstant(std::numeric_limits<float>::quiet_NaN());
  const registration::AlignmentResult without_guess = plugin.align(request);
  ASSERT_EQ(without_guess.failure, registration::FailureCode::kNone);
  ASSERT_TRUE(without_guess.converged);
  EXPECT_TRUE(without_guess.final_transformation.isIdentity(0.0F));
  ASSERT_TRUE(without_guess.aligned_source);
  expectCloudEqual(*request.source, *without_guess.aligned_source);
}

TEST(RegistrationPluginTemplate, RejectsInvalidInputAndResetClearsState)
{
  lidarslam_registration_plugin_template::IdentityRegistration plugin;
  ASSERT_TRUE(plugin.configure({}, nullptr));
  const registration::PointCloud::Ptr target = makeCloud();
  ASSERT_TRUE(plugin.setInputTarget(target, nullptr));

  registration::AlignmentRequest request;
  const registration::AlignmentResult missing_source = plugin.align(request);
  EXPECT_EQ(missing_source.failure, registration::FailureCode::kInvalidInput);

  registration::PointCloud::Ptr non_finite_cloud = makeCloud();
  (*non_finite_cloud)[0].x = std::numeric_limits<float>::quiet_NaN();
  request.source = non_finite_cloud;
  const registration::AlignmentResult non_finite_result = plugin.align(request);
  EXPECT_EQ(non_finite_result.failure, registration::FailureCode::kInvalidInput);

  plugin.reset();
  const registration::AlignmentResult after_reset = plugin.align(request);
  EXPECT_EQ(after_reset.failure, registration::FailureCode::kNotConfigured);
  EXPECT_FALSE(after_reset.aligned_source);
}

TEST(RegistrationPluginTemplate, RejectsNonFiniteTarget)
{
  lidarslam_registration_plugin_template::IdentityRegistration plugin;
  ASSERT_TRUE(plugin.configure({}, nullptr));
  const registration::PointCloud::Ptr target = makeCloud();
  (*target)[0].intensity = std::numeric_limits<float>::infinity();
  std::string error;
  EXPECT_FALSE(plugin.setInputTarget(target, &error));
  EXPECT_FALSE(error.empty());
}
