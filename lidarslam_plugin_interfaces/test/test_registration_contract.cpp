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

#include <cstdint>
#include <stdexcept>

#include "lidarslam_plugin_interfaces/registration.hpp"

namespace registration = lidarslam::plugins::registration;

TEST(RegistrationApiVersion, RequiresSameMajorAndSupportedMinor)
{
  EXPECT_TRUE(registration::isApiCompatible({1, 2}, {1, 0}));
  EXPECT_TRUE(registration::isApiCompatible({1, 2}, {1, 2}));
  EXPECT_FALSE(registration::isApiCompatible({1, 2}, {1, 3}));
  EXPECT_FALSE(registration::isApiCompatible({1, 2}, {2, 0}));
}

TEST(RegistrationCapabilities, AreExplicitAndComposable)
{
  registration::Capabilities capabilities;
  capabilities.add(registration::Capability::kInitialGuess)
  .add(registration::Capability::kRotationPrior)
  .add(registration::Capability::kDeterministic)
  .setTargetPolicy(registration::TargetPolicy::kRequiresRawTarget)
  .setCorrespondenceMetric(registration::CorrespondenceMetric::kMeanDistance)
  .setThreadModel(registration::ThreadModel::kSerializedOwner);

  EXPECT_TRUE(capabilities.has(registration::Capability::kInitialGuess));
  EXPECT_TRUE(capabilities.has(registration::Capability::kRotationPrior));
  EXPECT_TRUE(capabilities.has(registration::Capability::kDeterministic));
  EXPECT_FALSE(capabilities.has(registration::Capability::kTranslationPrior));
  EXPECT_EQ(capabilities.targetPolicy(), registration::TargetPolicy::kRequiresRawTarget);
  EXPECT_EQ(
    capabilities.correspondenceMetric(), registration::CorrespondenceMetric::kMeanDistance);
  EXPECT_EQ(capabilities.threadModel(), registration::ThreadModel::kSerializedOwner);
}

TEST(RegistrationParameters, PreserveTypesAndStableMapOrdering)
{
  registration::ParameterMap parameters;
  parameters.emplace("threads", registration::ParameterValue(std::int64_t{4}));
  parameters.emplace("resolution", registration::ParameterValue(1.5));
  parameters.emplace("deterministic", registration::ParameterValue(true));
  parameters.emplace("mode", registration::ParameterValue("DIRECT7"));

  EXPECT_EQ(parameters.begin()->first, "deterministic");
  EXPECT_EQ(parameters.at("threads").asInteger(), 4);
  EXPECT_DOUBLE_EQ(parameters.at("resolution").asDouble(), 1.5);
  EXPECT_TRUE(parameters.at("deterministic").asBool());
  EXPECT_EQ(parameters.at("mode").asString(), "DIRECT7");
  EXPECT_THROW(parameters.at("mode").asDouble(), std::logic_error);
}

TEST(RegistrationRequest, RejectsMissingInputAndUnsupportedHints)
{
  registration::AlignmentRequest request;
  registration::Capabilities capabilities;

  EXPECT_EQ(
    registration::validateRequest(request, capabilities),
    registration::FailureCode::kInvalidInput);

  registration::PointCloud::Ptr source(new registration::PointCloud());
  source->push_back(registration::PointT{});
  request.source = source;
  request.initial_guess_enabled = false;
  request.rotation_prior.enabled = true;
  EXPECT_EQ(
    registration::validateRequest(request, capabilities),
    registration::FailureCode::kUnsupportedCapability);

  capabilities.add(registration::Capability::kRotationPrior);
  EXPECT_EQ(
    registration::validateRequest(request, capabilities),
    registration::FailureCode::kNone);

  request.translation_prior.enabled = true;
  EXPECT_EQ(
    registration::validateRequest(request, capabilities),
    registration::FailureCode::kUnsupportedCapability);

  request.translation_prior.enabled = false;
  request.maximum_correspondence_distance_enabled = true;
  EXPECT_EQ(
    registration::validateRequest(request, capabilities),
    registration::FailureCode::kUnsupportedCapability);

  request.maximum_correspondence_distance_enabled = false;
  request.initial_guess_enabled = true;
  EXPECT_EQ(
    registration::validateRequest(request, capabilities),
    registration::FailureCode::kUnsupportedCapability);

  capabilities.add(registration::Capability::kInitialGuess);
  EXPECT_EQ(
    registration::validateRequest(request, capabilities),
    registration::FailureCode::kNone);
}

TEST(RegistrationResult, DefaultsToAnInvalidNonConvergedResult)
{
  const registration::AlignmentResult result;
  EXPECT_FALSE(result.converged);
  EXPECT_EQ(result.failure, registration::FailureCode::kAlignmentFailed);
  EXPECT_TRUE(result.final_transformation.isIdentity());
  EXPECT_FALSE(result.aligned_source);
  EXPECT_FALSE(result.diagnostics.mean_correspondence_distance_valid);
  EXPECT_FALSE(result.diagnostics.covariance_valid);
}
