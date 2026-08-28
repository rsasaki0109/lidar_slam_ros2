// Copyright 2026 Sasaki
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
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

#include "scanmatcher/registration_runtime.hpp"

using lidarslam::plugins::registration::AlignmentResult;
using lidarslam::plugins::registration::CorrespondenceMetric;
using lidarslam::plugins::registration::TargetPolicy;

TEST(RegistrationRuntime, TargetPolicySelectsMapPreparation)
{
  EXPECT_TRUE(graphslam::registrationRuntimeUsesRawTarget(
    TargetPolicy::kRequiresRawTarget));
  EXPECT_FALSE(graphslam::registrationRuntimeUsesHostPreparedTarget(
    TargetPolicy::kRequiresRawTarget));
  EXPECT_TRUE(graphslam::registrationRuntimeUsesHostPreparedTarget(
    TargetPolicy::kAcceptHostPrepared));
  EXPECT_FALSE(graphslam::registrationRuntimeUsesRawTarget(
    TargetPolicy::kAcceptHostPrepared));
  EXPECT_FALSE(graphslam::registrationRuntimeUsesRawTarget(
    TargetPolicy::kPluginPreprocessesTarget));
  EXPECT_FALSE(graphslam::registrationRuntimeUsesHostPreparedTarget(
    TargetPolicy::kPluginPreprocessesTarget));
}

TEST(RegistrationRuntime, UsesAdvertisedMetricForAdaptiveState)
{
  AlignmentResult ndt_result;
  ndt_result.diagnostics.mean_correspondence_distance_valid = true;
  ndt_result.diagnostics.mean_correspondence_distance = 2.5;
  double metric = 0.0;
  ASSERT_TRUE(graphslam::registrationRuntimeMetricValue(
    ndt_result, CorrespondenceMetric::kMeanDistance, &metric));
  EXPECT_DOUBLE_EQ(metric, 2.5);

  AlignmentResult gicp_result;
  gicp_result.fitness_score = 9.0;
  ASSERT_TRUE(graphslam::registrationRuntimeMetricValue(
    gicp_result, CorrespondenceMetric::kSquareRootFitnessProxy, &metric));
  EXPECT_DOUBLE_EQ(metric, 3.0);
}

TEST(RegistrationRuntime, RejectsInvalidOrUnavailableMetric)
{
  AlignmentResult result;
  double metric = 0.0;
  EXPECT_FALSE(graphslam::registrationRuntimeMetricValue(
    result, CorrespondenceMetric::kMeanDistance, &metric));
  result.fitness_score = -1.0;
  EXPECT_FALSE(graphslam::registrationRuntimeMetricValue(
    result, CorrespondenceMetric::kSquareRootFitnessProxy, &metric));
  result.fitness_score = 1.0;
  EXPECT_FALSE(graphslam::registrationRuntimeMetricValue(
    result, CorrespondenceMetric::kUnavailable, &metric));
}
