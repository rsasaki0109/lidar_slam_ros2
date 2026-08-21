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

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <lidarslam_registration_loader/registration_plugin_loader.hpp>

namespace registration = lidarslam::plugins::registration;
namespace shell = lidarslam::plugins::registration::shell;
namespace
{

registration::PointCloud::Ptr makeCloud()
{
  registration::PointCloud::Ptr cloud(new registration::PointCloud());
  registration::PointT point;
  point.x = 1.0F;
  point.y = 2.0F;
  point.z = 3.0F;
  point.intensity = 4.0F;
  cloud->push_back(point);
  return cloud;
}

shell::LoadRequest makeLoadRequest()
{
  shell::LoadRequest request;
  request.class_id = "lidarslam_registration_plugin_template/Identity";
  request.parameters.emplace("mode", registration::ParameterValue("identity"));
  request.capabilities.require_initial_guess = true;
  request.capabilities.require_aligned_source = true;
  request.capabilities.require_deterministic = true;
  request.capabilities.require_target_policy = true;
  request.capabilities.target_policy = registration::TargetPolicy::kAcceptHostPrepared;
  request.capabilities.require_correspondence_metric = true;
  request.capabilities.correspondence_metric = registration::CorrespondenceMetric::kMeanDistance;
  return request;
}

}  // namespace

TEST(RegistrationPluginTemplateLoader, DiscoversConfiguresAndKeepsSessionAlive)
{
  std::shared_ptr<shell::RegistrationPluginSession> session;
  const shell::LoadRequest request = makeLoadRequest();
  {
    shell::RegistrationPluginLoader loader;
    ASSERT_TRUE(loader.initializationError().empty()) << loader.initializationError();
    const std::vector<std::string> classes = loader.availableClasses();
    ASSERT_NE(
      std::find(classes.begin(), classes.end(), request.class_id), classes.end());

    shell::LoadRequest mismatch = request;
    mismatch.capabilities.require_rotation_prior = true;
    const shell::LoadResult mismatch_result = loader.load(mismatch);
    ASSERT_FALSE(mismatch_result.ok());
    EXPECT_EQ(
      mismatch_result.failure.code, shell::LoadFailureCode::kCapabilityMismatch);

    const shell::LoadResult loaded = loader.load(request);
    ASSERT_TRUE(loaded.ok()) << loaded.failure.message;
    session = loaded.session;
  }

  ASSERT_TRUE(session);
  EXPECT_EQ(session->backendKind(), shell::BackendKind::kPluginlib);
  EXPECT_EQ(session->classId(), request.class_id);
  EXPECT_EQ(session->metadata().class_id, request.class_id);
  EXPECT_EQ(session->metadata().license, "BSD-2-Clause");
  EXPECT_FALSE(session->libraryPath().empty());
  EXPECT_FALSE(session->pluginManifestPath().empty());

  std::string error;
  ASSERT_TRUE(session->plugin()->setInputTarget(makeCloud(), &error)) << error;
  registration::AlignmentRequest alignment_request;
  alignment_request.source = makeCloud();
  const registration::AlignmentResult result = session->plugin()->align(alignment_request);
  EXPECT_TRUE(result.converged);
  EXPECT_EQ(result.failure, registration::FailureCode::kNone);
  ASSERT_TRUE(result.aligned_source);
  EXPECT_EQ(result.aligned_source->size(), alignment_request.source->size());

  session->plugin()->reset();
  const registration::AlignmentResult after_reset =
    session->plugin()->align(alignment_request);
  EXPECT_EQ(after_reset.failure, registration::FailureCode::kNotConfigured);
}
