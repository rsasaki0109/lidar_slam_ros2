// Copyright 2026 Sasaki
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
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
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "lidarslam_registration_loader/registration_plugin_loader.hpp"

namespace
{

namespace registration = lidarslam::plugins::registration;
namespace shell = lidarslam::plugins::registration::shell;

constexpr const char * kIdentity = "lidarslam_fake_registration_plugins/Identity";

shell::LoadRequest requestFor(const std::string & class_id)
{
  shell::LoadRequest request;
  request.class_id = class_id;
  return request;
}

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

class HostIdentityRegistration final : public registration::RegistrationPlugin
{
public:
  registration::PluginMetadata metadata() const override
  {
    registration::PluginMetadata metadata;
    metadata.class_id = "lidarslam_builtin/TestIdentity";
    metadata.implementation_version = "test";
    metadata.license = "BSD-2-Clause";
    metadata.api_version = registration::kHostApiVersion;
    return metadata;
  }

  registration::Capabilities capabilities() const override
  {
    registration::Capabilities capabilities;
    capabilities
    .add(registration::Capability::kInitialGuess)
    .add(registration::Capability::kAlignedSource)
    .add(registration::Capability::kDeterministic)
    .setTargetPolicy(registration::TargetPolicy::kRequiresRawTarget)
    .setCorrespondenceMetric(registration::CorrespondenceMetric::kMeanDistance)
    .setThreadModel(registration::ThreadModel::kSerializedOwner);
    return capabilities;
  }

  bool configure(
    const registration::ParameterMap & parameters, std::string * error) override
  {
    for (const auto & entry : parameters) {
      if (entry.first != "accept") {
        if (error != nullptr) {
          *error = "unknown host test parameter '" + entry.first + "'";
        }
        return false;
      }
      try {
        if (!entry.second.asBool()) {
          if (error != nullptr) {
            *error = "host test parameter 'accept' must be true";
          }
          return false;
        }
      } catch (const std::exception & exception) {
        if (error != nullptr) {
          *error = exception.what();
        }
        return false;
      }
    }
    configured_ = true;
    return true;
  }

  bool setInputTarget(
    const registration::PointCloudConstPtr & target, std::string * error) override
  {
    if (!configured_ || !target || target->empty()) {
      if (error != nullptr) {
        *error = "host test registration requires configured non-empty target";
      }
      return false;
    }
    target_ = target;
    return true;
  }

  registration::AlignmentResult align(
    const registration::AlignmentRequest & request) override
  {
    registration::AlignmentResult result;
    if (!configured_ || !target_) {
      result.failure = registration::FailureCode::kNotConfigured;
      return result;
    }
    result.failure = registration::validateRequest(request, capabilities());
    if (result.failure != registration::FailureCode::kNone) {
      return result;
    }
    result.final_transformation = request.initial_guess_enabled ?
      request.initial_guess : Eigen::Matrix4f::Identity();
    result.aligned_source.reset(new registration::PointCloud(*request.source));
    result.converged = true;
    result.failure = registration::FailureCode::kNone;
    return result;
  }

  void reset() noexcept override
  {
    configured_ = false;
    target_.reset();
  }

private:
  bool configured_{false};
  registration::PointCloudConstPtr target_;
};

shell::HostBuiltinRegistration hostIdentitySpec()
{
  shell::HostBuiltinRegistration registration;
  registration.class_id = "lidarslam_builtin/TestIdentity";
  registration.factory = []() {
      return std::make_shared<HostIdentityRegistration>();
    };
  return registration;
}

shell::LoadRequest hostIdentityRequest(bool accept)
{
  shell::LoadRequest request;
  request.class_id = "lidarslam_builtin/TestIdentity";
  request.parameters.emplace("accept", registration::ParameterValue(accept));
  request.capabilities.require_initial_guess = true;
  request.capabilities.require_aligned_source = true;
  request.capabilities.require_target_policy = true;
  request.capabilities.target_policy = registration::TargetPolicy::kRequiresRawTarget;
  return request;
}

}  // namespace

TEST(RegistrationPluginLoader, DiscoversInstalledFakeExternalClasses)
{
  shell::RegistrationPluginLoader loader;
  const std::vector<std::string> classes = loader.availableClasses();
  EXPECT_NE(std::find(classes.begin(), classes.end(), kIdentity), classes.end());
  EXPECT_NE(
    std::find(classes.begin(), classes.end(), "lidarslam_fake_registration_plugins/MissingLibrary"),
    classes.end());
}

TEST(RegistrationPluginLoader, DiscoversInstalledDefaultClasses)
{
  shell::RegistrationPluginLoader loader;
  const std::vector<std::string> classes = loader.availableClasses();
  EXPECT_NE(
    std::find(classes.begin(), classes.end(), "lidarslam_default_plugins/NdtOmp"), classes.end());
  EXPECT_NE(
    std::find(classes.begin(), classes.end(), "lidarslam_default_plugins/GicpOmp"), classes.end());
}

TEST(RegistrationPluginLoader, LoadsExternalPluginAndKeepsLoaderAlive)
{
  std::shared_ptr<shell::RegistrationPluginSession> session;
  {
    shell::RegistrationPluginLoader loader;
    shell::LoadRequest request = requestFor(kIdentity);
    request.parameters.emplace("accept", registration::ParameterValue(true));
    request.capabilities.require_initial_guess = true;
    request.capabilities.require_aligned_source = true;
    const shell::LoadResult loaded = loader.load(request);
    ASSERT_TRUE(loaded.ok()) << loaded.failure.message;
    ASSERT_NE(loaded.session, nullptr);
    EXPECT_EQ(loaded.session->metadata().class_id, kIdentity);
    EXPECT_EQ(loaded.session->metadata().license, "BSD-2-Clause");
    EXPECT_FALSE(loaded.session->libraryPath().empty());
    EXPECT_FALSE(loaded.session->pluginManifestPath().empty());
    session = loaded.session;
  }

  ASSERT_NE(session, nullptr);
  registration::PointCloud::Ptr cloud = makeCloud();
  std::string error;
  ASSERT_TRUE(session->plugin()->setInputTarget(cloud, &error)) << error;
  registration::AlignmentRequest request;
  request.source = cloud;
  request.initial_guess_enabled = false;
  // The no-guess path must not depend on the contents of initial_guess.
  const registration::AlignmentResult aligned = session->plugin()->align(request);
  EXPECT_EQ(aligned.failure, registration::FailureCode::kNone);
  EXPECT_TRUE(aligned.converged);
  ASSERT_NE(aligned.aligned_source, nullptr);
  EXPECT_EQ(aligned.aligned_source->size(), cloud->size());
}

TEST(RegistrationPluginLoader, RejectsUnknownClassWithoutFallback)
{
  shell::RegistrationPluginLoader loader;
  const shell::LoadResult loaded = loader.load(requestFor("registration/does_not_exist"));
  EXPECT_FALSE(loaded.ok());
  EXPECT_EQ(loaded.failure.code, shell::LoadFailureCode::kUnknownClass);
  EXPECT_NE(loaded.failure.message.find(kIdentity), std::string::npos);
  EXPECT_EQ(loaded.session, nullptr);
}

TEST(RegistrationPluginLoader, RejectsMissingLibraryWithActionableDiagnostic)
{
  shell::RegistrationPluginLoader loader;
  const shell::LoadResult loaded = loader.load(
    requestFor("lidarslam_fake_registration_plugins/MissingLibrary"));
  EXPECT_FALSE(loaded.ok());
  EXPECT_EQ(loaded.failure.code, shell::LoadFailureCode::kLibraryLoad);
  EXPECT_NE(loaded.failure.message.find("MissingLibrary"), std::string::npos);
}

TEST(RegistrationPluginLoader, RejectsApiMajorMismatch)
{
  shell::RegistrationPluginLoader loader;
  const shell::LoadResult loaded = loader.load(
    requestFor("lidarslam_fake_registration_plugins/BadApi"));
  EXPECT_FALSE(loaded.ok());
  EXPECT_EQ(loaded.failure.code, shell::LoadFailureCode::kApiMismatch);
  EXPECT_NE(loaded.failure.message.find("API 2.0"), std::string::npos);
  EXPECT_NE(loaded.failure.message.find("host API 1.0"), std::string::npos);
}

TEST(RegistrationPluginLoader, RejectsInvalidMetadataAndLicense)
{
  shell::RegistrationPluginLoader loader;
  const shell::LoadResult invalid_metadata = loader.load(
    requestFor("lidarslam_fake_registration_plugins/BadMetadata"));
  EXPECT_FALSE(invalid_metadata.ok());
  EXPECT_EQ(invalid_metadata.failure.code, shell::LoadFailureCode::kMetadataInvalid);
  EXPECT_NE(invalid_metadata.failure.message.find("metadata.class_id"), std::string::npos);

  const shell::LoadResult unlicensed = loader.load(
    requestFor("lidarslam_fake_registration_plugins/Unlicensed"));
  EXPECT_FALSE(unlicensed.ok());
  EXPECT_EQ(unlicensed.failure.code, shell::LoadFailureCode::kMetadataInvalid);
  EXPECT_NE(unlicensed.failure.message.find("GPL-3.0-only"), std::string::npos);
}

TEST(RegistrationPluginLoader, ValidatesConfiguredCapabilities)
{
  shell::RegistrationPluginLoader loader;
  shell::LoadRequest request = requestFor("lidarslam_fake_registration_plugins/NoGuess");
  request.capabilities.require_initial_guess = true;
  const shell::LoadResult loaded = loader.load(request);
  EXPECT_FALSE(loaded.ok());
  EXPECT_EQ(loaded.failure.code, shell::LoadFailureCode::kCapabilityMismatch);
  EXPECT_NE(loaded.failure.message.find("initial_guess"), std::string::npos);
}

TEST(RegistrationPluginLoader, ReportsPluginConfigurationFailure)
{
  shell::RegistrationPluginLoader loader;
  const shell::LoadResult loaded = loader.load(
    requestFor("lidarslam_fake_registration_plugins/Rejecting"));
  EXPECT_FALSE(loaded.ok());
  EXPECT_EQ(loaded.failure.code, shell::LoadFailureCode::kConfigurationFailure);
  EXPECT_NE(loaded.failure.message.find("rejects configuration"), std::string::npos);
}

TEST(RegistrationPluginLoader, ConfiguresDefaultNdtThroughInstalledPluginXml)
{
  shell::RegistrationPluginLoader loader;
  shell::LoadRequest request = requestFor("lidarslam_default_plugins/NdtOmp");
  request.parameters.emplace("resolution", registration::ParameterValue(2.0));
  request.parameters.emplace("transformation_epsilon", registration::ParameterValue(0.01));
  request.parameters.emplace(
    "maximum_iterations", registration::ParameterValue(std::int64_t{35}));
  request.parameters.emplace("step_size", registration::ParameterValue(0.1));
  request.parameters.emplace("outlier_ratio", registration::ParameterValue(0.55));
  request.parameters.emplace("num_threads", registration::ParameterValue(std::int64_t{1}));
  request.parameters.emplace(
    "neighborhood_search_method", registration::ParameterValue("DIRECT7"));
  request.capabilities.require_initial_guess = true;
  request.capabilities.require_deterministic = true;
  const shell::LoadResult loaded = loader.load(request);
  ASSERT_TRUE(loaded.ok()) << loaded.failure.message;
  EXPECT_TRUE(loaded.session->capabilities().has(registration::Capability::kDeterministic));
}

TEST(RegistrationPluginLoader, ConfiguresDefaultGicpThroughInstalledPluginXml)
{
  shell::RegistrationPluginLoader loader;
  const shell::LoadRequest request = [&]() {
      shell::LoadRequest value = requestFor("lidarslam_default_plugins/GicpOmp");
      value.parameters.emplace(
        "maximum_correspondence_distance", registration::ParameterValue(5.0));
      value.parameters.emplace("transformation_epsilon", registration::ParameterValue(1e-8));
      value.parameters.emplace(
        "adaptive_correspondence_threshold", registration::ParameterValue(false));
      value.capabilities.require_initial_guess = true;
      value.capabilities.require_aligned_source = true;
      value.capabilities.require_target_policy = true;
      value.capabilities.target_policy = registration::TargetPolicy::kAcceptHostPrepared;
      value.capabilities.require_correspondence_metric = true;
      value.capabilities.correspondence_metric =
        registration::CorrespondenceMetric::kSquareRootFitnessProxy;
      return value;
    }();
  const shell::LoadResult loaded = loader.load(request);
  ASSERT_TRUE(loaded.ok()) << loaded.failure.message;
  ASSERT_NE(loaded.session, nullptr);
  EXPECT_EQ(loaded.session->backendKind(), shell::BackendKind::kPluginlib);
  EXPECT_EQ(
    loaded.session->metadata().class_id, "lidarslam_default_plugins/GicpOmp");
  EXPECT_EQ(
    loaded.session->capabilities().targetPolicy(), registration::TargetPolicy::kAcceptHostPrepared);
  EXPECT_EQ(
    loaded.session->capabilities().correspondenceMetric(),
    registration::CorrespondenceMetric::kSquareRootFitnessProxy);
  EXPECT_FALSE(loaded.session->libraryPath().empty());
  EXPECT_FALSE(loaded.session->pluginManifestPath().empty());
}

TEST(RegistrationPluginLoader, DiscoversAndConfiguresOptionalSmallClassesOnlyWhenAdvertised)
{
  shell::RegistrationPluginLoader loader;
  const std::vector<std::string> classes = loader.availableClasses();
  const std::string gicp_class = "lidarslam_default_plugins/SmallGicpPcl";
  const std::string vgicp_class = "lidarslam_default_plugins/SmallVGicpPcl";
  const bool has_gicp =
    std::find(classes.begin(), classes.end(), gicp_class) != classes.end();
  const bool has_vgicp =
    std::find(classes.begin(), classes.end(), vgicp_class) != classes.end();

  // The optional pair is advertised atomically.  An installation without
  // small_gicp must not expose a class that cannot be created, and must not
  // silently route either selector to NDT/GICP.
  EXPECT_EQ(has_gicp, has_vgicp);
  if (!has_gicp || !has_vgicp) {
    return;
  }

  const auto make_request = [](const std::string & class_id, const bool voxelized) {
      shell::LoadRequest request = requestFor(class_id);
      request.parameters.emplace(
        "maximum_correspondence_distance", registration::ParameterValue(5.0));
      request.parameters.emplace(
        "transformation_epsilon", registration::ParameterValue(1e-6));
      request.parameters.emplace(
        "maximum_iterations", registration::ParameterValue(std::int64_t{35}));
      request.parameters.emplace("num_threads", registration::ParameterValue(std::int64_t{1}));
      request.parameters.emplace(
        "adaptive_correspondence_threshold", registration::ParameterValue(false));
      if (voxelized) {
        request.parameters.emplace("voxel_resolution", registration::ParameterValue(0.6));
      }
      request.capabilities.require_initial_guess = true;
      request.capabilities.require_aligned_source = true;
      request.capabilities.require_target_policy = true;
      request.capabilities.target_policy = registration::TargetPolicy::kAcceptHostPrepared;
      request.capabilities.require_correspondence_metric = true;
      request.capabilities.correspondence_metric =
        registration::CorrespondenceMetric::kSquareRootFitnessProxy;
      return request;
    };

  const shell::LoadResult gicp = loader.load(make_request(gicp_class, false));
  ASSERT_TRUE(gicp.ok()) << gicp.failure.message;
  ASSERT_NE(gicp.session, nullptr);
  EXPECT_EQ(gicp.session->metadata().class_id, gicp_class);
  EXPECT_EQ(gicp.session->backendKind(), shell::BackendKind::kPluginlib);
  EXPECT_TRUE(gicp.session->capabilities().has(registration::Capability::kInitialGuess));
  EXPECT_FALSE(gicp.session->libraryPath().empty());
  EXPECT_NE(
    gicp.session->libraryPath().find("liblidarslam_small_gicp_plugins"),
    std::string::npos);
  EXPECT_NE(
    gicp.session->pluginManifestPath().find("registration_plugins_small.xml"),
    std::string::npos);

  const shell::LoadResult vgicp = loader.load(make_request(vgicp_class, true));
  ASSERT_TRUE(vgicp.ok()) << vgicp.failure.message;
  ASSERT_NE(vgicp.session, nullptr);
  EXPECT_EQ(vgicp.session->metadata().class_id, vgicp_class);
  EXPECT_EQ(vgicp.session->backendKind(), shell::BackendKind::kPluginlib);
  EXPECT_TRUE(vgicp.session->capabilities().has(registration::Capability::kInitialGuess));
  EXPECT_FALSE(vgicp.session->libraryPath().empty());
  EXPECT_NE(
    vgicp.session->libraryPath().find("liblidarslam_small_gicp_plugins"),
    std::string::npos);
  EXPECT_NE(
    vgicp.session->pluginManifestPath().find("registration_plugins_small.xml"),
    std::string::npos);
}

TEST(RegistrationResolver, ResolvesHostBuiltinAndRetainsSessionAfterResolverDestruction)
{
  std::shared_ptr<shell::RegistrationPluginSession> session;
  {
    shell::RegistrationResolver resolver({hostIdentitySpec()});
    const shell::LoadResult loaded = resolver.resolve(hostIdentityRequest(true));
    ASSERT_TRUE(loaded.ok()) << loaded.failure.message;
    ASSERT_NE(loaded.session, nullptr);
    EXPECT_EQ(loaded.session->backendKind(), shell::BackendKind::kHostBuiltIn);
    EXPECT_EQ(loaded.session->metadata().class_id, "lidarslam_builtin/TestIdentity");
    EXPECT_TRUE(loaded.session->libraryPath().empty());
    EXPECT_TRUE(loaded.session->pluginManifestPath().empty());
    EXPECT_EQ(loaded.session->parameters().at("accept").asBool(), true);
    session = loaded.session;
  }

  const registration::PointCloud::Ptr cloud = makeCloud();
  std::string error;
  ASSERT_TRUE(session->plugin()->setInputTarget(cloud, &error)) << error;
  registration::AlignmentRequest request;
  request.source = cloud;
  request.initial_guess_enabled = false;
  const registration::AlignmentResult aligned = session->plugin()->align(request);
  EXPECT_EQ(aligned.failure, registration::FailureCode::kNone);
  EXPECT_TRUE(aligned.converged);
}

TEST(RegistrationResolver, ResolvesInstalledExternalThroughPluginlib)
{
  std::shared_ptr<shell::RegistrationPluginSession> session;
  {
    shell::RegistrationResolver resolver({hostIdentitySpec()});
    shell::LoadRequest request = requestFor(kIdentity);
    request.parameters.emplace("accept", registration::ParameterValue(true));
    request.capabilities.require_initial_guess = true;
    request.capabilities.require_aligned_source = true;
    const shell::LoadResult loaded = resolver.resolve(request);
    ASSERT_TRUE(loaded.ok()) << loaded.failure.message;
    ASSERT_NE(loaded.session, nullptr);
    EXPECT_EQ(loaded.session->backendKind(), shell::BackendKind::kPluginlib);
    EXPECT_FALSE(loaded.session->libraryPath().empty());
    EXPECT_FALSE(loaded.session->pluginManifestPath().empty());
    session = loaded.session;
  }

  const registration::PointCloud::Ptr cloud = makeCloud();
  std::string error;
  ASSERT_TRUE(session->plugin()->setInputTarget(cloud, &error)) << error;
  registration::AlignmentRequest request;
  request.source = cloud;
  request.initial_guess_enabled = false;
  const registration::AlignmentResult aligned = session->plugin()->align(request);
  EXPECT_EQ(aligned.failure, registration::FailureCode::kNone);
  EXPECT_TRUE(aligned.converged);
}

TEST(RegistrationResolver, RejectsHostConfigurationWithoutFallback)
{
  shell::RegistrationResolver resolver({hostIdentitySpec()});
  const shell::LoadResult loaded = resolver.resolve(hostIdentityRequest(false));
  EXPECT_FALSE(loaded.ok());
  EXPECT_EQ(loaded.failure.code, shell::LoadFailureCode::kConfigurationFailure);
  EXPECT_NE(loaded.failure.message.find("must be true"), std::string::npos);
  EXPECT_EQ(loaded.session, nullptr);
}

TEST(RegistrationResolver, RejectsDuplicateHostClassIds)
{
  shell::RegistrationResolver resolver({hostIdentitySpec(), hostIdentitySpec()});
  EXPECT_NE(resolver.initializationError().find("duplicate"), std::string::npos);
  const shell::LoadResult loaded = resolver.resolve(hostIdentityRequest(true));
  EXPECT_FALSE(loaded.ok());
  EXPECT_EQ(loaded.failure.code, shell::LoadFailureCode::kClassCollision);
}

TEST(RegistrationPluginLoader, RejectsReservedHostNamespace)
{
  shell::RegistrationPluginLoader loader;
  shell::LoadRequest request = requestFor("lidarslam_builtin/ExternalShadow");
  const shell::LoadResult loaded = loader.load(request);
  EXPECT_FALSE(loaded.ok());
  EXPECT_EQ(loaded.failure.code, shell::LoadFailureCode::kNamespaceViolation);
  EXPECT_NE(loaded.failure.message.find("cannot be resolved through pluginlib"), std::string::npos);
}
