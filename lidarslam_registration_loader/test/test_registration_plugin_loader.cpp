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

#include <unistd.h>

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
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

class HostIdentityRegistration final
  : public registration::RegistrationPlugin,
  public registration::RegistrationPluginDescriptorProvider
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

  registration::RegistrationRuntimeDescriptor registrationDescriptor() const override
  {
    const auto current = capabilities();
    return registration::makeRegistrationRuntimeDescriptor(
      metadata(), current,
      static_cast<std::uint64_t>(registration::Capability::kInitialGuess) |
      static_cast<std::uint64_t>(registration::Capability::kAlignedSource),
      static_cast<std::uint64_t>(registration::Capability::kDeterministic),
      registration::registrationConfigSchemaForClassId(metadata().class_id));
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
  registration.expected_descriptor_factory = []() {
      HostIdentityRegistration plugin;
      return plugin.registrationDescriptor();
    };
  return registration;
}

class ThrowingProcessingRegistration final
  : public registration::RegistrationPlugin,
  public registration::RegistrationPluginDescriptorProvider
{
public:
  registration::PluginMetadata metadata() const override
  {
    registration::PluginMetadata metadata;
    metadata.class_id = "lidarslam_builtin/ThrowingProcessing";
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

  registration::RegistrationRuntimeDescriptor registrationDescriptor() const override
  {
    const auto current = capabilities();
    return registration::makeRegistrationRuntimeDescriptor(
      metadata(), current,
      static_cast<std::uint64_t>(registration::Capability::kInitialGuess) |
      static_cast<std::uint64_t>(registration::Capability::kAlignedSource),
      static_cast<std::uint64_t>(registration::Capability::kDeterministic),
      registration::registrationConfigSchemaForClassId(metadata().class_id));
  }

  bool configure(
    const registration::ParameterMap &, std::string *) override
  {
    return true;
  }

  bool setInputTarget(
    const registration::PointCloudConstPtr &, std::string *) override
  {
    throw std::runtime_error("synthetic target exception");
  }

  registration::AlignmentResult align(
    const registration::AlignmentRequest &) override
  {
    throw std::runtime_error("synthetic alignment exception");
  }

  void reset() noexcept override {}
};

shell::HostBuiltinRegistration throwingProcessingSpec()
{
  shell::HostBuiltinRegistration registration;
  registration.class_id = "lidarslam_builtin/ThrowingProcessing";
  registration.factory = []() {
      return std::make_shared<ThrowingProcessingRegistration>();
    };
  registration.expected_descriptor_factory = []() {
      ThrowingProcessingRegistration plugin;
      return plugin.registrationDescriptor();
    };
  return registration;
}

shell::LoadRequest throwingProcessingRequest()
{
  shell::LoadRequest request;
  request.class_id = "lidarslam_builtin/ThrowingProcessing";
  request.capabilities.require_initial_guess = true;
  request.capabilities.require_aligned_source = true;
  return request;
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

shell::RegistrationContractManifest makeSyntheticContractManifest(
  const std::filesystem::path & root, const std::string & class_id)
{
  const auto xml = root / "registration_plugins.xml";
  const auto dso = root / "libregistration.so";
  std::ofstream(xml) << "<library path=\"registration\"/>\n";
  std::ofstream(dso, std::ios::binary) << "synthetic dso bytes\n";
  shell::RegistrationContractManifest manifest;
  manifest.schema = "lidarslam-registration-contract-manifest-v1";
  manifest.schema_version = 1U;
  manifest.class_id = class_id;
  manifest.plugin_xml_sha256 = shell::registrationContractFileSha256(xml.string());
  manifest.plugin_xml_size_bytes = std::filesystem::file_size(xml);
  manifest.dso_sha256 = shell::registrationContractFileSha256(dso.string());
  manifest.dso_size_bytes = std::filesystem::file_size(dso);
  manifest.abi_epoch = registration::kRegistrationAbiEpoch;
  manifest.toolchain_tag = registration::registrationToolchainTag();
  manifest.interface_contract_sha256 = registration::kRegistrationInterfaceContractSha256;
  manifest.api_min = registration::kHostApiVersion;
  manifest.api_max = registration::kHostApiVersion;
  manifest.required_capability_bits =
    static_cast<std::uint64_t>(registration::Capability::kInitialGuess) |
    static_cast<std::uint64_t>(registration::Capability::kAlignedSource);
  manifest.optional_capability_bits =
    static_cast<std::uint64_t>(registration::Capability::kDeterministic);
  manifest.target_policy = registration::TargetPolicy::kRequiresRawTarget;
  manifest.correspondence_metric = registration::CorrespondenceMetric::kMeanDistance;
  manifest.thread_model = registration::ThreadModel::kSerializedOwner;
  manifest.cancellation_model = registration::CancellationModel::kNonInterruptibleAlign;
  manifest.config_schema_id = "lidarslam.registration.synthetic.v1";
  manifest.config_schema_version = 1U;
  manifest.config_schema_sha256 =
    "4d2d0f7f87d168fa5a04b22d1f3fbfc468c3576ae1e4f529dc8b6a7fca8a7c30";
  manifest.manifest_sha256 = shell::registrationContractManifestDigest(manifest);
  return manifest;
}

void writeSyntheticContractSidecar(
  const std::filesystem::path & manifest_path,
  const shell::RegistrationContractManifest & manifest)
{
  const auto sidecar = shell::registrationContractSidecarPath(
    manifest_path.string(), manifest.class_id);
  std::ofstream output(sidecar);
  output << "{\n"
         << "  \"schema\": \"" << manifest.schema << "\",\n"
         << "  \"schema_version\": " << manifest.schema_version << ",\n"
         << "  \"class_id\": \"" << manifest.class_id << "\",\n"
         << "  \"plugin_xml_sha256\": \"" << manifest.plugin_xml_sha256 << "\",\n"
         << "  \"plugin_xml_size_bytes\": " << manifest.plugin_xml_size_bytes << ",\n"
         << "  \"dso_sha256\": \"" << manifest.dso_sha256 << "\",\n"
         << "  \"dso_size_bytes\": " << manifest.dso_size_bytes << ",\n"
         << "  \"abi_epoch\": \"" << manifest.abi_epoch << "\",\n"
         << "  \"toolchain_tag\": \"" << manifest.toolchain_tag << "\",\n"
         << "  \"interface_contract_sha256\": \"" << manifest.interface_contract_sha256 << "\",\n"
         << "  \"api_min_major\": " << manifest.api_min.major << ",\n"
         << "  \"api_min_minor\": " << manifest.api_min.minor << ",\n"
         << "  \"api_max_major\": " << manifest.api_max.major << ",\n"
         << "  \"api_max_minor\": " << manifest.api_max.minor << ",\n"
         << "  \"required_capability_bits\": " << manifest.required_capability_bits << ",\n"
         << "  \"optional_capability_bits\": " << manifest.optional_capability_bits << ",\n"
         << "  \"target_policy\": " << static_cast<int>(manifest.target_policy) << ",\n"
         << "  \"correspondence_metric\": " << static_cast<int>(manifest.correspondence_metric) <<
    ",\n"
         << "  \"thread_model\": " << static_cast<int>(manifest.thread_model) << ",\n"
         << "  \"cancellation_model\": " << static_cast<int>(manifest.cancellation_model) << ",\n"
         << "  \"config_schema_id\": \"" << manifest.config_schema_id << "\",\n"
         << "  \"config_schema_version\": " << manifest.config_schema_version << ",\n"
         << "  \"config_schema_sha256\": \"" << manifest.config_schema_sha256 << "\",\n"
         << "  \"manifest_sha256\": \"" << manifest.manifest_sha256 << "\"\n}\n";
}

}  // namespace

TEST(RegistrationContractManifest, AcceptsExactSyntheticIdentity)
{
  const auto root = std::filesystem::temp_directory_path() /
    ("lidarslam_registration_contract_" +
    std::to_string(static_cast<std::int64_t>(getpid())));
  std::error_code error;
  std::filesystem::remove_all(root, error);
  std::filesystem::create_directories(root, error);
  ASSERT_FALSE(error) << error.message();
  const std::string class_id = "synthetic/Identity";
  const auto manifest = makeSyntheticContractManifest(root, class_id);
  const auto xml = root / "registration_plugins.xml";
  const auto dso = root / "libregistration.so";
  writeSyntheticContractSidecar(xml, manifest);
  shell::RegistrationContractManifest parsed;
  const auto result = shell::readAndValidateRegistrationContractManifest(
    class_id, dso.string(), xml.string(), &parsed);
  EXPECT_TRUE(result.ok()) << result.message;
  EXPECT_EQ(parsed.manifest_sha256, manifest.manifest_sha256);
  std::filesystem::remove_all(root, error);
}

TEST(RegistrationContractManifest, RejectsMissingAndByteDrift)
{
  const auto root = std::filesystem::temp_directory_path() /
    ("lidarslam_registration_contract_drift_" +
    std::to_string(static_cast<std::int64_t>(getpid())));
  std::error_code error;
  std::filesystem::remove_all(root, error);
  std::filesystem::create_directories(root, error);
  ASSERT_FALSE(error) << error.message();
  const std::string class_id = "synthetic/Identity";
  const auto manifest = makeSyntheticContractManifest(root, class_id);
  const auto xml = root / "registration_plugins.xml";
  const auto dso = root / "libregistration.so";
  shell::RegistrationContractManifest parsed;
  auto result = shell::readAndValidateRegistrationContractManifest(
    class_id, dso.string(), xml.string(), &parsed);
  EXPECT_EQ(result.code, shell::LoadFailureCode::kContractManifestMissing);
  writeSyntheticContractSidecar(xml, manifest);
  std::ofstream(dso, std::ios::app) << "drift\n";
  result = shell::readAndValidateRegistrationContractManifest(
    class_id, dso.string(), xml.string(), &parsed);
  EXPECT_EQ(result.code, shell::LoadFailureCode::kContractManifestInvalid);
  std::filesystem::remove_all(root, error);
}

TEST(RegistrationContractManifest, RejectsRuntimeDescriptorDrift)
{
  shell::RegistrationContractManifest manifest;
  manifest.schema = "lidarslam-registration-contract-manifest-v1";
  manifest.schema_version = 1U;
  manifest.class_id = "synthetic/Identity";
  manifest.api_min = registration::kHostApiVersion;
  manifest.api_max = registration::kHostApiVersion;
  manifest.required_capability_bits = 129U;
  manifest.optional_capability_bits = 64U;
  manifest.target_policy = registration::TargetPolicy::kRequiresRawTarget;
  manifest.correspondence_metric = registration::CorrespondenceMetric::kMeanDistance;
  manifest.thread_model = registration::ThreadModel::kSerializedOwner;
  manifest.cancellation_model = registration::CancellationModel::kNonInterruptibleAlign;
  manifest.abi_epoch = registration::kRegistrationAbiEpoch;
  manifest.toolchain_tag = registration::registrationToolchainTag();
  manifest.config_schema_id = "lidarslam.registration.synthetic.v1";
  manifest.config_schema_version = 1U;
  manifest.config_schema_sha256 =
    "4d2d0f7f87d168fa5a04b22d1f3fbfc468c3576ae1e4f529dc8b6a7fca8a7c30";
  manifest.interface_contract_sha256 = registration::kRegistrationInterfaceContractSha256;
  registration::RegistrationRuntimeDescriptor descriptor;
  descriptor.schema = registration::kRegistrationDescriptorSchema;
  descriptor.schema_version = registration::kRegistrationDescriptorSchemaVersion;
  descriptor.class_id = manifest.class_id;
  descriptor.api_min = manifest.api_min;
  descriptor.api_max = manifest.api_max;
  descriptor.required_capability_bits = manifest.required_capability_bits;
  descriptor.optional_capability_bits = manifest.optional_capability_bits;
  descriptor.target_policy = manifest.target_policy;
  descriptor.correspondence_metric = manifest.correspondence_metric;
  descriptor.thread_model = manifest.thread_model;
  descriptor.cancellation_model = manifest.cancellation_model;
  descriptor.abi_epoch = manifest.abi_epoch;
  descriptor.toolchain_tag = manifest.toolchain_tag;
  descriptor.config_schema_id = manifest.config_schema_id;
  descriptor.config_schema_version = manifest.config_schema_version;
  descriptor.config_schema_sha256 = manifest.config_schema_sha256;
  descriptor.interface_contract_sha256 = manifest.interface_contract_sha256;
  EXPECT_TRUE(shell::validateRegistrationRuntimeDescriptor(
    manifest.class_id, descriptor, &manifest, nullptr).ok());
  descriptor.toolchain_tag += ".drift";
  const auto result = shell::validateRegistrationRuntimeDescriptor(
    manifest.class_id, descriptor, &manifest, nullptr);
  EXPECT_EQ(result.code, shell::LoadFailureCode::kDescriptorMismatch);
}

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

TEST(RegistrationPluginLoader, ExternalSessionPublishesProvenanceAndRetainsLoaderLease)
{
  std::shared_ptr<shell::RegistrationPluginSession> session;
  {
    shell::RegistrationPluginLoader loader;
    shell::LoadRequest request = requestFor(kIdentity);
    request.parameters.emplace("accept", registration::ParameterValue(true));
    const shell::LoadResult loaded = loader.load(request);
    ASSERT_TRUE(loaded.ok()) << loaded.failure.message;
    session = loaded.session;
  }

  ASSERT_NE(session, nullptr);
  EXPECT_TRUE(session->hasExternalLoaderLease());
  ASSERT_TRUE(std::filesystem::path(session->libraryPath()).is_absolute());
  ASSERT_TRUE(std::filesystem::path(session->pluginManifestPath()).is_absolute());
  EXPECT_TRUE(std::filesystem::is_regular_file(session->libraryPath()));
  EXPECT_TRUE(std::filesystem::is_regular_file(session->pluginManifestPath()));
  EXPECT_FALSE(std::filesystem::is_symlink(session->libraryPath()));
  // colcon --symlink-install commonly represents package XML resources as
  // symlinks; the loader validates their resolved target instead.
  EXPECT_TRUE(shell::validateExternalDsoProvenance(
    session->classId(), session->libraryPath(), session->pluginManifestPath()).ok());
}

TEST(RegistrationPluginLoader, ActivationRejectKeepsPreviousExternalSessionAndLease)
{
  shell::RegistrationPluginLoader loader;
  shell::LoadRequest request = requestFor(kIdentity);
  request.parameters.emplace("accept", registration::ParameterValue(true));
  const shell::LoadResult previous = loader.load(request);
  shell::LoadResult candidate = loader.load(request);
  ASSERT_TRUE(previous.ok()) << previous.failure.message;
  ASSERT_TRUE(candidate.ok()) << candidate.failure.message;

  std::shared_ptr<shell::RegistrationPluginSession> active_session = previous.session;
  std::shared_ptr<registration::RegistrationPlugin> active_plugin = previous.session->plugin();
  registration::Capabilities active_capabilities = previous.session->capabilities();
  registration::TargetPolicy active_policy = active_capabilities.targetPolicy();
  registration::CorrespondenceMetric active_metric = active_capabilities.correspondenceMetric();
  const std::weak_ptr<shell::RegistrationPluginSession> candidate_weak = candidate.session;

  shell::RegistrationActivationSlots slots;
  slots.session = &active_session;
  slots.plugin = &active_plugin;
  slots.capabilities = &active_capabilities;
  slots.target_policy = &active_policy;
  slots.correspondence_metric = &active_metric;
  {
    shell::RegistrationActivationTransaction transaction(slots);
    shell::LoadFailure failure;
    ASSERT_TRUE(transaction.prepare(candidate.session, &failure)) << failure.message;
    EXPECT_EQ(active_session, previous.session);
    EXPECT_FALSE(transaction.validate(
        [](const shell::RegistrationActivationSnapshot &) {
          return shell::LoadFailure{
          shell::LoadFailureCode::kCapabilityMismatch,
          "test candidate capability rejection"};
      }, &failure));
    EXPECT_EQ(failure.code, shell::LoadFailureCode::kCapabilityMismatch);
    EXPECT_FALSE(transaction.committed());
    EXPECT_EQ(active_session, previous.session);
    EXPECT_EQ(active_plugin, previous.session->plugin());
  }

  // Release the caller's candidate reference.  The uncommitted transaction
  // must have dropped its reference without touching the previous lease.
  candidate.session.reset();
  EXPECT_TRUE(candidate_weak.expired());
  EXPECT_TRUE(previous.session->hasExternalLoaderLease());
  EXPECT_EQ(active_session, previous.session);
  EXPECT_EQ(active_plugin, previous.session->plugin());
  registration::PointCloud::Ptr cloud = makeCloud();
  std::string error;
  ASSERT_TRUE(active_plugin->setInputTarget(cloud, &error)) << error;
  registration::AlignmentRequest alignment_request;
  alignment_request.source = cloud;
  alignment_request.initial_guess_enabled = false;
  const registration::AlignmentResult aligned = active_plugin->align(alignment_request);
  EXPECT_EQ(aligned.failure, registration::FailureCode::kNone);
  EXPECT_TRUE(aligned.converged);
}

TEST(RegistrationPluginLoader, ActivationCommitSwapsPairAndReleasesPreviousAfterLease)
{
  shell::RegistrationPluginLoader loader;
  shell::LoadRequest request = requestFor(kIdentity);
  request.parameters.emplace("accept", registration::ParameterValue(true));
  shell::LoadResult previous = loader.load(request);
  shell::LoadResult candidate = loader.load(request);
  ASSERT_TRUE(previous.ok()) << previous.failure.message;
  ASSERT_TRUE(candidate.ok()) << candidate.failure.message;

  std::shared_ptr<shell::RegistrationPluginSession> active_session = previous.session;
  std::shared_ptr<registration::RegistrationPlugin> active_plugin = previous.session->plugin();
  const std::weak_ptr<shell::RegistrationPluginSession> previous_weak = previous.session;
  const auto candidate_session = candidate.session;
  shell::RegistrationActivationSlots slots{&active_session, &active_plugin, nullptr, nullptr,
    nullptr};
  {
    shell::RegistrationActivationTransaction transaction(slots);
    shell::LoadFailure failure;
    ASSERT_TRUE(transaction.prepare(candidate_session, &failure)) << failure.message;
    ASSERT_TRUE(transaction.validate(
        [](const shell::RegistrationActivationSnapshot & snapshot) {
          if (snapshot.session->classId() != kIdentity ||
          !snapshot.session->hasExternalLoaderLease())
          {
            return shell::LoadFailure{
            shell::LoadFailureCode::kProvenanceInvalid,
            "test candidate provenance mismatch"};
          }
          return shell::LoadFailure{};
      }, &failure)) << failure.message;
    ASSERT_TRUE(transaction.commit(&failure)) << failure.message;
    EXPECT_TRUE(transaction.committed());
    EXPECT_EQ(active_session, candidate_session);
    EXPECT_EQ(active_plugin, candidate_session->plugin());
  }

  previous.session.reset();
  EXPECT_TRUE(previous_weak.expired());
  EXPECT_TRUE(active_session->hasExternalLoaderLease());
  EXPECT_EQ(active_session, candidate_session);
}

TEST(RegistrationPluginLoader, ActivationRejectsConcurrentHostSlotMutation)
{
  shell::RegistrationPluginLoader loader;
  shell::LoadRequest request = requestFor(kIdentity);
  request.parameters.emplace("accept", registration::ParameterValue(true));
  shell::LoadResult previous = loader.load(request);
  shell::LoadResult candidate = loader.load(request);
  shell::LoadResult concurrent = loader.load(request);
  ASSERT_TRUE(previous.ok()) << previous.failure.message;
  ASSERT_TRUE(candidate.ok()) << candidate.failure.message;
  ASSERT_TRUE(concurrent.ok()) << concurrent.failure.message;

  std::shared_ptr<shell::RegistrationPluginSession> active_session = previous.session;
  std::shared_ptr<registration::RegistrationPlugin> active_plugin = previous.session->plugin();
  shell::RegistrationActivationSlots slots{&active_session, &active_plugin, nullptr, nullptr,
    nullptr};
  shell::RegistrationActivationTransaction transaction(slots);
  shell::LoadFailure failure;
  ASSERT_TRUE(transaction.prepare(candidate.session, &failure)) << failure.message;
  ASSERT_TRUE(transaction.validate(
      [](const shell::RegistrationActivationSnapshot &) {return shell::LoadFailure{};},
    &failure)) << failure.message;

  // Simulate another host-owned startup path touching the slot while the
  // candidate was being validated.  The transaction must not clobber it.
  active_plugin = concurrent.session->plugin();
  EXPECT_FALSE(transaction.commit(&failure));
  EXPECT_EQ(failure.code, shell::LoadFailureCode::kInvalidRequest);
  EXPECT_EQ(active_session, previous.session);
  EXPECT_EQ(active_plugin, concurrent.session->plugin());
}

TEST(RegistrationPluginLoader, ActivationRollbackRestoresPreviousPairAfterCommit)
{
  shell::RegistrationPluginLoader loader;
  shell::LoadRequest request = requestFor(kIdentity);
  request.parameters.emplace("accept", registration::ParameterValue(true));
  shell::LoadResult previous = loader.load(request);
  shell::LoadResult candidate = loader.load(request);
  ASSERT_TRUE(previous.ok()) << previous.failure.message;
  ASSERT_TRUE(candidate.ok()) << candidate.failure.message;

  std::shared_ptr<shell::RegistrationPluginSession> active_session = previous.session;
  std::shared_ptr<registration::RegistrationPlugin> active_plugin = previous.session->plugin();
  const std::weak_ptr<shell::RegistrationPluginSession> candidate_weak = candidate.session;
  shell::RegistrationActivationSlots slots{&active_session, &active_plugin, nullptr, nullptr,
    nullptr};
  {
    shell::RegistrationActivationTransaction transaction(slots);
    shell::LoadFailure failure;
    ASSERT_TRUE(transaction.prepare(candidate.session, &failure)) << failure.message;
    ASSERT_TRUE(transaction.validate(
        [](const shell::RegistrationActivationSnapshot &) {return shell::LoadFailure{};},
      &failure)) << failure.message;
    ASSERT_TRUE(transaction.commit(&failure)) << failure.message;
    EXPECT_EQ(active_session, candidate.session);
    ASSERT_TRUE(transaction.rollback(&failure)) << failure.message;
    EXPECT_FALSE(transaction.committed());
    EXPECT_EQ(active_session, previous.session);
    EXPECT_EQ(active_plugin, previous.session->plugin());
  }

  candidate.session.reset();
  EXPECT_TRUE(candidate_weak.expired());
  EXPECT_TRUE(active_session->hasExternalLoaderLease());
}

TEST(RegistrationPluginLoader, RejectsUnstableExternalDsoProvenance)
{
  const shell::LoadFailure relative = shell::validateExternalDsoProvenance(
    kIdentity, "libregistration.so", "/tmp/registration_plugins.xml");
  EXPECT_EQ(relative.code, shell::LoadFailureCode::kProvenanceInvalid);

  const shell::LoadFailure missing = shell::validateExternalDsoProvenance(
    kIdentity, "/definitely/missing/libregistration.so", "/definitely/missing/plugins.xml");
  EXPECT_EQ(missing.code, shell::LoadFailureCode::kProvenanceInvalid);

  const std::filesystem::path root = std::filesystem::temp_directory_path() /
    ("lidarslam-registration-provenance-" +
    std::to_string(static_cast<std::int64_t>(::getpid())));
  std::error_code error;
  std::filesystem::remove_all(root, error);
  error.clear();
  std::filesystem::create_directory(root, error);
  ASSERT_FALSE(error) << error.message();
  const std::filesystem::path library = root / "libregistration.so";
  const std::filesystem::path manifest = root / "registration_plugins.xml";
  {
    std::ofstream(library) << "not an executable library";
    std::ofstream(manifest) << "<library/>";
  }

  const shell::LoadFailure same_file = shell::validateExternalDsoProvenance(
    kIdentity, library.string(), library.string());
  EXPECT_EQ(same_file.code, shell::LoadFailureCode::kProvenanceInvalid);

  const std::filesystem::path link = root / "libregistration-link.so";
  std::filesystem::create_symlink(library, link, error);
  if (!error) {
    const shell::LoadFailure symlink = shell::validateExternalDsoProvenance(
      kIdentity, link.string(), manifest.string());
    EXPECT_EQ(symlink.code, shell::LoadFailureCode::kProvenanceInvalid);
  }
  error.clear();
  const std::filesystem::path manifest_link = root / "registration_plugins-link.xml";
  std::filesystem::create_symlink(manifest, manifest_link, error);
  if (!error) {
    EXPECT_TRUE(shell::validateExternalDsoProvenance(
      kIdentity, library.string(), manifest_link.string()).ok());
  }
  std::filesystem::remove_all(root, error);
}

TEST(RegistrationPluginLoader, RejectsSymlinkedDsoBeforePluginConstructor)
{
  std::filesystem::path real_library;
  {
    shell::RegistrationPluginLoader installed_loader;
    const shell::LoadResult installed = installed_loader.load(requestFor(kIdentity));
    ASSERT_TRUE(installed.ok()) << installed.failure.message;
    real_library = installed.session->libraryPath();
  }

  const std::filesystem::path root = std::filesystem::temp_directory_path() /
    ("lidarslam-registration-preload-" +
    std::to_string(static_cast<std::int64_t>(::getpid())));
  std::error_code error;
  std::filesystem::remove_all(root, error);
  error.clear();
  std::filesystem::create_directories(
    root / "share" / "lidarslam_fake_registration_plugins", error);
  ASSERT_FALSE(error) << error.message();
  std::filesystem::create_directories(
    root / "share" / "ament_index" / "resource_index" / "packages", error);
  ASSERT_FALSE(error) << error.message();
  std::filesystem::create_directory(root / "lib", error);
  ASSERT_FALSE(error) << error.message();

  const std::filesystem::path package_xml =
    root / "share" / "lidarslam_fake_registration_plugins" / "package.xml";
  const std::filesystem::path plugin_xml =
    root / "share" / "lidarslam_fake_registration_plugins" / "preload_plugins.xml";
  const std::filesystem::path resource_index = root / "share" / "ament_index" /
    "resource_index" / "packages" / "lidarslam_fake_registration_plugins";
  const std::filesystem::path dso_link = root / "lib" /
    "liblidarslam_fake_registration_plugins.so";
  const std::filesystem::path marker = root / "constructor.marker";
  {
    std::ofstream(package_xml) <<
      "<?xml version=\"1.0\"?><package format=\"3\"><name>"
      "lidarslam_fake_registration_plugins</name></package>\n";
    std::ofstream(resource_index) << "\n";
    std::ofstream(plugin_xml) <<
      "<?xml version=\"1.0\"?>\n"
      "<library path=\"lidarslam_fake_registration_plugins\">\n"
      "  <class name=\"lidarslam_fake_registration_plugins/ConstructorProbe\" "
      "type=\"lidarslam_fake_registration_plugins::ConstructorProbeRegistration\" "
      "base_class_type=\"lidarslam::plugins::registration::RegistrationPlugin\">\n"
      "    <description>pre-load constructor probe</description>\n"
      "  </class>\n"
      "</library>\n";
  }
  std::filesystem::create_symlink(real_library, dso_link, error);
  if (error) {
    std::filesystem::remove_all(root, error);
    GTEST_SKIP() << "symlink creation is unavailable: " << error.message();
  }

  const char * previous_prefix = std::getenv("AMENT_PREFIX_PATH");
  const std::string previous_prefix_value =
    previous_prefix == nullptr ? std::string() : std::string(previous_prefix);
  const char * previous_marker = std::getenv("LIDARSLAM_FAKE_PLUGIN_CONSTRUCTOR_MARKER");
  const std::string previous_marker_value =
    previous_marker == nullptr ? std::string() : std::string(previous_marker);
  const bool had_prefix = previous_prefix != nullptr;
  const bool had_marker = previous_marker != nullptr;
  const std::string prefix_value = root.string() + ":" + previous_prefix_value;
  (void)setenv("AMENT_PREFIX_PATH", prefix_value.c_str(), 1);
  (void)setenv("LIDARSLAM_FAKE_PLUGIN_CONSTRUCTOR_MARKER", marker.c_str(), 1);

  shell::RegistrationPluginLoader loader(
    shell::kRegistrationPluginBasePackage, {plugin_xml.string()});
  shell::LoadRequest request = requestFor(
    "lidarslam_fake_registration_plugins/ConstructorProbe");
  const shell::LoadResult loaded = loader.load(request);
  EXPECT_FALSE(loaded.ok());
  EXPECT_EQ(loaded.failure.code, shell::LoadFailureCode::kProvenanceInvalid);
  EXPECT_EQ(loaded.session, nullptr);
  EXPECT_FALSE(std::filesystem::exists(marker));

  if (had_prefix) {
    (void)setenv("AMENT_PREFIX_PATH", previous_prefix_value.c_str(), 1);
  } else {
    (void)unsetenv("AMENT_PREFIX_PATH");
  }
  if (had_marker) {
    (void)setenv(
      "LIDARSLAM_FAKE_PLUGIN_CONSTRUCTOR_MARKER", previous_marker_value.c_str(), 1);
  } else {
    (void)unsetenv("LIDARSLAM_FAKE_PLUGIN_CONSTRUCTOR_MARKER");
  }
  std::filesystem::remove_all(root, error);
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

TEST(RegistrationPluginLoader, DiscoversAndConfiguresOptionalFastClassesOnlyWhenAdvertised)
{
  shell::RegistrationPluginLoader loader;
  const std::vector<std::string> classes = loader.availableClasses();
  const std::string gicp_class = "lidarslam_default_plugins/FastGicp";
  const std::string vgicp_class = "lidarslam_default_plugins/FastVGicp";
  const bool has_gicp =
    std::find(classes.begin(), classes.end(), gicp_class) != classes.end();
  const bool has_vgicp =
    std::find(classes.begin(), classes.end(), vgicp_class) != classes.end();

  // The optional pair is advertised atomically.  A package without
  // fast_gicp must expose neither class and must never reinterpret a FAST
  // selector as NDT or GICP.
  EXPECT_EQ(has_gicp, has_vgicp);
  if (!has_gicp || !has_vgicp) {
    return;
  }

  const auto make_request = [](const std::string & class_id, const bool voxelized) {
      shell::LoadRequest request = requestFor(class_id);
      request.parameters.emplace(
        "maximum_correspondence_distance", registration::ParameterValue(5.0));
      request.parameters.emplace("transformation_epsilon", registration::ParameterValue(1e-6));
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
  EXPECT_FALSE(gicp.session->libraryPath().empty());
  EXPECT_NE(
    gicp.session->libraryPath().find("liblidarslam_fast_gicp_plugins"),
    std::string::npos);
  EXPECT_NE(
    gicp.session->pluginManifestPath().find("registration_plugins_fast.xml"),
    std::string::npos);

  const shell::LoadResult vgicp = loader.load(make_request(vgicp_class, true));
  ASSERT_TRUE(vgicp.ok()) << vgicp.failure.message;
  ASSERT_NE(vgicp.session, nullptr);
  EXPECT_EQ(vgicp.session->metadata().class_id, vgicp_class);
  EXPECT_EQ(vgicp.session->backendKind(), shell::BackendKind::kPluginlib);
  EXPECT_FALSE(vgicp.session->libraryPath().empty());
  EXPECT_NE(
    vgicp.session->libraryPath().find("liblidarslam_fast_gicp_plugins"),
    std::string::npos);
  EXPECT_NE(
    vgicp.session->pluginManifestPath().find("registration_plugins_fast.xml"),
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
    EXPECT_FALSE(loaded.session->hasExternalLoaderLease());
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

TEST(RegistrationResolver, SessionBoundaryConvertsAndLatchesProcessingExceptions)
{
  shell::RegistrationResolver resolver({throwingProcessingSpec()});
  const registration::PointCloud::Ptr cloud = makeCloud();

  const shell::LoadResult target_loaded = resolver.resolve(throwingProcessingRequest());
  ASSERT_TRUE(target_loaded.ok()) << target_loaded.failure.message;
  std::string target_error;
  EXPECT_FALSE(target_loaded.session->setInputTarget(cloud, &target_error));
  EXPECT_NE(target_error.find("synthetic target exception"), std::string::npos);
  EXPECT_TRUE(target_loaded.session->faulted());
  EXPECT_FALSE(target_loaded.session->setInputTarget(cloud, &target_error));
  EXPECT_NE(target_error.find("faulted"), std::string::npos);

  const shell::LoadResult align_loaded = resolver.resolve(throwingProcessingRequest());
  ASSERT_TRUE(align_loaded.ok()) << align_loaded.failure.message;
  registration::AlignmentRequest request;
  request.source = cloud;
  request.initial_guess_enabled = false;
  const registration::AlignmentResult aligned = align_loaded.session->align(request);
  EXPECT_EQ(aligned.failure, registration::FailureCode::kInternalError);
  EXPECT_NE(aligned.diagnostics.detail.find("synthetic alignment exception"), std::string::npos);
  EXPECT_TRUE(align_loaded.session->faulted());
}

TEST(RegistrationResolver, SessionCancellationBlocksFutureProcessing)
{
  shell::RegistrationResolver resolver({hostIdentitySpec()});
  const shell::LoadResult loaded = resolver.resolve(hostIdentityRequest(true));
  ASSERT_TRUE(loaded.ok()) << loaded.failure.message;
  const registration::PointCloud::Ptr cloud = makeCloud();
  std::string error;
  ASSERT_TRUE(loaded.session->setInputTarget(cloud, &error)) << error;

  loaded.session->cancel();
  EXPECT_TRUE(loaded.session->cancelled());
  registration::AlignmentRequest request;
  request.source = cloud;
  request.initial_guess_enabled = false;
  const registration::AlignmentResult aligned = loaded.session->align(request);
  EXPECT_EQ(aligned.failure, registration::FailureCode::kCancelled);
  EXPECT_NE(aligned.diagnostics.detail.find("cancelled"), std::string::npos);
  EXPECT_FALSE(loaded.session->setInputTarget(cloud, &error));
  EXPECT_NE(error.find("cancelled"), std::string::npos);
  loaded.session->shutdown();
  loaded.session->shutdown();
  EXPECT_TRUE(loaded.session->cancelled());
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
