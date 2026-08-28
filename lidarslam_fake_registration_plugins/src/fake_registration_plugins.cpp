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

#include "lidarslam_fake_registration_plugins/fake_registration_plugins.hpp"

#include <cmath>
#include <cstdlib>
#include <exception>
#include <fstream>

#include <pluginlib/class_list_macros.hpp>

namespace lidarslam_fake_registration_plugins
{
namespace registration = lidarslam::plugins::registration;

BasicRegistration::BasicRegistration(std::string class_id, std::uint16_t api_major)
: class_id_(std::move(class_id)), api_major_(api_major) {}

registration::PluginMetadata BasicRegistration::metadata() const
{
  registration::PluginMetadata metadata;
  metadata.class_id = class_id_;
  metadata.implementation_version = "0.1.0";
  metadata.license = "BSD-2-Clause";
  metadata.api_version = {api_major_, 0};
  return metadata;
}

registration::Capabilities BasicRegistration::capabilities() const
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

registration::RegistrationRuntimeDescriptor BasicRegistration::registrationDescriptor() const
{
  const std::uint64_t deterministic =
    static_cast<std::uint64_t>(registration::Capability::kDeterministic);
  const registration::Capabilities current = capabilities();
  return registration::makeRegistrationRuntimeDescriptor(
    metadata(), current, current.bits() & ~deterministic, deterministic,
    registration::registrationConfigSchemaForClassId(class_id_));
}

bool BasicRegistration::configure(
  const registration::ParameterMap & parameters, std::string * error)
{
  for (const auto & entry : parameters) {
    if (entry.first != "accept") {
      if (error != nullptr) {
        *error = "unknown fake parameter '" + entry.first + "'";
      }
      return false;
    }
    try {
      if (entry.second.asBool() != true) {
        if (error != nullptr) {
          *error = "fake parameter 'accept' must be true";
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

bool BasicRegistration::setInputTarget(
  const registration::PointCloudConstPtr & target, std::string * error)
{
  if (!configured_) {
    if (error != nullptr) {
      *error = "fake registration is not configured";
    }
    return false;
  }
  if (!target || target->empty()) {
    if (error != nullptr) {
      *error = "fake registration target is empty";
    }
    return false;
  }
  target_ = target;
  return true;
}

registration::AlignmentResult BasicRegistration::align(
  const registration::AlignmentRequest & request)
{
  registration::AlignmentResult result;
  if (!configured_) {
    result.failure = registration::FailureCode::kNotConfigured;
    result.diagnostics.detail = "fake registration is not configured";
    return result;
  }
  if (!target_ || target_->empty()) {
    result.failure = registration::FailureCode::kInvalidInput;
    result.diagnostics.detail = "fake registration target is empty";
    return result;
  }
  const registration::FailureCode request_status =
    registration::validateRequest(request, capabilities());
  if (request_status != registration::FailureCode::kNone) {
    result.failure = request_status;
    result.diagnostics.detail = "fake registration request rejected";
    return result;
  }
  result.final_transformation = request.initial_guess_enabled ?
    request.initial_guess : Eigen::Matrix4f::Identity();
  result.aligned_source.reset(new registration::PointCloud(*request.source));
  result.converged = true;
  result.failure = registration::FailureCode::kNone;
  result.diagnostics.mean_correspondence_distance_valid = true;
  result.diagnostics.mean_correspondence_distance = 0.0;
  result.fitness_score = 0.0;
  return result;
}

void BasicRegistration::reset() noexcept
{
  configured_ = false;
  target_.reset();
}

IdentityRegistration::IdentityRegistration()
: BasicRegistration("lidarslam_fake_registration_plugins/Identity") {}

NoGuessRegistration::NoGuessRegistration()
: BasicRegistration("lidarslam_fake_registration_plugins/NoGuess") {}

registration::Capabilities NoGuessRegistration::capabilities() const
{
  registration::Capabilities capabilities = BasicRegistration::capabilities();
  return registration::Capabilities(
    capabilities.bits() &
    ~static_cast<std::uint64_t>(registration::Capability::kInitialGuess))
         .setTargetPolicy(capabilities.targetPolicy())
         .setCorrespondenceMetric(capabilities.correspondenceMetric())
         .setThreadModel(capabilities.threadModel());
}

BadApiRegistration::BadApiRegistration()
: BasicRegistration("lidarslam_fake_registration_plugins/BadApi", 2) {}

BadMetadataRegistration::BadMetadataRegistration()
: BasicRegistration("lidarslam_fake_registration_plugins/BadMetadata") {}

registration::PluginMetadata BadMetadataRegistration::metadata() const
{
  registration::PluginMetadata metadata;
  metadata.implementation_version = "0.1.0";
  metadata.api_version = registration::kHostApiVersion;
  return metadata;
}

RejectingRegistration::RejectingRegistration()
: BasicRegistration("lidarslam_fake_registration_plugins/Rejecting") {}

bool RejectingRegistration::configure(
  const registration::ParameterMap &, std::string * error)
{
  if (error != nullptr) {
    *error = "fake plugin rejects configuration by contract";
  }
  return false;
}

UnlicensedRegistration::UnlicensedRegistration()
: BasicRegistration("lidarslam_fake_registration_plugins/Unlicensed") {}

registration::PluginMetadata UnlicensedRegistration::metadata() const
{
  registration::PluginMetadata metadata = BasicRegistration::metadata();
  metadata.license = "GPL-3.0-only";
  return metadata;
}

ConstructorProbeRegistration::ConstructorProbeRegistration()
: BasicRegistration("lidarslam_fake_registration_plugins/ConstructorProbe")
{
  const char * marker = std::getenv("LIDARSLAM_FAKE_PLUGIN_CONSTRUCTOR_MARKER");
  if (marker != nullptr && *marker != '\0') {
    std::ofstream output(marker, std::ios::app);
    output << "constructor\n";
  }
}

}  // namespace lidarslam_fake_registration_plugins

PLUGINLIB_EXPORT_CLASS(
  lidarslam_fake_registration_plugins::IdentityRegistration,
  lidarslam::plugins::registration::RegistrationPlugin)
PLUGINLIB_EXPORT_CLASS(
  lidarslam_fake_registration_plugins::NoGuessRegistration,
  lidarslam::plugins::registration::RegistrationPlugin)
PLUGINLIB_EXPORT_CLASS(
  lidarslam_fake_registration_plugins::BadApiRegistration,
  lidarslam::plugins::registration::RegistrationPlugin)
PLUGINLIB_EXPORT_CLASS(
  lidarslam_fake_registration_plugins::BadMetadataRegistration,
  lidarslam::plugins::registration::RegistrationPlugin)
PLUGINLIB_EXPORT_CLASS(
  lidarslam_fake_registration_plugins::RejectingRegistration,
  lidarslam::plugins::registration::RegistrationPlugin)
PLUGINLIB_EXPORT_CLASS(
  lidarslam_fake_registration_plugins::UnlicensedRegistration,
  lidarslam::plugins::registration::RegistrationPlugin)
PLUGINLIB_EXPORT_CLASS(
  lidarslam_fake_registration_plugins::ConstructorProbeRegistration,
  lidarslam::plugins::registration::RegistrationPlugin)
