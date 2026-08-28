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

#include "lidarslam_registration_plugin_template/template_registration.hpp"

#include <cmath>
#include <exception>

#include <pcl/common/transforms.h>  // NOLINT(build/include_order)
#include <pluginlib/class_list_macros.hpp>  // NOLINT(build/include_order)

namespace lidarslam_registration_plugin_template
{
namespace registration = lidarslam::plugins::registration;
namespace
{

bool finitePoint(const registration::PointT & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z) && std::isfinite(point.intensity);
}

bool finiteCloud(const registration::PointCloud & cloud)
{
  for (const auto & point : cloud.points) {
    if (!finitePoint(point)) {
      return false;
    }
  }
  return true;
}

registration::AlignmentResult failureResult(
  registration::FailureCode code, const std::string & detail)
{
  registration::AlignmentResult result;
  result.failure = code;
  result.diagnostics.detail = detail;
  return result;
}

}  // namespace

registration::PluginMetadata IdentityRegistration::metadata() const
{
  registration::PluginMetadata metadata;
  metadata.class_id = "lidarslam_registration_plugin_template/Identity";
  metadata.implementation_version = "0.1.0";
  metadata.license = "BSD-2-Clause";
  metadata.api_version = registration::kHostApiVersion;
  return metadata;
}

registration::Capabilities IdentityRegistration::capabilities() const
{
  registration::Capabilities capabilities;
  capabilities
  .add(registration::Capability::kInitialGuess)
  .add(registration::Capability::kAlignedSource)
  .add(registration::Capability::kDeterministic)
  .setTargetPolicy(registration::TargetPolicy::kAcceptHostPrepared)
  .setCorrespondenceMetric(registration::CorrespondenceMetric::kMeanDistance)
  .setThreadModel(registration::ThreadModel::kSerializedOwner);
  return capabilities;
}

registration::RegistrationRuntimeDescriptor IdentityRegistration::registrationDescriptor() const
{
  const std::uint64_t deterministic =
    static_cast<std::uint64_t>(registration::Capability::kDeterministic);
  const registration::Capabilities current = capabilities();
  return registration::makeRegistrationRuntimeDescriptor(
    metadata(), current, current.bits() & ~deterministic, deterministic,
    registration::registrationConfigSchemaForClassId(metadata().class_id));
}

bool IdentityRegistration::configure(
  const registration::ParameterMap & parameters, std::string * error)
{
  try {
    std::string requested_mode = "identity";
    for (const auto & entry : parameters) {
      if (entry.first != "mode") {
        if (error != nullptr) {
          *error = "unknown parameter '" + entry.first + "'";
        }
        return false;
      }
      requested_mode = entry.second.asString();
    }
    if (requested_mode != "identity") {
      if (error != nullptr) {
        *error = "parameter 'mode' must be 'identity'";
      }
      return false;
    }
    mode_ = requested_mode;
    configured_ = true;
    return true;
  } catch (const std::exception & exception) {
    if (error != nullptr) {
      *error = exception.what();
    }
    return false;
  } catch (...) {
    if (error != nullptr) {
      *error = "unknown exception while validating parameters";
    }
    return false;
  }
}

bool IdentityRegistration::setInputTarget(
  const registration::PointCloudConstPtr & target, std::string * error)
{
  try {
    if (!configured_) {
      if (error != nullptr) {
        *error = "registration plugin is not configured";
      }
      return false;
    }
    if (!target || target->empty() || !finiteCloud(*target)) {
      if (error != nullptr) {
        *error = "registration target must be non-empty and finite";
      }
      return false;
    }
    target_.reset(new registration::PointCloud(*target));
    return true;
  } catch (const std::exception & exception) {
    if (error != nullptr) {
      *error = exception.what();
    }
    return false;
  } catch (...) {
    if (error != nullptr) {
      *error = "unknown exception while setting registration target";
    }
    return false;
  }
}

registration::AlignmentResult IdentityRegistration::align(
  const registration::AlignmentRequest & request)
{
  try {
    if (!configured_ || mode_ != "identity") {
      return failureResult(
        registration::FailureCode::kNotConfigured,
        "registration plugin is not configured");
    }
    if (!target_ || target_->empty()) {
      return failureResult(
        registration::FailureCode::kInvalidInput,
        "registration target has not been set");
    }
    if (!request.source || request.source->empty() || !finiteCloud(*request.source)) {
      return failureResult(
        registration::FailureCode::kInvalidInput,
        "registration source must be non-empty and finite");
    }
    const registration::FailureCode request_status =
      registration::validateRequest(request, capabilities());
    if (request_status != registration::FailureCode::kNone) {
      return failureResult(request_status, "registration request rejected by capabilities");
    }
    if (request.initial_guess_enabled && !request.initial_guess.allFinite()) {
      return failureResult(
        registration::FailureCode::kInvalidInput,
        "enabled registration initial guess must be finite");
    }

    registration::AlignmentResult result;
    result.final_transformation = request.initial_guess_enabled ?
      request.initial_guess : Eigen::Matrix4f::Identity();
    result.aligned_source.reset(new registration::PointCloud());
    pcl::transformPointCloud(
      *request.source, *result.aligned_source,
      request.initial_guess_enabled ? request.initial_guess : Eigen::Matrix4f::Identity());
    result.converged = true;
    result.fitness_score = 0.0;
    result.failure = registration::FailureCode::kNone;
    result.diagnostics.mean_correspondence_distance_valid = true;
    result.diagnostics.mean_correspondence_distance = 0.0;
    return result;
  } catch (const std::exception & exception) {
    return failureResult(registration::FailureCode::kInternalError, exception.what());
  } catch (...) {
    return failureResult(
      registration::FailureCode::kInternalError,
      "unknown exception during registration alignment");
  }
}

void IdentityRegistration::reset() noexcept
{
  configured_ = false;
  mode_ = "identity";
  target_.reset();
}

}  // namespace lidarslam_registration_plugin_template

PLUGINLIB_EXPORT_CLASS(
  lidarslam_registration_plugin_template::IdentityRegistration,
  lidarslam::plugins::registration::RegistrationPlugin)
