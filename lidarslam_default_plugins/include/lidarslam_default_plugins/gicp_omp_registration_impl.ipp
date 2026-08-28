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
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
// AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
// THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef LIDARSLAM_DEFAULT_PLUGINS__GICP_OMP_REGISTRATION_IMPL_IPP_
#define LIDARSLAM_DEFAULT_PLUGINS__GICP_OMP_REGISTRATION_IMPL_IPP_

#include "lidarslam_default_plugins/gicp_omp_registration.hpp"

#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <sstream>
#include <string>

#include <pclomp/gicp_omp.h>  // NOLINT(build/include_order)
#include <pclomp/gicp_omp_impl.hpp>  // NOLINT(build/include_order)

namespace lidarslam_default_plugins
{
namespace
{

namespace registration = lidarslam::plugins::registration;
using registration::AlignmentRequest;
using registration::AlignmentResult;
using registration::Capabilities;
using registration::Capability;
using registration::CorrespondenceMetric;
using registration::FailureCode;
using registration::ParameterMap;
using registration::PluginMetadata;
using registration::RegistrationRuntimeDescriptor;
using registration::PointCloud;
using registration::PointCloudConstPtr;
using registration::PointT;
using registration::TargetPolicy;
using registration::ThreadModel;
using registration::kHostApiVersion;
using registration::validateRequest;

constexpr double kLegacyMaximumCorrespondenceDistance = 5.0;
constexpr double kLegacyTransformationEpsilon = 1e-8;

struct GicpConfiguration
{
  double maximum_correspondence_distance{kLegacyMaximumCorrespondenceDistance};
  double transformation_epsilon{kLegacyTransformationEpsilon};
  bool adaptive_correspondence_threshold{false};
};

void gicpSetError(std::string * error, const std::string & message)
{
  if (error != nullptr) {
    *error = message;
  }
}

std::string gicpParameterError(const std::string & key, const std::string & detail)
{
  return "invalid GICP parameter '" + key + "': " + detail;
}

bool gicpFinite(const double value)
{
  return std::isfinite(value) != 0;
}

bool gicpFinitePointCloud(const PointCloudConstPtr & cloud)
{
  if (!cloud || cloud->empty()) {
    return false;
  }
  for (const PointT & point : cloud->points) {
    if (
      !gicpFinite(static_cast<double>(point.x)) ||
      !gicpFinite(static_cast<double>(point.y)) ||
      !gicpFinite(static_cast<double>(point.z)) ||
      !gicpFinite(static_cast<double>(point.intensity)))
    {
      return false;
    }
  }
  return true;
}

bool parseConfiguration(
  const ParameterMap & parameters, GicpConfiguration * configuration, std::string * error)
{
  if (configuration == nullptr) {
    gicpSetError(error, "internal error: null GICP configuration");
    return false;
  }
  *configuration = GicpConfiguration();
  bool saw_maximum_correspondence_distance = false;
  bool saw_transformation_epsilon = false;
  bool saw_adaptive_correspondence_threshold = false;
  for (const auto & entry : parameters) {
    const std::string & key = entry.first;
    try {
      if (key == "maximum_correspondence_distance") {
        const double value = entry.second.asDouble();
        if (!gicpFinite(value) || value <= 0.0) {
          gicpSetError(error, gicpParameterError(key, "must be finite and greater than zero"));
          return false;
        }
        configuration->maximum_correspondence_distance = value;
        saw_maximum_correspondence_distance = true;
      } else if (key == "transformation_epsilon") {
        const double value = entry.second.asDouble();
        if (!gicpFinite(value) || value != kLegacyTransformationEpsilon) {
          gicpSetError(
            error,
            gicpParameterError(key, "must equal the legacy fixed value 1e-8"));
          return false;
        }
        configuration->transformation_epsilon = value;
        saw_transformation_epsilon = true;
      } else if (key == "adaptive_correspondence_threshold") {
        configuration->adaptive_correspondence_threshold = entry.second.asBool();
        saw_adaptive_correspondence_threshold = true;
      } else {
        gicpSetError(error, gicpParameterError(key, "unknown key"));
        return false;
      }
    } catch (const std::exception & exception) {
      gicpSetError(error, gicpParameterError(key, exception.what()));
      return false;
    } catch (...) {
      gicpSetError(error, gicpParameterError(key, "type conversion failed"));
      return false;
    }
  }
  if (!saw_maximum_correspondence_distance) {
    gicpSetError(
      error,
      gicpParameterError("maximum_correspondence_distance", "is required"));
    return false;
  }
  if (!saw_transformation_epsilon) {
    gicpSetError(error, gicpParameterError("transformation_epsilon", "is required"));
    return false;
  }
  if (!saw_adaptive_correspondence_threshold) {
    gicpSetError(
      error,
      gicpParameterError("adaptive_correspondence_threshold", "is required"));
    return false;
  }
  return true;
}

const char * gicpFailureName(const FailureCode failure)
{
  switch (failure) {
    case FailureCode::kNone:
      return "none";
    case FailureCode::kNotConfigured:
      return "not configured";
    case FailureCode::kInvalidInput:
      return "invalid input";
    case FailureCode::kUnsupportedCapability:
      return "unsupported capability";
    case FailureCode::kAlignmentFailed:
      return "alignment failed";
    case FailureCode::kCancelled:
      return "cancelled";
    case FailureCode::kInternalError:
      return "internal error";
  }
  return "unknown failure";
}

}  // namespace

struct GicpOmpRegistration::Impl
{
  using Gicp = pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>;

  GicpConfiguration configuration;
  typename Gicp::Ptr gicp;
  PointCloudConstPtr target;
  bool configured{false};
  bool reset_distance_after_call{false};
};

void GicpOmpRegistration::clearPerCallState(Impl * implementation) noexcept
{
  if (implementation == nullptr || !implementation->gicp) {
    return;
  }
  if (implementation->reset_distance_after_call) {
    try {
      // This is the historical scanmatcher reset value after an adaptive
      // GICP call.  It is intentionally not restored to the configured
      // threshold.
      implementation->gicp->setMaxCorrespondenceDistance(
        std::numeric_limits<double>::max());
    } catch (...) {
    }
    implementation->reset_distance_after_call = false;
  }
}

struct GicpOmpRegistration::PerCallStateGuard
{
  explicit PerCallStateGuard(Impl * implementation)
  : implementation(implementation) {}

  ~PerCallStateGuard() noexcept
  {
    GicpOmpRegistration::clearPerCallState(implementation);
  }

  Impl * implementation;
};

GicpOmpRegistration::GicpOmpRegistration()
: impl_(new Impl())
{
}

GicpOmpRegistration::~GicpOmpRegistration() = default;

PluginMetadata GicpOmpRegistration::metadata() const
{
  PluginMetadata metadata;
  metadata.class_id = "lidarslam_default_plugins/GicpOmp";
  metadata.implementation_version = "1.0.0";
  metadata.license = "BSD-2-Clause";
  metadata.api_version = kHostApiVersion;
  return metadata;
}

RegistrationRuntimeDescriptor GicpOmpRegistration::registrationDescriptor() const
{
  const std::uint64_t required =
    static_cast<std::uint64_t>(Capability::kInitialGuess) |
    static_cast<std::uint64_t>(Capability::kMaximumCorrespondenceDistance) |
    static_cast<std::uint64_t>(Capability::kAlignedSource);
  return makeRegistrationRuntimeDescriptor(
    metadata(), capabilities(), required, 0U,
    registration::registrationConfigSchemaForClassId(metadata().class_id));
}

Capabilities GicpOmpRegistration::capabilities() const
{
  Capabilities capabilities;
  capabilities
  .add(Capability::kInitialGuess)
  .add(Capability::kMaximumCorrespondenceDistance)
  .add(Capability::kAlignedSource)
  .setTargetPolicy(TargetPolicy::kAcceptHostPrepared)
  .setCorrespondenceMetric(CorrespondenceMetric::kSquareRootFitnessProxy)
  .setThreadModel(ThreadModel::kSerializedOwner);
  return capabilities;
}

bool GicpOmpRegistration::configure(
  const ParameterMap & parameters, std::string * error)
{
  if (!impl_) {
    gicpSetError(error, "internal error: adapter implementation is unavailable");
    return false;
  }
  if (impl_->configured) {
    gicpSetError(error, "GICP adapter is already configured; call reset() before configure()");
    return false;
  }

  GicpConfiguration configuration;
  if (!parseConfiguration(parameters, &configuration, error)) {
    return false;
  }
  try {
    Impl::Gicp::Ptr gicp(new Impl::Gicp());
    // Keep this order and these values identical to the legacy GICP branch.
    gicp->setMaxCorrespondenceDistance(configuration.maximum_correspondence_distance);
    gicp->setTransformationEpsilon(configuration.transformation_epsilon);
    impl_->configuration = configuration;
    impl_->gicp = gicp;
    impl_->target.reset();
    impl_->reset_distance_after_call = false;
    impl_->configured = true;
    return true;
  } catch (const std::exception & exception) {
    impl_->gicp.reset();
    impl_->target.reset();
    impl_->configured = false;
    gicpSetError(error, std::string("failed to configure GICP: ") + exception.what());
    return false;
  } catch (...) {
    impl_->gicp.reset();
    impl_->target.reset();
    impl_->configured = false;
    gicpSetError(error, "failed to configure GICP: unknown exception");
    return false;
  }
}

bool GicpOmpRegistration::setInputTarget(
  const PointCloudConstPtr & target, std::string * error)
{
  if (!impl_ || !impl_->configured || !impl_->gicp) {
    gicpSetError(error, "GICP adapter is not configured");
    return false;
  }
  if (!gicpFinitePointCloud(target)) {
    gicpSetError(error, "GICP target must be non-empty and contain finite PointXYZI fields");
    return false;
  }
  try {
    impl_->gicp->setInputTarget(target);
    impl_->target = target;
    return true;
  } catch (const std::exception & exception) {
    gicpSetError(error, std::string("failed to set GICP target: ") + exception.what());
    return false;
  } catch (...) {
    gicpSetError(error, "failed to set GICP target: unknown exception");
    return false;
  }
}

AlignmentResult GicpOmpRegistration::align(const AlignmentRequest & request)
{
  AlignmentResult result;
  if (!impl_ || !impl_->configured || !impl_->gicp) {
    result.failure = FailureCode::kNotConfigured;
    result.diagnostics.detail = gicpFailureName(result.failure);
    return result;
  }
  if (!impl_->target || impl_->target->empty()) {
    result.failure = FailureCode::kInvalidInput;
    result.diagnostics.detail = "GICP target has not been set";
    return result;
  }

  const FailureCode request_status = validateRequest(request, capabilities());
  if (request_status != FailureCode::kNone) {
    result.failure = request_status;
    result.diagnostics.detail = gicpFailureName(request_status);
    return result;
  }
  if (
    !gicpFinitePointCloud(request.source) ||
    (request.initial_guess_enabled && !request.initial_guess.allFinite()))
  {
    result.failure = FailureCode::kInvalidInput;
    result.diagnostics.detail = "source and initial guess must contain finite values";
    return result;
  }
  if (
    request.maximum_correspondence_distance_enabled &&
    (!gicpFinite(request.maximum_correspondence_distance) ||
    request.maximum_correspondence_distance <= 0.0))
  {
    result.failure = FailureCode::kInvalidInput;
    result.diagnostics.detail =
      "maximum correspondence distance must be finite and greater than zero";
    return result;
  }

  PerCallStateGuard state_guard(impl_.get());
  try {
    impl_->gicp->setInputSource(request.source);
    // The legacy adaptive branch resets the distance after every call, even
    // when the first call has no EMA-derived override yet.
    impl_->reset_distance_after_call =
      impl_->configuration.adaptive_correspondence_threshold;
    if (request.maximum_correspondence_distance_enabled) {
      impl_->gicp->setMaxCorrespondenceDistance(request.maximum_correspondence_distance);
      impl_->reset_distance_after_call = true;
    }

    PointCloud aligned;
    if (request.initial_guess_enabled) {
      impl_->gicp->align(aligned, request.initial_guess);
    } else {
      impl_->gicp->align(aligned);
    }

    result.converged = impl_->gicp->hasConverged();
    result.final_transformation = impl_->gicp->getFinalTransformation();
    result.fitness_score = impl_->gicp->getFitnessScore();
    if (gicpFinite(result.fitness_score) && result.fitness_score >= 0.0) {
      result.diagnostics.mean_correspondence_distance_valid = true;
      result.diagnostics.mean_correspondence_distance =
        result.fitness_score > 0.0 ? std::sqrt(result.fitness_score) : 0.0;
    }
    if (
      aligned.empty() || aligned.size() != request.source->size() ||
      !result.final_transformation.allFinite() ||
      !gicpFinite(result.fitness_score) ||
      !result.diagnostics.mean_correspondence_distance_valid)
    {
      result.failure = FailureCode::kAlignmentFailed;
      result.diagnostics.detail = "pclomp returned a non-finite or incomplete alignment";
      return result;
    }
    result.aligned_source.reset(new PointCloud(aligned));
    result.failure = FailureCode::kNone;
    return result;
  } catch (const std::exception & exception) {
    result.failure = FailureCode::kInternalError;
    result.diagnostics.detail = std::string("GICP exception: ") + exception.what();
    return result;
  } catch (...) {
    result.failure = FailureCode::kInternalError;
    result.diagnostics.detail = "GICP exception: unknown exception";
    return result;
  }
}

void GicpOmpRegistration::reset() noexcept
{
  if (!impl_) {
    return;
  }
  clearPerCallState(impl_.get());
  impl_->target.reset();
  impl_->gicp.reset();
  impl_->configuration = GicpConfiguration();
  impl_->configured = false;
}

}  // namespace lidarslam_default_plugins

#endif  // LIDARSLAM_DEFAULT_PLUGINS__GICP_OMP_REGISTRATION_IMPL_IPP_
