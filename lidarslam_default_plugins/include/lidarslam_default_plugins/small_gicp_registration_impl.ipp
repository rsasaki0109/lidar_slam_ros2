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

#ifndef LIDARSLAM_DEFAULT_PLUGINS__SMALL_GICP_REGISTRATION_IMPL_IPP_
#define LIDARSLAM_DEFAULT_PLUGINS__SMALL_GICP_REGISTRATION_IMPL_IPP_

#include "lidarslam_default_plugins/small_gicp_registration.hpp"

#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <string>

#include <small_gicp/pcl/pcl_registration.hpp>  // NOLINT(build/include_order)
#include <small_gicp/pcl/pcl_registration_impl.hpp>  // NOLINT(build/include_order)

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
using registration::PointCloud;
using registration::PointCloudConstPtr;
using registration::PointT;
using registration::TargetPolicy;
using registration::ThreadModel;
using registration::kHostApiVersion;
using registration::validateRequest;

// This name is intentionally unique: the implementation include is also
// embedded in scanmatcher_component.cpp alongside the GICP adapter include.
constexpr double kSmallLegacyTransformationEpsilon = 1e-6;

struct SmallGicpConfiguration
{
  double maximum_correspondence_distance{5.0};
  double transformation_epsilon{kSmallLegacyTransformationEpsilon};
  int maximum_iterations{35};
  int num_threads{0};
  double voxel_resolution{1.0};
  bool adaptive_correspondence_threshold{false};
};

void smallSetError(std::string * error, const std::string & message)
{
  if (error != nullptr) {
    *error = message;
  }
}

std::string smallParameterError(const std::string & key, const std::string & detail)
{
  return "invalid small_gicp parameter '" + key + "': " + detail;
}

bool smallFinite(const double value)
{
  return std::isfinite(value) != 0;
}

bool smallFinitePointCloud(const PointCloudConstPtr & cloud)
{
  if (!cloud || cloud->empty()) {
    return false;
  }
  for (const PointT & point : cloud->points) {
    if (
      !smallFinite(static_cast<double>(point.x)) ||
      !smallFinite(static_cast<double>(point.y)) ||
      !smallFinite(static_cast<double>(point.z)) ||
      !smallFinite(static_cast<double>(point.intensity)))
    {
      return false;
    }
  }
  return true;
}

bool parseSmallConfiguration(
  const ParameterMap & parameters,
  const bool voxelized,
  SmallGicpConfiguration * configuration,
  std::string * error)
{
  if (configuration == nullptr) {
    smallSetError(error, "internal error: null small_gicp configuration");
    return false;
  }
  *configuration = SmallGicpConfiguration();
  bool saw_maximum_correspondence_distance = false;
  bool saw_transformation_epsilon = false;
  bool saw_maximum_iterations = false;
  bool saw_num_threads = false;
  bool saw_voxel_resolution = false;
  bool saw_adaptive = false;

  for (const auto & entry : parameters) {
    const std::string & key = entry.first;
    try {
      if (key == "maximum_correspondence_distance") {
        const double value = entry.second.asDouble();
        if (!smallFinite(value) || value <= 0.0) {
          smallSetError(error, smallParameterError(key, "must be finite and greater than zero"));
          return false;
        }
        configuration->maximum_correspondence_distance = value;
        saw_maximum_correspondence_distance = true;
      } else if (key == "transformation_epsilon") {
        const double value = entry.second.asDouble();
        if (!smallFinite(value) || value != kSmallLegacyTransformationEpsilon) {
          smallSetError(error, smallParameterError(key, "must equal the legacy fixed value 1e-6"));
          return false;
        }
        configuration->transformation_epsilon = value;
        saw_transformation_epsilon = true;
      } else if (key == "maximum_iterations") {
        const std::int64_t value = entry.second.asInteger();
        if (value < 1 || value > static_cast<std::int64_t>(std::numeric_limits<int>::max())) {
          smallSetError(error, smallParameterError(key, "must fit a positive int"));
          return false;
        }
        configuration->maximum_iterations = static_cast<int>(value);
        saw_maximum_iterations = true;
      } else if (key == "num_threads") {
        const std::int64_t value = entry.second.asInteger();
        if (value < 0 || value > static_cast<std::int64_t>(std::numeric_limits<int>::max())) {
          smallSetError(error, smallParameterError(key, "must fit a non-negative int"));
          return false;
        }
        configuration->num_threads = static_cast<int>(value);
        saw_num_threads = true;
      } else if (key == "voxel_resolution") {
        const double value = entry.second.asDouble();
        if (!smallFinite(value) || value <= 0.0) {
          smallSetError(error, smallParameterError(key, "must be finite and greater than zero"));
          return false;
        }
        configuration->voxel_resolution = value;
        saw_voxel_resolution = true;
      } else if (key == "adaptive_correspondence_threshold") {
        configuration->adaptive_correspondence_threshold = entry.second.asBool();
        saw_adaptive = true;
      } else {
        smallSetError(error, smallParameterError(key, "unknown key"));
        return false;
      }
    } catch (const std::exception & exception) {
      smallSetError(error, smallParameterError(key, exception.what()));
      return false;
    } catch (...) {
      smallSetError(error, smallParameterError(key, "type conversion failed"));
      return false;
    }
  }

  if (!saw_maximum_correspondence_distance) {
    smallSetError(error, smallParameterError("maximum_correspondence_distance", "is required"));
    return false;
  }
  if (!saw_transformation_epsilon) {
    smallSetError(error, smallParameterError("transformation_epsilon", "is required"));
    return false;
  }
  if (!saw_maximum_iterations) {
    smallSetError(error, smallParameterError("maximum_iterations", "is required"));
    return false;
  }
  if (!saw_num_threads) {
    smallSetError(error, smallParameterError("num_threads", "is required"));
    return false;
  }
  if (voxelized && !saw_voxel_resolution) {
    smallSetError(error, smallParameterError("voxel_resolution", "is required for SMALL_VGICP"));
    return false;
  }
  if (!voxelized && saw_voxel_resolution) {
    smallSetError(error, smallParameterError("voxel_resolution", "is not valid for SMALL_GICP"));
    return false;
  }
  if (!saw_adaptive) {
    smallSetError(error, smallParameterError("adaptive_correspondence_threshold", "is required"));
    return false;
  }
  return true;
}

const char * smallFailureName(const FailureCode failure)
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
    case FailureCode::kInternalError:
      return "internal error";
  }
  return "unknown failure";
}

}  // namespace

struct SmallGicpRegistration::Impl
{
  using SmallGicp = small_gicp::RegistrationPCL<PointT, PointT>;

  explicit Impl(const bool voxelized_in)
  : voxelized(voxelized_in) {}

  SmallGicpConfiguration configuration;
  typename SmallGicp::Ptr registration;
  PointCloudConstPtr target;
  bool voxelized{false};
  bool configured{false};
  bool reset_distance_after_call{false};
};

void SmallGicpRegistration::clearPerCallState(Impl * implementation) noexcept
{
  if (implementation == nullptr || !implementation->registration) {
    return;
  }
  if (implementation->reset_distance_after_call) {
    try {
      // The legacy scanmatcher resets adaptive SMALL_GICP correspondence
      // distance to DBL_MAX after every call, rather than restoring the
      // configured threshold.
      implementation->registration->setMaxCorrespondenceDistance(
        std::numeric_limits<double>::max());
    } catch (...) {
    }
    implementation->reset_distance_after_call = false;
  }
}

struct SmallGicpRegistration::PerCallStateGuard
{
  explicit PerCallStateGuard(Impl * implementation_in)
  : implementation(implementation_in) {}

  ~PerCallStateGuard() noexcept
  {
    SmallGicpRegistration::clearPerCallState(implementation);
  }

  Impl * implementation;
};

SmallGicpRegistration::SmallGicpRegistration()
: SmallGicpRegistration(false) {}

SmallGicpRegistration::SmallGicpRegistration(const bool voxelized)
: impl_(new Impl(voxelized)) {}

SmallGicpRegistration::~SmallGicpRegistration() = default;

bool SmallGicpRegistration::voxelized() const noexcept
{
  return impl_ != nullptr && impl_->voxelized;
}

PluginMetadata SmallGicpRegistration::metadata() const
{
  PluginMetadata metadata;
  metadata.class_id = "lidarslam_default_plugins/SmallGicpPcl";
  metadata.implementation_version = "1.0.0";
  metadata.license = "BSD-2-Clause";
  metadata.api_version = kHostApiVersion;
  return metadata;
}

PluginMetadata SmallVgicpRegistration::metadata() const
{
  PluginMetadata metadata;
  metadata.class_id = "lidarslam_default_plugins/SmallVGicpPcl";
  metadata.implementation_version = "1.0.0";
  metadata.license = "BSD-2-Clause";
  metadata.api_version = kHostApiVersion;
  return metadata;
}

Capabilities SmallGicpRegistration::capabilities() const
{
  Capabilities capabilities;
  capabilities
  .add(Capability::kInitialGuess)
  .add(Capability::kMaximumCorrespondenceDistance)
  .add(Capability::kAlignedSource)
  .setTargetPolicy(TargetPolicy::kAcceptHostPrepared)
  .setCorrespondenceMetric(CorrespondenceMetric::kSquareRootFitnessProxy)
  .setThreadModel(ThreadModel::kSerializedOwner);
  // small_gicp's reduction order is only fixed for the explicit one-thread
  // configuration.  num_threads=0 retains its library default and must not
  // claim determinism.
  if (impl_ != nullptr && impl_->configured && impl_->configuration.num_threads == 1) {
    capabilities.add(Capability::kDeterministic);
  }
  return capabilities;
}

bool SmallGicpRegistration::configure(const ParameterMap & parameters, std::string * error)
{
  if (!impl_) {
    smallSetError(error, "internal error: small_gicp implementation is unavailable");
    return false;
  }
  if (impl_->configured) {
    smallSetError(error, "small_gicp adapter is already configured; call reset() before configure()");
    return false;
  }

  SmallGicpConfiguration configuration;
  if (!parseSmallConfiguration(parameters, impl_->voxelized, &configuration, error)) {
    return false;
  }
  try {
    typename Impl::SmallGicp::Ptr registration(new Impl::SmallGicp());
    // Keep the construction order and values aligned with the legacy branch.
    registration->setRegistrationType(impl_->voxelized ? "VGICP" : "GICP");
    if (impl_->voxelized) {
      registration->setVoxelResolution(configuration.voxel_resolution);
    }
    registration->setMaxCorrespondenceDistance(configuration.maximum_correspondence_distance);
    registration->setTransformationEpsilon(configuration.transformation_epsilon);
    registration->setMaximumIterations(configuration.maximum_iterations);
    if (configuration.num_threads > 0) {
      registration->setNumThreads(configuration.num_threads);
    }
    impl_->configuration = configuration;
    impl_->registration = registration;
    impl_->target.reset();
    impl_->reset_distance_after_call = false;
    impl_->configured = true;
    return true;
  } catch (const std::exception & exception) {
    impl_->registration.reset();
    impl_->target.reset();
    impl_->configured = false;
    smallSetError(error, std::string("failed to configure small_gicp: ") + exception.what());
    return false;
  } catch (...) {
    impl_->registration.reset();
    impl_->target.reset();
    impl_->configured = false;
    smallSetError(error, "failed to configure small_gicp: unknown exception");
    return false;
  }
}

bool SmallGicpRegistration::setInputTarget(const PointCloudConstPtr & target, std::string * error)
{
  if (!impl_ || !impl_->configured || !impl_->registration) {
    smallSetError(error, "small_gicp adapter is not configured");
    return false;
  }
  if (!smallFinitePointCloud(target)) {
    smallSetError(error, "small_gicp target must be non-empty and contain finite PointXYZI fields");
    return false;
  }
  try {
    impl_->registration->setInputTarget(target);
    impl_->target = target;
    return true;
  } catch (const std::exception & exception) {
    smallSetError(error, std::string("failed to set small_gicp target: ") + exception.what());
    return false;
  } catch (...) {
    smallSetError(error, "failed to set small_gicp target: unknown exception");
    return false;
  }
}

AlignmentResult SmallGicpRegistration::align(const AlignmentRequest & request)
{
  AlignmentResult result;
  if (!impl_ || !impl_->configured || !impl_->registration) {
    result.failure = FailureCode::kNotConfigured;
    result.diagnostics.detail = smallFailureName(result.failure);
    return result;
  }
  if (!impl_->target || impl_->target->empty()) {
    result.failure = FailureCode::kInvalidInput;
    result.diagnostics.detail = "small_gicp target has not been set";
    return result;
  }

  const FailureCode request_status = validateRequest(request, capabilities());
  if (request_status != FailureCode::kNone) {
    result.failure = request_status;
    result.diagnostics.detail = smallFailureName(request_status);
    return result;
  }
  if (
    !smallFinitePointCloud(request.source) ||
    (request.initial_guess_enabled && !request.initial_guess.allFinite()))
  {
    result.failure = FailureCode::kInvalidInput;
    result.diagnostics.detail = "source and initial guess must contain finite values";
    return result;
  }
  if (
    request.maximum_correspondence_distance_enabled &&
    (!smallFinite(request.maximum_correspondence_distance) ||
    request.maximum_correspondence_distance <= 0.0))
  {
    result.failure = FailureCode::kInvalidInput;
    result.diagnostics.detail =
      "maximum correspondence distance must be finite and greater than zero";
    return result;
  }

  PerCallStateGuard state_guard(impl_.get());
  try {
    impl_->registration->setInputSource(request.source);
    impl_->reset_distance_after_call =
      impl_->configuration.adaptive_correspondence_threshold;
    if (request.maximum_correspondence_distance_enabled) {
      impl_->registration->setMaxCorrespondenceDistance(request.maximum_correspondence_distance);
      impl_->reset_distance_after_call = true;
    }

    PointCloud aligned;
    if (request.initial_guess_enabled) {
      impl_->registration->align(aligned, request.initial_guess);
    } else {
      impl_->registration->align(aligned);
    }

    result.converged = impl_->registration->hasConverged();
    result.final_transformation = impl_->registration->getFinalTransformation();
    result.fitness_score = impl_->registration->getFitnessScore();
    if (smallFinite(result.fitness_score) && result.fitness_score >= 0.0) {
      result.diagnostics.mean_correspondence_distance_valid = true;
      result.diagnostics.mean_correspondence_distance =
        result.fitness_score > 0.0 ? std::sqrt(result.fitness_score) : 0.0;
    }
    if (
      aligned.empty() || aligned.size() != request.source->size() ||
      !result.final_transformation.allFinite() ||
      !smallFinite(result.fitness_score) ||
      !result.diagnostics.mean_correspondence_distance_valid)
    {
      result.failure = FailureCode::kAlignmentFailed;
      result.diagnostics.detail = "small_gicp returned a non-finite or incomplete alignment";
      return result;
    }
    result.aligned_source.reset(new PointCloud(aligned));
    result.failure = FailureCode::kNone;
    return result;
  } catch (const std::exception & exception) {
    result.failure = FailureCode::kInternalError;
    result.diagnostics.detail = std::string("small_gicp exception: ") + exception.what();
    return result;
  } catch (...) {
    result.failure = FailureCode::kInternalError;
    result.diagnostics.detail = "small_gicp exception: unknown exception";
    return result;
  }
}

void SmallGicpRegistration::reset() noexcept
{
  if (!impl_) {
    return;
  }
  clearPerCallState(impl_.get());
  impl_->target.reset();
  impl_->registration.reset();
  impl_->configuration = SmallGicpConfiguration();
  impl_->configured = false;
}

SmallVgicpRegistration::SmallVgicpRegistration()
: SmallGicpRegistration(true) {}

SmallVgicpRegistration::~SmallVgicpRegistration() = default;

}  // namespace lidarslam_default_plugins

#endif  // LIDARSLAM_DEFAULT_PLUGINS__SMALL_GICP_REGISTRATION_IMPL_IPP_
