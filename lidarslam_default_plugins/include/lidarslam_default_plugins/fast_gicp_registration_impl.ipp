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

#ifndef LIDARSLAM_DEFAULT_PLUGINS__FAST_GICP_REGISTRATION_IMPL_IPP_
#define LIDARSLAM_DEFAULT_PLUGINS__FAST_GICP_REGISTRATION_IMPL_IPP_

#include "lidarslam_default_plugins/fast_gicp_registration.hpp"

#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <string>
#include <type_traits>

#include <fast_gicp/gicp/fast_gicp.hpp>  // NOLINT(build/include_order)
#include <fast_gicp/gicp/fast_vgicp.hpp>  // NOLINT(build/include_order)

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

constexpr double kFastTransformationEpsilon = 1e-6;

struct FastConfiguration
{
  double maximum_correspondence_distance{5.0};
  double transformation_epsilon{kFastTransformationEpsilon};
  int maximum_iterations{35};
  int num_threads{0};
  bool adaptive_correspondence_threshold{false};
  double voxel_resolution{0.0};
};

void fastSetError(std::string * error, const std::string & message)
{
  if (error != nullptr) {
    *error = message;
  }
}

std::string fastParameterError(const std::string & key, const std::string & detail)
{
  return "invalid FAST_GICP parameter '" + key + "': " + detail;
}

bool fastFinite(const double value)
{
  return std::isfinite(value) != 0;
}

bool fastFinitePointCloud(const PointCloudConstPtr & cloud)
{
  if (!cloud || cloud->empty()) {
    return false;
  }
  for (const PointT & point : cloud->points) {
    if (
      !fastFinite(static_cast<double>(point.x)) ||
      !fastFinite(static_cast<double>(point.y)) ||
      !fastFinite(static_cast<double>(point.z)) ||
      !fastFinite(static_cast<double>(point.intensity)))
    {
      return false;
    }
  }
  return true;
}

bool parseFastConfiguration(
  const ParameterMap & parameters, const bool voxelized,
  FastConfiguration * configuration, std::string * error)
{
  if (configuration == nullptr) {
    fastSetError(error, "internal error: null FAST_GICP configuration");
    return false;
  }
  *configuration = FastConfiguration();
  bool saw_maximum_correspondence_distance = false;
  bool saw_transformation_epsilon = false;
  bool saw_maximum_iterations = false;
  bool saw_num_threads = false;
  bool saw_adaptive_correspondence_threshold = false;
  bool saw_voxel_resolution = !voxelized;

  for (const auto & entry : parameters) {
    const std::string & key = entry.first;
    try {
      if (key == "maximum_correspondence_distance") {
        const double value = entry.second.asDouble();
        if (!fastFinite(value) || value <= 0.0) {
          fastSetError(error, fastParameterError(key, "must be finite and greater than zero"));
          return false;
        }
        configuration->maximum_correspondence_distance = value;
        saw_maximum_correspondence_distance = true;
      } else if (key == "transformation_epsilon") {
        const double value = entry.second.asDouble();
        if (!fastFinite(value) || value != kFastTransformationEpsilon) {
          fastSetError(error, fastParameterError(key, "must equal the legacy fixed value 1e-6"));
          return false;
        }
        configuration->transformation_epsilon = value;
        saw_transformation_epsilon = true;
      } else if (key == "maximum_iterations") {
        const std::int64_t value = entry.second.asInteger();
        if (value < 1 || value > std::numeric_limits<int>::max()) {
          fastSetError(error, fastParameterError(key, "must be a positive 32-bit integer"));
          return false;
        }
        configuration->maximum_iterations = static_cast<int>(value);
        saw_maximum_iterations = true;
      } else if (key == "num_threads") {
        const std::int64_t value = entry.second.asInteger();
        if (value < 0 || value > std::numeric_limits<int>::max()) {
          fastSetError(error, fastParameterError(key, "must be a non-negative 32-bit integer"));
          return false;
        }
        configuration->num_threads = static_cast<int>(value);
        saw_num_threads = true;
      } else if (key == "adaptive_correspondence_threshold") {
        configuration->adaptive_correspondence_threshold = entry.second.asBool();
        saw_adaptive_correspondence_threshold = true;
      } else if (key == "voxel_resolution" && voxelized) {
        const double value = entry.second.asDouble();
        if (!fastFinite(value) || value <= 0.0) {
          fastSetError(error, fastParameterError(key, "must be finite and greater than zero"));
          return false;
        }
        configuration->voxel_resolution = value;
        saw_voxel_resolution = true;
      } else {
        fastSetError(error, fastParameterError(key, "unknown key for selected FAST variant"));
        return false;
      }
    } catch (const std::exception & exception) {
      fastSetError(error, fastParameterError(key, exception.what()));
      return false;
    } catch (...) {
      fastSetError(error, fastParameterError(key, "type conversion failed"));
      return false;
    }
  }

  if (!saw_maximum_correspondence_distance || !saw_transformation_epsilon ||
    !saw_maximum_iterations || !saw_num_threads || !saw_adaptive_correspondence_threshold ||
    !saw_voxel_resolution)
  {
    fastSetError(
      error,
      "maximum distance, epsilon, iterations, num_threads, adaptive flag, and the selected "
      "variant's voxel resolution are required");
    return false;
  }
  return true;
}

template<typename RegistrationT>
struct IsFastVgicp : std::false_type {};

template<>
struct IsFastVgicp<fast_gicp::FastVGICP<PointT, PointT>> : std::true_type {};

template<typename RegistrationT>
void setFastVoxelResolution(RegistrationT &, const double, const std::false_type &)
{
}

template<typename RegistrationT>
void setFastVoxelResolution(
  RegistrationT & registration, const double resolution, const std::true_type &)
{
  registration.setResolution(resolution);
}

template<typename RegistrationT>
struct FastAdapterState
{
  using Registration = RegistrationT;
  using RegistrationPtr = typename RegistrationT::Ptr;

  FastConfiguration configuration;
  RegistrationPtr registration;
  PointCloudConstPtr target;
  bool configured{false};
  bool reset_distance_after_call{false};
};

template<typename State>
void clearFastPerCallState(State * state) noexcept
{
  if (state == nullptr || !state->registration) {
    return;
  }
  if (state->reset_distance_after_call) {
    try {
      state->registration->setMaxCorrespondenceDistance(std::numeric_limits<double>::max());
    } catch (...) {
    }
    state->reset_distance_after_call = false;
  }
}

template<typename State>
bool configureFast(
  State * state, const ParameterMap & parameters, const bool voxelized, std::string * error)
{
  if (state == nullptr) {
    fastSetError(error, "internal error: FAST_GICP state is unavailable");
    return false;
  }
  if (state->configured) {
    fastSetError(error, "FAST_GICP adapter is already configured; call reset() first");
    return false;
  }

  FastConfiguration configuration;
  if (!parseFastConfiguration(parameters, voxelized, &configuration, error)) {
    return false;
  }
  try {
    typename State::RegistrationPtr registration(new typename State::Registration());
    registration->setMaxCorrespondenceDistance(configuration.maximum_correspondence_distance);
    registration->setTransformationEpsilon(configuration.transformation_epsilon);
    registration->setMaximumIterations(configuration.maximum_iterations);
    if (configuration.num_threads > 0) {
      registration->setNumThreads(configuration.num_threads);
    }
    setFastVoxelResolution(
      *registration, configuration.voxel_resolution,
      IsFastVgicp<typename State::Registration>());
    state->configuration = configuration;
    state->registration = registration;
    state->target.reset();
    state->reset_distance_after_call = false;
    state->configured = true;
    return true;
  } catch (const std::exception & exception) {
    state->registration.reset();
    state->target.reset();
    state->configured = false;
    fastSetError(error, std::string("failed to configure FAST_GICP: ") + exception.what());
    return false;
  } catch (...) {
    state->registration.reset();
    state->target.reset();
    state->configured = false;
    fastSetError(error, "failed to configure FAST_GICP: unknown exception");
    return false;
  }
}

template<typename State>
bool setFastInputTarget(
  State * state, const PointCloudConstPtr & target, std::string * error)
{
  if (state == nullptr || !state->configured || !state->registration) {
    fastSetError(error, "FAST_GICP adapter is not configured");
    return false;
  }
  if (!fastFinitePointCloud(target)) {
    fastSetError(error, "FAST_GICP target must be non-empty and finite");
    return false;
  }
  try {
    state->registration->setInputTarget(target);
    state->target = target;
    return true;
  } catch (const std::exception & exception) {
    fastSetError(error, std::string("failed to set FAST_GICP target: ") + exception.what());
    return false;
  } catch (...) {
    fastSetError(error, "failed to set FAST_GICP target: unknown exception");
    return false;
  }
}

const char * fastFailureName(const FailureCode failure)
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

Capabilities fastCapabilities()
{
  Capabilities capabilities;
  capabilities
  .add(Capability::kInitialGuess)
  .add(Capability::kMaximumCorrespondenceDistance)
  .add(Capability::kAlignedSource)
  .setTargetPolicy(TargetPolicy::kAcceptHostPrepared)
  .setCorrespondenceMetric(CorrespondenceMetric::kSquareRootFitnessProxy)
  .setThreadModel(ThreadModel::kSerializedOwner);
  // Determinism is intentionally not advertised until a dependency-specific
  // replay gate proves the fast_gicp reduction order for the selected build.
  return capabilities;
}

template<typename State>
AlignmentResult alignFast(State * state, const AlignmentRequest & request)
{
  AlignmentResult result;
  if (state == nullptr || !state->configured || !state->registration) {
    result.failure = FailureCode::kNotConfigured;
    result.diagnostics.detail = fastFailureName(result.failure);
    return result;
  }
  if (!state->target || state->target->empty()) {
    result.failure = FailureCode::kInvalidInput;
    result.diagnostics.detail = "FAST_GICP target has not been set";
    return result;
  }
  const FailureCode request_status = validateRequest(request, fastCapabilities());
  if (request_status != FailureCode::kNone) {
    result.failure = request_status;
    result.diagnostics.detail = fastFailureName(request_status);
    return result;
  }
  if (
    !fastFinitePointCloud(request.source) ||
    (request.initial_guess_enabled && !request.initial_guess.allFinite()))
  {
    result.failure = FailureCode::kInvalidInput;
    result.diagnostics.detail = "FAST_GICP source or initial guess must be finite";
    return result;
  }
  if (
    request.maximum_correspondence_distance_enabled &&
    (!fastFinite(request.maximum_correspondence_distance) ||
    request.maximum_correspondence_distance <= 0.0))
  {
    result.failure = FailureCode::kInvalidInput;
    result.diagnostics.detail = "FAST_GICP maximum correspondence distance is invalid";
    return result;
  }

  try {
    clearFastPerCallState(state);
    state->registration->setInputSource(request.source);
    state->reset_distance_after_call =
      state->configuration.adaptive_correspondence_threshold;
    if (request.maximum_correspondence_distance_enabled) {
      state->registration->setMaxCorrespondenceDistance(
        request.maximum_correspondence_distance);
      state->reset_distance_after_call = true;
    }

    PointCloud aligned;
    if (request.initial_guess_enabled) {
      state->registration->align(aligned, request.initial_guess);
    } else {
      state->registration->align(aligned);
    }
    result.converged = state->registration->hasConverged();
    result.final_transformation = state->registration->getFinalTransformation();
    result.fitness_score = state->registration->getFitnessScore();
    if (fastFinite(result.fitness_score) && result.fitness_score >= 0.0) {
      result.diagnostics.mean_correspondence_distance_valid = true;
      result.diagnostics.mean_correspondence_distance =
        result.fitness_score > 0.0 ? std::sqrt(result.fitness_score) : 0.0;
    }
    if (
      aligned.empty() || aligned.size() != request.source->size() ||
      !result.final_transformation.allFinite() ||
      !fastFinite(result.fitness_score) ||
      !result.diagnostics.mean_correspondence_distance_valid)
    {
      result.failure = FailureCode::kAlignmentFailed;
      result.diagnostics.detail = "fast_gicp returned a non-finite or incomplete alignment";
      return result;
    }
    result.aligned_source.reset(new PointCloud(aligned));
    result.failure = FailureCode::kNone;
    return result;
  } catch (const std::exception & exception) {
    result.failure = FailureCode::kInternalError;
    result.diagnostics.detail = std::string("FAST_GICP exception: ") + exception.what();
    return result;
  } catch (...) {
    result.failure = FailureCode::kInternalError;
    result.diagnostics.detail = "FAST_GICP exception: unknown exception";
    return result;
  }
}

template<typename State>
void resetFast(State * state) noexcept
{
  if (state == nullptr) {
    return;
  }
  clearFastPerCallState(state);
  state->target.reset();
  state->registration.reset();
  state->configuration = FastConfiguration();
  state->configured = false;
}

}  // namespace

struct FastGicpRegistration::Impl
  : FastAdapterState<fast_gicp::FastGICP<PointT, PointT>> {};

struct FastVgicpRegistration::Impl
  : FastAdapterState<fast_gicp::FastVGICP<PointT, PointT>> {};

struct FastGicpRegistration::PerCallStateGuard
{
  explicit PerCallStateGuard(Impl * implementation)
  : implementation(implementation) {}

  ~PerCallStateGuard() noexcept
  {
    FastGicpRegistration::clearPerCallState(implementation);
  }

  Impl * implementation;
};

struct FastVgicpRegistration::PerCallStateGuard
{
  explicit PerCallStateGuard(Impl * implementation)
  : implementation(implementation) {}

  ~PerCallStateGuard() noexcept
  {
    FastVgicpRegistration::clearPerCallState(implementation);
  }

  Impl * implementation;
};

FastGicpRegistration::FastGicpRegistration()
: impl_(new Impl()) {}

FastGicpRegistration::~FastGicpRegistration() = default;

PluginMetadata FastGicpRegistration::metadata() const
{
  PluginMetadata metadata;
  metadata.class_id = "lidarslam_default_plugins/FastGicp";
  metadata.implementation_version = "1.0.0";
  metadata.license = "BSD-2-Clause";
  metadata.api_version = kHostApiVersion;
  return metadata;
}

RegistrationRuntimeDescriptor FastGicpRegistration::registrationDescriptor() const
{
  const std::uint64_t required =
    static_cast<std::uint64_t>(Capability::kInitialGuess) |
    static_cast<std::uint64_t>(Capability::kMaximumCorrespondenceDistance) |
    static_cast<std::uint64_t>(Capability::kAlignedSource);
  return makeRegistrationRuntimeDescriptor(
    metadata(), capabilities(), required, 0U,
    registration::registrationConfigSchemaForClassId(metadata().class_id));
}

Capabilities FastGicpRegistration::capabilities() const
{
  return fastCapabilities();
}

bool FastGicpRegistration::configure(const ParameterMap & parameters, std::string * error)
{
  return configureFast(impl_.get(), parameters, false, error);
}

bool FastGicpRegistration::setInputTarget(
  const PointCloudConstPtr & target, std::string * error)
{
  return setFastInputTarget(impl_.get(), target, error);
}

AlignmentResult FastGicpRegistration::align(const AlignmentRequest & request)
{
  PerCallStateGuard state_guard(impl_.get());
  return alignFast(impl_.get(), request);
}

void FastGicpRegistration::clearPerCallState(Impl * implementation) noexcept
{
  clearFastPerCallState(implementation);
}

void FastGicpRegistration::reset() noexcept
{
  resetFast(impl_.get());
}

FastVgicpRegistration::FastVgicpRegistration()
: impl_(new Impl()) {}

FastVgicpRegistration::~FastVgicpRegistration() = default;

PluginMetadata FastVgicpRegistration::metadata() const
{
  PluginMetadata metadata;
  metadata.class_id = "lidarslam_default_plugins/FastVGicp";
  metadata.implementation_version = "1.0.0";
  metadata.license = "BSD-2-Clause";
  metadata.api_version = kHostApiVersion;
  return metadata;
}

RegistrationRuntimeDescriptor FastVgicpRegistration::registrationDescriptor() const
{
  const std::uint64_t required =
    static_cast<std::uint64_t>(Capability::kInitialGuess) |
    static_cast<std::uint64_t>(Capability::kMaximumCorrespondenceDistance) |
    static_cast<std::uint64_t>(Capability::kAlignedSource);
  return makeRegistrationRuntimeDescriptor(
    metadata(), capabilities(), required, 0U,
    registration::registrationConfigSchemaForClassId(metadata().class_id));
}

Capabilities FastVgicpRegistration::capabilities() const
{
  return fastCapabilities();
}

bool FastVgicpRegistration::configure(const ParameterMap & parameters, std::string * error)
{
  return configureFast(impl_.get(), parameters, true, error);
}

bool FastVgicpRegistration::setInputTarget(
  const PointCloudConstPtr & target, std::string * error)
{
  return setFastInputTarget(impl_.get(), target, error);
}

AlignmentResult FastVgicpRegistration::align(const AlignmentRequest & request)
{
  PerCallStateGuard state_guard(impl_.get());
  return alignFast(impl_.get(), request);
}

void FastVgicpRegistration::clearPerCallState(Impl * implementation) noexcept
{
  clearFastPerCallState(implementation);
}

void FastVgicpRegistration::reset() noexcept
{
  resetFast(impl_.get());
}

}  // namespace lidarslam_default_plugins

#endif  // LIDARSLAM_DEFAULT_PLUGINS__FAST_GICP_REGISTRATION_IMPL_IPP_
