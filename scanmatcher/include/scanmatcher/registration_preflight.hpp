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

#ifndef SCANMATCHER__REGISTRATION_PREFLIGHT_HPP_
#define SCANMATCHER__REGISTRATION_PREFLIGHT_HPP_

#include <cstdint>
#include <string>

#include "lidarslam_registration_loader/registration_plugin_loader.hpp"
#include "scanmatcher/registration_config.hpp"

namespace graphslam
{
namespace registration_config
{

// Canonical legacy values used by both the component-owned startup preflight
// and the offline runner receipt.  Keeping this as a ROS-free value object
// prevents the two shells from silently constructing different LoadRequests.
struct RegistrationPreflightParameters
{
  std::string method;
  std::string class_id;
  double ndt_resolution{5.0};
  double ndt_transformation_epsilon{0.01};
  int ndt_max_iterations{35};
  double ndt_step_size{0.1};
  double ndt_outlier_ratio{0.55};
  int ndt_num_threads{0};
  double gicp_maximum_correspondence_distance{5.0};
  bool adaptive_correspondence_threshold{false};
  bool require_rotation_prior{false};
  bool require_translation_prior{false};
};

inline lidarslam::plugins::registration::RegistrationRuntimeDescriptor
makeExpectedHostRegistrationDescriptor(const std::string & class_id)
{
  using namespace lidarslam::plugins::registration;  // NOLINT(build/namespaces)
  PluginMetadata metadata;
  metadata.class_id = class_id;
  metadata.implementation_version = "host-preflight";
  metadata.license = "BSD-2-Clause";
  metadata.api_version = kHostApiVersion;
  Capabilities capabilities;
  std::uint64_t required = 0U;
  std::uint64_t optional = 0U;
  if (class_id.find("NdtOmp") != std::string::npos) {
    required = static_cast<std::uint64_t>(Capability::kInitialGuess) |
      static_cast<std::uint64_t>(Capability::kRotationPrior) |
      static_cast<std::uint64_t>(Capability::kTranslationPrior) |
      static_cast<std::uint64_t>(Capability::kMaximumCorrespondenceDistance) |
      static_cast<std::uint64_t>(Capability::kMeanCorrespondenceDistance) |
      static_cast<std::uint64_t>(Capability::kAlignedSource);
    optional = static_cast<std::uint64_t>(Capability::kDeterministic);
    capabilities.add(Capability::kInitialGuess).add(Capability::kRotationPrior)
      .add(Capability::kTranslationPrior).add(Capability::kMaximumCorrespondenceDistance)
      .add(Capability::kMeanCorrespondenceDistance).add(Capability::kAlignedSource)
      .setTargetPolicy(TargetPolicy::kRequiresRawTarget)
      .setCorrespondenceMetric(CorrespondenceMetric::kMeanDistance)
      .setThreadModel(ThreadModel::kSerializedOwner);
  } else if (class_id.find("Gicp") != std::string::npos ||
    class_id.find("VGicp") != std::string::npos)
  {
    required = static_cast<std::uint64_t>(Capability::kInitialGuess) |
      static_cast<std::uint64_t>(Capability::kMaximumCorrespondenceDistance) |
      static_cast<std::uint64_t>(Capability::kAlignedSource);
    if (class_id.find("Small") != std::string::npos) {
      optional = static_cast<std::uint64_t>(Capability::kDeterministic);
    }
    capabilities.add(Capability::kInitialGuess).add(Capability::kMaximumCorrespondenceDistance)
      .add(Capability::kAlignedSource)
      .setTargetPolicy(TargetPolicy::kAcceptHostPrepared)
      .setCorrespondenceMetric(CorrespondenceMetric::kSquareRootFitnessProxy)
      .setThreadModel(ThreadModel::kSerializedOwner);
  }
  auto descriptor = makeRegistrationRuntimeDescriptor(
    metadata, capabilities, required, optional,
    registrationConfigSchemaForClassId(class_id));
  return descriptor;
}

inline bool makeRegistrationPluginLoadRequest(
  const RegistrationPreflightParameters & values,
  lidarslam::plugins::registration::shell::LoadRequest * request,
  std::string * error)
{
  if (request == nullptr) {
    if (error != nullptr) {
      *error = "registration preflight request output is null";
    }
    return false;
  }
  if (values.class_id.empty()) {
    if (error != nullptr) {
      *error = "registration_plugin_class must be non-empty when registration plugins are enabled";
    }
    return false;
  }
  if (
    values.method != "NDT" && values.method != "GICP" &&
    values.method != "SMALL_GICP" && values.method != "SMALL_VGICP" &&
    values.method != "FAST_GICP" && values.method != "FAST_VGICP")
  {
    if (error != nullptr) {
      *error = "registration plugin preflight does not support registration_method='" +
        values.method + "'; other methods remain legacy or unavailable";
    }
    return false;
  }
  if (values.ndt_max_iterations < 1 || values.ndt_num_threads < 0) {
    if (error != nullptr) {
      *error = "registration plugin integer parameters must be positive iterations and "
               "non-negative num_threads";
    }
    return false;
  }
  // A caller may reuse a request object for several selector attempts.  Reset
  // every field before filling the canonical request so an earlier, stricter
  // capability or license policy cannot leak into a later preflight.
  *request = lidarslam::plugins::registration::shell::LoadRequest{};
  request->class_id = values.class_id;
  request->enforce_permissive_license = true;
  request->capabilities = {};
  request->capabilities.require_initial_guess = true;
  request->capabilities.require_aligned_source = true;
  request->capabilities.require_target_policy = true;

  if (
    registration_config::isFastGicpMethod(values.method) &&
    !registration_config::fastGicpAvailable())
  {
    if (error != nullptr) {
      *error = "registration plugin preflight requested " + values.method +
        " but fast_gicp is unavailable in this build; no fallback is allowed";
    }
    return false;
  }
  if (
    registration_config::isFastGicpMethod(values.method) &&
    !registration_config::isCanonicalFastGicpClassId(values.method, values.class_id))
  {
    if (error != nullptr) {
      *error =
        "FAST registration_method='" + values.method + "' requires canonical class ID '" +
        registration_config::fastGicpHostClassId(values.method) + "' or '" +
        registration_config::fastGicpPluginClassId(values.method) + "'";
    }
    return false;
  }

  if (values.method == "NDT") {
    request->parameters = makeNdtParameterMap(
      values.ndt_resolution,
      values.ndt_transformation_epsilon,
      values.ndt_max_iterations,
      values.ndt_step_size,
      values.ndt_outlier_ratio,
      values.ndt_num_threads);
    request->capabilities.target_policy =
      lidarslam::plugins::registration::TargetPolicy::kRequiresRawTarget;
    request->capabilities.require_mean_correspondence_distance = true;
    request->capabilities.require_correspondence_metric = true;
    request->capabilities.correspondence_metric =
      lidarslam::plugins::registration::CorrespondenceMetric::kMeanDistance;
    request->capabilities.require_maximum_correspondence_distance =
      values.adaptive_correspondence_threshold;
    request->capabilities.require_rotation_prior = values.require_rotation_prior;
    request->capabilities.require_translation_prior = values.require_translation_prior;
  } else if (values.method == "GICP") {
    request->parameters = makeGicpParameterMap(
      values.gicp_maximum_correspondence_distance,
      values.adaptive_correspondence_threshold);
    request->capabilities.target_policy =
      lidarslam::plugins::registration::TargetPolicy::kAcceptHostPrepared;
    request->capabilities.require_correspondence_metric = true;
    request->capabilities.correspondence_metric =
      lidarslam::plugins::registration::CorrespondenceMetric::kSquareRootFitnessProxy;
    request->capabilities.require_maximum_correspondence_distance =
      values.adaptive_correspondence_threshold;
  } else if (values.method == "SMALL_GICP" || values.method == "SMALL_VGICP") {
    const bool voxelized = values.method == "SMALL_VGICP";
    request->parameters = makeSmallGicpParameterMap(
      values.gicp_maximum_correspondence_distance,
      1e-6,
      values.ndt_max_iterations,
      values.ndt_num_threads,
      values.adaptive_correspondence_threshold,
      voxelized,
      values.ndt_resolution);
    request->capabilities.target_policy =
      lidarslam::plugins::registration::TargetPolicy::kAcceptHostPrepared;
    request->capabilities.require_correspondence_metric = true;
    request->capabilities.correspondence_metric =
      lidarslam::plugins::registration::CorrespondenceMetric::kSquareRootFitnessProxy;
    request->capabilities.require_maximum_correspondence_distance =
      values.adaptive_correspondence_threshold;
  } else {
    const bool voxelized = values.method == "FAST_VGICP";
    request->parameters = makeFastGicpParameterMap(
      values.gicp_maximum_correspondence_distance,
      values.ndt_max_iterations,
      values.ndt_num_threads,
      values.adaptive_correspondence_threshold,
      voxelized,
      values.ndt_resolution);
    request->capabilities.target_policy =
      lidarslam::plugins::registration::TargetPolicy::kAcceptHostPrepared;
    request->capabilities.require_correspondence_metric = true;
    request->capabilities.correspondence_metric =
      lidarslam::plugins::registration::CorrespondenceMetric::kSquareRootFitnessProxy;
    request->capabilities.require_maximum_correspondence_distance =
      values.adaptive_correspondence_threshold;
  }
  return true;
}

}  // namespace registration_config
}  // namespace graphslam

#endif  // SCANMATCHER__REGISTRATION_PREFLIGHT_HPP_
