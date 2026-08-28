// Copyright 2026 Sasaki
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
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

#ifndef GRAPH_BASED_SLAM__BACKEND_REGISTRATION_PREFLIGHT_HPP_
#define GRAPH_BASED_SLAM__BACKEND_REGISTRATION_PREFLIGHT_HPP_

#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <ostream>
#include <sstream>
#include <string>
#include <utility>

#include "lidarslam_registration_loader/registration_plugin_loader.hpp"

namespace graphslam
{
namespace backend_registration
{

// The backend and scanmatcher have separate role profiles even when they use
// the same concrete adapter.  Keep this role in the shell-side request and
// every receipt so a frontend configuration cannot be mistaken for a loop
// closure configuration.
constexpr const char * kBackendRegistrationRole = "backend_loop";
// Backend loop closure opts into a small, deterministic per-instance NDT
// target-cell cache.  Frontend and external-plugin requests omit this field
// (the adapter default is zero), so their historical behavior is unchanged.
constexpr int kBackendNdtTargetCellCacheCapacity = 3;

// These values preserve the historical backend NDT construction/defaults. The
// adapter receives every typed field explicitly so a future default change
// cannot silently alter the backend loop-closure contract.
struct NdtConfig
{
  double resolution{5.0};
  double transformation_epsilon{0.01};
  int maximum_iterations{100};
  double step_size{0.1};
  double outlier_ratio{0.55};
  int num_threads{0};
  int target_cell_cache_capacity{kBackendNdtTargetCellCacheCapacity};
};

struct BackendRegistrationRequest
{
  std::string role{kBackendRegistrationRole};
  lidarslam::plugins::registration::shell::LoadRequest request;
};

inline lidarslam::plugins::registration::shell::HostBuiltinRegistration
makeNdtHostBuiltinRegistration(
  lidarslam::plugins::registration::shell::HostBuiltinFactory factory)
{
  lidarslam::plugins::registration::shell::HostBuiltinRegistration registration;
  registration.class_id = "lidarslam_builtin/NdtOmp";
  registration.metadata_class_id = "lidarslam_default_plugins/NdtOmp";
  registration.factory = std::move(factory);
  registration.expected_descriptor_factory = []() {
      using namespace lidarslam::plugins::registration;  // NOLINT(build/namespaces)
      PluginMetadata metadata;
      metadata.class_id = "lidarslam_builtin/NdtOmp";
      metadata.implementation_version = "host-preflight";
      metadata.license = "BSD-2-Clause";
      metadata.api_version = kHostApiVersion;
      Capabilities capabilities;
      capabilities.add(Capability::kInitialGuess).add(Capability::kRotationPrior)
      .add(Capability::kTranslationPrior).add(Capability::kMaximumCorrespondenceDistance)
      .add(Capability::kMeanCorrespondenceDistance).add(Capability::kAlignedSource)
      .setTargetPolicy(TargetPolicy::kRequiresRawTarget)
      .setCorrespondenceMetric(CorrespondenceMetric::kMeanDistance)
      .setThreadModel(ThreadModel::kSerializedOwner);
      return makeRegistrationRuntimeDescriptor(
        metadata, capabilities,
        static_cast<std::uint64_t>(Capability::kInitialGuess) |
        static_cast<std::uint64_t>(Capability::kRotationPrior) |
        static_cast<std::uint64_t>(Capability::kTranslationPrior) |
        static_cast<std::uint64_t>(Capability::kMaximumCorrespondenceDistance) |
        static_cast<std::uint64_t>(Capability::kMeanCorrespondenceDistance) |
        static_cast<std::uint64_t>(Capability::kAlignedSource),
        static_cast<std::uint64_t>(Capability::kDeterministic),
        registrationConfigSchemaForClassId(metadata.class_id));
    };
  return registration;
}

inline bool makeNdtLoadRequest(
  const NdtConfig & config,
  lidarslam::plugins::registration::shell::LoadRequest * request,
  std::string * error)
{
  if (request == nullptr) {
    if (error != nullptr) {
      *error = "backend NDT preflight output request is null";
    }
    return false;
  }
  if (
    !std::isfinite(config.resolution) || config.resolution <= 0.0 ||
    !std::isfinite(config.transformation_epsilon) || config.transformation_epsilon <= 0.0 ||
    config.maximum_iterations < 1 ||
    !std::isfinite(config.step_size) || config.step_size <= 0.0 ||
    !std::isfinite(config.outlier_ratio) || config.outlier_ratio <= 0.0 ||
    config.outlier_ratio >= 1.0 || config.num_threads < 0 ||
    config.target_cell_cache_capacity < 0)
  {
    if (error != nullptr) {
      *error =
        "backend NDT configuration is invalid; expected positive finite resolution, epsilon, "
        "step_size, iterations, outlier_ratio in (0,1), and non-negative threads/cache capacity";
    }
    return false;
  }

  using namespace lidarslam::plugins::registration;  // NOLINT(build/namespaces)
  using shell::CapabilityRequirements;
  using shell::LoadRequest;
  *request = LoadRequest{};
  request->class_id = "lidarslam_builtin/NdtOmp";
  request->enforce_permissive_license = true;
  request->parameters.emplace("resolution", ParameterValue(config.resolution));
  request->parameters.emplace(
    "transformation_epsilon", ParameterValue(config.transformation_epsilon));
  request->parameters.emplace(
    "maximum_iterations",
    ParameterValue(static_cast<std::int64_t>(config.maximum_iterations)));
  request->parameters.emplace("step_size", ParameterValue(config.step_size));
  request->parameters.emplace("outlier_ratio", ParameterValue(config.outlier_ratio));
  request->parameters.emplace(
    "num_threads", ParameterValue(static_cast<std::int64_t>(config.num_threads)));
  request->parameters.emplace(
    "target_cell_cache_capacity",
    ParameterValue(static_cast<std::int64_t>(config.target_cell_cache_capacity)));
  request->parameters.emplace("neighborhood_search_method", ParameterValue("DIRECT7"));

  CapabilityRequirements & requirements = request->capabilities;
  requirements.require_initial_guess = true;
  requirements.require_aligned_source = true;
  requirements.require_mean_correspondence_distance = true;
  requirements.require_target_policy = true;
  requirements.target_policy = TargetPolicy::kRequiresRawTarget;
  requirements.require_correspondence_metric = true;
  requirements.correspondence_metric = CorrespondenceMetric::kMeanDistance;
  return true;
}

// Canonical overload used by both the live component and the offline shell.
// The legacy LoadRequest overload above remains for small ROS-free tests and
// callers which do not need a role-bearing receipt.
inline bool makeNdtLoadRequest(
  const NdtConfig & config,
  BackendRegistrationRequest * request,
  std::string * error)
{
  if (request == nullptr) {
    if (error != nullptr) {
      *error = "backend NDT preflight output request is null";
    }
    return false;
  }
  request->role = kBackendRegistrationRole;
  return makeNdtLoadRequest(config, &request->request, error);
}

inline const char * parameterTypeName(
  const lidarslam::plugins::registration::ParameterValue::Type type)
{
  using Type = lidarslam::plugins::registration::ParameterValue::Type;
  switch (type) {
    case Type::kBool:
      return "bool";
    case Type::kInteger:
      return "integer";
    case Type::kDouble:
      return "double";
    case Type::kString:
      return "string";
  }
  return "unknown";
}

inline std::string parameterValueString(
  const lidarslam::plugins::registration::ParameterValue & value)
{
  using Type = lidarslam::plugins::registration::ParameterValue::Type;
  std::ostringstream stream;
  stream << std::setprecision(17);
  switch (value.type()) {
    case Type::kBool:
      return value.asBool() ? "true" : "false";
    case Type::kInteger:
      stream << value.asInteger();
      return stream.str();
    case Type::kDouble:
      stream << value.asDouble();
      return stream.str();
    case Type::kString:
      return value.asString();
  }
  return "<unknown>";
}

inline const char * targetPolicyName(
  const lidarslam::plugins::registration::TargetPolicy policy)
{
  using Policy = lidarslam::plugins::registration::TargetPolicy;
  switch (policy) {
    case Policy::kAcceptHostPrepared:
      return "host_prepared";
    case Policy::kRequiresRawTarget:
      return "raw_target";
    case Policy::kPluginPreprocessesTarget:
      return "plugin_preprocesses";
  }
  return "unknown";
}

inline const char * correspondenceMetricName(
  const lidarslam::plugins::registration::CorrespondenceMetric metric)
{
  using Metric = lidarslam::plugins::registration::CorrespondenceMetric;
  switch (metric) {
    case Metric::kUnavailable:
      return "unavailable";
    case Metric::kMeanDistance:
      return "mean_distance";
    case Metric::kSquareRootFitnessProxy:
      return "sqrt_fitness_proxy";
  }
  return "unknown";
}

inline const char * threadModelName(
  const lidarslam::plugins::registration::ThreadModel model)
{
  using Model = lidarslam::plugins::registration::ThreadModel;
  switch (model) {
    case Model::kSerializedOwner:
      return "serialized_owner";
    case Model::kReentrant:
      return "reentrant";
  }
  return "unknown";
}

// Path-independent identity used by R4 tests.  ParameterMap is a std::map,
// so iteration order is canonical and does not depend on YAML or insertion
// order.  Install-specific library/manifest paths are deliberately excluded.
inline std::string canonicalBackendRegistrationIdentity(
  const BackendRegistrationRequest & request,
  const lidarslam::plugins::registration::shell::RegistrationPluginSession & session)
{
  const auto & metadata = session.metadata();
  const auto & capabilities = session.capabilities();
  std::ostringstream stream;
  stream << "role=" << request.role << "\n";
  stream << "class_id=" << session.classId() << "\n";
  stream << "metadata_class_id=" << metadata.class_id << "\n";
  stream << "implementation_version=" << metadata.implementation_version << "\n";
  stream << "license=" << metadata.license << "\n";
  stream << "api=" << metadata.api_version.major << "." << metadata.api_version.minor << "\n";
  stream << "capabilities_bits=" << capabilities.bits() << "\n";
  stream << "target_policy=" << targetPolicyName(capabilities.targetPolicy()) << "\n";
  stream << "correspondence_metric=" <<
    correspondenceMetricName(capabilities.correspondenceMetric()) << "\n";
  stream << "thread_model=" << threadModelName(capabilities.threadModel()) << "\n";
  for (const auto & entry : session.parameters()) {
    stream << "parameter." << entry.first << ":" << parameterTypeName(entry.second.type()) <<
      "=" << parameterValueString(entry.second) << "\n";
  }
  return stream.str();
}

// Machine-readable, deterministic receipt writer shared by the offline
// runner and its R4 fixture.  String values are quoted so paths and metadata
// remain valid YAML even when an install prefix contains punctuation.
inline bool writeBackendRegistrationReceipt(
  std::ostream & output,
  const BackendRegistrationRequest & request,
  const lidarslam::plugins::registration::shell::RegistrationPluginSession & session,
  std::string * error)
{
  if (!output.good()) {
    if (error != nullptr) {
      *error = "backend registration receipt stream is not writable";
    }
    return false;
  }
  const auto & metadata = session.metadata();
  const auto & capabilities = session.capabilities();
  const auto & load_request = request.request;
  output << "schema: 1\n";
  output << "role: " << std::quoted(request.role) << "\n";
  output << "backend_kind: " << std::quoted(
    lidarslam::plugins::registration::shell::backendKindName(session.backendKind())) << "\n";
  output << "requested_class: " << std::quoted(load_request.class_id) << "\n";
  output << "resolved_class: " << std::quoted(session.classId()) << "\n";
  output << "metadata_class_id: " << std::quoted(metadata.class_id) << "\n";
  output << "implementation_version: " << std::quoted(metadata.implementation_version) << "\n";
  output << "license: " << std::quoted(metadata.license) << "\n";
  output << "api_major: " << metadata.api_version.major << "\n";
  output << "api_minor: " << metadata.api_version.minor << "\n";
  output << "capabilities_bits: " << capabilities.bits() << "\n";
  output << "target_policy: " << std::quoted(targetPolicyName(capabilities.targetPolicy())) << "\n";
  output << "correspondence_metric: " << std::quoted(
    correspondenceMetricName(capabilities.correspondenceMetric())) << "\n";
  output << "thread_model: " << std::quoted(threadModelName(capabilities.threadModel())) << "\n";
  output << "library_path: " << std::quoted(session.libraryPath()) << "\n";
  output << "plugin_manifest_path: " << std::quoted(session.pluginManifestPath()) << "\n";
  output << "requirements:\n";
  output << "  initial_guess: " <<
    (load_request.capabilities.require_initial_guess ? "true" : "false") << "\n";
  output << "  rotation_prior: " <<
    (load_request.capabilities.require_rotation_prior ? "true" : "false") << "\n";
  output << "  translation_prior: " <<
    (load_request.capabilities.require_translation_prior ? "true" : "false") << "\n";
  output << "  maximum_correspondence_distance: " <<
    (load_request.capabilities.require_maximum_correspondence_distance ? "true" : "false") << "\n";
  output << "  mean_correspondence_distance: " <<
    (load_request.capabilities.require_mean_correspondence_distance ? "true" : "false") << "\n";
  output << "  aligned_source: " <<
    (load_request.capabilities.require_aligned_source ? "true" : "false") << "\n";
  output << "  deterministic: " <<
    (load_request.capabilities.require_deterministic ? "true" : "false") << "\n";
  output << "parameters:\n";
  for (const auto & entry : session.parameters()) {
    output << "  " << std::quoted(entry.first) << ":\n";
    output << "    type: " << std::quoted(parameterTypeName(entry.second.type())) << "\n";
    output << "    value: " << std::quoted(parameterValueString(entry.second)) << "\n";
  }
  if (!output.good()) {
    if (error != nullptr) {
      *error = "failed while writing backend registration receipt";
    }
    return false;
  }
  return true;
}

}  // namespace backend_registration
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__BACKEND_REGISTRATION_PREFLIGHT_HPP_
