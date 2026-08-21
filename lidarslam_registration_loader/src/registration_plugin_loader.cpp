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

#include "lidarslam_registration_loader/registration_plugin_loader.hpp"

#include <algorithm>
#include <exception>
#include <sstream>

#include <pluginlib/exceptions.hpp>

namespace lidarslam
{
namespace plugins
{
namespace registration
{
namespace shell
{
namespace
{

LoadFailure makeFailure(LoadFailureCode code, const std::string & message)
{
  LoadFailure failure;
  failure.code = code;
  failure.message = message;
  return failure;
}

std::string describeClasses(std::vector<std::string> classes)
{
  std::sort(classes.begin(), classes.end());
  classes.erase(std::unique(classes.begin(), classes.end()), classes.end());
  if (classes.empty()) {
    return "<none>";
  }
  std::ostringstream stream;
  for (std::size_t index = 0; index < classes.size(); ++index) {
    if (index != 0U) {
      stream << ", ";
    }
    stream << classes[index];
  }
  return stream.str();
}

bool isPermissiveLicense(const std::string & license)
{
  return license == "Apache-2.0" || license == "BSD-2-Clause" ||
         license == "BSD-3-Clause" || license == "ISC" || license == "MIT" ||
         license == "Zlib";
}

LoadFailure validateMetadata(
  const std::string & requested_class_id, const PluginMetadata & metadata)
{
  if (metadata.class_id.empty()) {
    return makeFailure(
      LoadFailureCode::kMetadataInvalid,
      "registration plugin '" + requested_class_id + "' returned empty metadata.class_id");
  }
  if (metadata.class_id != requested_class_id) {
    return makeFailure(
      LoadFailureCode::kMetadataInvalid,
      "registration plugin metadata.class_id='" + metadata.class_id +
      "' does not match requested class '" + requested_class_id + "'");
  }
  if (metadata.implementation_version.empty()) {
    return makeFailure(
      LoadFailureCode::kMetadataInvalid,
      "registration plugin '" + requested_class_id +
      "' returned empty metadata.implementation_version");
  }
  if (metadata.license.empty()) {
    return makeFailure(
      LoadFailureCode::kMetadataInvalid,
      "registration plugin '" + requested_class_id + "' returned empty metadata.license");
  }
  return LoadFailure();
}

LoadFailure validateCapabilities(
  const std::string & class_id, const Capabilities & capabilities,
  const CapabilityRequirements & requirements)
{
  const auto missing = [class_id](const std::string & name) {
      return makeFailure(
        LoadFailureCode::kCapabilityMismatch,
        "registration plugin '" + class_id + "' is missing required capability '" + name + "'");
    };
  if (requirements.require_initial_guess && !capabilities.has(Capability::kInitialGuess)) {
    return missing("initial_guess");
  }
  if (requirements.require_rotation_prior && !capabilities.has(Capability::kRotationPrior)) {
    return missing("rotation_prior");
  }
  if (requirements.require_translation_prior && !capabilities.has(Capability::kTranslationPrior)) {
    return missing("translation_prior");
  }
  if (
    requirements.require_maximum_correspondence_distance &&
    !capabilities.has(Capability::kMaximumCorrespondenceDistance))
  {
    return missing("maximum_correspondence_distance");
  }
  if (
    requirements.require_mean_correspondence_distance &&
    !capabilities.has(Capability::kMeanCorrespondenceDistance))
  {
    return missing("mean_correspondence_distance");
  }
  if (requirements.require_aligned_source && !capabilities.has(Capability::kAlignedSource)) {
    return missing("aligned_source");
  }
  if (requirements.require_deterministic && !capabilities.has(Capability::kDeterministic)) {
    return missing("deterministic");
  }
  if (
    requirements.require_target_policy &&
    capabilities.targetPolicy() != requirements.target_policy)
  {
    return makeFailure(
      LoadFailureCode::kCapabilityMismatch,
      "registration plugin '" + class_id + "' has target policy " +
      std::to_string(static_cast<int>(capabilities.targetPolicy())) +
      ", requested " + std::to_string(static_cast<int>(requirements.target_policy)));
  }
  if (
    requirements.require_correspondence_metric &&
    capabilities.correspondenceMetric() != requirements.correspondence_metric)
  {
    return makeFailure(
      LoadFailureCode::kCapabilityMismatch,
      "registration plugin '" + class_id +
      "' does not provide the requested correspondence metric");
  }
  return LoadFailure();
}

}  // namespace

RegistrationPluginLoader::RegistrationPluginLoader(const std::string & interface_package)
{
  if (interface_package.empty()) {
    initialization_error_ = "registration plugin loader base package is empty";
    return;
  }
  try {
    class_loader_.reset(new RegistrationPluginClassLoader(
      interface_package, kRegistrationPluginBaseClass));
  } catch (const pluginlib::PluginlibException & exception) {
    initialization_error_ =
      "failed to initialize registration plugin loader for package '" + interface_package +
      "': " + exception.what();
  } catch (const std::exception & exception) {
    initialization_error_ =
      "failed to initialize registration plugin loader for package '" + interface_package +
      "': " + exception.what();
  } catch (...) {
    initialization_error_ =
      "failed to initialize registration plugin loader for package '" + interface_package +
      "': unknown exception";
  }
}

std::vector<std::string> RegistrationPluginLoader::availableClasses() const
{
  if (!class_loader_) {
    return {};
  }
  try {
    std::vector<std::string> classes = class_loader_->getDeclaredClasses();
    std::sort(classes.begin(), classes.end());
    return classes;
  } catch (...) {
    return {};
  }
}

LoadResult RegistrationPluginLoader::load(const LoadRequest & request) const
{
  LoadResult result;
  if (!class_loader_) {
    result.failure = makeFailure(
      LoadFailureCode::kLoaderUnavailable,
      initialization_error_.empty() ?
      "registration plugin loader is unavailable" : initialization_error_);
    return result;
  }
  if (request.class_id.empty()) {
    result.failure = makeFailure(
      LoadFailureCode::kInvalidRequest,
      "registration plugin class ID is empty; select an explicit class and no fallback is applied");
    return result;
  }
  if (isHostBuiltinClassId(request.class_id)) {
    result.failure = makeFailure(
      LoadFailureCode::kNamespaceViolation,
      "reserved host-built-in registration class '" + request.class_id +
      "' cannot be resolved through pluginlib");
    return result;
  }

  std::vector<std::string> classes;
  try {
    classes = class_loader_->getDeclaredClasses();
  } catch (const pluginlib::PluginlibException & exception) {
    result.failure = makeFailure(
      LoadFailureCode::kManifestError,
      "failed to inspect registration plugin manifests for requested class '" +
      request.class_id + "': " + exception.what());
    return result;
  } catch (const std::exception & exception) {
    result.failure = makeFailure(
      LoadFailureCode::kManifestError,
      "failed to inspect registration plugin manifests for requested class '" +
      request.class_id + "': " + exception.what());
    return result;
  } catch (...) {
    result.failure = makeFailure(
      LoadFailureCode::kManifestError,
      "failed to inspect registration plugin manifests for requested class '" +
      request.class_id + "': unknown exception");
    return result;
  }

  if (std::find(classes.begin(), classes.end(), request.class_id) == classes.end()) {
    result.failure = makeFailure(
      LoadFailureCode::kUnknownClass,
      "unknown registration plugin class '" + request.class_id + "'; available classes: " +
      describeClasses(classes));
    return result;
  }

  std::shared_ptr<RegistrationPlugin> plugin;
  try {
    plugin = class_loader_->createSharedInstance(request.class_id);
  } catch (const pluginlib::LibraryLoadException & exception) {
    result.failure = makeFailure(
      LoadFailureCode::kLibraryLoad,
      "failed to load registration plugin library for '" + request.class_id + "': " +
      exception.what());
    return result;
  } catch (const pluginlib::CreateClassException & exception) {
    result.failure = makeFailure(
      LoadFailureCode::kCreateInstance,
      "failed to create registration plugin '" + request.class_id + "': " + exception.what());
    return result;
  } catch (const pluginlib::PluginlibException & exception) {
    result.failure = makeFailure(
      LoadFailureCode::kCreateInstance,
      "pluginlib failed to create registration plugin '" + request.class_id + "': " +
      exception.what());
    return result;
  } catch (const std::exception & exception) {
    result.failure = makeFailure(
      LoadFailureCode::kPluginException,
      "registration plugin '" + request.class_id + "' threw during construction: " +
      exception.what());
    return result;
  } catch (...) {
    result.failure = makeFailure(
      LoadFailureCode::kPluginException,
      "registration plugin '" + request.class_id +
      "' threw an unknown exception during construction");
    return result;
  }
  if (!plugin) {
    result.failure = makeFailure(
      LoadFailureCode::kCreateInstance,
      "pluginlib returned a null registration plugin for '" + request.class_id + "'");
    return result;
  }

  PluginMetadata metadata;
  Capabilities capabilities;
  try {
    metadata = plugin->metadata();
    capabilities = plugin->capabilities();
  } catch (const std::exception & exception) {
    result.failure = makeFailure(
      LoadFailureCode::kPluginException,
      "registration plugin '" + request.class_id +
      "' threw while reporting metadata/capabilities: " +
      exception.what());
    return result;
  } catch (...) {
    result.failure = makeFailure(
      LoadFailureCode::kPluginException,
      "registration plugin '" + request.class_id +
      "' threw an unknown exception while reporting metadata/capabilities");
    return result;
  }

  result.failure = validateMetadata(request.class_id, metadata);
  if (!result.failure.ok()) {
    return result;
  }
  if (metadata.api_version.major != kHostApiVersion.major ||
    metadata.api_version.minor > kHostApiVersion.minor)
  {
    result.failure = makeFailure(
      LoadFailureCode::kApiMismatch,
      "registration plugin '" + request.class_id + "' API " +
      std::to_string(metadata.api_version.major) + "." +
      std::to_string(metadata.api_version.minor) +
      " is incompatible with host API " + std::to_string(kHostApiVersion.major) + "." +
      std::to_string(kHostApiVersion.minor) +
      " (exact major; plugin minor must not be newer)");
    return result;
  }
  if (request.enforce_permissive_license && !isPermissiveLicense(metadata.license)) {
    result.failure = makeFailure(
      LoadFailureCode::kMetadataInvalid,
      "registration plugin '" + request.class_id + "' license '" + metadata.license +
      "' is not allowed by the shell permissive-license policy");
    return result;
  }

  std::string configuration_error;
  bool configured = false;
  try {
    configured = plugin->configure(request.parameters, &configuration_error);
  } catch (const std::exception & exception) {
    result.failure = makeFailure(
      LoadFailureCode::kPluginException,
      "registration plugin '" + request.class_id + "' threw during configure(): " +
      exception.what());
    return result;
  } catch (...) {
    result.failure = makeFailure(
      LoadFailureCode::kPluginException,
      "registration plugin '" + request.class_id +
      "' threw an unknown exception during configure()");
    return result;
  }
  if (!configured) {
    if (configuration_error.empty()) {
      configuration_error = "plugin returned false without a diagnostic";
    }
    result.failure = makeFailure(
      LoadFailureCode::kConfigurationFailure,
      "registration plugin '" + request.class_id + "' rejected configuration: " +
      configuration_error);
    return result;
  }

  // Some capabilities are configuration-dependent.  In particular, the
  // built-in NDT adapter only advertises deterministic execution after its
  // fixed thread count has been accepted.  Re-query after configure() and
  // validate the negotiated contract before returning the session.
  try {
    capabilities = plugin->capabilities();
  } catch (const std::exception & exception) {
    result.failure = makeFailure(
      LoadFailureCode::kPluginException,
      "registration plugin '" + request.class_id +
      "' threw while reporting configured capabilities: " + exception.what());
    return result;
  } catch (...) {
    result.failure = makeFailure(
      LoadFailureCode::kPluginException,
      "registration plugin '" + request.class_id +
      "' threw an unknown exception while reporting configured capabilities");
    return result;
  }
  result.failure = validateCapabilities(request.class_id, capabilities, request.capabilities);
  if (!result.failure.ok()) {
    return result;
  }

  std::string library_path;
  std::string plugin_manifest_path;
  try {
    library_path = class_loader_->getClassLibraryPath(request.class_id);
    plugin_manifest_path = class_loader_->getPluginManifestPath(request.class_id);
  } catch (const std::exception & exception) {
    result.failure = makeFailure(
      LoadFailureCode::kManifestError,
      "registration plugin '" + request.class_id +
      "' loaded but its manifest provenance could not be resolved: " + exception.what());
    return result;
  } catch (...) {
    result.failure = makeFailure(
      LoadFailureCode::kManifestError,
      "registration plugin '" + request.class_id +
      "' loaded but its manifest provenance could not be resolved: unknown exception");
    return result;
  }

  result.session.reset(new RegistrationPluginSession(
      class_loader_, plugin, BackendKind::kPluginlib, metadata, capabilities,
      request.parameters, request.class_id, library_path, plugin_manifest_path));
  return result;
}

RegistrationResolver::RegistrationResolver(
  std::vector<HostBuiltinRegistration> host_builtins,
  const std::string & interface_package)
: host_builtins_(std::move(host_builtins))
{
  std::vector<std::string> host_ids;
  for (const auto & builtin : host_builtins_) {
    if (builtin.class_id.empty()) {
      initialization_error_ =
        "host-built-in registration class ID is empty; no implicit selector is allowed";
      initialization_failure_code_ = LoadFailureCode::kNamespaceViolation;
      continue;
    }
    if (!isHostBuiltinClassId(builtin.class_id)) {
      initialization_error_ =
        "host-built-in registration class '" + builtin.class_id +
        "' must use the reserved namespace '" + kHostBuiltinClassPrefix + "'";
      initialization_failure_code_ = LoadFailureCode::kNamespaceViolation;
      continue;
    }
    if (!builtin.factory) {
      initialization_error_ =
        "host-built-in registration class '" + builtin.class_id +
        "' has no factory";
      initialization_failure_code_ = LoadFailureCode::kInvalidRequest;
      continue;
    }
    if (std::find(host_ids.begin(), host_ids.end(), builtin.class_id) != host_ids.end()) {
      initialization_error_ =
        "duplicate host-built-in registration class '" + builtin.class_id +
        "'; class shadowing is forbidden";
      initialization_failure_code_ = LoadFailureCode::kClassCollision;
      continue;
    }
    host_ids.push_back(builtin.class_id);
  }

  plugin_loader_.reset(new RegistrationPluginLoader(interface_package));
  if (plugin_loader_ && !plugin_loader_->initializationError().empty()) {
    plugin_loader_error_ = plugin_loader_->initializationError();
  }

  // A host ID must never be shadowed by an installed pluginlib manifest.  The
  // check is performed at resolver construction, before a session exists.
  if (initialization_error_.empty() && plugin_loader_error_.empty()) {
    const std::vector<std::string> external_classes = plugin_loader_->availableClasses();
    for (const auto & external_class : external_classes) {
      if (isHostBuiltinClassId(external_class)) {
        initialization_error_ =
          "pluginlib manifest declares reserved host-built-in registration class '" +
          external_class + "'; external plugins must not use namespace '" +
          kHostBuiltinClassPrefix + "'";
        initialization_failure_code_ = LoadFailureCode::kNamespaceViolation;
        break;
      }
    }
    if (!initialization_error_.empty()) {
      return;
    }
    for (const auto & host_id : host_ids) {
      if (std::find(external_classes.begin(), external_classes.end(), host_id) !=
        external_classes.end())
      {
        initialization_error_ =
          "registration class collision for '" + host_id +
          "': a host-built-in ID is also declared by pluginlib; shadowing is forbidden";
        initialization_failure_code_ = LoadFailureCode::kClassCollision;
        break;
      }
    }
  }
}

std::vector<std::string> RegistrationResolver::availableClasses() const
{
  std::vector<std::string> classes;
  classes.reserve(host_builtins_.size());
  for (const auto & builtin : host_builtins_) {
    if (!builtin.class_id.empty()) {
      classes.push_back(builtin.class_id);
    }
  }
  if (plugin_loader_) {
    const std::vector<std::string> external_classes = plugin_loader_->availableClasses();
    classes.insert(classes.end(), external_classes.begin(), external_classes.end());
  }
  std::sort(classes.begin(), classes.end());
  classes.erase(std::unique(classes.begin(), classes.end()), classes.end());
  return classes;
}

LoadResult RegistrationResolver::resolve(const LoadRequest & request) const
{
  LoadResult result;
  if (request.class_id.empty()) {
    result.failure = makeFailure(
      LoadFailureCode::kInvalidRequest,
      "registration class ID is empty; choose an explicit host-built-in or pluginlib class");
    return result;
  }
  if (!initialization_error_.empty()) {
    result.failure = makeFailure(
      initialization_failure_code_,
      "registration resolver is not usable: " + initialization_error_);
    return result;
  }

  const auto host_it = std::find_if(
    host_builtins_.begin(), host_builtins_.end(),
    [&request](const HostBuiltinRegistration & builtin) {
      return builtin.class_id == request.class_id;
    });

  if (!isHostBuiltinClassId(request.class_id)) {
    // A non-reserved ID is always external.  There is intentionally no
    // attempt to reinterpret it as a host alias after a pluginlib failure.
    if (!plugin_loader_) {
      result.failure = makeFailure(
        LoadFailureCode::kLoaderUnavailable,
        "pluginlib resolver is unavailable for external registration class '" +
        request.class_id + "'");
      return result;
    }
    return plugin_loader_->load(request);
  }

  if (host_it == host_builtins_.end()) {
    result.failure = makeFailure(
      LoadFailureCode::kNamespaceViolation,
      "unknown host-built-in registration class '" + request.class_id +
      "'; host IDs are reserved and are never resolved through pluginlib; available classes: " +
      describeClasses(availableClasses()));
    return result;
  }
  if (plugin_loader_error_.empty() && plugin_loader_) {
    const std::vector<std::string> external_classes = plugin_loader_->availableClasses();
    if (std::find(external_classes.begin(), external_classes.end(), request.class_id) !=
      external_classes.end())
    {
      result.failure = makeFailure(
        LoadFailureCode::kClassCollision,
        "registration class '" + request.class_id +
        "' is declared by both host and pluginlib namespaces; shadowing is forbidden");
      return result;
    }
  }

  std::shared_ptr<RegistrationPlugin> plugin;
  try {
    plugin = host_it->factory();
  } catch (const std::exception & exception) {
    result.failure = makeFailure(
      LoadFailureCode::kPluginException,
      "host-built-in registration plugin '" + request.class_id +
      "' threw during construction: " + exception.what());
    return result;
  } catch (...) {
    result.failure = makeFailure(
      LoadFailureCode::kPluginException,
      "host-built-in registration plugin '" + request.class_id +
      "' threw an unknown exception during construction");
    return result;
  }
  if (!plugin) {
    result.failure = makeFailure(
      LoadFailureCode::kCreateInstance,
      "host-built-in factory returned a null registration plugin for '" + request.class_id +
      "'");
    return result;
  }

  PluginMetadata metadata;
  Capabilities capabilities;
  try {
    metadata = plugin->metadata();
    capabilities = plugin->capabilities();
  } catch (const std::exception & exception) {
    result.failure = makeFailure(
      LoadFailureCode::kPluginException,
      "host-built-in registration plugin '" + request.class_id +
      "' threw while reporting metadata/capabilities: " + exception.what());
    return result;
  } catch (...) {
    result.failure = makeFailure(
      LoadFailureCode::kPluginException,
      "host-built-in registration plugin '" + request.class_id +
      "' threw an unknown exception while reporting metadata/capabilities");
    return result;
  }

  // The same implementation may also be exported as an external fixture.
  // Accept only the explicitly registered alias, then canonicalize the
  // metadata stored in the resolved session to the selected host ID.
  if (!host_it->metadata_class_id.empty() &&
    metadata.class_id == host_it->metadata_class_id)
  {
    metadata.class_id = request.class_id;
  }
  result.failure = validateMetadata(request.class_id, metadata);
  if (!result.failure.ok()) {
    return result;
  }
  if (!isApiCompatible(kHostApiVersion, metadata.api_version)) {
    result.failure = makeFailure(
      LoadFailureCode::kApiMismatch,
      "host-built-in registration plugin '" + request.class_id + "' API " +
      std::to_string(metadata.api_version.major) + "." +
      std::to_string(metadata.api_version.minor) +
      " is incompatible with host API " + std::to_string(kHostApiVersion.major) + "." +
      std::to_string(kHostApiVersion.minor));
    return result;
  }
  if (request.enforce_permissive_license && !isPermissiveLicense(metadata.license)) {
    result.failure = makeFailure(
      LoadFailureCode::kMetadataInvalid,
      "host-built-in registration plugin '" + request.class_id + "' license '" +
      metadata.license + "' is not allowed by the shell permissive-license policy");
    return result;
  }

  std::string configuration_error;
  bool configured = false;
  try {
    configured = plugin->configure(request.parameters, &configuration_error);
  } catch (const std::exception & exception) {
    result.failure = makeFailure(
      LoadFailureCode::kPluginException,
      "host-built-in registration plugin '" + request.class_id +
      "' threw during configure(): " + exception.what());
    return result;
  } catch (...) {
    result.failure = makeFailure(
      LoadFailureCode::kPluginException,
      "host-built-in registration plugin '" + request.class_id +
      "' threw an unknown exception during configure()");
    return result;
  }
  if (!configured) {
    if (configuration_error.empty()) {
      configuration_error = "plugin returned false without a diagnostic";
    }
    result.failure = makeFailure(
      LoadFailureCode::kConfigurationFailure,
      "host-built-in registration plugin '" + request.class_id +
      "' rejected configuration: " + configuration_error);
    return result;
  }

  try {
    capabilities = plugin->capabilities();
  } catch (const std::exception & exception) {
    result.failure = makeFailure(
      LoadFailureCode::kPluginException,
      "host-built-in registration plugin '" + request.class_id +
      "' threw while reporting configured capabilities: " + exception.what());
    return result;
  } catch (...) {
    result.failure = makeFailure(
      LoadFailureCode::kPluginException,
      "host-built-in registration plugin '" + request.class_id +
      "' threw an unknown exception while reporting configured capabilities");
    return result;
  }
  result.failure = validateCapabilities(request.class_id, capabilities, request.capabilities);
  if (!result.failure.ok()) {
    return result;
  }

  result.session.reset(new RegistrationPluginSession(
      nullptr, plugin, BackendKind::kHostBuiltIn, metadata, capabilities,
      request.parameters, request.class_id, "", ""));
  return result;
}

}  // namespace shell
}  // namespace registration
}  // namespace plugins
}  // namespace lidarslam
