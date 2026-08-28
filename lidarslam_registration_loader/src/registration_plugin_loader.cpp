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
#include <filesystem>
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

namespace fs = std::filesystem;

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
  if (requirements.require_cooperative_cancel &&
    capabilities.cancellationModel() != CancellationModel::kCooperativeCancel)
  {
    return missing("cooperative_cancel");
  }
  return LoadFailure();
}

LoadFailure validateDescriptorAgainstCapabilities(
  const std::string & class_id,
  const RegistrationRuntimeDescriptor & descriptor,
  const Capabilities & capabilities)
{
  if ((descriptor.required_capability_bits & ~capabilities.bits()) != 0U) {
    return makeFailure(
      LoadFailureCode::kDescriptorMismatch,
      "registration plugin '" + class_id +
      "' runtime descriptor requires capabilities it did not advertise");
  }
  if ((descriptor.required_capability_bits & descriptor.optional_capability_bits) != 0U) {
    return makeFailure(
      LoadFailureCode::kDescriptorMismatch,
      "registration plugin '" + class_id +
      "' runtime descriptor has overlapping required/optional capability IDs");
  }
  if (descriptor.target_policy != capabilities.targetPolicy() ||
    descriptor.correspondence_metric != capabilities.correspondenceMetric() ||
    descriptor.thread_model != capabilities.threadModel() ||
    descriptor.cancellation_model != capabilities.cancellationModel())
  {
    return makeFailure(
      LoadFailureCode::kDescriptorMismatch,
      "registration plugin '" + class_id +
      "' runtime descriptor disagrees with negotiated capability policy");
  }
  return LoadFailure();
}

LoadFailure readPluginDescriptor(
  const std::string & class_id,
  const std::shared_ptr<RegistrationPlugin> & plugin,
  const Capabilities & capabilities,
  const RegistrationContractManifest * manifest,
  const RegistrationRuntimeDescriptor * expected,
  RegistrationRuntimeDescriptor * descriptor)
{
  if (plugin == nullptr || descriptor == nullptr) {
    return makeFailure(
      LoadFailureCode::kDescriptorMismatch,
      "registration plugin '" + class_id + "' descriptor validation received null state");
  }
  auto * provider = dynamic_cast<RegistrationPluginDescriptorProvider *>(plugin.get());
  if (provider == nullptr) {
    return makeFailure(
      LoadFailureCode::kDescriptorMismatch,
      "registration plugin '" + class_id +
      "' does not implement the required runtime descriptor provider");
  }
  try {
    *descriptor = provider->registrationDescriptor();
  } catch (const std::exception & exception) {
    return makeFailure(
      LoadFailureCode::kPluginException,
      "registration plugin '" + class_id +
      "' threw while reporting its runtime descriptor: " + exception.what());
  } catch (...) {
    return makeFailure(
      LoadFailureCode::kPluginException,
      "registration plugin '" + class_id +
      "' threw an unknown exception while reporting its runtime descriptor");
  }
  LoadFailure failure = validateRegistrationRuntimeDescriptor(
    class_id, *descriptor, manifest, expected);
  if (!failure.ok()) {
    return failure;
  }
  return validateDescriptorAgainstCapabilities(class_id, *descriptor, capabilities);
}

bool makeStaticHostDescriptor(
  const std::string & class_id, RegistrationRuntimeDescriptor * descriptor)
{
  if (descriptor == nullptr || !isHostBuiltinClassId(class_id)) {
    return false;
  }
  PluginMetadata metadata;
  metadata.class_id = class_id;
  metadata.implementation_version = "host-preflight";
  metadata.license = "BSD-2-Clause";
  metadata.api_version = kHostApiVersion;
  Capabilities capabilities;
  std::uint64_t required = 0U;
  std::uint64_t optional = 0U;
  const bool is_gicp = class_id.find("Gicp") != std::string::npos ||
    class_id.find("VGicp") != std::string::npos;
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
  } else if (is_gicp) {
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
  } else {
    return false;
  }
  *descriptor = makeRegistrationRuntimeDescriptor(
    metadata, capabilities, required, optional,
    registrationConfigSchemaForClassId(class_id));
  return descriptor->logicallyComplete();
}

LoadFailure readHostPluginDescriptor(
  const std::string & class_id,
  const std::string & metadata_class_id,
  const std::shared_ptr<RegistrationPlugin> & plugin,
  const Capabilities & capabilities,
  const RegistrationRuntimeDescriptor & expected,
  RegistrationRuntimeDescriptor * descriptor)
{
  if (plugin == nullptr || descriptor == nullptr) {
    return makeFailure(LoadFailureCode::kDescriptorMismatch,
      "host registration descriptor validation received null state");
  }
  auto * provider = dynamic_cast<RegistrationPluginDescriptorProvider *>(plugin.get());
  if (provider == nullptr) {
    return makeFailure(LoadFailureCode::kDescriptorMismatch,
      "host registration plugin '" + class_id +
      "' does not implement the required runtime descriptor provider");
  }
  try {
    *descriptor = provider->registrationDescriptor();
  } catch (const std::exception & exception) {
    return makeFailure(LoadFailureCode::kPluginException,
      "host registration plugin '" + class_id +
      "' threw while reporting its runtime descriptor: " + exception.what());
  } catch (...) {
    return makeFailure(LoadFailureCode::kPluginException,
      "host registration plugin '" + class_id +
      "' threw an unknown exception while reporting its runtime descriptor");
  }
  if (!metadata_class_id.empty() && descriptor->class_id == metadata_class_id) {
    descriptor->class_id = class_id;
  }
  LoadFailure result = validateRegistrationRuntimeDescriptor(
    class_id, *descriptor, nullptr, &expected);
  if (result.ok()) {
    result = validateDescriptorAgainstCapabilities(class_id, *descriptor, capabilities);
  }
  return result;
}

LoadFailure validateProvenancePath(
  const std::string & class_id, const std::string & label, const std::string & value,
  const bool reject_symlink)
{
  if (value.empty()) {
    return makeFailure(
      LoadFailureCode::kProvenanceInvalid,
      "registration plugin '" + class_id + "' has an empty " + label + " provenance path");
  }

  const fs::path path(value);
  if (!path.is_absolute()) {
    return makeFailure(
      LoadFailureCode::kProvenanceInvalid,
      "registration plugin '" + class_id + "' " + label +
      " provenance path must be absolute: '" + value + "'");
  }

  std::error_code status_error;
  const fs::file_status status = fs::symlink_status(path, status_error);
  if (status_error) {
    return makeFailure(
      LoadFailureCode::kProvenanceInvalid,
      "registration plugin '" + class_id + "' " + label +
      " provenance could not be inspected: " + status_error.message());
  }
  if (fs::is_symlink(status) && reject_symlink) {
    return makeFailure(
      LoadFailureCode::kProvenanceInvalid,
      "registration plugin '" + class_id + "' " + label +
      " provenance must not be a symlink: '" + value + "'");
  }
  std::error_code target_error;
  const fs::file_status target_status = fs::status(path, target_error);
  if (target_error || !fs::is_regular_file(target_status)) {
    return makeFailure(
      LoadFailureCode::kProvenanceInvalid,
      "registration plugin '" + class_id + "' " + label +
      " provenance is not a regular file: '" + value + "'");
  }
  return LoadFailure();
}

AlignmentResult makeSessionProcessingFailure(
  const FailureCode code, const std::string & detail)
{
  AlignmentResult result;
  result.failure = code;
  result.diagnostics.detail = detail;
  return result;
}

}  // namespace

LoadFailure validateExternalDsoProvenance(
  const std::string & class_id,
  const std::string & library_path,
  const std::string & plugin_manifest_path)
{
  if (class_id.empty()) {
    return makeFailure(
      LoadFailureCode::kProvenanceInvalid,
      "external registration provenance requires a non-empty class ID");
  }
  LoadFailure failure = validateProvenancePath(class_id, "library", library_path, true);
  if (!failure.ok()) {
    return failure;
  }
  failure = validateProvenancePath(class_id, "plugin manifest", plugin_manifest_path, false);
  if (!failure.ok()) {
    return failure;
  }

  std::error_code equivalent_error;
  if (std::filesystem::equivalent(
      std::filesystem::path(library_path), std::filesystem::path(plugin_manifest_path),
      equivalent_error))
  {
    return makeFailure(
      LoadFailureCode::kProvenanceInvalid,
      "registration plugin '" + class_id +
      "' library and plugin manifest resolve to the same file");
  }
  if (equivalent_error) {
    return makeFailure(
      LoadFailureCode::kProvenanceInvalid,
      "registration plugin '" + class_id +
      "' library/manifest equivalence could not be checked: " + equivalent_error.message());
  }
  return LoadFailure();
}

std::shared_ptr<RegistrationPluginSession> RegistrationPluginSession::createHostSession(
  std::shared_ptr<RegistrationPlugin> plugin,
  const LoadRequest & request,
  const std::string & metadata_class_id,
  const bool configure_plugin,
  LoadFailure * failure)
{
  if (failure != nullptr) {
    *failure = LoadFailure{};
  }
  const auto reject = [failure](const LoadFailureCode code, const std::string & message) {
      if (failure != nullptr) {
        *failure = makeFailure(code, message);
      }
      return std::shared_ptr<RegistrationPluginSession>();
    };

  if (!plugin) {
    return reject(
      LoadFailureCode::kCreateInstance,
      "host registration session cannot adopt a null plugin");
  }
  if (request.class_id.empty() || !isHostBuiltinClassId(request.class_id)) {
    return reject(
      LoadFailureCode::kNamespaceViolation,
      "host registration session requires a non-empty class ID in the reserved host namespace");
  }
  if (!configure_plugin && !request.parameters.empty()) {
    return reject(
      LoadFailureCode::kInvalidRequest,
      "preconfigured host registration adoption requires an empty parameter map");
  }

  PluginMetadata metadata;
  Capabilities capabilities;
  try {
    metadata = plugin->metadata();
    capabilities = plugin->capabilities();
  } catch (const std::exception & exception) {
    return reject(
      LoadFailureCode::kPluginException,
      "host registration plugin '" + request.class_id +
      "' threw while reporting metadata/capabilities: " + exception.what());
  } catch (...) {
    return reject(
      LoadFailureCode::kPluginException,
      "host registration plugin '" + request.class_id +
      "' threw an unknown exception while reporting metadata/capabilities");
  }

  // A host adapter may retain an implementation-facing identity.  Accept
  // only the caller-declared alias and canonicalize the session receipt to
  // the immutable host selector; arbitrary metadata spoofing is rejected.
  if (!metadata_class_id.empty() && metadata.class_id == metadata_class_id) {
    metadata.class_id = request.class_id;
  }
  LoadFailure validation = validateMetadata(request.class_id, metadata);
  if (!validation.ok()) {
    if (failure != nullptr) {
      *failure = validation;
    }
    return nullptr;
  }
  if (!isApiCompatible(kHostApiVersion, metadata.api_version)) {
    return reject(
      LoadFailureCode::kApiMismatch,
      "host registration plugin '" + request.class_id + "' API " +
      std::to_string(metadata.api_version.major) + "." +
      std::to_string(metadata.api_version.minor) +
      " is incompatible with host API " + std::to_string(kHostApiVersion.major) + "." +
      std::to_string(kHostApiVersion.minor));
  }
  if (request.enforce_permissive_license && !isPermissiveLicense(metadata.license)) {
    return reject(
      LoadFailureCode::kMetadataInvalid,
      "host registration plugin '" + request.class_id + "' license '" + metadata.license +
      "' is not allowed by the shell permissive-license policy");
  }

  // Host-adopted adapters have no physical XML/DSO sidecar.  Their logical
  // identity is nevertheless preflighted before configuration/activation
  // using a deterministic expected descriptor, then checked again after
  // configuration below.
  RegistrationRuntimeDescriptor expected = makeRegistrationRuntimeDescriptor(
    metadata, capabilities, capabilities.bits(), 0U,
    registrationConfigSchemaForClassId(request.class_id));
  expected.class_id = request.class_id;
  RegistrationRuntimeDescriptor descriptor;
  auto * provider = dynamic_cast<RegistrationPluginDescriptorProvider *>(plugin.get());
  if (provider == nullptr) {
    return reject(
      LoadFailureCode::kDescriptorMismatch,
      "host registration plugin '" + request.class_id +
      "' does not implement the required runtime descriptor provider");
  }
  try {
    descriptor = provider->registrationDescriptor();
  } catch (const std::exception & exception) {
    return reject(
      LoadFailureCode::kPluginException,
      "host registration plugin '" + request.class_id +
      "' threw while reporting its runtime descriptor: " + exception.what());
  } catch (...) {
    return reject(
      LoadFailureCode::kPluginException,
      "host registration plugin '" + request.class_id +
      "' threw an unknown exception while reporting its runtime descriptor");
  }
  if (!metadata_class_id.empty() && descriptor.class_id == metadata_class_id) {
    descriptor.class_id = request.class_id;
  }
  validation = validateRegistrationRuntimeDescriptor(
    request.class_id, descriptor, nullptr, &expected);
  if (validation.ok()) {
    validation = validateDescriptorAgainstCapabilities(request.class_id, descriptor, capabilities);
  }
  if (!validation.ok()) {
    if (failure != nullptr) {
      *failure = validation;
    }
    return nullptr;
  }

  if (configure_plugin) {
    std::string configuration_error;
    bool configured = false;
    try {
      configured = plugin->configure(request.parameters, &configuration_error);
    } catch (const std::exception & exception) {
      return reject(
        LoadFailureCode::kPluginException,
        "host registration plugin '" + request.class_id +
        "' threw during configure(): " + exception.what());
    } catch (...) {
      return reject(
        LoadFailureCode::kPluginException,
        "host registration plugin '" + request.class_id +
        "' threw an unknown exception during configure()");
    }
    if (!configured) {
      if (configuration_error.empty()) {
        configuration_error = "plugin returned false without a diagnostic";
      }
      return reject(
        LoadFailureCode::kConfigurationFailure,
        "host registration plugin '" + request.class_id +
        "' rejected configuration: " + configuration_error);
    }
  }

  try {
    capabilities = plugin->capabilities();
  } catch (const std::exception & exception) {
    return reject(
      LoadFailureCode::kPluginException,
      "host registration plugin '" + request.class_id +
      "' threw while reporting configured capabilities: " + exception.what());
  } catch (...) {
    return reject(
      LoadFailureCode::kPluginException,
      "host registration plugin '" + request.class_id +
      "' threw an unknown exception while reporting configured capabilities");
  }
  validation = validateCapabilities(request.class_id, capabilities, request.capabilities);
  if (!validation.ok()) {
    if (failure != nullptr) {
      *failure = validation;
    }
    return nullptr;
  }

  validation = readPluginDescriptor(
    request.class_id, plugin, capabilities, nullptr, &expected, &descriptor);
  if (!validation.ok() && !metadata_class_id.empty() && descriptor.class_id == metadata_class_id) {
    descriptor.class_id = request.class_id;
    validation = validateRegistrationRuntimeDescriptor(
      request.class_id, descriptor, nullptr, &expected);
    if (validation.ok()) {
      validation = validateDescriptorAgainstCapabilities(request.class_id, descriptor,
                capabilities);
    }
  }
  if (!validation.ok()) {
    if (failure != nullptr) {
      *failure = validation;
    }
    return nullptr;
  }

  return std::shared_ptr<RegistrationPluginSession>(new RegistrationPluginSession(
           nullptr, std::move(plugin), BackendKind::kHostBuiltIn, metadata, capabilities,
           request.parameters, request.class_id, "", "", std::move(descriptor)));
}

bool RegistrationPluginSession::setInputTarget(
  const PointCloudConstPtr & target, std::string * error)
{
  const std::shared_ptr<ProcessingState> state = processing_state_;
  if (!state || !plugin_) {
    if (error != nullptr) {
      *error = "registration plugin session has no live plugin object";
    }
    return false;
  }
  std::unique_lock<std::mutex> lock;
  const ProcessingState::EntryResult entry = state->begin(
    capabilities().threadModel() == ThreadModel::kSerializedOwner, &lock);
  if (entry != ProcessingState::EntryResult::kEntered) {
    if (error != nullptr) {
      *error = entry == ProcessingState::EntryResult::kFaulted ?
        "registration plugin session is faulted after a plugin exception" :
        "registration plugin session is cancelled or shut down";
    }
    return false;
  }
  try {
    bool accepted = plugin_->setInputTarget(target, error);
    if (state->cancelled.load() || state->shutdown.load()) {
      accepted = false;
      if (error != nullptr) {
        *error = "registration plugin session was cancelled during target processing";
      }
    }
    if (!accepted && error != nullptr && error->empty()) {
      *error = "registration plugin rejected the input target without a reason";
    }
    state->end(&lock);
    return accepted;
  } catch (const std::exception & exception) {
    state->faulted.store(true);
    state->end(&lock);
    if (error != nullptr) {
      *error = std::string("registration plugin setInputTarget threw: ") + exception.what();
    }
    return false;
  } catch (...) {
    state->faulted.store(true);
    state->end(&lock);
    if (error != nullptr) {
      *error = "registration plugin setInputTarget threw an unknown exception";
    }
    return false;
  }
}

AlignmentResult RegistrationPluginSession::align(const AlignmentRequest & request)
{
  const std::shared_ptr<ProcessingState> state = processing_state_;
  if (!state || !plugin_) {
    return makeSessionProcessingFailure(
      FailureCode::kInternalError,
      "registration plugin session has no live plugin object");
  }
  std::unique_lock<std::mutex> lock;
  const ProcessingState::EntryResult entry = state->begin(
    capabilities().threadModel() == ThreadModel::kSerializedOwner, &lock);
  if (entry != ProcessingState::EntryResult::kEntered) {
    return makeSessionProcessingFailure(
      entry == ProcessingState::EntryResult::kFaulted ?
      FailureCode::kInternalError : FailureCode::kCancelled,
      entry == ProcessingState::EntryResult::kFaulted ?
      "registration plugin session is faulted after a plugin exception" :
      "registration plugin session was cancelled before alignment");
  }
  try {
    AlignmentResult result = plugin_->align(request);
    // This is the post-call checkpoint.  A cooperative provider should also
    // observe requestCancel() internally, while a non-interruptible provider
    // is allowed to finish here before the session reports cancellation.
    if (state->cancelled.load() || state->shutdown.load()) {
      result = makeSessionProcessingFailure(
        FailureCode::kCancelled,
        "registration plugin session was cancelled during alignment");
    }
    state->end(&lock);
    return result;
  } catch (const std::exception & exception) {
    state->faulted.store(true);
    state->end(&lock);
    return makeSessionProcessingFailure(
      FailureCode::kInternalError,
      std::string("registration plugin align threw: ") + exception.what());
  } catch (...) {
    state->faulted.store(true);
    state->end(&lock);
    return makeSessionProcessingFailure(
      FailureCode::kInternalError,
      "registration plugin align threw an unknown exception");
  }
}

void RegistrationPluginSession::cancel() noexcept
{
  const std::shared_ptr<ProcessingState> state = processing_state_;
  if (state) {
    state->cancelled.store(true);
    if (capabilities().cancellationModel() == CancellationModel::kCooperativeCancel &&
      plugin_)
    {
      plugin_->requestCancel();
    }
  }
}

void RegistrationPluginSession::shutdown() noexcept
{
  const std::shared_ptr<ProcessingState> state = processing_state_;
  if (!state || state->shutdown.exchange(true)) {
    return;
  }
  state->cancelled.store(true);
  if (capabilities().cancellationModel() == CancellationModel::kCooperativeCancel &&
    plugin_)
  {
    plugin_->requestCancel();
  }
  // Serialized and reentrant providers share this quiescence barrier.  For a
  // non-interruptible provider shutdown may wait for the current align to
  // finish; it must never reset or unload a DSO while a callback is active.
  std::unique_lock<std::mutex> lock(state->mutex);
  state->quiesced.wait(lock, [state]() {return state->in_flight == 0U;});
  if (plugin_) {
    plugin_->reset();
  }
}

bool RegistrationPluginSession::cancelled() const noexcept
{
  return processing_state_ &&
         (processing_state_->cancelled.load() || processing_state_->shutdown.load());
}

bool RegistrationPluginSession::faulted() const noexcept
{
  return processing_state_ && processing_state_->faulted.load();
}

RegistrationPluginLoader::RegistrationPluginLoader(const std::string & interface_package)
: RegistrationPluginLoader(interface_package, {})
{
}

RegistrationPluginLoader::RegistrationPluginLoader(
  const std::string & interface_package,
  const std::vector<std::string> & plugin_xml_paths)
{
  if (interface_package.empty()) {
    initialization_error_ = "registration plugin loader base package is empty";
    return;
  }
  try {
    class_loader_.reset(new RegistrationPluginClassLoader(
      interface_package, kRegistrationPluginBaseClass, "plugin", plugin_xml_paths));
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

  // pluginlib exposes both paths from its parsed ClassDesc without loading
  // the DSO.  Validate them before createSharedInstance() so a rejected
  // provenance cannot run a plugin constructor or static initializer.
  std::string library_path;
  std::string plugin_manifest_path;
  try {
    library_path = class_loader_->getClassLibraryPath(request.class_id);
    plugin_manifest_path = class_loader_->getPluginManifestPath(request.class_id);
  } catch (const pluginlib::LibraryLoadException & exception) {
    // Preserve the public failure classification used when pluginlib resolved
    // the path lazily inside createSharedInstance().  The lookup is still
    // pre-load: this exception is raised while resolving the ClassDesc path,
    // before pluginlib is allowed to dlopen the DSO.
    result.failure = makeFailure(
      LoadFailureCode::kLibraryLoad,
      "failed to resolve registration plugin library before instance creation for '" +
      request.class_id + "': " + exception.what());
    return result;
  } catch (const std::exception & exception) {
    result.failure = makeFailure(
      LoadFailureCode::kManifestError,
      "failed to resolve registration plugin provenance before instance creation for '" +
      request.class_id + "': " + exception.what());
    return result;
  } catch (...) {
    result.failure = makeFailure(
      LoadFailureCode::kManifestError,
      "failed to resolve registration plugin provenance before instance creation for '" +
      request.class_id + "': unknown exception");
    return result;
  }
  result.failure = validateExternalDsoProvenance(
    request.class_id, library_path, plugin_manifest_path);
  if (!result.failure.ok()) {
    return result;
  }
  RegistrationContractManifest contract_manifest;
  result.failure = readAndValidateRegistrationContractManifest(
    request.class_id, library_path, plugin_manifest_path, &contract_manifest);
  if (!result.failure.ok()) {
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

  RegistrationRuntimeDescriptor descriptor;
  result.failure = readPluginDescriptor(
    request.class_id, plugin, capabilities, &contract_manifest, nullptr, &descriptor);
  if (!result.failure.ok()) {
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

  result.failure = readPluginDescriptor(
    request.class_id, plugin, capabilities, &contract_manifest, nullptr, &descriptor);
  if (!result.failure.ok()) {
    return result;
  }

  result.session.reset(new RegistrationPluginSession(
      class_loader_, plugin, BackendKind::kPluginlib, metadata, capabilities,
      request.parameters, request.class_id, library_path, plugin_manifest_path,
      std::move(descriptor)));
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

  RegistrationRuntimeDescriptor expected_descriptor;
  bool has_expected_descriptor = false;
  if (host_it->expected_descriptor_factory) {
    try {
      expected_descriptor = host_it->expected_descriptor_factory();
      has_expected_descriptor = true;
    } catch (const std::exception & exception) {
      result.failure = makeFailure(
        LoadFailureCode::kPluginException,
        "host-built-in registration descriptor preflight threw for '" + request.class_id +
        "': " + exception.what());
      return result;
    } catch (...) {
      result.failure = makeFailure(
        LoadFailureCode::kPluginException,
        "host-built-in registration descriptor preflight threw an unknown exception for '" +
        request.class_id + "'");
      return result;
    }
  } else {
    has_expected_descriptor = makeStaticHostDescriptor(request.class_id, &expected_descriptor);
  }
  if (!has_expected_descriptor || !expected_descriptor.logicallyComplete() ||
    expected_descriptor.class_id != request.class_id)
  {
    result.failure = makeFailure(
      LoadFailureCode::kDescriptorMismatch,
      "host-built-in registration '" + request.class_id +
      "' has no complete deterministic pre-instantiation descriptor");
    return result;
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

  RegistrationRuntimeDescriptor descriptor;
  result.failure = readHostPluginDescriptor(
    request.class_id, host_it->metadata_class_id, plugin, capabilities,
    expected_descriptor, &descriptor);
  if (!result.failure.ok()) {
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

  result.failure = readHostPluginDescriptor(
    request.class_id, host_it->metadata_class_id, plugin, capabilities,
    expected_descriptor, &descriptor);
  if (!result.failure.ok()) {
    return result;
  }

  result.session.reset(new RegistrationPluginSession(
      nullptr, plugin, BackendKind::kHostBuiltIn, metadata, capabilities,
      request.parameters, request.class_id, "", "", std::move(descriptor)));
  return result;
}

}  // namespace shell
}  // namespace registration
}  // namespace plugins
}  // namespace lidarslam
