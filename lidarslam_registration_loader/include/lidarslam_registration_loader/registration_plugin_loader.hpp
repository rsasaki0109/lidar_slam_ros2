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

#ifndef LIDARSLAM_REGISTRATION_LOADER__REGISTRATION_PLUGIN_LOADER_HPP_
#define LIDARSLAM_REGISTRATION_LOADER__REGISTRATION_PLUGIN_LOADER_HPP_

#include <memory>
#include <functional>
#include <exception>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#if __cplusplus >= 201703L
#include <pluginlib/class_loader.hpp>
#else
// The public session/adapter surface is C++14.  A C++14 consumer does not
// instantiate the ROS/pluginlib discovery shell, so keep its ClassLoader type
// incomplete here instead of pulling Jazzy's C++17-only headers into the SDK.
namespace pluginlib
{
template<typename T>
class ClassLoader;
}  // namespace pluginlib
#endif

#include <lidarslam_plugin_interfaces/registration.hpp>

namespace lidarslam
{
namespace plugins
{
namespace registration
{
namespace shell
{

constexpr const char * kRegistrationPluginBasePackage = "lidarslam_plugin_interfaces";
constexpr const char * kRegistrationPluginBaseClass =
  "lidarslam::plugins::registration::RegistrationPlugin";
// This namespace is reserved for factories that construct the implementation
// in the host process.  All other class IDs are external pluginlib IDs.
constexpr const char * kHostBuiltinClassPrefix = "lidarslam_builtin/";

enum class BackendKind
{
  kHostBuiltIn,
  kPluginlib,
};

inline const char * backendKindName(const BackendKind kind)
{
  return kind == BackendKind::kHostBuiltIn ? "host_builtin" : "pluginlib";
}

inline bool isHostBuiltinClassId(const std::string & class_id)
{
  return class_id.compare(0, std::string(kHostBuiltinClassPrefix).size(),
    kHostBuiltinClassPrefix) == 0;
}

enum class LoadFailureCode
{
  kNone,
  kInvalidRequest,
  kLoaderUnavailable,
  kManifestError,
  kUnknownClass,
  kLibraryLoad,
  kCreateInstance,
  kPluginException,
  kApiMismatch,
  kMetadataInvalid,
  kCapabilityMismatch,
  kConfigurationFailure,
  kNamespaceViolation,
  kClassCollision,
  // Appended to preserve the numeric values of the original shell failure
  // codes for external consumers compiled against an older loader header.
  kProvenanceInvalid,
  kContractManifestMissing,
  kContractManifestInvalid,
  kAbiMismatch,
  kConfigSchemaMismatch,
  kDescriptorMismatch,
};

struct LoadFailure
{
  LoadFailureCode code{LoadFailureCode::kNone};
  std::string message;

  bool ok() const {return code == LoadFailureCode::kNone;}
};

/**
 * Validate the immutable file provenance returned by pluginlib for an
 * external DSO.  The loader calls this before publishing a session.  A
 * Relative, missing, or non-regular paths are rejected.  The DSO itself must
 * not be a final-component symlink; package-resource manifests may be
 * symlink-installed by colcon, but their resolved target must be regular.
 */
LoadFailure validateExternalDsoProvenance(
  const std::string & class_id,
  const std::string & library_path,
  const std::string & plugin_manifest_path);

/**
 * Immutable installed identity for one pluginlib XML class.  The contract
 * digest covers the logical fields below; it is not a claim that a raw
 * header hash proves a complete C++ ABI.  The ABI epoch and toolchain tag
 * are checked separately, and distro/toolchain rebuilds remain required when
 * those identities differ.
 */
struct RegistrationContractManifest
{
  std::string schema;
  std::uint32_t schema_version{0U};
  std::string class_id;
  std::string plugin_xml_sha256;
  std::uint64_t plugin_xml_size_bytes{0U};
  std::string dso_sha256;
  std::uint64_t dso_size_bytes{0U};
  std::string abi_epoch;
  std::string toolchain_tag;
  std::string interface_contract_sha256;
  ApiVersion api_min;
  ApiVersion api_max;
  std::uint64_t required_capability_bits{0U};
  std::uint64_t optional_capability_bits{0U};
  TargetPolicy target_policy{TargetPolicy::kAcceptHostPrepared};
  CorrespondenceMetric correspondence_metric{CorrespondenceMetric::kUnavailable};
  ThreadModel thread_model{ThreadModel::kSerializedOwner};
  CancellationModel cancellation_model{CancellationModel::kNonInterruptibleAlign};
  std::string config_schema_id;
  std::uint32_t config_schema_version{0U};
  std::string config_schema_sha256;
  std::string manifest_sha256;
};

std::string registrationContractSidecarPath(
  const std::string & plugin_manifest_path, const std::string & class_id);

std::string registrationContractManifestDigest(const RegistrationContractManifest & manifest);

std::string registrationContractFileSha256(const std::string & path);

LoadFailure readAndValidateRegistrationContractManifest(
  const std::string & class_id,
  const std::string & library_path,
  const std::string & plugin_manifest_path,
  RegistrationContractManifest * manifest);

LoadFailure validateRegistrationRuntimeDescriptor(
  const std::string & requested_class_id,
  const RegistrationRuntimeDescriptor & descriptor,
  const RegistrationContractManifest * manifest,
  const RegistrationRuntimeDescriptor * expected);

struct CapabilityRequirements
{
  bool require_initial_guess{false};
  bool require_rotation_prior{false};
  bool require_translation_prior{false};
  bool require_maximum_correspondence_distance{false};
  bool require_mean_correspondence_distance{false};
  bool require_aligned_source{false};
  bool require_deterministic{false};
  bool require_target_policy{false};
  TargetPolicy target_policy{TargetPolicy::kAcceptHostPrepared};
  bool require_correspondence_metric{false};
  CorrespondenceMetric correspondence_metric{CorrespondenceMetric::kUnavailable};
  bool require_cooperative_cancel{false};
};

struct LoadRequest
{
  std::string class_id;
  ParameterMap parameters;
  CapabilityRequirements capabilities;
  bool enforce_permissive_license{true};
};

using RegistrationPlugin = lidarslam::plugins::registration::RegistrationPlugin;
using RegistrationPluginClassLoader = pluginlib::ClassLoader<RegistrationPlugin>;

using HostBuiltinFactory = std::function<std::shared_ptr<RegistrationPlugin>()>;

/**
 * A host-resident registration entry.  The selected class ID must live under
 * kHostBuiltinClassPrefix.  metadata_class_id is an optional implementation
 * identity alias for adapters that also expose a pluginlib-facing metadata
 * ID; the resolved session canonicalizes metadata.class_id to class_id.
 */
struct HostBuiltinRegistration
{
  std::string class_id;
  HostBuiltinFactory factory;
  std::string metadata_class_id;
  // Production host registrations bind this before construction because
  // built-ins do not have a pluginlib sidecar.  Empty is retained only for
  // source-compatible legacy/test factories, which still undergo runtime
  // descriptor validation.
  std::function<RegistrationRuntimeDescriptor()> expected_descriptor_factory;
};

struct ResolvedRegistrationProvenance
{
  BackendKind backend_kind{BackendKind::kPluginlib};
  std::string class_id;
  PluginMetadata metadata;
  Capabilities capabilities;
  ParameterMap parameters;
  std::string library_path;
  std::string plugin_manifest_path;
  RegistrationRuntimeDescriptor descriptor;
};

/**
 * Owns a configured plugin and the ClassLoader which created it.
 *
 * The loader shared pointer is intentionally a member of the session.  This
 * makes it safe to return a session from a short-lived shell helper: the
 * plugin library cannot be unloaded while the plugin object is still alive.
 */
class RegistrationPluginSession final
{
public:
  ~RegistrationPluginSession() = default;

  RegistrationPluginSession(const RegistrationPluginSession &) = delete;
  RegistrationPluginSession & operator=(const RegistrationPluginSession &) = delete;
  RegistrationPluginSession(RegistrationPluginSession &&) = default;
  RegistrationPluginSession & operator=(RegistrationPluginSession &&) = default;

  /**
   * Adopt a host-constructed implementation into the same session boundary
   * used by pluginlib implementations.  This is the escape hatch for
   * legacy PCL objects whose construction/configuration must remain in the
   * estimator translation unit: it never asks pluginlib to reinterpret the
   * host object, and it still performs metadata/API/capability/license
   * validation before exposing a processing lease.
   *
   * When configure_plugin is false the caller is asserting that the object
   * was fully configured before adoption.  The request must then contain the
   * empty startup parameter map; configuration is never attempted implicitly.
   */
  static std::shared_ptr<RegistrationPluginSession> createHostSession(
    std::shared_ptr<RegistrationPlugin> plugin,
    const LoadRequest & request,
    const std::string & metadata_class_id,
    bool configure_plugin,
    LoadFailure * failure);

  std::shared_ptr<RegistrationPlugin> plugin() const {return plugin_;}
  RegistrationPlugin * get() const {return plugin_.get();}
  // External sessions retain the pluginlib ClassLoader for the complete
  // plugin lifetime. Retaining a plugin pointer after its loader is destroyed
  // is invalid, so clean consumers can assert this lease explicitly.
  bool hasExternalLoaderLease() const {return loader_ != nullptr;}
  const ResolvedRegistrationProvenance & provenance() const {return provenance_;}
  BackendKind backendKind() const {return provenance_.backend_kind;}
  const PluginMetadata & metadata() const {return provenance_.metadata;}
  const Capabilities & capabilities() const {return provenance_.capabilities;}
  const ParameterMap & parameters() const {return provenance_.parameters;}
  const std::string & classId() const {return provenance_.class_id;}
  const std::string & libraryPath() const {return provenance_.library_path;}
  const std::string & pluginManifestPath() const {return provenance_.plugin_manifest_path;}
  const RegistrationRuntimeDescriptor & descriptor() const {return provenance_.descriptor;}

  /**
   * Session-owned processing boundary.
   *
   * Consumers that received a session must use these methods for target and
   * alignment calls.  They preserve the loader lease, serialize a
   * kSerializedOwner plugin, convert plugin exceptions into a structured
   * kInternalError result, and permanently fault the session after such an
   * exception.  The raw plugin() accessor remains source-compatible for
   * legacy adapters but is not a safe live-processing boundary.
   */
  bool setInputTarget(const PointCloudConstPtr & target, std::string * error);
  AlignmentResult align(const AlignmentRequest & request);

  // Cancellation always blocks new admission and performs pre/post
  // checkpoints.  A kCooperativeCancel provider also receives
  // requestCancel(); a kNonInterruptibleAlign provider may finish an active
  // call. shutdown() waits for all in-flight calls before reset(), so a
  // reentrant provider is never reset concurrently with align().
  void cancel() noexcept;
  void shutdown() noexcept;
  bool cancelled() const noexcept;
  bool faulted() const noexcept;

private:
  friend class RegistrationPluginLoader;
  friend class RegistrationResolver;

  struct ProcessingState final
  {
    enum class EntryResult
    {
      kEntered,
      kCancelled,
      kFaulted,
    };

    std::mutex mutex;
    std::condition_variable quiesced;
    std::size_t in_flight{0U};
    std::atomic<bool> cancelled{false};
    std::atomic<bool> faulted{false};
    std::atomic<bool> shutdown{false};

    EntryResult begin(
      const bool serialized, std::unique_lock<std::mutex> * lock)
    {
      if (lock == nullptr) {
        return EntryResult::kFaulted;
      }
      *lock = std::unique_lock<std::mutex>(mutex);
      if (cancelled.load() || shutdown.load()) {
        return EntryResult::kCancelled;
      }
      if (faulted.load()) {
        return EntryResult::kFaulted;
      }
      ++in_flight;
      if (!serialized) {
        lock->unlock();
      }
      return EntryResult::kEntered;
    }

    void end(std::unique_lock<std::mutex> * lock)
    {
      if (lock == nullptr) {
        return;
      }
      if (!lock->owns_lock()) {
        lock->lock();
      }
      if (in_flight > 0U) {
        --in_flight;
      }
      if (in_flight == 0U) {
        quiesced.notify_all();
      }
    }
  };

  RegistrationPluginSession(
    std::shared_ptr<RegistrationPluginClassLoader> loader,
    std::shared_ptr<RegistrationPlugin> plugin,
    BackendKind backend_kind,
    const PluginMetadata & metadata,
    const Capabilities & capabilities,
    ParameterMap parameters,
    std::string class_id,
    std::string library_path,
    std::string plugin_manifest_path,
    RegistrationRuntimeDescriptor descriptor)
  : loader_(std::move(loader)),
    plugin_(std::move(plugin)),
    provenance_{
      backend_kind,
      std::move(class_id),
      metadata,
      capabilities,
      std::move(parameters),
      std::move(library_path),
      std::move(plugin_manifest_path),
      std::move(descriptor)},
    processing_state_(std::make_shared<ProcessingState>()) {}

  // Declaration order is deliberate: plugin_ is destroyed before loader_.
  std::shared_ptr<RegistrationPluginClassLoader> loader_;
  std::shared_ptr<RegistrationPlugin> plugin_;
  ResolvedRegistrationProvenance provenance_;
  std::shared_ptr<ProcessingState> processing_state_;
};

/**
 * Typed facade for ROS-free cores which intentionally accept only the public
 * RegistrationPlugin interface.  It keeps those cores independent of
 * pluginlib while routing processing calls through a session's lease,
 * exception, cancellation, and thread-ownership boundary.  Configuration is
 * a startup operation and is therefore rejected once a session exists.
 */
class RegistrationPluginSessionAdapter final
  : public RegistrationPlugin,
  public RegistrationPluginDescriptorProvider
{
public:
  explicit RegistrationPluginSessionAdapter(RegistrationPluginSession & session)
  : session_(session) {}

  RegistrationPluginSessionAdapter(const RegistrationPluginSessionAdapter &) = delete;
  RegistrationPluginSessionAdapter & operator=(const RegistrationPluginSessionAdapter &) = delete;

  PluginMetadata metadata() const override {return session_.metadata();}
  Capabilities capabilities() const override {return session_.capabilities();}
  RegistrationRuntimeDescriptor registrationDescriptor() const override
  {
    return session_.descriptor();
  }

  bool configure(const ParameterMap &, std::string * error) override
  {
    if (error != nullptr) {
      *error = "registration plugin session adapter cannot reconfigure an activated session";
    }
    return false;
  }

  bool setInputTarget(const PointCloudConstPtr & target, std::string * error) override
  {
    return session_.setInputTarget(target, error);
  }

  AlignmentResult align(const AlignmentRequest & request) override
  {
    return session_.align(request);
  }

  void reset() noexcept override
  {
    session_.shutdown();
  }

private:
  RegistrationPluginSession & session_;
};

struct LoadResult
{
  std::shared_ptr<RegistrationPluginSession> session;
  LoadFailure failure;

  bool ok() const {return session != nullptr && failure.ok();}
  explicit operator bool() const {return ok();}
};

/**
 * Host-owned registration state slots used by startup activation.
 *
 * The resolver owns discovery, DSO loading, construction, and contract
 * validation.  This small transaction owns only the host-side hand-off: a
 * candidate session is prepared and validated while the currently active
 * session remains untouched, then all supplied slots are swapped at commit.
 * It deliberately does not claim to undo constructor/static-initializer side
 * effects inside an external DSO.  The rollback boundary is the host-owned
 * session/plugin/contract state (and callers must commit before exposing
 * subscriptions or publishers).
 */
struct RegistrationActivationSlots
{
  std::shared_ptr<RegistrationPluginSession> * session{nullptr};
  std::shared_ptr<RegistrationPlugin> * plugin{nullptr};
  Capabilities * capabilities{nullptr};
  TargetPolicy * target_policy{nullptr};
  CorrespondenceMetric * correspondence_metric{nullptr};
};

struct RegistrationActivationSnapshot
{
  std::shared_ptr<RegistrationPluginSession> session;
  std::shared_ptr<RegistrationPlugin> plugin;
  Capabilities capabilities;
  TargetPolicy target_policy{TargetPolicy::kAcceptHostPrepared};
  CorrespondenceMetric correspondence_metric{CorrespondenceMetric::kUnavailable};
};

class RegistrationActivationTransaction final
{
public:
  using Validator = std::function<LoadFailure(const RegistrationActivationSnapshot &)>;

  explicit RegistrationActivationTransaction(const RegistrationActivationSlots & slots)
  : slots_(slots), previous_(snapshotFromSlots(slots)) {}

  RegistrationActivationTransaction(const RegistrationActivationTransaction &) = delete;
  RegistrationActivationTransaction & operator=(const RegistrationActivationTransaction &) = delete;

  ~RegistrationActivationTransaction()
  {
    // Explicit reset order is part of the loader lease contract: release the
    // typed plugin pointer before releasing its session/ClassLoader lease.
    committed_state_.plugin.reset();
    committed_state_.session.reset();
    candidate_.plugin.reset();
    candidate_.session.reset();
  }

  bool prepare(
    const std::shared_ptr<RegistrationPluginSession> & candidate,
    LoadFailure * failure)
  {
    if (prepared_) {
      return reject(failure, LoadFailureCode::kInvalidRequest,
        "registration activation prepare called more than once");
    }
    if (!slots_.session || !slots_.plugin) {
      return reject(failure, LoadFailureCode::kInvalidRequest,
        "registration activation requires session and plugin slots");
    }
    if (!candidate || !candidate->plugin()) {
      return reject(failure, LoadFailureCode::kInvalidRequest,
        "registration activation candidate session/plugin is null");
    }
    candidate_.session = candidate;
    candidate_.plugin = candidate->plugin();
    candidate_.capabilities = candidate->capabilities();
    candidate_.target_policy = candidate->capabilities().targetPolicy();
    candidate_.correspondence_metric = candidate->capabilities().correspondenceMetric();
    prepared_ = true;
    return true;
  }

  bool validate(const Validator & validator, LoadFailure * failure)
  {
    if (!prepared_) {
      return reject(failure, LoadFailureCode::kInvalidRequest,
        "registration activation validate called before prepare");
    }
    if (!validator) {
      return reject(failure, LoadFailureCode::kInvalidRequest,
        "registration activation validator is empty");
    }
    LoadFailure result;
    try {
      result = validator(candidate_);
    } catch (const std::exception & exception) {
      return reject(failure, LoadFailureCode::kPluginException,
        std::string("registration activation validator threw: ") + exception.what());
    } catch (...) {
      return reject(failure, LoadFailureCode::kPluginException,
        "registration activation validator threw an unknown exception");
    }
    if (!result.ok()) {
      if (failure != nullptr) {
        *failure = result;
      }
      return false;
    }
    validated_ = true;
    if (failure != nullptr) {
      *failure = LoadFailure{};
    }
    return true;
  }

  bool commit(LoadFailure * failure)
  {
    if (!prepared_ || !validated_) {
      return reject(failure, LoadFailureCode::kInvalidRequest,
        "registration activation commit requires successful prepare and validate");
    }
    if (committed_) {
      return reject(failure, LoadFailureCode::kInvalidRequest,
        "registration activation commit called more than once");
    }
    if (rolled_back_) {
      return reject(failure, LoadFailureCode::kInvalidRequest,
        "registration activation cannot commit after rollback");
    }
    if (!activeStateUnchanged()) {
      return reject(failure, LoadFailureCode::kInvalidRequest,
        "registration activation active state changed during validation");
    }
    committed_state_ = candidate_;
    has_committed_state_ = true;

    // shared_ptr::swap is noexcept.  Swap plugin before session so the old
    // session still owns its loader while the old plugin slot is withdrawn.
    slots_.plugin->swap(candidate_.plugin);
    slots_.session->swap(candidate_.session);
    if (slots_.capabilities != nullptr) {
      *slots_.capabilities = candidate_.capabilities;
    }
    if (slots_.target_policy != nullptr) {
      *slots_.target_policy = candidate_.target_policy;
    }
    if (slots_.correspondence_metric != nullptr) {
      *slots_.correspondence_metric = candidate_.correspondence_metric;
    }
    committed_ = true;
    if (failure != nullptr) {
      *failure = LoadFailure{};
    }
    return true;
  }

  /**
   * Undo a committed host-owned hand-off while the caller is still inside the
   * startup barrier.  This is used when publisher/subscription creation fails
   * after commit.  It is fail-closed if another owner changed the active
   * slots in the meantime; callers must then tear down the whole startup
   * object rather than guessing which state is authoritative.
   */
  bool rollback(LoadFailure * failure)
  {
    if (!committed_) {
      if (failure != nullptr) {
        *failure = LoadFailure{};
      }
      return true;
    }
    if (!has_committed_state_ || !activeStateIsCommitted()) {
      return reject(failure, LoadFailureCode::kInvalidRequest,
        "registration activation rollback found an unexpected active state");
    }

    slots_.plugin->swap(candidate_.plugin);
    slots_.session->swap(candidate_.session);
    if (slots_.capabilities != nullptr) {
      *slots_.capabilities = previous_.capabilities;
    }
    if (slots_.target_policy != nullptr) {
      *slots_.target_policy = previous_.target_policy;
    }
    if (slots_.correspondence_metric != nullptr) {
      *slots_.correspondence_metric = previous_.correspondence_metric;
    }
    committed_ = false;
    rolled_back_ = true;
    committed_state_.plugin.reset();
    committed_state_.session.reset();
    has_committed_state_ = false;
    if (failure != nullptr) {
      *failure = LoadFailure{};
    }
    return true;
  }

  bool committed() const {return committed_;}
  bool prepared() const {return prepared_;}
  bool validated() const {return validated_;}
  const RegistrationActivationSnapshot & candidate() const {return candidate_;}
  const RegistrationActivationSnapshot & previous() const {return previous_;}

private:
  static RegistrationActivationSnapshot snapshotFromSlots(
    const RegistrationActivationSlots & slots)
  {
    RegistrationActivationSnapshot snapshot;
    if (slots.session != nullptr) {
      snapshot.session = *slots.session;
    }
    if (slots.plugin != nullptr) {
      snapshot.plugin = *slots.plugin;
    }
    if (slots.capabilities != nullptr) {
      snapshot.capabilities = *slots.capabilities;
    }
    if (slots.target_policy != nullptr) {
      snapshot.target_policy = *slots.target_policy;
    }
    if (slots.correspondence_metric != nullptr) {
      snapshot.correspondence_metric = *slots.correspondence_metric;
    }
    return snapshot;
  }

  static bool reject(LoadFailure * failure, LoadFailureCode code, const std::string & message)
  {
    if (failure != nullptr) {
      *failure = LoadFailure{code, message};
    }
    return false;
  }

  bool activeStateUnchanged() const
  {
    if (*slots_.session != previous_.session || *slots_.plugin != previous_.plugin) {
      return false;
    }
    if (
      slots_.capabilities != nullptr &&
      (slots_.capabilities->bits() != previous_.capabilities.bits() ||
      slots_.capabilities->targetPolicy() != previous_.capabilities.targetPolicy() ||
      slots_.capabilities->correspondenceMetric() !=
      previous_.capabilities.correspondenceMetric() ||
      slots_.capabilities->threadModel() != previous_.capabilities.threadModel()))
    {
      return false;
    }
    if (slots_.target_policy != nullptr && *slots_.target_policy != previous_.target_policy) {
      return false;
    }
    if (
      slots_.correspondence_metric != nullptr &&
      *slots_.correspondence_metric != previous_.correspondence_metric)
    {
      return false;
    }
    return true;
  }

  bool activeStateIsCommitted() const
  {
    if (*slots_.session != committed_state_.session || *slots_.plugin != committed_state_.plugin) {
      return false;
    }
    if (
      slots_.capabilities != nullptr &&
      (slots_.capabilities->bits() != committed_state_.capabilities.bits() ||
      slots_.capabilities->targetPolicy() != committed_state_.capabilities.targetPolicy() ||
      slots_.capabilities->correspondenceMetric() != committed_state_.correspondence_metric ||
      slots_.capabilities->threadModel() != committed_state_.capabilities.threadModel()))
    {
      return false;
    }
    if (slots_.target_policy != nullptr &&
      *slots_.target_policy != committed_state_.target_policy)
    {
      return false;
    }
    if (
      slots_.correspondence_metric != nullptr &&
      *slots_.correspondence_metric != committed_state_.correspondence_metric)
    {
      return false;
    }
    return true;
  }

  RegistrationActivationSlots slots_;
  RegistrationActivationSnapshot previous_;
  RegistrationActivationSnapshot candidate_;
  RegistrationActivationSnapshot committed_state_;
  bool prepared_{false};
  bool validated_{false};
  bool committed_{false};
  bool rolled_back_{false};
  bool has_committed_state_{false};
};

/**
 * Shell-side discovery and startup validation for RegistrationPlugin.
 *
 * This class is intentionally not part of the ROS-free registration
 * interface.  A live node or an offline runner owns one loader, resolves one
 * class at startup, and injects the resulting session into its core.
 */
class RegistrationPluginLoader final
{
public:
  // Keep the original one-argument symbol for binary-compatible consumers;
  // the custom XML path overload is additive and used by isolated tests.
  explicit RegistrationPluginLoader(
    const std::string & interface_package = kRegistrationPluginBasePackage);
  RegistrationPluginLoader(
    const std::string & interface_package,
    const std::vector<std::string> & plugin_xml_paths);
  ~RegistrationPluginLoader() = default;

  RegistrationPluginLoader(const RegistrationPluginLoader &) = delete;
  RegistrationPluginLoader & operator=(const RegistrationPluginLoader &) = delete;

  LoadResult load(const LoadRequest & request) const;
  std::vector<std::string> availableClasses() const;
  const std::string & initializationError() const {return initialization_error_;}

private:
  std::shared_ptr<RegistrationPluginClassLoader> class_loader_;
  std::string initialization_error_;
};

/**
 * Resolves an explicit class ID from either a host-resident factory registry
 * or pluginlib.  Host IDs and external IDs are disjoint by construction; a
 * collision is a startup error rather than an opportunity for shadowing.
 */
class RegistrationResolver final
{
public:
  explicit RegistrationResolver(
    std::vector<HostBuiltinRegistration> host_builtins = {},
    const std::string & interface_package = kRegistrationPluginBasePackage);
  ~RegistrationResolver() = default;

  RegistrationResolver(const RegistrationResolver &) = delete;
  RegistrationResolver & operator=(const RegistrationResolver &) = delete;

  LoadResult resolve(const LoadRequest & request) const;
  std::vector<std::string> availableClasses() const;
  const std::string & initializationError() const {return initialization_error_;}

private:
  std::unique_ptr<RegistrationPluginLoader> plugin_loader_;
  std::vector<HostBuiltinRegistration> host_builtins_;
  std::string initialization_error_;
  std::string plugin_loader_error_;
  LoadFailureCode initialization_failure_code_{LoadFailureCode::kNone};
};

}  // namespace shell
}  // namespace registration
}  // namespace plugins
}  // namespace lidarslam

#endif  // LIDARSLAM_REGISTRATION_LOADER__REGISTRATION_PLUGIN_LOADER_HPP_
