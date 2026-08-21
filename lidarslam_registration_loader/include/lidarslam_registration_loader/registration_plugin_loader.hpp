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
#include <string>
#include <utility>
#include <vector>

#include <pluginlib/class_loader.hpp>

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
};

struct LoadFailure
{
  LoadFailureCode code{LoadFailureCode::kNone};
  std::string message;

  bool ok() const {return code == LoadFailureCode::kNone;}
};

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

  std::shared_ptr<RegistrationPlugin> plugin() const {return plugin_;}
  RegistrationPlugin * get() const {return plugin_.get();}
  const ResolvedRegistrationProvenance & provenance() const {return provenance_;}
  BackendKind backendKind() const {return provenance_.backend_kind;}
  const PluginMetadata & metadata() const {return provenance_.metadata;}
  const Capabilities & capabilities() const {return provenance_.capabilities;}
  const ParameterMap & parameters() const {return provenance_.parameters;}
  const std::string & classId() const {return provenance_.class_id;}
  const std::string & libraryPath() const {return provenance_.library_path;}
  const std::string & pluginManifestPath() const {return provenance_.plugin_manifest_path;}

private:
  friend class RegistrationPluginLoader;
  friend class RegistrationResolver;

  RegistrationPluginSession(
    std::shared_ptr<RegistrationPluginClassLoader> loader,
    std::shared_ptr<RegistrationPlugin> plugin,
    BackendKind backend_kind,
    const PluginMetadata & metadata,
    const Capabilities & capabilities,
    ParameterMap parameters,
    std::string class_id,
    std::string library_path,
    std::string plugin_manifest_path)
  : loader_(std::move(loader)),
    plugin_(std::move(plugin)),
    provenance_{
      backend_kind,
      std::move(class_id),
      metadata,
      capabilities,
      std::move(parameters),
      std::move(library_path),
      std::move(plugin_manifest_path)} {}

  // Declaration order is deliberate: plugin_ is destroyed before loader_.
  std::shared_ptr<RegistrationPluginClassLoader> loader_;
  std::shared_ptr<RegistrationPlugin> plugin_;
  ResolvedRegistrationProvenance provenance_;
};

struct LoadResult
{
  std::shared_ptr<RegistrationPluginSession> session;
  LoadFailure failure;

  bool ok() const {return session != nullptr && failure.ok();}
  explicit operator bool() const {return ok();}
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
  explicit RegistrationPluginLoader(
    const std::string & interface_package = kRegistrationPluginBasePackage);
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
