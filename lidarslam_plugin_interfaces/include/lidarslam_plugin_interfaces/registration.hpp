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
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef LIDARSLAM_PLUGIN_INTERFACES__REGISTRATION_HPP_
#define LIDARSLAM_PLUGIN_INTERFACES__REGISTRATION_HPP_

#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>

#include <Eigen/Core>  // NOLINT(build/include_order)
#include <pcl/point_cloud.h>  // NOLINT(build/include_order)
#include <pcl/point_types.h>  // NOLINT(build/include_order)

namespace lidarslam
{
namespace plugins
{
namespace registration
{

struct ApiVersion
{
  std::uint16_t major{0};
  std::uint16_t minor{0};
};

constexpr ApiVersion kHostApiVersion{1, 0};

inline bool isApiCompatible(const ApiVersion & host, const ApiVersion & plugin)
{
  return host.major == plugin.major && plugin.minor <= host.minor;
}

enum class Capability : std::uint64_t
{
  kInitialGuess = 1ULL << 0,
  kRotationPrior = 1ULL << 1,
  kTranslationPrior = 1ULL << 2,
  kMaximumCorrespondenceDistance = 1ULL << 3,
  kMeanCorrespondenceDistance = 1ULL << 4,
  kCovariance = 1ULL << 5,
  kDeterministic = 1ULL << 6,
  kAlignedSource = 1ULL << 7,
};

enum class TargetPolicy
{
  kAcceptHostPrepared,
  kRequiresRawTarget,
  kPluginPreprocessesTarget,
};

enum class CorrespondenceMetric
{
  kUnavailable,
  kMeanDistance,
  kSquareRootFitnessProxy,
};

enum class ThreadModel
{
  kSerializedOwner,
  kReentrant,
};

// Cancellation is an explicit implementation contract.  A
// kNonInterruptibleAlign provider may finish an already-entered align call
// after cancel(); the session only guarantees admission/post-call
// checkpoints and waits for quiescence before reset().  A kCooperativeCancel
// provider must implement requestCancel() and observe it at its own bounded
// checkpoints.
enum class CancellationModel
{
  kNonInterruptibleAlign,
  kCooperativeCancel,
};

class Capabilities
{
public:
  Capabilities() = default;
  explicit Capabilities(std::uint64_t bits)
  : bits_(bits) {}

  Capabilities & add(Capability capability)
  {
    bits_ |= static_cast<std::uint64_t>(capability);
    return *this;
  }

  bool has(Capability capability) const
  {
    return (bits_ & static_cast<std::uint64_t>(capability)) != 0U;
  }

  std::uint64_t bits() const {return bits_;}

  Capabilities & setTargetPolicy(TargetPolicy policy)
  {
    target_policy_ = policy;
    return *this;
  }

  TargetPolicy targetPolicy() const {return target_policy_;}

  Capabilities & setCorrespondenceMetric(CorrespondenceMetric metric)
  {
    correspondence_metric_ = metric;
    return *this;
  }

  CorrespondenceMetric correspondenceMetric() const {return correspondence_metric_;}

  Capabilities & setThreadModel(ThreadModel model)
  {
    thread_model_ = model;
    return *this;
  }

  ThreadModel threadModel() const {return thread_model_;}

  Capabilities & setCancellationModel(CancellationModel model)
  {
    cancellation_model_ = model;
    return *this;
  }

  CancellationModel cancellationModel() const {return cancellation_model_;}

private:
  std::uint64_t bits_{0};
  TargetPolicy target_policy_{TargetPolicy::kAcceptHostPrepared};
  CorrespondenceMetric correspondence_metric_{CorrespondenceMetric::kUnavailable};
  ThreadModel thread_model_{ThreadModel::kSerializedOwner};
  CancellationModel cancellation_model_{CancellationModel::kNonInterruptibleAlign};
};

class ParameterValue
{
public:
  enum class Type {kBool, kInteger, kDouble, kString};

  explicit ParameterValue(bool value)
  : type_(Type::kBool), bool_value_(value) {}
  explicit ParameterValue(std::int64_t value)
  : type_(Type::kInteger), integer_value_(value) {}
  explicit ParameterValue(double value)
  : type_(Type::kDouble), double_value_(value) {}
  explicit ParameterValue(std::string value)
  : type_(Type::kString), string_value_(std::move(value)) {}
  explicit ParameterValue(const char * value)
  : type_(Type::kString), string_value_(value == nullptr ? "" : value) {}

  Type type() const {return type_;}

  bool asBool() const
  {
    require(Type::kBool);
    return bool_value_;
  }

  std::int64_t asInteger() const
  {
    require(Type::kInteger);
    return integer_value_;
  }

  double asDouble() const
  {
    require(Type::kDouble);
    return double_value_;
  }

  const std::string & asString() const
  {
    require(Type::kString);
    return string_value_;
  }

private:
  void require(Type expected) const
  {
    if (type_ != expected) {
      throw std::logic_error("registration parameter type mismatch");
    }
  }

  Type type_;
  bool bool_value_{false};
  std::int64_t integer_value_{0};
  double double_value_{0.0};
  std::string string_value_;
};

using ParameterMap = std::map<std::string, ParameterValue>;
using PointT = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointT>;
using PointCloudConstPtr = PointCloud::ConstPtr;

struct PluginMetadata
{
  std::string class_id;
  std::string implementation_version;
  std::string license;
  ApiVersion api_version;
};

// The descriptor is deliberately a separate opt-in interface rather than a
// new virtual on RegistrationPlugin.  That keeps the C++14 vtable unchanged
// for old binaries while the loader can reject old external DSOs from their
// pre-instantiation sidecar.  A plugin that is accepted by the new loader
// must implement RegistrationPluginDescriptorProvider and return this exact
// logical identity after configure().
constexpr const char * kRegistrationDescriptorSchema =
  "lidarslam-registration-runtime-descriptor-v1";
constexpr std::uint32_t kRegistrationDescriptorSchemaVersion = 1U;
constexpr const char * kRegistrationAbiEpoch = "lidarslam.registration.cpp14.abi1";
// This is a canonical contract digest, not a claim that one header SHA
// proves a complete C++ ABI.  The supported ABI scope and toolchain tag are
// intentionally checked separately and are documented by the shell.
constexpr const char * kRegistrationInterfaceContractSha256 =
  "de43eef540d5e6836ed41d69074590641b810acc0f6661eb73c4f0e90d4c369c";

struct RegistrationConfigSchemaIdentity
{
  std::string id;
  std::uint32_t version{0U};
  std::string sha256;
};

struct RegistrationRuntimeDescriptor
{
  std::string schema;
  std::uint32_t schema_version{0U};
  std::string class_id;
  ApiVersion api_min;
  ApiVersion api_max;
  std::uint64_t required_capability_bits{0U};
  std::uint64_t optional_capability_bits{0U};
  TargetPolicy target_policy{TargetPolicy::kAcceptHostPrepared};
  CorrespondenceMetric correspondence_metric{CorrespondenceMetric::kUnavailable};
  ThreadModel thread_model{ThreadModel::kSerializedOwner};
  CancellationModel cancellation_model{CancellationModel::kNonInterruptibleAlign};
  std::string abi_epoch;
  std::string toolchain_tag;
  std::string config_schema_id;
  std::uint32_t config_schema_version{0U};
  std::string config_schema_sha256;
  std::string interface_contract_sha256;

  bool logicallyComplete() const
  {
    return schema == kRegistrationDescriptorSchema &&
           schema_version == kRegistrationDescriptorSchemaVersion &&
           !class_id.empty() &&
           (cancellation_model == CancellationModel::kNonInterruptibleAlign ||
           cancellation_model == CancellationModel::kCooperativeCancel) &&
           !abi_epoch.empty() &&
           !toolchain_tag.empty() &&
           !config_schema_id.empty() &&
           config_schema_version > 0U &&
           config_schema_sha256.size() == 64U &&
           interface_contract_sha256.size() == 64U;
  }
};

inline RegistrationConfigSchemaIdentity registrationConfigSchemaForClassId(
  const std::string & class_id)
{
  RegistrationConfigSchemaIdentity identity;
  const bool is_fast_gicp = class_id.find("FastGicp") != std::string::npos ||
    class_id.find("FastVGicp") != std::string::npos;
  const bool is_legacy = class_id.find("LegacyPcl") != std::string::npos ||
    class_id.find("LegacyBackendGicp") != std::string::npos;
  const bool is_small_gicp = class_id.find("SmallGicp") != std::string::npos ||
    class_id.find("SmallVGicp") != std::string::npos;
  const bool is_gicp = class_id.find("Gicp") != std::string::npos ||
    class_id.find("GICP") != std::string::npos;
  if (class_id.find("NdtOmp") != std::string::npos) {
    identity.id = "lidarslam.registration.ndt.v1";
    identity.sha256 = "0b38bd0c73d94d9d01fb6c656aec6210f434b65f8cf88f1aea9e61e433c9ba44";
  } else if (is_fast_gicp) {
    identity.id = "lidarslam.registration.fast_gicp.v1";
    identity.sha256 = "af44393771764fb1eff20d75743995995134bd35911b93d49839af14406b7bb5";
  } else if (is_legacy || is_small_gicp) {
    identity.id = is_legacy ?
      "lidarslam.registration.legacy_pcl.v1" : "lidarslam.registration.small_gicp.v1";
    identity.sha256 = is_legacy ?
      "fb631ab4420b5a14f72dfaa6339e56249347f0fdcf097551e9633eec2a309317" :
      "c5d1bce8fbe885537121606949bc1c0e7ae5dcd5e5279ea9e28581f83ff995f5";
  } else if (is_gicp) {
    identity.id = "lidarslam.registration.gicp.v1";
    identity.sha256 = "6c35ace8983dcfdc86d609ed9347c4743638b3c51665bec43b9f5a59b25b6f19";
  } else if (class_id.find("template") != std::string::npos) {
    identity.id = "lidarslam.registration.template.v1";
    identity.sha256 = "5b9eff0fabe6c6110fbc9ab2790715c39032f0a2e857bd34a2d0abb95cf8aab3";
  } else {
    identity.id = "lidarslam.registration.generic.v1";
    identity.sha256 = "4d2d0f7f87d168fa5a04b22d1f3fbfc468c3576ae1e4f529dc8b6a7fca8a7c30";
  }
  identity.version = 1U;
  return identity;
}

inline std::string registrationToolchainTag()
{
  std::string tag;
#if defined(__clang__)
  tag = "clang-" + std::to_string(__clang_major__);
#elif defined(__GNUC__)
  tag = "gcc-" + std::to_string(__GNUC__);
#elif defined(_MSC_VER)
  tag = "msvc-" + std::to_string(_MSC_VER);
#else
  tag = "unknown-compiler";
#endif
#if defined(_GLIBCXX_USE_CXX11_ABI)
  tag += ";libstdcxx-cxx11-abi-" +
    std::to_string(static_cast<int>(_GLIBCXX_USE_CXX11_ABI));
#else
  tag += ";stdlib-abi-unknown";
#endif
  return tag;
}

inline RegistrationRuntimeDescriptor makeRegistrationRuntimeDescriptor(
  const PluginMetadata & metadata,
  const Capabilities & capabilities,
  const std::uint64_t required_capability_bits,
  const std::uint64_t optional_capability_bits,
  const RegistrationConfigSchemaIdentity & config_schema)
{
  RegistrationRuntimeDescriptor descriptor;
  descriptor.schema = kRegistrationDescriptorSchema;
  descriptor.schema_version = kRegistrationDescriptorSchemaVersion;
  descriptor.class_id = metadata.class_id;
  descriptor.api_min = metadata.api_version;
  descriptor.api_max = metadata.api_version;
  descriptor.required_capability_bits = required_capability_bits;
  descriptor.optional_capability_bits = optional_capability_bits;
  descriptor.target_policy = capabilities.targetPolicy();
  descriptor.correspondence_metric = capabilities.correspondenceMetric();
  descriptor.thread_model = capabilities.threadModel();
  descriptor.cancellation_model = capabilities.cancellationModel();
  descriptor.abi_epoch = kRegistrationAbiEpoch;
  descriptor.toolchain_tag = registrationToolchainTag();
  descriptor.config_schema_id = config_schema.id;
  descriptor.config_schema_version = config_schema.version;
  descriptor.config_schema_sha256 = config_schema.sha256;
  descriptor.interface_contract_sha256 = kRegistrationInterfaceContractSha256;
  return descriptor;
}

class RegistrationPluginDescriptorProvider
{
public:
  virtual ~RegistrationPluginDescriptorProvider() = default;
  virtual RegistrationRuntimeDescriptor registrationDescriptor() const = 0;
};

struct RotationPrior
{
  bool enabled{false};
  Eigen::Vector3d roll_pitch_yaw{Eigen::Vector3d::Zero()};
  double weight{0.0};
  bool roll_pitch_only{false};
};

struct TranslationPrior
{
  bool enabled{false};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d weights{Eigen::Vector3d::Zero()};
};

struct AlignmentRequest
{
  PointCloudConstPtr source;
  bool initial_guess_enabled{true};
  Eigen::Matrix4f initial_guess{Eigen::Matrix4f::Identity()};
  RotationPrior rotation_prior;
  TranslationPrior translation_prior;
  bool maximum_correspondence_distance_enabled{false};
  double maximum_correspondence_distance{0.0};
};

enum class FailureCode
{
  kNone,
  kNotConfigured,
  kInvalidInput,
  kUnsupportedCapability,
  kAlignmentFailed,
  kInternalError,
  // Appended so existing result-code values remain stable for C++14
  // consumers.  The shell uses this value when a session has been
  // cancelled/shut down before an operation reaches the plugin.
  kCancelled,
};

struct AlignmentDiagnostics
{
  int iterations{-1};
  bool mean_correspondence_distance_valid{false};
  double mean_correspondence_distance{0.0};
  bool covariance_valid{false};
  Eigen::Matrix<double, 6, 6> covariance{Eigen::Matrix<double, 6, 6>::Zero()};
  std::string detail;
};

struct AlignmentResult
{
  bool converged{false};
  Eigen::Matrix4f final_transformation{Eigen::Matrix4f::Identity()};
  PointCloud::Ptr aligned_source;
  double fitness_score{0.0};
  FailureCode failure{FailureCode::kAlignmentFailed};
  AlignmentDiagnostics diagnostics;
};

inline FailureCode validateRequest(
  const AlignmentRequest & request, const Capabilities & capabilities)
{
  if (!request.source || request.source->empty()) {
    return FailureCode::kInvalidInput;
  }
  if (request.initial_guess_enabled && !capabilities.has(Capability::kInitialGuess)) {
    return FailureCode::kUnsupportedCapability;
  }
  if (request.rotation_prior.enabled && !capabilities.has(Capability::kRotationPrior)) {
    return FailureCode::kUnsupportedCapability;
  }
  if (request.translation_prior.enabled && !capabilities.has(Capability::kTranslationPrior)) {
    return FailureCode::kUnsupportedCapability;
  }
  if (
    request.maximum_correspondence_distance_enabled &&
    !capabilities.has(Capability::kMaximumCorrespondenceDistance))
  {
    return FailureCode::kUnsupportedCapability;
  }
  return FailureCode::kNone;
}

class RegistrationPlugin
{
public:
  virtual ~RegistrationPlugin() = default;

  virtual PluginMetadata metadata() const = 0;
  virtual Capabilities capabilities() const = 0;
  virtual bool configure(const ParameterMap & parameters, std::string * error) = 0;
  virtual bool setInputTarget(const PointCloudConstPtr & target, std::string * error) = 0;
  virtual AlignmentResult align(const AlignmentRequest & request) = 0;
  // A cooperative provider must override this method and make its align
  // implementation observe the request at bounded internal checkpoints.
  // The default is deliberately a no-op for non-interruptible providers;
  // their capabilities must retain kNonInterruptibleAlign.
  virtual void requestCancel() noexcept {}
  virtual void reset() noexcept = 0;
};

}  // namespace registration
}  // namespace plugins
}  // namespace lidarslam

#endif  // LIDARSLAM_PLUGIN_INTERFACES__REGISTRATION_HPP_
