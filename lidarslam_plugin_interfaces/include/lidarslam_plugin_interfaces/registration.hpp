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

private:
  std::uint64_t bits_{0};
  TargetPolicy target_policy_{TargetPolicy::kAcceptHostPrepared};
  CorrespondenceMetric correspondence_metric_{CorrespondenceMetric::kUnavailable};
  ThreadModel thread_model_{ThreadModel::kSerializedOwner};
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
  virtual void reset() noexcept = 0;
};

}  // namespace registration
}  // namespace plugins
}  // namespace lidarslam

#endif  // LIDARSLAM_PLUGIN_INTERFACES__REGISTRATION_HPP_
