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
//
// This file is an implementation include.  It is intentionally kept separate
// from the public C++14 registration contract so a shell can instantiate the
// pclomp templates in the same translation unit as its legacy backend when a
// bitwise compatibility gate requires that build mode.

#ifndef LIDARSLAM_DEFAULT_PLUGINS__NDT_OMP_REGISTRATION_IMPL_IPP_
#define LIDARSLAM_DEFAULT_PLUGINS__NDT_OMP_REGISTRATION_IMPL_IPP_

#include "lidarslam_default_plugins/ndt_omp_registration.hpp"

#include <cfloat>
#include <climits>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <pclomp/ndt_omp.h>  // NOLINT(build/include_order)
// ndt_omp_ros2 is header-only at the template call sites.  Keep these includes
// here (rather than in the public adapter header) so the public API remains
// independent of pclomp implementation details.
#include <pclomp/ndt_omp_impl.hpp>  // NOLINT(build/include_order)
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>  // NOLINT(build/include_order)

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
using registration::RotationPrior;
using registration::TargetPolicy;
using registration::ThreadModel;
using registration::TranslationPrior;
using registration::kHostApiVersion;
using registration::validateRequest;

constexpr float kDefaultResolution = 5.0F;
constexpr double kDefaultTransformationEpsilon = 0.01;
constexpr int kDefaultMaximumIterations = 35;
constexpr double kDefaultStepSize = 0.1;
constexpr double kDefaultOutlierRatio = 0.55;
constexpr int kDefaultNumThreads = 0;
constexpr int kDefaultTargetCellCacheCapacity = 0;
constexpr const char * kDirect7 = "DIRECT7";

struct NdtConfiguration
{
  float resolution{kDefaultResolution};
  double transformation_epsilon{kDefaultTransformationEpsilon};
  int maximum_iterations{kDefaultMaximumIterations};
  double step_size{kDefaultStepSize};
  double outlier_ratio{kDefaultOutlierRatio};
  int num_threads{kDefaultNumThreads};
  int target_cell_cache_capacity{kDefaultTargetCellCacheCapacity};
};

void setError(std::string * error, const std::string & message)
{
  if (error != nullptr) {
    *error = message;
  }
}

std::string parameterError(const std::string & key, const std::string & detail)
{
  return "invalid NDT parameter '" + key + "': " + detail;
}

bool finite(double value)
{
  return std::isfinite(value) != 0;
}

bool finitePointCloud(const PointCloudConstPtr & cloud)
{
  if (!cloud || cloud->empty()) {
    return false;
  }
  for (const PointT & point : cloud->points) {
    if (
      !finite(static_cast<double>(point.x)) ||
      !finite(static_cast<double>(point.y)) ||
      !finite(static_cast<double>(point.z)) ||
      !finite(static_cast<double>(point.intensity)))
    {
      return false;
    }
  }
  return true;
}

bool parseConfiguration(
  const ParameterMap & parameters, NdtConfiguration * configuration, std::string * error)
{
  if (configuration == nullptr) {
    setError(error, "internal error: null NDT configuration");
    return false;
  }

  *configuration = NdtConfiguration();
  for (const auto & entry : parameters) {
    const std::string & key = entry.first;
    try {
      if (key == "resolution") {
        const double value = entry.second.asDouble();
        if (!finite(value) || value <= 0.0 || value > static_cast<double>(FLT_MAX)) {
          setError(error, parameterError(key, "must be finite and greater than zero"));
          return false;
        }
        const float converted = static_cast<float>(value);
        if (!finite(static_cast<double>(converted)) || converted <= 0.0F) {
          setError(error, parameterError(key, "must be representable as a positive float"));
          return false;
        }
        configuration->resolution = converted;
      } else if (key == "transformation_epsilon") {
        const double value = entry.second.asDouble();
        if (!finite(value) || value <= 0.0) {
          setError(error, parameterError(key, "must be finite and greater than zero"));
          return false;
        }
        configuration->transformation_epsilon = value;
      } else if (key == "maximum_iterations") {
        const std::int64_t value = entry.second.asInteger();
        if (value < 1 || value > static_cast<std::int64_t>(INT_MAX)) {
          setError(error, parameterError(key, "must be an integer in [1, INT_MAX]"));
          return false;
        }
        configuration->maximum_iterations = static_cast<int>(value);
      } else if (key == "step_size") {
        const double value = entry.second.asDouble();
        if (!finite(value) || value <= 0.0) {
          setError(error, parameterError(key, "must be finite and greater than zero"));
          return false;
        }
        configuration->step_size = value;
      } else if (key == "outlier_ratio") {
        const double value = entry.second.asDouble();
        if (!finite(value) || value <= 0.0 || value >= 1.0) {
          setError(error, parameterError(key, "must be finite and strictly between zero and one"));
          return false;
        }
        configuration->outlier_ratio = value;
      } else if (key == "num_threads") {
        const std::int64_t value = entry.second.asInteger();
        if (value < 0 || value > static_cast<std::int64_t>(INT_MAX)) {
          setError(error, parameterError(key, "must be an integer in [0, INT_MAX]"));
          return false;
        }
        configuration->num_threads = static_cast<int>(value);
      } else if (key == "target_cell_cache_capacity") {
        const std::int64_t value = entry.second.asInteger();
        if (value < 0 || value > static_cast<std::int64_t>(INT_MAX)) {
          setError(error, parameterError(key, "must be an integer in [0, INT_MAX]"));
          return false;
        }
        configuration->target_cell_cache_capacity = static_cast<int>(value);
      } else if (key == "neighborhood_search_method") {
        if (entry.second.asString() != kDirect7) {
          setError(error, parameterError(key, "only DIRECT7 is supported"));
          return false;
        }
      } else {
        setError(error, parameterError(key, "unknown key"));
        return false;
      }
    } catch (const std::exception & exception) {
      setError(error, parameterError(key, exception.what()));
      return false;
    } catch (...) {
      setError(error, parameterError(key, "type conversion failed"));
      return false;
    }
  }
  return true;
}

bool validRotationPrior(const RotationPrior & prior)
{
  return prior.enabled && prior.roll_pitch_yaw.allFinite() && finite(prior.weight) &&
         prior.weight > 0.0;
}

bool validTranslationPrior(const TranslationPrior & prior)
{
  return prior.enabled && prior.position.allFinite() && prior.weights.allFinite() &&
         (prior.weights.array() >= 0.0).all() && (prior.weights.array() > 0.0).any();
}

const char * failureName(FailureCode failure)
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

// Each cached target owns a fully configured NDT instance.  This small derived
// type gives the shell a concrete pointer type for those sibling instances;
// no pclomp target-grid representation crosses the public plugin boundary.
class CachedNdt final : public pclomp::NormalDistributionsTransform<PointT, PointT>
{
public:
  using Base = pclomp::NormalDistributionsTransform<PointT, PointT>;
  using Ptr = boost::shared_ptr<CachedNdt>;
};

CachedNdt::Ptr makeConfiguredNdt(const NdtConfiguration & configuration)
{
  CachedNdt::Ptr ndt(new CachedNdt());
  // Keep every NDT instance on the same construction/configuration path.  A
  // cache miss creates a sibling instance rather than copying target_cells_,
  // which makes a cache hit an O(1) pointer switch.
  ndt->setResolution(configuration.resolution);
  ndt->setTransformationEpsilon(configuration.transformation_epsilon);
  ndt->setMaximumIterations(configuration.maximum_iterations);
  ndt->setStepSize(configuration.step_size);
  ndt->setOulierRatio(configuration.outlier_ratio);
  ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
  if (configuration.num_threads > 0) {
    ndt->setNumThreads(configuration.num_threads);
  }
  return ndt;
}

class NdtTargetCellCache
{
public:
  using NdtPtr = CachedNdt::Ptr;

  void reset(const std::size_t capacity)
  {
    entries_.clear();
    capacity_ = capacity;
    hits_ = 0U;
    misses_ = 0U;
    evictions_ = 0U;
    next_sequence_ = 0U;
  }

  bool lookup(
    const PointCloudConstPtr & target,
    NdtPtr * ndt)
  {
    if (ndt == nullptr) {
      return false;
    }
    ndt->reset();
    if (capacity_ == 0U || !target) {
      return false;
    }
    for (auto & entry : entries_) {
      if (entry.target.get() == target.get()) {
        entry.last_use = next_sequence_++;
        *ndt = entry.ndt;
        ++hits_;
        return true;
      }
    }
    ++misses_;
    return false;
  }

  void insert(const PointCloudConstPtr & target, const NdtPtr & ndt)
  {
    if (capacity_ == 0U || !target || !ndt) {
      return;
    }
    for (auto & entry : entries_) {
      if (entry.target.get() == target.get()) {
        entry.target = target;
        entry.ndt = ndt;
        entry.last_use = next_sequence_++;
        return;
      }
    }

    while (entries_.size() >= capacity_) {
      evictLeastRecentlyUsed();
    }
    entries_.push_back(Entry{target, ndt, next_sequence_++});
  }

  bool enabled() const noexcept
  {
    return capacity_ != 0U;
  }

  TargetCellCacheStats stats() const noexcept
  {
    TargetCellCacheStats result;
    result.capacity = capacity_;
    result.size = entries_.size();
    result.hits = hits_;
    result.misses = misses_;
    result.evictions = evictions_;
    return result;
  }

private:
  struct Entry
  {
    PointCloudConstPtr target;
    NdtPtr ndt;
    std::uint64_t last_use{0U};
  };

  void evictLeastRecentlyUsed()
  {
    if (entries_.empty()) {
      return;
    }
    auto victim = entries_.begin();
    for (auto it = entries_.begin() + 1; it != entries_.end(); ++it) {
      if (it->last_use < victim->last_use) {
        victim = it;
      }
    }
    entries_.erase(victim);
    ++evictions_;
  }

  std::size_t capacity_{0U};
  std::vector<Entry> entries_;
  std::size_t hits_{0U};
  std::size_t misses_{0U};
  std::size_t evictions_{0U};
  std::uint64_t next_sequence_{0U};
};

}  // namespace

struct NdtOmpRegistration::Impl
{
  using Ndt = CachedNdt;

  NdtConfiguration configuration;
  typename Ndt::Ptr ndt;
  NdtTargetCellCache target_cell_cache;
  PointCloudConstPtr target;
  bool configured{false};
};

void NdtOmpRegistration::clearPerCallState(Impl * implementation) noexcept
{
  if (implementation == nullptr || !implementation->ndt) {
    return;
  }
  try {
    implementation->ndt->clearRotationPrior();
    implementation->ndt->clearTranslationPrior();
    implementation->ndt->setMaxCorrespondenceDistance(0.0);
  } catch (...) {
    // The pclomp setters are noexcept in supported versions.  reset/align
    // must nevertheless preserve the interface guarantee if that changes.
  }
}

struct NdtOmpRegistration::PerCallStateGuard
{
  explicit PerCallStateGuard(Impl * implementation)
  : implementation(implementation) {}

  ~PerCallStateGuard() noexcept
  {
    NdtOmpRegistration::clearPerCallState(implementation);
  }

  Impl * implementation;
};

NdtOmpRegistration::NdtOmpRegistration()
: impl_(new Impl())
{
}

NdtOmpRegistration::~NdtOmpRegistration() = default;

PluginMetadata NdtOmpRegistration::metadata() const
{
  PluginMetadata metadata;
  metadata.class_id = "lidarslam_default_plugins/NdtOmp";
  metadata.implementation_version = "1.0.0";
  metadata.license = "BSD-2-Clause";
  metadata.api_version = kHostApiVersion;
  return metadata;
}

RegistrationRuntimeDescriptor NdtOmpRegistration::registrationDescriptor() const
{
  const std::uint64_t required =
    static_cast<std::uint64_t>(Capability::kInitialGuess) |
    static_cast<std::uint64_t>(Capability::kRotationPrior) |
    static_cast<std::uint64_t>(Capability::kTranslationPrior) |
    static_cast<std::uint64_t>(Capability::kMaximumCorrespondenceDistance) |
    static_cast<std::uint64_t>(Capability::kMeanCorrespondenceDistance) |
    static_cast<std::uint64_t>(Capability::kAlignedSource);
  return makeRegistrationRuntimeDescriptor(
    metadata(), capabilities(), required,
    static_cast<std::uint64_t>(Capability::kDeterministic),
    registration::registrationConfigSchemaForClassId(metadata().class_id));
}

Capabilities NdtOmpRegistration::capabilities() const
{
  Capabilities capabilities;
  capabilities
  .add(Capability::kInitialGuess)
  .add(Capability::kRotationPrior)
  .add(Capability::kTranslationPrior)
  .add(Capability::kMaximumCorrespondenceDistance)
  .add(Capability::kMeanCorrespondenceDistance)
  .add(Capability::kAlignedSource)
  .setTargetPolicy(TargetPolicy::kRequiresRawTarget)
  .setCorrespondenceMetric(CorrespondenceMetric::kMeanDistance)
  .setThreadModel(ThreadModel::kSerializedOwner);
  // The R1 bitwise guarantee is only established for an explicitly fixed
  // single thread.  num_threads==0 delegates to OpenMP's machine-dependent
  // default, and fixed multi-thread runs are characterization evidence rather
  // than a cross-thread determinism promise.
  if (impl_ && impl_->configured && impl_->configuration.num_threads == 1) {
    capabilities.add(Capability::kDeterministic);
  }
  return capabilities;
}

bool NdtOmpRegistration::configure(const ParameterMap & parameters, std::string * error)
{
  if (!impl_) {
    setError(error, "internal error: adapter implementation is unavailable");
    return false;
  }
  if (impl_->configured) {
    setError(error, "NDT adapter is already configured; call reset() before configure()");
    return false;
  }

  NdtConfiguration configuration;
  if (!parseConfiguration(parameters, &configuration, error)) {
    return false;
  }

  try {
    Impl::Ndt::Ptr ndt = makeConfiguredNdt(configuration);
    impl_->configuration = configuration;
    impl_->ndt = ndt;
    impl_->target_cell_cache.reset(
      static_cast<std::size_t>(configuration.target_cell_cache_capacity));
    impl_->target.reset();
    impl_->configured = true;
    return true;
  } catch (const std::exception & exception) {
    impl_->target_cell_cache.reset(0U);
    impl_->ndt.reset();
    impl_->target.reset();
    impl_->configured = false;
    setError(error, std::string("failed to configure NDT: ") + exception.what());
    return false;
  } catch (...) {
    impl_->target_cell_cache.reset(0U);
    impl_->ndt.reset();
    impl_->target.reset();
    impl_->configured = false;
    setError(error, "failed to configure NDT: unknown exception");
    return false;
  }
}

bool NdtOmpRegistration::setInputTarget(const PointCloudConstPtr & target, std::string * error)
{
  if (!impl_ || !impl_->configured || !impl_->ndt) {
    setError(error, "NDT adapter is not configured");
    return false;
  }
  if (!finitePointCloud(target)) {
    setError(error, "NDT target must be non-empty and contain finite PointXYZI fields");
    return false;
  }

  try {
    clearPerCallState(impl_.get());
    Impl::Ndt::Ptr cached_ndt;
    if (impl_->target_cell_cache.lookup(target, &cached_ndt)) {
      // A hit switches to the already configured NDT instance.  No target
      // grid copy or setInputTarget/init call occurs on this path.
      impl_->ndt = cached_ndt;
    } else if (impl_->target_cell_cache.enabled()) {
      // Build a sibling through the exact same configure helper, then publish
      // it atomically into the LRU only after target-cell construction has
      // completed successfully.
      Impl::Ndt::Ptr fresh_ndt = makeConfiguredNdt(impl_->configuration);
      fresh_ndt->setInputTarget(target);
      impl_->target_cell_cache.insert(target, fresh_ndt);
      impl_->ndt = fresh_ndt;
    } else {
      impl_->ndt->setInputTarget(target);
    }
    impl_->target = target;
    return true;
  } catch (const std::exception & exception) {
    impl_->target.reset();
    setError(error, std::string("failed to set NDT target: ") + exception.what());
    return false;
  } catch (...) {
    impl_->target.reset();
    setError(error, "failed to set NDT target: unknown exception");
    return false;
  }
}

AlignmentResult NdtOmpRegistration::align(const AlignmentRequest & request)
{
  AlignmentResult result;
  if (!impl_ || !impl_->configured || !impl_->ndt) {
    result.failure = FailureCode::kNotConfigured;
    result.diagnostics.detail = failureName(result.failure);
    return result;
  }
  if (!impl_->target || impl_->target->empty()) {
    result.failure = FailureCode::kInvalidInput;
    result.diagnostics.detail = "NDT target has not been set";
    return result;
  }

  const FailureCode request_status = validateRequest(request, capabilities());
  if (request_status != FailureCode::kNone) {
    result.failure = request_status;
    result.diagnostics.detail = failureName(request_status);
    return result;
  }
  if (
    !finitePointCloud(request.source) ||
    (request.initial_guess_enabled && !request.initial_guess.allFinite()))
  {
    result.failure = FailureCode::kInvalidInput;
    result.diagnostics.detail = "source and initial guess must contain finite values";
    return result;
  }
  if (request.rotation_prior.enabled && !validRotationPrior(request.rotation_prior)) {
    result.failure = FailureCode::kInvalidInput;
    result.diagnostics.detail = "rotation prior must have finite rpy and positive weight";
    return result;
  }
  if (
    request.translation_prior.enabled && !validTranslationPrior(request.translation_prior))
  {
    result.failure = FailureCode::kInvalidInput;
    result.diagnostics.detail =
      "translation prior must have finite values, non-negative weights, and one positive weight";
    return result;
  }
  if (
    request.maximum_correspondence_distance_enabled &&
    (!finite(request.maximum_correspondence_distance) ||
    request.maximum_correspondence_distance <= 0.0))
  {
    result.failure = FailureCode::kInvalidInput;
    result.diagnostics.detail =
      "maximum correspondence distance must be finite and greater than zero";
    return result;
  }

  PerCallStateGuard state_guard(impl_.get());
  try {
    // Clear first as well as in the guard: this protects against a previous
    // pclomp exception or a caller that reused an object after an interrupted
    // call.  No request is allowed to inherit a prior or distance limit.
    clearPerCallState(impl_.get());
    impl_->ndt->setInputSource(request.source);
    if (request.rotation_prior.enabled) {
      impl_->ndt->setRotationPrior(
        request.rotation_prior.roll_pitch_yaw,
        request.rotation_prior.weight,
        request.rotation_prior.roll_pitch_only);
    }
    if (request.translation_prior.enabled) {
      impl_->ndt->setTranslationPrior(
        request.translation_prior.position,
        request.translation_prior.weights);
    }
    if (request.maximum_correspondence_distance_enabled) {
      impl_->ndt->setMaxCorrespondenceDistance(request.maximum_correspondence_distance);
    }

    PointCloud aligned;
    if (request.initial_guess_enabled) {
      impl_->ndt->align(aligned, request.initial_guess);
    } else {
      // Deliberately use pclomp/PCL's no-explicit-guess overload.  The shell,
      // not this adapter, owns any previous-pose policy.
      impl_->ndt->align(aligned);
    }

    result.converged = impl_->ndt->hasConverged();
    result.final_transformation = impl_->ndt->getFinalTransformation();
    result.fitness_score = impl_->ndt->getFitnessScore();
    result.diagnostics.iterations = impl_->ndt->getFinalNumIteration();
    result.diagnostics.mean_correspondence_distance =
      impl_->ndt->getLastMeanCorrespondenceDistance();
    result.diagnostics.mean_correspondence_distance_valid =
      finite(result.diagnostics.mean_correspondence_distance);

    if (
      aligned.empty() || aligned.size() != request.source->size() ||
      !result.final_transformation.allFinite() || !finite(result.fitness_score) ||
      !result.diagnostics.mean_correspondence_distance_valid)
    {
      result.failure = FailureCode::kAlignmentFailed;
      result.diagnostics.detail = "pclomp returned a non-finite or incomplete alignment";
      return result;
    }

    result.aligned_source.reset(new PointCloud(aligned));
    // kNone means the call completed with a finite, contract-valid result;
    // convergence remains independently observable in result.converged.
    result.failure = FailureCode::kNone;
    return result;
  } catch (const std::exception & exception) {
    result.failure = FailureCode::kInternalError;
    result.diagnostics.detail = std::string("NDT exception: ") + exception.what();
    return result;
  } catch (...) {
    result.failure = FailureCode::kInternalError;
    result.diagnostics.detail = "NDT exception: unknown exception";
    return result;
  }
}

void NdtOmpRegistration::reset() noexcept
{
  if (!impl_) {
    return;
  }
  clearPerCallState(impl_.get());
  impl_->target.reset();
  impl_->target_cell_cache.reset(0U);
  impl_->ndt.reset();
  impl_->configuration = NdtConfiguration();
  impl_->configured = false;
}

TargetCellCacheStats NdtOmpRegistration::targetCellCacheStats() const noexcept
{
  if (!impl_) {
    return TargetCellCacheStats{};
  }
  return impl_->target_cell_cache.stats();
}

}  // namespace lidarslam_default_plugins

#endif  // LIDARSLAM_DEFAULT_PLUGINS__NDT_OMP_REGISTRATION_IMPL_IPP_
