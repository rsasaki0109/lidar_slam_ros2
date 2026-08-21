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

#ifndef GRAPH_BASED_SLAM__REGISTRATION_PLUGIN_ADAPTER_HPP_
#define GRAPH_BASED_SLAM__REGISTRATION_PLUGIN_ADAPTER_HPP_

#include <cmath>
#include <memory>
#include <string>

#include <boost/shared_ptr.hpp>
#include <pcl/registration/registration.h>  // NOLINT(build/include_order)

#include <lidarslam_plugin_interfaces/registration.hpp>

namespace graphslam
{
namespace backend_registration
{

/**
 * Transitional shell bridge for the legacy backend GICP path.
 *
 * BackendCore consumes only RegistrationPlugin.  The bridge is intentionally
 * kept in the ROS shell/compatibility layer so graph_slam_offline_runner can
 * retain its pre-step-7 PCL construction until that runner is migrated.  It
 * is not a public plugin implementation and does not perform discovery.
 */
class PclRegistrationAdapter final
  : public lidarslam::plugins::registration::RegistrationPlugin
{
public:
  using PclRegistration = pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>;
  using PointCloud = lidarslam::plugins::registration::PointCloud;
  using PointCloudConstPtr = lidarslam::plugins::registration::PointCloudConstPtr;

  explicit PclRegistrationAdapter(
    const boost::shared_ptr<PclRegistration> & registration,
    const std::string & class_id = "lidarslam_builtin/LegacyPclRegistration")
  : owned_registration_(registration),
    registration_(registration.get()),
    class_id_(class_id)
  {
  }

  explicit PclRegistrationAdapter(
    PclRegistration & registration,
    const std::string & class_id = "lidarslam_builtin/LegacyPclRegistration")
  : registration_(&registration),
    class_id_(class_id)
  {
  }

  ~PclRegistrationAdapter() override = default;

  lidarslam::plugins::registration::PluginMetadata metadata() const override
  {
    lidarslam::plugins::registration::PluginMetadata metadata;
    metadata.class_id = class_id_;
    metadata.implementation_version = "legacy-pcl-bridge";
    metadata.license = "BSD-2-Clause";
    metadata.api_version = lidarslam::plugins::registration::kHostApiVersion;
    return metadata;
  }

  lidarslam::plugins::registration::Capabilities capabilities() const override
  {
    using lidarslam::plugins::registration::Capability;
    using lidarslam::plugins::registration::Capabilities;
    using lidarslam::plugins::registration::CorrespondenceMetric;
    using lidarslam::plugins::registration::TargetPolicy;
    using lidarslam::plugins::registration::ThreadModel;
    Capabilities capabilities;
    capabilities
    .add(Capability::kInitialGuess)
    .add(Capability::kAlignedSource)
    .setTargetPolicy(TargetPolicy::kAcceptHostPrepared)
    .setCorrespondenceMetric(CorrespondenceMetric::kSquareRootFitnessProxy)
    .setThreadModel(ThreadModel::kSerializedOwner);
    return capabilities;
  }

  bool configure(
    const lidarslam::plugins::registration::ParameterMap &, std::string * error) override
  {
    setError(error, "legacy PCL registration bridge is preconfigured by the shell");
    return false;
  }

  bool setInputTarget(const PointCloudConstPtr & target, std::string * error) override
  {
    if (registration_ == nullptr) {
      setError(error, "legacy PCL registration object is null");
      return false;
    }
    if (!finitePointCloud(target)) {
      setError(error, "legacy PCL target must be non-empty and finite");
      return false;
    }
    try {
      registration_->setInputTarget(target);
      target_ = target;
      return true;
    } catch (const std::exception & exception) {
      setError(error, std::string("legacy PCL target setup failed: ") + exception.what());
      return false;
    } catch (...) {
      setError(error, "legacy PCL target setup failed: unknown exception");
      return false;
    }
  }

  lidarslam::plugins::registration::AlignmentResult align(
    const lidarslam::plugins::registration::AlignmentRequest & request) override
  {
    using lidarslam::plugins::registration::AlignmentResult;
    using lidarslam::plugins::registration::FailureCode;
    AlignmentResult result;
    if (registration_ == nullptr || !target_) {
      result.failure = FailureCode::kNotConfigured;
      result.diagnostics.detail = "legacy PCL registration target is not configured";
      return result;
    }
    const FailureCode request_status =
      lidarslam::plugins::registration::validateRequest(request, capabilities());
    if (request_status != FailureCode::kNone) {
      result.failure = request_status;
      result.diagnostics.detail = "legacy PCL request validation failed";
      return result;
    }
    if (!finitePointCloud(request.source) ||
      (request.initial_guess_enabled && !request.initial_guess.allFinite()))
    {
      result.failure = FailureCode::kInvalidInput;
      result.diagnostics.detail = "legacy PCL source or initial guess is not finite";
      return result;
    }
    try {
      registration_->setInputSource(request.source);
      PointCloud aligned;
      if (request.initial_guess_enabled) {
        registration_->align(aligned, request.initial_guess);
      } else {
        registration_->align(aligned);
      }
      result.converged = registration_->hasConverged();
      result.final_transformation = registration_->getFinalTransformation();
      result.fitness_score = registration_->getFitnessScore();
      if (std::isfinite(result.fitness_score) && result.fitness_score >= 0.0) {
        result.diagnostics.mean_correspondence_distance_valid = true;
        result.diagnostics.mean_correspondence_distance =
          result.fitness_score > 0.0 ? std::sqrt(result.fitness_score) : 0.0;
      }
      if (
        aligned.empty() || aligned.size() != request.source->size() ||
        !result.final_transformation.allFinite() ||
        !std::isfinite(result.fitness_score) ||
        !result.diagnostics.mean_correspondence_distance_valid)
      {
        result.failure = FailureCode::kAlignmentFailed;
        result.diagnostics.detail = "legacy PCL returned an incomplete alignment";
        return result;
      }
      result.aligned_source.reset(new PointCloud(aligned));
      result.failure = FailureCode::kNone;
      return result;
    } catch (const std::exception & exception) {
      result.failure = FailureCode::kInternalError;
      result.diagnostics.detail = std::string("legacy PCL alignment failed: ") + exception.what();
      return result;
    } catch (...) {
      result.failure = FailureCode::kInternalError;
      result.diagnostics.detail = "legacy PCL alignment failed: unknown exception";
      return result;
    }
  }

  void reset() noexcept override
  {
    target_.reset();
  }

private:
  static void setError(std::string * error, const std::string & message)
  {
    if (error != nullptr) {
      *error = message;
    }
  }

  static bool finitePointCloud(const PointCloudConstPtr & cloud)
  {
    if (!cloud || cloud->empty()) {
      return false;
    }
    for (const auto & point : cloud->points) {
      if (!std::isfinite(static_cast<double>(point.x)) ||
        !std::isfinite(static_cast<double>(point.y)) ||
        !std::isfinite(static_cast<double>(point.z)) ||
        !std::isfinite(static_cast<double>(point.intensity)))
      {
        return false;
      }
    }
    return true;
  }

  boost::shared_ptr<PclRegistration> owned_registration_;
  PclRegistration * registration_{nullptr};
  PointCloudConstPtr target_;
  std::string class_id_;
};

}  // namespace backend_registration
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__REGISTRATION_PLUGIN_ADAPTER_HPP_
