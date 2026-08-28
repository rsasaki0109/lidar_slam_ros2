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
//    copyright notice, this list of conditions and the following disclaimer
//    in the documentation and/or other materials provided with the
//    distribution.
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

#ifndef SCANMATCHER__REGISTRATION_CONFIG_HPP_
#define SCANMATCHER__REGISTRATION_CONFIG_HPP_

#include <cstdint>
#include <string>

#include <lidarslam_plugin_interfaces/registration.hpp>

namespace graphslam
{
namespace registration_config
{

inline bool isFastGicpMethod(const std::string & method)
{
  return method == "FAST_GICP" || method == "FAST_VGICP";
}

inline bool fastGicpAvailable()
{
#ifdef HAS_FAST_GICP
  return true;
#else
  return false;
#endif
}

inline const char * fastGicpUnavailableReason()
{
  return "optional dependency fast_gicp was not found at configure time; "
         "FAST_GICP/FAST_VGICP cannot be selected and no fallback is allowed";
}

inline const char * fastGicpHostClassId(const std::string & method)
{
  if (method == "FAST_GICP") {
    return "lidarslam_builtin/FastGicp";
  }
  if (method == "FAST_VGICP") {
    return "lidarslam_builtin/FastVGicp";
  }
  return "";
}

inline const char * fastGicpPluginClassId(const std::string & method)
{
  if (method == "FAST_GICP") {
    return "lidarslam_default_plugins/FastGicp";
  }
  if (method == "FAST_VGICP") {
    return "lidarslam_default_plugins/FastVGicp";
  }
  return "";
}

inline bool isCanonicalFastGicpClassId(
  const std::string & method, const std::string & class_id)
{
  return class_id == fastGicpHostClassId(method) ||
         class_id == fastGicpPluginClassId(method);
}

inline bool isSmallGicpMethod(const std::string & method)
{
  return method == "SMALL_GICP" || method == "SMALL_VGICP";
}

inline bool smallGicpAvailable()
{
#ifdef HAS_SMALL_GICP
  return true;
#else
  return false;
#endif
}

inline const char * smallGicpUnavailableReason()
{
  return "optional dependency small_gicp was not found at configure time; "
         "SMALL_GICP/SMALL_VGICP cannot be selected and no fallback is allowed";
}

// Convert the legacy ROS parameter names/values into the canonical typed map
// consumed by the built-in NDT adapter.  Keep this ROS-free so offline shells
// and unit tests use exactly the same mapping as the live component.
inline lidarslam::plugins::registration::ParameterMap makeNdtParameterMap(
  const double resolution,
  const double transformation_epsilon,
  const int maximum_iterations,
  const double step_size,
  const double outlier_ratio,
  const int num_threads)
{
  using lidarslam::plugins::registration::ParameterMap;
  using lidarslam::plugins::registration::ParameterValue;

  ParameterMap parameters;
  parameters.emplace("resolution", ParameterValue(resolution));
  parameters.emplace("transformation_epsilon", ParameterValue(transformation_epsilon));
  parameters.emplace(
    "maximum_iterations",
    ParameterValue(static_cast<std::int64_t>(maximum_iterations)));
  parameters.emplace("step_size", ParameterValue(step_size));
  parameters.emplace("outlier_ratio", ParameterValue(outlier_ratio));
  parameters.emplace(
    "num_threads", ParameterValue(static_cast<std::int64_t>(num_threads)));
  parameters.emplace("neighborhood_search_method", ParameterValue("DIRECT7"));
  return parameters;
}

// Convert the legacy GICP ROS parameters into the canonical typed map.  The
// adaptive flag is explicit because the historical implementation resets its
// correspondence distance after every adaptive call, including the first
// call before an EMA-derived override exists.
inline lidarslam::plugins::registration::ParameterMap makeGicpParameterMap(
  const double maximum_correspondence_distance,
  const bool adaptive_correspondence_threshold = false)
{
  using lidarslam::plugins::registration::ParameterMap;
  using lidarslam::plugins::registration::ParameterValue;

  ParameterMap parameters;
  parameters.emplace(
    "maximum_correspondence_distance",
    ParameterValue(maximum_correspondence_distance));
  parameters.emplace("transformation_epsilon", ParameterValue(1e-8));
  parameters.emplace(
    "adaptive_correspondence_threshold",
    ParameterValue(adaptive_correspondence_threshold));
  return parameters;
}

// Convert the legacy FAST_GICP/FAST_VGICP values into the typed optional
// adapter map. FAST retains the historical fixed 1e-6 epsilon; the
// voxelized variant additionally requires an explicit resolution.
inline lidarslam::plugins::registration::ParameterMap makeFastGicpParameterMap(
  const double maximum_correspondence_distance,
  const int maximum_iterations,
  const int num_threads,
  const bool adaptive_correspondence_threshold,
  const bool voxelized,
  const double voxel_resolution)
{
  using lidarslam::plugins::registration::ParameterMap;
  using lidarslam::plugins::registration::ParameterValue;

  ParameterMap parameters;
  parameters.emplace(
    "maximum_correspondence_distance",
    ParameterValue(maximum_correspondence_distance));
  parameters.emplace("transformation_epsilon", ParameterValue(1e-6));
  parameters.emplace(
    "maximum_iterations",
    ParameterValue(static_cast<std::int64_t>(maximum_iterations)));
  parameters.emplace("num_threads", ParameterValue(static_cast<std::int64_t>(num_threads)));
  parameters.emplace(
    "adaptive_correspondence_threshold",
    ParameterValue(adaptive_correspondence_threshold));
  if (voxelized) {
    parameters.emplace("voxel_resolution", ParameterValue(voxel_resolution));
  }
  return parameters;
}

// Convert the legacy SMALL_GICP/SMALL_VGICP values into the typed adapter
// map.  The variant is selected by the explicit class/registration method;
// the adapter rejects voxel_resolution for SMALL_GICP and requires it for
// SMALL_VGICP, so a class mismatch cannot silently fall back to another
// implementation.
inline lidarslam::plugins::registration::ParameterMap makeSmallGicpParameterMap(
  const double maximum_correspondence_distance,
  const double transformation_epsilon,
  const int maximum_iterations,
  const int num_threads,
  const bool adaptive_correspondence_threshold,
  const bool voxelized,
  const double voxel_resolution)
{
  using lidarslam::plugins::registration::ParameterMap;
  using lidarslam::plugins::registration::ParameterValue;

  ParameterMap parameters;
  parameters.emplace(
    "maximum_correspondence_distance",
    ParameterValue(maximum_correspondence_distance));
  parameters.emplace("transformation_epsilon", ParameterValue(transformation_epsilon));
  parameters.emplace(
    "maximum_iterations",
    ParameterValue(static_cast<std::int64_t>(maximum_iterations)));
  parameters.emplace("num_threads", ParameterValue(static_cast<std::int64_t>(num_threads)));
  parameters.emplace(
    "adaptive_correspondence_threshold",
    ParameterValue(adaptive_correspondence_threshold));
  if (voxelized) {
    parameters.emplace("voxel_resolution", ParameterValue(voxel_resolution));
  }
  return parameters;
}

}  // namespace registration_config
}  // namespace graphslam

#endif  // SCANMATCHER__REGISTRATION_CONFIG_HPP_
