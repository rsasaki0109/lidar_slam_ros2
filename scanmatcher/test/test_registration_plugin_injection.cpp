// Copyright 2026 Sasaki
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
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

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "lidarslam_registration_loader/registration_plugin_loader.hpp"
#include "scanmatcher/registration_config.hpp"
#include "scanmatcher/scanmatcher_component.h"

namespace
{

using lidarslam::plugins::registration::TargetPolicy;
using lidarslam::plugins::registration::shell::LoadRequest;
using lidarslam::plugins::registration::shell::RegistrationPluginLoader;

LoadRequest makeRequest()
{
  LoadRequest request;
  request.class_id = "lidarslam_default_plugins/NdtOmp";
  request.parameters = graphslam::registration_config::makeNdtParameterMap(
    5.0, 0.01, 35, 0.1, 0.55, 1);
  request.capabilities.require_initial_guess = true;
  request.capabilities.require_aligned_source = true;
  request.capabilities.require_target_policy = true;
  request.capabilities.target_policy = TargetPolicy::kRequiresRawTarget;
  return request;
}

LoadRequest makeGicpRequest()
{
  LoadRequest request;
  request.class_id = "lidarslam_builtin/GicpOmp";
  request.parameters = graphslam::registration_config::makeGicpParameterMap(5.0, false);
  request.capabilities.require_initial_guess = true;
  request.capabilities.require_aligned_source = true;
  request.capabilities.require_target_policy = true;
  request.capabilities.target_policy = TargetPolicy::kAcceptHostPrepared;
  request.capabilities.require_correspondence_metric = true;
  request.capabilities.correspondence_metric =
    lidarslam::plugins::registration::CorrespondenceMetric::kSquareRootFitnessProxy;
  return request;
}

#ifdef HAS_SMALL_GICP
LoadRequest makeSmallRequest(const bool voxelized)
{
  LoadRequest request;
  request.class_id = voxelized ?
    "lidarslam_builtin/SmallVGicpPcl" : "lidarslam_builtin/SmallGicpPcl";
  request.parameters = graphslam::registration_config::makeSmallGicpParameterMap(
    5.0, 1e-6, 35, 1, false, voxelized, 0.6);
  request.capabilities.require_initial_guess = true;
  request.capabilities.require_aligned_source = true;
  request.capabilities.require_target_policy = true;
  request.capabilities.target_policy = TargetPolicy::kAcceptHostPrepared;
  request.capabilities.require_correspondence_metric = true;
  request.capabilities.correspondence_metric =
    lidarslam::plugins::registration::CorrespondenceMetric::kSquareRootFitnessProxy;
  return request;
}

std::unique_ptr<lidarslam::plugins::registration::shell::RegistrationResolver> makeSmallResolver()
{
  lidarslam::plugins::registration::shell::HostBuiltinRegistration small_gicp;
  small_gicp.class_id = "lidarslam_builtin/SmallGicpPcl";
  small_gicp.factory = []() {
      return graphslam::makeHostBuiltinSmallGicpRegistration();
    };
  small_gicp.metadata_class_id = "lidarslam_default_plugins/SmallGicpPcl";
  lidarslam::plugins::registration::shell::HostBuiltinRegistration small_vgicp;
  small_vgicp.class_id = "lidarslam_builtin/SmallVGicpPcl";
  small_vgicp.factory = []() {
      return graphslam::makeHostBuiltinSmallVgicpRegistration();
    };
  small_vgicp.metadata_class_id = "lidarslam_default_plugins/SmallVGicpPcl";
  return std::unique_ptr<lidarslam::plugins::registration::shell::RegistrationResolver>(
    new lidarslam::plugins::registration::shell::RegistrationResolver({small_gicp, small_vgicp}));
}
#endif

}  // namespace

TEST(RegistrationPluginInjection, InjectsSessionBeforeAnySensorCloud)
{
  if (!rclcpp::ok()) {
    char ** argv = nullptr;
    rclcpp::init(0, argv);
  }

  RegistrationPluginLoader loader;
  const auto loaded = loader.load(makeRequest());
  ASSERT_TRUE(loaded.ok()) << loaded.failure.message;

  auto component = std::make_shared<graphslam::ScanMatcherComponent>(
    rclcpp::NodeOptions());
  std::string error;
  EXPECT_TRUE(component->setRegistrationPluginSession(loaded.session, &error)) << error;

  component.reset();
  rclcpp::shutdown();
}

TEST(RegistrationPluginInjection, ReplacingSessionPreservesLoaderLifetimeOrder)
{
  if (!rclcpp::ok()) {
    char ** argv = nullptr;
    rclcpp::init(0, argv);
  }

  // Keep the first plugin in an inner scope so the component is the only
  // owner of its session when the second injection replaces the slot.  This
  // exercises the required plugin-before-session destruction order for a
  // pluginlib-backed object.
  std::shared_ptr<lidarslam::plugins::registration::shell::RegistrationPluginSession>
    external_session;
  {
    RegistrationPluginLoader loader;
    const auto loaded = loader.load(makeRequest());
    ASSERT_TRUE(loaded.ok()) << loaded.failure.message;
    external_session = loaded.session;
  }

  lidarslam::plugins::registration::shell::HostBuiltinRegistration host_ndt;
  host_ndt.class_id = "lidarslam_builtin/NdtOmp";
  host_ndt.factory = []() {
      return graphslam::makeHostBuiltinNdtRegistration();
    };
  host_ndt.metadata_class_id = "lidarslam_default_plugins/NdtOmp";
  lidarslam::plugins::registration::shell::RegistrationResolver resolver({host_ndt});
  LoadRequest host_request = makeRequest();
  host_request.class_id = host_ndt.class_id;
  const auto host_loaded = resolver.resolve(host_request);
  ASSERT_TRUE(host_loaded.ok()) << host_loaded.failure.message;

  auto component = std::make_shared<graphslam::ScanMatcherComponent>(
    rclcpp::NodeOptions(), graphslam::RegistrationConstruction::kDeferredPluginInjection);
  std::string error;
  ASSERT_TRUE(component->setRegistrationPluginSession(external_session, &error)) << error;
  // Leave the component as the sole owner.  The next accepted injection must
  // therefore destroy the external plugin before releasing its loader-backed
  // session; keeping this reference until afterwards would not exercise the
  // unload boundary at all.
  external_session.reset();
  EXPECT_TRUE(component->setRegistrationPluginSession(host_loaded.session, &error)) << error;
  component.reset();
  rclcpp::shutdown();
}

TEST(RegistrationPluginInjection, ResolvesHostBuiltinFromSameTranslationUnit)
{
  if (!rclcpp::ok()) {
    char ** argv = nullptr;
    rclcpp::init(0, argv);
  }

  lidarslam::plugins::registration::shell::HostBuiltinRegistration host_ndt;
  host_ndt.class_id = "lidarslam_builtin/NdtOmp";
  host_ndt.factory = []() {
      return graphslam::makeHostBuiltinNdtRegistration();
    };
  host_ndt.metadata_class_id = "lidarslam_default_plugins/NdtOmp";
  lidarslam::plugins::registration::shell::RegistrationResolver resolver({host_ndt});

  LoadRequest request = makeRequest();
  request.class_id = "lidarslam_builtin/NdtOmp";
  const auto loaded = resolver.resolve(request);
  ASSERT_TRUE(loaded.ok()) << loaded.failure.message;
  ASSERT_NE(loaded.session, nullptr);
  EXPECT_EQ(
    loaded.session->backendKind(),
    lidarslam::plugins::registration::shell::BackendKind::kHostBuiltIn);
  EXPECT_TRUE(loaded.session->libraryPath().empty());
  EXPECT_TRUE(loaded.session->pluginManifestPath().empty());

  auto component = std::make_shared<graphslam::ScanMatcherComponent>(
    rclcpp::NodeOptions(), graphslam::RegistrationConstruction::kDeferredPluginInjection);
  std::string error;
  EXPECT_TRUE(component->setRegistrationPluginSession(loaded.session, &error)) << error;

  component.reset();
  rclcpp::shutdown();
}

TEST(RegistrationPluginInjection, ResolvesHostBuiltinGicpBeforeSensorCloud)
{
  if (!rclcpp::ok()) {
    char ** argv = nullptr;
    rclcpp::init(0, argv);
  }

  lidarslam::plugins::registration::shell::HostBuiltinRegistration host_gicp;
  host_gicp.class_id = "lidarslam_builtin/GicpOmp";
  host_gicp.factory = []() {
      return graphslam::makeHostBuiltinGicpRegistration();
    };
  host_gicp.metadata_class_id = "lidarslam_default_plugins/GicpOmp";
  lidarslam::plugins::registration::shell::RegistrationResolver resolver({host_gicp});
  const LoadRequest request = makeGicpRequest();
  const auto loaded = resolver.resolve(request);
  ASSERT_TRUE(loaded.ok()) << loaded.failure.message;
  EXPECT_EQ(loaded.session->backendKind(),
    lidarslam::plugins::registration::shell::BackendKind::kHostBuiltIn);

  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("registration_method", "GICP")});
  auto component = std::make_shared<graphslam::ScanMatcherComponent>(
    options, graphslam::RegistrationConstruction::kDeferredPluginInjection);
  std::string error;
  EXPECT_TRUE(component->setRegistrationPluginSession(loaded.session, &error)) << error;
  component.reset();
  rclcpp::shutdown();
}

TEST(RegistrationPluginInjection, RejectsGicpPluginForNdtRegistrationMethod)
{
  if (!rclcpp::ok()) {
    char ** argv = nullptr;
    rclcpp::init(0, argv);
  }

  lidarslam::plugins::registration::shell::HostBuiltinRegistration host_gicp;
  host_gicp.class_id = "lidarslam_builtin/GicpOmp";
  host_gicp.factory = []() {
      return graphslam::makeHostBuiltinGicpRegistration();
    };
  host_gicp.metadata_class_id = "lidarslam_default_plugins/GicpOmp";
  lidarslam::plugins::registration::shell::RegistrationResolver resolver({host_gicp});
  const auto loaded = resolver.resolve(makeGicpRequest());
  ASSERT_TRUE(loaded.ok()) << loaded.failure.message;

  auto component = std::make_shared<graphslam::ScanMatcherComponent>(
    rclcpp::NodeOptions(), graphslam::RegistrationConstruction::kDeferredPluginInjection);
  std::string error;
  EXPECT_FALSE(component->setRegistrationPluginSession(loaded.session, &error));
  EXPECT_NE(error.find("registration_method=NDT"), std::string::npos);
  component.reset();
  rclcpp::shutdown();
}

TEST(RegistrationPluginInjection, RejectsNdtPluginForGicpRegistrationMethod)
{
  if (!rclcpp::ok()) {
    char ** argv = nullptr;
    rclcpp::init(0, argv);
  }

  RegistrationPluginLoader loader;
  const auto loaded = loader.load(makeRequest());
  ASSERT_TRUE(loaded.ok()) << loaded.failure.message;

  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("registration_method", "GICP")});
  auto component = std::make_shared<graphslam::ScanMatcherComponent>(options);
  std::string error;
  EXPECT_FALSE(component->setRegistrationPluginSession(loaded.session, &error));
  EXPECT_NE(error.find("registration_method=GICP"), std::string::npos);

  component.reset();
  rclcpp::shutdown();
}

#ifdef HAS_SMALL_GICP
TEST(RegistrationPluginInjection, RejectsSmallVgicpPluginForSmallGicpMethod)
{
  if (!rclcpp::ok()) {
    char ** argv = nullptr;
    rclcpp::init(0, argv);
  }

  auto resolver = makeSmallResolver();
  LoadRequest request = makeSmallRequest(true);
  const auto loaded = resolver->resolve(request);
  ASSERT_TRUE(loaded.ok()) << loaded.failure.message;

  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("registration_method", "SMALL_GICP")});
  auto component = std::make_shared<graphslam::ScanMatcherComponent>(
    options, graphslam::RegistrationConstruction::kDeferredPluginInjection);
  std::string error;
  EXPECT_FALSE(component->setRegistrationPluginSession(loaded.session, &error));
  EXPECT_NE(error.find("selector mismatch"), std::string::npos);
  component.reset();
  rclcpp::shutdown();
}

TEST(RegistrationPluginInjection, RejectsSmallGicpPluginForSmallVgicpMethod)
{
  if (!rclcpp::ok()) {
    char ** argv = nullptr;
    rclcpp::init(0, argv);
  }

  auto resolver = makeSmallResolver();
  LoadRequest request = makeSmallRequest(false);
  const auto loaded = resolver->resolve(request);
  ASSERT_TRUE(loaded.ok()) << loaded.failure.message;

  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("registration_method", "SMALL_VGICP")});
  auto component = std::make_shared<graphslam::ScanMatcherComponent>(
    options, graphslam::RegistrationConstruction::kDeferredPluginInjection);
  std::string error;
  EXPECT_FALSE(component->setRegistrationPluginSession(loaded.session, &error));
  EXPECT_NE(error.find("selector mismatch"), std::string::npos);
  component.reset();
  rclcpp::shutdown();
}
#endif
