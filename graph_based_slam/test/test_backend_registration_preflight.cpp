// Copyright 2026 Sasaki
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>

#include <cstring>
#include <sstream>

#include <pclomp/ndt_omp.h>  // NOLINT(build/include_order)
#include <pclomp/ndt_omp_impl.hpp>  // NOLINT(build/include_order)
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>  // NOLINT(build/include_order)

#include "graph_based_slam/backend_registration_preflight.hpp"
#include "graph_based_slam/registration_plugin_adapter.hpp"
#include "lidarslam_default_plugins/ndt_omp_registration_impl.ipp"

namespace graphslam
{
namespace
{

TEST(BackendRegistrationPreflight, EncodesLegacyNdtDefaultsAndContract)
{
  backend_registration::NdtConfig config;
  lidarslam::plugins::registration::shell::LoadRequest request;
  std::string error;
  ASSERT_TRUE(backend_registration::makeNdtLoadRequest(config, &request, &error)) << error;

  EXPECT_EQ(request.class_id, "lidarslam_builtin/NdtOmp");
  ASSERT_EQ(request.parameters.size(), 8u);
  EXPECT_DOUBLE_EQ(request.parameters.at("resolution").asDouble(), 5.0);
  EXPECT_DOUBLE_EQ(request.parameters.at("transformation_epsilon").asDouble(), 0.01);
  EXPECT_EQ(request.parameters.at("maximum_iterations").asInteger(), 100);
  EXPECT_DOUBLE_EQ(request.parameters.at("step_size").asDouble(), 0.1);
  EXPECT_DOUBLE_EQ(request.parameters.at("outlier_ratio").asDouble(), 0.55);
  EXPECT_EQ(request.parameters.at("num_threads").asInteger(), 0);
  EXPECT_EQ(
    request.parameters.at("target_cell_cache_capacity").asInteger(),
    backend_registration::kBackendNdtTargetCellCacheCapacity);
  EXPECT_EQ(request.parameters.at("neighborhood_search_method").asString(), "DIRECT7");
  EXPECT_TRUE(request.capabilities.require_initial_guess);
  EXPECT_TRUE(request.capabilities.require_aligned_source);
  EXPECT_TRUE(request.capabilities.require_mean_correspondence_distance);
  EXPECT_TRUE(request.capabilities.require_target_policy);
  EXPECT_EQ(
    request.capabilities.target_policy,
    lidarslam::plugins::registration::TargetPolicy::kRequiresRawTarget);
  EXPECT_TRUE(request.capabilities.require_correspondence_metric);
  EXPECT_EQ(
    request.capabilities.correspondence_metric,
    lidarslam::plugins::registration::CorrespondenceMetric::kMeanDistance);
}

TEST(BackendRegistrationPreflight, RejectsInvalidConfigurationBeforeResolver)
{
  backend_registration::NdtConfig config;
  config.resolution = 0.0;
  lidarslam::plugins::registration::shell::LoadRequest request;
  std::string error;
  EXPECT_FALSE(backend_registration::makeNdtLoadRequest(config, &request, &error));
  EXPECT_NE(error.find("resolution"), std::string::npos);

  config = backend_registration::NdtConfig();
  config.num_threads = -1;
  error.clear();
  EXPECT_FALSE(backend_registration::makeNdtLoadRequest(config, &request, &error));
  EXPECT_NE(error.find("threads"), std::string::npos);

  config = backend_registration::NdtConfig();
  config.target_cell_cache_capacity = -1;
  error.clear();
  EXPECT_FALSE(backend_registration::makeNdtLoadRequest(config, &request, &error));
  EXPECT_NE(error.find("cache capacity"), std::string::npos);
}

TEST(BackendRegistrationPreflight, ResolvesHostNdtSessionWithProvenance)
{
  lidarslam::plugins::registration::shell::LoadRequest request;
  std::string error;
  ASSERT_TRUE(backend_registration::makeNdtLoadRequest(
    backend_registration::NdtConfig(), &request, &error)) << error;

  lidarslam::plugins::registration::shell::HostBuiltinRegistration host;
  host.class_id = request.class_id;
  host.metadata_class_id = "lidarslam_default_plugins/NdtOmp";
  host.factory = []() {
      return std::make_shared<lidarslam_default_plugins::NdtOmpRegistration>();
    };
  lidarslam::plugins::registration::shell::RegistrationResolver resolver({host});
  const auto loaded = resolver.resolve(request);
  ASSERT_TRUE(loaded.ok()) << loaded.failure.message;
  ASSERT_TRUE(loaded.session);
  EXPECT_EQ(
    loaded.session->backendKind(),
    lidarslam::plugins::registration::shell::BackendKind::kHostBuiltIn);
  EXPECT_EQ(loaded.session->classId(), "lidarslam_builtin/NdtOmp");
  EXPECT_EQ(loaded.session->metadata().class_id, "lidarslam_builtin/NdtOmp");
  EXPECT_EQ(loaded.session->metadata().license, "BSD-2-Clause");
  EXPECT_EQ(loaded.session->metadata().api_version.major, 1);
  EXPECT_TRUE(loaded.session->plugin());
  EXPECT_TRUE(loaded.session->libraryPath().empty());
  EXPECT_TRUE(loaded.session->pluginManifestPath().empty());
}

TEST(BackendRegistrationPreflight, CanonicalRoleIdentityAndReceiptAreStable)
{
  backend_registration::NdtConfig config;
  config.resolution = 2.0;
  config.maximum_iterations = 35;
  config.num_threads = 1;

  backend_registration::BackendRegistrationRequest live_request;
  backend_registration::BackendRegistrationRequest offline_request;
  std::string error;
  ASSERT_TRUE(backend_registration::makeNdtLoadRequest(config, &live_request, &error)) << error;
  ASSERT_TRUE(backend_registration::makeNdtLoadRequest(config, &offline_request, &error)) << error;
  EXPECT_EQ(live_request.role, backend_registration::kBackendRegistrationRole);
  EXPECT_EQ(offline_request.role, live_request.role);
  EXPECT_EQ(offline_request.request.class_id, live_request.request.class_id);
  EXPECT_EQ(offline_request.request.parameters.size(), live_request.request.parameters.size());
  EXPECT_EQ(
    offline_request.request.capabilities.target_policy,
    live_request.request.capabilities.target_policy);
  EXPECT_EQ(
    offline_request.request.capabilities.correspondence_metric,
    live_request.request.capabilities.correspondence_metric);

  auto make_resolved_session =
    [](const backend_registration::BackendRegistrationRequest & request) {
      auto host = backend_registration::makeNdtHostBuiltinRegistration([]() {
            return std::make_shared<lidarslam_default_plugins::NdtOmpRegistration>();
          });
      lidarslam::plugins::registration::shell::RegistrationResolver resolver({host});
      return resolver.resolve(request.request);
    };
  const auto live_loaded = make_resolved_session(live_request);
  const auto offline_loaded = make_resolved_session(offline_request);
  ASSERT_TRUE(live_loaded.ok()) << live_loaded.failure.message;
  ASSERT_TRUE(offline_loaded.ok()) << offline_loaded.failure.message;
  ASSERT_TRUE(live_loaded.session);
  ASSERT_TRUE(offline_loaded.session);

  EXPECT_EQ(
    backend_registration::canonicalBackendRegistrationIdentity(
      live_request, *live_loaded.session),
    backend_registration::canonicalBackendRegistrationIdentity(
      offline_request, *offline_loaded.session));

  std::ostringstream receipt;
  ASSERT_TRUE(backend_registration::writeBackendRegistrationReceipt(
      receipt, offline_request, *offline_loaded.session, &error)) << error;
  const std::string text = receipt.str();
  EXPECT_NE(text.find("role: \"backend_loop\""), std::string::npos);
  EXPECT_NE(text.find("backend_kind: \"host_builtin\""), std::string::npos);
  EXPECT_NE(text.find("resolved_class: \"lidarslam_builtin/NdtOmp\""), std::string::npos);
  EXPECT_NE(text.find("type: \"double\""), std::string::npos);
  EXPECT_NE(text.find("type: \"integer\""), std::string::npos);
  EXPECT_NE(text.find("\"target_cell_cache_capacity\""), std::string::npos);
  EXPECT_NE(text.find("value: \"3\""), std::string::npos);
  const std::size_t maximum_position = text.find("\"maximum_iterations\"");
  const std::size_t resolution_position = text.find("\"resolution\"");
  EXPECT_NE(maximum_position, std::string::npos);
  EXPECT_NE(resolution_position, std::string::npos);
  EXPECT_LT(maximum_position, resolution_position);
}

TEST(BackendRegistrationAdapter, ResetAndInvalidTargetAreFailClosed)
{
  pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
  backend_registration::PclRegistrationAdapter adapter(ndt, "lidarslam_builtin/TestNdt");
  lidarslam::plugins::registration::AlignmentRequest request;
  pcl::PointCloud<pcl::PointXYZI>::Ptr source(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointXYZI source_point;
  source_point.x = 0.0F;
  source_point.y = 0.0F;
  source_point.z = 0.0F;
  source_point.intensity = 0.0F;
  source->push_back(source_point);
  request.source = source;

  auto result = adapter.align(request);
  EXPECT_EQ(
    result.failure,
    lidarslam::plugins::registration::FailureCode::kNotConfigured);

  std::string error;
  lidarslam::plugins::registration::PointCloudConstPtr empty_target(
    new pcl::PointCloud<pcl::PointXYZI>);
  EXPECT_FALSE(adapter.setInputTarget(empty_target, &error));
  EXPECT_NE(error.find("non-empty"), std::string::npos);

  pcl::PointCloud<pcl::PointXYZI>::Ptr target(new pcl::PointCloud<pcl::PointXYZI>);
  target->push_back(source_point);
  ASSERT_TRUE(adapter.setInputTarget(target, &error)) << error;
  adapter.reset();
  result = adapter.align(request);
  EXPECT_EQ(
    result.failure,
    lidarslam::plugins::registration::FailureCode::kNotConfigured);
}

TEST(BackendRegistrationAdapter, NdtHostAdapterMatchesDirectPclFixture)
{
  using PointT = pcl::PointXYZI;
  using Cloud = pcl::PointCloud<PointT>;
  Cloud::Ptr source(new Cloud);
  Cloud::Ptr target(new Cloud);
  for (int x = -8; x <= 8; ++x) {
    for (int y = -8; y <= 8; ++y) {
      PointT source_point;
      source_point.x = static_cast<float>(x) * 0.4F;
      source_point.y = static_cast<float>(y) * 0.4F;
      source_point.z = static_cast<float>((x * 3 + y * 5) % 7) * 0.1F;
      source_point.intensity = static_cast<float>((x + y) & 3);
      source->push_back(source_point);
      PointT target_point = source_point;
      target_point.x += 0.2F;
      target_point.y -= 0.1F;
      target->push_back(target_point);
    }
  }

  pclomp::NormalDistributionsTransform<PointT, PointT> direct;
  direct.setResolution(2.0F);
  direct.setTransformationEpsilon(0.01);
  direct.setMaximumIterations(35);
  direct.setStepSize(0.1);
  direct.setOulierRatio(0.55);
  direct.setNeighborhoodSearchMethod(pclomp::DIRECT7);
  direct.setNumThreads(1);
  direct.setInputSource(source);
  direct.setInputTarget(target);
  Cloud direct_aligned;
  direct.align(direct_aligned, Eigen::Matrix4f::Identity());

  lidarslam_default_plugins::NdtOmpRegistration plugin;
  std::string error;
  backend_registration::NdtConfig config;
  config.resolution = 2.0;
  config.maximum_iterations = 35;
  config.num_threads = 1;
  lidarslam::plugins::registration::shell::LoadRequest request;
  ASSERT_TRUE(backend_registration::makeNdtLoadRequest(config, &request, &error)) << error;
  ASSERT_TRUE(plugin.configure(request.parameters, &error)) << error;
  ASSERT_TRUE(plugin.setInputTarget(target, &error)) << error;

  lidarslam::plugins::registration::AlignmentRequest alignment_request;
  alignment_request.source = source;
  alignment_request.initial_guess_enabled = true;
  alignment_request.initial_guess = Eigen::Matrix4f::Identity();
  const auto typed = plugin.align(alignment_request);
  ASSERT_TRUE(typed.aligned_source);
  EXPECT_EQ(typed.converged, direct.hasConverged());
  EXPECT_DOUBLE_EQ(typed.fitness_score, direct.getFitnessScore());
  EXPECT_EQ(
    std::memcmp(typed.final_transformation.data(), direct.getFinalTransformation().data(),
      sizeof(float) * 16U), 0);
  ASSERT_EQ(typed.aligned_source->points.size(), direct_aligned.points.size());
  EXPECT_EQ(
    std::memcmp(
      typed.aligned_source->points.data(), direct_aligned.points.data(),
      typed.aligned_source->points.size() * sizeof(PointT)), 0);
}

}  // namespace
}  // namespace graphslam
