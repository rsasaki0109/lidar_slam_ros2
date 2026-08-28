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

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Core>  // NOLINT(build/include_order)
#include <Eigen/Geometry>  // NOLINT(build/include_order)
#include <pcl/point_cloud.h>  // NOLINT(build/include_order)
#include <pcl/point_types.h>  // NOLINT(build/include_order)
#include <pclomp/ndt_omp.h>  // NOLINT(build/include_order)
// ndt_omp_ros2 installs the template implementation in its include tree.
#include <pclomp/ndt_omp_impl.hpp>  // NOLINT(build/include_order)
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>  // NOLINT(build/include_order)

#include "lidarslam_default_plugins/ndt_omp_registration.hpp"

namespace
{

namespace registration = lidarslam::plugins::registration;
using PointT = registration::PointT;
using Cloud = registration::PointCloud;

constexpr float kPi = 3.14159265358979323846F;

Cloud::Ptr makeStructuredCloud()
{
  Cloud::Ptr cloud(new Cloud());
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> jitter(-0.02F, 0.02F);

  auto add = [&cloud](float x, float y, float z) {
      PointT point;
      point.x = x;
      point.y = y;
      point.z = z;
      point.intensity = 0.0F;
      cloud->push_back(point);
    };

  // This is the frozen R1 fixture from graph_based_slam's direct pclomp test:
  // a 20 m ground plane, two walls, and an asymmetric pillar.
  for (float x = -10.0F; x <= 10.0F; x += 0.4F) {
    for (float y = -10.0F; y <= 10.0F; y += 0.4F) {
      add(x + jitter(rng), y + jitter(rng), jitter(rng));
    }
  }
  for (float y = -10.0F; y <= 10.0F; y += 0.3F) {
    for (float z = 0.0F; z <= 3.0F; z += 0.3F) {
      add(10.0F + jitter(rng), y + jitter(rng), z + jitter(rng));
    }
  }
  for (float x = -10.0F; x <= 10.0F; x += 0.3F) {
    for (float z = 0.0F; z <= 3.0F; z += 0.3F) {
      add(x + jitter(rng), -10.0F + jitter(rng), z + jitter(rng));
    }
  }
  for (float z = 0.0F; z <= 3.0F; z += 0.1F) {
    for (float a = 0.0F; a < 6.28F; a += 0.5F) {
      add(
        3.0F + 0.3F * std::cos(a) + jitter(rng),
        4.0F + 0.3F * std::sin(a) + jitter(rng),
        z + jitter(rng));
    }
  }
  return cloud;
}

Eigen::Matrix4f knownTransform()
{
  const Eigen::Affine3f transform = Eigen::Translation3f(0.4F, 0.2F, 0.05F) *
    Eigen::AngleAxisf(8.0F * kPi / 180.0F, Eigen::Vector3f::UnitZ());
  return transform.matrix();
}

Cloud::Ptr makeTargetCloud(const Cloud::Ptr & source)
{
  Cloud::Ptr target(new Cloud());
  target->reserve(source->size());
  const Eigen::Matrix4f transform = knownTransform();
  for (const PointT & point : *source) {
    const Eigen::Vector4f transformed =
      transform * Eigen::Vector4f(point.x, point.y, point.z, 1.0F);
    PointT target_point = point;
    target_point.x = transformed.x();
    target_point.y = transformed.y();
    target_point.z = transformed.z();
    target->push_back(target_point);
  }
  return target;
}

Cloud::Ptr makeCacheTarget(float offset)
{
  Cloud::Ptr target(new Cloud());
  for (int x = -4; x <= 4; ++x) {
    for (int y = -4; y <= 4; ++y) {
      PointT point;
      point.x = static_cast<float>(x) * 0.35F + offset;
      point.y = static_cast<float>(y) * 0.35F;
      point.z = static_cast<float>((x * 3 + y * 5) % 7) * 0.08F;
      point.intensity = static_cast<float>((x + y) & 3);
      target->push_back(point);
    }
  }
  return target;
}

registration::ParameterMap makeParameters(int num_threads, int target_cell_cache_capacity = 0)
{
  registration::ParameterMap parameters;
  parameters.emplace("resolution", registration::ParameterValue(2.0));
  parameters.emplace("transformation_epsilon", registration::ParameterValue(0.01));
  parameters.emplace(
    "maximum_iterations",
    registration::ParameterValue(static_cast<std::int64_t>(35)));
  parameters.emplace("step_size", registration::ParameterValue(0.1));
  parameters.emplace("outlier_ratio", registration::ParameterValue(0.55));
  parameters.emplace(
    "num_threads",
    registration::ParameterValue(static_cast<std::int64_t>(num_threads)));
  parameters.emplace(
    "target_cell_cache_capacity",
    registration::ParameterValue(static_cast<std::int64_t>(target_cell_cache_capacity)));
  parameters.emplace(
    "neighborhood_search_method", registration::ParameterValue("DIRECT7"));
  return parameters;
}

struct DirectResult
{
  Eigen::Matrix4f matrix{Eigen::Matrix4f::Identity()};
  double fitness{0.0};
  bool converged{false};
  int iterations{-1};
  double mean_distance{0.0};
  Cloud aligned;
};

DirectResult runDirect(
  const Cloud::Ptr & source, const Cloud::Ptr & target, int num_threads,
  bool initial_guess_enabled = true, const registration::RotationPrior & rotation_prior = {},
  const registration::TranslationPrior & translation_prior = {},
  bool max_distance_enabled = false, double max_distance = 0.0)
{
  pclomp::NormalDistributionsTransform<PointT, PointT> ndt;
  ndt.setNumThreads(num_threads);
  ndt.setResolution(2.0F);
  ndt.setTransformationEpsilon(0.01);
  ndt.setMaximumIterations(35);
  ndt.setStepSize(0.1);
  ndt.setOulierRatio(0.55);
  ndt.setNeighborhoodSearchMethod(pclomp::DIRECT7);
  ndt.setInputSource(source);
  ndt.setInputTarget(target);
  if (rotation_prior.enabled) {
    ndt.setRotationPrior(
      rotation_prior.roll_pitch_yaw,
      rotation_prior.weight,
      rotation_prior.roll_pitch_only);
  }
  if (translation_prior.enabled) {
    ndt.setTranslationPrior(translation_prior.position, translation_prior.weights);
  }
  if (max_distance_enabled) {
    ndt.setMaxCorrespondenceDistance(max_distance);
  }

  DirectResult result;
  if (initial_guess_enabled) {
    ndt.align(result.aligned, Eigen::Matrix4f::Identity());
  } else {
    ndt.align(result.aligned);
  }
  result.matrix = ndt.getFinalTransformation();
  result.fitness = ndt.getFitnessScore();
  result.converged = ndt.hasConverged();
  result.iterations = ndt.getFinalNumIteration();
  result.mean_distance = ndt.getLastMeanCorrespondenceDistance();
  return result;
}

registration::AlignmentResult runPlugin(
  int num_threads, const Cloud::Ptr & source, const Cloud::Ptr & target,
  bool initial_guess_enabled = true, const registration::RotationPrior & rotation_prior = {},
  const registration::TranslationPrior & translation_prior = {},
  bool max_distance_enabled = false, double max_distance = 0.0,
  int target_cell_cache_capacity = 0)
{
  lidarslam_default_plugins::NdtOmpRegistration plugin;
  std::string error;
  EXPECT_TRUE(plugin.configure(makeParameters(num_threads, target_cell_cache_capacity),
      &error)) << error;
  EXPECT_TRUE(plugin.setInputTarget(target, &error)) << error;

  registration::AlignmentRequest request;
  request.source = source;
  request.initial_guess_enabled = initial_guess_enabled;
  request.initial_guess = Eigen::Matrix4f::Identity();
  request.rotation_prior = rotation_prior;
  request.translation_prior = translation_prior;
  request.maximum_correspondence_distance_enabled = max_distance_enabled;
  request.maximum_correspondence_distance = max_distance;
  return plugin.align(request);
}

bool bitwiseEqual(const Eigen::Matrix4f & first, const Eigen::Matrix4f & second)
{
  return std::memcmp(first.data(), second.data(), sizeof(float) * 16U) == 0;
}

bool bitwiseEqual(const Cloud & first, const Cloud & second)
{
  if (
    first.width != second.width || first.height != second.height ||
    first.is_dense != second.is_dense || first.points.size() != second.points.size())
  {
    return false;
  }
  if (first.points.empty()) {
    return true;
  }
  return std::memcmp(
           first.points.data(), second.points.data(),
           first.points.size() * sizeof(PointT)) == 0;
}

float translationDelta(const Eigen::Matrix4f & first, const Eigen::Matrix4f & second)
{
  return (first.block<3, 1>(0, 3) - second.block<3, 1>(0, 3)).norm();
}

float rotationDeltaDeg(const Eigen::Matrix4f & first, const Eigen::Matrix4f & second)
{
  const Eigen::Matrix3f relative =
    first.block<3, 3>(0, 0).transpose() * second.block<3, 3>(0, 0);
  const float cosine = std::min(1.0F, std::max(-1.0F, (relative.trace() - 1.0F) / 2.0F));
  return std::acos(cosine) * 180.0F / kPi;
}

TEST(NdtOmpRegistration, MetadataAndCapabilitiesMatchContract)
{
  lidarslam_default_plugins::NdtOmpRegistration plugin;
  const registration::PluginMetadata metadata = plugin.metadata();
  EXPECT_EQ(metadata.class_id, "lidarslam_default_plugins/NdtOmp");
  EXPECT_EQ(metadata.implementation_version, "1.0.0");
  EXPECT_EQ(metadata.license, "BSD-2-Clause");
  EXPECT_TRUE(registration::isApiCompatible(registration::kHostApiVersion, metadata.api_version));

  const registration::Capabilities capabilities = plugin.capabilities();
  EXPECT_TRUE(capabilities.has(registration::Capability::kInitialGuess));
  EXPECT_TRUE(capabilities.has(registration::Capability::kRotationPrior));
  EXPECT_TRUE(capabilities.has(registration::Capability::kTranslationPrior));
  EXPECT_TRUE(capabilities.has(registration::Capability::kMaximumCorrespondenceDistance));
  EXPECT_TRUE(capabilities.has(registration::Capability::kMeanCorrespondenceDistance));
  EXPECT_FALSE(capabilities.has(registration::Capability::kDeterministic));
  EXPECT_TRUE(capabilities.has(registration::Capability::kAlignedSource));
  EXPECT_EQ(
    capabilities.targetPolicy(), registration::TargetPolicy::kRequiresRawTarget);
  EXPECT_EQ(
    capabilities.correspondenceMetric(), registration::CorrespondenceMetric::kMeanDistance);
  EXPECT_EQ(
    capabilities.threadModel(), registration::ThreadModel::kSerializedOwner);

  std::string error;
  ASSERT_TRUE(plugin.configure(makeParameters(1), &error)) << error;
  EXPECT_TRUE(plugin.capabilities().has(registration::Capability::kDeterministic));
  plugin.reset();
}

TEST(NdtOmpRegistration, RejectsInvalidTypedConfiguration)
{
  struct InvalidCase
  {
    const char * key;
    registration::ParameterValue value;
  };

  const std::vector<InvalidCase> invalid_cases{
    {"resolution", registration::ParameterValue(static_cast<std::int64_t>(2))},
    {"transformation_epsilon", registration::ParameterValue(0.0)},
    {"maximum_iterations", registration::ParameterValue(0.1)},
    {"step_size", registration::ParameterValue(-0.1)},
    {"outlier_ratio", registration::ParameterValue(1.0)},
    {"num_threads", registration::ParameterValue(static_cast<std::int64_t>(-1))},
    {"target_cell_cache_capacity", registration::ParameterValue(static_cast<std::int64_t>(-1))},
    {"neighborhood_search_method", registration::ParameterValue("KDTREE")},
    {"unknown", registration::ParameterValue(true)},
  };

  for (const InvalidCase & invalid : invalid_cases) {
    lidarslam_default_plugins::NdtOmpRegistration plugin;
    registration::ParameterMap parameters = makeParameters(1);
    parameters.erase(invalid.key);
    parameters.emplace(invalid.key, invalid.value);
    std::string error;
    EXPECT_FALSE(plugin.configure(parameters, &error)) << invalid.key;
    EXPECT_FALSE(error.empty()) << invalid.key;
  }

  lidarslam_default_plugins::NdtOmpRegistration unconfigured;
  std::string error;
  EXPECT_FALSE(unconfigured.setInputTarget(Cloud::Ptr(new Cloud()), &error));
  EXPECT_FALSE(error.empty());
  const registration::AlignmentResult result = unconfigured.align(registration::AlignmentRequest());
  EXPECT_EQ(result.failure, registration::FailureCode::kNotConfigured);
}

TEST(NdtOmpRegistration, MatchesDirectPclOmpOneThreadBitForBit)
{
  const Cloud::Ptr source = makeStructuredCloud();
  const Cloud::Ptr target = makeTargetCloud(source);
  const DirectResult direct = runDirect(source, target, 1);
  ASSERT_TRUE(direct.converged);

  const registration::AlignmentResult plugin = runPlugin(1, source, target);
  ASSERT_EQ(plugin.failure, registration::FailureCode::kNone) << plugin.diagnostics.detail;
  ASSERT_TRUE(plugin.aligned_source);
  EXPECT_EQ(plugin.converged, direct.converged);
  EXPECT_TRUE(bitwiseEqual(plugin.final_transformation, direct.matrix));
  EXPECT_DOUBLE_EQ(plugin.fitness_score, direct.fitness);
  EXPECT_EQ(plugin.diagnostics.iterations, direct.iterations);
  EXPECT_DOUBLE_EQ(
    plugin.diagnostics.mean_correspondence_distance_valid ?
    plugin.diagnostics.mean_correspondence_distance : -1.0,
    direct.mean_distance);
  EXPECT_TRUE(bitwiseEqual(*plugin.aligned_source, direct.aligned));
}

TEST(NdtOmpRegistration, MatchesDirectPclOmpTwoThreadsBitForBit)
{
  const Cloud::Ptr source = makeStructuredCloud();
  const Cloud::Ptr target = makeTargetCloud(source);
  const DirectResult direct = runDirect(source, target, 2);
  ASSERT_TRUE(direct.converged);

  const registration::AlignmentResult plugin = runPlugin(2, source, target);
  ASSERT_EQ(plugin.failure, registration::FailureCode::kNone) << plugin.diagnostics.detail;
  ASSERT_TRUE(plugin.aligned_source);
  EXPECT_EQ(plugin.converged, direct.converged);
  EXPECT_TRUE(bitwiseEqual(plugin.final_transformation, direct.matrix));
  EXPECT_DOUBLE_EQ(plugin.fitness_score, direct.fitness);
  EXPECT_EQ(plugin.diagnostics.iterations, direct.iterations);
  EXPECT_DOUBLE_EQ(
    plugin.diagnostics.mean_correspondence_distance_valid ?
    plugin.diagnostics.mean_correspondence_distance : -1.0,
    direct.mean_distance);
  EXPECT_TRUE(bitwiseEqual(*plugin.aligned_source, direct.aligned));
}

TEST(NdtOmpRegistration, TargetCellCacheIsOptInAndPreservesAlignment)
{
  const Cloud::Ptr source = makeStructuredCloud();
  const Cloud::Ptr target = makeTargetCloud(source);
  lidarslam_default_plugins::NdtOmpRegistration plugin;
  std::string error;
  ASSERT_TRUE(plugin.configure(makeParameters(1, 3), &error)) << error;
  ASSERT_EQ(plugin.targetCellCacheStats().capacity, 3U);
  ASSERT_TRUE(plugin.setInputTarget(target, &error)) << error;
  const auto after_miss = plugin.targetCellCacheStats();
  EXPECT_EQ(after_miss.size, 1U);
  EXPECT_EQ(after_miss.misses, 1U);
  EXPECT_EQ(after_miss.hits, 0U);

  registration::AlignmentRequest request;
  request.source = source;
  request.initial_guess_enabled = true;
  request.initial_guess = Eigen::Matrix4f::Identity();
  const auto first = plugin.align(request);
  ASSERT_EQ(first.failure, registration::FailureCode::kNone) << first.diagnostics.detail;
  ASSERT_TRUE(first.aligned_source);

  ASSERT_TRUE(plugin.setInputTarget(target, &error)) << error;
  const auto after_hit = plugin.targetCellCacheStats();
  EXPECT_EQ(after_hit.size, 1U);
  EXPECT_EQ(after_hit.misses, 1U);
  EXPECT_EQ(after_hit.hits, 1U);
  const auto second = plugin.align(request);
  ASSERT_EQ(second.failure, registration::FailureCode::kNone) << second.diagnostics.detail;
  ASSERT_TRUE(second.aligned_source);
  EXPECT_TRUE(bitwiseEqual(first.final_transformation, second.final_transformation));
  EXPECT_DOUBLE_EQ(first.fitness_score, second.fitness_score);
  EXPECT_TRUE(bitwiseEqual(*first.aligned_source, *second.aligned_source));

  // The default frontend/external configuration remains uncached.
  lidarslam_default_plugins::NdtOmpRegistration uncached;
  ASSERT_TRUE(uncached.configure(makeParameters(1), &error)) << error;
  EXPECT_EQ(uncached.targetCellCacheStats().capacity, 0U);
}

TEST(NdtOmpRegistration, TargetCellCacheUsesDeterministicBoundedLruAndReset)
{
  lidarslam_default_plugins::NdtOmpRegistration plugin;
  std::string error;
  ASSERT_TRUE(plugin.configure(makeParameters(1, 3), &error)) << error;
  const std::vector<Cloud::Ptr> targets{
    makeCacheTarget(0.0F), makeCacheTarget(10.0F),
    makeCacheTarget(20.0F), makeCacheTarget(30.0F)};

  for (int index = 0; index < 3; ++index) {
    ASSERT_TRUE(plugin.setInputTarget(targets[static_cast<std::size_t>(index)], &error)) << error;
  }
  auto stats = plugin.targetCellCacheStats();
  EXPECT_EQ(stats.capacity, 3U);
  EXPECT_EQ(stats.size, 3U);
  EXPECT_EQ(stats.misses, 3U);
  EXPECT_EQ(stats.hits, 0U);
  EXPECT_EQ(stats.evictions, 0U);

  // Refresh A, then insert D.  B is the deterministic LRU victim.
  ASSERT_TRUE(plugin.setInputTarget(targets[0], &error)) << error;
  ASSERT_TRUE(plugin.setInputTarget(targets[3], &error)) << error;
  stats = plugin.targetCellCacheStats();
  EXPECT_EQ(stats.size, 3U);
  EXPECT_EQ(stats.hits, 1U);
  EXPECT_EQ(stats.misses, 4U);
  EXPECT_EQ(stats.evictions, 1U);
  ASSERT_TRUE(plugin.setInputTarget(targets[1], &error)) << error;
  stats = plugin.targetCellCacheStats();
  EXPECT_EQ(stats.size, 3U);
  EXPECT_EQ(stats.misses, 5U);
  EXPECT_EQ(stats.evictions, 2U);

  plugin.reset();
  stats = plugin.targetCellCacheStats();
  EXPECT_EQ(stats.capacity, 0U);
  EXPECT_EQ(stats.size, 0U);
  EXPECT_EQ(stats.hits, 0U);
  EXPECT_EQ(stats.misses, 0U);
  EXPECT_EQ(stats.evictions, 0U);
}

TEST(NdtOmpRegistration, PriorsAndDistanceAreClearedBetweenCalls)
{
  const Cloud::Ptr source = makeStructuredCloud();
  const Cloud::Ptr target = makeTargetCloud(source);

  lidarslam_default_plugins::NdtOmpRegistration plugin;
  std::string error;
  ASSERT_TRUE(plugin.configure(makeParameters(1), &error)) << error;
  ASSERT_TRUE(plugin.setInputTarget(target, &error)) << error;

  registration::RotationPrior rotation_prior;
  rotation_prior.enabled = true;
  rotation_prior.roll_pitch_yaw = Eigen::Vector3d(0.01, -0.02, 0.03);
  rotation_prior.weight = 0.25;
  rotation_prior.roll_pitch_only = false;
  registration::TranslationPrior translation_prior;
  translation_prior.enabled = true;
  translation_prior.position = Eigen::Vector3d(0.4, 0.2, 0.05);
  translation_prior.weights = Eigen::Vector3d(0.05, 0.05, 0.05);

  registration::AlignmentRequest with_overrides;
  with_overrides.source = source;
  with_overrides.rotation_prior = rotation_prior;
  with_overrides.translation_prior = translation_prior;
  with_overrides.maximum_correspondence_distance_enabled = true;
  with_overrides.maximum_correspondence_distance = 3.0;
  const registration::AlignmentResult overridden = plugin.align(with_overrides);
  ASSERT_EQ(overridden.failure, registration::FailureCode::kNone)
    << overridden.diagnostics.detail;
  const DirectResult expected_overridden =
    runDirect(source, target, 1, true, rotation_prior, translation_prior, true, 3.0);
  EXPECT_TRUE(bitwiseEqual(overridden.final_transformation, expected_overridden.matrix));
  EXPECT_DOUBLE_EQ(overridden.fitness_score, expected_overridden.fitness);

  // The same configured object must now be equivalent to a fresh no-prior,
  // no-distance NDT.  This catches leakage of all three per-call setters.
  registration::AlignmentRequest clean;
  clean.source = source;
  clean.initial_guess_enabled = false;
  clean.initial_guess.setConstant(std::numeric_limits<float>::quiet_NaN());
  const registration::AlignmentResult after_clear = plugin.align(clean);
  ASSERT_EQ(after_clear.failure, registration::FailureCode::kNone)
    << after_clear.diagnostics.detail;
  const DirectResult expected_clean = runDirect(source, target, 1, false);
  EXPECT_TRUE(bitwiseEqual(after_clear.final_transformation, expected_clean.matrix));
  EXPECT_DOUBLE_EQ(after_clear.fitness_score, expected_clean.fitness);
  ASSERT_TRUE(after_clear.aligned_source);
  EXPECT_TRUE(bitwiseEqual(*after_clear.aligned_source, expected_clean.aligned));
}

TEST(NdtOmpRegistration, ResetRemovesTargetAndAllowsFreshConfiguration)
{
  const Cloud::Ptr source = makeStructuredCloud();
  const Cloud::Ptr target = makeTargetCloud(source);
  lidarslam_default_plugins::NdtOmpRegistration plugin;
  std::string error;
  ASSERT_TRUE(plugin.configure(makeParameters(1), &error)) << error;
  ASSERT_TRUE(plugin.setInputTarget(target, &error)) << error;
  plugin.reset();

  registration::AlignmentRequest request;
  request.source = source;
  registration::AlignmentResult result = plugin.align(request);
  EXPECT_EQ(result.failure, registration::FailureCode::kNotConfigured);
  EXPECT_FALSE(result.aligned_source);

  ASSERT_TRUE(plugin.configure(makeParameters(1), &error)) << error;
  ASSERT_TRUE(plugin.setInputTarget(target, &error)) << error;
  result = plugin.align(request);
  EXPECT_EQ(result.failure, registration::FailureCode::kNone) << result.diagnostics.detail;
}

TEST(NdtOmpRegistration, FixedThreadRepeatabilityIsMeasured)
{
  const Cloud::Ptr source = makeStructuredCloud();
  const Cloud::Ptr target = makeTargetCloud(source);
  for (const int threads : {2, 4}) {
    const registration::AlignmentResult first = runPlugin(threads, source, target);
    ASSERT_EQ(first.failure, registration::FailureCode::kNone) << first.diagnostics.detail;
    ASSERT_TRUE(first.converged);

    float max_matrix_deviation = 0.0F;
    double max_fitness_deviation = 0.0;
    for (int repeat = 1; repeat < 5; ++repeat) {
      const registration::AlignmentResult next = runPlugin(threads, source, target);
      ASSERT_EQ(next.failure, registration::FailureCode::kNone) << next.diagnostics.detail;
      ASSERT_TRUE(next.converged);
      max_matrix_deviation = std::max(
        max_matrix_deviation,
        (first.final_transformation - next.final_transformation).cwiseAbs().maxCoeff());
      max_fitness_deviation = std::max(
        max_fitness_deviation, std::abs(first.fitness_score - next.fitness_score));
      EXPECT_LT(translationDelta(first.final_transformation, next.final_transformation), 1e-3F);
    }
    std::cout << "[determinism] threads=" << threads
              << " max_matrix_dev=" << max_matrix_deviation
              << " max_fitness_dev=" << max_fitness_deviation << std::endl;
  }
}

TEST(NdtOmpRegistration, CrossThreadCountMatchesSingleThreadBounds)
{
  const Cloud::Ptr source = makeStructuredCloud();
  const Cloud::Ptr target = makeTargetCloud(source);
  const registration::AlignmentResult base = runPlugin(1, source, target);
  ASSERT_EQ(base.failure, registration::FailureCode::kNone) << base.diagnostics.detail;
  ASSERT_TRUE(base.converged);

  for (const int threads : {2, 4}) {
    const registration::AlignmentResult other = runPlugin(threads, source, target);
    ASSERT_EQ(other.failure, registration::FailureCode::kNone) << other.diagnostics.detail;
    ASSERT_TRUE(other.converged);
    const float translation_delta =
      ::translationDelta(base.final_transformation, other.final_transformation);
    const float rotation_delta =
      ::rotationDeltaDeg(base.final_transformation, other.final_transformation);
    std::cout << "[determinism] threads_1_vs_" << threads
              << " trans_delta_m=" << translation_delta
              << " rot_delta_deg=" << rotation_delta << std::endl;
    EXPECT_LT(translation_delta, 5e-3F);
    EXPECT_LT(rotation_delta, 0.1F);
  }
}

}  // namespace
