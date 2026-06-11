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

// Phase 0 determinism characterization for pclomp NDT registration
// (docs/roadmap/v0.6.md). Single-threaded alignment must be bitwise
// repeatable; multi-threaded repeatability and cross-thread-count
// consistency are measured and reported (grep "[determinism]" in the
// ctest log), not demanded.

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <iostream>
#include <random>
#include <vector>

#include <Eigen/Core>  // NOLINT(build/include_order)
#include <Eigen/Geometry>  // NOLINT(build/include_order)
#include <pcl/point_cloud.h>  // NOLINT(build/include_order)
#include <pcl/point_types.h>  // NOLINT(build/include_order)
#include <pclomp/ndt_omp.h>  // NOLINT(build/include_order)
// ndt_omp_ros2 installs no compiled library; template definitions come from
// the impl headers, the same way graph_based_slam_component.h consumes them.
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>

namespace
{

using CloudT = pcl::PointCloud<pcl::PointXYZ>;

CloudT::Ptr makeStructuredCloud()
{
  auto cloud = CloudT::Ptr(new CloudT());
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> jitter(-0.02F, 0.02F);

  // Ground plane 20 x 20 m at z = 0.
  for (float x = -10.0F; x <= 10.0F; x += 0.4F) {
    for (float y = -10.0F; y <= 10.0F; y += 0.4F) {
      cloud->push_back(pcl::PointXYZ(x + jitter(rng), y + jitter(rng), jitter(rng)));
    }
  }
  // Wall at x = 10, 3 m tall.
  for (float y = -10.0F; y <= 10.0F; y += 0.3F) {
    for (float z = 0.0F; z <= 3.0F; z += 0.3F) {
      cloud->push_back(pcl::PointXYZ(10.0F + jitter(rng), y + jitter(rng), z + jitter(rng)));
    }
  }
  // Wall at y = -10, 3 m tall.
  for (float x = -10.0F; x <= 10.0F; x += 0.3F) {
    for (float z = 0.0F; z <= 3.0F; z += 0.3F) {
      cloud->push_back(pcl::PointXYZ(x + jitter(rng), -10.0F + jitter(rng), z + jitter(rng)));
    }
  }
  // A free-standing pillar for asymmetry.
  for (float z = 0.0F; z <= 3.0F; z += 0.1F) {
    for (float a = 0.0F; a < 6.28F; a += 0.5F) {
      cloud->push_back(
        pcl::PointXYZ(3.0F + 0.3F * std::cos(a) + jitter(rng),
                      4.0F + 0.3F * std::sin(a) + jitter(rng), z + jitter(rng)));
    }
  }
  return cloud;
}

Eigen::Matrix4f knownTransform()
{
  Eigen::Affine3f t = Eigen::Translation3f(0.4F, 0.2F, 0.05F) *
    Eigen::AngleAxisf(8.0F * static_cast<float>(M_PI) / 180.0F, Eigen::Vector3f::UnitZ());
  return t.matrix();
}

CloudT::Ptr makeTargetCloud(const CloudT::Ptr & source)
{
  auto target = CloudT::Ptr(new CloudT());
  const Eigen::Matrix4f t = knownTransform();
  target->reserve(source->size());
  for (const auto & p : *source) {
    const Eigen::Vector4f v = t * Eigen::Vector4f(p.x, p.y, p.z, 1.0F);
    target->push_back(pcl::PointXYZ(v.x(), v.y(), v.z()));
  }
  return target;
}

struct AlignResult
{
  Eigen::Matrix4f matrix;
  double fitness;
  bool converged;
};

AlignResult runAlign(int num_threads)
{
  // Fresh inputs and a fresh NDT object every call: no shared mutable state.
  const CloudT::Ptr source = makeStructuredCloud();
  const CloudT::Ptr target = makeTargetCloud(source);

  pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  ndt.setNumThreads(num_threads);
  ndt.setNeighborhoodSearchMethod(pclomp::DIRECT7);
  ndt.setResolution(2.0F);
  ndt.setMaximumIterations(35);
  ndt.setTransformationEpsilon(0.01);
  ndt.setInputSource(source);
  ndt.setInputTarget(target);

  CloudT aligned;
  ndt.align(aligned, Eigen::Matrix4f::Identity());

  return AlignResult{ndt.getFinalTransformation(), ndt.getFitnessScore(), ndt.hasConverged()};
}

bool bitwiseEqual(const Eigen::Matrix4f & a, const Eigen::Matrix4f & b)
{
  return std::memcmp(a.data(), b.data(), sizeof(float) * 16) == 0;
}

float maxAbsDeviation(const Eigen::Matrix4f & a, const Eigen::Matrix4f & b)
{
  return (a - b).cwiseAbs().maxCoeff();
}

float translationDelta(const Eigen::Matrix4f & a, const Eigen::Matrix4f & b)
{
  return (a.block<3, 1>(0, 3) - b.block<3, 1>(0, 3)).norm();
}

float rotationDeltaDeg(const Eigen::Matrix4f & a, const Eigen::Matrix4f & b)
{
  const Eigen::Matrix3f r = a.block<3, 3>(0, 0).transpose() * b.block<3, 3>(0, 0);
  const float c = std::min(1.0F, std::max(-1.0F, (r.trace() - 1.0F) / 2.0F));
  return std::acos(c) * 180.0F / static_cast<float>(M_PI);
}

TEST(RegistrationDeterminism, SingleThreadIsBitwiseRepeatable)
{
  const AlignResult first = runAlign(1);
  ASSERT_TRUE(first.converged);

  for (int i = 1; i < 5; ++i) {
    const AlignResult next = runAlign(1);
    ASSERT_TRUE(next.converged);
    EXPECT_TRUE(bitwiseEqual(first.matrix, next.matrix))
      << "single-thread NDT diverged on repeat " << i
      << " max_dev=" << maxAbsDeviation(first.matrix, next.matrix);
    EXPECT_EQ(first.fitness, next.fitness) << "fitness diverged on repeat " << i;
  }
}

TEST(RegistrationDeterminism, MultiThreadRepeatabilityReport)
{
  for (int threads : {2, 4}) {
    const AlignResult first = runAlign(threads);
    ASSERT_TRUE(first.converged);

    float max_matrix_dev = 0.0F;
    double max_fitness_dev = 0.0;
    for (int i = 1; i < 5; ++i) {
      const AlignResult next = runAlign(threads);
      ASSERT_TRUE(next.converged);
      max_matrix_dev = std::max(max_matrix_dev, maxAbsDeviation(first.matrix, next.matrix));
      max_fitness_dev = std::max(max_fitness_dev, std::abs(first.fitness - next.fitness));
      // Characterization, not a contract: only a loose sanity bound.
      EXPECT_LT(translationDelta(first.matrix, next.matrix), 1e-3F);
    }
    std::cout << "[determinism] threads=" << threads << " max_matrix_dev=" << max_matrix_dev
              << " max_fitness_dev=" << max_fitness_dev << std::endl;
  }
}

TEST(RegistrationDeterminism, CrossThreadCountConsistencyReport)
{
  const AlignResult base = runAlign(1);
  ASSERT_TRUE(base.converged);

  for (int threads : {2, 4}) {
    const AlignResult other = runAlign(threads);
    ASSERT_TRUE(other.converged);
    const float trans_delta = translationDelta(base.matrix, other.matrix);
    const float rot_delta = rotationDeltaDeg(base.matrix, other.matrix);
    std::cout << "[determinism] threads_1_vs_" << threads << " trans_delta_m=" << trans_delta
              << " rot_delta_deg=" << rot_delta << std::endl;
    EXPECT_LT(trans_delta, 5e-3F);
    EXPECT_LT(rot_delta, 0.1F);
  }
}

}  // namespace
