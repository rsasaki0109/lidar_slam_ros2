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

// Characterization tests for the BackendCore descriptor ingestion moved
// out of searchLoop. These pin the historical catch-up semantics (each
// enabled database family queries the provider independently for exactly
// the missing indices, scan context stores a zero descriptor for empty
// clouds while the other families always store a computed one) and the
// determinism contract seed: the same ordered input produces bitwise
// identical descriptor state across instances.

#include <gtest/gtest.h>

#include <random>
#include <string>
#include <utility>
#include <vector>

#include <pclomp/ndt_omp.h>  // NOLINT(build/include_order)
// The prebuilt ndt_omp library only instantiates PointXYZ; pull in the
// template implementations for the PointXYZI instantiation used here.
#include <pclomp/ndt_omp_impl.hpp>  // NOLINT(build/include_order)
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>  // NOLINT(build/include_order)

#include "graph_based_slam/backend_core.hpp"

namespace graphslam
{
namespace
{

using backend_core::BackendCore;
using backend_core::DescriptorConfig;
using CloudPtr = BackendCore::CloudPtr;

CloudPtr makeCloud(int idx)
{
  CloudPtr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  for (int i = 0; i < 8; ++i) {
    for (int j = 0; j < 8; ++j) {
      pcl::PointXYZI pt;
      pt.x = static_cast<float>(i * 2 + idx);
      pt.y = static_cast<float>(j * 2 - idx);
      pt.z = static_cast<float>((i + j + idx) % 4);
      pt.intensity = 1.0f;
      cloud->push_back(pt);
    }
  }
  return cloud;
}

// Provider that records every requested index in call order.
struct RecordingProvider
{
  std::vector<int> calls;
  bool return_empty{false};

  BackendCore::LocalSubmapProvider asProvider()
  {
    return [this](int idx) -> CloudPtr {
             calls.push_back(idx);
             if (return_empty) {
               return CloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
             }
             return makeCloud(idx);
           };
  }
};

TEST(BackendCoreIngestion, CatchesUpOnlyTheMissingIndices)
{
  DescriptorConfig config;
  config.use_scan_context = true;
  BackendCore core;
  core.configure(config);
  RecordingProvider provider;

  core.ingestDescriptors(3, provider.asProvider());
  EXPECT_EQ(provider.calls, (std::vector<int>{0, 1, 2}));
  EXPECT_EQ(core.scanContextDb().nextSubmapIndex(), 3);

  provider.calls.clear();
  core.ingestDescriptors(5, provider.asProvider());
  EXPECT_EQ(provider.calls, (std::vector<int>{3, 4}));
  EXPECT_EQ(core.scanContextDb().nextSubmapIndex(), 5);

  provider.calls.clear();
  core.ingestDescriptors(5, provider.asProvider());
  EXPECT_TRUE(provider.calls.empty());
}

TEST(BackendCoreIngestion, DisabledFamiliesNeverQueryTheProvider)
{
  DescriptorConfig config;
  BackendCore core;
  core.configure(config);
  RecordingProvider provider;

  core.ingestDescriptors(4, provider.asProvider());
  EXPECT_TRUE(provider.calls.empty());
  EXPECT_EQ(core.scanContextDb().size(), 0);
  EXPECT_EQ(core.bevDescriptorDb().size(), 0);
  EXPECT_EQ(core.solidDescriptorDb().size(), 0);
  EXPECT_EQ(core.triangleDb().submapCount(), 0u);
}

TEST(BackendCoreIngestion, EachEnabledFamilyQueriesIndependentlyInOrder)
{
  DescriptorConfig config;
  config.use_scan_context = true;
  config.use_bev_descriptor = true;
  BackendCore core;
  core.configure(config);
  RecordingProvider provider;

  core.ingestDescriptors(2, provider.asProvider());
  EXPECT_EQ(provider.calls, (std::vector<int>{0, 1, 0, 1}));
  EXPECT_EQ(core.scanContextDb().size(), 2);
  EXPECT_EQ(core.bevDescriptorDb().size(), 2);
}

TEST(BackendCoreIngestion, EmptyCloudStoresZeroScanContextDescriptor)
{
  DescriptorConfig config;
  config.use_scan_context = true;
  config.use_solid_descriptor = true;
  BackendCore core;
  core.configure(config);
  RecordingProvider provider;
  provider.return_empty = true;

  core.ingestDescriptors(1, provider.asProvider());
  ASSERT_EQ(core.scanContextDb().size(), 1);
  const auto & desc = core.scanContextDb().descriptors[0];
  EXPECT_EQ(static_cast<int>(desc.rows()), static_cast<int>(ScanContext::NUM_RINGS));
  EXPECT_EQ(static_cast<int>(desc.cols()), static_cast<int>(ScanContext::NUM_SECTORS));
  EXPECT_DOUBLE_EQ(desc.cwiseAbs().maxCoeff(), 0.0);
  // The other families historically add a descriptor computed from the
  // empty cloud rather than skipping the index.
  EXPECT_EQ(core.solidDescriptorDb().size(), 1);
}

TEST(BackendCoreIngestion, TriangleFeaturesStayAlignedWithSubmapIndices)
{
  DescriptorConfig config;
  config.use_triangle_descriptor = true;
  BackendCore core;
  core.configure(config);
  RecordingProvider provider;

  core.ingestDescriptors(3, provider.asProvider());
  EXPECT_EQ(provider.calls, (std::vector<int>{0, 1, 2}));
  EXPECT_EQ(core.trianglePerSubmap().size(), 3u);
  EXPECT_EQ(core.triangleDb().submapCount(), 3u);

  provider.calls.clear();
  core.ingestDescriptors(3, provider.asProvider());
  EXPECT_TRUE(provider.calls.empty());
  EXPECT_EQ(core.trianglePerSubmap().size(), 3u);
}

TEST(BackendCoreIngestion, SameOrderedInputProducesBitwiseIdenticalState)
{
  DescriptorConfig config;
  config.use_scan_context = true;
  config.use_bev_descriptor = true;
  config.use_solid_descriptor = true;
  BackendCore core_a;
  BackendCore core_b;
  core_a.configure(config);
  core_b.configure(config);
  RecordingProvider provider_a;
  RecordingProvider provider_b;

  core_a.ingestDescriptors(2, provider_a.asProvider());
  core_a.ingestDescriptors(4, provider_a.asProvider());
  core_b.ingestDescriptors(2, provider_b.asProvider());
  core_b.ingestDescriptors(4, provider_b.asProvider());

  ASSERT_EQ(core_a.scanContextDb().size(), 4);
  ASSERT_EQ(core_b.scanContextDb().size(), 4);
  for (int i = 0; i < 4; ++i) {
    const auto & desc_a = core_a.scanContextDb().descriptors[i];
    const auto & desc_b = core_b.scanContextDb().descriptors[i];
    EXPECT_DOUBLE_EQ((desc_a - desc_b).cwiseAbs().maxCoeff(), 0.0);
    const auto & bev_a = core_a.bevDescriptorDb().descriptors[i];
    const auto & bev_b = core_b.bevDescriptorDb().descriptors[i];
    EXPECT_FLOAT_EQ((bev_a.occupancy - bev_b.occupancy).cwiseAbs().maxCoeff(), 0.0f);
    EXPECT_FLOAT_EQ((bev_a.density - bev_b.density).cwiseAbs().maxCoeff(), 0.0f);
    EXPECT_FLOAT_EQ((bev_a.max_height - bev_b.max_height).cwiseAbs().maxCoeff(), 0.0f);
    EXPECT_FLOAT_EQ((bev_a.coarse_key - bev_b.coarse_key).cwiseAbs().maxCoeff(), 0.0f);
    const auto & solid_a = core_a.solidDescriptorDb().descriptors[i];
    const auto & solid_b = core_b.solidDescriptorDb().descriptors[i];
    EXPECT_DOUBLE_EQ((solid_a.range - solid_b.range).cwiseAbs().maxCoeff(), 0.0);
    EXPECT_DOUBLE_EQ((solid_a.angle - solid_b.angle).cwiseAbs().maxCoeff(), 0.0);
    EXPECT_DOUBLE_EQ((solid_a.solid - solid_b.solid).cwiseAbs().maxCoeff(), 0.0);
  }
}

// --- searchLoopForSubmap characterization -------------------------------
//
// A revisit scenario driven by the DISTANCE source only: submap 0 and the
// query submap share the same structured cloud near the origin, the
// in-between submaps sit far away (and return empty clouds), so the only
// loop candidate is 0 and real NDT verifies it.

pcl::PointCloud<pcl::PointXYZI>::Ptr makeStructuredCloud()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> jitter(-0.05f, 0.05f);
  for (int i = 0; i < 20; ++i) {
    for (int j = 0; j < 20; ++j) {
      pcl::PointXYZI pt;
      pt.x = static_cast<float>(i) - 10.0f + jitter(rng);
      pt.y = static_cast<float>(j) - 10.0f + jitter(rng);
      pt.z = static_cast<float>((i * 7 + j * 3) % 5) * 0.4f + jitter(rng);
      pt.intensity = 1.0f;
      cloud->push_back(pt);
    }
  }
  return cloud;
}

std::vector<backend_core::SubmapMeta> makeRevisitTrajectory()
{
  std::vector<backend_core::SubmapMeta> submaps(11);
  for (int i = 0; i < 11; ++i) {
    submaps[i].travel_distance = 10.0 * i;
    if (i == 0) {
      submaps[i].pose = Eigen::Affine3d::Identity();
    } else if (i == 10) {
      submaps[i].pose = Eigen::Affine3d(Eigen::Translation3d(0.3, 0.0, 0.0));
    } else {
      submaps[i].pose = Eigen::Affine3d(Eigen::Translation3d(0.0, 1000.0 + i, 0.0));
    }
  }
  return submaps;
}

BackendCore::LocalSubmapProvider makeRevisitProvider()
{
  return [](int idx) -> CloudPtr {
           if (idx == 0 || idx == 10) {
             return makeStructuredCloud();
           }
           return CloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
         };
}

backend_core::LoopSearchConfig makeRevisitSearchConfig()
{
  backend_core::LoopSearchConfig config;
  config.search_submap_num = 1;
  config.aggregator.max_loop_candidate_count = 3;
  config.aggregator.distance_loop_closure = 20.0;
  config.aggregator.range_of_searching_loop_closure = 10.0;
  config.gates.generic_score_threshold = 10.0;
  config.gates.max_translation_m = 10.0;
  config.gates.max_rotation_deg = 180.0;
  return config;
}

struct SearchHarness
{
  BackendCore core;
  pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
  pcl::VoxelGrid<pcl::PointXYZI> voxelgrid;
  ThreeDBBSLoopVerifier bbs_verifier;

  SearchHarness()
  {
    DescriptorConfig descriptor_config;
    core.configure(descriptor_config);
    ndt.setNumThreads(1);
    ndt.setNeighborhoodSearchMethod(pclomp::DIRECT7);
    ndt.setResolution(2.0F);
    ndt.setMaximumIterations(35);
    ndt.setTransformationEpsilon(0.01);
    voxelgrid.setLeafSize(0.2f, 0.2f, 0.2f);
  }

  backend_core::LoopSearchOutput run(const backend_core::LoopSearchConfig & config)
  {
    return core.searchLoopForSubmap(
      makeRevisitTrajectory(), 10, config, makeRevisitProvider(), ndt, voxelgrid,
      bbs_verifier);
  }
};

TEST(BackendCoreSearch, FindsTheDistanceRevisitLoop)
{
  SearchHarness harness;
  const auto output = harness.run(makeRevisitSearchConfig());

  ASSERT_TRUE(output.proposal.found);
  EXPECT_EQ(output.proposal.pair_id, (std::pair<int, int>(0, 10)));
  EXPECT_LT(output.proposal.fitness_score, 10.0);
  ASSERT_FALSE(output.logs.empty());
  EXPECT_EQ(output.logs[0].text, "---");
  EXPECT_FALSE(output.logs[0].via_logger);
  bool has_id_line = false;
  for (const auto & line : output.logs) {
    if (line.text == "id_loop_point 1:0 id_loop_point 2:10") {
      has_id_line = true;
    }
  }
  EXPECT_TRUE(has_id_line);
}

TEST(BackendCoreSearch, SameInputProducesBitwiseIdenticalProposalAndLogs)
{
  SearchHarness harness_a;
  SearchHarness harness_b;
  const auto output_a = harness_a.run(makeRevisitSearchConfig());
  const auto output_b = harness_b.run(makeRevisitSearchConfig());

  ASSERT_TRUE(output_a.proposal.found);
  ASSERT_TRUE(output_b.proposal.found);
  EXPECT_DOUBLE_EQ(
    (output_a.proposal.relative_pose.matrix() -
    output_b.proposal.relative_pose.matrix()).cwiseAbs().maxCoeff(),
    0.0);
  EXPECT_DOUBLE_EQ(output_a.proposal.fitness_score, output_b.proposal.fitness_score);
  ASSERT_EQ(output_a.logs.size(), output_b.logs.size());
  for (std::size_t i = 0; i < output_a.logs.size(); ++i) {
    EXPECT_EQ(output_a.logs[i].via_logger, output_b.logs[i].via_logger);
    EXPECT_EQ(output_a.logs[i].text, output_b.logs[i].text);
  }
}

TEST(BackendCoreSearch, TranslationCapRejectionKeepsBestAttemptLine)
{
  SearchHarness harness;
  backend_core::LoopSearchConfig config = makeRevisitSearchConfig();
  config.aggregator.debug = true;
  config.gates.max_translation_m = 1e-9;
  const auto output = harness.run(config);

  EXPECT_FALSE(output.proposal.found);
  bool has_rejection = false;
  bool has_best_attempt = false;
  for (const auto & line : output.logs) {
    if (line.via_logger &&
      line.text.rfind("Rejected loop candidate 0 -> 10 because translation correction", 0) == 0)
    {
      has_rejection = true;
    }
    if (!line.via_logger && line.text.rfind("best_loop_candidate id:0", 0) == 0) {
      has_best_attempt = true;
    }
  }
  EXPECT_TRUE(has_rejection);
  EXPECT_TRUE(has_best_attempt);
}

TEST(BackendCoreSearch, EmptyCloudsReturnNoProposalAndNoLogs)
{
  SearchHarness harness;
  const auto provider = [](int) -> CloudPtr {
      return CloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
    };
  const auto output = harness.core.searchLoopForSubmap(
    makeRevisitTrajectory(), 10, makeRevisitSearchConfig(), provider, harness.ndt,
    harness.voxelgrid, harness.bbs_verifier);

  EXPECT_FALSE(output.proposal.found);
  EXPECT_TRUE(output.logs.empty());
}

}  // namespace
}  // namespace graphslam
