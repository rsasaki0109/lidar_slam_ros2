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

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include <pcl/registration/icp.h>  // NOLINT(build/include_order)

#include "graph_based_slam/graph_slam_application.hpp"

namespace graphslam
{
namespace
{

using Cloud = pcl::PointCloud<pcl::PointXYZI>;

struct Fixture
{
  backend_core::BackendCore backend;
  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> registration;
  pcl::VoxelGrid<pcl::PointXYZI> voxelgrid;
  ThreeDBBSLoopVerifier verifier;

  std::unique_ptr<GraphSlamApplication> makeApplication(int stride = 1)
  {
    GraphSlamApplicationConfig config;
    config.loop_search_query_stride = stride;
    return std::unique_ptr<GraphSlamApplication>(new GraphSlamApplication(
      config, {backend, registration, voxelgrid, verifier}));
  }
};

backend_core::BackendCore::CloudPtr emptyCloud(int index)
{
  static_cast<void>(index);
  backend_core::BackendCore::CloudPtr cloud(new Cloud);
  return cloud;
}

std::vector<backend_core::SubmapMeta> submaps(int count)
{
  std::vector<backend_core::SubmapMeta> result(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) {
    result[static_cast<std::size_t>(i)].pose.translation().x() = i * 10.0;
    result[static_cast<std::size_t>(i)].travel_distance = i * 10.0;
  }
  return result;
}

TEST(GraphSlamApplication, BatchAndIncrementalInputProduceTheSameQueryOrder)
{
  Fixture batch_fixture;
  auto batch = batch_fixture.makeApplication();
  const auto batch_events = batch->processSubmaps(submaps(5), emptyCloud, emptyCloud);

  Fixture incremental_fixture;
  auto incremental = incremental_fixture.makeApplication();
  std::vector<LoopSearchEvent> incremental_events;
  for (int count = 1; count <= 5; ++count) {
    const auto events = incremental->processSubmaps(submaps(count), emptyCloud, emptyCloud);
    incremental_events.insert(incremental_events.end(), events.begin(), events.end());
  }

  ASSERT_EQ(batch_events.size(), 4U);
  ASSERT_EQ(incremental_events.size(), batch_events.size());
  for (std::size_t i = 0; i < batch_events.size(); ++i) {
    EXPECT_EQ(batch_events[i].query_index, incremental_events[i].query_index);
    EXPECT_EQ(batch_events[i].registration_searched, incremental_events[i].registration_searched);
    EXPECT_EQ(batch_events[i].graph_changed, incremental_events[i].graph_changed);
    EXPECT_EQ(batch_events[i].loop_edges.size(), incremental_events[i].loop_edges.size());
  }
  EXPECT_EQ(batch->nextQueryIndex(), 5);
  EXPECT_EQ(incremental->nextQueryIndex(), 5);
}

TEST(GraphSlamApplication, StrideSkipsRegistrationButStillAdvancesEveryQuery)
{
  Fixture fixture;
  auto application = fixture.makeApplication(2);
  const auto events = application->processSubmaps(submaps(6), emptyCloud, emptyCloud);

  ASSERT_EQ(events.size(), 5U);
  EXPECT_TRUE(events[0].registration_searched);
  EXPECT_FALSE(events[1].registration_searched);
  EXPECT_TRUE(events[2].registration_searched);
  EXPECT_FALSE(events[3].registration_searched);
  EXPECT_TRUE(events[4].registration_searched);
  EXPECT_EQ(application->nextQueryIndex(), 6);
}

TEST(GraphSlamApplication, OwnsTheCanonicalDeduplicatedLoopEdgeSet)
{
  Fixture fixture;
  auto application = fixture.makeApplication();
  GraphSlamApplication::LoopEdge edge;
  edge.pair_id = {9, 2};
  edge.relative_pose.translation().x() = 7.0;
  edge.fitness_score = 0.3;

  ASSERT_TRUE(application->upsertLoopEdge(edge));
  const auto edges = application->loopEdges();
  ASSERT_EQ(edges.size(), 1U);
  EXPECT_EQ(edges[0].pair_id, std::make_pair(2, 9));
  EXPECT_DOUBLE_EQ(edges[0].relative_pose.translation().x(), -7.0);
  EXPECT_DOUBLE_EQ(edges[0].fitness_score, 0.3);
}

TEST(GraphSlamApplication, OptimizesPlainPoseGraphRequestsThroughTheSharedEntryPoint)
{
  Fixture fixture;
  auto application = fixture.makeApplication();
  PoseGraphRequest request;
  request.submaps.resize(1);
  request.submaps[0].pose.translation().x() = 3.5;

  const auto result = application->optimize(request);

  ASSERT_EQ(result.poses.size(), 1U);
  EXPECT_DOUBLE_EQ(result.poses[0].translation().x(), 3.5);
}

TEST(GraphSlamApplication, RejectsInvalidWorkflowConfiguration)
{
  Fixture fixture;
  GraphSlamApplicationConfig config;
  config.loop_search_query_stride = 0;
  EXPECT_THROW(
    GraphSlamApplication(config, {
        fixture.backend, fixture.registration, fixture.voxelgrid, fixture.verifier}),
    std::invalid_argument);
}

}  // namespace
}  // namespace graphslam
