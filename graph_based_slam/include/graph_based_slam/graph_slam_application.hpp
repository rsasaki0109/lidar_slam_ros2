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

#ifndef GRAPH_BASED_SLAM__GRAPH_SLAM_APPLICATION_HPP_
#define GRAPH_BASED_SLAM__GRAPH_SLAM_APPLICATION_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "graph_based_slam/backend_core.hpp"
#include "graph_based_slam/loop_edge_set.hpp"
#include "graph_based_slam/pose_graph_optimization.hpp"

namespace graphslam
{

// The ordered, ROS-free workflow boundary shared by live and replay adapters.
// Milestone 2 deliberately injects the compute-heavy engine objects; ownership
// of those resources moves into the backend in Milestone 3.
struct GraphSlamApplicationConfig
{
  backend_core::DescriptorConfig descriptors;
  backend_core::LoopSearchConfig loop_search;
  int loop_search_query_stride {1};
  int loop_edge_dedup_index_window {8};
};

struct GraphSlamApplicationDependencies
{
  backend_core::BackendCore & backend;
  pcl::Registration<pcl::PointXYZI, pcl::PointXYZI> & registration;
  pcl::VoxelGrid<pcl::PointXYZI> & voxelgrid;
  ThreeDBBSLoopVerifier & three_d_bbs_verifier;
};

struct LoopSearchEvent
{
  int query_index {-1};
  bool registration_searched {false};
  backend_core::LoopSearchOutput search_output;
  bool graph_changed {false};
  std::vector<backend_core::LoopEdgeSet::Edge> loop_edges;
};

struct PoseGraphRequest
{
  std::vector<pose_graph::SubmapNode> submaps;
  std::vector<backend_core::LoopEdgeSet::Edge> loop_edges;
  std::vector<pose_graph::ImuRotationConstraint> imu_constraints;
  std::vector<pose_graph::GnssConstraint> gnss_constraints;
  pose_graph::AdjacentEdgeConfig adjacent_config;
  pose_graph::LoopEdgeConfig loop_config;
  pose_graph::ImuEdgeConfig imu_config;
  pose_graph::Chi2Collection chi2_collection {pose_graph::Chi2Collection::NONE};
  bool fix_first_vertex {true};
  int iterations {10};
  std::string save_path;
  std::vector<pose_graph::PlaneRevisitConstraint> plane_constraints;
};

class GraphSlamApplication
{
public:
  using LocalSubmapProvider = backend_core::BackendCore::LocalSubmapProvider;
  using LoopEdge = backend_core::LoopEdgeSet::Edge;

  GraphSlamApplication(
    GraphSlamApplicationConfig config,
    GraphSlamApplicationDependencies dependencies);

  // Process every not-yet-observed query from the ordered prefix. The same
  // input prefix produces the same event order regardless of callback/bag
  // batching. Providers are invoked synchronously and are not retained.
  std::vector<LoopSearchEvent> processSubmaps(
    const std::vector<backend_core::SubmapMeta> & ordered_submaps,
    const LocalSubmapProvider & filtered_local_provider,
    const LocalSubmapProvider & raw_cloud_provider);

  bool upsertLoopEdge(const LoopEdge & edge);
  std::vector<LoopEdge> loopEdges() const;
  int nextQueryIndex() const;

  pose_graph::OptimizationResult optimize(const PoseGraphRequest & request) const;

private:
  GraphSlamApplicationConfig config_;
  GraphSlamApplicationDependencies dependencies_;
  mutable std::mutex mutex_;
  int next_query_index_ {1};
  backend_core::LoopEdgeSet loop_edges_;
};

}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__GRAPH_SLAM_APPLICATION_HPP_
