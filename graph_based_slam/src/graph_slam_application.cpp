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

#include "graph_based_slam/graph_slam_application.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

#include "graph_based_slam/loop_search_schedule.hpp"

namespace graphslam
{

GraphSlamApplication::GraphSlamApplication(
  GraphSlamApplicationConfig config,
  GraphSlamApplicationDependencies dependencies)
: config_(std::move(config)), dependencies_(dependencies)
{
  if (config_.loop_search_query_stride < 1) {
    throw std::invalid_argument("loop_search_query_stride must be at least 1");
  }
  if (config_.loop_edge_dedup_index_window < 0) {
    throw std::invalid_argument("loop_edge_dedup_index_window must not be negative");
  }
  dependencies_.backend.configure(config_.descriptors);
  loop_edges_.configure(config_.loop_edge_dedup_index_window);
}

std::vector<LoopSearchEvent> GraphSlamApplication::processSubmaps(
  const std::vector<backend_core::SubmapMeta> & ordered_submaps,
  const LocalSubmapProvider & filtered_local_provider,
  const LocalSubmapProvider & raw_cloud_provider)
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<LoopSearchEvent> events;
  const int submap_count = static_cast<int>(ordered_submaps.size());
  if (next_query_index_ >= submap_count) {
    return events;
  }

  events.reserve(static_cast<std::size_t>(submap_count - next_query_index_));
  for (; next_query_index_ < submap_count; ++next_query_index_) {
    LoopSearchEvent event;
    event.query_index = next_query_index_;
    dependencies_.backend.ingestDescriptors(next_query_index_ + 1, filtered_local_provider);
    event.registration_searched = loop_search_schedule::shouldSearch(
      next_query_index_, config_.loop_search_query_stride);
    if (event.registration_searched) {
      // Expose only the ordered prefix visible to this query. This makes
      // batching observationally equivalent to one-submap-at-a-time input.
      std::vector<backend_core::SubmapMeta> visible(
        ordered_submaps.begin(), ordered_submaps.begin() + next_query_index_ + 1);
      event.search_output = dependencies_.backend.searchLoopForSubmap(
        visible, next_query_index_, config_.loop_search, raw_cloud_provider,
        dependencies_.registration, dependencies_.voxelgrid,
        dependencies_.three_d_bbs_verifier);
      if (event.search_output.proposal.found) {
        LoopEdge edge;
        edge.pair_id = event.search_output.proposal.pair_id;
        edge.relative_pose = event.search_output.proposal.relative_pose;
        edge.fitness_score = event.search_output.proposal.fitness_score;
        event.graph_changed = loop_edges_.upsert(edge);
      }
    }
    event.loop_edges = loop_edges_.edges();
    events.push_back(std::move(event));
  }
  return events;
}

bool GraphSlamApplication::upsertLoopEdge(const LoopEdge & edge)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return loop_edges_.upsert(edge);
}

std::vector<GraphSlamApplication::LoopEdge> GraphSlamApplication::loopEdges() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return loop_edges_.edges();
}

int GraphSlamApplication::nextQueryIndex() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return next_query_index_;
}

pose_graph::OptimizationResult GraphSlamApplication::optimize(
  const PoseGraphRequest & request) const
{
  std::vector<pose_graph::LoopConstraint> loop_constraints;
  loop_constraints.reserve(request.loop_edges.size());
  for (const auto & edge : request.loop_edges) {
    pose_graph::LoopConstraint constraint;
    constraint.from = edge.pair_id.first;
    constraint.to = edge.pair_id.second;
    constraint.relative_pose = edge.relative_pose;
    constraint.fitness_score = edge.fitness_score;
    loop_constraints.push_back(constraint);
  }
  return pose_graph::optimizePoseGraph(
    request.submaps, loop_constraints, request.imu_constraints,
    request.gnss_constraints, request.adjacent_config, request.loop_config,
    request.imu_config, request.chi2_collection, request.fix_first_vertex,
    request.iterations, request.save_path, request.plane_constraints);
}

}  // namespace graphslam
