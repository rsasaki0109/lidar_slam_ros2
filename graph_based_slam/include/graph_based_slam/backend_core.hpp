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

#ifndef GRAPH_BASED_SLAM__BACKEND_CORE_HPP_
#define GRAPH_BASED_SLAM__BACKEND_CORE_HPP_

// The ROS-free, clock-free backend state (docs/roadmap/v0.6.md, Phase 2).
// This first stage owns the four loop-closure descriptor databases and
// their submap-ingestion logic; the search and optimization orchestration
// migrate here in later stages. The contract this class builds toward:
// the same ordered submap sequence plus the same config produce the same
// state, independent of wall-clock timing. The shell supplies clouds via
// a provider callback, so message-vs-PCD-cache stays its concern.

#include <functional>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>  // NOLINT(build/include_order)
#include <pcl/point_types.h>  // NOLINT(build/include_order)

#include "graph_based_slam/candidate_aggregator.hpp"
#include "graph_based_slam/scan_context.hpp"
#include "graph_based_slam/solid_descriptor.hpp"
#include "graph_based_slam/submap_bev_descriptor.hpp"
#include "graph_based_slam/triangle_descriptor.hpp"
#include "graph_based_slam/triangle_descriptor_database.hpp"

namespace graphslam
{
namespace backend_core
{

struct DescriptorConfig
{
  bool use_scan_context{false};
  bool use_bev_descriptor{false};
  bool use_solid_descriptor{false};
  bool use_triangle_descriptor{false};
  double bev_descriptor_grid_size_m{80.0};
  int bev_descriptor_grid_cells{40};
  int bev_descriptor_yaw_bins{24};
  std::string triangle_descriptor_keypoint_mode{"bev_max_height"};
  double triangle_descriptor_grid_size_m{60.0};
  int triangle_descriptor_grid_cells{100};
  double triangle_descriptor_min_salience_m{0.8};
  int triangle_descriptor_max_keypoints{40};
  double triangle_descriptor_edge_voxel_size_m{0.4};
  double triangle_descriptor_edge_neighbor_radius_m{1.0};
  int triangle_descriptor_edge_min_neighbors{6};
  double triangle_descriptor_edge_min_edgeness{0.5};
  double triangle_descriptor_edge_nms_radius_m{2.0};
  double triangle_descriptor_min_edge_m{2.0};
  double triangle_descriptor_max_edge_m{50.0};
  int triangle_descriptor_max_triangles{3000};
  double triangle_descriptor_edge_bin_m{0.5};
  double triangle_descriptor_quad_feature_bin_m{0.0};
};

// Backend-owned loop-closure state. Single-threaded by contract: the
// caller serializes access (today the component's SingleThreadedExecutor,
// later the shell's processing queue / the offline runner's bag loop).
class BackendCore
{
public:
  using CloudPtr = pcl::PointCloud<pcl::PointXYZI>::Ptr;
  // Returns the voxel-filtered aggregate of the submap at idx and its
  // recent neighbors, expressed in that submap's local frame.
  using LocalSubmapProvider = std::function<CloudPtr(int idx)>;

  void configure(const DescriptorConfig & config)
  {
    config_ = config;
    bev_descriptor_db_.configure(
      config.bev_descriptor_grid_size_m,
      config.bev_descriptor_grid_cells,
      config.bev_descriptor_yaw_bins);
  }

  // Keep every enabled descriptor database aligned 1:1 with submap
  // indices, computing descriptors for indices that arrived since the
  // last call. Each database family queries the provider independently
  // (historical behavior; the provider result is deterministic, so the
  // recompute is byte-identical).
  void ingestDescriptors(int num_submaps, const LocalSubmapProvider & provider)
  {
    if (config_.use_scan_context && scan_context_db_.nextSubmapIndex() < num_submaps) {
      for (int idx = scan_context_db_.nextSubmapIndex(); idx < num_submaps; ++idx) {
        const auto filtered_aggregated_cloud = provider(idx);
        if (filtered_aggregated_cloud->empty()) {
          scan_context_db_.add(
            idx, ScanContext::Descriptor::Zero(
              ScanContext::NUM_RINGS,
              ScanContext::NUM_SECTORS));
          continue;
        }
        scan_context_db_.add(idx, ScanContext::computeDescriptor(filtered_aggregated_cloud));
      }
    }

    if (config_.use_bev_descriptor && bev_descriptor_db_.nextSubmapIndex() < num_submaps) {
      for (int idx = bev_descriptor_db_.nextSubmapIndex(); idx < num_submaps; ++idx) {
        const auto filtered_aggregated_cloud = provider(idx);
        bev_descriptor_db_.add(
          idx,
          SubmapBEVDescriptor::computeDescriptor(
            filtered_aggregated_cloud,
            config_.bev_descriptor_grid_size_m,
            config_.bev_descriptor_grid_cells));
      }
    }
    if (config_.use_solid_descriptor && solid_descriptor_db_.nextSubmapIndex() < num_submaps) {
      for (int idx = solid_descriptor_db_.nextSubmapIndex(); idx < num_submaps; ++idx) {
        const auto filtered_aggregated_cloud = provider(idx);
        solid_descriptor_db_.add(
          idx,
          SolidDescriptor::computeDescriptor(filtered_aggregated_cloud));
      }
    }
    if (config_.use_triangle_descriptor && triangle_next_submap_idx_ < num_submaps) {
      triangle::KeypointExtractionConfig kp_cfg;
      if (config_.triangle_descriptor_keypoint_mode == "edge_3d") {
        kp_cfg.mode = triangle::KeypointMode::EDGE_3D;
      } else {
        kp_cfg.mode = triangle::KeypointMode::BEV_MAX_HEIGHT;
      }
      kp_cfg.grid_size_m = config_.triangle_descriptor_grid_size_m;
      kp_cfg.grid_cells = config_.triangle_descriptor_grid_cells;
      kp_cfg.min_salience_m = static_cast<float>(config_.triangle_descriptor_min_salience_m);
      kp_cfg.max_keypoints = config_.triangle_descriptor_max_keypoints;
      kp_cfg.edge_voxel_size_m =
        static_cast<float>(config_.triangle_descriptor_edge_voxel_size_m);
      kp_cfg.edge_neighbor_radius_m =
        static_cast<float>(config_.triangle_descriptor_edge_neighbor_radius_m);
      kp_cfg.edge_min_neighbors = config_.triangle_descriptor_edge_min_neighbors;
      kp_cfg.edge_min_edgeness =
        static_cast<float>(config_.triangle_descriptor_edge_min_edgeness);
      kp_cfg.edge_nms_radius_m =
        static_cast<float>(config_.triangle_descriptor_edge_nms_radius_m);
      triangle::TriangleBuildConfig build_cfg;
      build_cfg.min_edge_m = static_cast<float>(config_.triangle_descriptor_min_edge_m);
      build_cfg.max_edge_m = static_cast<float>(config_.triangle_descriptor_max_edge_m);
      build_cfg.max_triangles = config_.triangle_descriptor_max_triangles;
      triangle::HashConfig hash_cfg;
      hash_cfg.edge_bin_m = static_cast<float>(config_.triangle_descriptor_edge_bin_m);
      hash_cfg.quad_feature_bin_m =
        static_cast<float>(config_.triangle_descriptor_quad_feature_bin_m);
      for (int idx = triangle_next_submap_idx_; idx < num_submaps; ++idx) {
        const auto filtered_aggregated_cloud = provider(idx);
        std::vector<triangle::Keypoint> kps;
        std::vector<triangle::TriangleDescriptor> tris;
        if (filtered_aggregated_cloud && !filtered_aggregated_cloud->empty()) {
          kps = triangle::extractKeypoints(*filtered_aggregated_cloud, kp_cfg);
          tris = triangle::buildTriangles(kps, build_cfg);
        }
        candidate_aggregator::TriangleSubmapFeatures entry;
        entry.keypoints = kps;
        entry.triangles = tris;
        triangle_per_submap_.push_back(entry);
        triangle_db_.addSubmap(idx, kps, tris, hash_cfg);
      }
      triangle_next_submap_idx_ = num_submaps;
    }
  }

  const DescriptorConfig & config() const {return config_;}

  ScanContext::Database & scanContextDb() {return scan_context_db_;}
  SubmapBEVDescriptor::Database & bevDescriptorDb() {return bev_descriptor_db_;}
  SolidDescriptor::Database & solidDescriptorDb() {return solid_descriptor_db_;}
  triangle::TriangleDatabase & triangleDb() {return triangle_db_;}
  const std::vector<candidate_aggregator::TriangleSubmapFeatures> & trianglePerSubmap() const
  {
    return triangle_per_submap_;
  }

private:
  DescriptorConfig config_;
  ScanContext::Database scan_context_db_;
  SubmapBEVDescriptor::Database bev_descriptor_db_;
  SolidDescriptor::Database solid_descriptor_db_;
  triangle::TriangleDatabase triangle_db_;
  std::vector<candidate_aggregator::TriangleSubmapFeatures> triangle_per_submap_;
  int triangle_next_submap_idx_{0};
};

}  // namespace backend_core
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__BACKEND_CORE_HPP_
