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

#include "graph_based_slam_component_impl.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <unordered_map>

#include <pcl/io/pcd_io.h>  // NOLINT(build/include_order)
#include <pcl/filters/voxel_grid.h>  // NOLINT(build/include_order)
#include <pcl_conversions/pcl_conversions.h>  // NOLINT(build/include_order)
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include "graph_based_slam/adjacent_edge_auto_scale.hpp"
#include "graph_based_slam/backend_core.hpp"
#include "graph_based_slam/bev_mutual_visibility.hpp"
#include "graph_based_slam/candidate_aggregator.hpp"
#include "graph_based_slam/degeneracy_diagnostics_csv.hpp"
#include "graph_based_slam/dynamic_object_filter.hpp"
#include "graph_based_slam/gnss_alignment.hpp"
#include "graph_based_slam/gnss_geometry.hpp"
#include "graph_based_slam/gnss_weighting.hpp"
#include "graph_based_slam/loop_verifier.hpp"
#include "graph_based_slam/loop_search_schedule.hpp"
#include "graph_based_slam/map_saver.hpp"
#include "graph_based_slam/planar_map_filter.hpp"
#include "graph_based_slam/pose_graph_optimization.hpp"
#include "graph_based_slam/registration_factory.hpp"
#include "graph_based_slam/submap_creation.hpp"
#include "g2o/core/robust_kernel_impl.h"
#define GRAPH_BASED_SLAM_WITH_G2O 1
#include "graph_based_slam/loop_edge_robustifier.hpp"

using namespace std::chrono_literals;

namespace graphslam
{
struct GraphBasedSlamComponent::Impl::BackendWorkspace
{
  backend_core::BackendCore core;
  boost::shared_ptr<pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>> registration;
  pcl::VoxelGrid<pcl::PointXYZI> voxelgrid;
  ThreeDBBSLoopVerifier three_d_bbs_loop_verifier;
};

GraphBasedSlamComponent::GraphBasedSlamComponent(const rclcpp::NodeOptions & options)
: Node("graph_based_slam", options),
  impl_(std::make_unique<Impl>(*this))
{
}

GraphBasedSlamComponent::~GraphBasedSlamComponent() = default;

GraphBasedSlamComponent::Impl::Impl(GraphBasedSlamComponent & node)
: node_(node),
  clock_(RCL_ROS_TIME),
  tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
  listener_(tfbuffer_),
  broadcaster_(&node_),
  backend_(std::make_unique<BackendWorkspace>())
{
  RCLCPP_INFO(node_.get_logger(), "initialization start");
  std::string registration_method;
  double voxel_leaf_size;
  double ndt_resolution;
  int ndt_num_threads;

  node_.declare_parameter("registration_method", "NDT");
  node_.get_parameter("registration_method", registration_method);
  node_.declare_parameter("voxel_leaf_size", 0.2);
  node_.get_parameter("voxel_leaf_size", voxel_leaf_size);
  node_.declare_parameter("ndt_resolution", 5.0);
  node_.get_parameter("ndt_resolution", ndt_resolution);
  node_.declare_parameter("ndt_num_threads", 0);
  node_.get_parameter("ndt_num_threads", ndt_num_threads);
  node_.declare_parameter("threshold_loop_closure_score", 1.0);
  node_.get_parameter("threshold_loop_closure_score", threshold_loop_closure_score_);
  node_.declare_parameter("scan_context_loop_closure_score_threshold", -1.0);
  node_.get_parameter(
    "scan_context_loop_closure_score_threshold",
    scan_context_loop_closure_score_threshold_);
  node_.declare_parameter("distance_loop_closure", 20.0);
  node_.get_parameter("distance_loop_closure", distance_loop_closure_);
  node_.declare_parameter("range_of_searching_loop_closure", 20.0);
  node_.get_parameter("range_of_searching_loop_closure", range_of_searching_loop_closure_);
  node_.declare_parameter("search_submap_num", 3);
  node_.get_parameter("search_submap_num", search_submap_num_);
  node_.declare_parameter("loop_search_query_stride", 1);
  node_.get_parameter("loop_search_query_stride", loop_search_query_stride_);
  node_.declare_parameter("max_loop_candidate_count", 3);
  node_.get_parameter("max_loop_candidate_count", max_loop_candidate_count_);
  node_.declare_parameter("loop_edge_dedup_index_window", 8);
  node_.get_parameter("loop_edge_dedup_index_window", loop_edge_dedup_index_window_);
  node_.declare_parameter("loop_max_translation_delta", 15.0);
  node_.get_parameter("loop_max_translation_delta", loop_max_translation_delta_);
  node_.declare_parameter("loop_max_rotation_delta_deg", 45.0);
  node_.get_parameter("loop_max_rotation_delta_deg", loop_max_rotation_delta_deg_);
  node_.declare_parameter("loop_min_overlap_ratio", 0.0);
  node_.get_parameter("loop_min_overlap_ratio", loop_min_overlap_ratio_);
  node_.declare_parameter("loop_min_overlap_ratio_large_correction", 0.0);
  node_.get_parameter(
    "loop_min_overlap_ratio_large_correction",
    loop_min_overlap_ratio_large_correction_);
  node_.declare_parameter("loop_overlap_large_correction_translation_m", 0.0);
  node_.get_parameter(
    "loop_overlap_large_correction_translation_m",
    loop_overlap_large_correction_translation_m_);
  node_.declare_parameter("loop_overlap_max_distance_m", 0.5);
  node_.get_parameter("loop_overlap_max_distance_m", loop_overlap_max_distance_m_);
  node_.declare_parameter("loop_max_translation_delta_descriptor", -1.0);
  node_.get_parameter(
    "loop_max_translation_delta_descriptor", loop_max_translation_delta_descriptor_);
  node_.declare_parameter("loop_max_rotation_delta_deg_descriptor", -1.0);
  node_.get_parameter(
    "loop_max_rotation_delta_deg_descriptor", loop_max_rotation_delta_deg_descriptor_);
  node_.declare_parameter("num_adjacent_pose_cnstraints", 5);
  node_.get_parameter("num_adjacent_pose_cnstraints", num_adjacent_pose_cnstraints_);
  node_.declare_parameter("use_save_map_in_loop", true);
  node_.get_parameter("use_save_map_in_loop", use_save_map_in_loop_);
  node_.declare_parameter("debug_flag", false);
  node_.get_parameter("debug_flag", debug_flag_);
  node_.declare_parameter("adjacent_edge_info_weight", 1000.0);
  node_.get_parameter("adjacent_edge_info_weight", adjacent_edge_info_weight_);
  node_.declare_parameter("adjacent_edge_info_auto_scale", false);
  node_.get_parameter("adjacent_edge_info_auto_scale", adjacent_edge_info_auto_scale_);
  node_.declare_parameter("adjacent_edge_info_auto_scale_target_nis", 6.0);
  node_.get_parameter(
    "adjacent_edge_info_auto_scale_target_nis",
    adjacent_edge_info_auto_scale_target_nis_);
  node_.declare_parameter("adjacent_edge_info_auto_scale_ema_alpha", 0.3);
  node_.get_parameter(
    "adjacent_edge_info_auto_scale_ema_alpha",
    adjacent_edge_info_auto_scale_ema_alpha_);
  node_.declare_parameter("adjacent_edge_info_auto_scale_min", 1.0);
  node_.get_parameter("adjacent_edge_info_auto_scale_min", adjacent_edge_info_auto_scale_min_);
  node_.declare_parameter("adjacent_edge_info_auto_scale_max", 1.0e6);
  node_.get_parameter("adjacent_edge_info_auto_scale_max", adjacent_edge_info_auto_scale_max_);
  node_.declare_parameter("adjacent_edge_info_auto_scale_split_trans_rot", false);
  node_.get_parameter(
    "adjacent_edge_info_auto_scale_split_trans_rot",
    adjacent_edge_info_auto_scale_split_trans_rot_);
  node_.declare_parameter("adjacent_edge_info_weight_trans", -1.0);
  node_.get_parameter("adjacent_edge_info_weight_trans", adjacent_edge_info_weight_trans_);
  node_.declare_parameter("adjacent_edge_info_weight_rot", -1.0);
  node_.get_parameter("adjacent_edge_info_weight_rot", adjacent_edge_info_weight_rot_);
  node_.declare_parameter("adjacent_edge_info_auto_scale_target_nis_trans", 3.0);
  node_.get_parameter(
    "adjacent_edge_info_auto_scale_target_nis_trans",
    adjacent_edge_info_auto_scale_target_nis_trans_);
  node_.declare_parameter("adjacent_edge_info_auto_scale_target_nis_rot", 3.0);
  node_.get_parameter(
    "adjacent_edge_info_auto_scale_target_nis_rot",
    adjacent_edge_info_auto_scale_target_nis_rot_);
  // Trans/rot weights default to the unified weight when negative (i.e., user
  // has not provided an explicit per-block override). This keeps the split
  // mode safe to enable on existing YAMLs that only set
  // adjacent_edge_info_weight.
  if (adjacent_edge_info_weight_trans_ <= 0.0) {
    adjacent_edge_info_weight_trans_ = adjacent_edge_info_weight_;
  }
  if (adjacent_edge_info_weight_rot_ <= 0.0) {
    adjacent_edge_info_weight_rot_ = adjacent_edge_info_weight_;
  }
  node_.declare_parameter("loop_edge_info_weight", 100.0);
  node_.get_parameter("loop_edge_info_weight", loop_edge_info_weight_);
  node_.declare_parameter("loop_edge_robust_kernel_delta", 1.0);
  node_.get_parameter("loop_edge_robust_kernel_delta", loop_edge_robust_kernel_delta_);
  node_.declare_parameter("loop_edge_robust_kernel_type", std::string("huber"));
  node_.get_parameter("loop_edge_robust_kernel_type", loop_edge_robust_kernel_type_);
  node_.declare_parameter("use_scan_context", false);
  node_.get_parameter("use_scan_context", use_scan_context_);
  node_.declare_parameter("use_bev_descriptor", false);
  node_.get_parameter("use_bev_descriptor", use_bev_descriptor_);
  node_.declare_parameter("use_solid_descriptor", false);
  node_.get_parameter("use_solid_descriptor", use_solid_descriptor_);
  node_.declare_parameter("use_triangle_descriptor", false);
  node_.get_parameter("use_triangle_descriptor", use_triangle_descriptor_);
  node_.declare_parameter("triangle_descriptor_grid_size_m", 60.0);
  node_.get_parameter("triangle_descriptor_grid_size_m", triangle_descriptor_grid_size_m_);
  node_.declare_parameter("triangle_descriptor_grid_cells", 100);
  node_.get_parameter("triangle_descriptor_grid_cells", triangle_descriptor_grid_cells_);
  node_.declare_parameter("triangle_descriptor_max_keypoints", 40);
  node_.get_parameter("triangle_descriptor_max_keypoints", triangle_descriptor_max_keypoints_);
  node_.declare_parameter("triangle_descriptor_min_salience_m", 0.8);
  node_.get_parameter("triangle_descriptor_min_salience_m", triangle_descriptor_min_salience_m_);
  node_.declare_parameter("triangle_descriptor_min_edge_m", 2.0);
  node_.get_parameter("triangle_descriptor_min_edge_m", triangle_descriptor_min_edge_m_);
  node_.declare_parameter("triangle_descriptor_max_edge_m", 50.0);
  node_.get_parameter("triangle_descriptor_max_edge_m", triangle_descriptor_max_edge_m_);
  node_.declare_parameter("triangle_descriptor_max_triangles", 3000);
  node_.get_parameter("triangle_descriptor_max_triangles", triangle_descriptor_max_triangles_);
  node_.declare_parameter("triangle_descriptor_edge_bin_m", 0.5);
  node_.get_parameter("triangle_descriptor_edge_bin_m", triangle_descriptor_edge_bin_m_);
  node_.declare_parameter("triangle_descriptor_quad_feature_bin_m", 0.0);
  node_.get_parameter(
    "triangle_descriptor_quad_feature_bin_m",
    triangle_descriptor_quad_feature_bin_m_);
  node_.declare_parameter<std::string>(
    "triangle_descriptor_keypoint_mode", triangle_descriptor_keypoint_mode_);
  node_.get_parameter("triangle_descriptor_keypoint_mode", triangle_descriptor_keypoint_mode_);
  node_.declare_parameter("triangle_descriptor_edge_voxel_size_m", 0.4);
  node_.get_parameter(
    "triangle_descriptor_edge_voxel_size_m", triangle_descriptor_edge_voxel_size_m_);
  node_.declare_parameter("triangle_descriptor_edge_neighbor_radius_m", 1.0);
  node_.get_parameter(
    "triangle_descriptor_edge_neighbor_radius_m",
    triangle_descriptor_edge_neighbor_radius_m_);
  node_.declare_parameter("triangle_descriptor_edge_min_neighbors", 6);
  node_.get_parameter(
    "triangle_descriptor_edge_min_neighbors", triangle_descriptor_edge_min_neighbors_);
  node_.declare_parameter("triangle_descriptor_edge_min_edgeness", 0.5);
  node_.get_parameter(
    "triangle_descriptor_edge_min_edgeness", triangle_descriptor_edge_min_edgeness_);
  node_.declare_parameter("triangle_descriptor_edge_nms_radius_m", 2.0);
  node_.get_parameter(
    "triangle_descriptor_edge_nms_radius_m", triangle_descriptor_edge_nms_radius_m_);
  node_.declare_parameter("triangle_descriptor_min_votes", 6);
  node_.get_parameter("triangle_descriptor_min_votes", triangle_descriptor_min_votes_);
  node_.declare_parameter("triangle_descriptor_min_inliers", 4);
  node_.get_parameter("triangle_descriptor_min_inliers", triangle_descriptor_min_inliers_);
  node_.declare_parameter("triangle_descriptor_min_inlier_ratio", 0.0);
  node_.get_parameter(
    "triangle_descriptor_min_inlier_ratio",
    triangle_descriptor_min_inlier_ratio_);
  node_.declare_parameter("triangle_descriptor_max_pairs", 64);
  node_.get_parameter("triangle_descriptor_max_pairs", triangle_descriptor_max_pairs_);
  node_.declare_parameter("triangle_descriptor_min_4th_point_agreements", 0);
  node_.get_parameter(
    "triangle_descriptor_min_4th_point_agreements",
    triangle_descriptor_min_4th_point_agreements_);
  node_.declare_parameter("triangle_descriptor_fourth_point_max_distance_m", 2.0);
  node_.get_parameter(
    "triangle_descriptor_fourth_point_max_distance_m",
    triangle_descriptor_fourth_point_max_distance_m_);
  node_.declare_parameter("triangle_descriptor_refine_se3_with_all_inliers", false);
  node_.get_parameter(
    "triangle_descriptor_refine_se3_with_all_inliers",
    triangle_descriptor_refine_se3_with_all_inliers_);
  node_.declare_parameter("triangle_descriptor_skip_ransac", false);
  node_.get_parameter(
    "triangle_descriptor_skip_ransac",
    triangle_descriptor_skip_ransac_);
  node_.declare_parameter("triangle_descriptor_inlier_translation_m", 2.0);
  node_.get_parameter(
    "triangle_descriptor_inlier_translation_m",
    triangle_descriptor_inlier_translation_m_);
  node_.declare_parameter("triangle_descriptor_inlier_rotation_deg", 5.0);
  node_.get_parameter(
    "triangle_descriptor_inlier_rotation_deg",
    triangle_descriptor_inlier_rotation_deg_);
  node_.declare_parameter("triangle_descriptor_exclude_recent", 4);
  node_.get_parameter("triangle_descriptor_exclude_recent", triangle_descriptor_exclude_recent_);
  node_.declare_parameter("triangle_verify_with_bev", false);
  node_.get_parameter("triangle_verify_with_bev", triangle_verify_with_bev_);
  node_.declare_parameter("triangle_verify_bev_max_distance", 0.30);
  node_.get_parameter("triangle_verify_bev_max_distance", triangle_verify_bev_max_distance_);
  node_.declare_parameter("use_pcd_cache", false);
  node_.get_parameter("use_pcd_cache", use_pcd_cache_);
  node_.declare_parameter("pcd_cache_dir", std::string("/tmp/graph_slam_pcd_cache"));
  node_.get_parameter("pcd_cache_dir", pcd_cache_dir_);
  if (use_pcd_cache_) {
    std::filesystem::create_directories(pcd_cache_dir_);
    std::cout << "pcd_cache_dir:" << pcd_cache_dir_ << std::endl;
  }
  node_.declare_parameter("scan_context_threshold", 0.3);
  node_.get_parameter("scan_context_threshold", scan_context_threshold_);
  node_.declare_parameter("scan_context_query_stride", 1);
  node_.get_parameter("scan_context_query_stride", scan_context_query_stride_);
  node_.declare_parameter("scan_context_exclude_recent", ScanContext::EXCLUDE_RECENT);
  node_.get_parameter("scan_context_exclude_recent", scan_context_exclude_recent_);
  node_.declare_parameter("bev_descriptor_threshold", 0.20);
  node_.get_parameter("bev_descriptor_threshold", bev_descriptor_threshold_);
  node_.declare_parameter("bev_descriptor_grid_size_m", 80.0);
  node_.get_parameter("bev_descriptor_grid_size_m", bev_descriptor_grid_size_m_);
  node_.declare_parameter("bev_descriptor_grid_cells", 40);
  node_.get_parameter("bev_descriptor_grid_cells", bev_descriptor_grid_cells_);
  node_.declare_parameter("bev_descriptor_yaw_bins", 24);
  node_.get_parameter("bev_descriptor_yaw_bins", bev_descriptor_yaw_bins_);
  node_.declare_parameter("bev_descriptor_sequence_window", 0);
  node_.get_parameter("bev_descriptor_sequence_window", bev_descriptor_sequence_window_);
  node_.declare_parameter("bev_descriptor_sequence_threshold", -1.0);
  node_.get_parameter("bev_descriptor_sequence_threshold", bev_descriptor_sequence_threshold_);
  node_.declare_parameter("bev_descriptor_pose_consistency_threshold_m", -1.0);
  node_.get_parameter(
    "bev_descriptor_pose_consistency_threshold_m",
    bev_descriptor_pose_consistency_threshold_m_);
  node_.declare_parameter("bev_descriptor_max_euclidean_distance_m", -1.0);
  node_.get_parameter(
    "bev_descriptor_max_euclidean_distance_m",
    bev_descriptor_max_euclidean_distance_m_);
  node_.declare_parameter("bev_descriptor_rerank_weight_m", 100.0);
  node_.get_parameter("bev_descriptor_rerank_weight_m", bev_descriptor_rerank_weight_m_);
  node_.declare_parameter("bev_use_mutual_visibility", false);
  node_.get_parameter("bev_use_mutual_visibility", bev_use_mutual_visibility_);
  node_.declare_parameter("bev_mutual_visibility_min_overlap_ratio", 0.05);
  node_.get_parameter(
    "bev_mutual_visibility_min_overlap_ratio",
    bev_mutual_visibility_min_overlap_ratio_);
  node_.declare_parameter("bev_mutual_visibility_occupancy_eps", 0.5);
  node_.get_parameter(
    "bev_mutual_visibility_occupancy_eps",
    bev_mutual_visibility_occupancy_eps_);
  node_.declare_parameter("solid_descriptor_min_similarity", 0.70);
  node_.get_parameter("solid_descriptor_min_similarity", solid_descriptor_min_similarity_);
  node_.declare_parameter("solid_descriptor_sequence_window", 0);
  node_.get_parameter("solid_descriptor_sequence_window", solid_descriptor_sequence_window_);
  node_.declare_parameter("solid_descriptor_sequence_min_similarity", -1.0);
  node_.get_parameter(
    "solid_descriptor_sequence_min_similarity",
    solid_descriptor_sequence_min_similarity_);
  node_.declare_parameter("solid_descriptor_pose_consistency_threshold_m", -1.0);
  node_.get_parameter(
    "solid_descriptor_pose_consistency_threshold_m",
    solid_descriptor_pose_consistency_threshold_m_);
  node_.declare_parameter("solid_descriptor_max_euclidean_distance_m", -1.0);
  node_.get_parameter(
    "solid_descriptor_max_euclidean_distance_m",
    solid_descriptor_max_euclidean_distance_m_);
  node_.declare_parameter("prefer_scan_context_candidates", false);
  node_.get_parameter("prefer_scan_context_candidates", prefer_scan_context_candidates_);
  node_.declare_parameter("use_3d_bbs_for_scan_context", false);
  node_.get_parameter("use_3d_bbs_for_scan_context", use_3d_bbs_for_scan_context_);
  node_.declare_parameter("three_d_bbs_min_level_res", 1.0);
  node_.get_parameter("three_d_bbs_min_level_res", three_d_bbs_min_level_res_);
  node_.declare_parameter("three_d_bbs_max_level", 3);
  node_.get_parameter("three_d_bbs_max_level", three_d_bbs_max_level_);
  node_.declare_parameter("three_d_bbs_score_threshold_percentage", 0.25);
  node_.get_parameter(
    "three_d_bbs_score_threshold_percentage",
    three_d_bbs_score_threshold_percentage_);
  node_.declare_parameter("three_d_bbs_timeout_msec", 50);
  node_.get_parameter("three_d_bbs_timeout_msec", three_d_bbs_timeout_msec_);
  node_.declare_parameter("three_d_bbs_num_threads", 0);
  node_.get_parameter("three_d_bbs_num_threads", three_d_bbs_num_threads_);
  node_.declare_parameter("three_d_bbs_voxel_leaf_size", 1.0);
  node_.get_parameter("three_d_bbs_voxel_leaf_size", three_d_bbs_voxel_leaf_size_);
  node_.declare_parameter("three_d_bbs_source_submap_num", 2);
  node_.get_parameter("three_d_bbs_source_submap_num", three_d_bbs_source_submap_num_);
  node_.declare_parameter("three_d_bbs_target_submap_radius", 1);
  node_.get_parameter("three_d_bbs_target_submap_radius", three_d_bbs_target_submap_radius_);
  node_.declare_parameter("three_d_bbs_translation_search_margin_m", 15.0);
  node_.get_parameter(
    "three_d_bbs_translation_search_margin_m",
    three_d_bbs_translation_search_margin_m_);
  node_.declare_parameter("three_d_bbs_roll_pitch_search_deg", 10.0);
  node_.get_parameter(
    "three_d_bbs_roll_pitch_search_deg",
    three_d_bbs_roll_pitch_search_deg_);
  node_.declare_parameter("three_d_bbs_yaw_search_deg", 180.0);
  node_.get_parameter("three_d_bbs_yaw_search_deg", three_d_bbs_yaw_search_deg_);
  node_.declare_parameter("use_dynamic_object_filter", false);
  node_.get_parameter("use_dynamic_object_filter", use_dynamic_object_filter_);
  node_.declare_parameter("dynamic_object_filter_voxel_size", 0.3);
  node_.get_parameter("dynamic_object_filter_voxel_size", dynamic_object_filter_voxel_size_);
  node_.declare_parameter("dynamic_object_filter_min_observations", 2);
  node_.get_parameter(
    "dynamic_object_filter_min_observations",
    dynamic_object_filter_min_observations_);
  node_.declare_parameter("dynamic_object_filter_temporal_window", 5);
  node_.get_parameter(
    "dynamic_object_filter_temporal_window",
    dynamic_object_filter_temporal_window_);
  node_.declare_parameter("dynamic_object_filter_max_range_from_sensor_m", 30.0);
  node_.get_parameter(
    "dynamic_object_filter_max_range_from_sensor_m",
    dynamic_object_filter_max_range_from_sensor_m_);
  node_.declare_parameter("use_planar_map_filter", false);
  node_.get_parameter("use_planar_map_filter", use_planar_map_filter_);
  node_.declare_parameter("planar_map_filter_voxel_size", 0.1);
  node_.get_parameter("planar_map_filter_voxel_size", planar_map_filter_voxel_size_);
  node_.declare_parameter("planar_map_filter_min_neighbors", 3);
  node_.get_parameter("planar_map_filter_min_neighbors", planar_map_filter_min_neighbors_);
  node_.declare_parameter("planar_map_filter_max_small_eigenvalue_ratio", 0.24);
  node_.get_parameter(
    "planar_map_filter_max_small_eigenvalue_ratio",
    planar_map_filter_max_small_eigenvalue_ratio_);
  node_.declare_parameter("planar_map_filter_min_middle_eigenvalue_ratio", 0.0);
  node_.get_parameter(
    "planar_map_filter_min_middle_eigenvalue_ratio",
    planar_map_filter_min_middle_eigenvalue_ratio_);
  node_.declare_parameter("planar_map_filter_min_retained_ratio", 0.90);
  node_.get_parameter(
    "planar_map_filter_min_retained_ratio", planar_map_filter_min_retained_ratio_);
  node_.declare_parameter("map_save_dir", std::string("."));
  node_.get_parameter("map_save_dir", map_save_dir_);
  // The launch files have always passed this; until v0.6 Phase 1 it was
  // silently ignored and the graph went to the CWD. The default keeps the
  // historical CWD behaviour.
  node_.declare_parameter("save_pose_graph_path", std::string("pose_graph.g2o"));
  node_.get_parameter("save_pose_graph_path", save_pose_graph_path_);
  node_.declare_parameter("map_grid_size_x", 20.0);
  node_.get_parameter("map_grid_size_x", map_grid_size_x_);
  node_.declare_parameter("map_grid_size_y", 20.0);
  node_.get_parameter("map_grid_size_y", map_grid_size_y_);
  node_.declare_parameter("map_leaf_size", 0.2);
  node_.get_parameter("map_leaf_size", map_leaf_size_);
  node_.declare_parameter("use_gnss", false);
  node_.get_parameter("use_gnss", use_gnss_);
  node_.declare_parameter("gnss_topic", std::string("/gnss/fix"));
  node_.get_parameter("gnss_topic", gnss_topic_);
  node_.declare_parameter("gnss_info_weight", 1.0);
  node_.get_parameter("gnss_info_weight", gnss_info_weight_);
  node_.declare_parameter("gnss_use_covariance_weighting", true);
  node_.get_parameter("gnss_use_covariance_weighting", gnss_use_covariance_weighting_);
  node_.declare_parameter("gnss_covariance_min_variance_m2", 0.01);
  node_.get_parameter("gnss_covariance_min_variance_m2", gnss_covariance_min_variance_m2_);
  node_.declare_parameter("gnss_covariance_max_variance_m2", 25.0);
  node_.get_parameter("gnss_covariance_max_variance_m2", gnss_covariance_max_variance_m2_);
  node_.declare_parameter("gnss_rtk_fix_max_horizontal_stddev_m", 0.3);
  node_.get_parameter(
    "gnss_rtk_fix_max_horizontal_stddev_m",
    gnss_rtk_fix_max_horizontal_stddev_m_);
  node_.declare_parameter("gnss_rtk_fix_weight_scale", 3.0);
  node_.get_parameter("gnss_rtk_fix_weight_scale", gnss_rtk_fix_weight_scale_);
  node_.declare_parameter("gnss_non_rtk_weight_scale", 1.0);
  node_.get_parameter("gnss_non_rtk_weight_scale", gnss_non_rtk_weight_scale_);
  node_.declare_parameter("gnss_header_stamp_max_skew_sec", 30.0);
  node_.get_parameter("gnss_header_stamp_max_skew_sec", gnss_header_stamp_max_skew_sec_);
  node_.declare_parameter("gnss_align_yaw", true);
  node_.get_parameter("gnss_align_yaw", gnss_align_yaw_);
  node_.declare_parameter("gnss_yaw_alignment_min_anchors", 10);
  node_.get_parameter("gnss_yaw_alignment_min_anchors", gnss_yaw_alignment_min_anchors_);
  node_.declare_parameter("gnss_yaw_alignment_min_baseline_m", 5.0);
  node_.get_parameter("gnss_yaw_alignment_min_baseline_m", gnss_yaw_alignment_min_baseline_m_);
  node_.declare_parameter("gnss_origin_min_samples", 3);
  node_.get_parameter("gnss_origin_min_samples", gnss_origin_min_samples_);
  node_.declare_parameter("gnss_origin_consistency_threshold_m", 20.0);
  node_.get_parameter(
    "gnss_origin_consistency_threshold_m",
    gnss_origin_consistency_threshold_m_);
  node_.declare_parameter("use_imu_preintegration", false);
  node_.get_parameter("use_imu_preintegration", use_imu_preintegration_);
  node_.declare_parameter("imu_rotation_info_roll_pitch", 100.0);
  node_.get_parameter("imu_rotation_info_roll_pitch", imu_rotation_info_roll_pitch_);
  node_.declare_parameter("imu_rotation_info_yaw", 10.0);
  node_.get_parameter("imu_rotation_info_yaw", imu_rotation_info_yaw_);

  if (gnss_origin_min_samples_ < 1) {
    RCLCPP_WARN(
      node_.get_logger(),
      "gnss_origin_min_samples must be >= 1, clamping %d to 1",
      gnss_origin_min_samples_);
    gnss_origin_min_samples_ = 1;
  }
  if (gnss_origin_consistency_threshold_m_ <= 0.0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "gnss_origin_consistency_threshold_m must be positive, resetting %.3f to 20.0",
      gnss_origin_consistency_threshold_m_);
    gnss_origin_consistency_threshold_m_ = 20.0;
  }
  gnss_origin_accumulator_.configure(
    gnss_origin_min_samples_, gnss_origin_consistency_threshold_m_);
  if (gnss_covariance_min_variance_m2_ <= 0.0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "gnss_covariance_min_variance_m2 must be positive, resetting %.6f to 0.01",
      gnss_covariance_min_variance_m2_);
    gnss_covariance_min_variance_m2_ = 0.01;
  }
  if (gnss_covariance_max_variance_m2_ < gnss_covariance_min_variance_m2_) {
    RCLCPP_WARN(
      node_.get_logger(),
      "gnss_covariance_max_variance_m2 must be >= min variance, resetting %.6f to %.6f",
      gnss_covariance_max_variance_m2_, gnss_covariance_min_variance_m2_);
    gnss_covariance_max_variance_m2_ = gnss_covariance_min_variance_m2_;
  }
  if (gnss_rtk_fix_max_horizontal_stddev_m_ <= 0.0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "gnss_rtk_fix_max_horizontal_stddev_m must be positive, resetting %.3f to 0.3",
      gnss_rtk_fix_max_horizontal_stddev_m_);
    gnss_rtk_fix_max_horizontal_stddev_m_ = 0.3;
  }
  if (gnss_rtk_fix_weight_scale_ <= 0.0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "gnss_rtk_fix_weight_scale must be positive, resetting %.3f to 3.0",
      gnss_rtk_fix_weight_scale_);
    gnss_rtk_fix_weight_scale_ = 3.0;
  }
  if (gnss_non_rtk_weight_scale_ <= 0.0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "gnss_non_rtk_weight_scale must be positive, resetting %.3f to 1.0",
      gnss_non_rtk_weight_scale_);
    gnss_non_rtk_weight_scale_ = 1.0;
  }
  if (gnss_header_stamp_max_skew_sec_ <= 0.0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "gnss_header_stamp_max_skew_sec must be positive, resetting %.3f to 30.0",
      gnss_header_stamp_max_skew_sec_);
    gnss_header_stamp_max_skew_sec_ = 30.0;
  }
  if (search_submap_num_ < 1) {
    RCLCPP_WARN(
      node_.get_logger(),
      "search_submap_num must be >= 1, clamping %d to 1",
      search_submap_num_);
    search_submap_num_ = 1;
  }
  if (loop_search_query_stride_ < 1) {
    RCLCPP_WARN(
      node_.get_logger(),
      "loop_search_query_stride must be >= 1, clamping %d to 1",
      loop_search_query_stride_);
    loop_search_query_stride_ = 1;
  }
  if (max_loop_candidate_count_ < 1) {
    RCLCPP_WARN(
      node_.get_logger(),
      "max_loop_candidate_count must be >= 1, clamping %d to 1",
      max_loop_candidate_count_);
    max_loop_candidate_count_ = 1;
  }
  if (loop_edge_dedup_index_window_ < 0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "loop_edge_dedup_index_window must be >= 0, clamping %d to 0",
      loop_edge_dedup_index_window_);
    loop_edge_dedup_index_window_ = 0;
  }
  graph_state_.configureLoopEdgeDedupWindow(loop_edge_dedup_index_window_);
  if (loop_max_translation_delta_ <= 0.0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "loop_max_translation_delta must be positive, resetting %.3f to 15.0",
      loop_max_translation_delta_);
    loop_max_translation_delta_ = 15.0;
  }
  if (loop_max_rotation_delta_deg_ <= 0.0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "loop_max_rotation_delta_deg must be positive, resetting %.3f to 45.0",
      loop_max_rotation_delta_deg_);
    loop_max_rotation_delta_deg_ = 45.0;
  }
  if (loop_min_overlap_ratio_ < 0.0 || loop_min_overlap_ratio_ > 1.0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "loop_min_overlap_ratio must be in [0, 1], resetting %.3f to 0 (disabled)",
      loop_min_overlap_ratio_);
    loop_min_overlap_ratio_ = 0.0;
  }
  if (
    loop_min_overlap_ratio_large_correction_ < 0.0 ||
    loop_min_overlap_ratio_large_correction_ > 1.0)
  {
    RCLCPP_WARN(
      node_.get_logger(),
      "loop_min_overlap_ratio_large_correction must be in [0, 1], "
      "resetting %.3f to 0 (disabled)",
      loop_min_overlap_ratio_large_correction_);
    loop_min_overlap_ratio_large_correction_ = 0.0;
  }
  if (loop_overlap_large_correction_translation_m_ < 0.0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "loop_overlap_large_correction_translation_m must be non-negative, "
      "resetting %.3f to 0 (disabled)",
      loop_overlap_large_correction_translation_m_);
    loop_overlap_large_correction_translation_m_ = 0.0;
  }
  if (loop_overlap_max_distance_m_ <= 0.0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "loop_overlap_max_distance_m must be positive, resetting %.3f to 0.5",
      loop_overlap_max_distance_m_);
    loop_overlap_max_distance_m_ = 0.5;
  }
  // Descriptor overrides: -1.0 = disabled (fall back to generic cap).
  // Any other non-positive value is treated as invalid and clamped to -1.0
  // so that operators see clear feedback instead of silently disabling the
  // generic cap for descriptor sources.
  if (loop_max_translation_delta_descriptor_ <= 0.0 &&
    loop_max_translation_delta_descriptor_ != -1.0)
  {
    RCLCPP_WARN(
      node_.get_logger(),
      "loop_max_translation_delta_descriptor must be > 0 (override) or -1 "
      "(disabled); resetting %.3f to -1",
      loop_max_translation_delta_descriptor_);
    loop_max_translation_delta_descriptor_ = -1.0;
  }
  if (loop_max_rotation_delta_deg_descriptor_ <= 0.0 &&
    loop_max_rotation_delta_deg_descriptor_ != -1.0)
  {
    RCLCPP_WARN(
      node_.get_logger(),
      "loop_max_rotation_delta_deg_descriptor must be > 0 (override) or -1 "
      "(disabled); resetting %.3f to -1",
      loop_max_rotation_delta_deg_descriptor_);
    loop_max_rotation_delta_deg_descriptor_ = -1.0;
  }
  if (num_adjacent_pose_cnstraints_ < 1) {
    RCLCPP_WARN(
      node_.get_logger(),
      "num_adjacent_pose_cnstraints must be >= 1, clamping %d to 1",
      num_adjacent_pose_cnstraints_);
    num_adjacent_pose_cnstraints_ = 1;
  }
  if (adjacent_edge_info_weight_ <= 0.0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "adjacent_edge_info_weight must be positive, resetting %.3f to 1000.0",
      adjacent_edge_info_weight_);
    adjacent_edge_info_weight_ = 1000.0;
  }
  if (adjacent_edge_info_weight_trans_ <= 0.0) {
    adjacent_edge_info_weight_trans_ = adjacent_edge_info_weight_;
  }
  if (adjacent_edge_info_weight_rot_ <= 0.0) {
    adjacent_edge_info_weight_rot_ = adjacent_edge_info_weight_;
  }
  if (adjacent_edge_info_auto_scale_target_nis_trans_ <= 0.0) {
    adjacent_edge_info_auto_scale_target_nis_trans_ = 3.0;
  }
  if (adjacent_edge_info_auto_scale_target_nis_rot_ <= 0.0) {
    adjacent_edge_info_auto_scale_target_nis_rot_ = 3.0;
  }
  if (loop_edge_info_weight_ <= 0.0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "loop_edge_info_weight must be positive, resetting %.3f to 100.0",
      loop_edge_info_weight_);
    loop_edge_info_weight_ = 100.0;
  }
  if (loop_edge_robust_kernel_delta_ <= 0.0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "loop_edge_robust_kernel_delta must be positive, resetting %.3f to 1.0",
      loop_edge_robust_kernel_delta_);
    loop_edge_robust_kernel_delta_ = 1.0;
  }
  if (bev_descriptor_threshold_ <= 0.0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "bev_descriptor_threshold must be positive, resetting %.3f to 0.20",
      bev_descriptor_threshold_);
    bev_descriptor_threshold_ = 0.20;
  }
  if (bev_descriptor_grid_size_m_ <= 0.0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "bev_descriptor_grid_size_m must be positive, resetting %.3f to 80.0",
      bev_descriptor_grid_size_m_);
    bev_descriptor_grid_size_m_ = 80.0;
  }
  if (bev_descriptor_grid_cells_ < 8) {
    RCLCPP_WARN(
      node_.get_logger(),
      "bev_descriptor_grid_cells must be >= 8, clamping %d to 8",
      bev_descriptor_grid_cells_);
    bev_descriptor_grid_cells_ = 8;
  }
  if (bev_descriptor_yaw_bins_ < 1) {
    RCLCPP_WARN(
      node_.get_logger(),
      "bev_descriptor_yaw_bins must be >= 1, clamping %d to 1",
      bev_descriptor_yaw_bins_);
    bev_descriptor_yaw_bins_ = 1;
  }
  if (bev_descriptor_sequence_window_ < 0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "bev_descriptor_sequence_window must be >= 0, clamping %d to 0",
      bev_descriptor_sequence_window_);
    bev_descriptor_sequence_window_ = 0;
  }
  if (bev_descriptor_sequence_threshold_ <= 0.0) {
    bev_descriptor_sequence_threshold_ = bev_descriptor_threshold_;
  }
  if (bev_descriptor_rerank_weight_m_ < 0.0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "bev_descriptor_rerank_weight_m must be >= 0.0, clamping %.3f to 0.0",
      bev_descriptor_rerank_weight_m_);
    bev_descriptor_rerank_weight_m_ = 0.0;
  }
  if (bev_descriptor_pose_consistency_threshold_m_ == 0.0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "bev_descriptor_pose_consistency_threshold_m must be negative (disabled) or positive, "
      "resetting 0.0 to disabled");
    bev_descriptor_pose_consistency_threshold_m_ = -1.0;
  }
  if (triangle_descriptor_keypoint_mode_ != "bev_max_height" &&
    triangle_descriptor_keypoint_mode_ != "edge_3d")
  {
    RCLCPP_WARN(
      node_.get_logger(),
      "triangle_descriptor_keypoint_mode must be 'bev_max_height' or 'edge_3d', "
      "got '%s'; falling back to 'bev_max_height'",
      triangle_descriptor_keypoint_mode_.c_str());
    triangle_descriptor_keypoint_mode_ = "bev_max_height";
  }
  if (triangle_descriptor_edge_voxel_size_m_ < 0.0) {
    triangle_descriptor_edge_voxel_size_m_ = 0.0;
  }
  if (triangle_descriptor_edge_neighbor_radius_m_ <= 0.05) {
    triangle_descriptor_edge_neighbor_radius_m_ = 0.05;
  }
  if (triangle_descriptor_edge_min_neighbors_ < 4) {
    triangle_descriptor_edge_min_neighbors_ = 4;
  }
  if (triangle_descriptor_edge_min_edgeness_ < 0.0) {
    triangle_descriptor_edge_min_edgeness_ = 0.0;
  } else if (triangle_descriptor_edge_min_edgeness_ > 1.0) {
    triangle_descriptor_edge_min_edgeness_ = 1.0;
  }
  if (triangle_descriptor_edge_nms_radius_m_ < 0.0) {
    triangle_descriptor_edge_nms_radius_m_ = 0.0;
  }
  if (triangle_descriptor_grid_size_m_ <= 0.0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "triangle_descriptor_grid_size_m must be positive, resetting %.3f to 60.0",
      triangle_descriptor_grid_size_m_);
    triangle_descriptor_grid_size_m_ = 60.0;
  }
  if (triangle_descriptor_grid_cells_ < 8) {
    RCLCPP_WARN(
      node_.get_logger(),
      "triangle_descriptor_grid_cells must be >= 8, clamping %d to 8",
      triangle_descriptor_grid_cells_);
    triangle_descriptor_grid_cells_ = 8;
  }
  if (triangle_descriptor_max_keypoints_ < 4) {
    RCLCPP_WARN(
      node_.get_logger(),
      "triangle_descriptor_max_keypoints must be >= 4, clamping %d to 4",
      triangle_descriptor_max_keypoints_);
    triangle_descriptor_max_keypoints_ = 4;
  }
  if (triangle_descriptor_min_edge_m_ <= 0.0) {
    triangle_descriptor_min_edge_m_ = 2.0;
  }
  if (triangle_descriptor_max_edge_m_ <= triangle_descriptor_min_edge_m_) {
    RCLCPP_WARN(
      node_.get_logger(),
      "triangle_descriptor_max_edge_m (%.3f) must exceed min_edge_m (%.3f); using min*5",
      triangle_descriptor_max_edge_m_, triangle_descriptor_min_edge_m_);
    triangle_descriptor_max_edge_m_ = triangle_descriptor_min_edge_m_ * 5.0;
  }
  if (triangle_descriptor_max_triangles_ < 100) {
    triangle_descriptor_max_triangles_ = 100;
  }
  if (triangle_descriptor_edge_bin_m_ <= 0.0) {
    triangle_descriptor_edge_bin_m_ = 1.0;
  }
  if (triangle_descriptor_quad_feature_bin_m_ < 0.0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "triangle_descriptor_quad_feature_bin_m must be >= 0 (0 = disabled); "
      "resetting %.3f to 0",
      triangle_descriptor_quad_feature_bin_m_);
    triangle_descriptor_quad_feature_bin_m_ = 0.0;
  }
  if (triangle_descriptor_min_votes_ < 1) {
    triangle_descriptor_min_votes_ = 1;
  }
  if (triangle_descriptor_min_inliers_ < 1) {
    triangle_descriptor_min_inliers_ = 1;
  }
  if (triangle_descriptor_min_inlier_ratio_ < 0.0) {
    triangle_descriptor_min_inlier_ratio_ = 0.0;
  } else if (triangle_descriptor_min_inlier_ratio_ > 1.0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "triangle_descriptor_min_inlier_ratio must be in [0, 1]; clamping %.3f to 1.0",
      triangle_descriptor_min_inlier_ratio_);
    triangle_descriptor_min_inlier_ratio_ = 1.0;
  }
  if (triangle_descriptor_max_pairs_ < 3) {
    RCLCPP_WARN(
      node_.get_logger(),
      "triangle_descriptor_max_pairs must be >= 3; clamping %d to 3",
      triangle_descriptor_max_pairs_);
    triangle_descriptor_max_pairs_ = 3;
  }
  if (triangle_descriptor_min_4th_point_agreements_ < 0) {
    triangle_descriptor_min_4th_point_agreements_ = 0;
  }
  if (triangle_descriptor_fourth_point_max_distance_m_ <= 0.0) {
    triangle_descriptor_fourth_point_max_distance_m_ = 2.0;
  }
  if (triangle_descriptor_inlier_translation_m_ <= 0.0) {
    triangle_descriptor_inlier_translation_m_ = 2.0;
  }
  if (triangle_descriptor_inlier_rotation_deg_ <= 0.0) {
    triangle_descriptor_inlier_rotation_deg_ = 5.0;
  }
  if (triangle_descriptor_exclude_recent_ < 0) {
    triangle_descriptor_exclude_recent_ = 0;
  }
  if (triangle_verify_bev_max_distance_ <= 0.0) {
    triangle_verify_bev_max_distance_ = 0.30;
  }
  if (
    solid_descriptor_min_similarity_ <= -1.0 ||
    solid_descriptor_min_similarity_ > 1.0)
  {
    RCLCPP_WARN(
      node_.get_logger(),
      "solid_descriptor_min_similarity must be in (-1, 1], resetting %.3f to 0.70",
      solid_descriptor_min_similarity_);
    solid_descriptor_min_similarity_ = 0.70;
  }
  if (solid_descriptor_sequence_window_ < 0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "solid_descriptor_sequence_window must be >= 0, clamping %d to 0",
      solid_descriptor_sequence_window_);
    solid_descriptor_sequence_window_ = 0;
  }
  if (
    solid_descriptor_sequence_min_similarity_ <= -1.0 ||
    solid_descriptor_sequence_min_similarity_ > 1.0)
  {
    solid_descriptor_sequence_min_similarity_ = solid_descriptor_min_similarity_;
  }
  if (solid_descriptor_pose_consistency_threshold_m_ == 0.0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "solid_descriptor_pose_consistency_threshold_m must be negative (disabled) or positive, "
      "resetting 0.0 to disabled");
    solid_descriptor_pose_consistency_threshold_m_ = -1.0;
  }
  if (three_d_bbs_min_level_res_ <= 0.0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "three_d_bbs_min_level_res must be positive, resetting %.3f to 1.0",
      three_d_bbs_min_level_res_);
    three_d_bbs_min_level_res_ = 1.0;
  }
  if (three_d_bbs_max_level_ < 1) {
    RCLCPP_WARN(
      node_.get_logger(),
      "three_d_bbs_max_level must be >= 1, clamping %d to 1",
      three_d_bbs_max_level_);
    three_d_bbs_max_level_ = 1;
  }
  if (three_d_bbs_score_threshold_percentage_ <= 0.0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "three_d_bbs_score_threshold_percentage must be positive, resetting %.3f to 0.25",
      three_d_bbs_score_threshold_percentage_);
    three_d_bbs_score_threshold_percentage_ = 0.25;
  }
  if (three_d_bbs_voxel_leaf_size_ <= 0.0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "three_d_bbs_voxel_leaf_size must be positive, resetting %.3f to 1.0",
      three_d_bbs_voxel_leaf_size_);
    three_d_bbs_voxel_leaf_size_ = 1.0;
  }
  if (three_d_bbs_source_submap_num_ < 1) {
    RCLCPP_WARN(
      node_.get_logger(),
      "three_d_bbs_source_submap_num must be >= 1, clamping %d to 1",
      three_d_bbs_source_submap_num_);
    three_d_bbs_source_submap_num_ = 1;
  }
  if (three_d_bbs_target_submap_radius_ < 0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "three_d_bbs_target_submap_radius must be >= 0, clamping %d to 0",
      three_d_bbs_target_submap_radius_);
    three_d_bbs_target_submap_radius_ = 0;
  }
  if (three_d_bbs_translation_search_margin_m_ <= 0.0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "three_d_bbs_translation_search_margin_m must be positive, resetting %.3f to 15.0",
      three_d_bbs_translation_search_margin_m_);
    three_d_bbs_translation_search_margin_m_ = 15.0;
  }
  if (three_d_bbs_roll_pitch_search_deg_ <= 0.0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "three_d_bbs_roll_pitch_search_deg must be positive, resetting %.3f to 10.0",
      three_d_bbs_roll_pitch_search_deg_);
    three_d_bbs_roll_pitch_search_deg_ = 10.0;
  }
  if (three_d_bbs_yaw_search_deg_ <= 0.0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "three_d_bbs_yaw_search_deg must be positive, resetting %.3f to 180.0",
      three_d_bbs_yaw_search_deg_);
    three_d_bbs_yaw_search_deg_ = 180.0;
  }
  if (dynamic_object_filter_voxel_size_ <= 0.0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "dynamic_object_filter_voxel_size must be positive, resetting %.3f to 0.3",
      dynamic_object_filter_voxel_size_);
    dynamic_object_filter_voxel_size_ = 0.3;
  }
  if (dynamic_object_filter_min_observations_ < 1) {
    RCLCPP_WARN(
      node_.get_logger(),
      "dynamic_object_filter_min_observations must be >= 1, clamping %d to 1",
      dynamic_object_filter_min_observations_);
    dynamic_object_filter_min_observations_ = 1;
  }
  if (dynamic_object_filter_temporal_window_ < 0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "dynamic_object_filter_temporal_window must be >= 0, clamping %d to 0",
      dynamic_object_filter_temporal_window_);
    dynamic_object_filter_temporal_window_ = 0;
  }
  if (dynamic_object_filter_max_range_from_sensor_m_ <= 0.0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "dynamic_object_filter_max_range_from_sensor_m must be positive, resetting %.3f to 30.0",
      dynamic_object_filter_max_range_from_sensor_m_);
    dynamic_object_filter_max_range_from_sensor_m_ = 30.0;
  }
  if (planar_map_filter_voxel_size_ <= 0.0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "planar_map_filter_voxel_size must be positive, resetting %.3f to 0.1",
      planar_map_filter_voxel_size_);
    planar_map_filter_voxel_size_ = 0.1;
  }
  if (planar_map_filter_min_neighbors_ < 3) {
    RCLCPP_WARN(
      node_.get_logger(),
      "planar_map_filter_min_neighbors must be >= 3, clamping %d to 3",
      planar_map_filter_min_neighbors_);
    planar_map_filter_min_neighbors_ = 3;
  }
  planar_map_filter_max_small_eigenvalue_ratio_ = std::max(
    0.0, std::min(1.0, planar_map_filter_max_small_eigenvalue_ratio_));
  planar_map_filter_min_middle_eigenvalue_ratio_ = std::max(
    0.0, std::min(1.0, planar_map_filter_min_middle_eigenvalue_ratio_));
  planar_map_filter_min_retained_ratio_ = std::max(
    0.0, std::min(1.0, planar_map_filter_min_retained_ratio_));
  std::cout << "registration_method:" << registration_method << std::endl;
  std::cout << "voxel_leaf_size[m]:" << voxel_leaf_size << std::endl;
  std::cout << "ndt_resolution[m]:" << ndt_resolution << std::endl;
  std::cout << "ndt_num_threads:" << ndt_num_threads << std::endl;
  std::cout << "threshold_loop_closure_score:" << threshold_loop_closure_score_ << std::endl;
  std::cout << "scan_context_loop_closure_score_threshold:" <<
    scan_context_loop_closure_score_threshold_ << std::endl;
  std::cout << "distance_loop_closure[m]:" << distance_loop_closure_ << std::endl;
  std::cout << "range_of_searching_loop_closure[m]:" << range_of_searching_loop_closure_ <<
    std::endl;
  std::cout << "search_submap_num:" << search_submap_num_ << std::endl;
  std::cout << "loop_search_query_stride:" << loop_search_query_stride_ << std::endl;
  std::cout << "max_loop_candidate_count:" << max_loop_candidate_count_ << std::endl;
  std::cout << "loop_edge_dedup_index_window:" << loop_edge_dedup_index_window_ << std::endl;
  std::cout << "loop_max_translation_delta[m]:" << loop_max_translation_delta_ << std::endl;
  std::cout << "loop_max_rotation_delta[deg]:" << loop_max_rotation_delta_deg_ << std::endl;
  std::cout << "loop_min_overlap_ratio:" << loop_min_overlap_ratio_ << std::endl;
  std::cout << "loop_min_overlap_ratio_large_correction:" <<
    loop_min_overlap_ratio_large_correction_ << std::endl;
  std::cout << "loop_overlap_large_correction_translation_m[m]:" <<
    loop_overlap_large_correction_translation_m_ << std::endl;
  std::cout << "loop_overlap_max_distance_m[m]:" << loop_overlap_max_distance_m_ << std::endl;
  std::cout << "loop_max_translation_delta_descriptor[m]:" <<
    loop_max_translation_delta_descriptor_ << std::endl;
  std::cout << "loop_max_rotation_delta_deg_descriptor[deg]:" <<
    loop_max_rotation_delta_deg_descriptor_ << std::endl;
  std::cout << "num_adjacent_pose_cnstraints:" << num_adjacent_pose_cnstraints_ << std::endl;
  std::cout << "adjacent_edge_info_weight:" << adjacent_edge_info_weight_ << std::endl;
  std::cout << "adjacent_edge_info_auto_scale:" << std::boolalpha
            << adjacent_edge_info_auto_scale_ << std::endl;
  if (adjacent_edge_info_auto_scale_) {
    std::cout << "adjacent_edge_info_auto_scale_split_trans_rot:" << std::boolalpha
              << adjacent_edge_info_auto_scale_split_trans_rot_ << std::endl;
    if (adjacent_edge_info_auto_scale_split_trans_rot_) {
      std::cout << "adjacent_edge_info_weight_trans:"
                << adjacent_edge_info_weight_trans_ << std::endl;
      std::cout << "adjacent_edge_info_weight_rot:"
                << adjacent_edge_info_weight_rot_ << std::endl;
      std::cout << "adjacent_edge_info_auto_scale_target_nis_trans:"
                << adjacent_edge_info_auto_scale_target_nis_trans_ << std::endl;
      std::cout << "adjacent_edge_info_auto_scale_target_nis_rot:"
                << adjacent_edge_info_auto_scale_target_nis_rot_ << std::endl;
    } else {
      std::cout << "adjacent_edge_info_auto_scale_target_nis:"
                << adjacent_edge_info_auto_scale_target_nis_ << std::endl;
    }
  }
  std::cout << "loop_edge_info_weight:" << loop_edge_info_weight_ << std::endl;
  std::cout << "loop_edge_robust_kernel_delta:" << loop_edge_robust_kernel_delta_ << std::endl;
  std::cout << "loop_edge_robust_kernel_type:"
            << graphslam::robust::loopEdgeKernelTypeName(
    graphslam::robust::parseLoopEdgeKernelType(loop_edge_robust_kernel_type_))
            << std::endl;
  std::cout << "use_save_map_in_loop:" << std::boolalpha << use_save_map_in_loop_ << std::endl;
  std::cout << "debug_flag:" << std::boolalpha << debug_flag_ << std::endl;
  std::cout << "use_scan_context:" << std::boolalpha << use_scan_context_ << std::endl;
  if (use_scan_context_) {
    std::cout << "scan_context_threshold:" << scan_context_threshold_ << std::endl;
    std::cout << "scan_context_query_stride:" <<
      std::max(1, scan_context_query_stride_) << std::endl;
    std::cout << "scan_context_exclude_recent:" <<
      std::max(1, scan_context_exclude_recent_) << std::endl;
    std::cout << "prefer_scan_context_candidates:" << std::boolalpha <<
      prefer_scan_context_candidates_ << std::endl;
    std::cout << "use_3d_bbs_for_scan_context:" << std::boolalpha <<
      use_3d_bbs_for_scan_context_ << std::endl;
    if (use_3d_bbs_for_scan_context_) {
      std::cout << "three_d_bbs_min_level_res:" << three_d_bbs_min_level_res_ << std::endl;
      std::cout << "three_d_bbs_max_level:" << three_d_bbs_max_level_ << std::endl;
      std::cout << "three_d_bbs_score_threshold_percentage:" <<
        three_d_bbs_score_threshold_percentage_ << std::endl;
      std::cout << "three_d_bbs_timeout_msec:" << three_d_bbs_timeout_msec_ << std::endl;
      std::cout << "three_d_bbs_num_threads:" << three_d_bbs_num_threads_ << std::endl;
      std::cout << "three_d_bbs_voxel_leaf_size:" << three_d_bbs_voxel_leaf_size_ << std::endl;
      std::cout << "three_d_bbs_source_submap_num:" << three_d_bbs_source_submap_num_ <<
        std::endl;
      std::cout << "three_d_bbs_target_submap_radius:" << three_d_bbs_target_submap_radius_ <<
        std::endl;
      std::cout << "three_d_bbs_translation_search_margin_m:" <<
        three_d_bbs_translation_search_margin_m_ << std::endl;
      std::cout << "three_d_bbs_roll_pitch_search_deg:" <<
        three_d_bbs_roll_pitch_search_deg_ << std::endl;
      std::cout << "three_d_bbs_yaw_search_deg:" << three_d_bbs_yaw_search_deg_ << std::endl;
    }
  }
  std::cout << "use_bev_descriptor:" << std::boolalpha << use_bev_descriptor_ << std::endl;
  if (use_bev_descriptor_) {
    std::cout << "bev_descriptor_threshold:" << bev_descriptor_threshold_ << std::endl;
    std::cout << "bev_descriptor_grid_size_m:" << bev_descriptor_grid_size_m_ << std::endl;
    std::cout << "bev_descriptor_grid_cells:" << bev_descriptor_grid_cells_ << std::endl;
    std::cout << "bev_descriptor_yaw_bins:" << bev_descriptor_yaw_bins_ << std::endl;
    std::cout << "bev_descriptor_sequence_window:" << bev_descriptor_sequence_window_ <<
      std::endl;
    std::cout << "bev_descriptor_sequence_threshold:" << bev_descriptor_sequence_threshold_ <<
      std::endl;
    std::cout << "bev_descriptor_pose_consistency_threshold_m:" <<
      bev_descriptor_pose_consistency_threshold_m_ << std::endl;
    std::cout << "bev_descriptor_max_euclidean_distance_m:" <<
      bev_descriptor_max_euclidean_distance_m_ << std::endl;
    std::cout << "bev_descriptor_rerank_weight_m:" << bev_descriptor_rerank_weight_m_ <<
      std::endl;
    std::cout << "bev_use_mutual_visibility:" << std::boolalpha <<
      bev_use_mutual_visibility_ << std::endl;
    if (bev_use_mutual_visibility_) {
      std::cout << "bev_mutual_visibility_min_overlap_ratio:" <<
        bev_mutual_visibility_min_overlap_ratio_ << std::endl;
      std::cout << "bev_mutual_visibility_occupancy_eps:" <<
        bev_mutual_visibility_occupancy_eps_ << std::endl;
    }
  }
  std::cout << "use_solid_descriptor:" << std::boolalpha << use_solid_descriptor_ << std::endl;
  if (use_solid_descriptor_) {
    std::cout << "solid_descriptor_min_similarity:" << solid_descriptor_min_similarity_ <<
      std::endl;
    std::cout << "solid_descriptor_sequence_window:" << solid_descriptor_sequence_window_ <<
      std::endl;
    std::cout << "solid_descriptor_sequence_min_similarity:" <<
      solid_descriptor_sequence_min_similarity_ << std::endl;
    std::cout << "solid_descriptor_pose_consistency_threshold_m:" <<
      solid_descriptor_pose_consistency_threshold_m_ << std::endl;
    std::cout << "solid_descriptor_max_euclidean_distance_m:" <<
      solid_descriptor_max_euclidean_distance_m_ << std::endl;
  }
  std::cout << "use_triangle_descriptor:" << std::boolalpha <<
    use_triangle_descriptor_ << std::endl;
  if (use_triangle_descriptor_) {
    std::cout << "triangle_descriptor_keypoint_mode:" <<
      triangle_descriptor_keypoint_mode_ << std::endl;
    std::cout << "triangle_descriptor_grid_size_m:" <<
      triangle_descriptor_grid_size_m_ << std::endl;
    std::cout << "triangle_descriptor_grid_cells:" <<
      triangle_descriptor_grid_cells_ << std::endl;
    std::cout << "triangle_descriptor_max_keypoints:" <<
      triangle_descriptor_max_keypoints_ << std::endl;
    std::cout << "triangle_descriptor_min_salience_m:" <<
      triangle_descriptor_min_salience_m_ << std::endl;
    std::cout << "triangle_descriptor_edge_voxel_size_m:" <<
      triangle_descriptor_edge_voxel_size_m_ << std::endl;
    std::cout << "triangle_descriptor_edge_neighbor_radius_m:" <<
      triangle_descriptor_edge_neighbor_radius_m_ << std::endl;
    std::cout << "triangle_descriptor_edge_min_neighbors:" <<
      triangle_descriptor_edge_min_neighbors_ << std::endl;
    std::cout << "triangle_descriptor_edge_min_edgeness:" <<
      triangle_descriptor_edge_min_edgeness_ << std::endl;
    std::cout << "triangle_descriptor_edge_nms_radius_m:" <<
      triangle_descriptor_edge_nms_radius_m_ << std::endl;
    std::cout << "triangle_descriptor_min_edge_m:" <<
      triangle_descriptor_min_edge_m_ << std::endl;
    std::cout << "triangle_descriptor_max_edge_m:" <<
      triangle_descriptor_max_edge_m_ << std::endl;
    std::cout << "triangle_descriptor_max_triangles:" <<
      triangle_descriptor_max_triangles_ << std::endl;
    std::cout << "triangle_descriptor_edge_bin_m:" <<
      triangle_descriptor_edge_bin_m_ << std::endl;
    std::cout << "triangle_descriptor_quad_feature_bin_m:" <<
      triangle_descriptor_quad_feature_bin_m_ << std::endl;
    std::cout << "triangle_descriptor_min_votes:" <<
      triangle_descriptor_min_votes_ << std::endl;
    std::cout << "triangle_descriptor_min_inliers:" <<
      triangle_descriptor_min_inliers_ << std::endl;
    std::cout << "triangle_descriptor_min_inlier_ratio:" <<
      triangle_descriptor_min_inlier_ratio_ << std::endl;
    std::cout << "triangle_descriptor_max_pairs:" <<
      triangle_descriptor_max_pairs_ << std::endl;
    std::cout << "triangle_descriptor_min_4th_point_agreements:" <<
      triangle_descriptor_min_4th_point_agreements_ << std::endl;
    std::cout << "triangle_descriptor_fourth_point_max_distance_m:" <<
      triangle_descriptor_fourth_point_max_distance_m_ << std::endl;
    std::cout << "triangle_descriptor_refine_se3_with_all_inliers:" <<
      std::boolalpha << triangle_descriptor_refine_se3_with_all_inliers_ << std::endl;
    std::cout << "triangle_descriptor_skip_ransac:" <<
      std::boolalpha << triangle_descriptor_skip_ransac_ << std::endl;
    std::cout << "triangle_descriptor_inlier_translation_m:" <<
      triangle_descriptor_inlier_translation_m_ << std::endl;
    std::cout << "triangle_descriptor_inlier_rotation_deg:" <<
      triangle_descriptor_inlier_rotation_deg_ << std::endl;
    std::cout << "triangle_descriptor_exclude_recent:" <<
      triangle_descriptor_exclude_recent_ << std::endl;
  }
  std::cout << "use_dynamic_object_filter:" << std::boolalpha << use_dynamic_object_filter_ <<
    std::endl;
  if (use_dynamic_object_filter_) {
    std::cout << "dynamic_object_filter_voxel_size:" << dynamic_object_filter_voxel_size_ <<
      std::endl;
    std::cout << "dynamic_object_filter_min_observations:" <<
      dynamic_object_filter_min_observations_ << std::endl;
    std::cout << "dynamic_object_filter_temporal_window:" <<
      dynamic_object_filter_temporal_window_ << std::endl;
    std::cout << "dynamic_object_filter_max_range_from_sensor_m:" <<
      dynamic_object_filter_max_range_from_sensor_m_ << std::endl;
  }
  std::cout << "use_planar_map_filter:" << std::boolalpha << use_planar_map_filter_ << std::endl;
  if (use_planar_map_filter_) {
    std::cout << "planar_map_filter_voxel_size:" << planar_map_filter_voxel_size_ << std::endl;
    std::cout << "planar_map_filter_min_neighbors:" << planar_map_filter_min_neighbors_ <<
      std::endl;
    std::cout << "planar_map_filter_max_small_eigenvalue_ratio:" <<
      planar_map_filter_max_small_eigenvalue_ratio_ << std::endl;
    std::cout << "planar_map_filter_min_middle_eigenvalue_ratio:" <<
      planar_map_filter_min_middle_eigenvalue_ratio_ << std::endl;
    std::cout << "planar_map_filter_min_retained_ratio:" <<
      planar_map_filter_min_retained_ratio_ << std::endl;
  }
  node_.declare_parameter("use_odom_input", false);
  node_.get_parameter("use_odom_input", use_odom_input_);
  node_.declare_parameter("submap_distance_threshold", 1.5);
  node_.get_parameter("submap_distance_threshold", submap_distance_threshold_);
  node_.declare_parameter("odom_cloud_sync_queue_size", 100);
  node_.get_parameter("odom_cloud_sync_queue_size", odom_cloud_sync_queue_size_);
  // v0.8 Phase 1 report-only degeneracy diagnostics (opt-in, default off:
  // empty path / false flag leaves default behavior untouched).
  node_.declare_parameter("degeneracy_diagnostics_csv_path", std::string(""));
  node_.get_parameter("degeneracy_diagnostics_csv_path", degeneracy_diagnostics_csv_path_);
  node_.declare_parameter("save_degeneracy_report", false);
  node_.get_parameter("save_degeneracy_report", save_degeneracy_report_);
  std::cout << "use_odom_input:" << std::boolalpha << use_odom_input_ << std::endl;
  if (use_odom_input_) {
    std::cout << "submap_distance_threshold[m]:" << submap_distance_threshold_ << std::endl;
  }
  if (!degeneracy_diagnostics_csv_path_.empty()) {
    std::cout << "degeneracy_diagnostics_csv_path:" << degeneracy_diagnostics_csv_path_ <<
      std::endl;
  }
  std::cout << "save_degeneracy_report:" << std::boolalpha << save_degeneracy_report_ <<
    std::endl;
  if (!degeneracy_diagnostics_csv_path_.empty()) {
    degeneracy_csv_ofs_.open(degeneracy_diagnostics_csv_path_);
    if (degeneracy_csv_ofs_.is_open()) {
      degeneracy_csv_ofs_ << degeneracy::degeneracyDiagnosticsCsvHeaderLine() << "\n";
    } else {
      RCLCPP_WARN(
        node_.get_logger(), "failed to open degeneracy_diagnostics_csv_path: %s (CSV disabled)",
        degeneracy_diagnostics_csv_path_.c_str());
      degeneracy_diagnostics_csv_path_.clear();
    }
  }
  std::cout << "use_imu_preintegration:" << std::boolalpha << use_imu_preintegration_ << std::endl;
  if (use_imu_preintegration_) {
    std::cout << "imu_rotation_info_roll_pitch:" << imu_rotation_info_roll_pitch_ << std::endl;
    std::cout << "imu_rotation_info_yaw:" << imu_rotation_info_yaw_ << std::endl;
  }
  if (use_gnss_) {
    std::cout << "gnss_topic:" << gnss_topic_ << std::endl;
    std::cout << "gnss_info_weight:" << gnss_info_weight_ << std::endl;
    std::cout << "gnss_use_covariance_weighting:" << std::boolalpha <<
      gnss_use_covariance_weighting_ << std::endl;
    std::cout << "gnss_covariance_min_variance_m2:" << gnss_covariance_min_variance_m2_ <<
      std::endl;
    std::cout << "gnss_covariance_max_variance_m2:" << gnss_covariance_max_variance_m2_ <<
      std::endl;
    std::cout << "gnss_rtk_fix_max_horizontal_stddev_m:" <<
      gnss_rtk_fix_max_horizontal_stddev_m_ << std::endl;
    std::cout << "gnss_rtk_fix_weight_scale:" << gnss_rtk_fix_weight_scale_ << std::endl;
    std::cout << "gnss_non_rtk_weight_scale:" << gnss_non_rtk_weight_scale_ << std::endl;
    std::cout << "gnss_origin_min_samples:" << gnss_origin_min_samples_ << std::endl;
    std::cout << "gnss_origin_consistency_threshold_m:"
              << gnss_origin_consistency_threshold_m_ << std::endl;
  }
  std::cout << "------------------" << std::endl;

  backend_->voxelgrid.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);

  backend_->registration =
    backend_core::makeLoopRegistration(registration_method, ndt_resolution, ndt_num_threads);
  if (!backend_->registration) {
    RCLCPP_ERROR(node_.get_logger(), "invalid registration_method");
    exit(1);
  }

  backend_core::DescriptorConfig descriptor_config;
  descriptor_config.use_scan_context = use_scan_context_;
  descriptor_config.use_bev_descriptor = use_bev_descriptor_;
  descriptor_config.use_solid_descriptor = use_solid_descriptor_;
  descriptor_config.use_triangle_descriptor = use_triangle_descriptor_;
  descriptor_config.bev_descriptor_grid_size_m = bev_descriptor_grid_size_m_;
  descriptor_config.bev_descriptor_grid_cells = bev_descriptor_grid_cells_;
  descriptor_config.bev_descriptor_yaw_bins = bev_descriptor_yaw_bins_;
  descriptor_config.triangle_descriptor_keypoint_mode = triangle_descriptor_keypoint_mode_;
  descriptor_config.triangle_descriptor_grid_size_m = triangle_descriptor_grid_size_m_;
  descriptor_config.triangle_descriptor_grid_cells = triangle_descriptor_grid_cells_;
  descriptor_config.triangle_descriptor_min_salience_m = triangle_descriptor_min_salience_m_;
  descriptor_config.triangle_descriptor_max_keypoints = triangle_descriptor_max_keypoints_;
  descriptor_config.triangle_descriptor_edge_voxel_size_m =
    triangle_descriptor_edge_voxel_size_m_;
  descriptor_config.triangle_descriptor_edge_neighbor_radius_m =
    triangle_descriptor_edge_neighbor_radius_m_;
  descriptor_config.triangle_descriptor_edge_min_neighbors =
    triangle_descriptor_edge_min_neighbors_;
  descriptor_config.triangle_descriptor_edge_min_edgeness =
    triangle_descriptor_edge_min_edgeness_;
  descriptor_config.triangle_descriptor_edge_nms_radius_m =
    triangle_descriptor_edge_nms_radius_m_;
  descriptor_config.triangle_descriptor_min_edge_m = triangle_descriptor_min_edge_m_;
  descriptor_config.triangle_descriptor_max_edge_m = triangle_descriptor_max_edge_m_;
  descriptor_config.triangle_descriptor_max_triangles = triangle_descriptor_max_triangles_;
  descriptor_config.triangle_descriptor_edge_bin_m = triangle_descriptor_edge_bin_m_;
  descriptor_config.triangle_descriptor_quad_feature_bin_m =
    triangle_descriptor_quad_feature_bin_m_;
  backend_->core.configure(descriptor_config);

  initializePubSub();

  map_save_srv_ = node_.create_service<std_srvs::srv::Empty>(
    "map_save",
    std::bind(
      &GraphBasedSlamComponent::Impl::handleMapSaveRequest,
      this,
      std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3));
}  // NOLINT(readability/fn_size)

GraphBasedSlamComponent::Impl::~Impl() = default;

void GraphBasedSlamComponent::Impl::initializePubSub()
{
  RCLCPP_INFO(node_.get_logger(), "initialize Publishers and Subscribers");

  auto map_array_callback =
    [this](const typename lidarslam_msgs::msg::MapArray::SharedPtr msg_ptr) -> void
    {
      {
        // Preserve input order, but stage expensive conversion and compressed
        // PCD writes outside GraphStateStore's snapshot mutex. Readers keep
        // seeing the previous complete graph until the move-only commit.
        std::lock_guard<std::mutex> ingest_lock(submap_ingest_mtx_);
        lidarslam_msgs::msg::MapArray staged_map = *msg_ptr;
        if (use_pcd_cache_) {
          stageMapArrayCloudCache(staged_map);
        }
        graph_state_.replace(std::move(staged_map));
      }
      runEventDrivenLoopSearch();
    };

  map_array_sub_ =
    node_.create_subscription<lidarslam_msgs::msg::MapArray>(
    "map_array", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(), map_array_callback);

  if (use_odom_input_) {
    // Deep queues so a long loop-search callback cannot overflow the
    // subscription histories and drop frames, plus stamp-based pairing so
    // the submap pose/cloud match is a function of the data, not of the
    // executor schedule. Reliability is kept as before (odom reliable,
    // cloud sensor-data/best-effort) for publisher compatibility.
    const size_t sync_depth = static_cast<size_t>(std::max(odom_cloud_sync_queue_size_, 1));
    rmw_qos_profile_t odom_qos = rmw_qos_profile_default;
    odom_qos.depth = sync_depth;
    rmw_qos_profile_t cloud_qos = rmw_qos_profile_sensor_data;
    cloud_qos.depth = sync_depth;
    odom_sync_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(
      &node_, "odom_input", odom_qos);
    cloud_sync_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
      &node_, "cloud_input", cloud_qos);
    odom_cloud_sync_ = std::make_shared<message_filters::Synchronizer<OdomCloudSyncPolicy>>(
      OdomCloudSyncPolicy(static_cast<uint32_t>(sync_depth)), *odom_sync_sub_, *cloud_sync_sub_);
    odom_cloud_sync_->registerCallback(
      std::bind(
        &GraphBasedSlamComponent::Impl::receiveSyncedOdomCloud, this, std::placeholders::_1,
        std::placeholders::_2));
    RCLCPP_INFO(
      node_.get_logger(), "Direct odom+cloud input mode enabled (stamp-synced, queue %zu)",
      sync_depth);
  }

  modified_map_pub_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>(
    "modified_map",
    rclcpp::QoS(10));

  modified_map_array_pub_ = node_.create_publisher<lidarslam_msgs::msg::MapArray>(
    "modified_map_array", rclcpp::QoS(10));

  modified_path_pub_ = node_.create_publisher<nav_msgs::msg::Path>(
    "modified_path",
    rclcpp::QoS(10));

  if (use_imu_preintegration_) {
    auto imu_callback =
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) -> void
      {
        receiveImu(*msg);
      };
    imu_sub_ = node_.create_subscription<sensor_msgs::msg::Imu>(
      "/imu", rclcpp::SensorDataQoS(), imu_callback);
    RCLCPP_INFO(node_.get_logger(), "IMU preintegration enabled, subscribed to /imu");
  }

  if (use_gnss_) {
    gnss_sub_ = node_.create_subscription<sensor_msgs::msg::NavSatFix>(
      gnss_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {receiveNavSatFix(*msg);});
    RCLCPP_INFO(
      node_.get_logger(),
      "GNSS constraints enabled, subscribed to %s",
      gnss_topic_.c_str());
  }

  RCLCPP_INFO(node_.get_logger(), "initialization end");
}

void GraphBasedSlamComponent::Impl::handleMapSaveRequest(
  const MapSaveRequestHeader request_header,
  const MapSaveRequest request,
  const MapSaveResponse response)
{
  static_cast<void>(request_header);
  static_cast<void>(request);
  static_cast<void>(response);

  std::cout << "Received an request to save the map" << std::endl;
  lidarslam_msgs::msg::MapArray map_array_msg;
  LoopEdges loop_edges;
  if (!snapshotGraphState(map_array_msg, loop_edges)) {
    std::cout << "initial map is not received" << std::endl;
    return;
  }
  doPoseAdjustment(map_array_msg, loop_edges, true);
}

bool GraphBasedSlamComponent::Impl::snapshotGraphState(
  lidarslam_msgs::msg::MapArray & map_array_msg,
  LoopEdges & loop_edges)
{
  return graph_state_.snapshot(map_array_msg, loop_edges);
}

void GraphBasedSlamComponent::Impl::snapshotLoopEdges(LoopEdges & loop_edges)
{
  loop_edges = graph_state_.snapshotLoopEdges();
}

bool GraphBasedSlamComponent::Impl::upsertLoopEdge(const LoopEdge & loop_edge)
{
  return graph_state_.upsertLoopEdge(loop_edge);
}
GraphBasedSlamComponent::Impl::LocalSubmapProvider
GraphBasedSlamComponent::Impl::makeFilteredLocalSubmapProvider(
  const lidarslam_msgs::msg::MapArray & map_array_msg)
{
  return [this, &map_array_msg](int ref_idx) -> pcl::PointCloud<pcl::PointXYZI>::Ptr {
           pcl::PointCloud<pcl::PointXYZI>::Ptr aggregated_cloud(
             new pcl::PointCloud<pcl::PointXYZI>);
           Eigen::Affine3d reference_affine;
           tf2::fromMsg(map_array_msg.submaps[ref_idx].pose, reference_affine);
           for (int k = 0; k < search_submap_num_ && (ref_idx - k) >= 0; ++k) {
             const int src_idx = ref_idx - k;
             pcl::PointCloud<pcl::PointXYZI>::Ptr cloud =
               loadSubmapCloud(map_array_msg, src_idx);
             if (!cloud || cloud->empty()) {
               continue;
             }
             Eigen::Affine3d src_affine;
             tf2::fromMsg(map_array_msg.submaps[src_idx].pose, src_affine);
             pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(
               new pcl::PointCloud<pcl::PointXYZI>);
             const Eigen::Matrix4f local_transform =
               (reference_affine.inverse() * src_affine).matrix().cast<float>();
             pcl::transformPointCloud(*cloud, *transformed_cloud, local_transform);
             *aggregated_cloud += *transformed_cloud;
           }

           pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(
             new pcl::PointCloud<pcl::PointXYZI>);
           if (aggregated_cloud->empty()) {
             return filtered_cloud;
           }
           backend_->voxelgrid.setInputCloud(aggregated_cloud);
           backend_->voxelgrid.filter(*filtered_cloud);
           return filtered_cloud;
         };
}

void GraphBasedSlamComponent::Impl::runEventDrivenLoopSearch()
{
  // A component may be hosted by a MultiThreadedExecutor. Elect one callback
  // to own BackendCore and coalesce notifications that arrive while it drains.
  backend_work_drain_.request([this]() {drainEventDrivenLoopSearch();});
}

void GraphBasedSlamComponent::Impl::drainEventDrivenLoopSearch()
{
  while (rclcpp::ok()) {
    lidarslam_msgs::msg::MapArray map_array_msg;
    LoopEdges loop_edges;
    if (!snapshotGraphState(map_array_msg, loop_edges)) {return;}
    const int num_submaps = static_cast<int>(map_array_msg.submaps.size());
    if (num_submaps < 2) {return;}
    int query_idx = last_searched_submap_idx_ + 1;
    if (query_idx < 1) {query_idx = 1;}
    if (query_idx >= num_submaps) {return;}
    if (map_array_msg.cloud_coordinate != map_array_msg.LOCAL) {
      RCLCPP_WARN(node_.get_logger(), "cloud_coordinate should be local, but it's not local.");
    }
    const auto build_filtered_local_submap = makeFilteredLocalSubmapProvider(map_array_msg);
    // One immutable snapshot serves the whole currently available batch.
    // Each search receives an explicit prefix length, avoiding one full
    // MapArray deep-copy per query without exposing future submaps to q.
    for (; query_idx < num_submaps && rclcpp::ok(); ++query_idx) {
      if (debug_flag_) {
        RCLCPP_INFO(
          node_.get_logger(), "event-driven loop search, query submap %d of %d", query_idx,
          num_submaps);
      }
      backend_->core.ingestDescriptors(query_idx + 1, build_filtered_local_submap);
      if (loop_search_schedule::shouldSearch(query_idx, loop_search_query_stride_)) {
        searchLoopForLatest(map_array_msg, loop_edges, query_idx + 1, query_idx);
      } else if (debug_flag_) {
        RCLCPP_INFO(
          node_.get_logger(), "loop search registration skipped for query submap %d by stride %d",
          query_idx, loop_search_query_stride_);
      }
      last_searched_submap_idx_ = query_idx;
    }
  }
}

void GraphBasedSlamComponent::Impl::searchLoopForLatest(
  const lidarslam_msgs::msg::MapArray & map_array_msg,
  LoopEdges & loop_edges,
  int num_submaps,
  int latest_idx)
{
  if (num_submaps < 1 || num_submaps > static_cast<int>(map_array_msg.submaps.size())) {
    return;
  }
  std::vector<backend_core::SubmapMeta> submaps;
  submaps.reserve(num_submaps);
  for (int i = 0; i < num_submaps; ++i) {
    const auto & submap = map_array_msg.submaps[i];
    backend_core::SubmapMeta meta;
    tf2::fromMsg(submap.pose, meta.pose);
    meta.travel_distance = submap.distance;
    submaps.push_back(meta);
  }

  const auto raw_cloud_provider =
    [this, &map_array_msg](int idx) -> pcl::PointCloud<pcl::PointXYZI>::Ptr {
      return loadSubmapCloud(map_array_msg, idx);
    };

  backend_core::LoopSearchConfig search_config;
  search_config.search_submap_num = search_submap_num_;
  search_config.prefer_scan_context_candidates = prefer_scan_context_candidates_;
  search_config.use_3d_bbs_for_scan_context = use_3d_bbs_for_scan_context_;
  search_config.three_d_bbs_source_submap_num = three_d_bbs_source_submap_num_;
  search_config.three_d_bbs_target_submap_radius = three_d_bbs_target_submap_radius_;
  search_config.three_d_bbs_voxel_leaf_size = three_d_bbs_voxel_leaf_size_;
  search_config.three_d_bbs_min_level_res = three_d_bbs_min_level_res_;
  search_config.three_d_bbs_max_level = three_d_bbs_max_level_;
  search_config.three_d_bbs_score_threshold_percentage =
    three_d_bbs_score_threshold_percentage_;
  search_config.three_d_bbs_timeout_msec = three_d_bbs_timeout_msec_;
  search_config.three_d_bbs_num_threads = three_d_bbs_num_threads_;
  search_config.three_d_bbs_translation_search_margin_m =
    three_d_bbs_translation_search_margin_m_;
  search_config.three_d_bbs_roll_pitch_search_deg = three_d_bbs_roll_pitch_search_deg_;
  search_config.three_d_bbs_yaw_search_deg = three_d_bbs_yaw_search_deg_;

  candidate_aggregator::Config & aggregator_config = search_config.aggregator;
  aggregator_config.debug = debug_flag_;
  aggregator_config.max_loop_candidate_count = max_loop_candidate_count_;
  aggregator_config.distance_loop_closure = distance_loop_closure_;
  aggregator_config.range_of_searching_loop_closure = range_of_searching_loop_closure_;
  aggregator_config.scan_context_threshold = scan_context_threshold_;
  aggregator_config.scan_context_query_stride = std::max(1, scan_context_query_stride_);
  aggregator_config.scan_context_exclude_recent = std::max(1, scan_context_exclude_recent_);
  aggregator_config.bev_use_mutual_visibility = bev_use_mutual_visibility_;
  aggregator_config.bev_mutual_visibility_min_overlap_ratio =
    bev_mutual_visibility_min_overlap_ratio_;
  aggregator_config.bev_mutual_visibility_occupancy_eps = bev_mutual_visibility_occupancy_eps_;
  aggregator_config.bev_descriptor_yaw_bins = bev_descriptor_yaw_bins_;
  aggregator_config.bev_descriptor_max_euclidean_distance_m =
    bev_descriptor_max_euclidean_distance_m_;
  aggregator_config.bev_descriptor_threshold = bev_descriptor_threshold_;
  aggregator_config.bev_descriptor_sequence_window = bev_descriptor_sequence_window_;
  aggregator_config.bev_descriptor_sequence_threshold = bev_descriptor_sequence_threshold_;
  aggregator_config.bev_descriptor_pose_consistency_threshold_m =
    bev_descriptor_pose_consistency_threshold_m_;
  aggregator_config.bev_descriptor_rerank_weight_m = bev_descriptor_rerank_weight_m_;
  aggregator_config.solid_descriptor_max_euclidean_distance_m =
    solid_descriptor_max_euclidean_distance_m_;
  aggregator_config.solid_descriptor_min_similarity = solid_descriptor_min_similarity_;
  aggregator_config.solid_descriptor_sequence_window = solid_descriptor_sequence_window_;
  aggregator_config.solid_descriptor_sequence_min_similarity =
    solid_descriptor_sequence_min_similarity_;
  aggregator_config.solid_descriptor_pose_consistency_threshold_m =
    solid_descriptor_pose_consistency_threshold_m_;
  aggregator_config.triangle_descriptor_exclude_recent = triangle_descriptor_exclude_recent_;
  aggregator_config.triangle_descriptor_edge_bin_m = triangle_descriptor_edge_bin_m_;
  aggregator_config.triangle_descriptor_quad_feature_bin_m =
    triangle_descriptor_quad_feature_bin_m_;
  aggregator_config.triangle_descriptor_inlier_translation_m =
    triangle_descriptor_inlier_translation_m_;
  aggregator_config.triangle_descriptor_inlier_rotation_deg =
    triangle_descriptor_inlier_rotation_deg_;
  aggregator_config.triangle_descriptor_min_inliers = triangle_descriptor_min_inliers_;
  aggregator_config.triangle_descriptor_min_inlier_ratio = triangle_descriptor_min_inlier_ratio_;
  aggregator_config.triangle_descriptor_max_pairs = triangle_descriptor_max_pairs_;
  aggregator_config.triangle_descriptor_min_4th_point_agreements =
    triangle_descriptor_min_4th_point_agreements_;
  aggregator_config.triangle_descriptor_fourth_point_max_distance_m =
    triangle_descriptor_fourth_point_max_distance_m_;
  aggregator_config.triangle_descriptor_refine_se3_with_all_inliers =
    triangle_descriptor_refine_se3_with_all_inliers_;
  aggregator_config.triangle_descriptor_min_votes = triangle_descriptor_min_votes_;
  aggregator_config.triangle_descriptor_skip_ransac = triangle_descriptor_skip_ransac_;
  aggregator_config.triangle_verify_with_bev = triangle_verify_with_bev_;
  aggregator_config.triangle_verify_bev_max_distance = triangle_verify_bev_max_distance_;

  loop_verifier::GateConfig & gate_config = search_config.gates;
  gate_config.generic_score_threshold = threshold_loop_closure_score_;
  gate_config.scan_context_score_threshold = scan_context_loop_closure_score_threshold_;
  gate_config.max_translation_m = loop_max_translation_delta_;
  gate_config.max_rotation_deg = loop_max_rotation_delta_deg_;
  gate_config.min_overlap_ratio = loop_min_overlap_ratio_;
  gate_config.min_overlap_ratio_large_correction =
    loop_min_overlap_ratio_large_correction_;
  gate_config.overlap_large_correction_translation_m =
    loop_overlap_large_correction_translation_m_;
  gate_config.overlap_max_distance_m = loop_overlap_max_distance_m_;
  gate_config.max_translation_descriptor_m = loop_max_translation_delta_descriptor_;
  gate_config.max_rotation_descriptor_deg = loop_max_rotation_delta_deg_descriptor_;

  const backend_core::LoopSearchOutput search_output = backend_->core.searchLoopForSubmap(
    submaps,
    latest_idx,
    search_config,
    raw_cloud_provider,
    *backend_->registration,
    backend_->voxelgrid,
    backend_->three_d_bbs_loop_verifier);

  for (const auto & line : search_output.logs) {
    if (line.via_logger) {
      RCLCPP_INFO(node_.get_logger(), "%s", line.text.c_str());
    } else {
      std::cout << line.text << std::endl;
    }
  }

  if (!search_output.proposal.found) {
    return;
  }

  LoopEdge loop_edge;
  loop_edge.pair_id = search_output.proposal.pair_id;
  loop_edge.relative_pose = search_output.proposal.relative_pose;
  loop_edge.fitness_score = search_output.proposal.fitness_score;
  const bool graph_changed = upsertLoopEdge(loop_edge);
  if (!graph_changed) {
    std::cout << "loop edge skipped as redundant or lower quality" << std::endl;
    return;
  }
  snapshotLoopEdges(loop_edges);
  lidarslam_msgs::msg::MapArray query_prefix;
  query_prefix.header = map_array_msg.header;
  query_prefix.cloud_coordinate = map_array_msg.cloud_coordinate;
  query_prefix.submaps.assign(
    map_array_msg.submaps.begin(), map_array_msg.submaps.begin() + num_submaps);
  doPoseAdjustment(std::move(query_prefix), loop_edges, use_save_map_in_loop_);
}

void GraphBasedSlamComponent::Impl::doPoseAdjustment(
  lidarslam_msgs::msg::MapArray map_array_msg,
  const LoopEdges & loop_edges,
  bool do_save_map)
{
  /* Plain-data inputs for the extracted pose-graph optimization
     (graph_based_slam/pose_graph_optimization.hpp); the IMU/GNSS buffers
     and their matching stay in the node, the g2o assembly does not. */
  int submaps_size = map_array_msg.submaps.size();
  std::vector<pose_graph::SubmapNode> submap_nodes;
  submap_nodes.reserve(submaps_size);
  for (int i = 0; i < submaps_size; i++) {
    Eigen::Affine3d affine;
    Eigen::fromMsg(map_array_msg.submaps[i].pose, affine);
    pose_graph::SubmapNode node;
    node.pose = Eigen::Isometry3d(affine.matrix());
    submap_nodes.push_back(node);
  }

  /* IMU rotation constraints (pre-pass over the IMU buffer) */
  std::vector<pose_graph::ImuRotationConstraint> imu_constraints;
  if (use_imu_preintegration_ && submaps_size > 1) {
    std::lock_guard<std::mutex> imu_lock(imu_mtx_);
    for (int i = 1; i < submaps_size; i++) {
      double t0 = rclcpp::Time(map_array_msg.submaps[i - 1].header.stamp).seconds();
      double t1 = rclcpp::Time(map_array_msg.submaps[i].header.stamp).seconds();
      if (t1 <= t0 || t1 - t0 > 30.0) {continue;}

      Eigen::Quaterniond imu_delta_q = integrateImuRotation(t0, t1);
      if (imu_delta_q.isApprox(Eigen::Quaterniond::Identity(), 1e-8)) {continue;}

      // Relative measurement: translation from odometry, rotation from IMU
      Eigen::Isometry3d odom_relative =
        submap_nodes[i - 1].pose.inverse() * submap_nodes[i].pose;
      pose_graph::ImuRotationConstraint imu_constraint;
      imu_constraint.from = i - 1;
      imu_constraint.to = i;
      imu_constraint.measurement = Eigen::Isometry3d::Identity();
      imu_constraint.measurement.linear() = imu_delta_q.toRotationMatrix();
      imu_constraint.measurement.translation() = odom_relative.translation();
      imu_constraints.push_back(imu_constraint);
    }
    if (debug_flag_) {
      RCLCPP_INFO(
        node_.get_logger(), "Added %zu IMU rotation constraint edges", imu_constraints.size());
    }
  }

  /* loop edges -> plain constraints */
  std::vector<pose_graph::LoopConstraint> loop_constraints;
  loop_constraints.reserve(loop_edges.size());
  for (const auto & loop_edge : loop_edges) {
    pose_graph::LoopConstraint loop_constraint;
    loop_constraint.from = loop_edge.pair_id.first;
    loop_constraint.to = loop_edge.pair_id.second;
    loop_constraint.relative_pose = loop_edge.relative_pose;
    loop_constraint.fitness_score = loop_edge.fitness_score;
    loop_constraints.push_back(loop_constraint);
  }

  /* GNSS anchors (pre-pass: nearest-measurement match over the buffer) */
  std::vector<pose_graph::GnssConstraint> gnss_constraints;
  if (use_gnss_ && gnss_origin_set_) {
    std::lock_guard<std::mutex> gnss_lock(gnss_mtx_);
    int gnss_rtk_like_edges_added = 0;

    for (int i = 0; i < submaps_size; i++) {
      double submap_time = rclcpp::Time(map_array_msg.submaps[i].header.stamp).seconds();
      // Find nearest GNSS measurement
      double best_dt = std::numeric_limits<double>::max();
      GnssEnu best_gnss;
      bool found = false;
      for (const auto & g : gnss_buffer_) {
        double dt = std::abs(g.stamp - submap_time);
        if (dt < best_dt) {
          best_dt = dt;
          best_gnss = g;
          found = true;
        }
      }
      if (!found || best_dt > 1.0) {continue;}  // Skip if no GNSS within 1 second

      pose_graph::GnssConstraint gnss_constraint;
      gnss_constraint.submap_index = i;
      gnss_constraint.position = Eigen::Vector3d(best_gnss.x, best_gnss.y, best_gnss.z);
      gnss_constraint.info_diag =
        Eigen::Vector3d(best_gnss.info_x, best_gnss.info_y, best_gnss.info_z);
      gnss_constraints.push_back(gnss_constraint);
      if (best_gnss.rtk_like) {
        gnss_rtk_like_edges_added++;
      }
    }
    if (debug_flag_) {
      RCLCPP_INFO(
        node_.get_logger(),
        "Added %zu GNSS position constraint edges (%d RTK-like by covariance)",
        gnss_constraints.size(), gnss_rtk_like_edges_added);
    }
  }

  // GNSS yaw alignment: the odometry frame's x axis is the initial heading,
  // the anchors' x axis is east. Estimate the planar odom->ENU transform,
  // move the whole graph into the ENU frame, and release the vertex-0 gauge
  // so the anchors govern the global pose — without this the anchors shear
  // the map (docs/research/gnss-constraint-first-validation.md).
  bool fix_first_vertex = true;
  if (gnss_align_yaw_ && !gnss_constraints.empty()) {
    std::vector<Eigen::Vector2d> odom_xy;
    std::vector<Eigen::Vector2d> enu_xy;
    odom_xy.reserve(gnss_constraints.size());
    enu_xy.reserve(gnss_constraints.size());
    for (const auto & gnss_constraint : gnss_constraints) {
      const Eigen::Vector3d p = submap_nodes[gnss_constraint.submap_index].pose.translation();
      odom_xy.emplace_back(p.x(), p.y());
      enu_xy.emplace_back(gnss_constraint.position.x(), gnss_constraint.position.y());
    }
    const auto alignment = gnss_alignment::estimatePlanarAlignment(
      odom_xy, enu_xy, gnss_yaw_alignment_min_anchors_, gnss_yaw_alignment_min_baseline_m_);
    if (alignment.valid) {
      Eigen::Isometry3d enu_from_odom = Eigen::Isometry3d::Identity();
      enu_from_odom.linear() =
        Eigen::AngleAxisd(alignment.yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      enu_from_odom.translation() =
        Eigen::Vector3d(alignment.translation.x(), alignment.translation.y(), 0.0);
      for (auto & node : submap_nodes) {
        node.pose = enu_from_odom * node.pose;
      }
      fix_first_vertex = false;
      RCLCPP_INFO(
        node_.get_logger(),
        "GNSS yaw alignment applied: yaw=%.2f deg, baseline=%.1f m, rms=%.2f m; "
        "vertex-0 gauge released, graph moves to the ENU frame",
        alignment.yaw_rad * 180.0 / M_PI, alignment.baseline_m, alignment.rms_residual_m);
    } else if (debug_flag_) {
      RCLCPP_INFO(
        node_.get_logger(),
        "GNSS yaw alignment not applied (pairs=%zu, baseline=%.1f m); keeping the "
        "vertex-0 gauge",
        odom_xy.size(), alignment.baseline_m);
    }
  }

  pose_graph::AdjacentEdgeConfig adjacent_cfg;
  adjacent_cfg.num_adjacent_pose_constraints = num_adjacent_pose_cnstraints_;
  adjacent_cfg.split_trans_rot = adjacent_edge_info_auto_scale_split_trans_rot_;
  adjacent_cfg.info_weight = adjacent_edge_info_weight_;
  adjacent_cfg.info_weight_trans = adjacent_edge_info_weight_trans_;
  adjacent_cfg.info_weight_rot = adjacent_edge_info_weight_rot_;

  pose_graph::LoopEdgeConfig loop_cfg;
  loop_cfg.info_weight = loop_edge_info_weight_;
  loop_cfg.robust_kernel_type = loop_edge_robust_kernel_type_;
  loop_cfg.robust_kernel_delta = loop_edge_robust_kernel_delta_;

  pose_graph::ImuEdgeConfig imu_cfg;
  imu_cfg.info_roll_pitch = imu_rotation_info_roll_pitch_;
  imu_cfg.info_yaw = imu_rotation_info_yaw_;

  pose_graph::Chi2Collection chi2_collection = pose_graph::Chi2Collection::NONE;
  if (adjacent_edge_info_auto_scale_) {
    chi2_collection = adjacent_edge_info_auto_scale_split_trans_rot_ ?
      pose_graph::Chi2Collection::SPLIT : pose_graph::Chi2Collection::UNIFIED;
  }

  std::string pose_graph_save_path = save_pose_graph_path_;
  const std::string bundle_pose_graph_path = map_save_dir_ + "/pose_graph.g2o";
  if (do_save_map) {
    std::filesystem::create_directories(map_save_dir_);
    pose_graph_save_path = bundle_pose_graph_path;
  }
  const pose_graph::OptimizationResult opt_result = pose_graph::optimizePoseGraph(
    submap_nodes, loop_constraints, imu_constraints, gnss_constraints,
    adjacent_cfg, loop_cfg, imu_cfg, chi2_collection,
    fix_first_vertex, /*iterations=*/ 10, pose_graph_save_path);

  if (do_save_map && !save_pose_graph_path_.empty() &&
    std::filesystem::path(save_pose_graph_path_).lexically_normal() !=
    std::filesystem::path(bundle_pose_graph_path).lexically_normal())
  {
    std::error_code copy_error;
    std::filesystem::copy_file(
      bundle_pose_graph_path, save_pose_graph_path_,
      std::filesystem::copy_options::overwrite_existing, copy_error);
    if (copy_error) {
      RCLCPP_WARN(
        node_.get_logger(), "failed to copy pose graph to configured path %s: %s",
        save_pose_graph_path_.c_str(), copy_error.message().c_str());
    }
  }

  if (adjacent_edge_info_auto_scale_ && submaps_size > 1) {
    graphslam::detail::AutoScaleConfig cfg;
    cfg.ema_alpha = adjacent_edge_info_auto_scale_ema_alpha_;
    cfg.min_scale = adjacent_edge_info_auto_scale_min_;
    cfg.max_scale = adjacent_edge_info_auto_scale_max_;

    if (adjacent_edge_info_auto_scale_split_trans_rot_) {
      // Level 2: split the post-opt residuals into translation / rotation
      // blocks and rescale w_trans and w_rot independently (the chi2
      // vectors come back from optimizePoseGraph in edge order).
      const double median_chi2_trans =
        graphslam::detail::medianChi2(opt_result.adjacent_trans_chi2);
      const double median_chi2_rot =
        graphslam::detail::medianChi2(opt_result.adjacent_rot_chi2);

      cfg.target_nis = adjacent_edge_info_auto_scale_target_nis_trans_;
      const double prev_w_trans = adjacent_edge_info_weight_trans_;
      adjacent_edge_info_weight_trans_ =
        graphslam::detail::nextScale(prev_w_trans, median_chi2_trans, cfg);

      cfg.target_nis = adjacent_edge_info_auto_scale_target_nis_rot_;
      const double prev_w_rot = adjacent_edge_info_weight_rot_;
      adjacent_edge_info_weight_rot_ =
        graphslam::detail::nextScale(prev_w_rot, median_chi2_rot, cfg);

      RCLCPP_INFO(
        node_.get_logger(),
        "[auto_scale_split] trans median_chi2=%.3f target=%.3f w_trans=%.3f -> %.3f | "
        "rot median_chi2=%.3f target=%.3f w_rot=%.3f -> %.3f (n=%zu)",
        median_chi2_trans, adjacent_edge_info_auto_scale_target_nis_trans_,
        prev_w_trans, adjacent_edge_info_weight_trans_,
        median_chi2_rot, adjacent_edge_info_auto_scale_target_nis_rot_,
        prev_w_rot, adjacent_edge_info_weight_rot_,
        opt_result.adjacent_trans_chi2.size());
    } else {
      const double median_chi2 = graphslam::detail::medianChi2(opt_result.adjacent_chi2);

      cfg.target_nis = adjacent_edge_info_auto_scale_target_nis_;
      const double prev_weight = adjacent_edge_info_weight_;
      adjacent_edge_info_weight_ =
        graphslam::detail::nextScale(prev_weight, median_chi2, cfg);

      RCLCPP_INFO(
        node_.get_logger(),
        "[auto_scale] median_chi2=%.3f (n=%zu) target=%.3f weight=%.3f -> %.3f",
        median_chi2, opt_result.adjacent_chi2.size(), cfg.target_nis, prev_weight,
        adjacent_edge_info_weight_);
    }
  }

  /* modified_map publish */
  std::cout << "modified_map publish" << std::endl;
  lidarslam_msgs::msg::MapArray modified_map_array_msg;
  modified_map_array_msg.header = map_array_msg.header;
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  std::vector<TimedSubmapCloud> dynamic_filter_submaps;
  if (do_save_map && use_dynamic_object_filter_) {
    dynamic_filter_submaps.reserve(submaps_size);
  }
  for (int i = 0; i < submaps_size; i++) {
    Eigen::Affine3d se3(opt_result.poses[i].matrix());
    geometry_msgs::msg::Pose pose = tf2::toMsg(se3);

    /* map */
    Eigen::Affine3d previous_affine;
    tf2::fromMsg(map_array_msg.submaps[i].pose, previous_affine);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr = loadSubmapCloud(map_array_msg, i);
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(
      new pcl::PointCloud<pcl::PointXYZI>());

    pcl::transformPointCloud(*cloud_ptr, *transformed_cloud_ptr, se3.matrix().cast<float>());
    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*transformed_cloud_ptr, *cloud_msg_ptr);
    *map_ptr += *transformed_cloud_ptr;
    if (do_save_map && use_dynamic_object_filter_) {
      dynamic_filter_submaps.push_back(
        TimedSubmapCloud{
          i,
          Eigen::Vector3d(se3.translation().x(), se3.translation().y(), se3.translation().z()),
          transformed_cloud_ptr});
    }

    /* submap */
    lidarslam_msgs::msg::SubMap submap;
    submap.header = map_array_msg.submaps[i].header;
    submap.pose = pose;
    submap.cloud = *cloud_msg_ptr;
    modified_map_array_msg.submaps.push_back(submap);

    /* path */
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = submap.header;
    pose_stamped.pose = submap.pose;
    path.poses.push_back(pose_stamped);
  }

  modified_map_array_pub_->publish(modified_map_array_msg);
  modified_path_pub_->publish(path);

  sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*map_ptr, *map_msg_ptr);
  map_msg_ptr->header.frame_id = "map";
  modified_map_pub_->publish(*map_msg_ptr);
  if (do_save_map) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_to_save = map_ptr;
    if (use_dynamic_object_filter_) {
      DynamicObjectFilterConfig filter_config;
      filter_config.voxel_size = dynamic_object_filter_voxel_size_;
      filter_config.min_observations = dynamic_object_filter_min_observations_;
      filter_config.temporal_window = dynamic_object_filter_temporal_window_;
      filter_config.max_range_from_sensor_m = dynamic_object_filter_max_range_from_sensor_m_;
      const auto filter_result =
        buildDynamicObjectFilteredMap(dynamic_filter_submaps, filter_config);
      if (!filter_result.cloud->empty()) {
        map_to_save = filter_result.cloud;
      }
      RCLCPP_INFO(
        node_.get_logger(),
        "Dynamic object filter: input_points %zu, kept %zu/%zu candidate voxels, "
        "removed %zu, always_keep %zu, output_points %zu",
        filter_result.stats.input_points,
        filter_result.stats.kept_candidate_voxels,
        filter_result.stats.candidate_voxels,
        filter_result.stats.removed_candidate_voxels,
        filter_result.stats.always_keep_voxels,
        filter_result.stats.output_points);
    }
    saveGridDividedMap(map_to_save);
    writeMapBundleArtifacts(map_array_msg, loop_edges, opt_result.poses);
    writeDegeneracyReport();
  }
}

void GraphBasedSlamComponent::Impl::receiveNavSatFix(const sensor_msgs::msg::NavSatFix & msg)
{
  if (msg.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
    return;  // No valid fix
  }
  if (!isUsableGnssFix(msg)) {
    return;
  }

  std::lock_guard<std::mutex> lock(gnss_mtx_);

  if (!gnss_origin_set_) {
    tryInitializeGnssOrigin(msg.latitude, msg.longitude, msg.altitude);
    if (!gnss_origin_set_) {
      return;
    }
  }

  Eigen::Vector3d enu = geodeticToEnu(msg.latitude, msg.longitude, msg.altitude);
  detail::GnssWeightingConfig weighting_config;
  weighting_config.base_info_weight = gnss_info_weight_;
  weighting_config.vertical_weight_scale = 0.1;
  weighting_config.use_covariance_weighting = gnss_use_covariance_weighting_;
  weighting_config.covariance_min_variance_m2 = gnss_covariance_min_variance_m2_;
  weighting_config.covariance_max_variance_m2 = gnss_covariance_max_variance_m2_;
  weighting_config.rtk_fix_max_horizontal_stddev_m = gnss_rtk_fix_max_horizontal_stddev_m_;
  weighting_config.rtk_fix_weight_scale = gnss_rtk_fix_weight_scale_;
  weighting_config.non_rtk_weight_scale = gnss_non_rtk_weight_scale_;
  const detail::GnssConstraintWeights gnss_weights =
    detail::computeGnssConstraintWeights(msg, weighting_config);
  const double receive_time_sec = node_.get_clock()->now().seconds();
  const double header_time_sec = rclcpp::Time(msg.header.stamp).seconds();
  const detail::GnssTimestampResolution stamp_resolution =
    detail::resolveGnssMeasurementStamp(
    header_time_sec, receive_time_sec, gnss_header_stamp_max_skew_sec_);
  GnssEnu g;
  g.stamp = stamp_resolution.stamp_sec;
  g.x = enu.x();
  g.y = enu.y();
  g.z = enu.z();
  g.info_x = gnss_weights.info_x;
  g.info_y = gnss_weights.info_y;
  g.info_z = gnss_weights.info_z;
  g.covariance_valid = gnss_weights.covariance_valid;
  g.rtk_like = gnss_weights.rtk_like;
  g.horizontal_stddev_m = gnss_weights.horizontal_stddev_m;
  gnss_buffer_.push_back(g);

  if (debug_flag_ && stamp_resolution.used_fallback) {
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(),
      *node_.get_clock(),
      5000,
      "GNSS header stamp %.3f s differs from receive time %.3f s by more than "
      "%.3f s; using receive time",
      header_time_sec, receive_time_sec, gnss_header_stamp_max_skew_sec_);
  }

  if (debug_flag_ && gnss_weights.covariance_valid) {
    RCLCPP_INFO_THROTTLE(
      node_.get_logger(),
      *node_.get_clock(),
      5000,
      "GNSS covariance weighting: horizontal_stddev=%.3f m, class=%s, info=(%.3f, %.3f, %.3f)",
      gnss_weights.horizontal_stddev_m,
      gnss_weights.rtk_like ? "rtk_like" : "non_rtk",
      gnss_weights.info_x, gnss_weights.info_y, gnss_weights.info_z);
  }

  // Limit buffer size
  if (gnss_buffer_.size() > 100000) {
    gnss_buffer_.erase(gnss_buffer_.begin(), gnss_buffer_.begin() + 25000);
  }
}

bool GraphBasedSlamComponent::Impl::isUsableGnssFix(const sensor_msgs::msg::NavSatFix & msg) const
{
  return detail::isUsableGeodeticFix(msg.latitude, msg.longitude, msg.altitude);
}

void GraphBasedSlamComponent::Impl::tryInitializeGnssOrigin(double lat, double lon, double alt)
{
  const detail::GnssOriginUpdate update = gnss_origin_accumulator_.add(lat, lon, alt);
  if (update.reset_after_jump) {
    RCLCPP_WARN(
      node_.get_logger(),
      "Resetting GNSS origin initialization after %.1f m jump in candidate fixes",
      update.deviation_m);
  }
  if (update.restarted_after_inconsistency) {
    RCLCPP_WARN(
      node_.get_logger(),
      "GNSS origin candidates were inconsistent (max deviation %.1f m), restarting accumulation",
      update.deviation_m);
  }
  if (!update.initialized) {
    return;
  }

  gnss_origin_lat_ = update.origin.latitude_deg;
  gnss_origin_lon_ = update.origin.longitude_deg;
  gnss_origin_alt_ = update.origin.altitude_m;
  gnss_origin_set_ = true;
  RCLCPP_INFO(
    node_.get_logger(),
    "GNSS origin set from %d consistent fixes: lat=%.8f, lon=%.8f, alt=%.2f",
    gnss_origin_min_samples_, gnss_origin_lat_, gnss_origin_lon_, gnss_origin_alt_);
}

Eigen::Vector3d GraphBasedSlamComponent::Impl::geodeticToEnu(
  double lat, double lon, double alt) const
{
  const detail::GeodeticOrigin origin {
    gnss_origin_lat_, gnss_origin_lon_, gnss_origin_alt_};
  return detail::geodeticToEnu(lat, lon, alt, origin);
}

void GraphBasedSlamComponent::Impl::receiveImu(const sensor_msgs::msg::Imu & msg)
{
  std::lock_guard<std::mutex> lock(imu_mtx_);
  StampedImu imu;
  imu.stamp = rclcpp::Time(msg.header.stamp).seconds();
  imu.gx = msg.angular_velocity.x;
  imu.gy = msg.angular_velocity.y;
  imu.gz = msg.angular_velocity.z;
  imu.ax = msg.linear_acceleration.x;
  imu.ay = msg.linear_acceleration.y;
  imu.az = msg.linear_acceleration.z;
  imu.qx = msg.orientation.x;
  imu.qy = msg.orientation.y;
  imu.qz = msg.orientation.z;
  imu.qw = msg.orientation.w;
  imu_buffer_.push_back(imu);
  if (imu_buffer_.size() > kMaxImuBufferSize) {
    imu_buffer_.erase(imu_buffer_.begin(), imu_buffer_.begin() + kMaxImuBufferSize / 4);
  }
}

Eigen::Quaterniond GraphBasedSlamComponent::Impl::integrateImuRotation(double t0, double t1) const
{
  // Integrate gyroscope measurements between t0 and t1
  Eigen::Quaterniond delta_q = Eigen::Quaterniond::Identity();

  // Find first IMU sample >= t0
  auto it = std::lower_bound(
    imu_buffer_.begin(), imu_buffer_.end(), t0,
    [](const StampedImu & imu, double t) {return imu.stamp < t;});

  if (it == imu_buffer_.end()) {
    return delta_q;  // no data
  }

  double prev_t = t0;
  for (; it != imu_buffer_.end() && it->stamp <= t1; ++it) {
    double dt = it->stamp - prev_t;
    if (dt <= 0.0 || dt > 0.5) {
      prev_t = it->stamp;
      continue;
    }
    // Small angle quaternion integration
    Eigen::Vector3d omega(it->gx, it->gy, it->gz);
    double angle = omega.norm() * dt;
    if (angle > 1e-10) {
      Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, omega.normalized()));
      delta_q = delta_q * dq;
      delta_q.normalize();
    }
    prev_t = it->stamp;
  }

  return delta_q;
}

void GraphBasedSlamComponent::Impl::receiveSyncedOdomCloud(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg)
{
  Eigen::Vector3d pos(
    odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
    odom_msg->pose.pose.position.z);
  if (!std::isfinite(pos.x()) || !std::isfinite(pos.y()) || !std::isfinite(pos.z())) {
    return;
  }
  if (debug_flag_ && !first_synced_input_logged_) {
    first_synced_input_logged_ = true;
    RCLCPP_INFO(
      node_.get_logger(), "First synced odom+cloud pair: (%.2f, %.2f, %.2f), %zu bytes", pos.x(),
      pos.y(), pos.z(), cloud_msg->data.size());
  }
  recordScanDegeneracy(*odom_msg);
  tryCreateSubmap(*odom_msg, *cloud_msg);
}

void GraphBasedSlamComponent::Impl::recordScanDegeneracy(const nav_msgs::msg::Odometry & odom_msg)
{
  // Opt-in, report-only (v0.8 Phase 1): compute nothing at all unless one of
  // the two diagnostics outputs was requested, so the default path stays
  // byte-for-byte identical in behavior and cost.
  const bool csv_enabled = degeneracy_csv_ofs_.is_open();
  if (!csv_enabled && !save_degeneracy_report_) {
    return;
  }

  const degeneracy::CovarianceLocalizabilityResult result =
    degeneracy::analyzeOdometryCovariance(odom_msg.pose.covariance);
  const double stamp_sec = rclcpp::Time(odom_msg.header.stamp).seconds();

  std::lock_guard<std::mutex> lock(degeneracy_mtx_);
  if (csv_enabled) {
    degeneracy_csv_ofs_ << degeneracy::degeneracyDiagnosticsCsvRowLine(stamp_sec, result) << "\n";
  }
  if (save_degeneracy_report_) {
    degeneracy_accumulator_.add(stamp_sec, result);
  }
}

void GraphBasedSlamComponent::Impl::writeDegeneracyReport()
{
  if (!save_degeneracy_report_) {
    return;
  }
  // Best-effort, same as the other map-bundle artifacts
  // (map_projector_info.yaml etc.): a failure to write the report must not
  // fail the map save itself.
  degeneracy::DegeneracyReportSummary summary;
  {
    std::lock_guard<std::mutex> lock(degeneracy_mtx_);
    summary = degeneracy_accumulator_.summary();
  }
  const std::string report_path = map_save_dir_ + "/degeneracy_report.yaml";
  std::ofstream report(report_path);
  if (!report.is_open()) {
    RCLCPP_WARN(node_.get_logger(), "failed to write degeneracy report: %s", report_path.c_str());
    return;
  }
  const std::vector<std::string> lines = degeneracy::degeneracyReportYamlLines(summary);
  for (size_t i = 0; i < lines.size(); ++i) {
    report << lines[i] << "\n";
  }
  std::cout << "Degeneracy report: " << report_path << std::endl;
}

void GraphBasedSlamComponent::Impl::tryCreateSubmap(
  const nav_msgs::msg::Odometry & odom_msg,
  const sensor_msgs::msg::PointCloud2 & cloud_msg)
{
  std::unique_lock<std::mutex> ingest_lock(submap_ingest_mtx_);
  Eigen::Vector3d pos(
    odom_msg.pose.pose.position.x,
    odom_msg.pose.pose.position.y,
    odom_msg.pose.pose.position.z);

  // Check distance threshold (semantics pinned by test_submap_creation.cpp)
  const submap_creation::Decision decision = submap_creation::evaluate(
    pos, last_submap_position_valid_, last_submap_position_, submap_distance_threshold_);
  if (!decision.create) {return;}
  accumulated_distance_ += decision.distance;
  last_submap_position_ = pos;
  last_submap_position_valid_ = true;

  // Create SubMap (use "map" frame for SLAM output regardless of odom frame)
  lidarslam_msgs::msg::SubMap submap;
  submap.header.stamp = odom_msg.header.stamp;
  submap.header.frame_id = "map";
  submap.distance = accumulated_distance_;
  submap.pose = odom_msg.pose.pose;
  submap.cloud = cloud_msg;
  submap.cloud.header.frame_id = odom_msg.child_frame_id;

  const int submap_idx = static_cast<int>(graph_state_.submapCount());
  if (use_pcd_cache_) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(submap.cloud, *cloud);
    if (saveSubmapToPCD(submap_idx, cloud)) {
      submap.cloud = sensor_msgs::msg::PointCloud2();
    }
  }
  std_msgs::msg::Header map_header;
  map_header.stamp = odom_msg.header.stamp;
  map_header.frame_id = "map";
  const int n = static_cast<int>(graph_state_.append(std::move(submap), map_header));
  ingest_lock.unlock();

  if (n % 50 == 0) {
    RCLCPP_INFO(
      node_.get_logger(), "Odom input: %d submaps, distance: %.1fm", n,
      accumulated_distance_);
  }

  runEventDrivenLoopSearch();
}

void GraphBasedSlamComponent::Impl::stageMapArrayCloudCache(
  lidarslam_msgs::msg::MapArray & map_array_msg)
{
  // Treat a full MapArray cache refresh as one repository transaction so
  // backend readers never observe a mixture of old and newly written files.
  std::lock_guard<std::mutex> cache_lock(pcd_cache_mtx_);
  for (int i = 0; i < static_cast<int>(map_array_msg.submaps.size()); ++i) {
    auto & submap = map_array_msg.submaps[i];
    if (submap.cloud.data.empty()) {
      continue;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(submap.cloud, *cloud);
    if (!cloud->empty() && saveSubmapToPCDUnlocked(i, cloud)) {
      submap.cloud = sensor_msgs::msg::PointCloud2();
    }
  }
}

bool GraphBasedSlamComponent::Impl::saveSubmapToPCD(
  int idx,
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud)
{
  std::lock_guard<std::mutex> cache_lock(pcd_cache_mtx_);
  return saveSubmapToPCDUnlocked(idx, cloud);
}

bool GraphBasedSlamComponent::Impl::saveSubmapToPCDUnlocked(
  int idx,
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud)
{
  std::string path = map_saver::submapCachePath(pcd_cache_dir_, idx);
  if (pcl::io::savePCDFileBinaryCompressed(path, *cloud) == 0) {
    return true;
  }
  RCLCPP_WARN(node_.get_logger(), "Failed to save PCD: %s", path.c_str());
  return false;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr GraphBasedSlamComponent::Impl::loadSubmapFromPCD(int idx)
{
  std::lock_guard<std::mutex> cache_lock(pcd_cache_mtx_);
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  std::string path = map_saver::submapCachePath(pcd_cache_dir_, idx);
  if (pcl::io::loadPCDFile(path, *cloud) == -1) {
    RCLCPP_WARN(node_.get_logger(), "Failed to load PCD: %s", path.c_str());
  }
  return cloud;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr GraphBasedSlamComponent::Impl::loadSubmapCloud(
  const lidarslam_msgs::msg::MapArray & map_array_msg,
  int idx)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
  if (use_pcd_cache_) {
    cloud = loadSubmapFromPCD(idx);
    if (!cloud->empty()) {
      return cloud;
    }
  }
  cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
  if (idx >= 0 && idx < static_cast<int>(map_array_msg.submaps.size())) {
    pcl::fromROSMsg(map_array_msg.submaps[idx].cloud, *cloud);
  }
  return cloud;
}

void GraphBasedSlamComponent::Impl::writeMapBundleArtifacts(
  const lidarslam_msgs::msg::MapArray & map_array_msg,
  const LoopEdges & loop_edges,
  const std::vector<Eigen::Isometry3d> & optimized_poses)
{
  std::filesystem::create_directories(map_save_dir_);

  const std::string trajectory_path = map_save_dir_ + "/trajectory_optimized.tum";
  std::ofstream trajectory(trajectory_path);
  if (!trajectory.is_open()) {
    RCLCPP_WARN(
      node_.get_logger(), "failed to write optimized trajectory: %s", trajectory_path.c_str());
  } else {
    const std::size_t count = std::min(map_array_msg.submaps.size(), optimized_poses.size());
    for (std::size_t i = 0; i < count; ++i) {
      const Eigen::Vector3d t = optimized_poses[i].translation();
      const Eigen::Quaterniond q(optimized_poses[i].rotation());
      map_saver::TrajectoryPose record;
      record.timestamp = rclcpp::Time(map_array_msg.submaps[i].header.stamp).seconds();
      record.tx = t.x();
      record.ty = t.y();
      record.tz = t.z();
      record.qx = q.x();
      record.qy = q.y();
      record.qz = q.z();
      record.qw = q.w();
      trajectory << map_saver::trajectoryTumLine(record) << "\n";
    }
  }

  const std::string loop_edges_path = map_save_dir_ + "/loop_edges.csv";
  std::ofstream loop_csv(loop_edges_path);
  if (!loop_csv.is_open()) {
    RCLCPP_WARN(node_.get_logger(), "failed to write loop edges: %s", loop_edges_path.c_str());
  } else {
    loop_csv << map_saver::loopEdgesCsvHeader() << "\n";
    for (const auto & edge : loop_edges) {
      const Eigen::Vector3d t = edge.relative_pose.translation();
      const Eigen::Quaterniond q(edge.relative_pose.rotation());
      map_saver::LoopEdgeRecord record;
      record.from = edge.pair_id.first;
      record.to = edge.pair_id.second;
      record.fitness = edge.fitness_score;
      record.tx = t.x();
      record.ty = t.y();
      record.tz = t.z();
      record.qx = q.x();
      record.qy = q.y();
      record.qz = q.z();
      record.qw = q.w();
      loop_csv << map_saver::loopEdgeCsvLine(record) << "\n";
    }
  }

  map_saver::BundleManifest manifest;
  manifest.frame_id = map_array_msg.header.frame_id.empty() ? "map" : map_array_msg.header.frame_id;
  manifest.submap_count = optimized_poses.size();
  manifest.loop_edge_count = loop_edges.size();
  manifest.map_leaf_size = map_leaf_size_;
  manifest.grid_size_x = map_grid_size_x_;
  manifest.grid_size_y = map_grid_size_y_;
  manifest.dynamic_object_filter = use_dynamic_object_filter_;
  manifest.planar_map_filter = use_planar_map_filter_;
  manifest.planar_map_filter_voxel_size = planar_map_filter_voxel_size_;
  manifest.planar_map_filter_min_neighbors = planar_map_filter_min_neighbors_;
  manifest.planar_map_filter_max_small_eigenvalue_ratio =
    planar_map_filter_max_small_eigenvalue_ratio_;
  manifest.planar_map_filter_min_middle_eigenvalue_ratio =
    planar_map_filter_min_middle_eigenvalue_ratio_;
  manifest.planar_map_filter_min_retained_ratio = planar_map_filter_min_retained_ratio_;
  const std::string manifest_path = map_save_dir_ + "/map_bundle.yaml";
  std::ofstream manifest_file(manifest_path);
  if (!manifest_file.is_open()) {
    RCLCPP_WARN(
      node_.get_logger(), "failed to write map bundle manifest: %s", manifest_path.c_str());
  } else {
    manifest_file << map_saver::bundleManifestYaml(manifest);
  }

  RCLCPP_INFO(
    node_.get_logger(), "Map bundle artifacts: trajectory=%s loops=%s manifest=%s",
    trajectory_path.c_str(), loop_edges_path.c_str(), manifest_path.c_str());
}

void GraphBasedSlamComponent::Impl::saveGridDividedMap(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & map)
{
  if (map->empty()) {
    std::cout << "Map is empty, skipping save." << std::endl;
    return;
  }

  // Create output directory (clean existing PCD files to prevent orphans)
  std::string out_dir = map_save_dir_ + "/pointcloud_map";
  if (std::filesystem::exists(out_dir)) {
    for (auto & entry : std::filesystem::directory_iterator(out_dir)) {
      if (entry.path().extension() == ".pcd" || entry.path().extension() == ".yaml") {
        std::filesystem::remove(entry.path());
      }
    }
  }
  std::filesystem::create_directories(out_dir);

  // Downsample the map
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setInputCloud(map);
  vg.setLeafSize(map_leaf_size_, map_leaf_size_, map_leaf_size_);
  vg.filter(*downsampled);

  std::cout << map_saver::downsampleLogLine(map->size(), downsampled->size(), map_leaf_size_)
            << std::endl;

  // Apply map-quality refinement to the same leaf-sized point set that is
  // exported and evaluated. Filtering before this VoxelGrid changes local
  // covariance according to raw submap sampling density and does not match
  // the saved-map contract.
  if (use_planar_map_filter_) {
    PlanarMapFilterConfig filter_config;
    filter_config.voxel_size = planar_map_filter_voxel_size_;
    filter_config.min_neighbors = planar_map_filter_min_neighbors_;
    filter_config.max_small_eigenvalue_ratio =
      planar_map_filter_max_small_eigenvalue_ratio_;
    filter_config.min_middle_eigenvalue_ratio =
      planar_map_filter_min_middle_eigenvalue_ratio_;
    filter_config.min_retained_ratio = planar_map_filter_min_retained_ratio_;
    const auto filter_result = buildPlanarMapFilteredMap(downsampled, filter_config);
    downsampled = filter_result.cloud;
    RCLCPP_INFO(
      node_.get_logger(),
      "Planar map filter: input_points %zu, finite %zu, planar_voxels %zu/%zu, "
      "supported_points %zu, output_points %zu, fallback %s",
      filter_result.stats.input_points,
      filter_result.stats.finite_points,
      filter_result.stats.planar_voxels,
      filter_result.stats.voxel_count,
      filter_result.stats.supported_points,
      filter_result.stats.output_points,
      filter_result.stats.fallback_to_input ? "true" : "false");
  }

  // Compute bounding box and grid-aligned bounds (semantics pinned by
  // test_map_saver.cpp)
  pcl::PointXYZI min_pt, max_pt;
  pcl::getMinMax3D(*downsampled, min_pt, max_pt);

  map_saver::GridConfig grid_config;
  grid_config.grid_size_x = map_grid_size_x_;
  grid_config.grid_size_y = map_grid_size_y_;
  const map_saver::GridBounds grid_bounds =
    map_saver::computeGridBounds(min_pt.x, min_pt.y, max_pt.x, max_pt.y, grid_config);

  // Assign points to grid cells
  std::map<std::pair<int, int>, pcl::PointCloud<pcl::PointXYZI>::Ptr> grid_cells;
  for (const auto & pt : downsampled->points) {
    auto key = map_saver::cellIndexFor(pt.x, pt.y, grid_bounds, grid_config);
    if (grid_cells.find(key) == grid_cells.end()) {
      grid_cells[key] = pcl::PointCloud<pcl::PointXYZI>::Ptr(
        new pcl::PointCloud<pcl::PointXYZI>);
    }
    grid_cells[key]->push_back(pt);
  }

  // Save each grid cell as PCD and build the Autoware pointcloud_map_loader
  // metadata (content produced in map_saver.hpp)
  std::ofstream meta(out_dir + "/pointcloud_map_metadata.yaml");
  meta << map_saver::metadataHeader(grid_config);

  int saved = 0;
  for (auto & [key, cloud] : grid_cells) {
    if (cloud->empty()) {continue;}
    const map_saver::CellFile cell = map_saver::makeCellFile(key, grid_bounds, grid_config);
    pcl::io::savePCDFileBinaryCompressed(out_dir + "/" + cell.filename, *cloud);
    meta << map_saver::metadataEntry(cell);
    saved++;
  }

  meta.close();

  // Also save the full map as a single PCD for convenience
  pcl::io::savePCDFileBinaryCompressed(map_save_dir_ + "/map.pcd", *downsampled);

  std::cout << map_saver::savedMapLogLine(saved, grid_config, out_dir) << std::endl;
  std::cout << "Total points: " << downsampled->size() << std::endl;
  std::cout << "Metadata: " << out_dir << "/pointcloud_map_metadata.yaml" << std::endl;

  // Always emit map_projector_info.yaml so Autoware can load pointcloud-only maps.
  std::string proj_file = map_save_dir_ + "/map_projector_info.yaml";
  std::ofstream proj(proj_file);
  proj << map_saver::projectorInfoYaml(gnss_origin_set_, gnss_origin_lat_, gnss_origin_lon_);
  std::cout << map_saver::projectorInfoLogLine(gnss_origin_set_, proj_file) << std::endl;
  proj.close();
}
}  // namespace graphslam

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(graphslam::GraphBasedSlamComponent)
