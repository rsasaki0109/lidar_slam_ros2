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

#include "graph_based_slam/graph_based_slam_component.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <unordered_map>

#include "graph_based_slam/dynamic_object_filter.hpp"
#include "g2o/core/robust_kernel_impl.h"

using namespace std::chrono_literals;

namespace graphslam
{
GraphBasedSlamComponent::GraphBasedSlamComponent(const rclcpp::NodeOptions & options)
: Node("graph_based_slam", options),
  clock_(RCL_ROS_TIME),
  tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
  listener_(tfbuffer_),
  broadcaster_(this)
{
  RCLCPP_INFO(get_logger(), "initialization start");
  std::string registration_method;
  double voxel_leaf_size;
  double ndt_resolution;
  int ndt_num_threads;

  declare_parameter("registration_method", "NDT");
  get_parameter("registration_method", registration_method);
  declare_parameter("voxel_leaf_size", 0.2);
  get_parameter("voxel_leaf_size", voxel_leaf_size);
  declare_parameter("ndt_resolution", 5.0);
  get_parameter("ndt_resolution", ndt_resolution);
  declare_parameter("ndt_num_threads", 0);
  get_parameter("ndt_num_threads", ndt_num_threads);
  declare_parameter("loop_detection_period", 1000);
  get_parameter("loop_detection_period", loop_detection_period_);
  declare_parameter("threshold_loop_closure_score", 1.0);
  get_parameter("threshold_loop_closure_score", threshold_loop_closure_score_);
  declare_parameter("scan_context_loop_closure_score_threshold", -1.0);
  get_parameter(
    "scan_context_loop_closure_score_threshold",
    scan_context_loop_closure_score_threshold_);
  declare_parameter("distance_loop_closure", 20.0);
  get_parameter("distance_loop_closure", distance_loop_closure_);
  declare_parameter("range_of_searching_loop_closure", 20.0);
  get_parameter("range_of_searching_loop_closure", range_of_searching_loop_closure_);
  declare_parameter("search_submap_num", 3);
  get_parameter("search_submap_num", search_submap_num_);
  declare_parameter("max_loop_candidate_count", 3);
  get_parameter("max_loop_candidate_count", max_loop_candidate_count_);
  declare_parameter("loop_edge_dedup_index_window", 8);
  get_parameter("loop_edge_dedup_index_window", loop_edge_dedup_index_window_);
  declare_parameter("loop_max_translation_delta", 15.0);
  get_parameter("loop_max_translation_delta", loop_max_translation_delta_);
  declare_parameter("loop_max_rotation_delta_deg", 45.0);
  get_parameter("loop_max_rotation_delta_deg", loop_max_rotation_delta_deg_);
  declare_parameter("num_adjacent_pose_cnstraints", 5);
  get_parameter("num_adjacent_pose_cnstraints", num_adjacent_pose_cnstraints_);
  declare_parameter("use_save_map_in_loop", true);
  get_parameter("use_save_map_in_loop", use_save_map_in_loop_);
  declare_parameter("debug_flag", false);
  get_parameter("debug_flag", debug_flag_);
  declare_parameter("adjacent_edge_info_weight", 1000.0);
  get_parameter("adjacent_edge_info_weight", adjacent_edge_info_weight_);
  declare_parameter("loop_edge_info_weight", 100.0);
  get_parameter("loop_edge_info_weight", loop_edge_info_weight_);
  declare_parameter("loop_edge_robust_kernel_delta", 1.0);
  get_parameter("loop_edge_robust_kernel_delta", loop_edge_robust_kernel_delta_);
  declare_parameter("use_scan_context", false);
  get_parameter("use_scan_context", use_scan_context_);
  declare_parameter("use_bev_descriptor", false);
  get_parameter("use_bev_descriptor", use_bev_descriptor_);
  declare_parameter("use_solid_descriptor", false);
  get_parameter("use_solid_descriptor", use_solid_descriptor_);
  declare_parameter("use_pcd_cache", false);
  get_parameter("use_pcd_cache", use_pcd_cache_);
  declare_parameter("pcd_cache_dir", std::string("/tmp/graph_slam_pcd_cache"));
  get_parameter("pcd_cache_dir", pcd_cache_dir_);
  if (use_pcd_cache_) {
    std::filesystem::create_directories(pcd_cache_dir_);
    std::cout << "pcd_cache_dir:" << pcd_cache_dir_ << std::endl;
  }
  declare_parameter("scan_context_threshold", 0.3);
  get_parameter("scan_context_threshold", scan_context_threshold_);
  declare_parameter("bev_descriptor_threshold", 0.20);
  get_parameter("bev_descriptor_threshold", bev_descriptor_threshold_);
  declare_parameter("bev_descriptor_grid_size_m", 80.0);
  get_parameter("bev_descriptor_grid_size_m", bev_descriptor_grid_size_m_);
  declare_parameter("bev_descriptor_grid_cells", 40);
  get_parameter("bev_descriptor_grid_cells", bev_descriptor_grid_cells_);
  declare_parameter("bev_descriptor_yaw_bins", 24);
  get_parameter("bev_descriptor_yaw_bins", bev_descriptor_yaw_bins_);
  declare_parameter("bev_descriptor_sequence_window", 0);
  get_parameter("bev_descriptor_sequence_window", bev_descriptor_sequence_window_);
  declare_parameter("bev_descriptor_sequence_threshold", -1.0);
  get_parameter("bev_descriptor_sequence_threshold", bev_descriptor_sequence_threshold_);
  declare_parameter("bev_descriptor_pose_consistency_threshold_m", -1.0);
  get_parameter(
    "bev_descriptor_pose_consistency_threshold_m",
    bev_descriptor_pose_consistency_threshold_m_);
  declare_parameter("bev_descriptor_max_euclidean_distance_m", -1.0);
  get_parameter(
    "bev_descriptor_max_euclidean_distance_m",
    bev_descriptor_max_euclidean_distance_m_);
  declare_parameter("bev_descriptor_rerank_weight_m", 100.0);
  get_parameter("bev_descriptor_rerank_weight_m", bev_descriptor_rerank_weight_m_);
  declare_parameter("solid_descriptor_min_similarity", 0.70);
  get_parameter("solid_descriptor_min_similarity", solid_descriptor_min_similarity_);
  declare_parameter("solid_descriptor_sequence_window", 0);
  get_parameter("solid_descriptor_sequence_window", solid_descriptor_sequence_window_);
  declare_parameter("solid_descriptor_sequence_min_similarity", -1.0);
  get_parameter(
    "solid_descriptor_sequence_min_similarity",
    solid_descriptor_sequence_min_similarity_);
  declare_parameter("solid_descriptor_pose_consistency_threshold_m", -1.0);
  get_parameter(
    "solid_descriptor_pose_consistency_threshold_m",
    solid_descriptor_pose_consistency_threshold_m_);
  declare_parameter("solid_descriptor_max_euclidean_distance_m", -1.0);
  get_parameter(
    "solid_descriptor_max_euclidean_distance_m",
    solid_descriptor_max_euclidean_distance_m_);
  declare_parameter("prefer_scan_context_candidates", false);
  get_parameter("prefer_scan_context_candidates", prefer_scan_context_candidates_);
  declare_parameter("use_3d_bbs_for_scan_context", false);
  get_parameter("use_3d_bbs_for_scan_context", use_3d_bbs_for_scan_context_);
  declare_parameter("three_d_bbs_min_level_res", 1.0);
  get_parameter("three_d_bbs_min_level_res", three_d_bbs_min_level_res_);
  declare_parameter("three_d_bbs_max_level", 3);
  get_parameter("three_d_bbs_max_level", three_d_bbs_max_level_);
  declare_parameter("three_d_bbs_score_threshold_percentage", 0.25);
  get_parameter(
    "three_d_bbs_score_threshold_percentage",
    three_d_bbs_score_threshold_percentage_);
  declare_parameter("three_d_bbs_timeout_msec", 50);
  get_parameter("three_d_bbs_timeout_msec", three_d_bbs_timeout_msec_);
  declare_parameter("three_d_bbs_num_threads", 0);
  get_parameter("three_d_bbs_num_threads", three_d_bbs_num_threads_);
  declare_parameter("three_d_bbs_voxel_leaf_size", 1.0);
  get_parameter("three_d_bbs_voxel_leaf_size", three_d_bbs_voxel_leaf_size_);
  declare_parameter("three_d_bbs_source_submap_num", 2);
  get_parameter("three_d_bbs_source_submap_num", three_d_bbs_source_submap_num_);
  declare_parameter("three_d_bbs_target_submap_radius", 1);
  get_parameter("three_d_bbs_target_submap_radius", three_d_bbs_target_submap_radius_);
  declare_parameter("three_d_bbs_translation_search_margin_m", 15.0);
  get_parameter(
    "three_d_bbs_translation_search_margin_m",
    three_d_bbs_translation_search_margin_m_);
  declare_parameter("three_d_bbs_roll_pitch_search_deg", 10.0);
  get_parameter(
    "three_d_bbs_roll_pitch_search_deg",
    three_d_bbs_roll_pitch_search_deg_);
  declare_parameter("three_d_bbs_yaw_search_deg", 180.0);
  get_parameter("three_d_bbs_yaw_search_deg", three_d_bbs_yaw_search_deg_);
  declare_parameter("use_dynamic_object_filter", false);
  get_parameter("use_dynamic_object_filter", use_dynamic_object_filter_);
  declare_parameter("dynamic_object_filter_voxel_size", 0.3);
  get_parameter("dynamic_object_filter_voxel_size", dynamic_object_filter_voxel_size_);
  declare_parameter("dynamic_object_filter_min_observations", 2);
  get_parameter(
    "dynamic_object_filter_min_observations",
    dynamic_object_filter_min_observations_);
  declare_parameter("dynamic_object_filter_temporal_window", 5);
  get_parameter(
    "dynamic_object_filter_temporal_window",
    dynamic_object_filter_temporal_window_);
  declare_parameter("dynamic_object_filter_max_range_from_sensor_m", 30.0);
  get_parameter(
    "dynamic_object_filter_max_range_from_sensor_m",
    dynamic_object_filter_max_range_from_sensor_m_);
  declare_parameter("map_save_dir", std::string("."));
  get_parameter("map_save_dir", map_save_dir_);
  declare_parameter("map_grid_size_x", 20.0);
  get_parameter("map_grid_size_x", map_grid_size_x_);
  declare_parameter("map_grid_size_y", 20.0);
  get_parameter("map_grid_size_y", map_grid_size_y_);
  declare_parameter("map_leaf_size", 0.2);
  get_parameter("map_leaf_size", map_leaf_size_);
  declare_parameter("use_gnss", false);
  get_parameter("use_gnss", use_gnss_);
  declare_parameter("gnss_topic", std::string("/gnss/fix"));
  get_parameter("gnss_topic", gnss_topic_);
  declare_parameter("gnss_info_weight", 1.0);
  get_parameter("gnss_info_weight", gnss_info_weight_);
  declare_parameter("gnss_use_covariance_weighting", true);
  get_parameter("gnss_use_covariance_weighting", gnss_use_covariance_weighting_);
  declare_parameter("gnss_covariance_min_variance_m2", 0.01);
  get_parameter("gnss_covariance_min_variance_m2", gnss_covariance_min_variance_m2_);
  declare_parameter("gnss_covariance_max_variance_m2", 25.0);
  get_parameter("gnss_covariance_max_variance_m2", gnss_covariance_max_variance_m2_);
  declare_parameter("gnss_rtk_fix_max_horizontal_stddev_m", 0.3);
  get_parameter(
    "gnss_rtk_fix_max_horizontal_stddev_m",
    gnss_rtk_fix_max_horizontal_stddev_m_);
  declare_parameter("gnss_rtk_fix_weight_scale", 3.0);
  get_parameter("gnss_rtk_fix_weight_scale", gnss_rtk_fix_weight_scale_);
  declare_parameter("gnss_non_rtk_weight_scale", 1.0);
  get_parameter("gnss_non_rtk_weight_scale", gnss_non_rtk_weight_scale_);
  declare_parameter("gnss_header_stamp_max_skew_sec", 30.0);
  get_parameter("gnss_header_stamp_max_skew_sec", gnss_header_stamp_max_skew_sec_);
  declare_parameter("gnss_origin_min_samples", 3);
  get_parameter("gnss_origin_min_samples", gnss_origin_min_samples_);
  declare_parameter("gnss_origin_consistency_threshold_m", 20.0);
  get_parameter(
    "gnss_origin_consistency_threshold_m",
    gnss_origin_consistency_threshold_m_);
  declare_parameter("use_imu_preintegration", false);
  get_parameter("use_imu_preintegration", use_imu_preintegration_);
  declare_parameter("imu_rotation_info_roll_pitch", 100.0);
  get_parameter("imu_rotation_info_roll_pitch", imu_rotation_info_roll_pitch_);
  declare_parameter("imu_rotation_info_yaw", 10.0);
  get_parameter("imu_rotation_info_yaw", imu_rotation_info_yaw_);

  if (gnss_origin_min_samples_ < 1) {
    RCLCPP_WARN(
      get_logger(),
      "gnss_origin_min_samples must be >= 1, clamping %d to 1",
      gnss_origin_min_samples_);
    gnss_origin_min_samples_ = 1;
  }
  if (gnss_origin_consistency_threshold_m_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "gnss_origin_consistency_threshold_m must be positive, resetting %.3f to 20.0",
      gnss_origin_consistency_threshold_m_);
    gnss_origin_consistency_threshold_m_ = 20.0;
  }
  if (gnss_covariance_min_variance_m2_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "gnss_covariance_min_variance_m2 must be positive, resetting %.6f to 0.01",
      gnss_covariance_min_variance_m2_);
    gnss_covariance_min_variance_m2_ = 0.01;
  }
  if (gnss_covariance_max_variance_m2_ < gnss_covariance_min_variance_m2_) {
    RCLCPP_WARN(
      get_logger(),
      "gnss_covariance_max_variance_m2 must be >= min variance, resetting %.6f to %.6f",
      gnss_covariance_max_variance_m2_, gnss_covariance_min_variance_m2_);
    gnss_covariance_max_variance_m2_ = gnss_covariance_min_variance_m2_;
  }
  if (gnss_rtk_fix_max_horizontal_stddev_m_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "gnss_rtk_fix_max_horizontal_stddev_m must be positive, resetting %.3f to 0.3",
      gnss_rtk_fix_max_horizontal_stddev_m_);
    gnss_rtk_fix_max_horizontal_stddev_m_ = 0.3;
  }
  if (gnss_rtk_fix_weight_scale_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "gnss_rtk_fix_weight_scale must be positive, resetting %.3f to 3.0",
      gnss_rtk_fix_weight_scale_);
    gnss_rtk_fix_weight_scale_ = 3.0;
  }
  if (gnss_non_rtk_weight_scale_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "gnss_non_rtk_weight_scale must be positive, resetting %.3f to 1.0",
      gnss_non_rtk_weight_scale_);
    gnss_non_rtk_weight_scale_ = 1.0;
  }
  if (gnss_header_stamp_max_skew_sec_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "gnss_header_stamp_max_skew_sec must be positive, resetting %.3f to 30.0",
      gnss_header_stamp_max_skew_sec_);
    gnss_header_stamp_max_skew_sec_ = 30.0;
  }
  if (search_submap_num_ < 1) {
    RCLCPP_WARN(
      get_logger(),
      "search_submap_num must be >= 1, clamping %d to 1",
      search_submap_num_);
    search_submap_num_ = 1;
  }
  if (max_loop_candidate_count_ < 1) {
    RCLCPP_WARN(
      get_logger(),
      "max_loop_candidate_count must be >= 1, clamping %d to 1",
      max_loop_candidate_count_);
    max_loop_candidate_count_ = 1;
  }
  if (loop_edge_dedup_index_window_ < 0) {
    RCLCPP_WARN(
      get_logger(),
      "loop_edge_dedup_index_window must be >= 0, clamping %d to 0",
      loop_edge_dedup_index_window_);
    loop_edge_dedup_index_window_ = 0;
  }
  if (loop_max_translation_delta_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "loop_max_translation_delta must be positive, resetting %.3f to 15.0",
      loop_max_translation_delta_);
    loop_max_translation_delta_ = 15.0;
  }
  if (loop_max_rotation_delta_deg_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "loop_max_rotation_delta_deg must be positive, resetting %.3f to 45.0",
      loop_max_rotation_delta_deg_);
    loop_max_rotation_delta_deg_ = 45.0;
  }
  if (num_adjacent_pose_cnstraints_ < 1) {
    RCLCPP_WARN(
      get_logger(),
      "num_adjacent_pose_cnstraints must be >= 1, clamping %d to 1",
      num_adjacent_pose_cnstraints_);
    num_adjacent_pose_cnstraints_ = 1;
  }
  if (adjacent_edge_info_weight_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "adjacent_edge_info_weight must be positive, resetting %.3f to 1000.0",
      adjacent_edge_info_weight_);
    adjacent_edge_info_weight_ = 1000.0;
  }
  if (loop_edge_info_weight_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "loop_edge_info_weight must be positive, resetting %.3f to 100.0",
      loop_edge_info_weight_);
    loop_edge_info_weight_ = 100.0;
  }
  if (loop_edge_robust_kernel_delta_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "loop_edge_robust_kernel_delta must be positive, resetting %.3f to 1.0",
      loop_edge_robust_kernel_delta_);
    loop_edge_robust_kernel_delta_ = 1.0;
  }
  if (bev_descriptor_threshold_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "bev_descriptor_threshold must be positive, resetting %.3f to 0.20",
      bev_descriptor_threshold_);
    bev_descriptor_threshold_ = 0.20;
  }
  if (bev_descriptor_grid_size_m_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "bev_descriptor_grid_size_m must be positive, resetting %.3f to 80.0",
      bev_descriptor_grid_size_m_);
    bev_descriptor_grid_size_m_ = 80.0;
  }
  if (bev_descriptor_grid_cells_ < 8) {
    RCLCPP_WARN(
      get_logger(),
      "bev_descriptor_grid_cells must be >= 8, clamping %d to 8",
      bev_descriptor_grid_cells_);
    bev_descriptor_grid_cells_ = 8;
  }
  if (bev_descriptor_yaw_bins_ < 1) {
    RCLCPP_WARN(
      get_logger(),
      "bev_descriptor_yaw_bins must be >= 1, clamping %d to 1",
      bev_descriptor_yaw_bins_);
    bev_descriptor_yaw_bins_ = 1;
  }
  if (bev_descriptor_sequence_window_ < 0) {
    RCLCPP_WARN(
      get_logger(),
      "bev_descriptor_sequence_window must be >= 0, clamping %d to 0",
      bev_descriptor_sequence_window_);
    bev_descriptor_sequence_window_ = 0;
  }
  if (bev_descriptor_sequence_threshold_ <= 0.0) {
    bev_descriptor_sequence_threshold_ = bev_descriptor_threshold_;
  }
  if (bev_descriptor_rerank_weight_m_ < 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "bev_descriptor_rerank_weight_m must be >= 0.0, clamping %.3f to 0.0",
      bev_descriptor_rerank_weight_m_);
    bev_descriptor_rerank_weight_m_ = 0.0;
  }
  if (bev_descriptor_pose_consistency_threshold_m_ == 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "bev_descriptor_pose_consistency_threshold_m must be negative (disabled) or positive, "
      "resetting 0.0 to disabled");
    bev_descriptor_pose_consistency_threshold_m_ = -1.0;
  }
  if (
    solid_descriptor_min_similarity_ <= -1.0 ||
    solid_descriptor_min_similarity_ > 1.0)
  {
    RCLCPP_WARN(
      get_logger(),
      "solid_descriptor_min_similarity must be in (-1, 1], resetting %.3f to 0.70",
      solid_descriptor_min_similarity_);
    solid_descriptor_min_similarity_ = 0.70;
  }
  if (solid_descriptor_sequence_window_ < 0) {
    RCLCPP_WARN(
      get_logger(),
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
      get_logger(),
      "solid_descriptor_pose_consistency_threshold_m must be negative (disabled) or positive, "
      "resetting 0.0 to disabled");
    solid_descriptor_pose_consistency_threshold_m_ = -1.0;
  }
  if (three_d_bbs_min_level_res_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "three_d_bbs_min_level_res must be positive, resetting %.3f to 1.0",
      three_d_bbs_min_level_res_);
    three_d_bbs_min_level_res_ = 1.0;
  }
  if (three_d_bbs_max_level_ < 1) {
    RCLCPP_WARN(
      get_logger(),
      "three_d_bbs_max_level must be >= 1, clamping %d to 1",
      three_d_bbs_max_level_);
    three_d_bbs_max_level_ = 1;
  }
  if (three_d_bbs_score_threshold_percentage_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "three_d_bbs_score_threshold_percentage must be positive, resetting %.3f to 0.25",
      three_d_bbs_score_threshold_percentage_);
    three_d_bbs_score_threshold_percentage_ = 0.25;
  }
  if (three_d_bbs_voxel_leaf_size_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "three_d_bbs_voxel_leaf_size must be positive, resetting %.3f to 1.0",
      three_d_bbs_voxel_leaf_size_);
    three_d_bbs_voxel_leaf_size_ = 1.0;
  }
  if (three_d_bbs_source_submap_num_ < 1) {
    RCLCPP_WARN(
      get_logger(),
      "three_d_bbs_source_submap_num must be >= 1, clamping %d to 1",
      three_d_bbs_source_submap_num_);
    three_d_bbs_source_submap_num_ = 1;
  }
  if (three_d_bbs_target_submap_radius_ < 0) {
    RCLCPP_WARN(
      get_logger(),
      "three_d_bbs_target_submap_radius must be >= 0, clamping %d to 0",
      three_d_bbs_target_submap_radius_);
    three_d_bbs_target_submap_radius_ = 0;
  }
  if (three_d_bbs_translation_search_margin_m_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "three_d_bbs_translation_search_margin_m must be positive, resetting %.3f to 15.0",
      three_d_bbs_translation_search_margin_m_);
    three_d_bbs_translation_search_margin_m_ = 15.0;
  }
  if (three_d_bbs_roll_pitch_search_deg_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "three_d_bbs_roll_pitch_search_deg must be positive, resetting %.3f to 10.0",
      three_d_bbs_roll_pitch_search_deg_);
    three_d_bbs_roll_pitch_search_deg_ = 10.0;
  }
  if (three_d_bbs_yaw_search_deg_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "three_d_bbs_yaw_search_deg must be positive, resetting %.3f to 180.0",
      three_d_bbs_yaw_search_deg_);
    three_d_bbs_yaw_search_deg_ = 180.0;
  }
  if (dynamic_object_filter_voxel_size_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "dynamic_object_filter_voxel_size must be positive, resetting %.3f to 0.3",
      dynamic_object_filter_voxel_size_);
    dynamic_object_filter_voxel_size_ = 0.3;
  }
  if (dynamic_object_filter_min_observations_ < 1) {
    RCLCPP_WARN(
      get_logger(),
      "dynamic_object_filter_min_observations must be >= 1, clamping %d to 1",
      dynamic_object_filter_min_observations_);
    dynamic_object_filter_min_observations_ = 1;
  }
  if (dynamic_object_filter_temporal_window_ < 0) {
    RCLCPP_WARN(
      get_logger(),
      "dynamic_object_filter_temporal_window must be >= 0, clamping %d to 0",
      dynamic_object_filter_temporal_window_);
    dynamic_object_filter_temporal_window_ = 0;
  }
  if (dynamic_object_filter_max_range_from_sensor_m_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "dynamic_object_filter_max_range_from_sensor_m must be positive, resetting %.3f to 30.0",
      dynamic_object_filter_max_range_from_sensor_m_);
    dynamic_object_filter_max_range_from_sensor_m_ = 30.0;
  }
  std::cout << "registration_method:" << registration_method << std::endl;
  std::cout << "voxel_leaf_size[m]:" << voxel_leaf_size << std::endl;
  std::cout << "ndt_resolution[m]:" << ndt_resolution << std::endl;
  std::cout << "ndt_num_threads:" << ndt_num_threads << std::endl;
  std::cout << "loop_detection_period[Hz]:" << loop_detection_period_ << std::endl;
  std::cout << "threshold_loop_closure_score:" << threshold_loop_closure_score_ << std::endl;
  std::cout << "scan_context_loop_closure_score_threshold:" <<
    scan_context_loop_closure_score_threshold_ << std::endl;
  std::cout << "distance_loop_closure[m]:" << distance_loop_closure_ << std::endl;
  std::cout << "range_of_searching_loop_closure[m]:" << range_of_searching_loop_closure_ <<
    std::endl;
  std::cout << "search_submap_num:" << search_submap_num_ << std::endl;
  std::cout << "max_loop_candidate_count:" << max_loop_candidate_count_ << std::endl;
  std::cout << "loop_edge_dedup_index_window:" << loop_edge_dedup_index_window_ << std::endl;
  std::cout << "loop_max_translation_delta[m]:" << loop_max_translation_delta_ << std::endl;
  std::cout << "loop_max_rotation_delta[deg]:" << loop_max_rotation_delta_deg_ << std::endl;
  std::cout << "num_adjacent_pose_cnstraints:" << num_adjacent_pose_cnstraints_ << std::endl;
  std::cout << "adjacent_edge_info_weight:" << adjacent_edge_info_weight_ << std::endl;
  std::cout << "loop_edge_info_weight:" << loop_edge_info_weight_ << std::endl;
  std::cout << "loop_edge_robust_kernel_delta:" << loop_edge_robust_kernel_delta_ << std::endl;
  std::cout << "use_save_map_in_loop:" << std::boolalpha << use_save_map_in_loop_ << std::endl;
  std::cout << "debug_flag:" << std::boolalpha << debug_flag_ << std::endl;
  std::cout << "use_scan_context:" << std::boolalpha << use_scan_context_ << std::endl;
  if (use_scan_context_) {
    std::cout << "scan_context_threshold:" << scan_context_threshold_ << std::endl;
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
  declare_parameter("use_odom_input", false);
  get_parameter("use_odom_input", use_odom_input_);
  declare_parameter("submap_distance_threshold", 1.5);
  get_parameter("submap_distance_threshold", submap_distance_threshold_);
  std::cout << "use_odom_input:" << std::boolalpha << use_odom_input_ << std::endl;
  if (use_odom_input_) {
    std::cout << "submap_distance_threshold[m]:" << submap_distance_threshold_ << std::endl;
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

  voxelgrid_.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);

  if (registration_method == "NDT") {
    boost::shared_ptr<pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>>
    ndt(new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt->setMaximumIterations(100);
    ndt->setResolution(ndt_resolution);
    ndt->setTransformationEpsilon(0.01);
    // ndt->setTransformationEpsilon(1e-6);
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    if (ndt_num_threads > 0) {ndt->setNumThreads(ndt_num_threads);}
    registration_ = ndt;
  } else if (registration_method == "GICP") {
    boost::shared_ptr<pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>>
    gicp(new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    gicp->setMaxCorrespondenceDistance(30);
    gicp->setMaximumIterations(100);
    // gicp->setCorrespondenceRandomness(20);
    gicp->setTransformationEpsilon(1e-8);
    gicp->setEuclideanFitnessEpsilon(1e-6);
    gicp->setRANSACIterations(0);
    registration_ = gicp;
  } else {
    RCLCPP_ERROR(get_logger(), "invalid registration_method");
    exit(1);
  }

  bev_descriptor_db_.configure(
    bev_descriptor_grid_size_m_,
    bev_descriptor_grid_cells_,
    bev_descriptor_yaw_bins_);

  initializePubSub();

  map_save_srv_ = create_service<std_srvs::srv::Empty>(
    "map_save",
    std::bind(
      &GraphBasedSlamComponent::handleMapSaveRequest,
      this,
      std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3));
}  // NOLINT(readability/fn_size)

void GraphBasedSlamComponent::initializePubSub()
{
  RCLCPP_INFO(get_logger(), "initialize Publishers and Subscribers");

  auto map_array_callback =
    [this](const typename lidarslam_msgs::msg::MapArray::SharedPtr msg_ptr) -> void
    {
      std::lock_guard<std::mutex> lock(mtx_);
      map_array_msg_ = *msg_ptr;
      // Save new submaps to PCD and clear cloud from memory
      if (use_pcd_cache_) {
        for (int i = 0; i < static_cast<int>(map_array_msg_.submaps.size()); i++) {
          auto & sub = map_array_msg_.submaps[i];
          if (sub.cloud.data.size() > 0) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(sub.cloud, *cloud);
            if (cloud->size() > 0) {
              saveSubmapToPCD(i, cloud);
              sub.cloud = sensor_msgs::msg::PointCloud2();  // Free memory
            }
          }
        }
      }
      initial_map_array_received_ = true;
      is_map_array_updated_ = true;
    };

  map_array_sub_ =
    create_subscription<lidarslam_msgs::msg::MapArray>(
    "map_array", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(), map_array_callback);

  if (use_odom_input_) {
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom_input", 10,
      std::bind(&GraphBasedSlamComponent::receiveOdometry, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud_input", rclcpp::SensorDataQoS(),
      std::bind(&GraphBasedSlamComponent::receiveCloud, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Direct odom+cloud input mode enabled");
  }

  std::chrono::milliseconds period(loop_detection_period_);
  loop_detect_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&GraphBasedSlamComponent::searchLoop, this)
  );

  modified_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "modified_map",
    rclcpp::QoS(10));

  modified_map_array_pub_ = create_publisher<lidarslam_msgs::msg::MapArray>(
    "modified_map_array", rclcpp::QoS(10));

  modified_path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "modified_path",
    rclcpp::QoS(10));

  if (use_imu_preintegration_) {
    auto imu_callback =
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) -> void
      {
        receiveImu(*msg);
      };
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu", rclcpp::SensorDataQoS(), imu_callback);
    RCLCPP_INFO(get_logger(), "IMU preintegration enabled, subscribed to /imu");
  }

  if (use_gnss_) {
    gnss_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      gnss_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {receiveNavSatFix(*msg);});
    RCLCPP_INFO(
      get_logger(),
      "GNSS constraints enabled, subscribed to %s",
      gnss_topic_.c_str());
  }

  RCLCPP_INFO(get_logger(), "initialization end");
}

void GraphBasedSlamComponent::handleMapSaveRequest(
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
  if (!snapshotGraphState(map_array_msg, loop_edges, false)) {
    std::cout << "initial map is not received" << std::endl;
    return;
  }
  doPoseAdjustment(map_array_msg, loop_edges, true);
}

bool GraphBasedSlamComponent::snapshotGraphState(
  lidarslam_msgs::msg::MapArray & map_array_msg,
  LoopEdges & loop_edges,
  bool consume_map_update)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!initial_map_array_received_) {
    return false;
  }
  if (consume_map_update && !is_map_array_updated_) {
    return false;
  }

  map_array_msg = map_array_msg_;
  loop_edges = loop_edges_;
  if (consume_map_update) {
    is_map_array_updated_ = false;
  }
  return true;
}

void GraphBasedSlamComponent::snapshotLoopEdges(LoopEdges & loop_edges)
{
  std::lock_guard<std::mutex> lock(mtx_);
  loop_edges = loop_edges_;
}

bool GraphBasedSlamComponent::upsertLoopEdge(const LoopEdge & loop_edge)
{
  if (loop_edge.pair_id.first < 0 || loop_edge.pair_id.second < 0) {
    return false;
  }

  LoopEdge normalized = loop_edge;
  if (normalized.pair_id.first > normalized.pair_id.second) {
    std::swap(normalized.pair_id.first, normalized.pair_id.second);
    normalized.relative_pose = normalized.relative_pose.inverse();
  }
  if (normalized.pair_id.first == normalized.pair_id.second) {
    return false;
  }

  std::lock_guard<std::mutex> lock(mtx_);
  auto is_nearby_pair = [this](const LoopEdge & lhs, const LoopEdge & rhs) {
      return std::abs(lhs.pair_id.first - rhs.pair_id.first) <= loop_edge_dedup_index_window_ &&
             std::abs(lhs.pair_id.second - rhs.pair_id.second) <= loop_edge_dedup_index_window_;
    };
  for (auto & existing : loop_edges_) {
    if (!is_nearby_pair(existing, normalized)) {
      continue;
    }
    if (existing.fitness_score > 0.0 &&
      normalized.fitness_score >= existing.fitness_score)
    {
      return false;
    }
    existing = normalized;
    return true;
  }

  loop_edges_.push_back(normalized);
  return true;
}
void GraphBasedSlamComponent::searchLoop()
{
  lidarslam_msgs::msg::MapArray map_array_msg;
  LoopEdges loop_edges;
  if (!snapshotGraphState(map_array_msg, loop_edges, true)) {return;}
  if (map_array_msg.submaps.size() < 2) {return;}
  if (map_array_msg.cloud_coordinate != map_array_msg.LOCAL) {
    RCLCPP_WARN(get_logger(), "cloud_coordinate should be local, but it's not local.");
  }
  int num_submaps = map_array_msg.submaps.size();

  if (debug_flag_) {
    RCLCPP_INFO(get_logger(), "searching Loop, num_submaps:%d", num_submaps);
  }

  struct LoopCandidate
  {
    enum class Source
    {
      DISTANCE,
      SCAN_CONTEXT,
      BEV_DESCRIPTOR,
      SOLID_DESCRIPTOR
    };

    int index {-1};
    double selection_metric {std::numeric_limits<double>::max()};
    Source source {Source::DISTANCE};
    double yaw_rad {0.0};
  };

  struct LoopCandidateResult
  {
    bool valid {false};
    int index {-1};
    double selection_metric {std::numeric_limits<double>::max()};
    double fitness_score {std::numeric_limits<double>::max()};
    double travel_distance {0.0};
    double euclidean_distance {0.0};
    double translation_delta_m {0.0};
    double rotation_delta_deg {0.0};
    LoopCandidate::Source source {LoopCandidate::Source::DISTANCE};
    bool used_3d_bbs {false};
    double three_d_bbs_score_percentage {0.0};
    double three_d_bbs_elapsed_msec {0.0};
    Eigen::Matrix4f final_transformation {Eigen::Matrix4f::Identity()};
  };

  const auto candidate_source_name =
    [](LoopCandidate::Source source) -> const char * {
      switch (source) {
        case LoopCandidate::Source::SCAN_CONTEXT:
          return "scan_context";
        case LoopCandidate::Source::BEV_DESCRIPTOR:
          return "bev_descriptor";
        case LoopCandidate::Source::SOLID_DESCRIPTOR:
          return "solid_descriptor";
        case LoopCandidate::Source::DISTANCE:
        default:
          return "distance";
      }
    };

  const auto build_filtered_local_submap =
    [this, &map_array_msg](int ref_idx) -> pcl::PointCloud<pcl::PointXYZI>::Ptr {
      pcl::PointCloud<pcl::PointXYZI>::Ptr aggregated_cloud(
        new pcl::PointCloud<pcl::PointXYZI>);
      Eigen::Affine3d reference_affine;
      tf2::fromMsg(map_array_msg.submaps[ref_idx].pose, reference_affine);
      for (int k = 0; k < search_submap_num_ && (ref_idx - k) >= 0; ++k) {
        const int src_idx = ref_idx - k;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
        if (use_pcd_cache_) {
          cloud = loadSubmapFromPCD(src_idx);
        } else {
          cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
          pcl::fromROSMsg(map_array_msg.submaps[src_idx].cloud, *cloud);
        }
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
      voxelgrid_.setInputCloud(aggregated_cloud);
      voxelgrid_.filter(*filtered_cloud);
      return filtered_cloud;
    };

  // Keep Scan Context database aligned 1:1 with submap indices.
  if (use_scan_context_ && scan_context_db_.nextSubmapIndex() < num_submaps) {
    for (int idx = scan_context_db_.nextSubmapIndex(); idx < num_submaps; ++idx) {
      const auto filtered_aggregated_cloud = build_filtered_local_submap(idx);
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

  if (use_bev_descriptor_ && bev_descriptor_db_.nextSubmapIndex() < num_submaps) {
    for (int idx = bev_descriptor_db_.nextSubmapIndex(); idx < num_submaps; ++idx) {
      const auto filtered_aggregated_cloud = build_filtered_local_submap(idx);
      bev_descriptor_db_.add(
        idx,
        SubmapBEVDescriptor::computeDescriptor(
          filtered_aggregated_cloud,
          bev_descriptor_grid_size_m_,
          bev_descriptor_grid_cells_));
    }
  }
  if (use_solid_descriptor_ && solid_descriptor_db_.nextSubmapIndex() < num_submaps) {
    for (int idx = solid_descriptor_db_.nextSubmapIndex(); idx < num_submaps; ++idx) {
      const auto filtered_aggregated_cloud = build_filtered_local_submap(idx);
      solid_descriptor_db_.add(
        idx,
        SolidDescriptor::computeDescriptor(filtered_aggregated_cloud));
    }
  }

  const int latest_idx = num_submaps - 1;
  const auto & latest_submap = map_array_msg.submaps[latest_idx];
  Eigen::Affine3d latest_affine;
  tf2::fromMsg(latest_submap.pose, latest_affine);

  // Aggregate latest N submaps as source (improves matching quality)
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_latest_submap_cloud_ptr(
    new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_latest_submap_cloud_sc_ptr(
    new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr latest_submap_cloud_local_ptr(
    new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr latest_submap_cloud_local_bbs_ptr(
    new pcl::PointCloud<pcl::PointXYZI>);
  for (int k = 0; k < search_submap_num_ && (latest_idx - k) >= 0; k++) {
    int src_idx = latest_idx - k;
    const auto & src_submap = map_array_msg.submaps[src_idx];
    pcl::PointCloud<pcl::PointXYZI>::Ptr src_cloud;
    if (use_pcd_cache_) {
      src_cloud = loadSubmapFromPCD(src_idx);
    } else {
      src_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(src_submap.cloud, *src_cloud);
    }
    if (src_cloud->empty()) {
      continue;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_src(new pcl::PointCloud<pcl::PointXYZI>);
    Eigen::Affine3d src_affine;
    tf2::fromMsg(src_submap.pose, src_affine);
    pcl::transformPointCloud(*src_cloud, *transformed_src, src_affine.matrix().cast<float>());
    *transformed_latest_submap_cloud_ptr += *transformed_src;
    if (k < three_d_bbs_source_submap_num_) {
      *transformed_latest_submap_cloud_sc_ptr += *transformed_src;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_src_local(
      new pcl::PointCloud<pcl::PointXYZI>);
    const Eigen::Matrix4f latest_frame_transform =
      (latest_affine.inverse() * src_affine).matrix().cast<float>();
    pcl::transformPointCloud(*src_cloud, *transformed_src_local, latest_frame_transform);
    *latest_submap_cloud_local_ptr += *transformed_src_local;
    if (k < three_d_bbs_source_submap_num_) {
      *latest_submap_cloud_local_bbs_ptr += *transformed_src_local;
    }
  }
  if (
    transformed_latest_submap_cloud_ptr->empty() ||
    transformed_latest_submap_cloud_sc_ptr->empty() ||
    latest_submap_cloud_local_ptr->empty() ||
    latest_submap_cloud_local_bbs_ptr->empty())
  {
    return;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_source(new pcl::PointCloud<pcl::PointXYZI>);
  voxelgrid_.setInputCloud(transformed_latest_submap_cloud_ptr);
  voxelgrid_.filter(*filtered_source);
  if (filtered_source->empty()) {
    return;
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_source_sc(new pcl::PointCloud<pcl::PointXYZI>);
  voxelgrid_.setInputCloud(transformed_latest_submap_cloud_sc_ptr);
  voxelgrid_.filter(*filtered_source_sc);
  if (filtered_source_sc->empty()) {
    return;
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_source_local(new pcl::PointCloud<pcl::PointXYZI>);
  voxelgrid_.setInputCloud(latest_submap_cloud_local_ptr);
  voxelgrid_.filter(*filtered_source_local);
  if (filtered_source_local->empty()) {
    return;
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_source_local_bbs(
    new pcl::PointCloud<pcl::PointXYZI>);
  voxelgrid_.setInputCloud(latest_submap_cloud_local_bbs_ptr);
  voxelgrid_.filter(*filtered_source_local_bbs);
  if (filtered_source_local_bbs->empty()) {
    return;
  }
  registration_->setInputSource(filtered_source);

  const double latest_moving_distance = latest_submap.distance;
  const Eigen::Vector3d latest_submap_pos{
    latest_submap.pose.position.x,
    latest_submap.pose.position.y,
    latest_submap.pose.position.z};

  std::vector<LoopCandidate> candidates;
  auto add_candidate =
    [&candidates](
    int index,
    double selection_metric,
    LoopCandidate::Source source,
    double yaw_rad = 0.0)
    {
      if (index < 0) {
        return;
      }
      for (auto & candidate : candidates) {
        if (candidate.index != index || candidate.source != source) {
          continue;
        }
        candidate.selection_metric = std::min(candidate.selection_metric, selection_metric);
        candidate.yaw_rad = yaw_rad;
        return;
      }

      LoopCandidate candidate;
      candidate.index = index;
      candidate.selection_metric = selection_metric;
      candidate.source = source;
      candidate.yaw_rad = yaw_rad;
      candidates.push_back(candidate);
    };

  struct DescriptorRerankHint
  {
    double score = std::numeric_limits<double>::max();
    double yaw_rad = 0.0;
  };

  std::vector<std::pair<double, int>> distance_candidates;
  distance_candidates.reserve(num_submaps);
  for (int i = 0; i < latest_idx; i++) {
    const auto & submap = map_array_msg.submaps[i];
    const Eigen::Vector3d submap_pos{
      submap.pose.position.x,
      submap.pose.position.y,
      submap.pose.position.z};
    const double dist = (latest_submap_pos - submap_pos).norm();
    if (latest_moving_distance - submap.distance <= distance_loop_closure_) {
      continue;
    }
    if (dist >= range_of_searching_loop_closure_) {
      continue;
    }
    distance_candidates.emplace_back(dist, i);
  }
  std::sort(distance_candidates.begin(), distance_candidates.end());

  if (use_scan_context_ && scan_context_db_.size() > ScanContext::EXCLUDE_RECENT) {
    const auto sc_matches = scan_context_db_.queryTopMatchesWithYaw(
      scan_context_db_.descriptors.back(),
      max_loop_candidate_count_,
      ScanContext::NUM_CANDIDATES,
      ScanContext::EXCLUDE_RECENT,
      scan_context_threshold_);

    if (!sc_matches.empty()) {
      bool added_scan_context_candidate = false;
      for (const auto & sc_match : sc_matches) {
        const int sc_idx = sc_match.submap_id;
        const double sc_dist = sc_match.distance;
        if (sc_idx < 0 || sc_idx >= latest_idx) {
          continue;
        }
        const double sc_travel_distance =
          latest_moving_distance - map_array_msg.submaps[sc_idx].distance;
        if (sc_travel_distance <= distance_loop_closure_) {
          if (debug_flag_) {
            RCLCPP_INFO(
              get_logger(),
              "Skip ScanContext candidate %d because travel distance %.3f m is below %.3f m",
              sc_idx,
              sc_travel_distance,
              distance_loop_closure_);
          }
          continue;
        }
        double sc_yaw_rad =
          -static_cast<double>(sc_match.yaw_shift) * 2.0 * M_PI / ScanContext::NUM_SECTORS;
        while (sc_yaw_rad > M_PI) {
          sc_yaw_rad -= 2.0 * M_PI;
        }
        while (sc_yaw_rad < -M_PI) {
          sc_yaw_rad += 2.0 * M_PI;
        }
        add_candidate(sc_idx, sc_dist, LoopCandidate::Source::SCAN_CONTEXT, sc_yaw_rad);
        std::cout << "ScanContext loop candidate: id=" << sc_idx
                  << " sc_dist=" << sc_dist
                  << " yaw_deg=" << sc_yaw_rad * 180.0 / M_PI << std::endl;
        added_scan_context_candidate = true;
        break;
      }
      if (!added_scan_context_candidate && debug_flag_) {
        std::cout << "ScanContext matches exist but none satisfied travel-distance gating"
                  << std::endl;
      }
    } else if (debug_flag_) {
      auto [sc_idx, sc_dist] = scan_context_db_.query(
        scan_context_db_.descriptors.back(),
        ScanContext::NUM_CANDIDATES,
        ScanContext::EXCLUDE_RECENT,
        std::numeric_limits<double>::max());
      static_cast<void>(sc_idx);
      std::cout << "ScanContext no match: best_sc_dist=" << sc_dist
                << " threshold=" << scan_context_threshold_ << std::endl;
    }
  }

  if (use_bev_descriptor_ &&
    bev_descriptor_db_.size() > SubmapBEVDescriptor::DEFAULT_EXCLUDE_RECENT)
  {
    std::unordered_map<int, DescriptorRerankHint> bev_rerank_hints;
    const int bev_rerank_candidates = std::min(
      std::max(max_loop_candidate_count_ * 4, max_loop_candidate_count_),
      static_cast<int>(distance_candidates.size()));
    bool added_bev_candidate = false;
    double best_bev_dist = std::numeric_limits<double>::max();
    int best_bev_idx = -1;
    for (int i = 0; i < bev_rerank_candidates; ++i) {
      const int bev_idx = distance_candidates[i].second;
      if (bev_idx < 0 || bev_idx >= latest_idx || bev_idx >= bev_descriptor_db_.size()) {
        continue;
      }
      const auto & bev_submap = map_array_msg.submaps[bev_idx];
      const Eigen::Vector3d bev_submap_pos(
        bev_submap.pose.position.x,
        bev_submap.pose.position.y,
        bev_submap.pose.position.z);
      const double bev_euclidean_distance = (latest_submap_pos - bev_submap_pos).norm();
      if (
        bev_descriptor_max_euclidean_distance_m_ > 0.0 &&
        bev_euclidean_distance > bev_descriptor_max_euclidean_distance_m_)
      {
        if (debug_flag_) {
          RCLCPP_INFO(
            get_logger(),
            "Skip BEV candidate %d because euclidean distance %.3f m exceeds %.3f m",
            bev_idx,
            bev_euclidean_distance,
            bev_descriptor_max_euclidean_distance_m_);
        }
        continue;
      }

      const auto bev_match = SubmapBEVDescriptor::distanceWithAlignment(
        bev_descriptor_db_.descriptors.back(),
        bev_descriptor_db_.descriptors[bev_idx],
        bev_idx,
        bev_descriptor_yaw_bins_);
      if (bev_match.distance < best_bev_dist) {
        best_bev_dist = bev_match.distance;
        best_bev_idx = bev_idx;
      }
      if (bev_match.distance >= bev_descriptor_threshold_) {
        if (debug_flag_) {
          RCLCPP_INFO(
            get_logger(),
            "Skip BEV candidate %d because descriptor distance %.3f exceeds %.3f",
            bev_idx,
            bev_match.distance,
            bev_descriptor_threshold_);
        }
        continue;
      }

      double bev_yaw_rad = bev_match.yaw_rad;
      while (bev_yaw_rad > M_PI) {
        bev_yaw_rad -= 2.0 * M_PI;
      }
      while (bev_yaw_rad < -M_PI) {
        bev_yaw_rad += 2.0 * M_PI;
      }
      double bev_sequence_metric = bev_match.distance;
      if (bev_descriptor_sequence_window_ > 0) {
        double bev_sequence_distance_sum = bev_match.distance;
        int bev_sequence_count = 1;
        for (int offset = 1; offset <= bev_descriptor_sequence_window_; ++offset) {
          const int query_idx = latest_idx - offset;
          const int candidate_sequence_idx = bev_idx - offset;
          if (
            query_idx < 0 || candidate_sequence_idx < 0 ||
            query_idx >= bev_descriptor_db_.size() ||
            candidate_sequence_idx >= bev_descriptor_db_.size())
          {
            break;
          }
          const auto rotated_candidate_descriptor = SubmapBEVDescriptor::rotateDescriptor(
            bev_descriptor_db_.descriptors[candidate_sequence_idx],
            bev_yaw_rad);
          bev_sequence_distance_sum += SubmapBEVDescriptor::descriptorDistance(
            bev_descriptor_db_.descriptors[query_idx],
            rotated_candidate_descriptor);
          ++bev_sequence_count;
        }
        bev_sequence_metric = bev_sequence_distance_sum / static_cast<double>(bev_sequence_count);
      }
      if (bev_sequence_metric >= bev_descriptor_sequence_threshold_) {
        if (debug_flag_) {
          RCLCPP_INFO(
            get_logger(),
            "Skip BEV candidate %d because sequence metric %.3f exceeds %.3f",
            bev_idx,
            bev_sequence_metric,
            bev_descriptor_sequence_threshold_);
        }
        continue;
      }
      double bev_pose_consistency_metric = -1.0;
      if (
        bev_descriptor_pose_consistency_threshold_m_ > 0.0 &&
        bev_descriptor_sequence_window_ > 0)
      {
        Eigen::Affine3d bev_candidate_affine;
        tf2::fromMsg(map_array_msg.submaps[bev_idx].pose, bev_candidate_affine);
        const Eigen::AngleAxisd yaw_correction(bev_yaw_rad, Eigen::Vector3d::UnitZ());
        double bev_pose_consistency_sum = 0.0;
        int bev_pose_consistency_count = 0;
        for (int offset = 1; offset <= bev_descriptor_sequence_window_; ++offset) {
          const int query_idx = latest_idx - offset;
          const int candidate_sequence_idx = bev_idx - offset;
          if (query_idx < 0 || candidate_sequence_idx < 0) {
            break;
          }

          Eigen::Affine3d query_prev_affine;
          Eigen::Affine3d candidate_prev_affine;
          tf2::fromMsg(map_array_msg.submaps[query_idx].pose, query_prev_affine);
          tf2::fromMsg(map_array_msg.submaps[candidate_sequence_idx].pose, candidate_prev_affine);

          const Eigen::Vector3d query_delta =
            (latest_affine.inverse() * query_prev_affine).translation();
          const Eigen::Vector3d candidate_delta =
            yaw_correction * (bev_candidate_affine.inverse() * candidate_prev_affine).translation();
          bev_pose_consistency_sum +=
            (query_delta.head<2>() - candidate_delta.head<2>()).norm();
          ++bev_pose_consistency_count;
        }
        if (bev_pose_consistency_count > 0) {
          bev_pose_consistency_metric =
            bev_pose_consistency_sum / static_cast<double>(bev_pose_consistency_count);
          if (bev_pose_consistency_metric >= bev_descriptor_pose_consistency_threshold_m_) {
            if (debug_flag_) {
              RCLCPP_INFO(
                get_logger(),
                "Skip BEV candidate %d because pose consistency %.3f m exceeds %.3f m",
                bev_idx,
                bev_pose_consistency_metric,
                bev_descriptor_pose_consistency_threshold_m_);
            }
            continue;
          }
        }
      }
      auto & bev_hint = bev_rerank_hints[bev_idx];
      if (bev_sequence_metric < bev_hint.score) {
        bev_hint.score = bev_sequence_metric;
        bev_hint.yaw_rad = bev_yaw_rad;
      }
      std::cout << "BEV rerank hint: id=" << bev_idx
                << " bev_dist=" << bev_match.distance
                << " seq_dist=" << bev_sequence_metric
                << " pose_seq_m=" << bev_pose_consistency_metric
                << " yaw_deg=" << bev_yaw_rad * 180.0 / M_PI << std::endl;
      added_bev_candidate = true;
    }
    if (!added_bev_candidate && debug_flag_) {
      std::cout << "BEV rerank no candidate: best_idx=" << best_bev_idx
                << " best_bev_dist=" << best_bev_dist
                << " threshold=" << bev_descriptor_threshold_ << std::endl;
    }

    auto bev_adjusted_distance =
      [this, &bev_rerank_hints](const std::pair<double, int> & candidate) {
        const auto bev_hint = bev_rerank_hints.find(candidate.second);
        if (bev_hint == bev_rerank_hints.end()) {
          return candidate.first;
        }
        return candidate.first +
               bev_descriptor_rerank_weight_m_ *
               (bev_hint->second.score - bev_descriptor_threshold_);
      };

    std::stable_sort(
      distance_candidates.begin(),
      distance_candidates.end(),
      [&bev_adjusted_distance](const auto & lhs, const auto & rhs) {
        const double lhs_adjusted = bev_adjusted_distance(lhs);
        const double rhs_adjusted = bev_adjusted_distance(rhs);
        if (lhs_adjusted != rhs_adjusted) {
          return lhs_adjusted < rhs_adjusted;
        }
        return lhs.first < rhs.first;
      });

    const int num_distance_candidates =
      std::min(max_loop_candidate_count_, static_cast<int>(distance_candidates.size()));
    for (int i = 0; i < num_distance_candidates; ++i) {
      const int candidate_idx = distance_candidates[i].second;
      const auto bev_hint = bev_rerank_hints.find(candidate_idx);
      const double adjusted_distance = bev_adjusted_distance(distance_candidates[i]);
      if (bev_hint != bev_rerank_hints.end()) {
        add_candidate(
          candidate_idx,
          adjusted_distance,
          LoopCandidate::Source::DISTANCE,
          bev_hint->second.yaw_rad);
        std::cout << "Distance candidate reranked by BEV: id=" << candidate_idx
                  << " dist_m=" << distance_candidates[i].first
                  << " bev_score=" << bev_hint->second.score
                  << " adjusted_dist_m=" << adjusted_distance
                  << " yaw_deg=" << bev_hint->second.yaw_rad * 180.0 / M_PI << std::endl;
      } else {
        add_candidate(
          candidate_idx,
          adjusted_distance,
          LoopCandidate::Source::DISTANCE);
      }
    }
  } else {
    const int num_distance_candidates =
      std::min(max_loop_candidate_count_, static_cast<int>(distance_candidates.size()));
    for (int i = 0; i < num_distance_candidates; i++) {
      add_candidate(
        distance_candidates[i].second,
        distance_candidates[i].first,
        LoopCandidate::Source::DISTANCE);
    }
  }
  if (
    use_solid_descriptor_ &&
    solid_descriptor_db_.size() > SolidDescriptor::DEFAULT_EXCLUDE_RECENT)
  {
    const int solid_rerank_candidates = std::min(
      std::max(max_loop_candidate_count_ * 4, max_loop_candidate_count_),
      static_cast<int>(distance_candidates.size()));
    bool added_solid_candidate = false;
    double best_solid_similarity = -1.0;
    int best_solid_idx = -1;
    for (int i = 0; i < solid_rerank_candidates; ++i) {
      const int solid_idx = distance_candidates[i].second;
      if (solid_idx < 0 || solid_idx >= latest_idx || solid_idx >= solid_descriptor_db_.size()) {
        continue;
      }
      const auto & solid_submap = map_array_msg.submaps[solid_idx];
      const Eigen::Vector3d solid_submap_pos(
        solid_submap.pose.position.x,
        solid_submap.pose.position.y,
        solid_submap.pose.position.z);
      const double solid_euclidean_distance = (latest_submap_pos - solid_submap_pos).norm();
      if (
        solid_descriptor_max_euclidean_distance_m_ > 0.0 &&
        solid_euclidean_distance > solid_descriptor_max_euclidean_distance_m_)
      {
        if (debug_flag_) {
          RCLCPP_INFO(
            get_logger(),
            "Skip SOLiD candidate %d because euclidean distance %.3f m exceeds %.3f m",
            solid_idx,
            solid_euclidean_distance,
            solid_descriptor_max_euclidean_distance_m_);
        }
        continue;
      }

      const double solid_similarity = SolidDescriptor::loopSimilarity(
        solid_descriptor_db_.descriptors.back(),
        solid_descriptor_db_.descriptors[solid_idx]);
      if (solid_similarity > best_solid_similarity) {
        best_solid_similarity = solid_similarity;
        best_solid_idx = solid_idx;
      }
      if (solid_similarity < solid_descriptor_min_similarity_) {
        if (debug_flag_) {
          RCLCPP_INFO(
            get_logger(),
            "Skip SOLiD candidate %d because similarity %.3f is below %.3f",
            solid_idx,
            solid_similarity,
            solid_descriptor_min_similarity_);
        }
        continue;
      }

      double solid_yaw_rad = SolidDescriptor::poseYawRad(
        solid_descriptor_db_.descriptors.back(),
        solid_descriptor_db_.descriptors[solid_idx]);
      while (solid_yaw_rad > M_PI) {
        solid_yaw_rad -= 2.0 * M_PI;
      }
      while (solid_yaw_rad < -M_PI) {
        solid_yaw_rad += 2.0 * M_PI;
      }

      double solid_sequence_similarity = solid_similarity;
      if (solid_descriptor_sequence_window_ > 0) {
        double solid_sequence_similarity_sum = solid_similarity;
        int solid_sequence_count = 1;
        for (int offset = 1; offset <= solid_descriptor_sequence_window_; ++offset) {
          const int query_idx = latest_idx - offset;
          const int candidate_sequence_idx = solid_idx - offset;
          if (
            query_idx < 0 || candidate_sequence_idx < 0 ||
            query_idx >= solid_descriptor_db_.size() ||
            candidate_sequence_idx >= solid_descriptor_db_.size())
          {
            break;
          }
          solid_sequence_similarity_sum += SolidDescriptor::loopSimilarity(
            solid_descriptor_db_.descriptors[query_idx],
            solid_descriptor_db_.descriptors[candidate_sequence_idx]);
          ++solid_sequence_count;
        }
        solid_sequence_similarity =
          solid_sequence_similarity_sum / static_cast<double>(solid_sequence_count);
      }
      if (solid_sequence_similarity < solid_descriptor_sequence_min_similarity_) {
        if (debug_flag_) {
          RCLCPP_INFO(
            get_logger(),
            "Skip SOLiD candidate %d because sequence similarity %.3f is below %.3f",
            solid_idx,
            solid_sequence_similarity,
            solid_descriptor_sequence_min_similarity_);
        }
        continue;
      }

      double solid_pose_consistency_metric = -1.0;
      if (
        solid_descriptor_pose_consistency_threshold_m_ > 0.0 &&
        solid_descriptor_sequence_window_ > 0)
      {
        Eigen::Affine3d solid_candidate_affine;
        tf2::fromMsg(map_array_msg.submaps[solid_idx].pose, solid_candidate_affine);
        const Eigen::AngleAxisd yaw_correction(solid_yaw_rad, Eigen::Vector3d::UnitZ());
        double solid_pose_consistency_sum = 0.0;
        int solid_pose_consistency_count = 0;
        for (int offset = 1; offset <= solid_descriptor_sequence_window_; ++offset) {
          const int query_idx = latest_idx - offset;
          const int candidate_sequence_idx = solid_idx - offset;
          if (query_idx < 0 || candidate_sequence_idx < 0) {
            break;
          }

          Eigen::Affine3d query_prev_affine;
          Eigen::Affine3d candidate_prev_affine;
          tf2::fromMsg(map_array_msg.submaps[query_idx].pose, query_prev_affine);
          tf2::fromMsg(map_array_msg.submaps[candidate_sequence_idx].pose, candidate_prev_affine);

          const Eigen::Vector3d query_delta =
            (latest_affine.inverse() * query_prev_affine).translation();
          const Eigen::Vector3d candidate_delta =
            yaw_correction *
            (solid_candidate_affine.inverse() * candidate_prev_affine).translation();
          solid_pose_consistency_sum +=
            (query_delta.head<2>() - candidate_delta.head<2>()).norm();
          ++solid_pose_consistency_count;
        }
        if (solid_pose_consistency_count > 0) {
          solid_pose_consistency_metric =
            solid_pose_consistency_sum / static_cast<double>(solid_pose_consistency_count);
          if (
            solid_pose_consistency_metric >=
            solid_descriptor_pose_consistency_threshold_m_)
          {
            if (debug_flag_) {
              RCLCPP_INFO(
                get_logger(),
                "Skip SOLiD candidate %d because pose consistency %.3f m exceeds %.3f m",
                solid_idx,
                solid_pose_consistency_metric,
                solid_descriptor_pose_consistency_threshold_m_);
            }
            continue;
          }
        }
      }

      add_candidate(
        solid_idx,
        1.0 - solid_sequence_similarity,
        LoopCandidate::Source::SOLID_DESCRIPTOR,
        solid_yaw_rad);
      std::cout << "SOLiD rerank candidate: id=" << solid_idx
                << " solid_sim=" << solid_similarity
                << " seq_sim=" << solid_sequence_similarity
                << " pose_seq_m=" << solid_pose_consistency_metric
                << " yaw_deg=" << solid_yaw_rad * 180.0 / M_PI << std::endl;
      added_solid_candidate = true;
    }
    if (!added_solid_candidate && debug_flag_) {
      std::cout << "SOLiD rerank no candidate: best_idx=" << best_solid_idx
                << " best_similarity=" << best_solid_similarity
                << " threshold=" << solid_descriptor_min_similarity_ << std::endl;
    }
  }
  if (candidates.empty()) {
    return;
  }

  LoopCandidateResult best_candidate;
  LoopCandidateResult best_scan_context_candidate;
  LoopCandidateResult best_attempt;
  bool attempted_registration = false;

  for (const auto & candidate : candidates) {
    if (candidate.index < 0 || candidate.index >= latest_idx) {
      continue;
    }

    const auto & candidate_submap = map_array_msg.submaps[candidate.index];
    Eigen::Affine3d candidate_affine;
    tf2::fromMsg(candidate_submap.pose, candidate_affine);
    pcl::PointCloud<pcl::PointXYZI>::Ptr submap_clouds_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr submap_clouds_bbs_ptr(
      new pcl::PointCloud<pcl::PointXYZI>);
    for (int offset = -search_submap_num_; offset <= search_submap_num_; ++offset) {
      const int near_idx = candidate.index + offset;
      if (near_idx < 0 || near_idx >= num_submaps) {
        continue;
      }
      const auto & near_submap = map_array_msg.submaps[near_idx];
      pcl::PointCloud<pcl::PointXYZI>::Ptr submap_cloud_ptr;
      if (use_pcd_cache_) {
        submap_cloud_ptr = loadSubmapFromPCD(near_idx);
      } else {
        submap_cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(near_submap.cloud, *submap_cloud_ptr);
      }
      if (submap_cloud_ptr->empty()) {
        continue;
      }
      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_submap_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZI>);
      Eigen::Affine3d affine;
      tf2::fromMsg(near_submap.pose, affine);
      pcl::transformPointCloud(
        *submap_cloud_ptr, *transformed_submap_cloud_ptr,
        affine.matrix().cast<float>());
      *submap_clouds_ptr += *transformed_submap_cloud_ptr;
      if (std::abs(offset) <= three_d_bbs_target_submap_radius_) {
        *submap_clouds_bbs_ptr += *transformed_submap_cloud_ptr;
      }
    }
    if (submap_clouds_ptr->empty() || submap_clouds_bbs_ptr->empty()) {
      continue;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_clouds_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    voxelgrid_.setInputCloud(submap_clouds_ptr);
    voxelgrid_.filter(*filtered_clouds_ptr);
    if (filtered_clouds_ptr->empty()) {
      continue;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_clouds_sc_ptr(
      new pcl::PointCloud<pcl::PointXYZI>());
    voxelgrid_.setInputCloud(submap_clouds_bbs_ptr);
    voxelgrid_.filter(*filtered_clouds_sc_ptr);
    if (filtered_clouds_sc_ptr->empty()) {
      continue;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    bool used_3d_bbs = false;
    double three_d_bbs_score_percentage = 0.0;
    double three_d_bbs_elapsed_msec = 0.0;
    Eigen::Matrix4f initial_guess =
      (candidate_affine.matrix() * latest_affine.inverse().matrix()).cast<float>();
    if (std::abs(candidate.yaw_rad) > 1e-6) {
      Eigen::Affine3d yaw_correction = Eigen::Affine3d::Identity();
      yaw_correction.rotate(Eigen::AngleAxisd(candidate.yaw_rad, Eigen::Vector3d::UnitZ()));
      initial_guess =
        (candidate_affine.matrix() * yaw_correction.matrix() * latest_affine.inverse().matrix()).
        cast<float>();
    }
    if (candidate.source == LoopCandidate::Source::SCAN_CONTEXT) {
      registration_->setInputSource(filtered_source_sc);
      registration_->setInputTarget(filtered_clouds_sc_ptr);
    } else {
      registration_->setInputSource(filtered_source);
      registration_->setInputTarget(filtered_clouds_ptr);
    }
    if (candidate.source == LoopCandidate::Source::SCAN_CONTEXT && use_3d_bbs_for_scan_context_) {
      pcl::VoxelGrid<pcl::PointXYZI> three_d_bbs_voxelgrid;
      three_d_bbs_voxelgrid.setLeafSize(
        three_d_bbs_voxel_leaf_size_,
        three_d_bbs_voxel_leaf_size_,
        three_d_bbs_voxel_leaf_size_);
      pcl::PointCloud<pcl::PointXYZI>::Ptr three_d_bbs_source(
        new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr three_d_bbs_target(
        new pcl::PointCloud<pcl::PointXYZI>);
      three_d_bbs_voxelgrid.setInputCloud(filtered_source_local_bbs);
      three_d_bbs_voxelgrid.filter(*three_d_bbs_source);
      three_d_bbs_voxelgrid.setInputCloud(submap_clouds_bbs_ptr);
      three_d_bbs_voxelgrid.filter(*three_d_bbs_target);

      ThreeDBBSLoopVerifierConfig bbs_config;
      bbs_config.min_level_res = three_d_bbs_min_level_res_;
      bbs_config.max_level = three_d_bbs_max_level_;
      bbs_config.score_threshold_percentage = three_d_bbs_score_threshold_percentage_;
      bbs_config.timeout_msec = three_d_bbs_timeout_msec_;
      bbs_config.num_threads = three_d_bbs_num_threads_;
      bbs_config.translation_search_margin_m = three_d_bbs_translation_search_margin_m_;
      bbs_config.roll_pitch_search_deg = three_d_bbs_roll_pitch_search_deg_;
      bbs_config.yaw_search_deg = three_d_bbs_yaw_search_deg_;
      const auto bbs_result = three_d_bbs_loop_verifier_.localize(
        three_d_bbs_source,
        three_d_bbs_target,
        Eigen::Isometry3d(latest_affine.matrix()),
        Eigen::Isometry3d(candidate_affine.matrix()),
        bbs_config);
      if (bbs_result.available) {
        three_d_bbs_score_percentage = bbs_result.score_percentage;
        three_d_bbs_elapsed_msec = bbs_result.elapsed_msec;
        used_3d_bbs = bbs_result.localized;
        if (bbs_result.localized) {
          initial_guess = bbs_result.correction_guess;
        }
        if (debug_flag_) {
          RCLCPP_INFO(
            get_logger(),
            "3D-BBS %s for loop candidate %d -> %d "
            "(score=%.3f elapsed=%.2f ms timed_out=%s "
            "src=%zu tar=%zu)",
            bbs_result.localized ? "localized" : "missed",
            candidate.index,
            latest_idx,
            bbs_result.score_percentage,
            bbs_result.elapsed_msec,
            bbs_result.timed_out ? "true" : "false",
            three_d_bbs_source->size(),
            three_d_bbs_target->size());
        }
      }
    }
    if (candidate.source != LoopCandidate::Source::DISTANCE || used_3d_bbs) {
      registration_->align(*output_cloud_ptr, initial_guess);
    } else {
      registration_->align(*output_cloud_ptr);
    }
    attempted_registration = true;
    if (!registration_->hasConverged()) {
      if (debug_flag_) {
        RCLCPP_INFO(
          get_logger(),
          "Rejected loop candidate %d -> %d because registration did not converge",
          candidate.index,
          latest_idx);
      }
      continue;
    }

    const double fitness_score = registration_->getFitnessScore();
    const Eigen::Matrix4f final_transformation = registration_->getFinalTransformation();
    const Eigen::Vector3f translation = final_transformation.block<3, 1>(0, 3);
    const double translation_delta_m = translation.cast<double>().norm();
    const Eigen::Matrix3f rotation = final_transformation.block<3, 3>(0, 0);
    const double trace = static_cast<double>(rotation.trace());
    const double cos_theta = std::max(-1.0, std::min(1.0, 0.5 * (trace - 1.0)));
    const double rotation_delta_deg = std::acos(cos_theta) * 180.0 / M_PI;

    LoopCandidateResult candidate_result;
    candidate_result.index = candidate.index;
    candidate_result.selection_metric = candidate.selection_metric;
    candidate_result.fitness_score = fitness_score;
    candidate_result.travel_distance = latest_moving_distance - candidate_submap.distance;
    const Eigen::Vector3d candidate_submap_pos(
      candidate_submap.pose.position.x,
      candidate_submap.pose.position.y,
      candidate_submap.pose.position.z);
    candidate_result.euclidean_distance = (latest_submap_pos - candidate_submap_pos).norm();
    candidate_result.translation_delta_m = translation_delta_m;
    candidate_result.rotation_delta_deg = rotation_delta_deg;
    candidate_result.source = candidate.source;
    candidate_result.used_3d_bbs = used_3d_bbs;
    candidate_result.three_d_bbs_score_percentage = three_d_bbs_score_percentage;
    candidate_result.three_d_bbs_elapsed_msec = three_d_bbs_elapsed_msec;
    candidate_result.final_transformation = final_transformation;

    if (best_attempt.index < 0 || fitness_score < best_attempt.fitness_score) {
      best_attempt = candidate_result;
    }

    const double loop_score_threshold =
      (candidate.source == LoopCandidate::Source::SCAN_CONTEXT &&
      scan_context_loop_closure_score_threshold_ > 0.0) ?
      scan_context_loop_closure_score_threshold_ : threshold_loop_closure_score_;

    if (fitness_score >= loop_score_threshold) {
      if (debug_flag_) {
        RCLCPP_INFO(
          get_logger(),
          "Rejected loop candidate %d -> %d because fitness %.6f exceeds threshold %.6f",
          candidate.index,
          latest_idx,
          fitness_score,
          loop_score_threshold);
      }
      continue;
    }
    if (translation_delta_m > loop_max_translation_delta_) {
      if (debug_flag_) {
        RCLCPP_INFO(
          get_logger(),
          "Rejected loop candidate %d -> %d because translation correction %.3f m exceeds %.3f m",
          candidate.index,
          latest_idx,
          translation_delta_m,
          loop_max_translation_delta_);
      }
      continue;
    }
    if (rotation_delta_deg > loop_max_rotation_delta_deg_) {
      if (debug_flag_) {
        RCLCPP_INFO(
          get_logger(),
          "Rejected loop candidate %d -> %d because rotation correction %.3f deg exceeds %.3f deg",
          candidate.index,
          latest_idx,
          rotation_delta_deg,
          loop_max_rotation_delta_deg_);
      }
      continue;
    }

    candidate_result.valid = true;
    if (!best_candidate.valid || fitness_score < best_candidate.fitness_score) {
      best_candidate = candidate_result;
    }
    if (
      candidate.source == LoopCandidate::Source::SCAN_CONTEXT &&
      (!best_scan_context_candidate.valid ||
      fitness_score < best_scan_context_candidate.fitness_score))
    {
      best_scan_context_candidate = candidate_result;
    }
  }

  if (prefer_scan_context_candidates_ && best_scan_context_candidate.valid) {
    if (
      !best_candidate.valid ||
      best_candidate.index != best_scan_context_candidate.index ||
      best_candidate.source != LoopCandidate::Source::SCAN_CONTEXT)
    {
      std::cout << "Preferring valid ScanContext candidate id:" <<
        best_scan_context_candidate.index << " over best candidate id:" <<
        (best_candidate.valid ? std::to_string(best_candidate.index) : std::string("none"))
                << std::endl;
    }
    best_candidate = best_scan_context_candidate;
  }

  if (!best_candidate.valid) {
    if (best_attempt.index >= 0) {
      std::cout << "best_loop_candidate id:" << best_attempt.index
                << " source:" << candidate_source_name(best_attempt.source)
                << " latest_id:" << latest_idx
                << " travel_distance:" << best_attempt.travel_distance
                << " euclidean_distance:" << best_attempt.euclidean_distance
                << " fitness:" << best_attempt.fitness_score
                << " correction_translation:" << best_attempt.translation_delta_m
                << " correction_rotation_deg:" << best_attempt.rotation_delta_deg
                << " used_3d_bbs:" << best_attempt.used_3d_bbs
                << " 3d_bbs_score:" << best_attempt.three_d_bbs_score_percentage
                << std::endl;
    } else if (attempted_registration && debug_flag_) {
      RCLCPP_INFO(
        get_logger(), "No converged loop candidate remained for latest submap %d",
        latest_idx);
    }
    return;
  }

  Eigen::Affine3d init_affine;
  tf2::fromMsg(latest_submap.pose, init_affine);
  Eigen::Affine3d submap_affine;
  tf2::fromMsg(map_array_msg.submaps[best_candidate.index].pose, submap_affine);

  LoopEdge loop_edge;
  loop_edge.pair_id = std::pair<int, int>(best_candidate.index, latest_idx);
  Eigen::Isometry3d from = Eigen::Isometry3d(submap_affine.matrix());
  Eigen::Isometry3d to = Eigen::Isometry3d(
    best_candidate.final_transformation.cast<double>() * init_affine.matrix());

  loop_edge.relative_pose = Eigen::Isometry3d(from.inverse() * to);
  loop_edge.fitness_score = best_candidate.fitness_score;
  const bool graph_changed = upsertLoopEdge(loop_edge);

  std::cout << "---" << std::endl;
  std::cout << "PoseAdjustment distance:" << best_candidate.travel_distance
            << ", score:" << best_candidate.fitness_score << std::endl;
  std::cout << "id_loop_point 1:" << best_candidate.index
            << " id_loop_point 2:" << latest_idx << std::endl;
  std::cout << "loop_candidate_source:" << candidate_source_name(best_candidate.source) <<
    std::endl;
  if (best_candidate.used_3d_bbs) {
    std::cout << "3d_bbs_score_percentage:" << best_candidate.three_d_bbs_score_percentage
              << " elapsed_msec:" << best_candidate.three_d_bbs_elapsed_msec << std::endl;
  }
  std::cout << "correction translation[m]:" << best_candidate.translation_delta_m
            << " rotation[deg]:" << best_candidate.rotation_delta_deg << std::endl;
  std::cout << "final transformation:" << std::endl;
  std::cout << best_candidate.final_transformation << std::endl;
  if (!graph_changed) {
    std::cout << "loop edge skipped as redundant or lower quality" << std::endl;
    return;
  }
  snapshotLoopEdges(loop_edges);
  doPoseAdjustment(map_array_msg, loop_edges, use_save_map_in_loop_);
}  // NOLINT(readability/fn_size)

void GraphBasedSlamComponent::doPoseAdjustment(
  lidarslam_msgs::msg::MapArray map_array_msg,
  const LoopEdges & loop_edges,
  bool do_save_map)
{
  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linear_solver =
    std::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
  g2o::OptimizationAlgorithmLevenberg * solver = new g2o::OptimizationAlgorithmLevenberg(
    std::make_unique<g2o::BlockSolver_6_3>(std::move(linear_solver)));

  optimizer.setAlgorithm(solver);

  int submaps_size = map_array_msg.submaps.size();
  for (int i = 0; i < submaps_size; i++) {
    Eigen::Affine3d affine;
    Eigen::fromMsg(map_array_msg.submaps[i].pose, affine);
    Eigen::Isometry3d pose(affine.matrix());

    g2o::VertexSE3 * vertex_se3 = new g2o::VertexSE3();
    vertex_se3->setId(i);
    vertex_se3->setEstimate(pose);
    if (i == 0) {vertex_se3->setFixed(true);}
    optimizer.addVertex(vertex_se3);

    if (i > 0) {
      const int start_idx = std::max(0, i - num_adjacent_pose_cnstraints_);
      for (int pre_idx = start_idx; pre_idx < i; pre_idx++) {
        Eigen::Affine3d pre_affine;
        Eigen::fromMsg(map_array_msg.submaps[pre_idx].pose, pre_affine);
        Eigen::Isometry3d pre_pose(pre_affine.matrix());
        Eigen::Isometry3d relative_pose = pre_pose.inverse() * pose;

        const int separation = i - pre_idx;
        const double edge_weight = adjacent_edge_info_weight_ / static_cast<double>(separation);
        Eigen::Matrix<double, 6, 6> info_mat =
          Eigen::Matrix<double, 6, 6>::Identity() * edge_weight;
        g2o::EdgeSE3 * edge_se3 = new g2o::EdgeSE3();
        edge_se3->setMeasurement(relative_pose);
        edge_se3->setInformation(info_mat);
        edge_se3->vertices()[0] = optimizer.vertex(pre_idx);
        edge_se3->vertices()[1] = optimizer.vertex(i);
        optimizer.addEdge(edge_se3);
      }
    }
  }
  /* IMU rotation constraint edges */
  if (use_imu_preintegration_ && submaps_size > 1) {
    std::lock_guard<std::mutex> imu_lock(imu_mtx_);
    int imu_edges_added = 0;
    for (int i = 1; i < submaps_size; i++) {
      double t0 = rclcpp::Time(map_array_msg.submaps[i - 1].header.stamp).seconds();
      double t1 = rclcpp::Time(map_array_msg.submaps[i].header.stamp).seconds();
      if (t1 <= t0 || t1 - t0 > 30.0) {continue;}

      Eigen::Quaterniond imu_delta_q = integrateImuRotation(t0, t1);
      if (imu_delta_q.isApprox(Eigen::Quaterniond::Identity(), 1e-8)) {continue;}

      // Build relative pose measurement: translation from odometry, rotation from IMU
      Eigen::Affine3d affine_prev, affine_curr;
      Eigen::fromMsg(map_array_msg.submaps[i - 1].pose, affine_prev);
      Eigen::fromMsg(map_array_msg.submaps[i].pose, affine_curr);
      Eigen::Isometry3d odom_prev(affine_prev.matrix());
      Eigen::Isometry3d odom_curr(affine_curr.matrix());
      Eigen::Isometry3d odom_relative = odom_prev.inverse() * odom_curr;

      // Replace rotation with IMU-integrated rotation
      Eigen::Isometry3d imu_relative = Eigen::Isometry3d::Identity();
      imu_relative.linear() = imu_delta_q.toRotationMatrix();
      imu_relative.translation() = odom_relative.translation();

      g2o::EdgeSE3 * edge_se3 = new g2o::EdgeSE3();
      edge_se3->setMeasurement(imu_relative);

      // Information matrix: high for roll/pitch rotation, moderate for yaw, zero for translation
      Eigen::Matrix<double, 6, 6> imu_info = Eigen::Matrix<double, 6, 6>::Zero();
      // g2o EdgeSE3 information: [rot(3) | trans(3)] order
      imu_info(0, 0) = imu_rotation_info_roll_pitch_;  // roll
      imu_info(1, 1) = imu_rotation_info_roll_pitch_;  // pitch
      imu_info(2, 2) = imu_rotation_info_yaw_;         // yaw
      // translation: zero weight (don't trust IMU double integration)
      edge_se3->setInformation(imu_info);

      edge_se3->vertices()[0] = optimizer.vertex(i - 1);
      edge_se3->vertices()[1] = optimizer.vertex(i);
      optimizer.addEdge(edge_se3);
      imu_edges_added++;
    }
    if (debug_flag_) {
      RCLCPP_INFO(get_logger(), "Added %d IMU rotation constraint edges", imu_edges_added);
    }
  }

  /* loop edge */
  for (const auto & loop_edge : loop_edges) {
    g2o::EdgeSE3 * edge_se3 = new g2o::EdgeSE3();
    edge_se3->setMeasurement(loop_edge.relative_pose);
    const double fitness = std::max(loop_edge.fitness_score, 1e-3);
    Eigen::Matrix<double, 6, 6> loop_info_mat =
      Eigen::Matrix<double, 6, 6>::Identity() * (loop_edge_info_weight_ / fitness);
    edge_se3->setInformation(loop_info_mat);
    auto * robust_kernel = new g2o::RobustKernelHuber();
    robust_kernel->setDelta(loop_edge_robust_kernel_delta_);
    edge_se3->setRobustKernel(robust_kernel);
    edge_se3->vertices()[0] = optimizer.vertex(loop_edge.pair_id.first);
    edge_se3->vertices()[1] = optimizer.vertex(loop_edge.pair_id.second);
    optimizer.addEdge(edge_se3);
  }

  /* GNSS position constraints */
  if (use_gnss_ && gnss_origin_set_) {
    std::lock_guard<std::mutex> gnss_lock(gnss_mtx_);
    int gnss_edges_added = 0;
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

      // Create unary-like constraint: edge from vertex i to a fixed GNSS position
      // Use EdgeSE3 with vertex 0 = fixed GNSS pose, vertex 1 = submap
      int gnss_vertex_id = submaps_size + gnss_edges_added;
      g2o::VertexSE3 * gnss_vertex = new g2o::VertexSE3();
      gnss_vertex->setId(gnss_vertex_id);
      Eigen::Isometry3d gnss_pose = Eigen::Isometry3d::Identity();
      gnss_pose.translation() = Eigen::Vector3d(best_gnss.x, best_gnss.y, best_gnss.z);
      gnss_vertex->setEstimate(gnss_pose);
      gnss_vertex->setFixed(true);
      optimizer.addVertex(gnss_vertex);

      g2o::EdgeSE3 * edge = new g2o::EdgeSE3();
      edge->setMeasurement(Eigen::Isometry3d::Identity());
      Eigen::Matrix<double, 6, 6> gnss_info = Eigen::Matrix<double, 6, 6>::Zero();
      gnss_info(3, 3) = best_gnss.info_x;
      gnss_info(4, 4) = best_gnss.info_y;
      gnss_info(5, 5) = best_gnss.info_z;
      edge->setInformation(gnss_info);
      edge->vertices()[0] = gnss_vertex;
      edge->vertices()[1] = optimizer.vertex(i);
      optimizer.addEdge(edge);
      if (best_gnss.rtk_like) {
        gnss_rtk_like_edges_added++;
      }
      gnss_edges_added++;
    }
    if (debug_flag_) {
      RCLCPP_INFO(
        get_logger(),
        "Added %d GNSS position constraint edges (%d RTK-like by covariance)",
        gnss_edges_added, gnss_rtk_like_edges_added);
    }
  }

  optimizer.initializeOptimization();
  optimizer.optimize(10);
  optimizer.save("pose_graph.g2o");

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
    g2o::VertexSE3 * vertex_se3 = static_cast<g2o::VertexSE3 *>(optimizer.vertex(i));
    Eigen::Affine3d se3 = vertex_se3->estimate();
    geometry_msgs::msg::Pose pose = tf2::toMsg(se3);

    /* map */
    Eigen::Affine3d previous_affine;
    tf2::fromMsg(map_array_msg.submaps[i].pose, previous_affine);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr;
    if (use_pcd_cache_) {
      cloud_ptr = loadSubmapFromPCD(i);
    } else {
      cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(map_array_msg.submaps[i].cloud, *cloud_ptr);
    }
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
        get_logger(),
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
  }
}

void GraphBasedSlamComponent::receiveNavSatFix(const sensor_msgs::msg::NavSatFix & msg)
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
  const double receive_time_sec = get_clock()->now().seconds();
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
      get_logger(),
      *get_clock(),
      5000,
      "GNSS header stamp %.3f s differs from receive time %.3f s by more than "
      "%.3f s; using receive time",
      header_time_sec, receive_time_sec, gnss_header_stamp_max_skew_sec_);
  }

  if (debug_flag_ && gnss_weights.covariance_valid) {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
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

bool GraphBasedSlamComponent::isUsableGnssFix(const sensor_msgs::msg::NavSatFix & msg) const
{
  if (!std::isfinite(msg.latitude) || !std::isfinite(msg.longitude) ||
    !std::isfinite(msg.altitude))
  {
    return false;
  }
  if (msg.latitude < -90.0 || msg.latitude > 90.0) {
    return false;
  }
  if (msg.longitude < -180.0 || msg.longitude > 180.0) {
    return false;
  }
  if (std::abs(msg.latitude) < 1e-6 && std::abs(msg.longitude) < 1e-6) {
    return false;
  }
  return true;
}

void GraphBasedSlamComponent::tryInitializeGnssOrigin(double lat, double lon, double alt)
{
  GnssOriginSample sample {lat, lon, alt};

  if (!gnss_origin_candidates_.empty()) {
    double mean_lat = 0.0;
    double mean_lon = 0.0;
    double mean_alt = 0.0;
    for (const auto & candidate : gnss_origin_candidates_) {
      mean_lat += candidate.lat;
      mean_lon += candidate.lon;
      mean_alt += candidate.alt;
    }
    mean_lat /= gnss_origin_candidates_.size();
    mean_lon /= gnss_origin_candidates_.size();
    mean_alt /= gnss_origin_candidates_.size();

    const double jump_m = approximateGeodeticDistanceMeters(mean_lat, mean_lon, lat, lon);
    if (jump_m > gnss_origin_consistency_threshold_m_) {
      RCLCPP_WARN(
        get_logger(),
        "Resetting GNSS origin initialization after %.1f m jump in candidate fixes",
        jump_m);
      gnss_origin_candidates_.clear();
    }
  }

  gnss_origin_candidates_.push_back(sample);

  if (static_cast<int>(gnss_origin_candidates_.size()) < gnss_origin_min_samples_) {
    return;
  }

  double mean_lat = 0.0;
  double mean_lon = 0.0;
  double mean_alt = 0.0;
  for (const auto & candidate : gnss_origin_candidates_) {
    mean_lat += candidate.lat;
    mean_lon += candidate.lon;
    mean_alt += candidate.alt;
  }
  mean_lat /= gnss_origin_candidates_.size();
  mean_lon /= gnss_origin_candidates_.size();
  mean_alt /= gnss_origin_candidates_.size();

  double max_deviation_m = 0.0;
  for (const auto & candidate : gnss_origin_candidates_) {
    const double deviation_m = approximateGeodeticDistanceMeters(
      mean_lat, mean_lon, candidate.lat, candidate.lon);
    if (deviation_m > max_deviation_m) {
      max_deviation_m = deviation_m;
    }
  }

  if (max_deviation_m > gnss_origin_consistency_threshold_m_) {
    const GnssOriginSample latest = gnss_origin_candidates_.back();
    gnss_origin_candidates_.clear();
    gnss_origin_candidates_.push_back(latest);
    RCLCPP_WARN(
      get_logger(),
      "GNSS origin candidates were inconsistent (max deviation %.1f m), restarting accumulation",
      max_deviation_m);
    return;
  }

  gnss_origin_lat_ = mean_lat;
  gnss_origin_lon_ = mean_lon;
  gnss_origin_alt_ = mean_alt;
  gnss_origin_set_ = true;
  gnss_origin_candidates_.clear();
  RCLCPP_INFO(
    get_logger(),
    "GNSS origin set from %d consistent fixes: lat=%.8f, lon=%.8f, alt=%.2f",
    gnss_origin_min_samples_, gnss_origin_lat_, gnss_origin_lon_, gnss_origin_alt_);
}

double GraphBasedSlamComponent::approximateGeodeticDistanceMeters(
  double lat0, double lon0, double lat1, double lon1) const
{
  constexpr double kEarthRadiusM = 6378137.0;
  auto toRad = [](double deg) {return deg * M_PI / 180.0;};

  const double lat0_rad = toRad(lat0);
  const double lat1_rad = toRad(lat1);
  const double dlat = lat1_rad - lat0_rad;
  const double dlon = toRad(lon1 - lon0);
  const double x = dlon * std::cos((lat0_rad + lat1_rad) * 0.5);
  const double y = dlat;
  return std::sqrt(x * x + y * y) * kEarthRadiusM;
}

Eigen::Vector3d GraphBasedSlamComponent::geodeticToEnu(
  double lat, double lon, double alt) const
{
  // WGS84 parameters
  constexpr double a = 6378137.0;              // semi-major axis [m]
  constexpr double f = 1.0 / 298.257223563;    // flattening
  constexpr double e2 = 2 * f - f * f;         // eccentricity squared

  auto toRad = [](double deg) {return deg * M_PI / 180.0;};

  double lat0 = toRad(gnss_origin_lat_);
  double lon0 = toRad(gnss_origin_lon_);
  double lat1 = toRad(lat);
  double lon1 = toRad(lon);

  double dlat = lat1 - lat0;
  double dlon = lon1 - lon0;
  double dalt = alt - gnss_origin_alt_;

  double sin_lat0 = std::sin(lat0);
  double N = a / std::sqrt(1.0 - e2 * sin_lat0 * sin_lat0);
  double M = a * (1.0 - e2) / std::pow(1.0 - e2 * sin_lat0 * sin_lat0, 1.5);

  // ENU: East = dlon * N * cos(lat), North = dlat * M, Up = dalt
  double east = dlon * N * std::cos(lat0);
  double north = dlat * M;
  double up = dalt;

  return Eigen::Vector3d(east, north, up);
}

void GraphBasedSlamComponent::receiveImu(const sensor_msgs::msg::Imu & msg)
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

Eigen::Quaterniond GraphBasedSlamComponent::integrateImuRotation(double t0, double t1) const
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

void GraphBasedSlamComponent::receiveCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (debug_flag_ && !latest_cloud_) {
    RCLCPP_INFO(get_logger(), "First cloud received, %zu bytes", msg->data.size());
  }
  latest_cloud_ = msg;
  latest_cloud_stamp_ = rclcpp::Time(msg->header.stamp);
  // When cloud arrives, try to create submap with latest odom
  tryCreateSubmap();
}

void GraphBasedSlamComponent::receiveOdometry(const nav_msgs::msg::Odometry & msg)
{
  // Buffer latest odom
  Eigen::Vector3d pos(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
  if (!std::isfinite(pos.x()) || !std::isfinite(pos.y()) || !std::isfinite(pos.z())) {
    return;
  }
  if (debug_flag_ && !latest_odom_valid_) {
    RCLCPP_INFO(get_logger(), "First odom received: (%.2f, %.2f, %.2f)", pos.x(), pos.y(), pos.z());
  }
  latest_odom_ = msg;
  latest_odom_valid_ = true;
}

void GraphBasedSlamComponent::tryCreateSubmap()
{
  if (!latest_odom_valid_ || !latest_cloud_) {return;}

  Eigen::Vector3d pos(
    latest_odom_.pose.pose.position.x,
    latest_odom_.pose.pose.position.y,
    latest_odom_.pose.pose.position.z);

  // Check distance threshold
  if (last_submap_position_valid_) {
    double dist = (pos - last_submap_position_).norm();
    if (dist < submap_distance_threshold_) {return;}
    if (dist > 100.0) {return;}
    accumulated_distance_ += dist;
  }
  last_submap_position_ = pos;
  last_submap_position_valid_ = true;

  // Create SubMap (use "map" frame for SLAM output regardless of odom frame)
  lidarslam_msgs::msg::SubMap submap;
  submap.header.stamp = latest_odom_.header.stamp;
  submap.header.frame_id = "map";
  submap.distance = accumulated_distance_;
  submap.pose = latest_odom_.pose.pose;
  submap.cloud = *latest_cloud_;
  submap.cloud.header.frame_id = latest_odom_.child_frame_id;

  int n;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    map_array_msg_.header.stamp = latest_odom_.header.stamp;
    map_array_msg_.header.frame_id = "map";
    map_array_msg_.submaps.push_back(submap);
    n = map_array_msg_.submaps.size();

    // Save to PCD and clear cloud from memory
    if (use_pcd_cache_) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(submap.cloud, *cloud);
      saveSubmapToPCD(n - 1, cloud);
      // Clear cloud data from memory (keep pose and metadata)
      map_array_msg_.submaps.back().cloud = sensor_msgs::msg::PointCloud2();
    }

    initial_map_array_received_ = true;
    is_map_array_updated_ = true;
  }

  if (n % 50 == 0) {
    RCLCPP_INFO(get_logger(), "Odom input: %d submaps, distance: %.1fm", n, accumulated_distance_);
  }
}

void GraphBasedSlamComponent::saveSubmapToPCD(
  int idx,
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud)
{
  std::string path = pcd_cache_dir_ + "/submap_" + std::to_string(idx) + ".pcd";
  pcl::io::savePCDFileBinaryCompressed(path, *cloud);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr GraphBasedSlamComponent::loadSubmapFromPCD(int idx)
{
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  std::string path = pcd_cache_dir_ + "/submap_" + std::to_string(idx) + ".pcd";
  if (pcl::io::loadPCDFile(path, *cloud) == -1) {
    RCLCPP_WARN(get_logger(), "Failed to load PCD: %s", path.c_str());
  }
  return cloud;
}

void GraphBasedSlamComponent::saveGridDividedMap(
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

  std::cout << "Map points: " << map->size() << " -> " << downsampled->size()
            << " (leaf=" << map_leaf_size_ << "m)" << std::endl;

  // Compute bounding box
  pcl::PointXYZI min_pt, max_pt;
  pcl::getMinMax3D(*downsampled, min_pt, max_pt);

  // Compute grid bounds (align to grid)
  double x_min = std::floor(min_pt.x / map_grid_size_x_) * map_grid_size_x_;
  double y_min = std::floor(min_pt.y / map_grid_size_y_) * map_grid_size_y_;
  double x_max = std::ceil(max_pt.x / map_grid_size_x_) * map_grid_size_x_;
  double y_max = std::ceil(max_pt.y / map_grid_size_y_) * map_grid_size_y_;

  int nx = static_cast<int>((x_max - x_min) / map_grid_size_x_);
  int ny = static_cast<int>((y_max - y_min) / map_grid_size_y_);
  if (nx <= 0) {nx = 1;}
  if (ny <= 0) {ny = 1;}

  // Assign points to grid cells
  std::map<std::pair<int, int>, pcl::PointCloud<pcl::PointXYZI>::Ptr> grid_cells;
  for (const auto & pt : downsampled->points) {
    int gx = static_cast<int>(std::floor((pt.x - x_min) / map_grid_size_x_));
    int gy = static_cast<int>(std::floor((pt.y - y_min) / map_grid_size_y_));
    auto key = std::make_pair(gx, gy);
    if (grid_cells.find(key) == grid_cells.end()) {
      grid_cells[key] = pcl::PointCloud<pcl::PointXYZI>::Ptr(
        new pcl::PointCloud<pcl::PointXYZI>);
    }
    grid_cells[key]->push_back(pt);
  }

  // Save each grid cell as PCD and build metadata
  // Format: Autoware pointcloud_map_loader expects:
  //   x_resolution: 20.0
  //   y_resolution: 20.0
  //   filename.pcd: [x, y]   (lower-left corner of grid cell)
  std::ofstream meta(out_dir + "/pointcloud_map_metadata.yaml");
  meta << std::fixed;
  meta << "x_resolution: " << std::setprecision(1) << map_grid_size_x_ << std::endl;
  meta << "y_resolution: " << std::setprecision(1) << map_grid_size_y_ << std::endl;

  int saved = 0;
  for (auto & [key, cloud] : grid_cells) {
    if (cloud->empty()) {continue;}
    double cell_x = x_min + key.first * map_grid_size_x_;
    double cell_y = y_min + key.second * map_grid_size_y_;

    std::ostringstream filename;
    filename << static_cast<int>(cell_x) << "_"
             << static_cast<int>(cell_y) << ".pcd";
    std::string filepath = out_dir + "/" + filename.str();
    pcl::io::savePCDFileBinaryCompressed(filepath, *cloud);

    meta << filename.str() << ": ["
         << static_cast<int>(cell_x) << ", "
         << static_cast<int>(cell_y) << "]" << std::endl;
    saved++;
  }

  meta.close();

  // Also save the full map as a single PCD for convenience
  pcl::io::savePCDFileBinaryCompressed(map_save_dir_ + "/map.pcd", *downsampled);

  std::cout << "Saved grid-divided map: " << saved << " cells ("
            << map_grid_size_x_ << "x" << map_grid_size_y_ << "m) to " << out_dir
            << std::endl;
  std::cout << "Total points: " << downsampled->size() << std::endl;
  std::cout << "Metadata: " << out_dir << "/pointcloud_map_metadata.yaml" << std::endl;

  // Always emit map_projector_info.yaml so Autoware can load pointcloud-only maps.
  std::string proj_file = map_save_dir_ + "/map_projector_info.yaml";
  std::ofstream proj(proj_file);
  proj << std::fixed << std::setprecision(10);
  if (gnss_origin_set_) {
    proj << "projector_type: LocalCartesian" << std::endl;
    proj << "vertical_datum: WGS84" << std::endl;
    proj << "map_origin:" << std::endl;
    proj << "  latitude: " << gnss_origin_lat_ << std::endl;
    proj << "  longitude: " << gnss_origin_lon_ << std::endl;
    std::cout << "Saved Autoware map projector info (LocalCartesian): " << proj_file
              << std::endl;
  } else {
    proj << "projector_type: Local" << std::endl;
    std::cout << "Saved Autoware map projector info (Local): " << proj_file << std::endl;
  }
  proj.close();
}
}  // namespace graphslam

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(graphslam::GraphBasedSlamComponent)
