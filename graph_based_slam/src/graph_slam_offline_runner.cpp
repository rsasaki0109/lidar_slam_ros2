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

// The offline deterministic backend runner (docs/roadmap/v0.6.md, Phase 2):
// read a recorded backend-input bag (odometry + deskewed cloud pairs,
// e.g. from scripts/run_backend_replay_variance.sh --mode record) directly
// with rosbag2_cpp — no pub/sub, no executor, no wall clock — and feed the
// pairs through the same submap creation, event-driven BackendCore loop
// search and pose-graph optimization the live component uses. Bag order is
// a total order, so the contract "same bag + same config => identical
// loop-edge set" becomes testable; scripts/run_offline_determinism_check.sh
// runs this N times and diffs loop_edges.csv.
//
// The node is named graph_based_slam so the existing parameter presets
// (e.g. lidarslam/param/lidarslam_mid360_rko_graph.yaml) apply unchanged:
//
//   ros2 run graph_based_slam graph_slam_offline_runner --ros-args \
//     --params-file lidarslam/param/lidarslam_mid360_rko_graph.yaml \
//     -p bag_path:=output/backend_replay_x/backend_input \
//     -p output_dir:=/tmp/offline_run1

#include <pcl/io/pcd_io.h>  // NOLINT(build/include_order)
#include <pcl_conversions/pcl_conversions.h>  // NOLINT(build/include_order)

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "graph_based_slam/backend_core.hpp"
#include "graph_based_slam/degeneracy_diagnostics_csv.hpp"
#include "graph_based_slam/degeneracy_report_summary.hpp"
#include "graph_based_slam/map_refiner.hpp"
#include "graph_based_slam/pose_graph_optimization.hpp"
#include "graph_based_slam/registration_factory.hpp"
#include "graph_based_slam/submap_creation.hpp"
#include "graph_based_slam/three_d_bbs_loop_verifier.hpp"

namespace
{

using graphslam::backend_core::BackendCore;
using CloudPtr = BackendCore::CloudPtr;

struct SubmapRecord
{
  graphslam::backend_core::SubmapMeta meta;
  double stamp_sec{0.0};
  CloudPtr cloud;
};

void writeTum(
  const std::string & path, const std::vector<SubmapRecord> & records,
  const std::vector<Eigen::Isometry3d> & poses)
{
  std::ofstream out(path);
  char line[256];
  for (std::size_t i = 0; i < records.size(); ++i) {
    const Eigen::Vector3d t = poses[i].translation();
    const Eigen::Quaterniond q(poses[i].rotation());
    std::snprintf(
      line, sizeof(line), "%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f",
      records[i].stamp_sec, t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w());
    out << line << "\n";
  }
}

// v0.8 Phase 1 (docs/roadmap/v0.8.md §5): opt-in, report-only per-scan
// degeneracy diagnostics from the recorded odometry covariance (filled by
// the Thirdparty/rko_lio diagnostic patch). Default off: the existing
// deterministic artifacts (loop_edges.csv, TUM trajectories) are
// byte-identical whether or not these outputs are requested.
struct DegeneracyDiagnosticsSink
{
  std::ofstream csv;
  bool report_enabled{false};
  graphslam::degeneracy::DegeneracyReportAccumulator accumulator;

  void configure(rclcpp::Node & node, const rclcpp::Logger & logger)
  {
    std::string csv_path;
    node.get_parameter_or(
      "degeneracy_diagnostics_csv_path", csv_path, std::string());
    node.get_parameter_or("save_degeneracy_report", report_enabled, false);
    if (csv_path.empty()) {
      return;
    }
    csv.open(csv_path);
    if (csv.is_open()) {
      csv << graphslam::degeneracy::degeneracyDiagnosticsCsvHeaderLine() << "\n";
    } else {
      RCLCPP_WARN(
        logger, "failed to open degeneracy_diagnostics_csv_path: %s (CSV disabled)",
        csv_path.c_str());
    }
  }

  // One row per paired scan, before the submap-distance decision -- the
  // degeneracy signal is a property of the frontend solve, not of submap
  // spacing.
  void recordScan(const nav_msgs::msg::Odometry & odom)
  {
    if (!csv.is_open() && !report_enabled) {
      return;
    }
    const graphslam::degeneracy::CovarianceLocalizabilityResult result =
      graphslam::degeneracy::analyzeOdometryCovariance(odom.pose.covariance);
    const double stamp_sec = rclcpp::Time(odom.header.stamp).seconds();
    if (csv.is_open()) {
      csv << graphslam::degeneracy::degeneracyDiagnosticsCsvRowLine(stamp_sec, result) << "\n";
    }
    if (report_enabled) {
      accumulator.add(stamp_sec, result);
    }
  }

  void writeReport(const std::string & output_dir, const rclcpp::Logger & logger)
  {
    if (!report_enabled) {
      return;
    }
    const std::string report_path = output_dir + "/degeneracy_report.yaml";
    std::ofstream report(report_path);
    if (!report.is_open()) {
      RCLCPP_WARN(logger, "failed to write degeneracy report: %s", report_path.c_str());
      return;
    }
    const std::vector<std::string> lines =
      graphslam::degeneracy::degeneracyReportYamlLines(accumulator.summary());
    for (size_t i = 0; i < lines.size(); ++i) {
      report << lines[i] << "\n";
    }
    RCLCPP_INFO(logger, "Wrote %s", report_path.c_str());
  }
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "graph_based_slam",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto logger = node->get_logger();

  std::string bag_path;
  std::string output_dir;
  std::string odom_topic;
  std::string cloud_topic;
  node->get_parameter_or("bag_path", bag_path, std::string());
  node->get_parameter_or("output_dir", output_dir, std::string("."));
  node->get_parameter_or("offline_odom_topic", odom_topic, std::string("/rko_lio/odometry"));
  node->get_parameter_or("offline_cloud_topic", cloud_topic, std::string("/rko_lio/frame"));
  if (bag_path.empty()) {
    RCLCPP_ERROR(logger, "bag_path parameter is required");
    rclcpp::shutdown();
    return 1;
  }
  std::filesystem::create_directories(output_dir);

  // Same parameter names and defaults as the live component.
  std::string registration_method;
  double ndt_resolution;
  int ndt_num_threads;
  double voxel_leaf_size;
  double submap_distance_threshold;
  bool debug_flag;
  node->get_parameter_or("registration_method", registration_method, std::string("NDT"));
  node->get_parameter_or("ndt_resolution", ndt_resolution, 5.0);
  node->get_parameter_or("ndt_num_threads", ndt_num_threads, 0);
  node->get_parameter_or("voxel_leaf_size", voxel_leaf_size, 0.2);
  node->get_parameter_or("submap_distance_threshold", submap_distance_threshold, 1.5);
  node->get_parameter_or("debug_flag", debug_flag, false);

  // v0.7 (docs/roadmap/v0.7.md): offline plane-BA refinement of the
  // optimized submap poses. On by default since Phase 3 (holdout-validated
  // evidence in docs/research/map-quality-baseline.md); when it does not
  // improve anything the pose-graph solution is kept unchanged, and the
  // pose-graph artifacts (loop_edges.csv, trajectory_optimized.tum) are
  // byte-identical either way.
  bool refine = true;
  double refine_cloud_downsample = 0.10;
  int refine_window_size = 16;
  int refine_window_stride = 8;
  node->get_parameter_or("refine", refine, true);
  node->get_parameter_or("refine_cloud_downsample", refine_cloud_downsample, 0.10);
  node->get_parameter_or("refine_window_size", refine_window_size, 16);
  node->get_parameter_or("refine_window_stride", refine_window_stride, 8);
  bool refine_save_maps = false;
  node->get_parameter_or("refine_save_maps", refine_save_maps, false);

  DegeneracyDiagnosticsSink degeneracy_sink;
  degeneracy_sink.configure(*node, logger);

  graphslam::backend_core::DescriptorConfig descriptor_config;
  node->get_parameter_or("use_scan_context", descriptor_config.use_scan_context, false);
  node->get_parameter_or("use_bev_descriptor", descriptor_config.use_bev_descriptor, false);
  node->get_parameter_or("use_solid_descriptor", descriptor_config.use_solid_descriptor, false);
  node->get_parameter_or(
    "use_triangle_descriptor", descriptor_config.use_triangle_descriptor, false);
  node->get_parameter_or(
    "bev_descriptor_grid_size_m", descriptor_config.bev_descriptor_grid_size_m, 80.0);
  node->get_parameter_or(
    "bev_descriptor_grid_cells", descriptor_config.bev_descriptor_grid_cells, 40);
  node->get_parameter_or(
    "bev_descriptor_yaw_bins", descriptor_config.bev_descriptor_yaw_bins, 24);
  node->get_parameter_or(
    "triangle_descriptor_keypoint_mode",
    descriptor_config.triangle_descriptor_keypoint_mode, std::string("bev_max_height"));
  node->get_parameter_or(
    "triangle_descriptor_grid_size_m",
    descriptor_config.triangle_descriptor_grid_size_m, 60.0);
  node->get_parameter_or(
    "triangle_descriptor_grid_cells", descriptor_config.triangle_descriptor_grid_cells, 100);
  node->get_parameter_or(
    "triangle_descriptor_min_salience_m",
    descriptor_config.triangle_descriptor_min_salience_m, 0.8);
  node->get_parameter_or(
    "triangle_descriptor_max_keypoints",
    descriptor_config.triangle_descriptor_max_keypoints, 40);
  node->get_parameter_or(
    "triangle_descriptor_edge_voxel_size_m",
    descriptor_config.triangle_descriptor_edge_voxel_size_m, 0.4);
  node->get_parameter_or(
    "triangle_descriptor_edge_neighbor_radius_m",
    descriptor_config.triangle_descriptor_edge_neighbor_radius_m, 1.0);
  node->get_parameter_or(
    "triangle_descriptor_edge_min_neighbors",
    descriptor_config.triangle_descriptor_edge_min_neighbors, 6);
  node->get_parameter_or(
    "triangle_descriptor_edge_min_edgeness",
    descriptor_config.triangle_descriptor_edge_min_edgeness, 0.5);
  node->get_parameter_or(
    "triangle_descriptor_edge_nms_radius_m",
    descriptor_config.triangle_descriptor_edge_nms_radius_m, 2.0);
  node->get_parameter_or(
    "triangle_descriptor_min_edge_m", descriptor_config.triangle_descriptor_min_edge_m, 2.0);
  node->get_parameter_or(
    "triangle_descriptor_max_edge_m", descriptor_config.triangle_descriptor_max_edge_m, 50.0);
  node->get_parameter_or(
    "triangle_descriptor_max_triangles",
    descriptor_config.triangle_descriptor_max_triangles, 3000);
  node->get_parameter_or(
    "triangle_descriptor_edge_bin_m", descriptor_config.triangle_descriptor_edge_bin_m, 0.5);
  node->get_parameter_or(
    "triangle_descriptor_quad_feature_bin_m",
    descriptor_config.triangle_descriptor_quad_feature_bin_m, 0.0);

  graphslam::backend_core::LoopSearchConfig search_config;
  node->get_parameter_or("search_submap_num", search_config.search_submap_num, 3);
  node->get_parameter_or(
    "prefer_scan_context_candidates", search_config.prefer_scan_context_candidates, false);
  node->get_parameter_or(
    "use_3d_bbs_for_scan_context", search_config.use_3d_bbs_for_scan_context, false);
  node->get_parameter_or(
    "three_d_bbs_source_submap_num", search_config.three_d_bbs_source_submap_num, 2);
  node->get_parameter_or(
    "three_d_bbs_target_submap_radius", search_config.three_d_bbs_target_submap_radius, 1);
  node->get_parameter_or(
    "three_d_bbs_voxel_leaf_size", search_config.three_d_bbs_voxel_leaf_size, 1.0);
  node->get_parameter_or(
    "three_d_bbs_min_level_res", search_config.three_d_bbs_min_level_res, 1.0);
  node->get_parameter_or("three_d_bbs_max_level", search_config.three_d_bbs_max_level, 3);
  node->get_parameter_or(
    "three_d_bbs_score_threshold_percentage",
    search_config.three_d_bbs_score_threshold_percentage, 0.25);
  node->get_parameter_or("three_d_bbs_timeout_msec", search_config.three_d_bbs_timeout_msec, 50);
  node->get_parameter_or("three_d_bbs_num_threads", search_config.three_d_bbs_num_threads, 0);
  node->get_parameter_or(
    "three_d_bbs_translation_search_margin_m",
    search_config.three_d_bbs_translation_search_margin_m, 15.0);
  node->get_parameter_or(
    "three_d_bbs_roll_pitch_search_deg",
    search_config.three_d_bbs_roll_pitch_search_deg, 10.0);
  node->get_parameter_or(
    "three_d_bbs_yaw_search_deg", search_config.three_d_bbs_yaw_search_deg, 180.0);

  graphslam::candidate_aggregator::Config & aggregator = search_config.aggregator;
  aggregator.debug = debug_flag;
  node->get_parameter_or("max_loop_candidate_count", aggregator.max_loop_candidate_count, 3);
  node->get_parameter_or("distance_loop_closure", aggregator.distance_loop_closure, 20.0);
  node->get_parameter_or(
    "range_of_searching_loop_closure", aggregator.range_of_searching_loop_closure, 20.0);
  node->get_parameter_or("scan_context_threshold", aggregator.scan_context_threshold, 0.3);
  node->get_parameter_or(
    "bev_use_mutual_visibility", aggregator.bev_use_mutual_visibility, false);
  node->get_parameter_or(
    "bev_mutual_visibility_min_overlap_ratio",
    aggregator.bev_mutual_visibility_min_overlap_ratio, 0.05);
  node->get_parameter_or(
    "bev_mutual_visibility_occupancy_eps",
    aggregator.bev_mutual_visibility_occupancy_eps, 0.5);
  aggregator.bev_descriptor_yaw_bins = descriptor_config.bev_descriptor_yaw_bins;
  node->get_parameter_or(
    "bev_descriptor_max_euclidean_distance_m",
    aggregator.bev_descriptor_max_euclidean_distance_m, -1.0);
  node->get_parameter_or("bev_descriptor_threshold", aggregator.bev_descriptor_threshold, 0.20);
  node->get_parameter_or(
    "bev_descriptor_sequence_window", aggregator.bev_descriptor_sequence_window, 0);
  node->get_parameter_or(
    "bev_descriptor_sequence_threshold", aggregator.bev_descriptor_sequence_threshold, -1.0);
  node->get_parameter_or(
    "bev_descriptor_pose_consistency_threshold_m",
    aggregator.bev_descriptor_pose_consistency_threshold_m, -1.0);
  node->get_parameter_or(
    "bev_descriptor_rerank_weight_m", aggregator.bev_descriptor_rerank_weight_m, 100.0);
  node->get_parameter_or(
    "solid_descriptor_max_euclidean_distance_m",
    aggregator.solid_descriptor_max_euclidean_distance_m, -1.0);
  node->get_parameter_or(
    "solid_descriptor_min_similarity", aggregator.solid_descriptor_min_similarity, 0.70);
  node->get_parameter_or(
    "solid_descriptor_sequence_window", aggregator.solid_descriptor_sequence_window, 0);
  node->get_parameter_or(
    "solid_descriptor_sequence_min_similarity",
    aggregator.solid_descriptor_sequence_min_similarity, -1.0);
  node->get_parameter_or(
    "solid_descriptor_pose_consistency_threshold_m",
    aggregator.solid_descriptor_pose_consistency_threshold_m, -1.0);
  node->get_parameter_or(
    "triangle_descriptor_exclude_recent", aggregator.triangle_descriptor_exclude_recent, 4);
  aggregator.triangle_descriptor_edge_bin_m =
    descriptor_config.triangle_descriptor_edge_bin_m;
  aggregator.triangle_descriptor_quad_feature_bin_m =
    descriptor_config.triangle_descriptor_quad_feature_bin_m;
  node->get_parameter_or(
    "triangle_descriptor_inlier_translation_m",
    aggregator.triangle_descriptor_inlier_translation_m, 2.0);
  node->get_parameter_or(
    "triangle_descriptor_inlier_rotation_deg",
    aggregator.triangle_descriptor_inlier_rotation_deg, 5.0);
  node->get_parameter_or(
    "triangle_descriptor_min_inliers", aggregator.triangle_descriptor_min_inliers, 4);
  node->get_parameter_or(
    "triangle_descriptor_min_inlier_ratio",
    aggregator.triangle_descriptor_min_inlier_ratio, 0.0);
  node->get_parameter_or(
    "triangle_descriptor_max_pairs", aggregator.triangle_descriptor_max_pairs, 64);
  node->get_parameter_or(
    "triangle_descriptor_min_4th_point_agreements",
    aggregator.triangle_descriptor_min_4th_point_agreements, 0);
  node->get_parameter_or(
    "triangle_descriptor_fourth_point_max_distance_m",
    aggregator.triangle_descriptor_fourth_point_max_distance_m, 2.0);
  node->get_parameter_or(
    "triangle_descriptor_refine_se3_with_all_inliers",
    aggregator.triangle_descriptor_refine_se3_with_all_inliers, false);
  node->get_parameter_or(
    "triangle_descriptor_min_votes", aggregator.triangle_descriptor_min_votes, 6);
  node->get_parameter_or(
    "triangle_descriptor_skip_ransac", aggregator.triangle_descriptor_skip_ransac, false);
  node->get_parameter_or("triangle_verify_with_bev", aggregator.triangle_verify_with_bev, false);
  node->get_parameter_or(
    "triangle_verify_bev_max_distance", aggregator.triangle_verify_bev_max_distance, 0.30);

  graphslam::loop_verifier::GateConfig & gates = search_config.gates;
  node->get_parameter_or("threshold_loop_closure_score", gates.generic_score_threshold, 1.0);
  node->get_parameter_or(
    "scan_context_loop_closure_score_threshold", gates.scan_context_score_threshold, -1.0);
  node->get_parameter_or("loop_max_translation_delta", gates.max_translation_m, 15.0);
  node->get_parameter_or("loop_max_rotation_delta_deg", gates.max_rotation_deg, 45.0);
  node->get_parameter_or(
    "loop_max_translation_delta_descriptor", gates.max_translation_descriptor_m, -1.0);
  node->get_parameter_or(
    "loop_max_rotation_delta_deg_descriptor", gates.max_rotation_descriptor_deg, -1.0);

  int loop_edge_dedup_index_window;
  int num_adjacent_pose_constraints;
  double adjacent_edge_info_weight;
  double loop_edge_info_weight;
  double loop_edge_robust_kernel_delta;
  std::string loop_edge_robust_kernel_type;
  node->get_parameter_or("loop_edge_dedup_index_window", loop_edge_dedup_index_window, 8);
  node->get_parameter_or("num_adjacent_pose_cnstraints", num_adjacent_pose_constraints, 5);
  node->get_parameter_or("adjacent_edge_info_weight", adjacent_edge_info_weight, 1000.0);
  node->get_parameter_or("loop_edge_info_weight", loop_edge_info_weight, 100.0);
  node->get_parameter_or("loop_edge_robust_kernel_delta", loop_edge_robust_kernel_delta, 1.0);
  node->get_parameter_or(
    "loop_edge_robust_kernel_type", loop_edge_robust_kernel_type, std::string("huber"));

  auto registration = graphslam::backend_core::makeLoopRegistration(
    registration_method, ndt_resolution, ndt_num_threads);
  if (!registration) {
    RCLCPP_ERROR(logger, "invalid registration_method");
    rclcpp::shutdown();
    return 1;
  }
  pcl::VoxelGrid<pcl::PointXYZI> voxelgrid;
  voxelgrid.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  graphslam::ThreeDBBSLoopVerifier bbs_verifier;

  BackendCore core;
  core.configure(descriptor_config);
  graphslam::backend_core::LoopEdgeSet edge_set;
  edge_set.configure(loop_edge_dedup_index_window < 0 ? 0 : loop_edge_dedup_index_window);

  std::vector<SubmapRecord> records;
  bool last_position_valid = false;
  Eigen::Vector3d last_position = Eigen::Vector3d::Zero();
  double accumulated_distance = 0.0;
  int next_query_idx = 1;

  // The same voxel-filtered local-aggregate semantics as the component's
  // makeFilteredLocalSubmapProvider, over the in-memory record store.
  const auto filtered_local_provider = [&](int ref_idx) -> CloudPtr {
      CloudPtr aggregated_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      const Eigen::Affine3d & reference_affine = records[ref_idx].meta.pose;
      for (int k = 0; k < search_config.search_submap_num && (ref_idx - k) >= 0; ++k) {
        const int src_idx = ref_idx - k;
        const CloudPtr & cloud = records[src_idx].cloud;
        if (!cloud || cloud->empty()) {
          continue;
        }
        CloudPtr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        const Eigen::Matrix4f local_transform =
          (reference_affine.inverse() * records[src_idx].meta.pose).matrix().cast<float>();
        pcl::transformPointCloud(*cloud, *transformed_cloud, local_transform);
        *aggregated_cloud += *transformed_cloud;
      }
      CloudPtr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      if (aggregated_cloud->empty()) {
        return filtered_cloud;
      }
      voxelgrid.setInputCloud(aggregated_cloud);
      voxelgrid.filter(*filtered_cloud);
      return filtered_cloud;
    };
  const auto raw_cloud_provider = [&](int idx) -> CloudPtr {
      return records[idx].cloud;
    };

  const auto drain_queries = [&]() {
      const int num_submaps = static_cast<int>(records.size());
      while (next_query_idx < num_submaps) {
        const int query_idx = next_query_idx;
        core.ingestDescriptors(query_idx + 1, filtered_local_provider);
        std::vector<graphslam::backend_core::SubmapMeta> visible;
        visible.reserve(query_idx + 1);
        for (int i = 0; i <= query_idx; ++i) {
          visible.push_back(records[i].meta);
        }
        const graphslam::backend_core::LoopSearchOutput output = core.searchLoopForSubmap(
          visible, query_idx, search_config, raw_cloud_provider, *registration, voxelgrid,
          bbs_verifier);
        for (const auto & line : output.logs) {
          if (line.via_logger) {
            RCLCPP_INFO(logger, "%s", line.text.c_str());
          } else {
            std::cout << line.text << std::endl;
          }
        }
        if (output.proposal.found) {
          graphslam::backend_core::LoopEdgeSet::Edge edge;
          edge.pair_id = output.proposal.pair_id;
          edge.relative_pose = output.proposal.relative_pose;
          edge.fitness_score = output.proposal.fitness_score;
          if (!edge_set.upsert(edge)) {
            std::cout << "loop edge skipped as redundant or lower quality" << std::endl;
          }
        }
        next_query_idx = query_idx + 1;
      }
    };

  // --- Bag pass: pair odometry and cloud by exact stamp (the recorded
  // backend-input topics are emitted 1:1 with shared stamps), create
  // submaps with the same distance rule as the component, and drain the
  // event-driven loop search after every new submap.
  rosbag2_cpp::Reader reader;
  reader.open(bag_path);
  rclcpp::Serialization<nav_msgs::msg::Odometry> odom_serialization;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> cloud_serialization;
  std::map<uint64_t, nav_msgs::msg::Odometry> pending_odoms;
  std::map<uint64_t, sensor_msgs::msg::PointCloud2> pending_clouds;
  std::size_t paired_count = 0;

  const auto process_pair =
    [&](const nav_msgs::msg::Odometry & odom, const sensor_msgs::msg::PointCloud2 & cloud_msg) {
      ++paired_count;
      degeneracy_sink.recordScan(odom);
      const Eigen::Vector3d position(
        odom.pose.pose.position.x,
        odom.pose.pose.position.y,
        odom.pose.pose.position.z);
      const graphslam::submap_creation::Decision decision = graphslam::submap_creation::evaluate(
        position, last_position_valid, last_position, submap_distance_threshold);
      if (!decision.create) {
        return;
      }
      accumulated_distance += decision.distance;
      last_position = position;
      last_position_valid = true;

      SubmapRecord record;
      Eigen::Affine3d pose_affine;
      tf2::fromMsg(odom.pose.pose, pose_affine);
      record.meta.pose = pose_affine;
      record.meta.travel_distance = accumulated_distance;
      record.stamp_sec = rclcpp::Time(odom.header.stamp).seconds();
      record.cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(cloud_msg, *record.cloud);
      records.push_back(record);

      drain_queries();
    };

  while (reader.has_next()) {
    auto bag_message = reader.read_next();
    rclcpp::SerializedMessage serialized(*bag_message->serialized_data);
    if (bag_message->topic_name == odom_topic) {
      nav_msgs::msg::Odometry odom;
      odom_serialization.deserialize_message(&serialized, &odom);
      const uint64_t key =
        static_cast<uint64_t>(odom.header.stamp.sec) * 1000000000ull + odom.header.stamp.nanosec;
      const auto cloud_it = pending_clouds.find(key);
      if (cloud_it != pending_clouds.end()) {
        process_pair(odom, cloud_it->second);
        pending_clouds.erase(cloud_it);
      } else {
        pending_odoms[key] = odom;
      }
    } else if (bag_message->topic_name == cloud_topic) {
      sensor_msgs::msg::PointCloud2 cloud_msg;
      cloud_serialization.deserialize_message(&serialized, &cloud_msg);
      const uint64_t key =
        static_cast<uint64_t>(cloud_msg.header.stamp.sec) * 1000000000ull +
        cloud_msg.header.stamp.nanosec;
      const auto odom_it = pending_odoms.find(key);
      if (odom_it != pending_odoms.end()) {
        process_pair(odom_it->second, cloud_msg);
        pending_odoms.erase(odom_it);
      } else {
        pending_clouds[key] = cloud_msg;
      }
    }
  }

  RCLCPP_INFO(
    logger,
    "Offline run complete: %zu pairs, %zu submaps, %.1f m travelled, %zu loop edges "
    "(%zu odom / %zu cloud messages left unpaired)",
    paired_count, records.size(), accumulated_distance, edge_set.edges().size(),
    pending_odoms.size(), pending_clouds.size());

  // --- Final pose-graph optimization from raw odometry poses plus the
  // accumulated loop edges, exactly like the live doPoseAdjustment with
  // IMU / GNSS off.
  std::vector<graphslam::pose_graph::SubmapNode> submap_nodes;
  submap_nodes.reserve(records.size());
  for (const auto & record : records) {
    graphslam::pose_graph::SubmapNode submap_node;
    submap_node.pose = Eigen::Isometry3d(record.meta.pose.matrix());
    submap_nodes.push_back(submap_node);
  }
  std::vector<graphslam::pose_graph::LoopConstraint> loop_constraints;
  loop_constraints.reserve(edge_set.edges().size());
  for (const auto & edge : edge_set.edges()) {
    graphslam::pose_graph::LoopConstraint loop_constraint;
    loop_constraint.from = edge.pair_id.first;
    loop_constraint.to = edge.pair_id.second;
    loop_constraint.relative_pose = edge.relative_pose;
    loop_constraint.fitness_score = edge.fitness_score;
    loop_constraints.push_back(loop_constraint);
  }
  graphslam::pose_graph::AdjacentEdgeConfig adjacent_config;
  adjacent_config.num_adjacent_pose_constraints = num_adjacent_pose_constraints;
  adjacent_config.info_weight = adjacent_edge_info_weight;
  graphslam::pose_graph::LoopEdgeConfig loop_config;
  loop_config.info_weight = loop_edge_info_weight;
  loop_config.robust_kernel_type = loop_edge_robust_kernel_type;
  loop_config.robust_kernel_delta = loop_edge_robust_kernel_delta;
  graphslam::pose_graph::ImuEdgeConfig imu_config;

  std::vector<Eigen::Isometry3d> optimized_poses;
  optimized_poses.reserve(records.size());
  if (!records.empty()) {
    const graphslam::pose_graph::OptimizationResult result =
      graphslam::pose_graph::optimizePoseGraph(
      submap_nodes, loop_constraints, {}, {}, adjacent_config, loop_config, imu_config,
      graphslam::pose_graph::Chi2Collection::NONE);
    optimized_poses = result.poses;
  }

  // --- Deterministic outputs. loop_edges.csv is the Phase 2 hard-gate
  // artifact: same bag + same config must reproduce it byte-identically.
  {
    std::ofstream csv(output_dir + "/loop_edges.csv");
    csv << "from,to,fitness,tx,ty,tz,qx,qy,qz,qw\n";
    char line[512];
    for (const auto & edge : edge_set.edges()) {
      const Eigen::Vector3d t = edge.relative_pose.translation();
      const Eigen::Quaterniond q(edge.relative_pose.rotation());
      std::snprintf(
        line, sizeof(line), "%d,%d,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g",
        edge.pair_id.first, edge.pair_id.second, edge.fitness_score,
        t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w());
      csv << line << "\n";
    }
  }
  {
    std::vector<Eigen::Isometry3d> raw_poses;
    raw_poses.reserve(records.size());
    for (const auto & record : records) {
      raw_poses.push_back(Eigen::Isometry3d(record.meta.pose.matrix()));
    }
    writeTum(output_dir + "/trajectory_raw.tum", records, raw_poses);
    if (!optimized_poses.empty()) {
      writeTum(output_dir + "/trajectory_optimized.tum", records, optimized_poses);
    }
  }
  if (refine && !optimized_poses.empty()) {
    graphslam::map_refinement::MapRefinerConfig refiner_config;
    refiner_config.cloud_downsample_voxel = refine_cloud_downsample;
    refiner_config.pyramid.window_size = refine_window_size;
    refiner_config.pyramid.window_stride = refine_window_stride;

    std::vector<std::vector<Eigen::Vector3d>> local_clouds;
    local_clouds.reserve(records.size());
    for (const auto & record : records) {
      std::vector<Eigen::Vector3d> points;
      points.reserve(record.cloud->size());
      for (size_t i = 0; i < record.cloud->size(); ++i) {
        const auto & p = record.cloud->points[i];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {continue;}
        points.push_back(Eigen::Vector3d(p.x, p.y, p.z));
      }
      local_clouds.push_back(points);
    }
    std::vector<Eigen::Matrix4d> initial_matrices;
    initial_matrices.reserve(optimized_poses.size());
    for (const auto & pose : optimized_poses) {
      initial_matrices.push_back(pose.matrix());
    }

    const graphslam::map_refinement::MapRefinerResult refined =
      graphslam::map_refinement::refineSubmapPoses(
      local_clouds, initial_matrices, refiner_config);

    std::vector<Eigen::Isometry3d> refined_poses;
    refined_poses.reserve(refined.poses.size());
    for (const auto & pose : refined.poses) {
      refined_poses.push_back(Eigen::Isometry3d(pose));
    }
    writeTum(output_dir + "/trajectory_refined.tum", records, refined_poses);
    {
      std::ofstream report(output_dir + "/map_refinement_report.yaml");
      const std::vector<std::string> lines =
        graphslam::map_refinement::refinerReportYamlLines(refined, refiner_config);
      for (size_t i = 0; i < lines.size(); ++i) {
        report << lines[i] << "\n";
      }
    }
    RCLCPP_INFO(
      logger, "Refinement %s: %zu windows, status=%s",
      refined.accepted ? "accepted" : "rejected",
      refined.pyramid_result.windows.size(), refined.status.c_str());

    if (refine_save_maps) {
      // Before/after map PCDs for the map-quality gate stage. The clouds
      // are the runner's own deterministic submap clouds; only the poses
      // differ between the two maps.
      const auto save_map =
        [&](const std::vector<Eigen::Matrix4d> & poses, const std::string & path) {
          pcl::PointCloud<pcl::PointXYZ> map_cloud;
          for (size_t i = 0; i < local_clouds.size(); ++i) {
            const Eigen::Matrix3d rotation = poses[i].block<3, 3>(0, 0);
            const Eigen::Vector3d translation = poses[i].block<3, 1>(0, 3);
            for (size_t j = 0; j < local_clouds[i].size(); ++j) {
              const Eigen::Vector3d world = rotation * local_clouds[i][j] + translation;
              map_cloud.push_back(
                pcl::PointXYZ(
                  static_cast<float>(world.x()), static_cast<float>(world.y()),
                  static_cast<float>(world.z())));
            }
          }
          pcl::io::savePCDFileBinary(path, map_cloud);
        };
      save_map(initial_matrices, output_dir + "/map_optimized.pcd");
      save_map(refined.poses, output_dir + "/map_refined.pcd");
      RCLCPP_INFO(logger, "Wrote map_optimized.pcd and map_refined.pcd");
    }
  }

  degeneracy_sink.writeReport(output_dir, logger);

  RCLCPP_INFO(logger, "Wrote %s/loop_edges.csv and TUM trajectories", output_dir.c_str());

  rclcpp::shutdown();
  return 0;
}
