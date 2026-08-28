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

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
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
#include "graph_based_slam/backend_registration_preflight.hpp"
#include "graph_based_slam/degeneracy_diagnostics_csv.hpp"
#include "graph_based_slam/degeneracy_report_summary.hpp"
#include "graph_based_slam/dense_pose_correction.hpp"
#include "graph_based_slam/loop_search_schedule.hpp"
#include "graph_based_slam/map_refiner.hpp"
#include "graph_based_slam/map_quality_metrics.hpp"
#include "graph_based_slam/map_thickness_attribution.hpp"
#include "graph_based_slam/map_thickness_attribution_csv.hpp"
#include "graph_based_slam/plane_feature_association.hpp"
#include "graph_based_slam/plane_revisit_constraints.hpp"
#include "graph_based_slam/pointcloud2_conversion.hpp"
#include "graph_based_slam/pose_graph_optimization.hpp"
#include "graph_based_slam/probabilistic_surfel_map.hpp"
#include "graph_based_slam/registration_factory.hpp"
#include "graph_based_slam/registration_plugin_adapter.hpp"
#include "graph_based_slam/scan_surface_refiner.hpp"
#include "graph_based_slam/submap_creation.hpp"
#include "graph_based_slam/three_d_bbs_loop_verifier.hpp"
#include "graph_based_slam/trajectory_revisit_segmentation.hpp"
#include "lidarslam_default_plugins/ndt_omp_registration_impl.ipp"
#include "lidarslam_registration_loader/registration_plugin_loader.hpp"

namespace
{

using graphslam::backend_core::BackendCore;
using CloudPtr = BackendCore::CloudPtr;

std::shared_ptr<lidarslam::plugins::registration::RegistrationPlugin>
makeOfflineHostBuiltinNdtRegistration()
{
  // Keep this factory in the runner TU, just as the live component keeps its
  // factory in its own TU.  This is the host-resident path; linking the
  // default plugin DSO here would reintroduce the rejected ODR boundary.
  return std::make_shared<lidarslam_default_plugins::NdtOmpRegistration>();
}

struct SubmapRecord
{
  graphslam::backend_core::SubmapMeta meta;
  double stamp_sec{0.0};
  std::array<double, 36> odometry_covariance {};
  CloudPtr cloud;
};

struct ScanAttributionRecord
{
  double stamp_sec {0.0};
  Eigen::Isometry3d pose {Eigen::Isometry3d::Identity()};
  std::int64_t scan_id {0};
  std::int64_t submap_id {0};
  std::array<double, 36> odometry_covariance {};
  std::vector<Eigen::Vector3d> local_points;
};

double meanClampedCovarianceDiagonal(
  const std::array<double, 36> & covariance, const std::array<std::size_t, 3> & indices,
  const double fallback, const double maximum)
{
  double sum = 0.0;
  for (const std::size_t index : indices) {
    if (!std::isfinite(covariance[index]) || covariance[index] < 0.0) {return fallback;}
    sum += covariance[index];
  }
  const double mean = sum / 3.0;
  // A zero diagonal is the ROS "unknown" default, not perfect certainty.
  return mean > 0.0 ? std::min(mean, maximum) : fallback;
}

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

void loadFixedLoopEdges(
  const std::string & path,
  std::size_t submap_count,
  graphslam::backend_core::LoopEdgeSet & edge_set)
{
  std::ifstream input(path);
  if (!input.is_open()) {
    throw std::runtime_error("failed to open fixed loop edge CSV: " + path);
  }
  std::string line;
  if (!std::getline(input, line) || line != "from,to,fitness,tx,ty,tz,qx,qy,qz,qw") {
    throw std::runtime_error("invalid fixed loop edge CSV header: " + path);
  }
  int line_number = 1;
  while (std::getline(input, line)) {
    ++line_number;
    if (line.empty()) {
      continue;
    }
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) {
      fields.push_back(field);
    }
    if (fields.size() != 10) {
      throw std::runtime_error(
              "invalid fixed loop edge field count at line " + std::to_string(line_number));
    }
    graphslam::backend_core::LoopEdgeSet::Edge edge;
    try {
      edge.pair_id = {std::stoi(fields[0]), std::stoi(fields[1])};
      edge.fitness_score = std::stod(fields[2]);
      edge.relative_pose = Eigen::Isometry3d::Identity();
      edge.relative_pose.translation() = Eigen::Vector3d(
        std::stod(fields[3]), std::stod(fields[4]), std::stod(fields[5]));
      Eigen::Quaterniond quaternion(
        std::stod(fields[9]), std::stod(fields[6]), std::stod(fields[7]), std::stod(fields[8]));
      if (quaternion.norm() <= 0.0) {
        throw std::runtime_error("zero-norm quaternion");
      }
      quaternion.normalize();
      edge.relative_pose.linear() = quaternion.toRotationMatrix();
    } catch (const std::exception & error) {
      throw std::runtime_error(
              "invalid fixed loop edge at line " + std::to_string(line_number) + ": " +
              error.what());
    }
    if (
      edge.pair_id.first < 0 || edge.pair_id.second < 0 ||
      static_cast<std::size_t>(edge.pair_id.first) >= submap_count ||
      static_cast<std::size_t>(edge.pair_id.second) >= submap_count)
    {
      throw std::runtime_error(
              "fixed loop edge index out of range at line " + std::to_string(line_number));
    }
    if (!edge_set.upsert(edge)) {
      throw std::runtime_error(
              "fixed loop edge was invalid or deduplicated at line " +
              std::to_string(line_number));
    }
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
  std::string fixed_loop_edges_path;
  node->get_parameter_or("bag_path", bag_path, std::string());
  node->get_parameter_or("output_dir", output_dir, std::string("."));
  node->get_parameter_or("offline_odom_topic", odom_topic, std::string("/rko_lio/odometry"));
  node->get_parameter_or("offline_cloud_topic", cloud_topic, std::string("/rko_lio/frame"));
  node->get_parameter_or("fixed_loop_edges_path", fixed_loop_edges_path, std::string());
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

  // Report-only thickness attribution from every paired frontend scan. This
  // remains opt-in because retaining scan provenance increases replay memory.
  bool save_map_thickness_attribution = false;
  bool map_thickness_write_csv = false;
  double map_thickness_scan_voxel_size = 0.15;
  bool save_dense_pose_refined_map = false;
  double dense_pose_refined_scan_voxel_size = 0.10;
  double dense_pose_refined_output_voxel_size = 0.10;
  bool save_probabilistic_surfel_map = false;
  bool save_connected_surface_map = false;
  bool save_surface_consolidated_map = false;
  bool save_surfel_support_partition_maps = false;
  bool save_scan_persistence_filtered_map = false;
  bool save_visibility_aware_dynamic_map = false;
  bool save_scan_surface_refined_surfel_map = false;
  bool save_global_surface_ba_surfel_map = false;
  double probabilistic_surfel_scan_voxel_size = 0.05;
  graphslam::ProbabilisticSurfelMapConfig surfel_map_config;
  graphslam::scan_surface_refinement::ScanSurfaceRefinerConfig scan_surface_config;
  double surfel_pose_translation_fallback_variance_m2 = 1.0e-4;
  double surfel_pose_rotation_fallback_variance_rad2 = 1.0e-6;
  double surfel_pose_translation_max_variance_m2 = 1.0;
  double surfel_pose_rotation_max_variance_rad2 = 0.1;
  int probabilistic_surfel_input_scan_stride = 1;
  int probabilistic_surfel_input_scan_offset = 0;
  graphslam::map_thickness::RevisitSegmentationConfig revisit_config;
  node->get_parameter_or(
    "save_map_thickness_attribution", save_map_thickness_attribution, false);
  node->get_parameter_or("map_thickness_write_csv", map_thickness_write_csv, false);
  node->get_parameter_or(
    "map_thickness_scan_voxel_size", map_thickness_scan_voxel_size, 0.15);
  node->get_parameter_or("save_dense_pose_refined_map", save_dense_pose_refined_map, false);
  node->get_parameter_or(
    "dense_pose_refined_scan_voxel_size", dense_pose_refined_scan_voxel_size, 0.10);
  node->get_parameter_or(
    "dense_pose_refined_output_voxel_size", dense_pose_refined_output_voxel_size, 0.10);
  node->get_parameter_or(
    "save_probabilistic_surfel_map", save_probabilistic_surfel_map, false);
  node->get_parameter_or("save_connected_surface_map", save_connected_surface_map, false);
  node->get_parameter_or(
    "save_surface_consolidated_map", save_surface_consolidated_map, false);
  surfel_map_config.build_surface_consolidated_map = save_surface_consolidated_map;
  node->get_parameter_or(
    "surface_consolidation_min_projection_distance_m",
    surfel_map_config.surface_consolidation_min_projection_distance_m, 0.0);
  node->get_parameter_or(
    "save_surfel_support_partition_maps", save_surfel_support_partition_maps, false);
  surfel_map_config.build_support_partition_maps = save_surfel_support_partition_maps;
  node->get_parameter_or(
    "save_scan_persistence_filtered_map", save_scan_persistence_filtered_map, false);
  node->get_parameter_or(
    "save_visibility_aware_dynamic_map", save_visibility_aware_dynamic_map, false);
  node->get_parameter_or(
    "save_scan_surface_refined_surfel_map", save_scan_surface_refined_surfel_map, false);
  node->get_parameter_or(
    "save_global_surface_ba_surfel_map", save_global_surface_ba_surfel_map, false);
  node->get_parameter_or(
    "probabilistic_surfel_scan_voxel_size", probabilistic_surfel_scan_voxel_size, 0.05);
  node->get_parameter_or(
    "probabilistic_surfel_output_voxel_size", surfel_map_config.voxel_size_m, 0.10);
  node->get_parameter_or(
    "probabilistic_surfel_support_voxel_size",
    surfel_map_config.surfel_support_voxel_size_m, 0.30);
  node->get_parameter_or(
    "probabilistic_surfel_secondary_support_voxel_size",
    surfel_map_config.secondary_support_voxel_size_m, 0.0);
  node->get_parameter_or(
    "probabilistic_surfel_tertiary_support_voxel_size",
    surfel_map_config.tertiary_support_voxel_size_m, 0.0);
  node->get_parameter_or(
    "connected_surface_max_normal_angle_deg",
    surfel_map_config.connected_surface_max_normal_angle_deg, 8.0);
  node->get_parameter_or(
    "connected_surface_max_plane_distance_m",
    surfel_map_config.connected_surface_max_plane_distance_m, 0.04);
  int connected_surface_min_support_cells = 3;
  node->get_parameter_or(
    "connected_surface_min_support_cells", connected_surface_min_support_cells, 3);
  node->get_parameter_or(
    "connected_surface_extend_fallback",
    surfel_map_config.connected_surface_extend_fallback, true);
  node->get_parameter_or(
    "connected_surface_max_extension_distance_m",
    surfel_map_config.connected_surface_max_extension_distance_m, 0.04);
  int connected_surface_min_extension_support_cells = 2;
  node->get_parameter_or(
    "connected_surface_min_extension_support_cells",
    connected_surface_min_extension_support_cells, 2);
  surfel_map_config.connected_surface_min_extension_support_cells =
    static_cast<std::size_t>(std::max(2, connected_surface_min_extension_support_cells));
  surfel_map_config.build_connected_surface_map = save_connected_surface_map;
  surfel_map_config.connected_surface_min_support_cells = static_cast<std::size_t>(
    std::max(2, connected_surface_min_support_cells));
  node->get_parameter_or(
    "probabilistic_surfel_support_grid_phases",
    surfel_map_config.support_grid_phases, 1);
  node->get_parameter_or(
    "probabilistic_surfel_blend_support_phases",
    surfel_map_config.blend_support_phases, false);
  node->get_parameter_or(
    "probabilistic_surfel_support_phases_fallback_only",
    surfel_map_config.support_phases_fallback_only, false);
  int surfel_min_distinct_scans = 3;
  node->get_parameter_or(
    "probabilistic_surfel_min_distinct_scans", surfel_min_distinct_scans, 3);
  surfel_map_config.fusion.min_distinct_scans = static_cast<std::size_t>(
    std::max(2, surfel_min_distinct_scans));
  node->get_parameter_or(
    "probabilistic_surfel_max_small_eigenvalue_ratio",
    surfel_map_config.fusion.max_small_eigenvalue_ratio, 0.10);
  node->get_parameter_or(
    "probabilistic_surfel_min_middle_eigenvalue_ratio",
    surfel_map_config.fusion.min_middle_eigenvalue_ratio, 0.05);
  node->get_parameter_or(
    "probabilistic_surfel_base_range_sigma_m",
    surfel_map_config.fusion.base_range_sigma_m, 0.008);
  node->get_parameter_or(
    "probabilistic_surfel_range_sigma_per_meter",
    surfel_map_config.fusion.range_sigma_per_meter, 0.001);
  node->get_parameter_or(
    "probabilistic_surfel_tangential_sigma_m",
    surfel_map_config.fusion.tangential_sigma_m, 0.015);
  node->get_parameter_or(
    "probabilistic_surfel_huber_sigma", surfel_map_config.fusion.huber_sigma, 2.5);
  int persistence_min_distinct_scans = 3;
  int persistence_min_scan_span = 3;
  node->get_parameter_or(
    "scan_persistence_min_distinct_scans", persistence_min_distinct_scans, 3);
  node->get_parameter_or("scan_persistence_min_scan_span", persistence_min_scan_span, 3);
  node->get_parameter_or(
    "scan_persistence_max_filter_range_m",
    surfel_map_config.persistence_max_filter_range_m, 30.0);
  surfel_map_config.build_persistence_filtered_map = save_scan_persistence_filtered_map;
  surfel_map_config.persistence_min_distinct_scans = static_cast<std::size_t>(
    std::max(1, persistence_min_distinct_scans));
  surfel_map_config.persistence_min_scan_span = static_cast<std::uint64_t>(
    std::max(0, persistence_min_scan_span));
  int visibility_max_distinct_scans = 2;
  int visibility_max_scan_span = 2;
  int visibility_near_scan_offset = 5;
  int visibility_far_scan_offset = 15;
  int visibility_min_free_space_votes = 2;
  node->get_parameter_or(
    "visibility_dynamic_max_distinct_scans", visibility_max_distinct_scans, 2);
  node->get_parameter_or("visibility_dynamic_max_scan_span", visibility_max_scan_span, 2);
  node->get_parameter_or(
    "visibility_dynamic_near_scan_offset", visibility_near_scan_offset, 5);
  node->get_parameter_or(
    "visibility_dynamic_far_scan_offset", visibility_far_scan_offset, 15);
  node->get_parameter_or(
    "visibility_dynamic_min_free_space_votes", visibility_min_free_space_votes, 2);
  node->get_parameter_or(
    "visibility_dynamic_angular_resolution_rad",
    surfel_map_config.visibility_angular_resolution_rad, 0.004363323129985824);
  node->get_parameter_or(
    "visibility_dynamic_free_space_margin_m",
    surfel_map_config.visibility_free_space_margin_m, 0.50);
  node->get_parameter_or(
    "visibility_dynamic_max_range_m", surfel_map_config.visibility_max_range_m, 30.0);
  node->get_parameter_or(
    "visibility_dynamic_max_origin_displacement_m",
    surfel_map_config.visibility_max_origin_displacement_m, 3.0);
  surfel_map_config.build_visibility_filtered_map = save_visibility_aware_dynamic_map;
  surfel_map_config.visibility_max_distinct_scans = static_cast<std::size_t>(
    std::max(1, visibility_max_distinct_scans));
  surfel_map_config.visibility_max_scan_span = static_cast<std::uint64_t>(
    std::max(0, visibility_max_scan_span));
  const int sanitized_visibility_near_scan_offset = std::max(1, visibility_near_scan_offset);
  surfel_map_config.visibility_near_scan_offset = static_cast<std::size_t>(
    sanitized_visibility_near_scan_offset);
  surfel_map_config.visibility_far_scan_offset = static_cast<std::size_t>(
    std::max(sanitized_visibility_near_scan_offset, visibility_far_scan_offset));
  surfel_map_config.visibility_min_free_space_votes = static_cast<std::size_t>(
    std::max(1, std::min(255, visibility_min_free_space_votes)));
  int scan_surface_min_observations = 20;
  node->get_parameter_or(
    "scan_surface_refiner_scan_voxel_size",
    scan_surface_config.scan_downsample_voxel_size_m, 0.20);
  node->get_parameter_or(
    "scan_surface_refiner_support_voxel_size",
    scan_surface_config.support_voxel_size_m, 0.50);
  node->get_parameter_or(
    "scan_surface_refiner_cross_fit_scan_parity",
    scan_surface_config.cross_fit_scan_parity, false);
  node->get_parameter_or(
    "scan_surface_refiner_min_observations", scan_surface_min_observations, 20);
  node->get_parameter_or(
    "scan_surface_refiner_huber_delta_m", scan_surface_config.residual_huber_delta_m, 0.05);
  node->get_parameter_or(
    "scan_surface_refiner_measurement_sigma_floor_m",
    scan_surface_config.measurement_sigma_floor_m, 0.01);
  node->get_parameter_or(
    "scan_surface_refiner_absolute_prior_sigma_m",
    scan_surface_config.absolute_translation_prior_sigma_m, 0.02);
  node->get_parameter_or(
    "scan_surface_refiner_temporal_smoothness_sigma_m",
    scan_surface_config.temporal_smoothness_sigma_m, 0.01);
  node->get_parameter_or(
    "scan_surface_refiner_max_total_translation_correction_m",
    scan_surface_config.max_total_translation_correction_m, 0.01);
  scan_surface_config.min_surface_observations_per_scan = static_cast<std::size_t>(
    std::max(1, scan_surface_min_observations));
  scan_surface_config.fusion = surfel_map_config.fusion;
  node->get_parameter_or(
    "probabilistic_surfel_pose_translation_fallback_variance_m2",
    surfel_pose_translation_fallback_variance_m2, 1.0e-4);
  node->get_parameter_or(
    "probabilistic_surfel_pose_rotation_fallback_variance_rad2",
    surfel_pose_rotation_fallback_variance_rad2, 1.0e-6);
  node->get_parameter_or(
    "probabilistic_surfel_pose_translation_max_variance_m2",
    surfel_pose_translation_max_variance_m2, 1.0);
  node->get_parameter_or(
    "probabilistic_surfel_pose_rotation_max_variance_rad2",
    surfel_pose_rotation_max_variance_rad2, 0.1);
  node->get_parameter_or(
    "probabilistic_surfel_input_scan_stride",
    probabilistic_surfel_input_scan_stride, 1);
  node->get_parameter_or(
    "probabilistic_surfel_input_scan_offset",
    probabilistic_surfel_input_scan_offset, 0);
  probabilistic_surfel_input_scan_stride = std::max(
    1, probabilistic_surfel_input_scan_stride);
  probabilistic_surfel_input_scan_offset = std::max(
    0, std::min(
      probabilistic_surfel_input_scan_offset,
      probabilistic_surfel_input_scan_stride - 1));
  node->get_parameter_or(
    "map_thickness_revisit_match_radius_m", revisit_config.match_radius_m, 3.0);
  node->get_parameter_or(
    "map_thickness_revisit_min_prior_travel_m", revisit_config.min_prior_travel_m, 20.0);
  node->get_parameter_or(
    "map_thickness_revisit_exit_travel_m",
    revisit_config.exit_hysteresis_travel_m, 5.0);
  node->get_parameter_or(
    "map_thickness_revisit_min_epoch_travel_m",
    revisit_config.min_epoch_separation_m, 10.0);
  map_thickness_scan_voxel_size = std::max(0.0, map_thickness_scan_voxel_size);
  dense_pose_refined_scan_voxel_size = std::max(0.0, dense_pose_refined_scan_voxel_size);
  dense_pose_refined_output_voxel_size = std::max(0.0, dense_pose_refined_output_voxel_size);
  probabilistic_surfel_scan_voxel_size = std::max(0.0, probabilistic_surfel_scan_voxel_size);
  const bool retain_dense_scans =
    save_map_thickness_attribution || save_dense_pose_refined_map ||
    save_probabilistic_surfel_map || save_connected_surface_map ||
    save_surface_consolidated_map ||
    save_surfel_support_partition_maps ||
    save_scan_persistence_filtered_map ||
    save_visibility_aware_dynamic_map ||
    save_scan_surface_refined_surfel_map ||
    save_global_surface_ba_surfel_map;
  double retained_scan_voxel_size = map_thickness_scan_voxel_size;
  if (save_dense_pose_refined_map) {
    retained_scan_voxel_size = dense_pose_refined_scan_voxel_size;
  }
  if ((save_probabilistic_surfel_map || save_connected_surface_map ||
    save_surface_consolidated_map ||
    save_surfel_support_partition_maps ||
    save_scan_persistence_filtered_map ||
    save_visibility_aware_dynamic_map ||
    save_scan_surface_refined_surfel_map ||
    save_global_surface_ba_surfel_map) &&
    (!save_dense_pose_refined_map ||
    probabilistic_surfel_scan_voxel_size < retained_scan_voxel_size))
  {
    retained_scan_voxel_size = probabilistic_surfel_scan_voxel_size;
  }
  revisit_config.match_radius_m = std::max(0.0, revisit_config.match_radius_m);
  revisit_config.min_prior_travel_m = std::max(0.0, revisit_config.min_prior_travel_m);
  revisit_config.exit_hysteresis_travel_m = std::max(
    0.0, revisit_config.exit_hysteresis_travel_m);
  revisit_config.min_epoch_separation_m = std::max(
    0.0, revisit_config.min_epoch_separation_m);

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
  double refine_prior_translation_sigma = 0.30;
  double refine_prior_rotation_sigma_rad = 0.035;
  double refine_max_step_translation = 0.25;
  double refine_max_step_rotation_rad = 0.052;
  node->get_parameter_or("refine", refine, true);
  node->get_parameter_or("refine_cloud_downsample", refine_cloud_downsample, 0.10);
  node->get_parameter_or("refine_window_size", refine_window_size, 16);
  node->get_parameter_or("refine_window_stride", refine_window_stride, 8);
  node->get_parameter_or(
    "refine_prior_translation_sigma", refine_prior_translation_sigma, 0.30);
  node->get_parameter_or(
    "refine_prior_rotation_sigma_rad", refine_prior_rotation_sigma_rad, 0.035);
  node->get_parameter_or(
    "refine_max_step_translation", refine_max_step_translation, 0.25);
  node->get_parameter_or(
    "refine_max_step_rotation_rad", refine_max_step_rotation_rad, 0.052);
  bool refine_save_maps = false;
  node->get_parameter_or("refine_save_maps", refine_save_maps, false);

  // Optional long-baseline plane re-observation factors. Disabled by default
  // so historical deterministic outputs remain unchanged until explicitly
  // enabled for an A/B run.
  bool use_plane_revisit_constraints = false;
  double plane_revisit_cloud_downsample = 0.20;
  int plane_revisit_min_pose_separation = 5;
  int plane_revisit_max_constraints_per_feature = 4;
  double plane_revisit_normal_info_weight = 10.0;
  double plane_revisit_offset_info_weight = 10.0;
  double plane_revisit_root_voxel_size = 2.0;
  int plane_revisit_max_octree_depth = 1;
  double plane_revisit_max_plane_thickness = 0.08;
  double plane_revisit_min_planarity_ratio = 4.0;
  int plane_revisit_min_points_per_observation = 20;
  double plane_revisit_max_initial_normal_error_deg = 2.0;
  double plane_revisit_max_initial_offset_error_m = 0.03;
  node->get_parameter_or(
    "use_plane_revisit_constraints", use_plane_revisit_constraints, false);
  node->get_parameter_or(
    "plane_revisit_cloud_downsample", plane_revisit_cloud_downsample, 0.20);
  node->get_parameter_or(
    "plane_revisit_min_pose_separation", plane_revisit_min_pose_separation, 5);
  node->get_parameter_or(
    "plane_revisit_max_constraints_per_feature",
    plane_revisit_max_constraints_per_feature, 4);
  node->get_parameter_or(
    "plane_revisit_normal_info_weight", plane_revisit_normal_info_weight, 10.0);
  node->get_parameter_or(
    "plane_revisit_offset_info_weight", plane_revisit_offset_info_weight, 10.0);
  node->get_parameter_or(
    "plane_revisit_root_voxel_size", plane_revisit_root_voxel_size, 2.0);
  node->get_parameter_or(
    "plane_revisit_max_octree_depth", plane_revisit_max_octree_depth, 1);
  node->get_parameter_or(
    "plane_revisit_max_plane_thickness", plane_revisit_max_plane_thickness, 0.08);
  node->get_parameter_or(
    "plane_revisit_min_planarity_ratio", plane_revisit_min_planarity_ratio, 4.0);
  node->get_parameter_or(
    "plane_revisit_min_points_per_observation",
    plane_revisit_min_points_per_observation, 20);
  node->get_parameter_or(
    "plane_revisit_max_initial_normal_error_deg",
    plane_revisit_max_initial_normal_error_deg, 2.0);
  node->get_parameter_or(
    "plane_revisit_max_initial_offset_error_m",
    plane_revisit_max_initial_offset_error_m, 0.03);

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
  search_config.target_voxel_leaf_size = voxel_leaf_size;
  int loop_search_query_stride = 1;
  node->get_parameter_or("loop_search_query_stride", loop_search_query_stride, 1);
  loop_search_query_stride = std::max(1, loop_search_query_stride);
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
    "scan_context_query_stride", aggregator.scan_context_query_stride, 1);
  aggregator.scan_context_query_stride = std::max(1, aggregator.scan_context_query_stride);
  node->get_parameter_or(
    "scan_context_exclude_recent", aggregator.scan_context_exclude_recent,
    graphslam::ScanContext::EXCLUDE_RECENT);
  aggregator.scan_context_exclude_recent = std::max(1, aggregator.scan_context_exclude_recent);
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
  node->get_parameter_or("loop_min_overlap_ratio", gates.min_overlap_ratio, 0.0);
  node->get_parameter_or(
    "loop_min_overlap_ratio_large_correction",
    gates.min_overlap_ratio_large_correction, 0.0);
  node->get_parameter_or(
    "loop_overlap_large_correction_translation_m",
    gates.overlap_large_correction_translation_m, 0.0);
  node->get_parameter_or(
    "loop_overlap_max_distance_m", gates.overlap_max_distance_m, 0.5);
  node->get_parameter_or(
    "loop_max_translation_delta_descriptor", gates.max_translation_descriptor_m, -1.0);
  node->get_parameter_or(
    "loop_max_rotation_delta_deg_descriptor", gates.max_rotation_descriptor_deg, -1.0);

  int loop_edge_dedup_index_window;
  int num_adjacent_pose_constraints;
  double adjacent_edge_info_weight;
  bool use_degeneracy_covariance_weighting;
  double degeneracy_adjacent_edge_information_scale;
  double non_observable_adjacent_edge_information_scale;
  double loop_edge_info_weight;
  double loop_edge_robust_kernel_delta;
  std::string loop_edge_robust_kernel_type;
  node->get_parameter_or("loop_edge_dedup_index_window", loop_edge_dedup_index_window, 8);
  node->get_parameter_or("num_adjacent_pose_cnstraints", num_adjacent_pose_constraints, 5);
  node->get_parameter_or("adjacent_edge_info_weight", adjacent_edge_info_weight, 1000.0);
  node->get_parameter_or(
    "use_degeneracy_covariance_weighting", use_degeneracy_covariance_weighting, false);
  node->get_parameter_or(
    "degeneracy_adjacent_edge_information_scale",
    degeneracy_adjacent_edge_information_scale, 0.25);
  node->get_parameter_or(
    "non_observable_adjacent_edge_information_scale",
    non_observable_adjacent_edge_information_scale, 0.05);
  node->get_parameter_or("loop_edge_info_weight", loop_edge_info_weight, 100.0);
  node->get_parameter_or("loop_edge_robust_kernel_delta", loop_edge_robust_kernel_delta, 1.0);
  node->get_parameter_or(
    "loop_edge_robust_kernel_type", loop_edge_robust_kernel_type, std::string("huber"));

  using RegistrationPlugin = lidarslam::plugins::registration::RegistrationPlugin;
  using RegistrationPluginSession =
    lidarslam::plugins::registration::shell::RegistrationPluginSession;
  using RegistrationPluginSessionAdapter =
    lidarslam::plugins::registration::shell::RegistrationPluginSessionAdapter;
  using RegistrationResolver = lidarslam::plugins::registration::shell::RegistrationResolver;

  // The resolver and session are shell-owned.  Declaration order is
  // deliberate: the plugin owner is released before the session, and the
  // session remains alive until all typed BackendCore calls have completed.
  std::unique_ptr<RegistrationResolver> registration_resolver;
  std::shared_ptr<RegistrationPluginSession> registration_plugin_session;
  std::shared_ptr<RegistrationPlugin> registration_plugin_owner;
  boost::shared_ptr<pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>> legacy_registration;
  std::shared_ptr<graphslam::backend_registration::PclRegistrationAdapter>
  legacy_registration_bridge;
  RegistrationPlugin * registration_plugin = nullptr;
  graphslam::backend_registration::BackendRegistrationRequest backend_request;

  if (registration_method == "NDT") {
    graphslam::backend_registration::NdtConfig ndt_config;
    ndt_config.resolution = ndt_resolution;
    ndt_config.num_threads = ndt_num_threads;
    ndt_config.target_cell_cache_capacity =
      graphslam::backend_registration::kBackendNdtTargetCellCacheCapacity;
    std::string preflight_error;
    if (!graphslam::backend_registration::makeNdtLoadRequest(
        ndt_config, &backend_request, &preflight_error))
    {
      RCLCPP_ERROR(logger, "backend NDT preflight request failed: %s", preflight_error.c_str());
      rclcpp::shutdown();
      return 1;
    }

    auto host_ndt = graphslam::backend_registration::makeNdtHostBuiltinRegistration(
      []() {
        return makeOfflineHostBuiltinNdtRegistration();
      });
    registration_resolver.reset(new RegistrationResolver({host_ndt}));
    const auto loaded = registration_resolver->resolve(backend_request.request);
    if (!loaded.ok()) {
      RCLCPP_ERROR(
        logger,
        "backend NDT startup preflight failed role=%s class=%s code=%d: %s",
        backend_request.role.c_str(), backend_request.request.class_id.c_str(),
        static_cast<int>(loaded.failure.code), loaded.failure.message.c_str());
      rclcpp::shutdown();
      return 1;
    }
    // Use the same host-owned startup transaction as the live backend.  The
    // offline shell has no ROS publishers, but it still must not expose a
    // partially activated session if a future adapter validation step fails.
    using lidarslam::plugins::registration::shell::LoadFailure;
    using lidarslam::plugins::registration::shell::RegistrationActivationSlots;
    using lidarslam::plugins::registration::shell::RegistrationActivationTransaction;
    RegistrationActivationSlots activation_slots;
    activation_slots.session = &registration_plugin_session;
    activation_slots.plugin = &registration_plugin_owner;
    RegistrationActivationTransaction activation(activation_slots);
    LoadFailure activation_failure;
    if (!activation.prepare(loaded.session, &activation_failure) ||
      !activation.validate(
        [](const lidarslam::plugins::registration::shell::
        RegistrationActivationSnapshot & candidate)
        {
          if (!candidate.session || !candidate.plugin) {
            return LoadFailure{
            lidarslam::plugins::registration::shell::LoadFailureCode::kInvalidRequest,
            "offline registration activation candidate is null"};
          }
          return LoadFailure{};
        }, &activation_failure) ||
      !activation.commit(&activation_failure))
    {
      RCLCPP_ERROR(
        logger, "backend registration activation failed: %s", activation_failure.message.c_str());
      rclcpp::shutdown();
      return 1;
    }
    registration_plugin = registration_plugin_owner.get();

    const std::filesystem::path receipt_path =
      std::filesystem::path(output_dir) / "registration_plugin_receipt.yaml";
    std::ofstream receipt(receipt_path);
    std::string receipt_error;
    if (!receipt.is_open() || !graphslam::backend_registration::writeBackendRegistrationReceipt(
        receipt, backend_request, *registration_plugin_session, &receipt_error))
    {
      RCLCPP_ERROR(
        logger, "backend registration receipt failed before bag processing: %s",
        receipt_error.empty() ? "cannot open receipt" : receipt_error.c_str());
      rclcpp::shutdown();
      return 1;
    }
    const std::string capability_bits = std::to_string(
      registration_plugin_session->capabilities().bits());
    RCLCPP_INFO(
      logger,
      "backend registration preflight resolved role=%s backend_kind=%s class=%s "
      "api=%u.%u license=%s capabilities=%s library=%s manifest=%s",
      backend_request.role.c_str(),
      lidarslam::plugins::registration::shell::backendKindName(
        registration_plugin_session->backendKind()),
      registration_plugin_session->classId().c_str(),
      static_cast<unsigned int>(registration_plugin_session->metadata().api_version.major),
      static_cast<unsigned int>(registration_plugin_session->metadata().api_version.minor),
      registration_plugin_session->metadata().license.c_str(),
      capability_bits.c_str(),
      registration_plugin_session->libraryPath().empty() ? "<host>" :
      registration_plugin_session->libraryPath().c_str(),
      registration_plugin_session->pluginManifestPath().empty() ? "<host>" :
      registration_plugin_session->pluginManifestPath().c_str());
  } else if (registration_method == "GICP") {
    // Preserve the historical PCL construction/configuration, then adopt the
    // bridge into the common session boundary.  The host namespace is
    // intentional: this path must never reinterpret a built-in as a
    // pluginlib class or silently fall back to NDT.
    legacy_registration = graphslam::backend_core::makeLegacyGicpRegistration();
    if (!legacy_registration) {
      RCLCPP_ERROR(logger, "backend GICP legacy construction failed; no fallback is allowed");
      rclcpp::shutdown();
      return 1;
    }
    legacy_registration_bridge =
      std::make_shared<graphslam::backend_registration::PclRegistrationAdapter>(
      legacy_registration, "lidarslam_builtin/LegacyBackendGicp");
    const std::shared_ptr<RegistrationPlugin> candidate_owner = legacy_registration_bridge;
    lidarslam::plugins::registration::shell::LoadRequest request;
    request.class_id = "lidarslam_builtin/LegacyBackendGicp";
    request.capabilities.require_initial_guess = true;
    request.capabilities.require_aligned_source = true;
    request.capabilities.require_target_policy = true;
    request.capabilities.target_policy =
      lidarslam::plugins::registration::TargetPolicy::kAcceptHostPrepared;
    request.capabilities.require_correspondence_metric = true;
    request.capabilities.correspondence_metric =
      lidarslam::plugins::registration::CorrespondenceMetric::kSquareRootFitnessProxy;
    request.enforce_permissive_license = true;
    using lidarslam::plugins::registration::shell::LoadFailure;
    using lidarslam::plugins::registration::shell::RegistrationActivationSlots;
    using lidarslam::plugins::registration::shell::RegistrationActivationTransaction;
    LoadFailure session_failure;
    const auto candidate_session =
      RegistrationPluginSession::createHostSession(
      candidate_owner, request, "", false, &session_failure);
    if (!candidate_session) {
      RCLCPP_ERROR(
        logger, "backend GICP host session adoption failed: %s", session_failure.message.c_str());
      rclcpp::shutdown();
      return 1;
    }
    RegistrationActivationSlots activation_slots;
    activation_slots.session = &registration_plugin_session;
    activation_slots.plugin = &registration_plugin_owner;
    RegistrationActivationTransaction activation(activation_slots);
    LoadFailure activation_failure;
    if (!activation.prepare(candidate_session, &activation_failure) ||
      !activation.validate(
        [](const lidarslam::plugins::registration::shell::
        RegistrationActivationSnapshot & candidate)
        {
          if (!candidate.session || !candidate.plugin) {
            return LoadFailure{
            lidarslam::plugins::registration::shell::LoadFailureCode::kInvalidRequest,
            "offline backend GICP activation candidate is null"};
          }
          return LoadFailure{};
        }, &activation_failure) ||
      !activation.commit(&activation_failure))
    {
      RCLCPP_ERROR(
        logger, "backend GICP activation failed: %s", activation_failure.message.c_str());
      rclcpp::shutdown();
      return 1;
    }
    registration_plugin = registration_plugin_owner.get();
    RCLCPP_INFO(
      logger,
      "backend registration preflight resolved role=%s backend_kind=%s "
      "class=%s api=%u.%u license=%s "
      "library=<host> manifest=<host>",
      graphslam::backend_registration::kBackendRegistrationRole,
      lidarslam::plugins::registration::shell::backendKindName(
        registration_plugin_session->backendKind()),
      registration_plugin_session->classId().c_str(),
      static_cast<unsigned int>(registration_plugin_session->metadata().api_version.major),
      static_cast<unsigned int>(registration_plugin_session->metadata().api_version.minor),
      registration_plugin_session->metadata().license.c_str());
  } else {
    RCLCPP_ERROR(
      logger, "invalid registration_method='%s'; backend supports only NDT and GICP "
      "with no fallback", registration_method.c_str());
    rclcpp::shutdown();
    return 1;
  }

  if (registration_plugin == nullptr) {
    RCLCPP_ERROR(logger, "backend registration preflight produced no typed plugin");
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
      if (!fixed_loop_edges_path.empty()) {
        next_query_idx = num_submaps;
        return;
      }
      while (next_query_idx < num_submaps) {
        const int query_idx = next_query_idx;
        core.ingestDescriptors(query_idx + 1, filtered_local_provider);
        if (!graphslam::loop_search_schedule::shouldSearch(
            query_idx, loop_search_query_stride))
        {
          next_query_idx = query_idx + 1;
          continue;
        }
        std::vector<graphslam::backend_core::SubmapMeta> visible;
        visible.reserve(query_idx + 1);
        for (int i = 0; i <= query_idx; ++i) {
          visible.push_back(records[i].meta);
        }
        std::unique_ptr<RegistrationPluginSessionAdapter> registration_session_adapter;
        RegistrationPlugin * registration_for_search = registration_plugin;
        if (registration_plugin_session != nullptr) {
          registration_session_adapter.reset(
            new RegistrationPluginSessionAdapter(*registration_plugin_session));
          registration_for_search = registration_session_adapter.get();
        }
        const graphslam::backend_core::LoopSearchOutput output = core.searchLoopForSubmap(
          visible, query_idx, search_config, raw_cloud_provider, *registration_for_search,
        voxelgrid,
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
  std::size_t attribution_raw_finite_points = 0U;
  std::size_t attribution_retained_points = 0U;
  std::vector<ScanAttributionRecord> attribution_scans;

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
      CloudPtr scan_cloud;
      Eigen::Affine3d pose_affine = Eigen::Affine3d::Identity();
      if (decision.create || retain_dense_scans) {
        tf2::fromMsg(odom.pose.pose, pose_affine);
      }
      if (decision.create) {
        scan_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
        graphslam::pointcloud2_conversion::fromRosMsgPointXYZI(cloud_msg, *scan_cloud);
      }
      if (decision.create) {
        accumulated_distance += decision.distance;
        last_position = position;
        last_position_valid = true;

        SubmapRecord record;
        record.meta.pose = pose_affine;
        record.meta.travel_distance = accumulated_distance;
        record.meta.content_revision = graphslam::backend_core::targetCloudContentRevision(
          *scan_cloud);
        record.stamp_sec = rclcpp::Time(odom.header.stamp).seconds();
        record.odometry_covariance = odom.pose.covariance;
        record.cloud = scan_cloud;
        records.push_back(record);

        drain_queries();
      }

      if (retain_dense_scans && !records.empty()) {
        pcl::PointCloud<pcl::PointXYZ> attribution_cloud;
        pcl::fromROSMsg(cloud_msg, attribution_cloud);
        std::vector<Eigen::Vector3d> finite_points;
        finite_points.reserve(attribution_cloud.size());
        for (const auto & point : attribution_cloud.points) {
          if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
            finite_points.emplace_back(point.x, point.y, point.z);
          }
        }
        attribution_raw_finite_points += finite_points.size();
        ScanAttributionRecord scan;
        scan.stamp_sec = rclcpp::Time(odom.header.stamp).seconds();
        scan.pose = Eigen::Isometry3d(pose_affine.matrix());
        scan.scan_id = static_cast<std::int64_t>(paired_count - 1U);
        scan.submap_id = static_cast<std::int64_t>(records.size() - 1U);
        scan.odometry_covariance = odom.pose.covariance;
        scan.local_points = graphslam::map_quality::downsampleByVoxelCentroid(
          finite_points, retained_scan_voxel_size);
        attribution_retained_points += scan.local_points.size();
        attribution_scans.push_back(std::move(scan));
      }
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

  if (!fixed_loop_edges_path.empty()) {
    try {
      loadFixedLoopEdges(fixed_loop_edges_path, records.size(), edge_set);
      RCLCPP_INFO(
        logger, "Loaded %zu fixed loop edges from %s; descriptor search was skipped",
        edge_set.edges().size(), fixed_loop_edges_path.c_str());
    } catch (const std::exception & error) {
      RCLCPP_ERROR(logger, "%s", error.what());
      rclcpp::shutdown();
      return 1;
    }
  }

  RCLCPP_INFO(
    logger,
    "Offline run complete: %zu pairs, %zu submaps, %.1f m travelled, %zu loop edges "
    "(%zu odom / %zu cloud messages left unpaired)",
    paired_count, records.size(), accumulated_distance, edge_set.edges().size(),
    pending_odoms.size(), pending_clouds.size());

  if (save_map_thickness_attribution) {
    std::vector<Eigen::Vector3d> scan_positions;
    scan_positions.reserve(attribution_scans.size());
    for (const auto & scan : attribution_scans) {
      scan_positions.push_back(scan.pose.translation());
    }
    const graphslam::map_thickness::RevisitSegmentationResult revisit_result =
      graphslam::map_thickness::segmentTrajectoryRevisits(scan_positions, revisit_config);

    std::size_t retained_points = 0U;
    for (const auto & scan : attribution_scans) {
      retained_points += scan.local_points.size();
    }
    std::vector<graphslam::map_thickness::AttributedPoint> attributed_points;
    attributed_points.reserve(retained_points);
    for (std::size_t scan_index = 0; scan_index < attribution_scans.size(); ++scan_index) {
      const auto & scan = attribution_scans[scan_index];
      for (const Eigen::Vector3d & local_point : scan.local_points) {
        graphslam::map_thickness::AttributedPoint point;
        point.position = scan.pose * local_point;
        point.scan_id = scan.scan_id;
        point.submap_id = scan.submap_id;
        point.revisit_id = revisit_result.revisit_ids[scan_index];
        attributed_points.push_back(point);
      }
    }

    graphslam::map_thickness::AttributionConfig attribution_config;
    const graphslam::map_thickness::AttributionReport attribution_report =
      graphslam::map_thickness::computeAttribution(attributed_points, attribution_config);
    const std::string report_path = output_dir + "/map_thickness_attribution_raw.yaml";
    std::ofstream report_output(report_path);
    for (const std::string & line : graphslam::map_thickness::reportYamlLines(
        attribution_report, attribution_config))
    {
      report_output << line << "\n";
    }
    report_output << "map_thickness_provenance:\n";
    report_output << std::setprecision(17);
    report_output << "  coordinate_source: dense_frontend_odometry\n";
    report_output << "  paired_scans: " << attribution_scans.size() << "\n";
    report_output << "  physical_submap_count: " << records.size() << "\n";
    report_output << "  raw_finite_points: " << attribution_raw_finite_points << "\n";
    report_output << "  retained_points: " << retained_points << "\n";
    report_output << "  scan_voxel_size_m: " << retained_scan_voxel_size << "\n";
    report_output << "  revisit_segmentation: spatial_trajectory_overlap_v1\n";
    report_output << "  revisit_epoch_count: " << revisit_result.revisit_epoch_count << "\n";
    report_output << "  revisit_matched_scans: " << revisit_result.matched_scan_count << "\n";
    report_output << "  revisit_epoch_start_scan_ids: [";
    for (std::size_t i = 0; i < revisit_result.epoch_start_indices.size(); ++i) {
      if (i > 0U) {
        report_output << ", ";
      }
      report_output << attribution_scans[revisit_result.epoch_start_indices[i]].scan_id;
    }
    report_output << "]\n";
    report_output << "  revisit_epoch_start_travel_m: [";
    for (std::size_t i = 0; i < revisit_result.epoch_start_indices.size(); ++i) {
      if (i > 0U) {
        report_output << ", ";
      }
      report_output << revisit_result.cumulative_travel_m[
        revisit_result.epoch_start_indices[i]];
    }
    report_output << "]\n";
    report_output << "  revisit_epoch_start_positions_m: [";
    for (std::size_t i = 0; i < revisit_result.epoch_start_indices.size(); ++i) {
      if (i > 0U) {
        report_output << ", ";
      }
      const Eigen::Vector3d & position = scan_positions[
        revisit_result.epoch_start_indices[i]];
      report_output << "[" << position.x() << ", " << position.y() << ", " <<
        position.z() << "]";
    }
    report_output << "]\n";
    report_output << "  dense_trajectory_travel_m: " <<
      (revisit_result.cumulative_travel_m.empty() ?
    0.0 : revisit_result.cumulative_travel_m.back()) << "\n";
    report_output << "  revisit_match_radius_m: " << revisit_config.match_radius_m << "\n";
    report_output << "  revisit_min_prior_travel_m: " <<
      revisit_config.min_prior_travel_m << "\n";
    report_output << "  revisit_exit_travel_m: " <<
      revisit_config.exit_hysteresis_travel_m << "\n";
    report_output << "  revisit_min_epoch_travel_m: " <<
      revisit_config.min_epoch_separation_m << "\n";

    if (map_thickness_write_csv) {
      std::ofstream csv(output_dir + "/map_thickness_attributed_points.csv");
      csv << std::setprecision(17);
      csv << graphslam::map_thickness::attributedPointCsvHeader() << "\n";
      for (const auto & point : attributed_points) {
        csv << point.position.x() << "," << point.position.y() << "," <<
          point.position.z() << "," << point.scan_id << "," << point.submap_id << "," <<
          point.revisit_id << "\n";
      }
    }
    RCLCPP_INFO(
      logger,
      "Wrote raw thickness attribution from %zu scans, %zu retained points, %d epochs to %s",
      attribution_scans.size(), retained_points, revisit_result.revisit_epoch_count,
      report_path.c_str());
  }

  // --- Final pose-graph optimization from raw odometry poses plus the
  // accumulated loop edges, exactly like the live doPoseAdjustment with
  // IMU / GNSS off.
  std::vector<graphslam::pose_graph::SubmapNode> submap_nodes;
  submap_nodes.reserve(records.size());
  for (const auto & record : records) {
    graphslam::pose_graph::SubmapNode submap_node;
    submap_node.pose = Eigen::Isometry3d(record.meta.pose.matrix());
    submap_node.odometry_covariance = record.odometry_covariance;
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
  adjacent_config.covariance_weighting.enabled = use_degeneracy_covariance_weighting;
  adjacent_config.covariance_weighting.degenerate_information_scale = std::max(
    0.0, std::min(1.0, degeneracy_adjacent_edge_information_scale));
  adjacent_config.covariance_weighting.non_observable_information_scale = std::max(
    0.0, std::min(
      adjacent_config.covariance_weighting.degenerate_information_scale,
      non_observable_adjacent_edge_information_scale));
  graphslam::pose_graph::LoopEdgeConfig loop_config;
  loop_config.info_weight = loop_edge_info_weight;
  loop_config.robust_kernel_type = loop_edge_robust_kernel_type;
  loop_config.robust_kernel_delta = loop_edge_robust_kernel_delta;
  graphslam::pose_graph::ImuEdgeConfig imu_config;

  std::vector<std::vector<Eigen::Vector3d>> local_clouds;
  if (use_plane_revisit_constraints || refine) {
    local_clouds.reserve(records.size());
    for (const auto & record : records) {
      std::vector<Eigen::Vector3d> points;
      points.reserve(record.cloud->size());
      for (const auto & point : record.cloud->points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
          continue;
        }
        points.emplace_back(point.x, point.y, point.z);
      }
      local_clouds.push_back(std::move(points));
    }
  }

  graphslam::pose_graph::PlaneRevisitBuilderResult plane_revisit_result;
  graphslam::pose_graph::PlaneRevisitGateResult plane_revisit_gate_result;
  graphslam::map_refinement::AssociationResult plane_revisit_association;
  if (use_plane_revisit_constraints && !local_clouds.empty()) {
    std::vector<std::vector<Eigen::Vector3d>> downsampled_clouds;
    downsampled_clouds.reserve(local_clouds.size());
    for (const auto & cloud : local_clouds) {
      downsampled_clouds.push_back(
        graphslam::map_quality::downsampleByVoxelCentroid(
          cloud, plane_revisit_cloud_downsample));
    }
    std::vector<Eigen::Matrix4d> pose_matrices;
    pose_matrices.reserve(submap_nodes.size());
    for (const auto & node_data : submap_nodes) {
      pose_matrices.push_back(node_data.pose.matrix());
    }
    graphslam::map_refinement::AssociationConfig association_config;
    association_config.min_observing_poses = 2;
    association_config.min_points_per_observation =
      plane_revisit_min_points_per_observation;
    association_config.extraction.root_voxel_size = plane_revisit_root_voxel_size;
    association_config.extraction.max_octree_depth = plane_revisit_max_octree_depth;
    association_config.extraction.max_plane_thickness =
      plane_revisit_max_plane_thickness;
    association_config.extraction.min_planarity_ratio =
      plane_revisit_min_planarity_ratio;
    plane_revisit_association = graphslam::map_refinement::associatePlaneFeatures(
      downsampled_clouds, pose_matrices, association_config);
    graphslam::pose_graph::PlaneRevisitBuilderConfig builder_config;
    builder_config.min_pose_separation = plane_revisit_min_pose_separation;
    builder_config.max_constraints_per_feature =
      plane_revisit_max_constraints_per_feature;
    builder_config.normal_info_weight = plane_revisit_normal_info_weight;
    builder_config.offset_info_weight = plane_revisit_offset_info_weight;
    plane_revisit_result = graphslam::pose_graph::buildPlaneRevisitConstraints(
      plane_revisit_association.features, builder_config);
    std::vector<Eigen::Isometry3d> initial_poses;
    initial_poses.reserve(submap_nodes.size());
    for (const auto & node_data : submap_nodes) {
      initial_poses.push_back(node_data.pose);
    }
    plane_revisit_gate_result =
      graphslam::pose_graph::gatePlaneRevisitConstraintsByInitialResidual(
      plane_revisit_result.constraints, initial_poses,
      plane_revisit_max_initial_normal_error_deg,
      plane_revisit_max_initial_offset_error_m);
    plane_revisit_result.constraints = plane_revisit_gate_result.constraints;
    RCLCPP_INFO(
      logger,
      "Plane revisit association: %d/%d patches, %zu constraints (%d rejected observations)",
      plane_revisit_association.patches_used, plane_revisit_association.patches_total,
      plane_revisit_result.constraints.size(), plane_revisit_result.observations_rejected);
  }

  std::vector<Eigen::Isometry3d> optimized_poses;
  optimized_poses.reserve(records.size());
  if (!records.empty()) {
    const graphslam::pose_graph::OptimizationResult result =
      graphslam::pose_graph::optimizePoseGraph(
      submap_nodes, loop_constraints, {}, {}, adjacent_config, loop_config, imu_config,
      graphslam::pose_graph::Chi2Collection::NONE, true, 10, std::string(),
      plane_revisit_result.constraints);
    optimized_poses = result.poses;
    if (use_plane_revisit_constraints) {
      std::ofstream report(output_dir + "/plane_revisit_report.yaml");
      report << "plane_revisit:\n";
      report << "  enabled: true\n";
      report << "  root_voxel_size_m: " << plane_revisit_root_voxel_size << "\n";
      report << "  max_octree_depth: " << plane_revisit_max_octree_depth << "\n";
      report << "  max_plane_thickness_m: " << plane_revisit_max_plane_thickness << "\n";
      report << "  min_planarity_ratio: " << plane_revisit_min_planarity_ratio << "\n";
      report << "  min_points_per_observation: " <<
        plane_revisit_min_points_per_observation << "\n";
      report << "  max_initial_normal_error_deg: " <<
        plane_revisit_max_initial_normal_error_deg << "\n";
      report << "  max_initial_offset_error_m: " <<
        plane_revisit_max_initial_offset_error_m << "\n";
      report << "  patches_total: " << plane_revisit_association.patches_total << "\n";
      report << "  patches_used: " << plane_revisit_association.patches_used << "\n";
      report << "  points_used: " << plane_revisit_association.points_used << "\n";
      report << "  features_seen: " << plane_revisit_result.features_seen << "\n";
      report << "  features_with_constraints: " <<
        plane_revisit_result.features_with_constraints << "\n";
      report << "  observations_seen: " << plane_revisit_result.observations_seen << "\n";
      report << "  observations_rejected: " << plane_revisit_result.observations_rejected << "\n";
      report << "  max_pose_separation: " <<
        plane_revisit_result.max_pose_separation << "\n";
      report << "  candidate_constraints: " <<
        plane_revisit_result.constraints.size() + plane_revisit_gate_result.rejected << "\n";
      report << "  constraints_rejected_initial_residual: " <<
        plane_revisit_gate_result.rejected << "\n";
      report << "  accepted_max_initial_normal_error_deg: " <<
        plane_revisit_gate_result.accepted_max_normal_error_deg << "\n";
      report << "  accepted_max_initial_offset_error_m: " <<
        plane_revisit_gate_result.accepted_max_offset_error_m << "\n";
      report << "  constraints: " << result.plane_revisit_edges << "\n";
      report << "  chi2_before: " << result.plane_revisit_chi2_before << "\n";
      report << "  chi2_after: " << result.plane_revisit_chi2_after << "\n";
    }
  }

  if ((save_dense_pose_refined_map || save_probabilistic_surfel_map ||
    save_connected_surface_map || save_surface_consolidated_map ||
    save_surfel_support_partition_maps ||
    save_scan_persistence_filtered_map || save_visibility_aware_dynamic_map ||
    save_scan_surface_refined_surfel_map ||
    save_global_surface_ba_surfel_map) &&
    !optimized_poses.empty() && !attribution_scans.empty())
  {
    using graphslam::dense_pose_correction::TimedPose;
    std::vector<TimedPose> raw_anchors;
    std::vector<TimedPose> corrected_anchors;
    raw_anchors.reserve(records.size());
    corrected_anchors.reserve(records.size());
    for (std::size_t i = 0; i < records.size(); ++i) {
      TimedPose raw_anchor;
      raw_anchor.stamp_sec = records[i].stamp_sec;
      raw_anchor.pose = Eigen::Isometry3d(records[i].meta.pose.matrix());
      raw_anchors.push_back(raw_anchor);
      TimedPose corrected_anchor;
      corrected_anchor.stamp_sec = records[i].stamp_sec;
      corrected_anchor.pose = optimized_poses[i];
      corrected_anchors.push_back(corrected_anchor);
    }
    const auto correction_anchors =
      graphslam::dense_pose_correction::buildCorrectionAnchors(
      raw_anchors, corrected_anchors);

    std::vector<TimedPose> dense_raw_poses;
    dense_raw_poses.reserve(attribution_scans.size());
    for (const auto & scan : attribution_scans) {
      TimedPose sample;
      sample.stamp_sec = scan.stamp_sec;
      sample.pose = scan.pose;
      dense_raw_poses.push_back(sample);
    }
    const std::vector<TimedPose> dense_corrected_poses =
      graphslam::dense_pose_correction::applyDenseCorrections(
      dense_raw_poses, correction_anchors);

    const auto write_dense_tum =
      [](const std::string & path, const std::vector<TimedPose> & poses) {
        std::ofstream output(path);
        char line[256];
        for (const TimedPose & sample : poses) {
          const Eigen::Vector3d translation = sample.pose.translation();
          const Eigen::Quaterniond quaternion(sample.pose.rotation());
          std::snprintf(
            line, sizeof(line), "%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f",
            sample.stamp_sec, translation.x(), translation.y(), translation.z(),
            quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());
          output << line << "\n";
        }
      };
    if (save_dense_pose_refined_map) {
      write_dense_tum(output_dir + "/trajectory_raw_dense.tum", dense_raw_poses);
      write_dense_tum(
        output_dir + "/trajectory_optimized_dense.tum", dense_corrected_poses);
    }

    const auto save_dense_map =
      [&](const std::vector<TimedPose> & poses, const std::string & path) {
        std::vector<Eigen::Vector3d> world_points;
        world_points.reserve(attribution_retained_points);
        for (std::size_t i = 0; i < attribution_scans.size(); ++i) {
          for (const Eigen::Vector3d & local_point : attribution_scans[i].local_points) {
            world_points.push_back(poses[i].pose * local_point);
          }
        }
        const std::vector<Eigen::Vector3d> output_points =
          graphslam::map_quality::downsampleByVoxelCentroid(
          world_points, dense_pose_refined_output_voxel_size);
        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.reserve(output_points.size());
        for (const Eigen::Vector3d & point : output_points) {
          cloud.push_back(pcl::PointXYZ(
              static_cast<float>(point.x()), static_cast<float>(point.y()),
              static_cast<float>(point.z())));
        }
        if (pcl::io::savePCDFileBinary(path, cloud) != 0) {
          throw std::runtime_error("failed to save dense rebuilt map: " + path);
        }
        return output_points.size();
      };
    if (save_dense_pose_refined_map) {
      const std::size_t raw_map_points = save_dense_map(
        dense_raw_poses, output_dir + "/map_dense_raw.pcd");
      const std::size_t optimized_map_points = save_dense_map(
        dense_corrected_poses, output_dir + "/map_dense_optimized.pcd");

      std::ofstream report(output_dir + "/dense_pose_refined_map_report.yaml");
      report << std::setprecision(17);
      report << "dense_pose_refined_map:\n";
      report << "  schema_version: 1\n";
      report << "  correction_interpolation: world_translation_linear_quaternion_slerp\n";
      report << "  dense_scans: " << attribution_scans.size() << "\n";
      report << "  correction_anchors: " << correction_anchors.size() << "\n";
      report << "  raw_finite_points: " << attribution_raw_finite_points << "\n";
      report << "  retained_local_points: " << attribution_retained_points << "\n";
      report << "  scan_voxel_size_m: " << retained_scan_voxel_size << "\n";
      report << "  output_voxel_size_m: " << dense_pose_refined_output_voxel_size << "\n";
      report << "  raw_map_points: " << raw_map_points << "\n";
      report << "  optimized_map_points: " << optimized_map_points << "\n";
      RCLCPP_INFO(
        logger, "Wrote dense raw/optimized maps from %zu scans (%zu / %zu points)",
        attribution_scans.size(), raw_map_points, optimized_map_points);
    }

    if (save_probabilistic_surfel_map || save_connected_surface_map ||
      save_surface_consolidated_map ||
      save_surfel_support_partition_maps ||
      save_scan_persistence_filtered_map ||
      save_visibility_aware_dynamic_map || save_scan_surface_refined_surfel_map ||
      save_global_surface_ba_surfel_map)
    {
      std::vector<graphslam::ProbabilisticSurfelMapScan> surfel_scans;
      surfel_scans.reserve(attribution_scans.size());
      std::vector<graphslam::scan_surface_refinement::ScanSurfaceRefinerScan>
      scan_surface_scans;
      if (save_scan_surface_refined_surfel_map) {
        scan_surface_scans.reserve(attribution_scans.size());
      }
      for (std::size_t i = 0; i < attribution_scans.size(); ++i) {
        if (static_cast<int>(attribution_scans[i].scan_id %
          probabilistic_surfel_input_scan_stride) != probabilistic_surfel_input_scan_offset)
        {
          continue;
        }
        graphslam::ProbabilisticSurfelMapScan surfel_scan;
        surfel_scan.scan_id = static_cast<std::uint64_t>(attribution_scans[i].scan_id);
        surfel_scan.sensor_origin = dense_corrected_poses[i].pose.translation();
        const double translation_variance = meanClampedCovarianceDiagonal(
          attribution_scans[i].odometry_covariance, {0U, 7U, 14U},
          surfel_pose_translation_fallback_variance_m2,
          surfel_pose_translation_max_variance_m2);
        const double rotation_variance = meanClampedCovarianceDiagonal(
          attribution_scans[i].odometry_covariance, {21U, 28U, 35U},
          surfel_pose_rotation_fallback_variance_rad2,
          surfel_pose_rotation_max_variance_rad2);
        surfel_scan.pose_translation_variance_m2 = translation_variance;
        surfel_scan.pose_rotation_variance_rad2 = rotation_variance;
        surfel_scan.world_points.reserve(attribution_scans[i].local_points.size());
        for (const Eigen::Vector3d & local_point : attribution_scans[i].local_points) {
          surfel_scan.world_points.push_back(dense_corrected_poses[i].pose * local_point);
        }
        surfel_scans.push_back(std::move(surfel_scan));
        if (save_scan_surface_refined_surfel_map) {
          graphslam::scan_surface_refinement::ScanSurfaceRefinerScan surface_scan;
          surface_scan.scan_id = static_cast<std::uint64_t>(attribution_scans[i].scan_id);
          surface_scan.stamp_sec = attribution_scans[i].stamp_sec;
          surface_scan.world_pose = dense_corrected_poses[i].pose.matrix();
          surface_scan.pose_translation_variance_m2 = translation_variance;
          surface_scan.pose_rotation_variance_rad2 = rotation_variance;
          surface_scan.local_points_view = &attribution_scans[i].local_points;
          scan_surface_scans.push_back(surface_scan);
        }
      }
      graphslam::ProbabilisticSurfelMapResult surfel_result =
        graphslam::buildProbabilisticSurfelMap(surfel_scans, surfel_map_config);
      const auto save_points = [](const std::string & path,
        const std::vector<Eigen::Vector3d> & points) {
          pcl::PointCloud<pcl::PointXYZ> cloud;
          cloud.reserve(points.size());
          for (const Eigen::Vector3d & point : points) {
            cloud.push_back(pcl::PointXYZ(
                static_cast<float>(point.x()), static_cast<float>(point.y()),
                static_cast<float>(point.z())));
          }
          if (pcl::io::savePCDFileBinary(path, cloud) != 0) {
            throw std::runtime_error("failed to save probabilistic surfel map: " + path);
          }
        };
      save_points(
        output_dir + "/map_surfel_baseline_centroid.pcd",
        surfel_result.baseline_centroids);
      save_points(output_dir + "/map_surfel_fused.pcd", surfel_result.fused_points);
      if (save_connected_surface_map) {
        save_points(
          output_dir + "/map_surfel_connected_surface.pcd",
          surfel_result.connected_surface_points);
      }
      if (save_surface_consolidated_map) {
        save_points(
          output_dir + "/map_surfel_surface_consolidated.pcd",
          surfel_result.surface_consolidated_points);
      }
      if (save_surfel_support_partition_maps) {
        save_points(
          output_dir + "/map_surfel_supported_partition.pcd",
          surfel_result.supported_partition_points);
        save_points(
          output_dir + "/map_surfel_fallback_partition.pcd",
          surfel_result.fallback_partition_points);
      }
      if (save_scan_persistence_filtered_map) {
        save_points(
          output_dir + "/map_surfel_persistence_filtered.pcd",
          surfel_result.persistence_filtered_points);
      }
      if (save_visibility_aware_dynamic_map) {
        save_points(
          output_dir + "/map_surfel_visibility_filtered.pcd",
          surfel_result.visibility_filtered_points);
      }

      const auto & stats = surfel_result.stats;
      std::ofstream report(output_dir + "/probabilistic_surfel_map_report.yaml");
      report << std::setprecision(17);
      report << "probabilistic_surfel_map:\n";
      report << "  schema_version: 1\n";
      report << "  enabled_by_default: false\n";
      report << "  pose_source: dense_pose_graph_correction\n";
      report << "  covariance_source: odometry_pose_covariance_diagonal_mean\n";
      report << "  scan_voxel_size_m: " << retained_scan_voxel_size << "\n";
      report << "  output_voxel_size_m: " << surfel_map_config.voxel_size_m << "\n";
      report << "  support_voxel_size_m: " <<
        surfel_map_config.surfel_support_voxel_size_m << "\n";
      report << "  secondary_support_voxel_size_m: " <<
        surfel_map_config.secondary_support_voxel_size_m << "\n";
      report << "  tertiary_support_voxel_size_m: " <<
        surfel_map_config.tertiary_support_voxel_size_m << "\n";
      report << "  support_grid_phases: " << surfel_map_config.support_grid_phases << "\n";
      report << "  blend_support_phases: " << std::boolalpha <<
        surfel_map_config.blend_support_phases << "\n";
      report << "  support_phases_fallback_only: " << std::boolalpha <<
        surfel_map_config.support_phases_fallback_only << "\n";
      report << "  surface_consolidation_enabled: " << std::boolalpha <<
        save_surface_consolidated_map << "\n";
      report << "  surface_consolidation_min_projection_distance_m: " <<
        surfel_map_config.surface_consolidation_min_projection_distance_m << "\n";
      report << "  connected_surface_enabled: " << std::boolalpha <<
        save_connected_surface_map << "\n";
      report << "  connected_surface_max_normal_angle_deg: " <<
        surfel_map_config.connected_surface_max_normal_angle_deg << "\n";
      report << "  connected_surface_max_plane_distance_m: " <<
        surfel_map_config.connected_surface_max_plane_distance_m << "\n";
      report << "  connected_surface_min_support_cells: " <<
        surfel_map_config.connected_surface_min_support_cells << "\n";
      report << "  connected_surface_extend_fallback: " << std::boolalpha <<
        surfel_map_config.connected_surface_extend_fallback << "\n";
      report << "  connected_surface_max_extension_distance_m: " <<
        surfel_map_config.connected_surface_max_extension_distance_m << "\n";
      report << "  connected_surface_min_extension_support_cells: " <<
        surfel_map_config.connected_surface_min_extension_support_cells << "\n";
      report << "  min_distinct_scans: " << surfel_map_config.fusion.min_distinct_scans << "\n";
      report << "  input_scan_stride: " << probabilistic_surfel_input_scan_stride << "\n";
      report << "  input_scan_offset: " << probabilistic_surfel_input_scan_offset << "\n";
      report << "  input_scans: " << stats.input_scans << "\n";
      report << "  input_points: " << stats.input_points << "\n";
      report << "  finite_points: " << stats.finite_points << "\n";
      report << "  occupied_voxels: " << stats.occupied_voxels << "\n";
      report << "  occupied_support_voxels: " << stats.occupied_support_voxels << "\n";
      report << "  valid_support_surfels: " << stats.valid_support_surfels << "\n";
      report << "  baseline_points: " << surfel_result.baseline_centroids.size() << "\n";
      report << "  fused_points: " << surfel_result.fused_points.size() << "\n";
      report << "  fused_surfel_voxels: " << stats.fused_surfel_voxels << "\n";
      report << "  fallback_centroid_voxels: " << stats.fallback_centroid_voxels << "\n";
      report << "  shifted_phase_fused_voxels: " <<
        stats.shifted_phase_fused_voxels << "\n";
      report << "  surface_consolidation_input_points: " <<
        stats.surface_consolidation_input_points << "\n";
      report << "  surface_consolidation_selected_points: " <<
        stats.surface_consolidation_selected_points << "\n";
      report << "  surface_consolidation_output_points: " <<
        stats.surface_consolidation_output_points << "\n";
      report << "  surface_consolidation_merged_points: " <<
        stats.surface_consolidation_merged_points << "\n";
      report << "  support_partition_enabled: " << std::boolalpha <<
        save_surfel_support_partition_maps << "\n";
      report << "  supported_partition_points: " <<
        surfel_result.supported_partition_points.size() << "\n";
      report << "  fallback_partition_points: " <<
        surfel_result.fallback_partition_points.size() << "\n";
      report << "  connected_surface_support_cells: " <<
        stats.connected_surface_support_cells << "\n";
      report << "  connected_surface_merged_cells: " <<
        stats.connected_surface_merged_cells << "\n";
      report << "  connected_surface_projected_voxels: " <<
        stats.connected_surface_projected_voxels << "\n";
      report << "  connected_surface_extended_fallback_voxels: " <<
        stats.connected_surface_extended_fallback_voxels << "\n";
      report << "  connected_surface_output_points: " <<
        surfel_result.connected_surface_points.size() << "\n";
      report << "  mean_raw_normal_rms_m: " << stats.mean_raw_normal_rms_m << "\n";
      report << "  mean_fused_normal_sigma_m: " << stats.mean_fused_normal_sigma_m << "\n";
      report << "  persistence_filter_enabled: " << std::boolalpha <<
        save_scan_persistence_filtered_map << "\n";
      report << "  persistence_min_distinct_scans: " <<
        surfel_map_config.persistence_min_distinct_scans << "\n";
      report << "  persistence_min_scan_span: " <<
        surfel_map_config.persistence_min_scan_span << "\n";
      report << "  persistence_max_filter_range_m: " <<
        surfel_map_config.persistence_max_filter_range_m << "\n";
      report << "  persistence_candidate_voxels: " <<
        stats.persistence_candidate_voxels << "\n";
      report << "  persistence_kept_voxels: " << stats.persistence_kept_voxels << "\n";
      report << "  persistence_removed_voxels: " << stats.persistence_removed_voxels << "\n";
      report << "  persistence_far_range_keep_voxels: " <<
        stats.persistence_far_range_keep_voxels << "\n";
      report << "  persistence_output_points: " <<
        surfel_result.persistence_filtered_points.size() << "\n";
      report << "  visibility_filter_enabled: " << std::boolalpha <<
        save_visibility_aware_dynamic_map << "\n";
      report << "  visibility_max_distinct_scans: " <<
        surfel_map_config.visibility_max_distinct_scans << "\n";
      report << "  visibility_max_scan_span: " <<
        surfel_map_config.visibility_max_scan_span << "\n";
      report << "  visibility_near_scan_offset: " <<
        surfel_map_config.visibility_near_scan_offset << "\n";
      report << "  visibility_far_scan_offset: " <<
        surfel_map_config.visibility_far_scan_offset << "\n";
      report << "  visibility_min_free_space_votes: " <<
        surfel_map_config.visibility_min_free_space_votes << "\n";
      report << "  visibility_angular_resolution_rad: " <<
        surfel_map_config.visibility_angular_resolution_rad << "\n";
      report << "  visibility_free_space_margin_m: " <<
        surfel_map_config.visibility_free_space_margin_m << "\n";
      report << "  visibility_max_range_m: " <<
        surfel_map_config.visibility_max_range_m << "\n";
      report << "  visibility_max_origin_displacement_m: " <<
        surfel_map_config.visibility_max_origin_displacement_m << "\n";
      report << "  visibility_candidate_voxels: " <<
        stats.visibility_candidate_voxels << "\n";
      report << "  visibility_tested_voxels: " << stats.visibility_tested_voxels << "\n";
      report << "  visibility_contradicted_voxels: " <<
        stats.visibility_contradicted_voxels << "\n";
      report << "  visibility_removed_voxels: " << stats.visibility_removed_voxels << "\n";
      report << "  visibility_kept_voxels: " << stats.visibility_kept_voxels << "\n";
      report << "  visibility_output_points: " <<
        surfel_result.visibility_filtered_points.size() << "\n";
      RCLCPP_INFO(
        logger, "Wrote surfel baseline/fused maps: %zu occupied, %zu fused, %zu fallback",
        stats.occupied_voxels, stats.fused_surfel_voxels, stats.fallback_centroid_voxels);
      report.close();

      if (save_scan_surface_refined_surfel_map) {
        surfel_scans.clear();
        surfel_scans.shrink_to_fit();
        surfel_result.baseline_centroids.clear();
        surfel_result.baseline_centroids.shrink_to_fit();
        surfel_result.fused_points.clear();
        surfel_result.fused_points.shrink_to_fit();
        surfel_result.connected_surface_points.clear();
        surfel_result.connected_surface_points.shrink_to_fit();
        surfel_result.surface_consolidated_points.clear();
        surfel_result.surface_consolidated_points.shrink_to_fit();
        surfel_result.supported_partition_points.clear();
        surfel_result.supported_partition_points.shrink_to_fit();
        surfel_result.fallback_partition_points.clear();
        surfel_result.fallback_partition_points.shrink_to_fit();
        surfel_result.persistence_filtered_points.clear();
        surfel_result.persistence_filtered_points.shrink_to_fit();
        surfel_result.visibility_filtered_points.clear();
        surfel_result.visibility_filtered_points.shrink_to_fit();

        const graphslam::scan_surface_refinement::ScanSurfaceRefinerResult surface_result =
          graphslam::scan_surface_refinement::refineScanSurfaceTranslations(
          scan_surface_scans, scan_surface_config);
        {
          std::ofstream trajectory(output_dir + "/trajectory_scan_surface_refined_dense.tum");
          char line[256];
          for (std::size_t i = 0; i < surface_result.corrected_poses.size(); ++i) {
            const Eigen::Matrix4d & pose = surface_result.corrected_poses[i];
            const Eigen::Vector3d translation = pose.block<3, 1>(0, 3);
            const Eigen::Quaterniond quaternion(pose.block<3, 3>(0, 0));
            std::snprintf(
              line, sizeof(line), "%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f",
              scan_surface_scans[i].stamp_sec,
              translation.x(), translation.y(), translation.z(),
              quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());
            trajectory << line << "\n";
          }
        }

        graphslam::ProbabilisticSurfelMapResult corrected_surfel_result;
        if (surface_result.accepted) {
          std::vector<graphslam::ProbabilisticSurfelMapScan> corrected_scans;
          corrected_scans.reserve(scan_surface_scans.size());
          for (std::size_t i = 0; i < scan_surface_scans.size(); ++i) {
            graphslam::ProbabilisticSurfelMapScan corrected_scan;
            corrected_scan.scan_id = scan_surface_scans[i].scan_id;
            corrected_scan.sensor_origin =
              surface_result.corrected_poses[i].block<3, 1>(0, 3);
            corrected_scan.pose_translation_variance_m2 =
              scan_surface_scans[i].pose_translation_variance_m2;
            corrected_scan.pose_rotation_variance_rad2 =
              scan_surface_scans[i].pose_rotation_variance_rad2;
            const Eigen::Matrix3d rotation =
              surface_result.corrected_poses[i].block<3, 3>(0, 0);
            const Eigen::Vector3d translation = corrected_scan.sensor_origin;
            const std::vector<Eigen::Vector3d> & local_points =
              scan_surface_scans[i].points();
            corrected_scan.world_points.reserve(local_points.size());
            for (const Eigen::Vector3d & local_point : local_points) {
              corrected_scan.world_points.push_back(rotation * local_point + translation);
            }
            corrected_scans.push_back(std::move(corrected_scan));
          }
          graphslam::ProbabilisticSurfelMapConfig corrected_config = surfel_map_config;
          corrected_config.build_persistence_filtered_map = false;
          corrected_config.build_visibility_filtered_map = false;
          corrected_surfel_result = graphslam::buildProbabilisticSurfelMap(
            corrected_scans, corrected_config);
          save_points(
            output_dir + "/map_scan_surface_refined_centroid.pcd",
            corrected_surfel_result.baseline_centroids);
          save_points(
            output_dir + "/map_scan_surface_refined_surfel.pcd",
            corrected_surfel_result.fused_points);
        }
        {
          std::ofstream surface_report(output_dir + "/scan_surface_refiner_report.yaml");
          surface_report << std::setprecision(17);
          surface_report << "scan_surface_refiner:\n";
          surface_report << "  schema_version: 1\n";
          surface_report << "  enabled_by_default: false\n";
          surface_report << "  accepted: " << std::boolalpha << surface_result.accepted << "\n";
          surface_report << "  scan_downsample_voxel_size_m: " <<
            scan_surface_config.scan_downsample_voxel_size_m << "\n";
          surface_report << "  support_voxel_size_m: " <<
            scan_surface_config.support_voxel_size_m << "\n";
          surface_report << "  cross_fit_scan_parity: " <<
            scan_surface_config.cross_fit_scan_parity << "\n";
          surface_report << "  absolute_translation_prior_sigma_m: " <<
            scan_surface_config.absolute_translation_prior_sigma_m << "\n";
          surface_report << "  temporal_smoothness_sigma_m: " <<
            scan_surface_config.temporal_smoothness_sigma_m << "\n";
          surface_report << "  max_total_translation_correction_m: " <<
            scan_surface_config.max_total_translation_correction_m << "\n";
          surface_report << "  input_scans: " << scan_surface_scans.size() << "\n";
          surface_report << "  input_points: " << surface_result.input_points << "\n";
          surface_report << "  downsampled_points: " << surface_result.downsampled_points << "\n";
          surface_report << "  occupied_support_voxels: " <<
            surface_result.occupied_support_voxels << "\n";
          surface_report << "  valid_support_surfels: " <<
            surface_result.valid_support_surfels << "\n";
          surface_report << "  surface_observations: " <<
            surface_result.surface_observations << "\n";
          surface_report << "  constrained_scans: " <<
            surface_result.constrained_scans << "\n";
          surface_report << "  initial_objective: " << surface_result.initial_objective << "\n";
          surface_report << "  final_objective: " << surface_result.final_objective << "\n";
          surface_report << "  initial_surface_rms_m: " <<
            surface_result.initial_surface_rms_m << "\n";
          surface_report << "  final_surface_rms_m: " <<
            surface_result.final_surface_rms_m << "\n";
          surface_report << "  correction_rms_m: " << surface_result.correction_rms_m << "\n";
          surface_report << "  correction_max_m: " << surface_result.correction_max_m << "\n";
          surface_report << "  output_centroid_points: " <<
            corrected_surfel_result.baseline_centroids.size() << "\n";
          surface_report << "  output_surfel_points: " <<
            corrected_surfel_result.fused_points.size() << "\n";
        }
        RCLCPP_INFO(
          logger,
          "Scan-surface refinement %s: %.6f -> %.6f m surface RMS, %.6f m correction RMS",
          surface_result.accepted ? "accepted" : "rejected",
          surface_result.initial_surface_rms_m, surface_result.final_surface_rms_m,
          surface_result.correction_rms_m);
      }
    }
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
    refiner_config.pyramid.window_ba.prior_translation_sigma =
      refine_prior_translation_sigma;
    refiner_config.pyramid.window_ba.prior_rotation_sigma_rad =
      refine_prior_rotation_sigma_rad;
    refiner_config.pyramid.window_ba.max_step_translation =
      refine_max_step_translation;
    refiner_config.pyramid.window_ba.max_step_rotation_rad =
      refine_max_step_rotation_rad;
    refiner_config.pyramid.global_ba.prior_translation_sigma =
      refine_prior_translation_sigma;
    refiner_config.pyramid.global_ba.prior_rotation_sigma_rad =
      refine_prior_rotation_sigma_rad;
    refiner_config.pyramid.global_ba.max_step_translation =
      refine_max_step_translation;
    refiner_config.pyramid.global_ba.max_step_rotation_rad =
      refine_max_step_rotation_rad;

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

    if (save_global_surface_ba_surfel_map && refined.accepted &&
      !attribution_scans.empty())
    {
      using graphslam::dense_pose_correction::TimedPose;
      std::vector<TimedPose> raw_anchors;
      std::vector<TimedPose> refined_anchors;
      raw_anchors.reserve(records.size());
      refined_anchors.reserve(records.size());
      for (std::size_t i = 0; i < records.size(); ++i) {
        TimedPose raw_anchor;
        raw_anchor.stamp_sec = records[i].stamp_sec;
        raw_anchor.pose = Eigen::Isometry3d(records[i].meta.pose.matrix());
        raw_anchors.push_back(raw_anchor);
        TimedPose refined_anchor;
        refined_anchor.stamp_sec = records[i].stamp_sec;
        refined_anchor.pose = refined_poses[i];
        refined_anchors.push_back(refined_anchor);
      }
      const std::vector<graphslam::dense_pose_correction::CorrectionAnchor> correction_anchors =
        graphslam::dense_pose_correction::buildCorrectionAnchors(
        raw_anchors, refined_anchors);
      std::vector<TimedPose> dense_raw_poses;
      dense_raw_poses.reserve(attribution_scans.size());
      for (const ScanAttributionRecord & scan : attribution_scans) {
        TimedPose sample;
        sample.stamp_sec = scan.stamp_sec;
        sample.pose = scan.pose;
        dense_raw_poses.push_back(sample);
      }
      const std::vector<TimedPose> dense_refined_poses =
        graphslam::dense_pose_correction::applyDenseCorrections(
        dense_raw_poses, correction_anchors);

      {
        std::ofstream trajectory(output_dir + "/trajectory_global_surface_ba_dense.tum");
        char line[256];
        for (const TimedPose & sample : dense_refined_poses) {
          const Eigen::Vector3d translation = sample.pose.translation();
          const Eigen::Quaterniond quaternion(sample.pose.rotation());
          std::snprintf(
            line, sizeof(line), "%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f",
            sample.stamp_sec, translation.x(), translation.y(), translation.z(),
            quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());
          trajectory << line << "\n";
        }
      }

      std::vector<graphslam::ProbabilisticSurfelMapScan> global_surfel_scans;
      global_surfel_scans.reserve(attribution_scans.size());
      for (std::size_t i = 0; i < attribution_scans.size(); ++i) {
        if (static_cast<int>(attribution_scans[i].scan_id %
          probabilistic_surfel_input_scan_stride) != probabilistic_surfel_input_scan_offset)
        {
          continue;
        }
        graphslam::ProbabilisticSurfelMapScan surfel_scan;
        surfel_scan.scan_id = static_cast<std::uint64_t>(attribution_scans[i].scan_id);
        surfel_scan.sensor_origin = dense_refined_poses[i].pose.translation();
        surfel_scan.pose_translation_variance_m2 = meanClampedCovarianceDiagonal(
          attribution_scans[i].odometry_covariance, {0U, 7U, 14U},
          surfel_pose_translation_fallback_variance_m2,
          surfel_pose_translation_max_variance_m2);
        surfel_scan.pose_rotation_variance_rad2 = meanClampedCovarianceDiagonal(
          attribution_scans[i].odometry_covariance, {21U, 28U, 35U},
          surfel_pose_rotation_fallback_variance_rad2,
          surfel_pose_rotation_max_variance_rad2);
        surfel_scan.world_points.reserve(attribution_scans[i].local_points.size());
        for (const Eigen::Vector3d & local_point : attribution_scans[i].local_points) {
          surfel_scan.world_points.push_back(dense_refined_poses[i].pose * local_point);
        }
        global_surfel_scans.push_back(std::move(surfel_scan));
      }
      graphslam::ProbabilisticSurfelMapConfig global_surfel_config = surfel_map_config;
      global_surfel_config.build_persistence_filtered_map = false;
      global_surfel_config.build_visibility_filtered_map = false;
      const graphslam::ProbabilisticSurfelMapResult global_surfel_result =
        graphslam::buildProbabilisticSurfelMap(global_surfel_scans, global_surfel_config);
      const auto save_global_points = [](const std::string & path,
        const std::vector<Eigen::Vector3d> & points) {
          pcl::PointCloud<pcl::PointXYZ> cloud;
          cloud.reserve(points.size());
          for (const Eigen::Vector3d & point : points) {
            cloud.push_back(pcl::PointXYZ(
                static_cast<float>(point.x()), static_cast<float>(point.y()),
                static_cast<float>(point.z())));
          }
          if (pcl::io::savePCDFileBinary(path, cloud) != 0) {
            throw std::runtime_error("failed to save global surface BA surfel map: " + path);
          }
        };
      save_global_points(
        output_dir + "/map_global_surface_ba_centroid.pcd",
        global_surfel_result.baseline_centroids);
      save_global_points(
        output_dir + "/map_global_surface_ba_surfel.pcd",
        global_surfel_result.fused_points);
      {
        const auto & stats = global_surfel_result.stats;
        std::ofstream report(output_dir + "/global_surface_ba_surfel_report.yaml");
        report << std::setprecision(17);
        report << "global_surface_ba_surfel_map:\n";
        report << "  schema_version: 1\n";
        report << "  enabled_by_default: false\n";
        report << "  map_refinement_status: " << refined.status << "\n";
        report << "  correction_anchors: " << correction_anchors.size() << "\n";
        report << "  dense_scans: " << attribution_scans.size() << "\n";
        report << "  input_scan_stride: " << probabilistic_surfel_input_scan_stride << "\n";
        report << "  input_scan_offset: " << probabilistic_surfel_input_scan_offset << "\n";
        report << "  output_voxel_size_m: " << global_surfel_config.voxel_size_m << "\n";
        report << "  support_voxel_size_m: " <<
          global_surfel_config.surfel_support_voxel_size_m << "\n";
        report << "  secondary_support_voxel_size_m: " <<
          global_surfel_config.secondary_support_voxel_size_m << "\n";
        report << "  support_grid_phases: " << global_surfel_config.support_grid_phases << "\n";
        report << "  blend_support_phases: " << std::boolalpha <<
          global_surfel_config.blend_support_phases << "\n";
        report << "  occupied_voxels: " << stats.occupied_voxels << "\n";
        report << "  valid_support_surfels: " << stats.valid_support_surfels << "\n";
        report << "  fused_surfel_voxels: " << stats.fused_surfel_voxels << "\n";
        report << "  fallback_centroid_voxels: " << stats.fallback_centroid_voxels << "\n";
      }
      RCLCPP_INFO(
        logger, "Wrote global-surface-BA dense surfel map: %zu points, %zu fused",
        global_surfel_result.fused_points.size(),
        global_surfel_result.stats.fused_surfel_voxels);
    }

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
}  // NOLINT(readability/fn_size)
