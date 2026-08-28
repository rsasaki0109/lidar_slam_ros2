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

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include <pcl/common/transforms.h>  // NOLINT(build/include_order)
#include <pcl/filters/voxel_grid.h>  // NOLINT(build/include_order)
#include <pcl/io/pcd_io.h>  // NOLINT(build/include_order)
#include <pcl/search/kdtree.h>  // NOLINT(build/include_order)
#include <pcl_conversions/pcl_conversions.h>  // NOLINT(build/include_order)
#include <pclomp/ndt_omp.h>  // NOLINT(build/include_order)
#include <pclomp/ndt_omp_impl.hpp>  // NOLINT(build/include_order)
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>  // NOLINT(build/include_order)

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "graph_based_slam/ndt_localization_target.hpp"
#include "graph_based_slam/ndt_trajectory_io.hpp"

namespace
{

struct Options
{
  std::string map_path;
  std::string secondary_map_path;
  std::string tertiary_map_path;
  std::string bag_path;
  std::string output_path;
  std::string csv_path;
  std::string trajectory_path;
  std::string odom_topic {"/rko_lio/odometry"};
  std::string cloud_topic {"/rko_lio/frame"};
  std::size_t stride {100U};
  std::size_t offset {0U};
  double resolution_m {1.0};
  double source_voxel_m {0.5};
  double max_correspondence_m {2.0};
  int max_iterations {10};
  std::size_t secondary_map_repeat {1U};
  std::size_t secondary_map_stride {1U};
  double perturb_translation_x_m {0.0};
  double perturb_yaw_deg {0.0};
  double pose_update_gain {1.0};
  double tangent_sampling_radius_m {0.0};
  double tangent_inner_radius_m {0.0};
  bool tangent_diagonal_samples {false};
  bool tangent_angular_midpoints {false};
  std::size_t tangent_angular_midpoint_pairs {0U};
};

Options parseOptions(int argc, char ** argv)
{
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (i + 1 >= argc) {throw std::runtime_error("missing value for " + arg);}
    const std::string value(argv[++i]);
    if (arg == "--map") {
      options.map_path = value;
    } else if (arg == "--map-secondary") {
      options.secondary_map_path = value;
    } else if (arg == "--map-tertiary") {
      options.tertiary_map_path = value;
    } else if (arg == "--secondary-repeat") {
      options.secondary_map_repeat = static_cast<std::size_t>(std::stoul(value));
    } else if (arg == "--secondary-stride") {
      options.secondary_map_stride = static_cast<std::size_t>(std::stoul(value));
    } else if (arg == "--bag") {
      options.bag_path = value;
    } else if (arg == "--output") {
      options.output_path = value;
    } else if (arg == "--csv") {
      options.csv_path = value;
    } else if (arg == "--trajectory") {
      options.trajectory_path = value;
    } else if (arg == "--odom-topic") {
      options.odom_topic = value;
    } else if (arg == "--cloud-topic") {
      options.cloud_topic = value;
    } else if (arg == "--stride") {
      options.stride = static_cast<std::size_t>(std::stoul(value));
    } else if (arg == "--offset") {
      options.offset = static_cast<std::size_t>(std::stoul(value));
    } else if (arg == "--resolution") {
      options.resolution_m = std::stod(value);
    } else if (arg == "--source-voxel") {
      options.source_voxel_m = std::stod(value);
    } else if (arg == "--max-correspondence") {
      options.max_correspondence_m = std::stod(value);
    } else if (arg == "--max-iterations") {
      options.max_iterations = std::stoi(value);
    } else if (arg == "--perturb-translation-x") {
      options.perturb_translation_x_m = std::stod(value);
    } else if (arg == "--perturb-yaw-deg") {
      options.perturb_yaw_deg = std::stod(value);
    } else if (arg == "--pose-update-gain") {
      options.pose_update_gain = std::stod(value);
    } else if (arg == "--tangent-sampling-radius") {
      options.tangent_sampling_radius_m = std::stod(value);
    } else if (arg == "--tangent-inner-radius") {
      options.tangent_inner_radius_m = std::stod(value);
    } else if (arg == "--tangent-angular-midpoints") {
      options.tangent_angular_midpoints = value == "true" || value == "1";
    } else if (arg == "--tangent-angular-midpoint-pairs") {
      options.tangent_angular_midpoint_pairs = static_cast<std::size_t>(std::stoul(value));
    } else if (arg == "--tangent-diagonal-samples") {
      options.tangent_diagonal_samples = value == "true" || value == "1";
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }
  if (options.map_path.empty() || options.bag_path.empty() || options.output_path.empty() ||
    options.stride == 0U || options.offset >= options.stride || !(options.resolution_m > 0.0) ||
    !(options.source_voxel_m > 0.0) || !(options.max_correspondence_m > 0.0) ||
    options.max_iterations < 1 || options.secondary_map_repeat == 0U ||
    !std::isfinite(options.pose_update_gain) || options.pose_update_gain < 0.0 ||
    options.pose_update_gain > 1.0 ||
    !(options.tangent_sampling_radius_m >= 0.0) ||
    !std::isfinite(options.tangent_sampling_radius_m) ||
    options.tangent_sampling_radius_m > 0.5 * options.resolution_m ||
    !(options.tangent_inner_radius_m >= 0.0) ||
    !std::isfinite(options.tangent_inner_radius_m) ||
    (options.tangent_inner_radius_m > 0.0 &&
    options.tangent_inner_radius_m >= options.tangent_sampling_radius_m) ||
    options.tangent_angular_midpoint_pairs > 4U ||
    options.secondary_map_stride == 0U)
  {
    throw std::runtime_error(
            "usage: map_ndt_residual_report --map MAP.pcd --bag BAG --output REPORT.yaml "
            "[--map-secondary MAP.pcd] [--map-tertiary MAP.pcd] "
            "[--secondary-repeat N] [--secondary-stride N] "
            "[--stride N] [--offset N] [--resolution M] [--source-voxel M] "
            "[--pose-update-gain FRACTION] "
            "[--tangent-sampling-radius M] [--tangent-inner-radius M] "
            "[--trajectory REGISTERED.tum]");
  }
  return options;
}

struct NdtSample
{
  std::size_t pair_index {0U};
  double stamp_sec {0.0};
  bool converged {false};
  double fitness_m2 {0.0};
  double translation_delta_m {0.0};
  double pose_translation_error_m {0.0};
  double pose_rotation_error_deg {0.0};
  double translation_variance_min_m2 {0.0};
  double translation_variance_max_m2 {0.0};
  double rotation_variance_max_rad2 {0.0};
};

std::size_t addPlanarTangentSamples(
  pcl::PointCloud<pcl::PointXYZ> & cloud, const double voxel_size, const double radius,
  const double inner_radius, const bool add_diagonals, const bool add_angular_midpoints,
  const std::size_t angular_midpoint_pairs)
{
  if (!(radius > 0.0)) {return 0U;}
  std::vector<Eigen::Vector3d> input;
  input.reserve(cloud.size());
  for (const pcl::PointXYZ & point : cloud) {
    input.emplace_back(point.x, point.y, point.z);
  }
  graphslam::ndt_localization::TangentSamplingConfig config;
  config.voxel_size_m = voxel_size;
  config.radius_m = radius;
  config.inner_radius_m = inner_radius;
  config.add_diagonals = add_diagonals;
  config.add_angular_midpoints = add_angular_midpoints;
  config.angular_midpoint_pairs = angular_midpoint_pairs;
  const auto result = graphslam::ndt_localization::buildTangentSampledTarget(input, config);
  if (result.points.empty()) {throw std::runtime_error("tangent target construction failed");}
  cloud.clear();
  cloud.reserve(result.points.size());
  for (const Eigen::Vector3d & point : result.points) {
    cloud.push_back(pcl::PointXYZ(
      static_cast<float>(point.x()), static_cast<float>(point.y()),
      static_cast<float>(point.z())));
  }
  return result.sampled_points;
}

std::uint64_t stampKey(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::uint64_t>(stamp.sec) * 1000000000ULL + stamp.nanosec;
}

double percentile(std::vector<double> values, const double fraction)
{
  if (values.empty()) {return 0.0;}
  std::sort(values.begin(), values.end());
  const std::size_t index = static_cast<std::size_t>(
    std::floor(fraction * static_cast<double>(values.size() - 1U)));
  return values[index];
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    const Options options = parseOptions(argc, argv);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile(options.map_path, *target) != 0 || target->empty()) {
      throw std::runtime_error("failed to load target map: " + options.map_path);
    }
    const std::size_t primary_map_points = target->size();
    const std::size_t tangent_sample_points = addPlanarTangentSamples(
      *target, options.resolution_m, options.tangent_sampling_radius_m,
      options.tangent_inner_radius_m, options.tangent_diagonal_samples,
      options.tangent_angular_midpoints, options.tangent_angular_midpoint_pairs);
    std::size_t secondary_map_points = 0U;
    std::size_t secondary_map_used_points = 0U;
    if (!options.secondary_map_path.empty()) {
      pcl::PointCloud<pcl::PointXYZ> secondary;
      if (pcl::io::loadPCDFile(options.secondary_map_path, secondary) != 0 ||
        secondary.empty())
      {
        throw std::runtime_error(
                "failed to load secondary target map: " + options.secondary_map_path);
      }
      secondary_map_points = secondary.size();
      pcl::PointCloud<pcl::PointXYZ> selected_secondary;
      selected_secondary.reserve(
        (secondary.size() + options.secondary_map_stride - 1U) /
        options.secondary_map_stride);
      for (std::size_t index = 0U; index < secondary.size();
        index += options.secondary_map_stride)
      {
        selected_secondary.push_back(secondary[index]);
      }
      secondary_map_used_points = selected_secondary.size();
      for (std::size_t repeat = 0U; repeat < options.secondary_map_repeat; ++repeat) {
        *target += selected_secondary;
      }
    }
    std::size_t tertiary_map_points = 0U;
    if (!options.tertiary_map_path.empty()) {
      pcl::PointCloud<pcl::PointXYZ> tertiary;
      if (pcl::io::loadPCDFile(options.tertiary_map_path, tertiary) != 0 || tertiary.empty()) {
        throw std::runtime_error(
                "failed to load tertiary target map: " + options.tertiary_map_path);
      }
      tertiary_map_points = tertiary.size();
      *target += tertiary;
    }

    pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setNumThreads(1);
    ndt.setNeighborhoodSearchMethod(pclomp::DIRECT7);
    ndt.setResolution(options.resolution_m);
    ndt.setTransformationEpsilon(1.0e-4);
    ndt.setMaximumIterations(options.max_iterations);
    ndt.setInputTarget(target);
    pcl::search::KdTree<pcl::PointXYZ> fitness_tree;
    fitness_tree.setInputCloud(target);

    rosbag2_cpp::Reader reader;
    reader.open(options.bag_path);
    rclcpp::Serialization<nav_msgs::msg::Odometry> odom_serialization;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> cloud_serialization;
    std::map<std::uint64_t, nav_msgs::msg::Odometry> pending_odoms;
    std::map<std::uint64_t, sensor_msgs::msg::PointCloud2> pending_clouds;
    std::size_t paired_count = 0U;
    std::size_t sampled_count = 0U;
    std::size_t converged_count = 0U;
    std::vector<double> fitness_scores;
    std::vector<double> translation_deltas;
    std::vector<double> pose_translation_errors;
    std::vector<double> pose_rotation_errors_deg;
    std::vector<NdtSample> samples;
    std::vector<graphslam::ndt_localization::RegisteredPose> registered_poses;
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setLeafSize(
      static_cast<float>(options.source_voxel_m),
      static_cast<float>(options.source_voxel_m),
      static_cast<float>(options.source_voxel_m));

    const auto process_pair = [&](const nav_msgs::msg::Odometry & odom,
      const sensor_msgs::msg::PointCloud2 & cloud_msg) {
        const std::size_t pair_index = paired_count++;
        if (pair_index % options.stride != options.offset) {return;}
        pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(cloud_msg, *source);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
        voxel_grid.setInputCloud(source);
        voxel_grid.filter(*filtered);
        if (filtered->empty()) {return;}
        Eigen::Affine3d reference = Eigen::Affine3d::Identity();
        tf2::fromMsg(odom.pose.pose, reference);
        Eigen::Affine3d perturbation = Eigen::Affine3d::Identity();
        perturbation.translation().x() = options.perturb_translation_x_m;
        perturbation.linear() = Eigen::AngleAxisd(
          options.perturb_yaw_deg * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        const Eigen::Affine3d initial = reference * perturbation;
        ndt.setInputSource(filtered);
        pcl::PointCloud<pcl::PointXYZ> aligned;
        ndt.align(aligned, initial.matrix().cast<float>());
        ++sampled_count;
        const bool converged = ndt.hasConverged();
        if (converged) {++converged_count;}
        const Eigen::Matrix4d optimized = ndt.getFinalTransformation().cast<double>();
        const Eigen::Affine3d guarded = graphslam::ndt_localization::regularizePoseUpdate(
          initial, optimized, options.pose_update_gain);
        double fitness = ndt.getFitnessScore(options.max_correspondence_m);
        if (options.pose_update_gain < 1.0) {
          double fitness_sum = 0.0;
          std::size_t fitness_count = 0U;
          pcl::PointCloud<pcl::PointXYZ> guarded_source;
          pcl::transformPointCloud(*filtered, guarded_source, guarded.matrix().cast<float>());
          std::vector<int> neighbor_index(1);
          std::vector<float> neighbor_distance_squared(1);
          for (const pcl::PointXYZ & point : guarded_source) {
            if (fitness_tree.nearestKSearch(
                point, 1, neighbor_index, neighbor_distance_squared) > 0 &&
              neighbor_distance_squared.front() <= options.max_correspondence_m)
            {
              fitness_sum += neighbor_distance_squared.front();
              ++fitness_count;
            }
          }
          fitness = fitness_count > 0U ? fitness_sum / static_cast<double>(fitness_count) :
            std::numeric_limits<double>::infinity();
        }
        if (std::isfinite(fitness)) {fitness_scores.push_back(fitness);}
        const double translation_delta =
          (guarded.translation() - initial.translation()).norm();
        const double pose_translation_error =
          (guarded.translation() - reference.translation()).norm();
        translation_deltas.push_back(translation_delta);
        pose_translation_errors.push_back(pose_translation_error);
        const Eigen::Matrix3d rotation_error =
          reference.rotation().transpose() * guarded.linear();
        const Eigen::AngleAxisd angle_axis(rotation_error);
        const double pose_rotation_error_deg =
          std::abs(angle_axis.angle()) * 180.0 / M_PI;
        pose_rotation_errors_deg.push_back(pose_rotation_error_deg);
        if (guarded.matrix().allFinite()) {
          registered_poses.push_back(graphslam::ndt_localization::RegisteredPose{
          pair_index, rclcpp::Time(odom.header.stamp).seconds(), converged,
          guarded.translation(), Eigen::Quaterniond(guarded.linear())});
        }
        const std::array<double, 3> translation_variances{
          odom.pose.covariance[0], odom.pose.covariance[7], odom.pose.covariance[14]};
        const std::array<double, 3> rotation_variances{
          odom.pose.covariance[21], odom.pose.covariance[28], odom.pose.covariance[35]};
        samples.push_back(NdtSample{
        pair_index, rclcpp::Time(odom.header.stamp).seconds(), converged, fitness,
        translation_delta, pose_translation_error, pose_rotation_error_deg,
        *std::min_element(translation_variances.begin(), translation_variances.end()),
        *std::max_element(translation_variances.begin(), translation_variances.end()),
        *std::max_element(rotation_variances.begin(), rotation_variances.end())});
      };

    while (reader.has_next()) {
      const auto message = reader.read_next();
      rclcpp::SerializedMessage serialized(*message->serialized_data);
      if (message->topic_name == options.odom_topic) {
        nav_msgs::msg::Odometry odom;
        odom_serialization.deserialize_message(&serialized, &odom);
        const std::uint64_t key = stampKey(odom.header.stamp);
        const auto found = pending_clouds.find(key);
        if (found != pending_clouds.end()) {
          process_pair(odom, found->second);
          pending_clouds.erase(found);
        } else {
          pending_odoms[key] = odom;
        }
      } else if (message->topic_name == options.cloud_topic) {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud_serialization.deserialize_message(&serialized, &cloud);
        const std::uint64_t key = stampKey(cloud.header.stamp);
        const auto found = pending_odoms.find(key);
        if (found != pending_odoms.end()) {
          process_pair(found->second, cloud);
          pending_odoms.erase(found);
        } else {
          pending_clouds[key] = cloud;
        }
      }
    }
    if (!options.trajectory_path.empty()) {
      std::ofstream trajectory(options.trajectory_path);
      if (!trajectory.is_open()) {
        throw std::runtime_error("failed to open trajectory: " + options.trajectory_path);
      }
      graphslam::ndt_localization::writeRegisteredPoseTum(trajectory, registered_poses);
    }
    if (fitness_scores.empty()) {throw std::runtime_error("no finite NDT fitness scores");}
    if (!options.csv_path.empty()) {
      std::ofstream csv(options.csv_path);
      if (!csv.is_open()) {throw std::runtime_error("failed to open CSV: " + options.csv_path);}
      csv << std::setprecision(17);
      csv << "pair_index,stamp_sec,converged,fitness_m2,translation_delta_m,"
        "pose_translation_error_m,pose_rotation_error_deg,"
        "translation_variance_min_m2,translation_variance_max_m2,"
        "rotation_variance_max_rad2\n";
      for (const NdtSample & sample : samples) {
        csv << sample.pair_index << ',' << sample.stamp_sec << ',' << std::boolalpha <<
          sample.converged << ',' << sample.fitness_m2 << ',' << sample.translation_delta_m <<
          ',' << sample.pose_translation_error_m << ',' << sample.pose_rotation_error_deg << ',' <<
          sample.translation_variance_min_m2 << ',' << sample.translation_variance_max_m2 << ',' <<
          sample.rotation_variance_max_rad2 << '\n';
      }
    }
    const double fitness_mean = std::accumulate(
      fitness_scores.begin(), fitness_scores.end(), 0.0) /
      static_cast<double>(fitness_scores.size());
    const double delta_mean = std::accumulate(
      translation_deltas.begin(), translation_deltas.end(), 0.0) /
      static_cast<double>(translation_deltas.size());
    const double pose_translation_error_mean = std::accumulate(
      pose_translation_errors.begin(), pose_translation_errors.end(), 0.0) /
      static_cast<double>(pose_translation_errors.size());
    const double pose_rotation_error_mean = std::accumulate(
      pose_rotation_errors_deg.begin(), pose_rotation_errors_deg.end(), 0.0) /
      static_cast<double>(pose_rotation_errors_deg.size());
    std::ofstream output(options.output_path);
    output << std::setprecision(17);
    output << "map_ndt_residual_report:\n";
    output << "  schema_version: 1\n";
    output << "  csv_path: " << options.csv_path << "\n";
    output << "  trajectory_path: " << options.trajectory_path << "\n";
    output << "  trajectory_poses: " << registered_poses.size() << "\n";
    output << "  map_path: " << options.map_path << "\n";
    output << "  bag_path: " << options.bag_path << "\n";
    output << "  odom_topic: " << options.odom_topic << "\n";
    output << "  cloud_topic: " << options.cloud_topic << "\n";
    output << "  secondary_map_path: " << options.secondary_map_path << "\n";
    output << "  secondary_map_repeat: " << options.secondary_map_repeat << "\n";
    output << "  secondary_map_stride: " << options.secondary_map_stride << "\n";
    output << "  primary_map_points: " << primary_map_points << "\n";
    output << "  tangent_sampling_radius_m: " << options.tangent_sampling_radius_m << "\n";
    output << "  tangent_inner_radius_m: " << options.tangent_inner_radius_m << "\n";
    output << "  tangent_diagonal_samples: " << std::boolalpha <<
      options.tangent_diagonal_samples << "\n";
    output << "  tangent_angular_midpoints: " << options.tangent_angular_midpoints << "\n";
    output << "  tangent_angular_midpoint_pairs: " <<
      options.tangent_angular_midpoint_pairs << "\n";
    output << "  tangent_sample_points: " << tangent_sample_points << "\n";
    output << "  secondary_map_points: " << secondary_map_points << "\n";
    output << "  secondary_map_used_points: " << secondary_map_used_points << "\n";
    output << "  tertiary_map_path: " << options.tertiary_map_path << "\n";
    output << "  tertiary_map_points: " << tertiary_map_points << "\n";
    output << "  target_map_points: " << target->size() << "\n";
    output << "  paired_scans: " << paired_count << "\n";
    output << "  stride: " << options.stride << "\n";
    output << "  offset: " << options.offset << "\n";
    output << "  sampled_scans: " << sampled_count << "\n";
    output << "  converged_scans: " << converged_count << "\n";
    output << "  finite_fitness_scans: " << fitness_scores.size() << "\n";
    output << "  resolution_m: " << options.resolution_m << "\n";
    output << "  source_voxel_m: " << options.source_voxel_m << "\n";
    output << "  max_correspondence_m: " << options.max_correspondence_m << "\n";
    output << "  max_iterations: " << options.max_iterations << "\n";
    output << "  perturb_translation_x_m: " << options.perturb_translation_x_m << "\n";
    output << "  perturb_yaw_deg: " << options.perturb_yaw_deg << "\n";
    output << "  pose_update_gain: " << options.pose_update_gain << "\n";
    output << "  fitness_mean_m2: " << fitness_mean << "\n";
    output << "  fitness_median_m2: " << percentile(fitness_scores, 0.5) << "\n";
    output << "  fitness_p95_m2: " << percentile(fitness_scores, 0.95) << "\n";
    output << "  translation_delta_mean_m: " << delta_mean << "\n";
    output << "  translation_delta_p95_m: " << percentile(translation_deltas, 0.95) << "\n";
    output << "  pose_translation_error_mean_m: " << pose_translation_error_mean << "\n";
    output << "  pose_translation_error_p95_m: " <<
      percentile(pose_translation_errors, 0.95) << "\n";
    output << "  pose_rotation_error_mean_deg: " << pose_rotation_error_mean << "\n";
    output << "  pose_rotation_error_p95_deg: " <<
      percentile(pose_rotation_errors_deg, 0.95) << "\n";
  } catch (const std::exception & error) {
    std::cerr << error.what() << "\n";
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
