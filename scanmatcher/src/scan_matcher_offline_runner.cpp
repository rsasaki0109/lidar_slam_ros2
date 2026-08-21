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

// The offline deterministic frontend runner (docs/roadmap/v0.6.md, Phase 4):
// read a raw sensor bag (LiDAR clouds + optional IMU) directly with
// rosbag2_cpp and drive the real ScanMatcherComponent in lockstep — publish
// one message at a time over intra-process pub/sub and drain a
// single-threaded executor until idle before the next message. Bag order is
// a total order, the async map-update worker is forbidden
// (async_map_update must be false), and a fixed registration thread count
// keeps ndt_omp bitwise reproducible, so the contract "same bag + same
// config => identical frontend trajectory and submap stream" becomes
// testable; scripts/run_frontend_determinism_check.sh runs this N times and
// diffs the outputs.
//
// The component keeps its production node name (scan_matcher) so existing
// presets (e.g. lidarslam/param/lidarslam.yaml) apply unchanged:
//
//   ros2 run scanmatcher scan_matcher_offline_runner --ros-args \
//     --params-file lidarslam/param/lidarslam.yaml \
//     -p async_map_update:=false \
//     -p bag_path:=demo_data/ntu_viral/tnp_01_points_restamped_vn100_rosbag2 \
//     -p cloud_topic:=/os1_cloud_node1/points \
//     -p output_dir:=/tmp/frontend_run1
//
// TF determinism note: the component's tf2 listener spins on its own
// thread, so feeding the sensor->base extrinsic over /tf_static would race
// the first cloud lookup. Instead the runner rewrites the cloud frame_id to
// the component's robot_frame_id (force_cloud_frame_to_robot_frame, default
// true): lookupTransform(robot, robot) is an identity that needs no TF
// data, and tf2::doTransform with the identity is a bitwise copy of the
// points. Mount extrinsics, when they matter, belong in a pre-transformed
// bag, exactly like the backend-input recording flow.

#include <pcl_conversions/pcl_conversions.h>  // NOLINT(build/include_order)

#include <chrono>
#include <climits>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lidarslam_msgs/msg/map_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "scanmatcher/offline_map_export.hpp"
#include "scanmatcher/registration_config.hpp"
#include "scanmatcher/scanmatcher_component.h"
#include "lidarslam_registration_loader/registration_plugin_loader.hpp"

namespace
{

void writeTum(const std::string & path, const std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
  std::ofstream out(path);
  char line[256];
  for (const auto & pose : poses) {
    const double stamp_sec = pose.header.stamp.sec + pose.header.stamp.nanosec * 1e-9;
    std::snprintf(
      line, sizeof(line), "%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f",
      stamp_sec,
      pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
      pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z,
      pose.pose.orientation.w);
    out << line << "\n";
  }
}

void writeSubmapsCsv(const std::string & path, const lidarslam_msgs::msg::MapArray & map_array)
{
  std::ofstream out(path);
  out << "index,stamp_sec,distance,x,y,z,qx,qy,qz,qw,cloud_points\n";
  char line[512];
  for (std::size_t i = 0; i < map_array.submaps.size(); ++i) {
    const auto & submap = map_array.submaps[i];
    const double stamp_sec = submap.header.stamp.sec + submap.header.stamp.nanosec * 1e-9;
    const std::size_t cloud_points =
      static_cast<std::size_t>(submap.cloud.width) * static_cast<std::size_t>(submap.cloud.height);
    std::snprintf(
      line, sizeof(line),
      "%zu,%.9f,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%zu",
      i, stamp_sec, submap.distance,
      submap.pose.position.x, submap.pose.position.y, submap.pose.position.z,
      submap.pose.orientation.x, submap.pose.orientation.y, submap.pose.orientation.z,
      submap.pose.orientation.w, cloud_points);
    out << line << "\n";
  }
}

std::string parameterValueToString(
  const lidarslam::plugins::registration::ParameterValue & value)
{
  using Type = lidarslam::plugins::registration::ParameterValue::Type;
  std::ostringstream stream;
  stream << std::setprecision(17);
  switch (value.type()) {
    case Type::kBool:
      return value.asBool() ? "true" : "false";
    case Type::kInteger:
      stream << value.asInteger();
      return stream.str();
    case Type::kDouble:
      stream << value.asDouble();
      return stream.str();
    case Type::kString:
      return value.asString();
  }
  return "<unknown>";
}

bool writeRegistrationPluginReceipt(
  const std::filesystem::path & path,
  const lidarslam::plugins::registration::shell::LoadRequest & request,
  const lidarslam::plugins::registration::shell::RegistrationPluginSession & session,
  std::string * error)
{
  std::ofstream out(path);
  if (!out) {
    if (error != nullptr) {
      *error = "cannot open registration plugin receipt: " + path.string();
    }
    return false;
  }
  const auto & metadata = session.metadata();
  const auto & capabilities = session.capabilities();
  out << "schema: 1\n";
  out << "backend_kind: " <<
    lidarslam::plugins::registration::shell::backendKindName(session.backendKind()) << "\n";
  out << "requested_class: " << request.class_id << "\n";
  out << "resolved_class: " << session.classId() << "\n";
  out << "metadata_class_id: " << metadata.class_id << "\n";
  out << "implementation_version: " << metadata.implementation_version << "\n";
  out << "license: " << metadata.license << "\n";
  out << "api_major: " << metadata.api_version.major << "\n";
  out << "api_minor: " << metadata.api_version.minor << "\n";
  out << "capabilities_bits: " << capabilities.bits() << "\n";
  out << "target_policy: " << static_cast<int>(capabilities.targetPolicy()) << "\n";
  out << "correspondence_metric: " << static_cast<int>(capabilities.correspondenceMetric()) << "\n";
  out << "library_path: " << session.libraryPath() << "\n";
  out << "plugin_manifest_path: " << session.pluginManifestPath() << "\n";
  out << "requirements:\n";
  out << "  initial_guess: " << (request.capabilities.require_initial_guess ? "true" : "false") << "\n";
  out << "  rotation_prior: " << (request.capabilities.require_rotation_prior ? "true" : "false") << "\n";
  out << "  translation_prior: " << (request.capabilities.require_translation_prior ? "true" : "false") << "\n";
  out << "  maximum_correspondence_distance: " <<
    (request.capabilities.require_maximum_correspondence_distance ? "true" : "false") << "\n";
  out << "  mean_correspondence_distance: " <<
    (request.capabilities.require_mean_correspondence_distance ? "true" : "false") << "\n";
  out << "  aligned_source: " << (request.capabilities.require_aligned_source ? "true" : "false") << "\n";
  out << "  deterministic: " << (request.capabilities.require_deterministic ? "true" : "false") << "\n";
  out << "parameters:\n";
  for (const auto & entry : session.parameters()) {
    out << "  " << entry.first << ": " << parameterValueToString(entry.second) << "\n";
  }
  if (!out) {
    if (error != nullptr) {
      *error = "failed while writing registration plugin receipt: " + path.string();
    }
    return false;
  }
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto runner = std::make_shared<rclcpp::Node>(
    "scan_matcher_offline_runner",
    rclcpp::NodeOptions().use_intra_process_comms(true));
  const std::string bag_path = runner->declare_parameter<std::string>("bag_path", "");
  const std::string output_dir = runner->declare_parameter<std::string>("output_dir", "");
  const std::string cloud_topic = runner->declare_parameter<std::string>("cloud_topic", "");
  const std::string imu_topic = runner->declare_parameter<std::string>("imu_topic", "");
  const bool force_cloud_frame = runner->declare_parameter<bool>(
    "force_cloud_frame_to_robot_frame", true);
  // Empty keeps the historical frontend-only output. When set, the runner
  // emits the same world-frame submap merge as publishMap() for map-quality
  // evidence without changing trajectory or submap serialization.
  const std::string map_output_path = runner->declare_parameter<std::string>(
    "map_output_path", "");
  // 0 = whole bag; a positive cap supports smoke tests and quick gates.
  const int max_clouds = runner->declare_parameter<int>("max_clouds", 0);
  // The plugin path is deliberately runner-only and opt-in.  Empty/false is
  // the compatibility default; live scanmatcher nodes never declare or read
  // these selectors.
  const bool registration_plugin_enable = runner->declare_parameter<bool>(
    "registration_plugin_enable", false);
  const std::string registration_plugin_class = runner->declare_parameter<std::string>(
    "registration_plugin_class", "");

  if (bag_path.empty() || output_dir.empty() || cloud_topic.empty()) {
    RCLCPP_ERROR(
      runner->get_logger(),
      "bag_path, output_dir and cloud_topic are required");
    rclcpp::shutdown();
    return 1;
  }
  if (!registration_plugin_enable && !registration_plugin_class.empty()) {
    RCLCPP_ERROR(
      runner->get_logger(),
      "registration_plugin_class is set while registration_plugin_enable is false; "
      "set both explicitly for offline characterization");
    rclcpp::shutdown();
    return 1;
  }
  if (registration_plugin_enable && registration_plugin_class.empty()) {
    RCLCPP_ERROR(
      runner->get_logger(),
      "registration_plugin_enable=true requires a non-empty registration_plugin_class; "
      "no fallback is applied");
    rclcpp::shutdown();
    return 1;
  }
  std::filesystem::create_directories(output_dir);

  rclcpp::NodeOptions component_options;
  component_options.use_intra_process_comms(true);
  auto component = registration_plugin_enable ?
    std::make_shared<graphslam::ScanMatcherComponent>(
      component_options,
      graphslam::RegistrationConstruction::kDeferredPluginInjection) :
    std::make_shared<graphslam::ScanMatcherComponent>(component_options);

  // The async map-update worker hands updateMap to a wall-clock-raced
  // thread; the determinism contract requires the synchronous path.
  if (component->get_parameter("async_map_update").as_bool()) {
    RCLCPP_ERROR(
      runner->get_logger(),
      "async_map_update must be false for the offline runner "
      "(add -p async_map_update:=false)");
    rclcpp::shutdown();
    return 1;
  }
  if (!component->get_parameter("set_initial_pose").as_bool()) {
    RCLCPP_ERROR(
      runner->get_logger(),
      "set_initial_pose must be true for the offline runner "
      "(there is no initial_pose publisher in the lockstep pipeline)");
    rclcpp::shutdown();
    return 1;
  }
  const std::string robot_frame_id =
    component->get_parameter("robot_frame_id").as_string();

  std::unique_ptr<lidarslam::plugins::registration::shell::RegistrationResolver>
    registration_resolver;
  std::shared_ptr<lidarslam::plugins::registration::shell::RegistrationPluginSession>
    registration_plugin_session;
  if (registration_plugin_enable) {
    const std::string registration_method =
      component->get_parameter("registration_method").as_string();
    if (
      registration_method != "NDT" && registration_method != "GICP" &&
      registration_method != "SMALL_GICP" && registration_method != "SMALL_VGICP")
    {
      RCLCPP_ERROR(
        runner->get_logger(),
        "registration plugin characterization supports NDT, GICP, SMALL_GICP, and SMALL_VGICP; "
        "registration_method=%s", registration_method.c_str());
      rclcpp::shutdown();
      return 1;
    }
    try {
      lidarslam::plugins::registration::shell::LoadRequest request;
      request.class_id = registration_plugin_class;
      request.capabilities.require_initial_guess = true;
      request.capabilities.require_aligned_source = true;
      request.capabilities.require_target_policy = true;
      const bool adaptive = component->get_parameter(
        "adaptive_correspondence_threshold").as_bool();
      if (registration_method == "NDT") {
        const double ndt_resolution = component->get_parameter("ndt_resolution").as_double();
        const double ndt_transformation_epsilon =
          component->get_parameter("ndt_transformation_epsilon").as_double();
        const std::int64_t ndt_max_iterations =
          component->get_parameter("ndt_max_iterations").as_int();
        const double ndt_step_size = component->get_parameter("ndt_step_size").as_double();
        const double ndt_outlier_ratio = component->get_parameter("ndt_outlier_ratio").as_double();
        const std::int64_t ndt_num_threads = component->get_parameter("ndt_num_threads").as_int();
        if (
          ndt_max_iterations < 1 || ndt_max_iterations > INT_MAX ||
          ndt_num_threads < 0 || ndt_num_threads > INT_MAX)
        {
          RCLCPP_ERROR(
            runner->get_logger(),
            "NDT plugin parameters maximum_iterations and num_threads must fit the adapter integer range");
          rclcpp::shutdown();
          return 1;
        }
        request.parameters = graphslam::registration_config::makeNdtParameterMap(
          ndt_resolution,
          ndt_transformation_epsilon,
          static_cast<int>(ndt_max_iterations),
          ndt_step_size,
          ndt_outlier_ratio,
          static_cast<int>(ndt_num_threads));
        request.capabilities.target_policy =
          lidarslam::plugins::registration::TargetPolicy::kRequiresRawTarget;
        request.capabilities.require_mean_correspondence_distance = true;
        request.capabilities.require_correspondence_metric = true;
        request.capabilities.correspondence_metric =
          lidarslam::plugins::registration::CorrespondenceMetric::kMeanDistance;
        request.capabilities.require_maximum_correspondence_distance = adaptive;
        request.capabilities.require_rotation_prior =
          component->get_parameter("imu_ndt_prior_enable").as_bool() &&
          component->get_parameter("imu_ndt_prior_weight").as_double() > 0.0;
        request.capabilities.require_translation_prior =
          component->get_parameter("imu_z_prior_enable").as_bool() &&
          component->get_parameter("imu_z_prior_weight").as_double() > 0.0;
      } else if (registration_method == "GICP") {
        const double gicp_corr_dist_threshold =
          component->get_parameter("gicp_corr_dist_threshold").as_double();
        request.parameters = graphslam::registration_config::makeGicpParameterMap(
          gicp_corr_dist_threshold, adaptive);
        request.capabilities.target_policy =
          lidarslam::plugins::registration::TargetPolicy::kAcceptHostPrepared;
        request.capabilities.require_correspondence_metric = true;
        request.capabilities.correspondence_metric =
          lidarslam::plugins::registration::CorrespondenceMetric::kSquareRootFitnessProxy;
        request.capabilities.require_maximum_correspondence_distance = adaptive;
      } else {
        const std::int64_t ndt_max_iterations =
          component->get_parameter("ndt_max_iterations").as_int();
        const std::int64_t ndt_num_threads =
          component->get_parameter("ndt_num_threads").as_int();
        if (
          ndt_max_iterations < 1 || ndt_max_iterations > INT_MAX ||
          ndt_num_threads < 0 || ndt_num_threads > INT_MAX)
        {
          RCLCPP_ERROR(
            runner->get_logger(),
            "SMALL_GICP parameters maximum_iterations and num_threads must fit the adapter integer range");
          rclcpp::shutdown();
          return 1;
        }
        const bool voxelized = registration_method == "SMALL_VGICP";
        request.parameters = graphslam::registration_config::makeSmallGicpParameterMap(
          component->get_parameter("gicp_corr_dist_threshold").as_double(),
          1e-6,
          static_cast<int>(ndt_max_iterations),
          static_cast<int>(ndt_num_threads),
          adaptive,
          voxelized,
          component->get_parameter("ndt_resolution").as_double());
        request.capabilities.target_policy =
          lidarslam::plugins::registration::TargetPolicy::kAcceptHostPrepared;
        request.capabilities.require_correspondence_metric = true;
        request.capabilities.correspondence_metric =
          lidarslam::plugins::registration::CorrespondenceMetric::kSquareRootFitnessProxy;
        request.capabilities.require_maximum_correspondence_distance = adaptive;
      }

      lidarslam::plugins::registration::shell::HostBuiltinRegistration host_ndt;
      host_ndt.class_id = "lidarslam_builtin/NdtOmp";
      host_ndt.factory = []() {
          return graphslam::makeHostBuiltinNdtRegistration();
      };
      host_ndt.metadata_class_id = "lidarslam_default_plugins/NdtOmp";
      lidarslam::plugins::registration::shell::HostBuiltinRegistration host_gicp;
      host_gicp.class_id = "lidarslam_builtin/GicpOmp";
      host_gicp.factory = []() {
          return graphslam::makeHostBuiltinGicpRegistration();
        };
      host_gicp.metadata_class_id = "lidarslam_default_plugins/GicpOmp";
#ifdef HAS_SMALL_GICP
      lidarslam::plugins::registration::shell::HostBuiltinRegistration host_small_gicp;
      host_small_gicp.class_id = "lidarslam_builtin/SmallGicpPcl";
      host_small_gicp.factory = []() {
          return graphslam::makeHostBuiltinSmallGicpRegistration();
        };
      host_small_gicp.metadata_class_id = "lidarslam_default_plugins/SmallGicpPcl";
      lidarslam::plugins::registration::shell::HostBuiltinRegistration host_small_vgicp;
      host_small_vgicp.class_id = "lidarslam_builtin/SmallVGicpPcl";
      host_small_vgicp.factory = []() {
          return graphslam::makeHostBuiltinSmallVgicpRegistration();
        };
      host_small_vgicp.metadata_class_id = "lidarslam_default_plugins/SmallVGicpPcl";
#endif
#ifdef HAS_SMALL_GICP
      registration_resolver.reset(
        new lidarslam::plugins::registration::shell::RegistrationResolver(
          {host_ndt, host_gicp, host_small_gicp, host_small_vgicp}));
#else
      registration_resolver.reset(
        new lidarslam::plugins::registration::shell::RegistrationResolver(
          {host_ndt, host_gicp}));
#endif
      const auto loaded = registration_resolver->resolve(request);
      if (!loaded.ok()) {
        RCLCPP_ERROR(
          runner->get_logger(),
          "offline registration plugin preflight failed [%d]: %s",
          static_cast<int>(loaded.failure.code), loaded.failure.message.c_str());
        rclcpp::shutdown();
        return 1;
      }
      registration_plugin_session = loaded.session;
      std::string injection_error;
      if (!component->setRegistrationPluginSession(
          registration_plugin_session, &injection_error))
      {
        RCLCPP_ERROR(
          runner->get_logger(),
          "offline registration plugin injection failed before sensor processing: %s",
          injection_error.c_str());
        rclcpp::shutdown();
        return 1;
      }
      const auto receipt_path =
        std::filesystem::path(output_dir) / "registration_plugin_receipt.yaml";
      std::string receipt_error;
      if (!writeRegistrationPluginReceipt(
          receipt_path, request, *registration_plugin_session, &receipt_error))
      {
        RCLCPP_ERROR(
          runner->get_logger(),
          "offline registration plugin receipt failed before sensor processing: %s",
          receipt_error.c_str());
        rclcpp::shutdown();
        return 1;
      }
      RCLCPP_INFO(
        runner->get_logger(),
        "offline registration resolved: backend=%s class=%s library=%s manifest=%s",
        lidarslam::plugins::registration::shell::backendKindName(
          registration_plugin_session->backendKind()),
        registration_plugin_session->classId().c_str(),
        registration_plugin_session->libraryPath().c_str(),
        registration_plugin_session->pluginManifestPath().c_str());
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        runner->get_logger(),
        "offline registration plugin preflight threw before sensor processing: %s",
        exception.what());
      rclcpp::shutdown();
      return 1;
    } catch (...) {
      RCLCPP_ERROR(
        runner->get_logger(),
        "offline registration plugin preflight threw an unknown exception before sensor processing");
      rclcpp::shutdown();
      return 1;
    }
  }

  std::vector<geometry_msgs::msg::PoseStamped> poses;
  auto pose_sub = runner->create_subscription<geometry_msgs::msg::PoseStamped>(
    "current_pose", rclcpp::QoS(100),
    [&poses](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      poses.push_back(*msg);
    });
  lidarslam_msgs::msg::MapArray latest_map_array;
  bool map_array_received = false;
  auto map_array_sub = runner->create_subscription<lidarslam_msgs::msg::MapArray>(
    "map_array", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
    [&latest_map_array, &map_array_received](lidarslam_msgs::msg::MapArray::SharedPtr msg) {
      latest_map_array = *msg;
      map_array_received = true;
    });

  auto cloud_qos = rclcpp::SensorDataQoS();
  cloud_qos.keep_last(10);
  auto cloud_pub = runner->create_publisher<sensor_msgs::msg::PointCloud2>(
    "input_cloud", cloud_qos);
  auto imu_pub = runner->create_publisher<sensor_msgs::msg::Imu>(
    "imu", rclcpp::SensorDataQoS());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(component);
  executor.add_node(runner);
  // Let discovery/intra-process wiring settle before the first message.
  executor.spin_all(std::chrono::seconds(1));

  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> cloud_serialization;
  rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serialization;

  rosbag2_cpp::Reader reader;
  reader.open(bag_path);

  std::size_t cloud_count = 0;
  std::size_t imu_count = 0;
  while (reader.has_next() && rclcpp::ok()) {
    if (max_clouds > 0 && cloud_count >= static_cast<std::size_t>(max_clouds)) {
      break;
    }
    auto bag_message = reader.read_next();
    const rclcpp::SerializedMessage serialized(*bag_message->serialized_data);
    if (bag_message->topic_name == cloud_topic) {
      auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
      cloud_serialization.deserialize_message(&serialized, cloud.get());
      if (force_cloud_frame) {
        cloud->header.frame_id = robot_frame_id;
      }
      cloud_pub->publish(std::move(cloud));
      executor.spin_all(std::chrono::seconds(60));
      ++cloud_count;
      if (cloud_count % 500 == 0) {
        RCLCPP_INFO(
          runner->get_logger(), "processed %zu clouds (%zu poses)",
          cloud_count, poses.size());
      }
    } else if (!imu_topic.empty() && bag_message->topic_name == imu_topic) {
      auto imu = std::make_unique<sensor_msgs::msg::Imu>();
      imu_serialization.deserialize_message(&serialized, imu.get());
      imu_pub->publish(std::move(imu));
      executor.spin_all(std::chrono::seconds(60));
      ++imu_count;
    }
  }
  // rclcpp installs a graceful SIGINT handler.  Treat an interrupted bag as
  // a failed run unless the requested cloud cap was already reached; this
  // keeps the determinism script from touching .complete for a partial
  // characterization artifact.
  if (!rclcpp::ok() && !(max_clouds > 0 && cloud_count >= static_cast<std::size_t>(max_clouds))) {
    RCLCPP_ERROR(
      runner->get_logger(),
      "offline runner interrupted before completion (clouds=%zu imu=%zu)",
      cloud_count, imu_count);
    rclcpp::shutdown();
    return 1;
  }
  executor.spin_all(std::chrono::seconds(60));

  if (poses.empty() || !map_array_received) {
    RCLCPP_ERROR(
      runner->get_logger(),
      "frontend produced no output (clouds=%zu poses=%zu map_array=%d) — "
      "check cloud_topic and the scan_matcher parameters",
      cloud_count, poses.size(), static_cast<int>(map_array_received));
    rclcpp::shutdown();
    return 1;
  }

  const auto trajectory_path =
    (std::filesystem::path(output_dir) / "trajectory_frontend.tum").string();
  const auto submaps_path =
    (std::filesystem::path(output_dir) / "submaps_frontend.csv").string();
  writeTum(trajectory_path, poses);
  writeSubmapsCsv(submaps_path, latest_map_array);

  if (!map_output_path.empty()) {
    const std::filesystem::path map_path(map_output_path);
    if (map_path.has_parent_path()) {
      std::filesystem::create_directories(map_path.parent_path());
    }
    const auto map_cloud = graphslam::offline_map_export::mergeSubmaps(latest_map_array);
    if (map_cloud.empty() ||
      !graphslam::offline_map_export::saveBinaryCompressed(map_output_path, map_cloud))
    {
      RCLCPP_ERROR(
        runner->get_logger(), "failed to write non-empty map PCD: %s", map_output_path.c_str());
      rclcpp::shutdown();
      return 1;
    }
    RCLCPP_INFO(
      runner->get_logger(), "map PCD: %s (%zu points)",
      map_output_path.c_str(), map_cloud.size());
  }

  RCLCPP_INFO(
    runner->get_logger(),
    "frontend offline run complete: clouds=%zu imu=%zu poses=%zu submaps=%zu",
    cloud_count, imu_count, poses.size(), latest_map_array.submaps.size());
  RCLCPP_INFO(runner->get_logger(), "trajectory: %s", trajectory_path.c_str());
  RCLCPP_INFO(runner->get_logger(), "submaps:    %s", submaps_path.c_str());

  rclcpp::shutdown();
  return 0;
}
