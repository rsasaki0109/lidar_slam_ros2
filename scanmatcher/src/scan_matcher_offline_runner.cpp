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
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lidarslam_msgs/msg/map_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "scanmatcher/scanmatcher_component.h"

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
  // 0 = whole bag; a positive cap supports smoke tests and quick gates.
  const int max_clouds = runner->declare_parameter<int>("max_clouds", 0);

  if (bag_path.empty() || output_dir.empty() || cloud_topic.empty()) {
    RCLCPP_ERROR(
      runner->get_logger(),
      "bag_path, output_dir and cloud_topic are required");
    rclcpp::shutdown();
    return 1;
  }
  std::filesystem::create_directories(output_dir);

  auto component = std::make_shared<graphslam::ScanMatcherComponent>(
    rclcpp::NodeOptions().use_intra_process_comms(true));

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

  RCLCPP_INFO(
    runner->get_logger(),
    "frontend offline run complete: clouds=%zu imu=%zu poses=%zu submaps=%zu",
    cloud_count, imu_count, poses.size(), latest_map_array.submaps.size());
  RCLCPP_INFO(runner->get_logger(), "trajectory: %s", trajectory_path.c_str());
  RCLCPP_INFO(runner->get_logger(), "submaps:    %s", submaps_path.c_str());

  rclcpp::shutdown();
  return 0;
}
