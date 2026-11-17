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
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the
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
//
// The stages are graphslam::offline::OfflinePipeline (offline_pipeline.hpp), which has no ROS
// dependency; this file is the ROS 2 shell: node parameters in, rosbag2 messages in, rclcpp
// logging out.

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "graph_based_slam/cloud_message.hpp"
#include "graph_based_slam/offline_pipeline.hpp"
#include "graph_based_slam/offline_runtime.hpp"
#include "offline_ros_bridge.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "graph_based_slam",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  const graphslam::offline::RosLog log(node->get_logger());
  const graphslam::offline::Parameters parameters = graphslam::offline::nodeParameters(*node);

  graphslam::offline::OfflinePipeline pipeline(parameters, log);
  if (!pipeline.configure()) {
    rclcpp::shutdown();
    return 1;
  }

  rosbag2_cpp::Reader reader;
  reader.open(pipeline.inputPath());
  rclcpp::Serialization<nav_msgs::msg::Odometry> odom_serialization;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> cloud_serialization;
  rclcpp::Serialization<sensor_msgs::msg::NavSatFix> fix_serialization;
  pipeline.begin();
  while (reader.has_next()) {
    auto bag_message = reader.read_next();
    rclcpp::SerializedMessage serialized(*bag_message->serialized_data);
    if (bag_message->topic_name == pipeline.odometryTopic()) {
      nav_msgs::msg::Odometry ros_odom;
      odom_serialization.deserialize_message(&serialized, &ros_odom);
      pipeline.addOdometry(graphslam::offline::toPlain(ros_odom));
    } else if (bag_message->topic_name == pipeline.cloudTopic()) {
      sensor_msgs::msg::PointCloud2 ros_cloud;
      cloud_serialization.deserialize_message(&serialized, &ros_cloud);
      pipeline.addCloud(graphslam::offline::toPlain(std::move(ros_cloud)));
    } else if (pipeline.usesGnss() && bag_message->topic_name == pipeline.gnssTopic()) {
      sensor_msgs::msg::NavSatFix ros_fix;
      fix_serialization.deserialize_message(&serialized, &ros_fix);
      pipeline.addGnssFix(graphslam::offline::toPlain(ros_fix));
    }
  }

  const bool ok = pipeline.finish();
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
