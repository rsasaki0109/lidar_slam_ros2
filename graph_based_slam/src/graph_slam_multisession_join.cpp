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

// graph_slam_multisession_join: joins several sessions of one site into one pose graph by place
// recognition (multisession_join_tool.hpp describes the inputs, the join and the outputs). The
// work is graphslam::multisession::runJoinTool, which has no ROS dependency; this file is the ROS 2
// shell: node parameters in, the sessions' backend input bags read with rosbag2, rclcpp logging
// out. The Python package runs the same tool on logs read with rosbags (lidarslam.multisession).

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "graph_based_slam/cloud_message.hpp"
#include "graph_based_slam/multisession_join_tool.hpp"
#include "graph_based_slam/offline_runtime.hpp"
#include "offline_ros_bridge.hpp"

namespace
{

class RosbagSessionReader : public graphslam::multisession::SessionLogReader
{
public:
  void readOdometryAndClouds(
    const std::string & path, const std::string & odometry_topic, const std::string & cloud_topic,
    const std::function<void(graphslam::msg::Odometry &&)> & odometry,
    const std::function<void(graphslam::msg::PointCloud2 &&)> & cloud) override
  {
    rosbag2_cpp::Reader reader;
    reader.open(path);
    rclcpp::Serialization<nav_msgs::msg::Odometry> odom_serialization;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> cloud_serialization;
    while (reader.has_next()) {
      auto message = reader.read_next();
      rclcpp::SerializedMessage serialized(*message->serialized_data);
      if (message->topic_name == odometry_topic) {
        nav_msgs::msg::Odometry odom;
        odom_serialization.deserialize_message(&serialized, &odom);
        odometry(graphslam::offline::toPlain(odom));
      } else if (message->topic_name == cloud_topic) {
        sensor_msgs::msg::PointCloud2 ros_cloud;
        cloud_serialization.deserialize_message(&serialized, &ros_cloud);
        cloud(graphslam::offline::toPlain(std::move(ros_cloud)));
      }
    }
  }

  void readNavSatFixes(
    const std::string & path, const std::string & topic,
    const std::function<void(const graphslam::multisession::NavSatFixMessage &)> & fix) override
  {
    rosbag2_cpp::Reader reader;
    reader.open(path);
    bool has_topic = false;
    for (const auto & t : reader.get_all_topics_and_types()) {
      has_topic = has_topic || t.name == topic;
    }
    if (!has_topic) {
      return;
    }
    rosbag2_storage::StorageFilter filter;
    filter.topics = {topic};
    reader.set_filter(filter);
    rclcpp::Serialization<sensor_msgs::msg::NavSatFix> serialization;
    while (reader.has_next()) {
      auto message = reader.read_next();
      if (message->topic_name != topic) {
        continue;
      }
      rclcpp::SerializedMessage serialized(*message->serialized_data);
      sensor_msgs::msg::NavSatFix ros_fix;
      serialization.deserialize_message(&serialized, &ros_fix);
      graphslam::multisession::NavSatFixMessage plain;
      plain.stamp.sec = ros_fix.header.stamp.sec;
      plain.stamp.nanosec = ros_fix.header.stamp.nanosec;
      plain.status = ros_fix.status.status;
      plain.latitude = ros_fix.latitude;
      plain.longitude = ros_fix.longitude;
      plain.altitude = ros_fix.altitude;
      plain.position_covariance = ros_fix.position_covariance;
      fix(plain);
    }
  }
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "graph_based_slam",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  const graphslam::offline::RosLog log(node->get_logger());
  RosbagSessionReader reader;
  const int code = graphslam::multisession::runJoinTool(
    graphslam::offline::nodeParameters(*node), log, reader);
  rclcpp::shutdown();
  return code;
}
