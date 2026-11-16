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
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "graph_based_slam/cloud_message.hpp"
#include "graph_based_slam/offline_pipeline.hpp"
#include "graph_based_slam/offline_runtime.hpp"

namespace
{

class RosLog : public graphslam::offline::Log
{
public:
  explicit RosLog(rclcpp::Logger logger)
  : logger_(std::move(logger)) {}
  void info(const std::string & message) const override
  {
    RCLCPP_INFO(logger_, "%s", message.c_str());
  }
  void warn(const std::string & message) const override
  {
    RCLCPP_WARN(logger_, "%s", message.c_str());
  }
  void error(const std::string & message) const override
  {
    RCLCPP_ERROR(logger_, "%s", message.c_str());
  }

private:
  rclcpp::Logger logger_;
};

// Every parameter the node holds (its overrides, declared automatically), with its ROS type.
graphslam::offline::Parameters nodeParameters(rclcpp::Node & node)
{
  graphslam::offline::Parameters parameters;
  const auto names = node.list_parameters({}, 0).names;
  for (const rclcpp::Parameter & parameter : node.get_parameters(names)) {
    switch (parameter.get_type()) {
      case rclcpp::ParameterType::PARAMETER_BOOL:
        parameters.set(parameter.get_name(), parameter.as_bool());
        break;
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        parameters.set(parameter.get_name(), static_cast<std::int64_t>(parameter.as_int()));
        break;
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
        parameters.set(parameter.get_name(), parameter.as_double());
        break;
      case rclcpp::ParameterType::PARAMETER_STRING:
        parameters.set(parameter.get_name(), parameter.as_string());
        break;
      case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
        parameters.set(parameter.get_name(), parameter.as_byte_array());
        break;
      case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
        parameters.set(parameter.get_name(), parameter.as_bool_array());
        break;
      case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
        {
          const auto & values = parameter.as_integer_array();
          parameters.set(
            parameter.get_name(), std::vector<std::int64_t>(values.begin(), values.end()));
          break;
        }
      case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
        parameters.set(parameter.get_name(), parameter.as_double_array());
        break;
      case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
        parameters.set(parameter.get_name(), parameter.as_string_array());
        break;
      case rclcpp::ParameterType::PARAMETER_NOT_SET:
        break;
    }
  }
  return parameters;
}

graphslam::msg::Odometry toPlain(const nav_msgs::msg::Odometry & in)
{
  graphslam::msg::Odometry out;
  out.header.stamp.sec = in.header.stamp.sec;
  out.header.stamp.nanosec = in.header.stamp.nanosec;
  out.header.frame_id = in.header.frame_id;
  out.child_frame_id = in.child_frame_id;
  out.pose.pose.position.x = in.pose.pose.position.x;
  out.pose.pose.position.y = in.pose.pose.position.y;
  out.pose.pose.position.z = in.pose.pose.position.z;
  out.pose.pose.orientation.x = in.pose.pose.orientation.x;
  out.pose.pose.orientation.y = in.pose.pose.orientation.y;
  out.pose.pose.orientation.z = in.pose.pose.orientation.z;
  out.pose.pose.orientation.w = in.pose.pose.orientation.w;
  out.pose.covariance = in.pose.covariance;
  out.twist.twist.linear.x = in.twist.twist.linear.x;
  out.twist.twist.linear.y = in.twist.twist.linear.y;
  out.twist.twist.linear.z = in.twist.twist.linear.z;
  out.twist.twist.angular.x = in.twist.twist.angular.x;
  out.twist.twist.angular.y = in.twist.twist.angular.y;
  out.twist.twist.angular.z = in.twist.twist.angular.z;
  out.twist.covariance = in.twist.covariance;
  return out;
}

graphslam::msg::PointCloud2 toPlain(sensor_msgs::msg::PointCloud2 && in)
{
  graphslam::msg::PointCloud2 out;
  out.header.stamp.sec = in.header.stamp.sec;
  out.header.stamp.nanosec = in.header.stamp.nanosec;
  out.header.frame_id = in.header.frame_id;
  out.height = in.height;
  out.width = in.width;
  out.fields.resize(in.fields.size());
  for (std::size_t i = 0; i < in.fields.size(); ++i) {
    out.fields[i].name = in.fields[i].name;
    out.fields[i].offset = in.fields[i].offset;
    out.fields[i].datatype = in.fields[i].datatype;
    out.fields[i].count = in.fields[i].count;
  }
  out.is_bigendian = in.is_bigendian;
  out.point_step = in.point_step;
  out.row_step = in.row_step;
  out.data = std::move(in.data);
  out.is_dense = in.is_dense;
  return out;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "graph_based_slam",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  const RosLog log(node->get_logger());
  const graphslam::offline::Parameters parameters = nodeParameters(*node);

  graphslam::offline::OfflinePipeline pipeline(parameters, log);
  if (!pipeline.configure()) {
    rclcpp::shutdown();
    return 1;
  }

  rosbag2_cpp::Reader reader;
  reader.open(pipeline.inputPath());
  rclcpp::Serialization<nav_msgs::msg::Odometry> odom_serialization;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> cloud_serialization;
  pipeline.begin();
  while (reader.has_next()) {
    auto bag_message = reader.read_next();
    rclcpp::SerializedMessage serialized(*bag_message->serialized_data);
    if (bag_message->topic_name == pipeline.odometryTopic()) {
      nav_msgs::msg::Odometry ros_odom;
      odom_serialization.deserialize_message(&serialized, &ros_odom);
      pipeline.addOdometry(toPlain(ros_odom));
    } else if (bag_message->topic_name == pipeline.cloudTopic()) {
      sensor_msgs::msg::PointCloud2 ros_cloud;
      cloud_serialization.deserialize_message(&serialized, &ros_cloud);
      pipeline.addCloud(toPlain(std::move(ros_cloud)));
    }
  }

  const bool ok = pipeline.finish();
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
