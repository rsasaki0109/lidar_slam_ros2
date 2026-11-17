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

// The ROS 2 side of the offline executables (graph_slam_offline_runner,
// graph_slam_multisession_join): node parameters as offline::Parameters, rclcpp logging as
// offline::Log, and the backend input's messages as the plain messages of cloud_message.hpp.

#ifndef OFFLINE_ROS_BRIDGE_HPP_
#define OFFLINE_ROS_BRIDGE_HPP_

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "graph_based_slam/cloud_message.hpp"
#include "graph_based_slam/gnss_anchor_store.hpp"
#include "graph_based_slam/offline_runtime.hpp"

namespace graphslam
{
namespace offline
{


class RosLog : public Log
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
inline Parameters nodeParameters(rclcpp::Node & node)
{
  Parameters parameters;
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

inline msg::Odometry toPlain(const nav_msgs::msg::Odometry & in)
{
  msg::Odometry out;
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

inline msg::PointCloud2 toPlain(sensor_msgs::msg::PointCloud2 && in)
{
  msg::PointCloud2 out;
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

inline gnss::Fix toPlain(const sensor_msgs::msg::NavSatFix & in)
{
  gnss::Fix out;
  out.header_stamp_sec = cloud_msg::stampSeconds(in.header.stamp);
  out.status = in.status.status;
  out.latitude = in.latitude;
  out.longitude = in.longitude;
  out.altitude = in.altitude;
  out.position_covariance = in.position_covariance;
  out.position_covariance_type = in.position_covariance_type;
  return out;
}

}  // namespace offline
}  // namespace graphslam

#endif  // OFFLINE_ROS_BRIDGE_HPP_
