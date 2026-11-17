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

// The multi-session join tool (graph_slam_multisession_join) without ROS 2: its parameters, the
// reading of each session (runner outputs and keyframe maps from the backend input log), the
// joins, the pose graph and the output files. The ROS 2 executable reads the logs with rosbag2 and
// the Python package (lidarslam.multisession) with rosbags; both give the same messages in the
// same order, so both write the same files.

#ifndef GRAPH_BASED_SLAM__MULTISESSION_JOIN_TOOL_HPP_
#define GRAPH_BASED_SLAM__MULTISESSION_JOIN_TOOL_HPP_

#include <array>
#include <cstdint>
#include <functional>
#include <string>

#include "graph_based_slam/cloud_message.hpp"
#include "graph_based_slam/offline_runtime.hpp"

namespace graphslam
{
namespace multisession
{

// sensor_msgs/NavSatFix, the fields the GNSS gate reads.
struct NavSatFixMessage
{
  msg::Time stamp;
  std::int8_t status = 0;  // NavSatStatus.status
  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
  std::array<double, 9> position_covariance{};
};

// Where the sessions' logs come from.
class SessionLogReader
{
public:
  virtual ~SessionLogReader() = default;
  // Every message of odometry_topic and cloud_topic in the log at path, in the log's order.
  virtual void readOdometryAndClouds(
    const std::string & path, const std::string & odometry_topic, const std::string & cloud_topic,
    const std::function<void(msg::Odometry &&)> & odometry,
    const std::function<void(msg::PointCloud2 &&)> & cloud) = 0;
  // Every message of topic in the log at path, in the log's order; none when the log has no such
  // topic.
  virtual void readNavSatFixes(
    const std::string & path, const std::string & topic,
    const std::function<void(const NavSatFixMessage &)> & fix) = 0;
};

// Runs graph_slam_multisession_join with its parameters (node graph_based_slam): returns the
// executable's exit code, 0 when the outputs are written, 1 (after logging why) when the
// parameters or the sessions cannot be used.
int runJoinTool(
  const offline::Parameters & parameters, const offline::Log & log, SessionLogReader & reader);

}  // namespace multisession
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__MULTISESSION_JOIN_TOOL_HPP_
