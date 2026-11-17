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

// The offline backend (graph_slam_offline_runner) as a library without ROS.
//
//   graphslam::offline::OfflinePipeline pipeline(parameters, log);
//   if (!pipeline.configure()) {...}           // parameters, loop registration, receipt
//   pipeline.begin();
//   for each recorded message on pipeline.odometryTopic() / pipeline.cloudTopic(), in bag order:
//     pipeline.addOdometry(odometry) / pipeline.addCloud(std::move(cloud));
//   if (!pipeline.finish()) {...}             // pose graph, refinement, outputs in outputDir()
//
// The parameters are those of the ROS 2 runner (the node graph_based_slam, e.g.
// lidarslam/param/lidarslam_mid360_rko_graph.yaml); odometry and clouds are paired by equal
// stamps. Given the same parameters and messages, the outputs are byte-identical to the ROS 2
// runner's, which is a thin shell around this class.

#ifndef GRAPH_BASED_SLAM__OFFLINE_PIPELINE_HPP_
#define GRAPH_BASED_SLAM__OFFLINE_PIPELINE_HPP_

#include <memory>
#include <string>

#include "graph_based_slam/cloud_message.hpp"
#include "graph_based_slam/gnss_anchor_store.hpp"
#include "graph_based_slam/offline_runtime.hpp"

namespace graphslam
{
namespace offline
{

class OfflinePipeline
{
public:
  // Both are borrowed and must outlive the pipeline.
  OfflinePipeline(const Parameters & parameters, const Log & log);
  ~OfflinePipeline();
  OfflinePipeline(const OfflinePipeline &) = delete;
  OfflinePipeline & operator=(const OfflinePipeline &) = delete;

  // Reads the parameters and activates the loop registration (writes
  // registration_plugin_receipt.yaml). Returns false, after logging why, when that fails.
  // Throws ParameterTypeError for a parameter of the wrong type.
  bool configure();

  // After configure(): bag_path, odom_topic, cloud_topic and output_dir.
  const std::string & inputPath() const;
  const std::string & odometryTopic() const;
  const std::string & cloudTopic() const;
  const std::string & outputDir() const;

  // The input, in recorded order, between begin() and finish().
  void begin();
  void addOdometry(const msg::Odometry & odometry);
  void addCloud(msg::PointCloud2 && cloud);
  // use_gnss: the NavSatFix messages of gnssTopic(), in recorded order with the odometry and the
  // clouds (ignored when use_gnss is false).
  bool usesGnss() const;
  const std::string & gnssTopic() const;
  void addGnssFix(const gnss::Fix & fix);

  // The remaining stages and all outputs. Returns false, after logging why, when the fixed loop
  // edges (fixed_loop_edges_path) cannot be loaded.
  bool finish();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace offline
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__OFFLINE_PIPELINE_HPP_
