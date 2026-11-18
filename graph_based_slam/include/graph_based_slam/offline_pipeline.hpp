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

#include <Eigen/Geometry>

#include <cstddef>
#include <functional>
#include <memory>
#include <string>

#include "graph_based_slam/cloud_message.hpp"
#include "graph_based_slam/gnss_anchor_store.hpp"
#include "graph_based_slam/map_export.hpp"
#include "graph_based_slam/map_store.hpp"
#include "graph_based_slam/offline_runtime.hpp"

namespace graphslam
{
namespace offline
{

// What a live system could output for one odometry / cloud pair, as the causal trajectory
// (causal_trajectory_output) defines it: the pair's odometry pose and that pose corrected by the
// latest pose-graph optimization the background worker had finished by the pair's stamp.
struct LivePose
{
  double stamp_sec {0.0};
  Eigen::Isometry3d odometry {Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d map {Eigen::Isometry3d::Identity()};
  // The pair started a submap.
  bool new_submap {false};
  // Submaps and corrections (optimizations whose result applies) so far.
  std::size_t submaps {0U};
  std::size_t corrections_applied {0U};
};

using LivePoseListener = std::function<void (const LivePose &)>;

// The map-saving parameters (offline_map_bundle's map_leaf_size, map_grid_size_x, ...), with the
// live node's defaults. Throws ParameterTypeError for a parameter of the wrong type.
map_export::MapExportConfig mapExportConfig(const Parameters & parameters);

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

  // Called for every odometry / cloud pair, as it is processed, with its live pose. Only with
  // causal_trajectory_output (without it the listener is not called). Set before begin(). The
  // outputs are the same with or without a listener.
  void setLivePoseListener(LivePoseListener listener);

  // Keep the session's submaps (stamps, odometry and optimized poses, compressed scans) for
  // sessionMap(), the session of a long-term map store (map_store.hpp). Set before begin(). The
  // outputs are the same either way.
  void keepSessionMap(bool keep);

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

  // After finish() with keepSessionMap(true): the session's submaps at their optimized poses
  // (the poses of the map bundle), its loop edge count, GNSS origin and pose graph.
  const map_store::SessionMap & sessionMap() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace offline
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__OFFLINE_PIPELINE_HPP_
