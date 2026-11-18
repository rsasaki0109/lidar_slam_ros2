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

// The Autoware map bundle without ROS: the downsampled map split into grid cells
// (pointcloud_map/*.pcd + pointcloud_map_metadata.yaml), map.pcd, map_projector_info.yaml and the
// map_bundle.yaml manifest, written through a MapOutputPort. The live node (map_save) and the
// offline pipeline (offline_save_map_bundle) both write their bundle with these functions; the
// file contents come from map_saver.hpp.

#ifndef GRAPH_BASED_SLAM__MAP_EXPORT_HPP_
#define GRAPH_BASED_SLAM__MAP_EXPORT_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>  // NOLINT(build/include_order)

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

#include "graph_based_slam/dynamic_object_filter.hpp"
#include "graph_based_slam/external_io_ports.hpp"
#include "graph_based_slam/map_point.hpp"
#include "graph_based_slam/map_saver.hpp"
#include "graph_based_slam/planar_map_filter.hpp"

namespace graphslam
{
namespace map_export
{

// The map-saving parameters of graph_based_slam (GraphSlamConfig), with the same defaults.
struct MapExportConfig
{
  double map_leaf_size {0.2};
  double map_grid_size_x {20.0};
  double map_grid_size_y {20.0};
  // Above this many points the map is downsampled in passes without being assembled
  // (map_saver::voxelGridFilterStreamed); 0 always assembles it.
  int map_save_points_per_pass {2000000};
  bool use_planar_map_filter {false};
  PlanarMapFilterConfig planar_map_filter;
  bool use_dynamic_object_filter {false};
  DynamicObjectFilterConfig dynamic_object_filter;

  map_saver::GridConfig gridConfig() const;
};

// The GNSS origin written to map_projector_info.yaml (local projector when not set).
struct ProjectorOrigin
{
  bool set {false};
  double latitude {0.0};
  double longitude {0.0};
};

// Where the progress lines go: `line` for the lines the live node prints to stdout, `info` for
// the ones it logs.
struct MapExportLog
{
  std::function<void(const std::string &)> line;
  std::function<void(const std::string &)> info;
};

// A submap's cloud in the submap frame, by index.
using SubmapCloudLoader = std::function<pcl::PointCloud<pcl::PointXYZI>::Ptr(std::size_t)>;

// Submap i's cloud moved by poses[i] (float transform, as pcl::transformPointCloud).
void transformSubmapCloud(
  const SubmapCloudLoader & load, const std::vector<Eigen::Isometry3d> & poses, std::size_t i,
  pcl::PointCloud<pcl::PointXYZI> & transformed);

// The map assembled from every submap cloud moved by its pose, in submap order.
MapCloud::Ptr assembleMap(
  const SubmapCloudLoader & load, const std::vector<Eigen::Isometry3d> & poses,
  std::size_t map_points);

// Downsamples the map at map_leaf_size, frees it, and saves the bundle's map files to save_dir.
void saveGridDividedMap(
  MapCloud::Ptr map, const std::string & save_dir, const MapExportConfig & config,
  const ProjectorOrigin & origin, ports::MapOutputPort & output, const MapExportLog & log);

// As saveGridDividedMap for the map of the submap clouds moved by their poses, without assembling
// it when it has more than map_save_points_per_pass points.
void saveStreamedMap(
  const SubmapCloudLoader & load, const std::vector<Eigen::Isometry3d> & poses,
  std::size_t map_points, const std::string & save_dir, const MapExportConfig & config,
  const ProjectorOrigin & origin, ports::MapOutputPort & output, const MapExportLog & log);

// Saves an already downsampled map: pointcloud_map/ cells and metadata, map.pcd and
// map_projector_info.yaml (with the planar map filter first when it is on).
void saveDownsampledMap(
  MapCloud::Ptr downsampled, std::size_t map_points, const std::string & save_dir,
  const MapExportConfig & config, const ProjectorOrigin & origin,
  ports::MapOutputPort & output, const MapExportLog & log);

// The map_bundle.yaml manifest of a bundle saved with `config`.
map_saver::BundleManifest bundleManifest(
  const MapExportConfig & config, const std::string & frame_id, std::size_t submap_count,
  std::size_t loop_edge_count);

// The whole map bundle of the submap clouds moved by their poses, as the live node saves it:
// pose_graph.g2o (when not empty), the map files (the dynamic-object-filtered map with
// use_dynamic_object_filter, else streamed or assembled by map_save_points_per_pass) and
// map_bundle.yaml. map_points is the submaps' point count.
void writeBundle(
  const SubmapCloudLoader & load, const std::vector<Eigen::Isometry3d> & poses,
  std::size_t map_points, std::size_t loop_edge_count, const std::string & pose_graph_g2o,
  const std::string & save_dir, const MapExportConfig & config, const ProjectorOrigin & origin,
  ports::MapOutputPort & output, const MapExportLog & log);

}  // namespace map_export
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__MAP_EXPORT_HPP_
