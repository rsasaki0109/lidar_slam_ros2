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

// Moved from graph_slam_ros_adapter.cpp (saveGridDividedMap, saveStreamedMap, saveDownsampledMap,
// assembleSavedMap, moveSubmapCloud) so that the offline pipeline writes the same bundle; the
// steps and their order are unchanged.

#include "graph_based_slam/map_export.hpp"

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <cstdint>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "graph_based_slam/voxel_grid_overflow.hpp"

namespace graphslam
{
namespace map_export
{

map_saver::GridConfig MapExportConfig::gridConfig() const
{
  map_saver::GridConfig result;
  result.grid_size_x = map_grid_size_x;
  result.grid_size_y = map_grid_size_y;
  return result;
}

void transformSubmapCloud(
  const SubmapCloudLoader & load, const std::vector<Eigen::Isometry3d> & poses, std::size_t i,
  pcl::PointCloud<pcl::PointXYZI> & transformed)
{
  // As doPoseAdjustment moves a submap cloud into the map.
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr = load(i);
  const Eigen::Affine3d se3(poses[i].matrix());
  pcl::transformPointCloud(*cloud_ptr, transformed, se3.matrix().cast<float>());
}

MapCloud::Ptr assembleMap(
  const SubmapCloudLoader & load, const std::vector<Eigen::Isometry3d> & poses,
  std::size_t map_points)
{
  // As doPoseAdjustment assembles a map it only saves.
  MapCloud::Ptr map(new MapCloud);
  map->reserve(map_points);
  pcl::PointCloud<pcl::PointXYZI> transformed;
  for (std::size_t i = 0; i < poses.size(); ++i) {
    transformSubmapCloud(load, poses, i, transformed);
    for (const auto & p : transformed.points) {
      map->points.emplace_back(p);
    }
    map->width = static_cast<std::uint32_t>(map->size());
    map->height = 1;
    map->is_dense = map->is_dense && transformed.is_dense;
  }
  return map;
}

void saveGridDividedMap(
  MapCloud::Ptr map, const std::string & save_dir, const MapExportConfig & config,
  const ProjectorOrigin & origin, ports::MapOutputPort & output, const MapExportLog & log)
{
  if (map->empty()) {
    log.line("Map is empty, skipping save.");
    return;
  }

  // Downsample the map, then free the full map: everything below uses the downsampled one.
  // MapPoint clouds give the same points and files as PointXYZI ones (map_point.hpp).
  MapCloud::Ptr downsampled(new MapCloud);
  const std::size_t map_points = map->size();
  const float leaf_size = static_cast<float>(config.map_leaf_size);
  if (map_saver::voxelGridWouldCopyInput(*map, leaf_size)) {
    // VoxelGrid would only copy the map (32-bit voxel index overflow): downsample it in parts.
    std::ostringstream message;
    message << "[map save] map too large for VoxelGrid's 32-bit voxel indices at leaf " <<
      leaf_size << " m; downsampling it in parts";
    log.line(message.str());
    map_saver::voxelGridFilterInParts(*map, leaf_size, *downsampled);
  } else {
    map_saver::voxelGridFilter(map, leaf_size, *downsampled);
  }
  map.reset();
  saveDownsampledMap(std::move(downsampled), map_points, save_dir, config, origin, output, log);
}

void saveStreamedMap(
  const SubmapCloudLoader & load, const std::vector<Eigen::Isometry3d> & poses,
  std::size_t map_points, const std::string & save_dir, const MapExportConfig & config,
  const ProjectorOrigin & origin, ports::MapOutputPort & output, const MapExportLog & log)
{
  const std::size_t points_per_pass =
    static_cast<std::size_t>(config.map_save_points_per_pass);
  if (map_points <= points_per_pass) {
    // The map fits in one pass: assembled and downsampled with one VoxelGrid as before, which
    // takes less memory than a pass (and gives the same bytes).
    saveGridDividedMap(
      assembleMap(load, poses, map_points), save_dir, config, origin, output, log);
    return;
  }
  MapCloud::Ptr downsampled(new MapCloud);
  const std::size_t passes = map_saver::voxelGridFilterStreamed(
    poses.size(),
    [&](std::size_t i, MapCloud & part) {
      pcl::PointCloud<pcl::PointXYZI> transformed;
      transformSubmapCloud(load, poses, i, transformed);
      part.points.reserve(transformed.size());
      for (const auto & p : transformed.points) {
        part.points.emplace_back(p);
      }
    },
    static_cast<float>(config.map_leaf_size), points_per_pass, *downsampled);
  std::ostringstream message;
  message << "[map save] " << map_points << " points downsampled without assembling the map, in " <<
    passes << " passes of at most " << points_per_pass << " points";
  log.line(message.str());
  saveDownsampledMap(std::move(downsampled), map_points, save_dir, config, origin, output, log);
}

void saveDownsampledMap(
  MapCloud::Ptr downsampled, std::size_t map_points, const std::string & save_dir,
  const MapExportConfig & config, const ProjectorOrigin & origin,
  ports::MapOutputPort & output, const MapExportLog & log)
{
  // Create output directory (clean existing PCD files to prevent orphans)
  std::string out_dir = save_dir + "/pointcloud_map";
  output.prepareDirectory(out_dir);
  output.removeByExtension(out_dir, {".pcd", ".yaml"});

  log.line(map_saver::downsampleLogLine(map_points, downsampled->size(), config.map_leaf_size));

  // Apply map-quality refinement to the same leaf-sized point set that is
  // exported and evaluated. Filtering before this VoxelGrid changes local
  // covariance according to raw submap sampling density and does not match
  // the saved-map contract.
  if (config.use_planar_map_filter) {
    const auto filter_result =
      buildPlanarMapFilteredMap(toPointXYZICloud(*downsampled), config.planar_map_filter);
    downsampled = toMapCloud(*filter_result.cloud);
    std::ostringstream message;
    message << "Planar map filter: input_points " << filter_result.stats.input_points <<
      ", finite " << filter_result.stats.finite_points << ", planar_voxels " <<
      filter_result.stats.planar_voxels << "/" << filter_result.stats.voxel_count <<
      ", supported_points " << filter_result.stats.supported_points << ", output_points " <<
      filter_result.stats.output_points << ", fallback " <<
      (filter_result.stats.fallback_to_input ? "true" : "false");
    log.info(message.str());
  }

  // Compute bounding box and grid-aligned bounds (semantics pinned by
  // test_map_saver.cpp)
  MapPoint min_pt, max_pt;
  pcl::getMinMax3D(*downsampled, min_pt, max_pt);

  const map_saver::GridConfig grid_config = config.gridConfig();
  const map_saver::GridBounds grid_bounds =
    map_saver::computeGridBounds(min_pt.x, min_pt.y, max_pt.x, max_pt.y, grid_config);

  // Assign points to grid cells. Only the point indices are grouped; each cell's cloud
  // is built (points in map order, as before) and written one cell at a time.
  std::map<std::pair<int, int>, std::vector<std::size_t>> grid_cells;
  for (std::size_t j = 0; j < downsampled->points.size(); ++j) {
    const auto & pt = downsampled->points[j];
    grid_cells[map_saver::cellIndexFor(pt.x, pt.y, grid_bounds, grid_config)].push_back(j);
  }

  // Save each grid cell as PCD and build the Autoware pointcloud_map_loader
  // metadata (content produced in map_saver.hpp)
  std::ostringstream meta;
  meta << map_saver::metadataHeader(grid_config);

  int saved = 0;
  for (auto & [key, indices] : grid_cells) {
    if (indices.empty()) {continue;}
    MapCloud cloud;
    cloud.reserve(indices.size());
    for (const std::size_t j : indices) {
      cloud.push_back(downsampled->points[j]);
    }
    std::vector<std::size_t>().swap(indices);
    const map_saver::CellFile cell = map_saver::makeCellFile(key, grid_bounds, grid_config);
    output.writeMapCloud(out_dir + "/" + cell.filename, cloud);
    meta << map_saver::metadataEntry(cell);
    saved++;
  }
  output.writeBytes(out_dir + "/pointcloud_map_metadata.yaml", meta.str());

  // Also save the full map as a single PCD for convenience
  output.writeMapCloud(save_dir + "/map.pcd", *downsampled);

  log.line(map_saver::savedMapLogLine(saved, grid_config, out_dir));
  log.line("Total points: " + std::to_string(downsampled->size()));
  log.line("Metadata: " + out_dir + "/pointcloud_map_metadata.yaml");

  // Always emit map_projector_info.yaml so Autoware can load pointcloud-only maps.
  std::string proj_file = save_dir + "/map_projector_info.yaml";
  output.writeBytes(
    proj_file, map_saver::projectorInfoYaml(origin.set, origin.latitude, origin.longitude));
  log.line(map_saver::projectorInfoLogLine(origin.set, proj_file));
}

map_saver::BundleManifest bundleManifest(
  const MapExportConfig & config, const std::string & frame_id, std::size_t submap_count,
  std::size_t loop_edge_count)
{
  map_saver::BundleManifest manifest;
  manifest.frame_id = frame_id.empty() ? "map" : frame_id;
  manifest.submap_count = submap_count;
  manifest.loop_edge_count = loop_edge_count;
  manifest.map_leaf_size = config.map_leaf_size;
  manifest.grid_size_x = config.map_grid_size_x;
  manifest.grid_size_y = config.map_grid_size_y;
  manifest.dynamic_object_filter = config.use_dynamic_object_filter;
  manifest.planar_map_filter = config.use_planar_map_filter;
  manifest.planar_map_filter_voxel_size = config.planar_map_filter.voxel_size;
  manifest.planar_map_filter_min_neighbors = config.planar_map_filter.min_neighbors;
  manifest.planar_map_filter_max_small_eigenvalue_ratio =
    config.planar_map_filter.max_small_eigenvalue_ratio;
  manifest.planar_map_filter_min_middle_eigenvalue_ratio =
    config.planar_map_filter.min_middle_eigenvalue_ratio;
  manifest.planar_map_filter_min_retained_ratio = config.planar_map_filter.min_retained_ratio;
  return manifest;
}

void writeBundle(
  const SubmapCloudLoader & load, const std::vector<Eigen::Isometry3d> & poses,
  std::size_t map_points, std::size_t loop_edge_count, const std::string & pose_graph_g2o,
  const std::string & save_dir, const MapExportConfig & config, const ProjectorOrigin & origin,
  ports::MapOutputPort & output, const MapExportLog & log)
{
  output.prepareDirectory(save_dir);
  if (!pose_graph_g2o.empty()) {
    output.writeBytes(save_dir + "/pose_graph.g2o", pose_graph_g2o);
  }
  if (config.use_dynamic_object_filter) {
    // As the live node: the assembled map, or the dynamic-filtered one when it is not empty.
    std::vector<TimedSubmapCloud> submaps;
    MapCloud::Ptr map = assembleMap(load, poses, map_points);
    for (std::size_t i = 0; i < poses.size(); ++i) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>);
      transformSubmapCloud(load, poses, i, *transformed);
      const Eigen::Vector3d position = poses[i].translation();
      submaps.push_back(TimedSubmapCloud{static_cast<int>(i), position, transformed});
    }
    const auto filter_result = buildDynamicObjectFilteredMap(submaps, config.dynamic_object_filter);
    if (!filter_result.cloud->empty()) {
      map = toMapCloud(*filter_result.cloud);
    }
    submaps.clear();
    saveGridDividedMap(std::move(map), save_dir, config, origin, output, log);
  } else if (config.map_save_points_per_pass > 0) {
    saveStreamedMap(load, poses, map_points, save_dir, config, origin, output, log);
  } else {
    saveGridDividedMap(
      assembleMap(load, poses, map_points), save_dir, config, origin, output, log);
  }
  output.writeBytes(
    save_dir + "/map_bundle.yaml",
    map_saver::bundleManifestYaml(bundleManifest(config, "map", poses.size(), loop_edge_count)));
}

}  // namespace map_export
}  // namespace graphslam
