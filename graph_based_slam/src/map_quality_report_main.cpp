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

// map_quality_report: deterministic map-quality metrics over a PCD map
// (docs/roadmap/v0.7.md, Phase 1). Input is a single .pcd file or a
// directory of .pcd cells (e.g. the Autoware pointcloud_map/ bundle,
// loaded in sorted filename order); output is map_quality_report.yaml
// with Mean Map Entropy, plane-thickness statistics (with planar
// coverage and an explicit not-meaningful state) and density stats.
// No ROS, no wall clock, no randomness: the same input bytes produce
// the same report bytes, and the release gate relies on that.

#include <pcl/io/pcd_io.h>  // NOLINT(build/include_order)
#include <pcl/point_types.h>  // NOLINT(build/include_order)

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "graph_based_slam/map_quality_metrics.hpp"

namespace
{

void printUsage()
{
  std::cout <<
    "usage: map_quality_report --input <map.pcd | pcd_dir> --output-dir <dir>\n"
    "  [--downsample <m>]            deterministic voxel-centroid downsample (default 0 = off)\n"
    "  [--mme-radius <m>]            Mean Map Entropy neighborhood radius (default 0.5)\n"
    "  [--mme-min-neighbors <n>]     minimum neighbors for a valid MME point (default 8)\n"
    "  [--root-voxel-size <m>]       plane extractor root voxel size (default 1.0)\n"
    "  [--min-meaningful-coverage <f>] planar coverage floor for meaningful plane metrics\n"
    "                                  (default 0.10)\n";
}

bool loadPcdInto(const std::string & path, std::vector<Eigen::Vector3d> & points)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, cloud) == -1) {
    std::cerr << "error: failed to load " << path << std::endl;
    return false;
  }
  points.reserve(points.size() + cloud.size());
  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto & p = cloud.points[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {continue;}
    points.push_back(Eigen::Vector3d(p.x, p.y, p.z));
  }
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string input;
  std::string output_dir;
  graphslam::map_quality::MapQualityConfig config;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const bool has_value = i + 1 < argc;
    if (arg == "--input" && has_value) {
      input = argv[++i];
    } else if (arg == "--output-dir" && has_value) {
      output_dir = argv[++i];
    } else if (arg == "--downsample" && has_value) {
      config.downsample_voxel_size = std::stod(argv[++i]);
    } else if (arg == "--mme-radius" && has_value) {
      config.mme_radius = std::stod(argv[++i]);
    } else if (arg == "--mme-min-neighbors" && has_value) {
      config.mme_min_neighbors = std::stoi(argv[++i]);
    } else if (arg == "--root-voxel-size" && has_value) {
      config.plane_config.root_voxel_size = std::stod(argv[++i]);
    } else if (arg == "--min-meaningful-coverage" && has_value) {
      config.min_meaningful_planar_coverage = std::stod(argv[++i]);
    } else {
      printUsage();
      return arg == "--help" ? 0 : 1;
    }
  }
  if (input.empty() || output_dir.empty()) {
    printUsage();
    return 1;
  }

  std::vector<Eigen::Vector3d> points;
  if (std::filesystem::is_directory(input)) {
    std::vector<std::string> pcd_files;
    for (const auto & entry : std::filesystem::directory_iterator(input)) {
      if (entry.path().extension() == ".pcd") {
        pcd_files.push_back(entry.path().string());
      }
    }
    // Sorted filename order: the input point order (and therefore the
    // report bytes) must not depend on directory iteration order.
    std::sort(pcd_files.begin(), pcd_files.end());
    if (pcd_files.empty()) {
      std::cerr << "error: no .pcd files in " << input << std::endl;
      return 1;
    }
    for (size_t i = 0; i < pcd_files.size(); ++i) {
      if (!loadPcdInto(pcd_files[i], points)) {return 1;}
    }
    std::cout << "loaded " << pcd_files.size() << " pcd cells, " << points.size() <<
      " finite points" << std::endl;
  } else {
    if (!loadPcdInto(input, points)) {return 1;}
    std::cout << "loaded " << points.size() << " finite points" << std::endl;
  }

  const graphslam::map_quality::MapQualityReport report =
    graphslam::map_quality::computeMapQuality(points, config);
  const std::vector<std::string> lines =
    graphslam::map_quality::reportYamlLines(report, config);

  std::filesystem::create_directories(output_dir);
  const std::string yaml_path = output_dir + "/map_quality_report.yaml";
  std::ofstream yaml(yaml_path, std::ios::binary);
  if (!yaml) {
    std::cerr << "error: cannot write " << yaml_path << std::endl;
    return 1;
  }
  for (size_t i = 0; i < lines.size(); ++i) {
    yaml << lines[i] << "\n";
    std::cout << lines[i] << "\n";
  }
  yaml.close();
  std::cout << "wrote " << yaml_path << std::endl;
  return 0;
}
