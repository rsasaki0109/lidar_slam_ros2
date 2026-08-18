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

#include <pcl/io/pcd_io.h>  // NOLINT(build/include_order)
#include <pcl/point_types.h>  // NOLINT(build/include_order)

#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "graph_based_slam/ndt_localization_target.hpp"

namespace
{

struct Options
{
  std::string input_path;
  std::string output_path;
  std::string report_path;
  graphslam::ndt_localization::TangentSamplingConfig config;
};

Options parseOptions(int argc, char ** argv)
{
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (i + 1 >= argc) {throw std::runtime_error("missing value for " + arg);}
    const std::string value(argv[++i]);
    if (arg == "--input") {
      options.input_path = value;
    } else if (arg == "--output") {
      options.output_path = value;
    } else if (arg == "--report") {
      options.report_path = value;
    } else if (arg == "--resolution") {
      options.config.voxel_size_m = std::stod(value);
    } else if (arg == "--radius") {
      options.config.radius_m = std::stod(value);
    } else if (arg == "--inner-radius") {
      options.config.inner_radius_m = std::stod(value);
    } else if (arg == "--angular-midpoints") {
      options.config.add_angular_midpoints = value == "true" || value == "1";
    } else if (arg == "--angular-midpoint-pairs") {
      options.config.angular_midpoint_pairs = static_cast<std::size_t>(std::stoul(value));
    } else if (arg == "--diagonals") {
      options.config.add_diagonals = value == "true" || value == "1";
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }
  if (options.input_path.empty() || options.output_path.empty() || options.report_path.empty()) {
    throw std::runtime_error(
            "usage: ndt_localization_target --input MAP.pcd --output TARGET.pcd "
            "--report REPORT.yaml [--resolution M] [--radius M] [--inner-radius M] "
            "[--diagonals true|false] [--angular-midpoints true|false] "
            "[--angular-midpoint-pairs 0..4]");
  }
  return options;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Options options = parseOptions(argc, argv);
    pcl::PointCloud<pcl::PointXYZ> input_cloud;
    if (pcl::io::loadPCDFile(options.input_path, input_cloud) != 0 || input_cloud.empty()) {
      throw std::runtime_error("failed to load input map: " + options.input_path);
    }
    std::vector<Eigen::Vector3d> input;
    input.reserve(input_cloud.size());
    for (const pcl::PointXYZ & point : input_cloud) {
      input.emplace_back(point.x, point.y, point.z);
    }
    const auto result = graphslam::ndt_localization::buildTangentSampledTarget(
      input, options.config);
    if (result.points.empty()) {throw std::runtime_error("tangent sampling produced no output");}

    pcl::PointCloud<pcl::PointXYZ> output_cloud;
    output_cloud.reserve(result.points.size());
    for (const Eigen::Vector3d & point : result.points) {
      output_cloud.push_back(pcl::PointXYZ(
        static_cast<float>(point.x()), static_cast<float>(point.y()),
        static_cast<float>(point.z())));
    }
    if (pcl::io::savePCDFileBinary(options.output_path, output_cloud) != 0) {
      throw std::runtime_error("failed to save output map: " + options.output_path);
    }

    std::ofstream report(options.report_path);
    if (!report.is_open()) {
      throw std::runtime_error("failed to open report: " + options.report_path);
    }
    report << std::setprecision(17);
    report << "ndt_localization_target:\n";
    report << "  schema_version: 1\n";
    report << "  enabled_by_default: false\n";
    report << "  input_path: " << options.input_path << "\n";
    report << "  output_path: " << options.output_path << "\n";
    report << "  voxel_size_m: " << options.config.voxel_size_m << "\n";
    report << "  tangent_radius_m: " << options.config.radius_m << "\n";
    report << "  tangent_inner_radius_m: " << options.config.inner_radius_m << "\n";
    report << "  diagonal_samples: " << std::boolalpha << options.config.add_diagonals << "\n";
    report << "  angular_midpoint_samples: " << options.config.add_angular_midpoints << "\n";
    report << "  angular_midpoint_pairs: " << options.config.angular_midpoint_pairs << "\n";
    report << "  input_points: " << result.input_points << "\n";
    report << "  planar_voxels: " << result.planar_voxels << "\n";
    report << "  planar_input_points: " << result.planar_input_points << "\n";
    report << "  sampled_points: " << result.sampled_points << "\n";
    report << "  output_points: " << result.points.size() << "\n";
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
