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

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

#include "graph_based_slam/planar_map_filter.hpp"

namespace
{

void printUsage(const char * program)
{
  std::cerr << "usage: " << program <<
    " INPUT.pcd OUTPUT.pcd [voxel_size] [min_neighbors] "
    "[max_small_ratio] [min_middle_ratio] [min_retained_ratio]\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3 || argc > 8) {
    printUsage(argv[0]);
    return 2;
  }

  graphslam::PlanarMapFilterConfig config;
  try {
    if (argc >= 4) {
      config.voxel_size = std::stod(argv[3]);
    }
    if (argc >= 5) {
      config.min_neighbors = std::stoi(argv[4]);
    }
    if (argc >= 6) {
      config.max_small_eigenvalue_ratio = std::stod(argv[5]);
    }
    if (argc >= 7) {
      config.min_middle_eigenvalue_ratio = std::stod(argv[6]);
    }
    if (argc >= 8) {
      config.min_retained_ratio = std::stod(argv[7]);
    }
  } catch (const std::exception & error) {
    std::cerr << "invalid filter argument: " << error.what() << '\n';
    printUsage(argv[0]);
    return 2;
  }

  auto input = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  if (pcl::io::loadPCDFile(argv[1], *input) != 0) {
    std::cerr << "failed to load input PCD: " << argv[1] << '\n';
    return 1;
  }

  const auto result = graphslam::buildPlanarMapFilteredMap(input, config);
  if (pcl::io::savePCDFileBinaryCompressed(argv[2], *result.cloud) != 0) {
    std::cerr << "failed to save output PCD: " << argv[2] << '\n';
    return 1;
  }

  const double retained_ratio = result.stats.input_points == 0U ? 0.0 :
    static_cast<double>(result.stats.output_points) /
    static_cast<double>(result.stats.input_points);
  std::cout << std::boolalpha << std::setprecision(17)
            << "input_points: " << result.stats.input_points << '\n'
            << "finite_points: " << result.stats.finite_points << '\n'
            << "voxel_count: " << result.stats.voxel_count << '\n'
            << "planar_voxels: " << result.stats.planar_voxels << '\n'
            << "supported_points: " << result.stats.supported_points << '\n'
            << "output_points: " << result.stats.output_points << '\n'
            << "retained_ratio: " << retained_ratio << '\n'
            << "fallback_to_input: " << result.stats.fallback_to_input << '\n';
  return 0;
}
