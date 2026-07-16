// Copyright 2026 Sasaki
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)

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
