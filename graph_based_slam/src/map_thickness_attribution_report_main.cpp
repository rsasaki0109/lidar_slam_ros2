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

#include <exception>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "graph_based_slam/map_thickness_attribution.hpp"
#include "graph_based_slam/map_thickness_attribution_csv.hpp"

namespace
{

void printUsage(const char * program)
{
  std::cerr << "usage: " << program <<
    " INPUT.csv OUTPUT.yaml [root_voxel_size_m] [max_octree_depth] "
    "[min_points_per_plane] [max_plane_thickness_m] [min_planarity_ratio]\n"
    "CSV header: " << graphslam::map_thickness::attributedPointCsvHeader() << '\n';
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3 || argc > 8) {
    printUsage(argv[0]);
    return 2;
  }

  graphslam::map_thickness::AttributionConfig config;
  try {
    if (argc >= 4) {config.plane_config.root_voxel_size = std::stod(argv[3]);}
    if (argc >= 5) {config.plane_config.max_octree_depth = std::stoi(argv[4]);}
    if (argc >= 6) {config.plane_config.min_points_per_plane = std::stoi(argv[5]);}
    if (argc >= 7) {config.plane_config.max_plane_thickness = std::stod(argv[6]);}
    if (argc >= 8) {config.plane_config.min_planarity_ratio = std::stod(argv[7]);}
  } catch (const std::exception & error) {
    std::cerr << "invalid extraction argument: " << error.what() << '\n';
    printUsage(argv[0]);
    return 2;
  }

  std::ifstream input(argv[1]);
  if (!input.is_open()) {
    std::cerr << "failed to open input CSV: " << argv[1] << '\n';
    return 1;
  }

  std::vector<graphslam::map_thickness::AttributedPoint> points;
  try {
    points = graphslam::map_thickness::readAttributedPointCsv(input);
  } catch (const std::exception & error) {
    std::cerr << "failed to parse input CSV: " << error.what() << '\n';
    return 1;
  }

  const auto report = graphslam::map_thickness::computeAttribution(points, config);
  const auto lines = graphslam::map_thickness::reportYamlLines(report, config);
  std::ofstream output(argv[2]);
  if (!output.is_open()) {
    std::cerr << "failed to open output YAML: " << argv[2] << '\n';
    return 1;
  }
  for (const std::string & line : lines) {
    output << line << '\n';
  }
  if (!output.good()) {
    std::cerr << "failed to write output YAML: " << argv[2] << '\n';
    return 1;
  }

  std::cout << "wrote " << argv[2] << " from " << report.input_points << " points; "
            << "planar coverage=" << report.planar_coverage << ", meaningful=" << std::boolalpha
            << report.meaningful << '\n';
  return report.meaningful ? 0 : 3;
}
