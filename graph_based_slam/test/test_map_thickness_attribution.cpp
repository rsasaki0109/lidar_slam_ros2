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

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

#include "graph_based_slam/map_thickness_attribution.hpp"
#include "graph_based_slam/map_thickness_attribution_csv.hpp"

namespace
{

using graphslam::map_thickness::AttributedPoint;
using graphslam::map_thickness::AttributionConfig;
using graphslam::map_thickness::AttributionReport;
using graphslam::map_thickness::computeAttribution;
using graphslam::map_thickness::readAttributedPointCsv;
using graphslam::map_thickness::reportYamlLines;

std::vector<AttributedPoint> makeHierarchicalPlane()
{
  std::vector<AttributedPoint> points;
  for (int revisit = 0; revisit < 2; ++revisit) {
    const double revisit_offset = revisit == 0 ? -0.08 : 0.08;
    for (int submap_local = 0; submap_local < 2; ++submap_local) {
      const double submap_offset = submap_local == 0 ? -0.04 : 0.04;
      const int submap = revisit * 2 + submap_local;
      for (int scan_local = 0; scan_local < 2; ++scan_local) {
        const double scan_offset = scan_local == 0 ? -0.02 : 0.02;
        const int scan = submap * 2 + scan_local;
        for (int ix = 0; ix < 4; ++ix) {
          for (int iy = 0; iy < 4; ++iy) {
            AttributedPoint point;
            const double within_offset = ((ix + iy) % 2 == 0) ? -0.01 : 0.01;
            point.position = Eigen::Vector3d(
              0.1 + 0.2 * static_cast<double>(ix),
              0.1 + 0.2 * static_cast<double>(iy),
              0.5 + revisit_offset + submap_offset + scan_offset + within_offset);
            point.scan_id = scan;
            point.submap_id = submap;
            point.revisit_id = revisit;
            points.push_back(point);
          }
        }
      }
    }
  }
  return points;
}

AttributionConfig testConfig()
{
  AttributionConfig config;
  config.plane_config.root_voxel_size = 1.0;
  config.plane_config.max_octree_depth = 0;
  config.plane_config.min_points_per_plane = 20;
  config.plane_config.max_plane_thickness = 0.15;
  config.plane_config.min_planarity_ratio = 4.0;
  config.plane_config.enable_quarter_test = false;
  return config;
}

}  // namespace

TEST(MapThicknessAttribution, DecomposesNestedVarianceIntoFourExactComponents)
{
  const std::vector<AttributedPoint> points = makeHierarchicalPlane();
  const AttributionReport report = computeAttribution(points, testConfig());

  ASSERT_TRUE(report.meaningful);
  ASSERT_EQ(report.plane_patch_count, 1);
  EXPECT_EQ(report.planar_points, static_cast<std::int64_t>(points.size()));
  EXPECT_EQ(report.distinct_scans, 8);
  EXPECT_EQ(report.distinct_submaps, 4);
  EXPECT_EQ(report.distinct_revisits, 2);
  EXPECT_NEAR(report.within_scan_rms_m, 0.01, 1.0e-12);
  EXPECT_NEAR(report.between_scan_rms_m, 0.02, 1.0e-12);
  EXPECT_NEAR(report.between_submap_rms_m, 0.04, 1.0e-12);
  EXPECT_NEAR(report.between_revisit_rms_m, 0.08, 1.0e-12);
  EXPECT_NEAR(report.total_rms_m, std::sqrt(0.0085), 1.0e-12);
  EXPECT_NEAR(
    report.within_scan_fraction + report.between_scan_fraction +
    report.between_submap_fraction + report.between_revisit_fraction,
    1.0, 1.0e-12);
  EXPECT_LT(report.closure_error, 1.0e-12);
}

TEST(MapThicknessAttribution, EmptyInputIsExplicitlyNotMeaningful)
{
  const AttributionReport report = computeAttribution({}, testConfig());
  EXPECT_EQ(report.input_points, 0);
  EXPECT_FALSE(report.meaningful);
  EXPECT_EQ(report.total_rms_m, 0.0);
}

TEST(MapThicknessAttribution, ReportIsByteDeterministic)
{
  const std::vector<AttributedPoint> points = makeHierarchicalPlane();
  const AttributionConfig config = testConfig();
  const AttributionReport first = computeAttribution(points, config);
  const AttributionReport second = computeAttribution(points, config);

  EXPECT_EQ(0, std::memcmp(&first.total_sse, &second.total_sse, sizeof(double)));
  EXPECT_EQ(
    0, std::memcmp(&first.within_scan_sse, &second.within_scan_sse, sizeof(double)));
  const std::vector<std::string> first_lines = reportYamlLines(first, config);
  const std::vector<std::string> second_lines = reportYamlLines(second, config);
  EXPECT_EQ(first_lines, second_lines);
}

TEST(MapThicknessAttribution, FullTupleKeepsReusedScanIdsDistinct)
{
  std::vector<AttributedPoint> points = makeHierarchicalPlane();
  for (auto & point : points) {
    point.scan_id %= 2;
    point.submap_id %= 2;
  }
  const AttributionReport report = computeAttribution(points, testConfig());

  EXPECT_EQ(report.distinct_scans, 8);
  EXPECT_EQ(report.distinct_submaps, 4);
  EXPECT_EQ(report.distinct_revisits, 2);
  EXPECT_NEAR(report.within_scan_rms_m, 0.01, 1.0e-12);
  EXPECT_LT(report.closure_error, 1.0e-12);
}

TEST(MapThicknessAttribution, CsvContractParsesStableIdsAndCoordinates)
{
  std::istringstream csv(
    "x,y,z,scan_id,submap_id,revisit_id\r\n"
    "1.25,-2.5,3,42,7,2\r\n");
  const std::vector<AttributedPoint> points = readAttributedPointCsv(csv);

  ASSERT_EQ(points.size(), 1U);
  EXPECT_EQ(points[0].position, Eigen::Vector3d(1.25, -2.5, 3.0));
  EXPECT_EQ(points[0].scan_id, 42);
  EXPECT_EQ(points[0].submap_id, 7);
  EXPECT_EQ(points[0].revisit_id, 2);
}

TEST(MapThicknessAttribution, CsvContractRejectsNonFiniteCoordinates)
{
  std::istringstream csv(
    "x,y,z,scan_id,submap_id,revisit_id\n"
    "nan,0,0,0,0,0\n");
  EXPECT_THROW(readAttributedPointCsv(csv), std::runtime_error);
}
