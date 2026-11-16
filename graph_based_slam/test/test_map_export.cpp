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

#include <gtest/gtest.h>

#include <cstring>
#include <random>
#include <string>
#include <vector>

#include "graph_based_slam/map_export.hpp"

namespace
{

using graphslam::map_export::MapExportConfig;
using graphslam::map_export::MapExportLog;
using graphslam::map_export::ProjectorOrigin;
using graphslam::map_export::SubmapCloudLoader;
using Cloud = pcl::PointCloud<pcl::PointXYZI>;

struct Submaps
{
  std::vector<Cloud::Ptr> clouds;
  std::vector<Eigen::Isometry3d> poses;
  std::size_t points = 0;

  SubmapCloudLoader loader() const
  {
    return [this](std::size_t i) {return Cloud::Ptr(new Cloud(*clouds[i]));};
  }
};

// Six submaps of a corridor 50 m long, each a wall and floor patch in its own frame.
Submaps makeSubmaps()
{
  Submaps s;
  std::mt19937 rng(7);
  std::uniform_real_distribution<float> u(-4.0F, 4.0F);
  for (int i = 0; i < 6; ++i) {
    Cloud::Ptr cloud(new Cloud);
    for (int k = 0; k < 3000; ++k) {
      pcl::PointXYZI p;
      p.x = u(rng);
      p.y = (k % 2 == 0) ? 2.0F : u(rng) * 0.5F;
      p.z = (k % 2 == 0) ? u(rng) * 0.3F : -1.0F;
      p.intensity = static_cast<float>(k % 50);
      cloud->push_back(p);
    }
    s.points += cloud->size();
    s.clouds.push_back(cloud);
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(8.0 * i, 0.1 * i, 0.0);
    pose.linear() = Eigen::AngleAxisd(0.05 * i, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    s.poses.push_back(pose);
  }
  return s;
}

MapExportLog quietLog(std::vector<std::string> * lines = nullptr)
{
  MapExportLog log;
  log.line = [lines](const std::string & line) {
      if (lines != nullptr) {
        lines->push_back(line);
      }
    };
  log.info = log.line;
  return log;
}

// Bitwise equal x, y, z and intensity. pcl::PointXYZI has padding that some PCL versions leave
// uninitialized and that no PCD file holds, so the points are not compared as whole structs.
bool sameCloud(const Cloud & a, const Cloud & b)
{
  if (a.size() != b.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.size(); ++i) {
    const pcl::PointXYZI & p = a.points[i];
    const pcl::PointXYZI & q = b.points[i];
    if (std::memcmp(p.data, q.data, 3 * sizeof(float)) != 0 ||
      std::memcmp(&p.intensity, &q.intensity, sizeof(float)) != 0)
    {
      return false;
    }
  }
  return true;
}

}  // namespace

TEST(MapExport, WritesTheGridDividedBundle)
{
  const Submaps s = makeSubmaps();
  MapExportConfig config;
  config.map_leaf_size = 0.5;
  graphslam::ports::InMemoryMapOutput output;
  graphslam::map_export::saveGridDividedMap(
    graphslam::map_export::assembleMap(s.loader(), s.poses, s.points), "out", config,
    ProjectorOrigin{}, output, quietLog());

  const std::string metadata = output.bytes("out/pointcloud_map/pointcloud_map_metadata.yaml");
  ASSERT_NE(metadata.find("x_resolution: 20"), std::string::npos);
  std::size_t cell_points = 0;
  std::size_t cells = 0;
  for (const std::string cell : {"-20_-20.pcd", "-20_0.pcd", "0_-20.pcd", "0_0.pcd", "20_-20.pcd",
      "20_0.pcd", "40_-20.pcd", "40_0.pcd"})
  {
    const std::size_t n = output.pointCloud("out/pointcloud_map/" + cell)->size();
    if (n > 0) {
      ++cells;
      cell_points += n;
      EXPECT_NE(metadata.find(cell), std::string::npos) << cell;
    }
  }
  const Cloud::Ptr map = output.pointCloud("out/map.pcd");
  EXPECT_GT(cells, 2U);
  EXPECT_GT(map->size(), 1000U);
  EXPECT_LT(map->size(), s.points);
  EXPECT_EQ(cell_points, map->size());
  EXPECT_EQ(output.bytes("out/map_projector_info.yaml"), "projector_type: Local\n");
}

TEST(MapExport, StreamedSaveOfAMapThatFitsInOnePassIsTheAssembledSave)
{
  const Submaps s = makeSubmaps();
  MapExportConfig config;
  config.map_leaf_size = 0.5;
  graphslam::ports::InMemoryMapOutput assembled;
  graphslam::map_export::saveGridDividedMap(
    graphslam::map_export::assembleMap(s.loader(), s.poses, s.points), "out", config,
    ProjectorOrigin{}, assembled, quietLog());
  graphslam::ports::InMemoryMapOutput streamed;
  config.map_save_points_per_pass = static_cast<int>(s.points);
  graphslam::map_export::saveStreamedMap(
    s.loader(), s.poses, s.points, "out", config, ProjectorOrigin{}, streamed, quietLog());
  EXPECT_TRUE(sameCloud(*assembled.pointCloud("out/map.pcd"), *streamed.pointCloud("out/map.pcd")));
  EXPECT_EQ(
    assembled.bytes("out/pointcloud_map/pointcloud_map_metadata.yaml"),
    streamed.bytes("out/pointcloud_map/pointcloud_map_metadata.yaml"));
}

TEST(MapExport, StreamedSaveInPassesKeepsTheVoxels)
{
  const Submaps s = makeSubmaps();
  MapExportConfig config;
  config.map_leaf_size = 0.5;
  graphslam::ports::InMemoryMapOutput assembled;
  graphslam::map_export::saveGridDividedMap(
    graphslam::map_export::assembleMap(s.loader(), s.poses, s.points), "out", config,
    ProjectorOrigin{}, assembled, quietLog());
  graphslam::ports::InMemoryMapOutput streamed;
  config.map_save_points_per_pass = 2000;  // several passes
  std::vector<std::string> lines;
  graphslam::map_export::saveStreamedMap(
    s.loader(), s.poses, s.points, "out", config, ProjectorOrigin{}, streamed, quietLog(&lines));
  // The same voxels (their means may differ in the last bits; map_point.hpp).
  EXPECT_EQ(
    assembled.pointCloud("out/map.pcd")->size(), streamed.pointCloud("out/map.pcd")->size());
  bool reported = false;
  for (const auto & line : lines) {
    reported = reported || line.find("without assembling the map") != std::string::npos;
  }
  EXPECT_TRUE(reported);
}

TEST(MapExport, ProjectorInfoAndManifestFollowTheConfig)
{
  const Submaps s = makeSubmaps();
  MapExportConfig config;
  config.use_planar_map_filter = true;
  config.planar_map_filter.min_neighbors = 4;
  graphslam::ports::InMemoryMapOutput output;
  std::vector<std::string> lines;
  ProjectorOrigin origin;
  origin.set = true;
  origin.latitude = 35.0;
  origin.longitude = 139.0;
  graphslam::map_export::saveGridDividedMap(
    graphslam::map_export::assembleMap(s.loader(), s.poses, s.points), "out", config, origin,
    output, quietLog(&lines));
  EXPECT_NE(
    output.bytes("out/map_projector_info.yaml").find("projector_type: LocalCartesian"),
    std::string::npos);
  EXPECT_NE(
    output.bytes("out/map_projector_info.yaml").find("latitude: 35.0000000000"),
    std::string::npos);
  bool planar = false;
  for (const auto & line : lines) {
    planar = planar || line.rfind("Planar map filter: input_points ", 0) == 0;
  }
  EXPECT_TRUE(planar);

  const graphslam::map_saver::BundleManifest manifest =
    graphslam::map_export::bundleManifest(config, "", 6, 2);
  EXPECT_EQ(manifest.frame_id, "map");
  EXPECT_EQ(manifest.submap_count, 6U);
  EXPECT_EQ(manifest.loop_edge_count, 2U);
  EXPECT_TRUE(manifest.planar_map_filter);
  EXPECT_EQ(manifest.planar_map_filter_min_neighbors, 4);
}
