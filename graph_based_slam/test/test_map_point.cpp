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
//    copyright notice, this list of conditions and the following disclaimer
//    in the documentation and/or other materials provided with the
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

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <random>
#include <string>
#include <vector>

#include <pcl/common/transforms.h>  // NOLINT(build/include_order)
#include <pcl/filters/voxel_grid.h>  // NOLINT(build/include_order)
#include <pcl/point_types.h>  // NOLINT(build/include_order)

#include "graph_based_slam/map_point.hpp"
#include "graph_based_slam/voxel_grid_overflow.hpp"

namespace
{

using graphslam::MapCloud;

// A cloud with `n` points uniform in [-extent, extent]^2 x [-extent / 10, extent / 10], every
// seventh one moved onto a 0.1 m voxel boundary in x.
pcl::PointCloud<pcl::PointXYZI>::Ptr uniformCloud(std::size_t n, float extent, unsigned seed)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> xy(-extent, extent);
  std::uniform_real_distribution<float> z(-0.1f * extent, 0.1f * extent);
  std::uniform_real_distribution<float> intensity(0.0f, 255.0f);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  for (std::size_t i = 0; i < n; ++i) {
    pcl::PointXYZI p;
    p.x = xy(rng);
    p.y = xy(rng);
    p.z = z(rng);
    if (i % 7 == 0) {
      p.x = std::round(p.x * 10.0f) / 10.0f;
    }
    p.intensity = intensity(rng);
    cloud->push_back(p);
  }
  return cloud;
}

bool sameFields(const pcl::PointXYZI & a, const graphslam::MapPoint & b)
{
  return std::memcmp(&a.x, &b.x, 3 * sizeof(float)) == 0 &&
         std::memcmp(&a.intensity, &b.intensity, sizeof(float)) == 0;
}

}  // namespace

TEST(MapPoint, IsSixteenBytesWithTheFieldsOfPointXYZI)
{
  EXPECT_EQ(sizeof(graphslam::MapPoint), 16U);
  std::vector<std::string> names;
  for (const auto & field : pcl::getFields<graphslam::MapPoint>()) {
    names.push_back(field.name);
  }
  std::vector<std::string> xyzi_names;
  for (const auto & field : pcl::getFields<pcl::PointXYZI>()) {
    if (field.name != "_") {
      xyzi_names.push_back(field.name);
    }
  }
  EXPECT_EQ(names, xyzi_names);
}

TEST(MapPoint, ConversionsKeepEveryPoint)
{
  const auto cloud = uniformCloud(1000, 10.0f, 3);
  cloud->is_dense = false;
  cloud->sensor_origin_ = Eigen::Vector4f(1.0f, 2.0f, 3.0f, 0.0f);
  const MapCloud::Ptr map = graphslam::toMapCloud(*cloud);
  ASSERT_EQ(map->size(), cloud->size());
  EXPECT_EQ(map->width, cloud->width);
  EXPECT_EQ(map->height, cloud->height);
  EXPECT_EQ(map->is_dense, cloud->is_dense);
  EXPECT_EQ(map->sensor_origin_, cloud->sensor_origin_);
  const auto back = graphslam::toPointXYZICloud(*map);
  ASSERT_EQ(back->size(), cloud->size());
  for (std::size_t i = 0; i < cloud->size(); ++i) {
    EXPECT_TRUE(sameFields((*cloud)[i], (*map)[i]));
    EXPECT_TRUE(sameFields((*back)[i], (*map)[i]));
  }
}

// The saved map relies on this: VoxelGrid gives the same points, bit for bit and in the same
// order, for a MapPoint cloud as for the PointXYZI cloud with the same points.
TEST(MapPoint, VoxelGridMatchesPointXYZIBitForBit)
{
  // From many points per voxel (summation order matters) to mostly one point per voxel.
  for (const float extent : {2.0f, 10.0f, 60.0f}) {
    const auto cloud = uniformCloud(300000, extent, 11);
    pcl::PointCloud<pcl::PointXYZI> expected;
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.1f, 0.1f, 0.1f);
    vg.filter(expected);

    MapCloud actual;
    graphslam::map_saver::voxelGridFilter(graphslam::toMapCloud(*cloud), 0.1f, actual);
    ASSERT_EQ(actual.size(), expected.size()) << "extent " << extent;
    EXPECT_EQ(actual.width, expected.width);
    EXPECT_EQ(actual.height, expected.height);
    EXPECT_EQ(actual.is_dense, expected.is_dense);
    std::size_t differing = 0;
    for (std::size_t i = 0; i < expected.size(); ++i) {
      differing += sameFields(expected[i], actual[i]) ? 0 : 1;
    }
    EXPECT_EQ(differing, 0U) << "extent " << extent;
  }
}

TEST(MapPoint, InPartsMatchesVoxelGridWhenTheMapFits)
{
  const MapCloud::Ptr cloud = graphslam::toMapCloud(*uniformCloud(100000, 10.0f, 5));
  ASSERT_FALSE(graphslam::map_saver::voxelGridWouldCopyInput(*cloud, 0.1f));
  MapCloud expected;
  graphslam::map_saver::voxelGridFilter(cloud, 0.1f, expected);
  MapCloud actual;
  graphslam::map_saver::voxelGridFilterInParts(*cloud, 0.1f, actual);
  ASSERT_EQ(actual.size(), expected.size());
  EXPECT_EQ(actual.width, expected.width);
  EXPECT_EQ(actual.height, expected.height);
  EXPECT_EQ(actual.is_dense, expected.is_dense);
  std::size_t differing = 0;
  for (std::size_t i = 0; i < expected.size(); ++i) {
    differing += std::memcmp(expected[i].data, actual[i].data, sizeof(expected[i].data)) == 0 ?
      0 : 1;
  }
  EXPECT_EQ(differing, 0U);
}

// A map VoxelGrid would return unchanged gets one point per occupied leaf-sized voxel, at the
// centroid of that voxel's points, as one VoxelGrid pass without the index limit would give.
TEST(MapPoint, InPartsDownsamplesAMapVoxelGridWouldCopy)
{
  const float leaf = 0.1f;
  std::mt19937 rng(17);
  std::uniform_real_distribution<float> unit(0.0f, 1.0f);
  MapCloud cloud;
  const auto add = [&cloud](float x, float y, float z, float intensity) {
      graphslam::MapPoint p;
      p.x = x;
      p.y = y;
      p.z = z;
      p.intensity = intensity;
      cloud.push_back(p);
    };
  // Two dense 2 m blocks 1000 m apart, and sparse points across the whole box, so the split
  // planes cut through occupied voxels.
  for (const std::array<float, 3> & origin : {std::array<float, 3>{-3.0f, -3.0f, -1.0f},
      std::array<float, 3>{997.0f, 997.0f, 59.0f}})
  {
    for (int i = 0; i < 40000; ++i) {
      add(origin[0] + 2.0f * unit(rng), origin[1] + 2.0f * unit(rng),
        origin[2] + 2.0f * unit(rng), 255.0f * unit(rng));
    }
  }
  for (int i = 0; i < 20000; ++i) {
    add(-3.0f + 1002.0f * unit(rng), -3.0f + 1002.0f * unit(rng), -1.0f + 62.0f * unit(rng),
      255.0f * unit(rng));
  }
  add(std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f, 1.0f);
  ASSERT_TRUE(graphslam::map_saver::voxelGridWouldCopyInput(cloud, leaf));

  const float inverse_leaf = 1.0f / leaf;
  std::map<std::array<int, 3>, std::array<double, 5>> voxels;
  for (const auto & p : cloud) {
    if (!std::isfinite(p.x)) {
      continue;
    }
    const std::array<int, 3> key{static_cast<int>(std::floor(p.x * inverse_leaf)),
      static_cast<int>(std::floor(p.y * inverse_leaf)),
      static_cast<int>(std::floor(p.z * inverse_leaf))};
    auto & sum = voxels[key];
    sum[0] += p.x;
    sum[1] += p.y;
    sum[2] += p.z;
    sum[3] += p.intensity;
    sum[4] += 1.0;
  }

  MapCloud actual;
  graphslam::map_saver::voxelGridFilterInParts(cloud, leaf, actual);
  ASSERT_EQ(actual.size(), voxels.size());
  EXPECT_EQ(actual.width, actual.size());
  EXPECT_EQ(actual.height, 1U);
  EXPECT_TRUE(actual.is_dense);
  std::map<std::array<int, 3>, int> seen;
  for (const auto & p : actual) {
    const std::array<int, 3> key{static_cast<int>(std::floor(p.x * inverse_leaf)),
      static_cast<int>(std::floor(p.y * inverse_leaf)),
      static_cast<int>(std::floor(p.z * inverse_leaf))};
    const auto voxel = voxels.find(key);
    ASSERT_NE(voxel, voxels.end()) << p.x << " " << p.y << " " << p.z;
    const auto & sum = voxel->second;
    EXPECT_NEAR(p.x, sum[0] / sum[4], 1e-3);
    EXPECT_NEAR(p.y, sum[1] / sum[4], 1e-3);
    EXPECT_NEAR(p.z, sum[2] / sum[4], 1e-3);
    EXPECT_NEAR(p.intensity, sum[3] / sum[4], 1e-2);
    EXPECT_EQ(++seen[key], 1);
  }
}

namespace
{

// Parts of a map: clouds moved by rigid transforms, as submap clouds by their poses.
std::vector<MapCloud> movedParts(std::size_t parts, std::size_t points, unsigned seed)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> angle(-0.3f, 0.3f), shift(-5.0f, 5.0f);
  std::vector<MapCloud> out;
  for (std::size_t i = 0; i < parts; ++i) {
    Eigen::Affine3f t = Eigen::Affine3f::Identity();
    t.rotate(Eigen::AngleAxisf(angle(rng) * 10.0f, Eigen::Vector3f::UnitZ()));
    t.rotate(Eigen::AngleAxisf(angle(rng), Eigen::Vector3f::UnitX()));
    t.translation() = Eigen::Vector3f(shift(rng), shift(rng), 0.2f * shift(rng));
    pcl::PointCloud<pcl::PointXYZI> moved;
    pcl::transformPointCloud(*uniformCloud(points, 10.0f, seed + 1 + i), moved, t);
    out.push_back(*graphslam::toMapCloud(moved));
  }
  return out;
}

MapCloud streamed(const std::vector<MapCloud> & parts, float leaf, std::size_t points_per_pass)
{
  MapCloud out;
  graphslam::map_saver::voxelGridFilterStreamed(
    parts.size(), [&parts](std::size_t i, MapCloud & cloud) {cloud = parts[i];}, leaf,
    points_per_pass, out);
  return out;
}

}  // namespace

// The voxels of one VoxelGrid pass over the assembled map, in the same order; the means agree
// to float rounding. How the passes are cut does not change the result.
TEST(MapPoint, StreamedMatchesVoxelGridOverTheAssembledMap)
{
  const auto parts = movedParts(20, 5000, 31);
  MapCloud::Ptr assembled(new MapCloud);
  for (const auto & part : parts) {
    *assembled += part;
  }
  MapCloud expected;
  graphslam::map_saver::voxelGridFilter(assembled, 0.1f, expected);
  const MapCloud actual = streamed(parts, 0.1f, 7000);
  ASSERT_EQ(actual.size(), expected.size());
  EXPECT_EQ(actual.width, expected.width);
  EXPECT_EQ(actual.height, 1U);
  EXPECT_TRUE(actual.is_dense);
  float worst_xyz = 0.0f, worst_intensity = 0.0f;
  for (std::size_t i = 0; i < expected.size(); ++i) {
    for (int k = 0; k < 3; ++k) {
      worst_xyz = std::max(worst_xyz, std::abs(actual[i].data[k] - expected[i].data[k]));
    }
    worst_intensity = std::max(worst_intensity,
      std::abs(actual[i].intensity - expected[i].intensity));
  }
  EXPECT_LT(worst_xyz, 1e-4f);
  EXPECT_LT(worst_intensity, 1e-3f);
  for (const std::size_t per_pass : {std::size_t{1}, std::size_t{1000000}}) {
    const MapCloud other = streamed(parts, 0.1f, per_pass);
    ASSERT_EQ(other.size(), actual.size());
    EXPECT_EQ(std::memcmp(other.points.data(), actual.points.data(), actual.size() * 16), 0);
  }
}

// A map too large for VoxelGrid's 32-bit index: the voxels of voxelGridFilterInParts (in another
// order).
TEST(MapPoint, StreamedDownsamplesAMapVoxelGridWouldCopy)
{
  const float leaf = 0.1f;
  std::vector<MapCloud> parts = movedParts(4, 20000, 41);
  MapCloud far = parts.back();
  for (auto & p : far.points) {
    p.x += 1000.0f;
    p.y += 1000.0f;
    p.z += 60.0f;
  }
  parts.push_back(far);
  graphslam::MapPoint nan;
  nan.x = std::numeric_limits<float>::quiet_NaN();
  parts.front().push_back(nan);
  MapCloud assembled;
  for (const auto & part : parts) {
    assembled += part;
  }
  ASSERT_TRUE(graphslam::map_saver::voxelGridWouldCopyInput(assembled, leaf));
  MapCloud expected;
  graphslam::map_saver::voxelGridFilterInParts(assembled, leaf, expected);
  const MapCloud actual = streamed(parts, leaf, 15000);
  ASSERT_EQ(actual.size(), expected.size());
  const float inverse_leaf = 1.0f / leaf;
  const auto by_voxel = [inverse_leaf](const MapCloud & cloud) {
      std::map<std::array<int, 3>, graphslam::MapPoint> out;
      for (const auto & p : cloud) {
        out[{static_cast<int>(std::floor(p.z * inverse_leaf)),
            static_cast<int>(std::floor(p.y * inverse_leaf)),
            static_cast<int>(std::floor(p.x * inverse_leaf))}] = p;
      }
      return out;
    };
  const auto a = by_voxel(actual), e = by_voxel(expected);
  ASSERT_EQ(a.size(), e.size());
  float worst = 0.0f;
  for (auto ia = a.begin(), ie = e.begin(); ia != a.end(); ++ia, ++ie) {
    ASSERT_EQ(ia->first, ie->first);
    for (int k = 0; k < 3; ++k) {
      worst = std::max(worst, std::abs(ia->second.data[k] - ie->second.data[k]));
    }
  }
  EXPECT_LT(worst, 1e-3f);
}
