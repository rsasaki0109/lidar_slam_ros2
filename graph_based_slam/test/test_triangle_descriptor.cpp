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
#include <vector>

#include "graph_based_slam/triangle_descriptor.hpp"

namespace graphslam
{
namespace triangle
{
namespace
{

pcl::PointCloud<pcl::PointXYZI>::Ptr makeCloud()
{
  return pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
}

void addPillar(
  pcl::PointCloud<pcl::PointXYZI> & cloud,
  float x, float y, float top_z, float bottom_z = 0.0f, float step = 0.2f)
{
  // Simulate a vertical structure by stacking points; the BEV cell sees them
  // all and picks the tallest one (top_z) for max_height.
  for (float z = bottom_z; z <= top_z; z += step) {
    pcl::PointXYZI p;
    p.x = x;
    p.y = y;
    p.z = z;
    p.intensity = 1.0f;
    cloud.push_back(p);
  }
}

void addGroundPatch(
  pcl::PointCloud<pcl::PointXYZI> & cloud,
  float x_min, float x_max, float y_min, float y_max,
  float step = 0.5f)
{
  // Sprinkle ground-level points so neighborhood floor is well-defined.
  for (float x = x_min; x <= x_max; x += step) {
    for (float y = y_min; y <= y_max; y += step) {
      pcl::PointXYZI p;
      p.x = x;
      p.y = y;
      p.z = 0.0f;
      p.intensity = 0.5f;
      cloud.push_back(p);
    }
  }
}

// ----- keypoint extraction -----

TEST(TriangleDescriptorKeypoint, EmptyCloudYieldsNoKeypoints)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  const auto kps = extractKeypointsBEV(cloud, {});
  EXPECT_TRUE(kps.empty());
}

TEST(TriangleDescriptorKeypoint, PicksTallPillarsAboveGround)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  addGroundPatch(cloud, -25.0f, 25.0f, -25.0f, 25.0f);
  addPillar(cloud, 5.0f, 5.0f, 4.0f);
  addPillar(cloud, -5.0f, 5.0f, 3.5f);
  addPillar(cloud, 0.0f, -10.0f, 5.0f);

  KeypointExtractionConfig cfg;
  cfg.grid_size_m = 60.0;
  cfg.grid_cells = 60;
  cfg.neighborhood_radius_cells = 2;
  cfg.min_salience_m = 1.0f;
  cfg.max_keypoints = 10;
  const auto kps = extractKeypointsBEV(cloud, cfg);
  EXPECT_EQ(3u, kps.size());
  for (const auto & kp : kps) {
    EXPECT_GT(kp.salience, 1.0f);
  }
}

TEST(TriangleDescriptorKeypoint, RespectsMaxKeypoints)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  addGroundPatch(cloud, -25.0f, 25.0f, -25.0f, 25.0f);
  // Six pillars of decreasing height; only the top 3 should survive.
  for (int i = 0; i < 6; ++i) {
    addPillar(cloud, static_cast<float>(-12 + 5 * i), 0.0f, 5.0f - 0.5f * i);
  }
  KeypointExtractionConfig cfg;
  cfg.min_salience_m = 0.5f;
  cfg.max_keypoints = 3;
  const auto kps = extractKeypointsBEV(cloud, cfg);
  EXPECT_EQ(3u, kps.size());
  // Highest salience first.
  EXPECT_GE(kps[0].salience, kps[1].salience);
  EXPECT_GE(kps[1].salience, kps[2].salience);
}

TEST(TriangleDescriptorKeypoint, FiltersLowSaliencePillars)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  addGroundPatch(cloud, -10.0f, 10.0f, -10.0f, 10.0f);
  addPillar(cloud, 0.0f, 0.0f, 0.1f);  // basically ground-height, should be filtered
  KeypointExtractionConfig cfg;
  cfg.min_salience_m = 0.5f;
  const auto kps = extractKeypointsBEV(cloud, cfg);
  EXPECT_TRUE(kps.empty());
}

// ----- triangle enumeration -----

std::vector<Keypoint> threePoints(
  const Eigen::Vector3f & a, const Eigen::Vector3f & b, const Eigen::Vector3f & c)
{
  std::vector<Keypoint> kps(3);
  kps[0].position = a;
  kps[1].position = b;
  kps[2].position = c;
  return kps;
}

TEST(TriangleDescriptorBuild, ReturnsEmptyForFewerThanThreeKeypoints)
{
  std::vector<Keypoint> kps(2);
  const auto tris = buildTriangles(kps, {});
  EXPECT_TRUE(tris.empty());
}

TEST(TriangleDescriptorBuild, EdgesAreSortedAscending)
{
  const auto kps = threePoints(
    Eigen::Vector3f(0, 0, 0),
    Eigen::Vector3f(10, 0, 0),
    Eigen::Vector3f(10, 4, 0));
  TriangleBuildConfig cfg;
  cfg.min_edge_m = 0.0f;
  cfg.max_edge_m = 100.0f;
  cfg.min_angle_deg = 0.0f;
  const auto tris = buildTriangles(kps, cfg);
  ASSERT_EQ(1u, tris.size());
  const auto & e = tris[0].edges;
  EXPECT_LE(e[0], e[1]);
  EXPECT_LE(e[1], e[2]);
  // The 3-4-5 right triangle has edges 4, sqrt(116) ~= 10.77, 10.
  // Wait — the right triangle 10x4 has edges 4, 10, sqrt(116) ~= 10.77.
  EXPECT_NEAR(4.0f, e[0], 1e-4);
  EXPECT_NEAR(10.0f, e[1], 1e-4);
  EXPECT_NEAR(std::sqrt(116.0f), e[2], 1e-4);
}

TEST(TriangleDescriptorBuild, FiltersByEdgeLengthRange)
{
  // Long-and-short configuration: edges 1, 100, sqrt(10001).
  const auto kps = threePoints(
    Eigen::Vector3f(0, 0, 0),
    Eigen::Vector3f(1, 0, 0),
    Eigen::Vector3f(1, 100, 0));
  TriangleBuildConfig cfg;
  cfg.min_edge_m = 2.0f;
  cfg.max_edge_m = 50.0f;
  cfg.min_angle_deg = 0.0f;
  EXPECT_TRUE(buildTriangles(kps, cfg).empty());
}

TEST(TriangleDescriptorBuild, FiltersNearCollinearTriangles)
{
  // Three nearly-collinear points; smallest angle approaches 0 deg.
  const auto kps = threePoints(
    Eigen::Vector3f(0, 0, 0),
    Eigen::Vector3f(10, 0, 0),
    Eigen::Vector3f(20, 0.05f, 0));
  TriangleBuildConfig cfg;
  cfg.min_edge_m = 0.0f;
  cfg.max_edge_m = 100.0f;
  cfg.min_angle_deg = 5.0f;
  EXPECT_TRUE(buildTriangles(kps, cfg).empty());
}

TEST(TriangleDescriptorBuild, EnumeratesAllChooseThreeTriplets)
{
  std::vector<Keypoint> kps;
  // 6 keypoints arranged in a hexagon should give C(6, 3) = 20 triangles when
  // the angle filter is permissive.
  for (int i = 0; i < 6; ++i) {
    const float angle = 2.0f * static_cast<float>(M_PI) * i / 6.0f;
    Keypoint kp;
    kp.position = Eigen::Vector3f(10.0f * std::cos(angle), 10.0f * std::sin(angle), 0.0f);
    kps.push_back(kp);
  }
  TriangleBuildConfig cfg;
  cfg.min_edge_m = 0.0f;
  cfg.max_edge_m = 100.0f;
  cfg.min_angle_deg = 0.0f;
  const auto tris = buildTriangles(kps, cfg);
  EXPECT_EQ(20u, tris.size());
}

TEST(TriangleDescriptorBuild, RespectsMaxTrianglesCap)
{
  std::vector<Keypoint> kps;
  for (int i = 0; i < 6; ++i) {
    const float angle = 2.0f * static_cast<float>(M_PI) * i / 6.0f;
    Keypoint kp;
    kp.position = Eigen::Vector3f(10.0f * std::cos(angle), 10.0f * std::sin(angle), 0.0f);
    kps.push_back(kp);
  }
  TriangleBuildConfig cfg;
  cfg.min_edge_m = 0.0f;
  cfg.max_edge_m = 100.0f;
  cfg.min_angle_deg = 0.0f;
  cfg.max_triangles = 5;
  const auto tris = buildTriangles(kps, cfg);
  EXPECT_EQ(5u, tris.size());
  // First triangle should have the largest longest-edge.
  for (std::size_t i = 1; i < tris.size(); ++i) {
    EXPECT_GE(tris[0].edges[2], tris[i].edges[2]);
  }
}

// ----- rigid SE(3) recovery -----

TEST(TriangleDescriptorRigid, IdenticalTriangleYieldsIdentity)
{
  const std::array<Eigen::Vector3f, 3> pts = {{
    Eigen::Vector3f(1, 2, 3),
    Eigen::Vector3f(5, -1, 2),
    Eigen::Vector3f(0, 4, -1),
  }};
  const Eigen::Matrix4f T = estimateRigidFromTriangle(pts, pts);
  EXPECT_TRUE(T.isApprox(Eigen::Matrix4f::Identity(), 1e-4f));
}

TEST(TriangleDescriptorRigid, RecoversPureTranslation)
{
  const std::array<Eigen::Vector3f, 3> src = {{
    Eigen::Vector3f(0, 0, 0),
    Eigen::Vector3f(10, 0, 0),
    Eigen::Vector3f(0, 5, 0),
  }};
  const Eigen::Vector3f shift(3.0f, -2.0f, 1.0f);
  std::array<Eigen::Vector3f, 3> dst = src;
  for (auto & p : dst) {
    p += shift;
  }
  const Eigen::Matrix4f T = estimateRigidFromTriangle(src, dst);
  const Eigen::Matrix3f R_recovered = T.block<3, 3>(0, 0);
  const Eigen::Vector3f t_recovered = T.block<3, 1>(0, 3);
  EXPECT_TRUE(R_recovered.isApprox(Eigen::Matrix3f::Identity(), 1e-4f));
  EXPECT_TRUE(t_recovered.isApprox(shift, 1e-4f));
}

TEST(TriangleDescriptorRigid, RecoversYawRotationAboutOrigin)
{
  const float yaw = static_cast<float>(M_PI / 3.0);  // 60°
  const Eigen::Matrix3f R = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  const std::array<Eigen::Vector3f, 3> src = {{
    Eigen::Vector3f(2, 0, 0),
    Eigen::Vector3f(6, 0, 0),
    Eigen::Vector3f(3, 4, 0),
  }};
  std::array<Eigen::Vector3f, 3> dst = src;
  for (auto & p : dst) {
    p = R * p;
  }
  const Eigen::Matrix4f T = estimateRigidFromTriangle(src, dst);
  const Eigen::Matrix3f R_recovered = T.block<3, 3>(0, 0);
  const Eigen::Vector3f t_recovered = T.block<3, 1>(0, 3);
  EXPECT_TRUE(R_recovered.isApprox(R, 1e-4f));
  // isApprox compares relative to the smaller norm; for the zero vector
  // that degenerates, so fall back to an absolute tolerance.
  EXPECT_LT(t_recovered.norm(), 1e-4f);
}

TEST(TriangleDescriptorRigid, RecoversRotationPlusTranslation)
{
  const float yaw = static_cast<float>(M_PI / 4.0);
  const Eigen::Matrix3f R = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  const Eigen::Vector3f shift(7.0f, -3.0f, 0.5f);
  const std::array<Eigen::Vector3f, 3> src = {{
    Eigen::Vector3f(0, 0, 0),
    Eigen::Vector3f(5, 0, 0),
    Eigen::Vector3f(0, 8, 0),
  }};
  std::array<Eigen::Vector3f, 3> dst = src;
  for (auto & p : dst) {
    p = R * p + shift;
  }
  const Eigen::Matrix4f T = estimateRigidFromTriangle(src, dst);
  const Eigen::Matrix3f R_recovered = T.block<3, 3>(0, 0);
  const Eigen::Vector3f t_recovered = T.block<3, 1>(0, 3);
  EXPECT_TRUE(R_recovered.isApprox(R, 1e-4f));
  EXPECT_TRUE(t_recovered.isApprox(shift, 1e-4f));
}

TEST(TriangleDescriptorRigid, ResultIsProperRotation)
{
  // Mirror-image triangle (reflection). Umeyama with det-correction must
  // still return a proper rotation (det = +1), even if the fit residual is
  // large.
  const std::array<Eigen::Vector3f, 3> src = {{
    Eigen::Vector3f(0, 0, 0),
    Eigen::Vector3f(1, 0, 0),
    Eigen::Vector3f(0, 1, 0),
  }};
  const std::array<Eigen::Vector3f, 3> dst = {{
    Eigen::Vector3f(0, 0, 0),
    Eigen::Vector3f(-1, 0, 0),
    Eigen::Vector3f(0, 1, 0),
  }};
  const Eigen::Matrix4f T = estimateRigidFromTriangle(src, dst);
  const Eigen::Matrix3f R_recovered = T.block<3, 3>(0, 0);
  EXPECT_NEAR(1.0f, R_recovered.determinant(), 1e-4f);
}

}  // namespace
}  // namespace triangle
}  // namespace graphslam
