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

#include <cmath>
#include <random>
#include <vector>

#include "graph_based_slam/multisession_join.hpp"

namespace graphslam
{
namespace
{

using multisession::Keyframe;

// A synthetic site: a ground strip and box-shaped buildings (their vertical faces), as points.
std::vector<Eigen::Vector3d> boxes(
  std::mt19937 & rng, int count, double x0, double x1, double y_min_abs,
  double y_max_abs)
{
  std::uniform_real_distribution<double> ux(x0, x1), uy(y_min_abs, y_max_abs), us(1.5, 6.0),
  uh(1.0, 5.0),
  coin(0.0, 1.0);
  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < count; ++i) {
    const double cx = ux(rng), cy = (coin(rng) < 0.5 ? -1.0 : 1.0) * uy(rng);
    const double sx = us(rng), sy = us(rng), h = uh(rng);
    for (double z = -0.6; z <= h; z += 0.5) {
      for (double t = -0.5; t <= 0.5; t += 0.5 / std::max(sx, sy)) {
        points.emplace_back(cx + t * sx, cy - 0.5 * sy, z);
        points.emplace_back(cx + t * sx, cy + 0.5 * sy, z);
        points.emplace_back(cx - 0.5 * sx, cy + t * sy, z);
        points.emplace_back(cx + 0.5 * sx, cy + t * sy, z);
      }
    }
  }
  return points;
}

void ground(std::vector<Eigen::Vector3d> * points, double x0, double x1, double half_width)
{
  for (double x = x0; x <= x1; x += 1.0) {
    for (double y = -half_width; y <= half_width; y += 1.0) {
      points->emplace_back(x, y, -0.6);
    }
  }
}

// Keyframes every 4 m along a straight walk in the world; each keyframe's frame is given in the
// session's own odometry frame (world_to_session times the world pose) and its map is the site
// within 30 m. (Sparse, so that the tests run in seconds.)
std::vector<Keyframe> walk(
  const std::vector<Eigen::Vector3d> & site, double x0, double x1, double y, double yaw,
  const Eigen::Isometry3d & world_to_session)
{
  std::vector<Keyframe> out;
  for (double x = x0; x <= x1 + 1e-9; x += 4.0) {
    Eigen::Isometry3d world = Eigen::Isometry3d::Identity();
    world.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    world.translation() = Eigen::Vector3d(x, y, 0.0);
    Keyframe k;
    k.id = static_cast<int>(out.size());
    k.frame = world_to_session * world;
    for (const auto & p : site) {
      const Eigen::Vector3d q = world.inverse() * p;
      if (q.head<2>().norm() < 30.0) {
        k.points.push_back(q.cast<float>());
      }
    }
    out.push_back(k);
  }
  return out;
}

Eigen::Isometry3d planar(double x, double y, double z, double yaw)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  t.translation() = Eigen::Vector3d(x, y, z);
  return t;
}

TEST(MultisessionJoin, FindsTheTransformBetweenTwoWalksOfOneSite)
{
  std::mt19937 rng(7);
  std::vector<Eigen::Vector3d> site = boxes(rng, 60, -20.0, 80.0, 4.0, 25.0);
  ground(&site, -20.0, 80.0, 25.0);
  // B's odometry frame is the world moved and turned: the join must find the inverse.
  const Eigen::Isometry3d world_to_b = planar(15.0, -7.0, 0.4, 1.0);
  const auto a = walk(site, 0.0, 40.0, 0.0, 0.0, Eigen::Isometry3d::Identity());
  const auto b = walk(site, 10.0, 50.0, 1.0, 0.3, world_to_b);
  multisession::JoinConfig config;
  config.num_threads = 2;
  const auto result = multisession::joinSessions(a, b, config);
  ASSERT_TRUE(result.joined);
  const Eigen::Isometry3d truth = world_to_b.inverse();
  for (const auto & k : b) {
    const Eigen::Vector3d p = k.frame.translation();
    EXPECT_LT((result.transform * p - truth * p).norm(), 0.03);
  }
  EXPECT_LT(multisession::rotationDeg(result.transform.rotation().transpose() * truth.rotation()),
        0.2);
  ASSERT_FALSE(result.attempts.empty());
  EXPECT_TRUE(result.attempts.back().passed);
}

TEST(MultisessionJoin, DoesNotJoinTwoDifferentSites)
{
  std::mt19937 rng_a(11), rng_b(12);
  std::vector<Eigen::Vector3d> site_a = boxes(rng_a, 60, -20.0, 80.0, 4.0, 25.0);
  std::vector<Eigen::Vector3d> site_b = boxes(rng_b, 60, -20.0, 80.0, 4.0, 25.0);
  ground(&site_a, -20.0, 80.0, 25.0);
  ground(&site_b, -20.0, 80.0, 25.0);
  const auto a = walk(site_a, 0.0, 40.0, 0.0, 0.0, Eigen::Isometry3d::Identity());
  const auto b = walk(site_b, 0.0, 40.0, 0.0, 0.0, Eigen::Isometry3d::Identity());
  multisession::JoinConfig config;
  config.num_threads = 2;
  EXPECT_FALSE(multisession::joinSessions(a, b, config).joined);
}

TEST(MultisessionJoin, WholeMapCheckRejectsAShiftAlongRepeatedStructure)
{
  // A corridor of identical pillars every 8 m between unique buildings: a transform shifted by one
  // pillar spacing lines the pillars up but not the rest of the site.
  std::mt19937 rng(5);
  std::vector<Eigen::Vector3d> site = boxes(rng, 80, -20.0, 100.0, 5.0, 25.0);
  for (double x = 0.0; x <= 80.0; x += 8.0) {
    for (double side : {-3.0, 3.0}) {
      for (double z = -0.6; z <= 3.0; z += 0.2) {
        for (double a = 0.0; a < 2.0 * M_PI; a += 0.4) {
          site.emplace_back(x + 0.3 * std::cos(a), side + 0.3 * std::sin(a), z);
        }
      }
    }
  }
  ground(&site, -20.0, 100.0, 4.0);
  const Eigen::Isometry3d world_to_b = planar(-5.0, 3.0, 0.0, 0.5);
  const auto a = walk(site, 0.0, 60.0, 0.0, 0.0, Eigen::Isometry3d::Identity());
  const auto b = walk(site, 10.0, 70.0, 0.0, 0.0, world_to_b);
  multisession::JoinConfig config;
  config.num_threads = 2;
  const Eigen::Isometry3d truth = world_to_b.inverse();
  const auto right = multisession::wholeMapCheck(a, b, truth, config);
  EXPECT_TRUE(right.passed);
  EXPECT_LE(right.share_below, 0.1);
  const auto shifted = multisession::wholeMapCheck(a, b, planar(8.0, 0.0, 0.0, 0.0) * truth,
        config);
  EXPECT_FALSE(shifted.passed);
  EXPECT_GT(shifted.share_below, 0.1);
}

TEST(MultisessionJoin, RefusesASiteThatLooksTheSameTurnedHalfway)
{
  // Every building has a twin turned by 180 deg about the origin: the site fits itself turned
  // halfway round, so a join to it cannot be told from a wrong one and must be refused.
  std::mt19937 rng(9);
  std::vector<Eigen::Vector3d> site = boxes(rng, 40, -40.0, 40.0, 4.0, 25.0);
  const std::size_t n = site.size();
  for (std::size_t i = 0; i < n; ++i) {
    site.emplace_back(-site[i].x(), -site[i].y(), site[i].z());
  }
  ground(&site, -40.0, 40.0, 25.0);
  const Eigen::Isometry3d world_to_b = planar(6.0, -4.0, 0.0, 0.7);
  const auto a = walk(site, -20.0, 20.0, 0.0, 0.0, Eigen::Isometry3d::Identity());
  const auto b = walk(site, -16.0, 24.0, 1.0, 0.2, world_to_b);
  multisession::JoinConfig config;
  config.num_threads = 2;
  const auto result = multisession::joinSessions(a, b, config);
  EXPECT_FALSE(result.joined);
  EXPECT_TRUE(result.ambiguous || !result.self_similar.empty());
}

TEST(MultisessionJoin, MaxCliqueStopsOnceLargeEnough)
{
  // a 4-clique {0, 1, 2, 3} and a separate edge {4, 5}
  std::vector<std::set<int>> adj(6);
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      if (i != j) {adj[i].insert(j);}
    }
  }
  adj[4].insert(5);
  adj[5].insert(4);
  EXPECT_EQ(multisession::maxClique(adj).size(), 4U);
  EXPECT_EQ(multisession::maxClique(adj, 3).size(), 3U);
}

}  // namespace
}  // namespace graphslam
