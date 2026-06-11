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

// Characterization + determinism-contract tests for the pose-graph
// optimization extracted from doPoseAdjustment() (v0.6 Phase 1).

#include <gtest/gtest.h>

#include <cstring>
#include <vector>

#include <Eigen/Core>  // NOLINT(build/include_order)
#include <Eigen/Geometry>  // NOLINT(build/include_order)

#include "graph_based_slam/pose_graph_optimization.hpp"

namespace graphslam
{
namespace
{

using pose_graph::AdjacentEdgeConfig;
using pose_graph::Chi2Collection;
using pose_graph::GnssConstraint;
using pose_graph::ImuEdgeConfig;
using pose_graph::ImuRotationConstraint;
using pose_graph::LoopConstraint;
using pose_graph::LoopEdgeConfig;
using pose_graph::SubmapNode;
using pose_graph::optimizePoseGraph;

// A straight 11-node chain along +x with 1 m spacing whose later nodes carry
// an injected +y drift, plus the ground-truth loop measurement that says node
// 10 should coincide with node 0.
std::vector<SubmapNode> makeDriftedChain()
{
  std::vector<SubmapNode> submaps;
  for (int i = 0; i <= 10; ++i) {
    SubmapNode node;
    const double x = (i <= 5) ? static_cast<double>(i) : static_cast<double>(10 - i);
    const double drift = 0.06 * std::max(0, i - 5);
    node.pose = Eigen::Isometry3d::Identity();
    node.pose.translation() = Eigen::Vector3d(x, drift, 0.0);
    submaps.push_back(node);
  }
  return submaps;
}

LoopConstraint makeClosingLoop()
{
  // Ground truth: node 10 sits exactly on node 0. A good registration
  // fitness (0.1) gives the loop edge information weight 100 / 0.1 = 1000,
  // on par with the adjacent odometry edges.
  LoopConstraint loop;
  loop.from = 0;
  loop.to = 10;
  loop.relative_pose = Eigen::Isometry3d::Identity();
  loop.fitness_score = 0.1;
  return loop;
}

TEST(PoseGraphOptimization, LoopClosurePullsDriftedEndpointHome)
{
  const auto submaps = makeDriftedChain();
  const double drift_before =
    (submaps.back().pose.translation() - submaps.front().pose.translation()).norm();
  ASSERT_GT(drift_before, 0.25);

  const auto result = optimizePoseGraph(
    submaps, {makeClosingLoop()}, {}, {},
    AdjacentEdgeConfig{}, LoopEdgeConfig{}, ImuEdgeConfig{}, Chi2Collection::NONE);

  ASSERT_EQ(result.poses.size(), submaps.size());
  const double drift_after =
    (result.poses.back().translation() - result.poses.front().translation()).norm();
  EXPECT_LT(drift_after, drift_before * 0.5)
    << "loop closure should pull the drifted endpoint toward node 0";
  // Vertex 0 is fixed.
  EXPECT_LT((result.poses.front().translation() -
    submaps.front().pose.translation()).norm(), 1e-12);
}

TEST(PoseGraphOptimization, NoConstraintsBeyondOdometryKeepsChainShape)
{
  const auto submaps = makeDriftedChain();
  const auto result = optimizePoseGraph(
    submaps, {}, {}, {},
    AdjacentEdgeConfig{}, LoopEdgeConfig{}, ImuEdgeConfig{}, Chi2Collection::NONE);

  ASSERT_EQ(result.poses.size(), submaps.size());
  // Adjacent constraints reproduce the input relative poses exactly, so the
  // optimum is the input chain itself.
  for (size_t i = 0; i < submaps.size(); ++i) {
    EXPECT_LT((result.poses[i].translation() - submaps[i].pose.translation()).norm(), 1e-6)
      << "node " << i << " moved without any loop/IMU/GNSS constraint";
  }
}

TEST(PoseGraphOptimization, SameInputsGiveBitwiseIdenticalPoses)
{
  // The Phase 2 determinism contract at the optimizer layer: identical
  // inputs must give identical outputs, bit for bit.
  const auto submaps = makeDriftedChain();
  const auto a = optimizePoseGraph(
    submaps, {makeClosingLoop()}, {}, {},
    AdjacentEdgeConfig{}, LoopEdgeConfig{}, ImuEdgeConfig{}, Chi2Collection::UNIFIED);
  const auto b = optimizePoseGraph(
    submaps, {makeClosingLoop()}, {}, {},
    AdjacentEdgeConfig{}, LoopEdgeConfig{}, ImuEdgeConfig{}, Chi2Collection::UNIFIED);

  ASSERT_EQ(a.poses.size(), b.poses.size());
  for (size_t i = 0; i < a.poses.size(); ++i) {
    EXPECT_EQ(
      0,
      std::memcmp(a.poses[i].matrix().data(), b.poses[i].matrix().data(), sizeof(double) * 16))
      << "pose " << i << " differs between identical runs";
  }
  ASSERT_EQ(a.adjacent_chi2.size(), b.adjacent_chi2.size());
  for (size_t i = 0; i < a.adjacent_chi2.size(); ++i) {
    EXPECT_EQ(a.adjacent_chi2[i], b.adjacent_chi2[i]);
  }
}

TEST(PoseGraphOptimization, GnssAnchorAsBuiltDoesNotPullTranslation)
{
  // CHARACTERIZATION OF A KNOWN MISPLACEMENT (preserved by the extraction):
  // the GNSS edge writes its weights into indices (3,3)..(5,5), which in
  // g2o's EdgeSE3 error order (x, y, z, qx, qy, qz) is the ROTATION block.
  // As built, the anchor exerts no translation pull at all — consistent
  // with the GNSS constraint having always been documented as untested.
  // The behavioural fix (moving the weights to the translation block) is a
  // separate follow-up PR, which must flip this test to EXPECT_LT.
  const auto submaps = makeDriftedChain();
  GnssConstraint gnss;
  gnss.submap_index = 10;
  gnss.position = submaps.front().pose.translation();  // anchor at ground truth
  gnss.info_diag = Eigen::Vector3d(1000.0, 1000.0, 1000.0);

  const auto without = optimizePoseGraph(
    submaps, {}, {}, {},
    AdjacentEdgeConfig{}, LoopEdgeConfig{}, ImuEdgeConfig{}, Chi2Collection::NONE);
  const auto with = optimizePoseGraph(
    submaps, {}, {}, {gnss},
    AdjacentEdgeConfig{}, LoopEdgeConfig{}, ImuEdgeConfig{}, Chi2Collection::NONE);

  const double err_without =
    (without.poses[10].translation() - gnss.position).norm();
  const double err_with = (with.poses[10].translation() - gnss.position).norm();
  EXPECT_NEAR(err_with, err_without, 1e-9)
    << "as built, the GNSS anchor must not change translation; if this fails "
    << "because err_with shrank, the block-order fix landed — update this test";
}

TEST(PoseGraphOptimization, Chi2CollectionModesPopulateExpectedVectors)
{
  const auto submaps = makeDriftedChain();
  const auto none = optimizePoseGraph(
    submaps, {makeClosingLoop()}, {}, {},
    AdjacentEdgeConfig{}, LoopEdgeConfig{}, ImuEdgeConfig{}, Chi2Collection::NONE);
  EXPECT_TRUE(none.adjacent_chi2.empty());
  EXPECT_TRUE(none.adjacent_trans_chi2.empty());

  const auto unified = optimizePoseGraph(
    submaps, {makeClosingLoop()}, {}, {},
    AdjacentEdgeConfig{}, LoopEdgeConfig{}, ImuEdgeConfig{}, Chi2Collection::UNIFIED);
  EXPECT_FALSE(unified.adjacent_chi2.empty());
  EXPECT_TRUE(unified.adjacent_trans_chi2.empty());

  AdjacentEdgeConfig split_cfg;
  split_cfg.split_trans_rot = true;
  const auto split = optimizePoseGraph(
    submaps, {makeClosingLoop()}, {}, {},
    split_cfg, LoopEdgeConfig{}, ImuEdgeConfig{}, Chi2Collection::SPLIT);
  EXPECT_TRUE(split.adjacent_chi2.empty());
  EXPECT_FALSE(split.adjacent_trans_chi2.empty());
  EXPECT_EQ(split.adjacent_trans_chi2.size(), split.adjacent_rot_chi2.size());
}

TEST(PoseGraphOptimization, ImuRotationConstraintAsBuiltDoesNotRotate)
{
  // CHARACTERIZATION OF A KNOWN MISPLACEMENT (preserved by the extraction):
  // the IMU edge writes roll/pitch/yaw weights into indices (0,0)..(2,2),
  // which in g2o's EdgeSE3 error order (x, y, z, qx, qy, qz) is the
  // TRANSLATION block; the rotation block stays zero, so as built the "IMU
  // rotation constraint" never constrained rotation. The behavioural fix is
  // a separate follow-up PR, which must flip this test to EXPECT_GT.
  std::vector<SubmapNode> submaps(2);
  submaps[0].pose = Eigen::Isometry3d::Identity();
  submaps[1].pose = Eigen::Isometry3d::Identity();
  submaps[1].pose.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);

  ImuRotationConstraint imu;
  imu.from = 0;
  imu.to = 1;
  imu.measurement = Eigen::Isometry3d::Identity();
  imu.measurement.linear() =
    Eigen::AngleAxisd(10.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  imu.measurement.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);

  ImuEdgeConfig imu_cfg;
  imu_cfg.info_roll_pitch = 1e6;
  imu_cfg.info_yaw = 1e6;

  AdjacentEdgeConfig weak_odom;
  weak_odom.info_weight = 1.0;

  const auto result = optimizePoseGraph(
    submaps, {}, {imu}, {}, weak_odom, LoopEdgeConfig{}, imu_cfg, Chi2Collection::NONE);

  const Eigen::AngleAxisd delta(
    result.poses[0].linear().transpose() * result.poses[1].linear());
  EXPECT_LT(delta.angle() * 180.0 / M_PI, 1e-6)
    << "as built, the IMU edge must not rotate anything; if this fails "
    << "because rotation appeared, the block-order fix landed — update this test";
}

}  // namespace
}  // namespace graphslam
