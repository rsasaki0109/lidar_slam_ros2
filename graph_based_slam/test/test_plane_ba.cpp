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

#include <cstring>
#include <random>
#include <vector>

#include "graph_based_slam/plane_ba.hpp"

namespace
{

using graphslam::map_refinement::leftPerturb;
using graphslam::map_refinement::logSe3;
using graphslam::map_refinement::makeCluster;
using graphslam::map_refinement::PlaneBaConfig;
using graphslam::map_refinement::PlaneBaResult;
using graphslam::map_refinement::PlaneFeature;
using graphslam::map_refinement::PlaneFeatureObservation;
using graphslam::map_refinement::solvePlaneBa;
using graphslam::map_refinement::Vector6d;

Eigen::Matrix4d inversePose(const Eigen::Matrix4d & pose)
{
  Eigen::Matrix4d inverse = Eigen::Matrix4d::Identity();
  const Eigen::Matrix3d rotation = pose.block<3, 3>(0, 0);
  inverse.block<3, 3>(0, 0) = rotation.transpose();
  inverse.block<3, 1>(0, 3) = -rotation.transpose() * pose.block<3, 1>(0, 3);
  return inverse;
}

// World points of three orthogonal noisy planes (z=0, x=2, y=2).
std::vector<Eigen::Vector3d> makeOrthogonalPlaneWorld(double noise_sigma)
{
  std::mt19937 rng(4242);
  std::normal_distribution<double> noise(0.0, noise_sigma);
  std::vector<Eigen::Vector3d> points;
  for (int a = 0; a < 14; ++a) {
    for (int b = 0; b < 14; ++b) {
      const double u = 0.15 * a;
      const double v = 0.15 * b;
      points.push_back(Eigen::Vector3d(u, v, noise(rng)));
      points.push_back(Eigen::Vector3d(2.0 + noise(rng), u, v));
      points.push_back(Eigen::Vector3d(u, 2.0 + noise(rng), v));
    }
  }
  return points;
}

int planeIdOf(int flat_index)
{
  return flat_index % 3;
}

// Build per-plane features observed by every pose: world points are
// transformed into each pose's LOCAL frame with the ground-truth poses,
// so the observations are exactly consistent at the ground truth.
std::vector<PlaneFeature> makeFeatures(
  const std::vector<Eigen::Vector3d> & world_points,
  const std::vector<Eigen::Matrix4d> & ground_truth_poses)
{
  std::vector<PlaneFeature> features(3);
  for (size_t p = 0; p < ground_truth_poses.size(); ++p) {
    const Eigen::Matrix4d world_to_local = inversePose(ground_truth_poses[p]);
    std::vector<std::vector<Eigen::Vector3d>> local_points(3);
    for (size_t i = 0; i < world_points.size(); ++i) {
      const Eigen::Vector3d local =
        world_to_local.block<3, 3>(0, 0) * world_points[i] +
        world_to_local.block<3, 1>(0, 3);
      local_points[planeIdOf(static_cast<int>(i))].push_back(local);
    }
    for (int plane = 0; plane < 3; ++plane) {
      PlaneFeatureObservation observation;
      observation.pose_index = static_cast<int>(p);
      observation.local_cluster = makeCluster(local_points[plane]);
      features[plane].observations.push_back(observation);
    }
  }
  return features;
}

std::vector<Eigen::Matrix4d> makeGroundTruthPoses()
{
  std::vector<Eigen::Matrix4d> poses(3, Eigen::Matrix4d::Identity());
  Vector6d twist1;
  twist1 << 0.4, -0.2, 0.1, 0.02, -0.03, 0.05;
  Vector6d twist2;
  twist2 << -0.3, 0.5, -0.1, -0.04, 0.01, -0.02;
  poses[1] = graphslam::map_refinement::expSe3(twist1);
  poses[2] = graphslam::map_refinement::expSe3(twist2);
  return poses;
}

std::vector<Eigen::Matrix4d> perturbPoses(
  const std::vector<Eigen::Matrix4d> & poses, double translation, double rotation)
{
  std::vector<Eigen::Matrix4d> perturbed = poses;
  Vector6d twist1;
  twist1 << translation, -translation, 0.5 * translation, rotation, -rotation,
    0.5 * rotation;
  Vector6d twist2;
  twist2 << -0.5 * translation, translation, -translation, -rotation,
    0.5 * rotation, rotation;
  perturbed[1] = leftPerturb(twist1, perturbed[1]);
  perturbed[2] = leftPerturb(twist2, perturbed[2]);
  return perturbed;
}

double poseTranslationError(const Eigen::Matrix4d & a, const Eigen::Matrix4d & b)
{
  const Vector6d twist = logSe3(a * inversePose(b));
  return twist.head<3>().norm();
}

double poseRotationError(const Eigen::Matrix4d & a, const Eigen::Matrix4d & b)
{
  const Vector6d twist = logSe3(a * inversePose(b));
  return twist.tail<3>().norm();
}

TEST(PlaneBa, RecoversPerturbedPosesOnOrthogonalPlanes) {
  const auto world = makeOrthogonalPlaneWorld(0.005);
  const auto ground_truth = makeGroundTruthPoses();
  const auto features = makeFeatures(world, ground_truth);
  const auto initial = perturbPoses(ground_truth, 0.05, 0.017);

  // Controlled test of the plane cost itself: priors are disabled (they
  // pull toward the perturbed INPUT poses by design; the unobservable
  // fixture covers their role separately).
  PlaneBaConfig config;
  config.prior_translation_sigma = 0.0;
  const PlaneBaResult result = solvePlaneBa(features, initial, config);
  ASSERT_EQ(result.poses.size(), 3u);
  EXPECT_TRUE(result.improved);
  EXPECT_GE(result.accepted_steps, 1);
  for (int p = 1; p < 3; ++p) {
    EXPECT_LT(poseTranslationError(result.poses[p], ground_truth[p]), 0.01);
    EXPECT_LT(poseRotationError(result.poses[p], ground_truth[p]), 0.005);
  }
}

TEST(PlaneBa, AlreadyOptimalInputBarelyMoves) {
  const auto world = makeOrthogonalPlaneWorld(0.005);
  const auto ground_truth = makeGroundTruthPoses();
  const auto features = makeFeatures(world, ground_truth);

  const PlaneBaResult result = solvePlaneBa(features, ground_truth, PlaneBaConfig());
  EXPECT_LE(result.final_cost, result.initial_cost);
  for (int p = 1; p < 3; ++p) {
    EXPECT_LT(poseTranslationError(result.poses[p], ground_truth[p]), 1e-3);
  }
}

TEST(PlaneBa, UnobservableFloorOnlySceneIsHeldByPriors) {
  // A single floor plane: in-plane translation and yaw are unconstrained
  // by the plane cost. The correct behaviour is "do not move": priors
  // hold the unconstrained directions and nothing becomes NaN.
  std::mt19937 rng(99);
  std::normal_distribution<double> noise(0.0, 0.004);
  std::vector<Eigen::Vector3d> world;
  for (int a = 0; a < 20; ++a) {
    for (int b = 0; b < 20; ++b) {
      world.push_back(Eigen::Vector3d(0.2 * a, 0.2 * b, noise(rng)));
    }
  }
  std::vector<Eigen::Matrix4d> ground_truth(2, Eigen::Matrix4d::Identity());
  Vector6d twist;
  twist << 1.0, 0.5, 0.0, 0.0, 0.0, 0.3;
  ground_truth[1] = graphslam::map_refinement::expSe3(twist);

  std::vector<PlaneFeature> features(1);
  for (int p = 0; p < 2; ++p) {
    const Eigen::Matrix4d world_to_local = inversePose(ground_truth[p]);
    std::vector<Eigen::Vector3d> local;
    for (size_t i = 0; i < world.size(); ++i) {
      local.push_back(
        world_to_local.block<3, 3>(0, 0) * world[i] +
        world_to_local.block<3, 1>(0, 3));
    }
    PlaneFeatureObservation observation;
    observation.pose_index = p;
    observation.local_cluster = makeCluster(local);
    features[0].observations.push_back(observation);
  }

  const PlaneBaResult result = solvePlaneBa(features, ground_truth, PlaneBaConfig());
  ASSERT_EQ(result.poses.size(), 2u);
  EXPECT_TRUE(result.poses[1].allFinite());
  EXPECT_LT(poseTranslationError(result.poses[1], ground_truth[1]), 0.02);
}

TEST(PlaneBa, NoValidFeaturesReturnsInputUnchanged) {
  std::vector<Eigen::Vector3d> tiny;
  tiny.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
  tiny.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));

  std::vector<PlaneFeature> features(1);
  PlaneFeatureObservation observation;
  observation.pose_index = 0;
  observation.local_cluster = makeCluster(tiny);
  features[0].observations.push_back(observation);

  std::vector<Eigen::Matrix4d> poses(2, Eigen::Matrix4d::Identity());
  Vector6d twist;
  twist << 0.3, 0.1, -0.2, 0.05, 0.0, 0.1;
  poses[1] = graphslam::map_refinement::expSe3(twist);

  const PlaneBaResult result = solvePlaneBa(features, poses, PlaneBaConfig());
  EXPECT_EQ(result.termination, "no_valid_features");
  EXPECT_FALSE(result.improved);
  for (size_t p = 0; p < poses.size(); ++p) {
    EXPECT_EQ(
      0,
      std::memcmp(poses[p].data(), result.poses[p].data(), 16 * sizeof(double)));
  }
}

TEST(PlaneBa, HugePerturbationStaysFiniteAndImproves) {
  const auto world = makeOrthogonalPlaneWorld(0.005);
  const auto ground_truth = makeGroundTruthPoses();
  const auto features = makeFeatures(world, ground_truth);
  const auto initial = perturbPoses(ground_truth, 0.5, 0.1);

  PlaneBaConfig config;
  config.max_iterations = 80;
  config.prior_translation_sigma = 0.0;
  const PlaneBaResult result = solvePlaneBa(features, initial, config);
  EXPECT_TRUE(result.improved);
  for (size_t p = 0; p < result.poses.size(); ++p) {
    EXPECT_TRUE(result.poses[p].allFinite());
  }
}

TEST(PlaneBa, FirstPoseIsBitwiseUnchangedWhenFixed) {
  const auto world = makeOrthogonalPlaneWorld(0.005);
  const auto ground_truth = makeGroundTruthPoses();
  const auto features = makeFeatures(world, ground_truth);
  const auto initial = perturbPoses(ground_truth, 0.05, 0.017);

  const PlaneBaResult result = solvePlaneBa(features, initial, PlaneBaConfig());
  EXPECT_EQ(
    0,
    std::memcmp(initial[0].data(), result.poses[0].data(), 16 * sizeof(double)));
}

TEST(PlaneBa, DeterministicAcrossRepeatedSolves) {
  const auto world = makeOrthogonalPlaneWorld(0.005);
  const auto ground_truth = makeGroundTruthPoses();
  const auto features = makeFeatures(world, ground_truth);
  const auto initial = perturbPoses(ground_truth, 0.05, 0.017);

  const PlaneBaResult first = solvePlaneBa(features, initial, PlaneBaConfig());
  const PlaneBaResult second = solvePlaneBa(features, initial, PlaneBaConfig());
  ASSERT_EQ(first.poses.size(), second.poses.size());
  for (size_t p = 0; p < first.poses.size(); ++p) {
    EXPECT_EQ(
      0,
      std::memcmp(first.poses[p].data(), second.poses[p].data(), 16 * sizeof(double)));
  }
  EXPECT_EQ(
    0, std::memcmp(&first.final_cost, &second.final_cost, sizeof(double)));
}

TEST(PlaneBa, CostNeverIncreases) {
  const auto world = makeOrthogonalPlaneWorld(0.005);
  const auto ground_truth = makeGroundTruthPoses();
  const auto features = makeFeatures(world, ground_truth);
  const auto initial = perturbPoses(ground_truth, 0.05, 0.017);

  const PlaneBaResult result = solvePlaneBa(features, initial, PlaneBaConfig());
  EXPECT_LE(result.final_cost, result.initial_cost);
  EXPECT_GE(result.accepted_steps, 1);
}

}  // namespace
