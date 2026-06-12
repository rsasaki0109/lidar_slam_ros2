// Copyright 2026 Sasaki
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <vector>

#include "graph_based_slam/scatter_eigen_cost.hpp"

namespace
{

using graphslam::map_refinement::PlaneCostConfig;
using graphslam::map_refinement::PlaneCostResult;
using graphslam::map_refinement::PlaneFeatureObservation;
using graphslam::map_refinement::PointCluster;
using graphslam::map_refinement::Vector6d;
using graphslam::map_refinement::evaluatePlaneCost;
using graphslam::map_refinement::expSe3;
using graphslam::map_refinement::leftPerturb;
using graphslam::map_refinement::makeCluster;

struct PlaneFixture
{
  std::vector<PlaneFeatureObservation> observations;
  std::vector<Eigen::Matrix4d> poses;
};

Eigen::Matrix4d MakePose(
  const Eigen::Vector3d & translation,
  const Eigen::Vector3d & rotation)
{
  Vector6d delta = Vector6d::Zero();
  delta.head<3>() = translation;
  delta.tail<3>() = rotation;
  return expSe3(delta);
}

Eigen::Vector3d TransformPoint(
  const Eigen::Matrix4d & transform,
  const Eigen::Vector3d & point)
{
  Eigen::Vector4d homogeneous;
  homogeneous << point.x(), point.y(), point.z(), 1.0;
  return (transform * homogeneous).head<3>();
}

std::vector<Eigen::Vector3d> TransformPoints(
  const Eigen::Matrix4d & transform,
  const std::vector<Eigen::Vector3d> & points)
{
  std::vector<Eigen::Vector3d> transformed;
  transformed.reserve(points.size());

  for (std::size_t i = 0; i < points.size(); ++i) {
    transformed.push_back(TransformPoint(transform, points[i]));
  }

  return transformed;
}

PlaneFeatureObservation MakeObservation(
  int pose_index,
  const Eigen::Matrix4d & pose,
  const std::vector<Eigen::Vector3d> & world_points)
{
  PlaneFeatureObservation observation;
  observation.pose_index = pose_index;
  observation.local_cluster = makeCluster(
    TransformPoints(pose.inverse(), world_points));
  return observation;
}

std::vector<Eigen::Vector3d> MakeCoplanarPoints()
{
  std::vector<Eigen::Vector3d> points;
  points.push_back(Eigen::Vector3d(-2.0, -1.0, 0.0));
  points.push_back(Eigen::Vector3d(-1.0, 1.0, 0.0));
  points.push_back(Eigen::Vector3d(0.0, -1.5, 0.0));
  points.push_back(Eigen::Vector3d(0.8, 0.4, 0.0));
  points.push_back(Eigen::Vector3d(1.7, -0.2, 0.0));
  points.push_back(Eigen::Vector3d(2.2, 1.3, 0.0));
  return points;
}

std::vector<Eigen::Vector3d> MakeNoisyPlanePoints()
{
  const double xy[][2] = {
    {-1.5, -0.8},
    {-0.4, -1.2},
    {0.7, -0.9},
    {1.6, -0.4},
    {-1.2, 0.2},
    {-0.2, 0.5},
    {0.9, 0.3},
    {1.4, 1.0},
    {-0.8, 1.3},
    {0.4, 1.5}
  };
  const double noise[] = {-0.020, 0.010, 0.000, 0.030, -0.010,
    0.020, -0.030, 0.010, -0.020, 0.020};

  std::vector<Eigen::Vector3d> points;
  points.reserve(sizeof(noise) / sizeof(noise[0]));

  for (std::size_t i = 0; i < sizeof(noise) / sizeof(noise[0]); ++i) {
    const double x = xy[i][0];
    const double y = xy[i][1];
    const double z = 0.25 * x - 0.18 * y + 0.4 + noise[i];
    points.push_back(Eigen::Vector3d(x, y, z));
  }

  return points;
}

std::vector<Eigen::Vector3d> MakeLinePoints()
{
  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < 8; ++i) {
    const double u = -1.75 + 0.5 * static_cast<double>(i);
    points.push_back(Eigen::Vector3d(u, 2.0 * u + 1.0, -0.5 * u));
  }
  return points;
}

PlaneFixture MakeNoisyPlaneFixture()
{
  const std::vector<Eigen::Vector3d> world_points = MakeNoisyPlanePoints();

  PlaneFixture fixture;
  fixture.poses.push_back(
    MakePose(Eigen::Vector3d(0.2, -0.1, 0.3), Eigen::Vector3d(0.04, -0.03, 0.02)));
  fixture.poses.push_back(
    MakePose(Eigen::Vector3d(-0.4, 0.3, -0.2), Eigen::Vector3d(-0.02, 0.05, -0.04)));

  fixture.observations.push_back(MakeObservation(0, fixture.poses[0], world_points));
  fixture.observations.push_back(MakeObservation(1, fixture.poses[1], world_points));
  return fixture;
}

PlaneCostResult EvaluateFixture(
  const PlaneFixture & fixture,
  const std::vector<Eigen::Matrix4d> & poses)
{
  PlaneCostConfig config;
  return evaluatePlaneCost(
    fixture.observations,
    poses,
    static_cast<int>(poses.size()),
    config);
}

std::vector<Eigen::Matrix4d> PerturbPose(
  const std::vector<Eigen::Matrix4d> & poses,
  int variable,
  double step)
{
  std::vector<Eigen::Matrix4d> perturbed = poses;
  Vector6d delta = Vector6d::Zero();
  const int pose_index = variable / 6;
  const int axis = variable % 6;
  delta(axis) = step;
  perturbed[static_cast<std::size_t>(pose_index)] =
    leftPerturb(delta, perturbed[static_cast<std::size_t>(pose_index)]);
  return perturbed;
}

}  // namespace

TEST(ScatterEigenCostTest, CoplanarTwoPoseFeatureAtIdentityIsFlat)
{
  PlaneFixture fixture;
  const std::vector<Eigen::Vector3d> points = MakeCoplanarPoints();
  fixture.poses.push_back(Eigen::Matrix4d::Identity());
  fixture.poses.push_back(Eigen::Matrix4d::Identity());
  fixture.observations.push_back(MakeObservation(0, fixture.poses[0], points));
  fixture.observations.push_back(MakeObservation(1, fixture.poses[1], points));

  const PlaneCostResult result = EvaluateFixture(fixture, fixture.poses);

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(
    result.cost,
    0.0,
    1e-12);
  EXPECT_GT(result.eigen_gap, 1e-9);
  EXPECT_LT(result.gradient.norm(), 1e-9);
}

TEST(ScatterEigenCostTest, GradientMatchesCentralFiniteDifference)
{
  const PlaneFixture fixture = MakeNoisyPlaneFixture();
  const PlaneCostResult result = EvaluateFixture(fixture, fixture.poses);
  ASSERT_TRUE(result.valid);

  const double step = 1e-6;
  const int variable_count = static_cast<int>(result.gradient.size());

  for (int variable = 0; variable < variable_count; ++variable) {
    const std::vector<Eigen::Matrix4d> plus_poses =
      PerturbPose(fixture.poses, variable, step);
    const std::vector<Eigen::Matrix4d> minus_poses =
      PerturbPose(fixture.poses, variable, -step);

    const PlaneCostResult plus = EvaluateFixture(fixture, plus_poses);
    const PlaneCostResult minus = EvaluateFixture(fixture, minus_poses);
    ASSERT_TRUE(plus.valid);
    ASSERT_TRUE(minus.valid);

    const double finite_difference = (plus.cost - minus.cost) / (2.0 * step);
    const double tolerance = 1e-5 * std::max(1.0, std::abs(finite_difference));

    EXPECT_NEAR(
      result.gradient(variable),
      finite_difference,
      tolerance) << "variable " << variable;
  }
}

TEST(ScatterEigenCostTest, HessianMatchesCentralGradientDifference)
{
  const PlaneFixture fixture = MakeNoisyPlaneFixture();
  const PlaneCostResult result = EvaluateFixture(fixture, fixture.poses);
  ASSERT_TRUE(result.valid);

  const double step = 1e-6;
  const int variable_count = static_cast<int>(result.gradient.size());

  for (int column = 0; column < variable_count; ++column) {
    const std::vector<Eigen::Matrix4d> plus_poses =
      PerturbPose(fixture.poses, column, step);
    const std::vector<Eigen::Matrix4d> minus_poses =
      PerturbPose(fixture.poses, column, -step);

    const PlaneCostResult plus = EvaluateFixture(fixture, plus_poses);
    const PlaneCostResult minus = EvaluateFixture(fixture, minus_poses);
    ASSERT_TRUE(plus.valid);
    ASSERT_TRUE(minus.valid);

    const Eigen::VectorXd finite_difference =
      (plus.gradient - minus.gradient) / (2.0 * step);

    for (int row = 0; row < variable_count; ++row) {
      const double expected = finite_difference(row);
      const double tolerance = 1e-4 * std::max(1.0, std::abs(expected));

      EXPECT_NEAR(
        result.hessian(row, column),
        expected,
        tolerance) << "row " << row << " column " << column;
    }
  }
}

TEST(ScatterEigenCostTest, CommonRigidTransformLeavesCostUnchanged)
{
  const PlaneFixture fixture = MakeNoisyPlaneFixture();
  const PlaneCostResult original = EvaluateFixture(fixture, fixture.poses);
  ASSERT_TRUE(original.valid);

  const Eigen::Matrix4d common_transform =
    MakePose(Eigen::Vector3d(0.7, -0.2, 0.5), Eigen::Vector3d(0.2, -0.1, 0.15));

  std::vector<Eigen::Matrix4d> moved_poses = fixture.poses;
  for (std::size_t i = 0; i < moved_poses.size(); ++i) {
    moved_poses[i] = common_transform * moved_poses[i];
  }

  const PlaneCostResult moved = EvaluateFixture(fixture, moved_poses);
  ASSERT_TRUE(moved.valid);

  EXPECT_NEAR(
    original.cost,
    moved.cost,
    1e-9);
}

TEST(ScatterEigenCostTest, LineFeatureIsDegenerate)
{
  PlaneFixture fixture;
  const std::vector<Eigen::Vector3d> points = MakeLinePoints();
  fixture.poses.push_back(Eigen::Matrix4d::Identity());
  fixture.observations.push_back(MakeObservation(0, fixture.poses[0], points));

  PlaneCostConfig config;
  const PlaneCostResult result = evaluatePlaneCost(
    fixture.observations,
    fixture.poses,
    1,
    config);

  EXPECT_FALSE(result.valid);
}

TEST(ScatterEigenCostTest, TooFewPointsIsInvalid)
{
  PlaneFixture fixture;
  std::vector<Eigen::Vector3d> points = MakeCoplanarPoints();
  points.resize(5);
  fixture.poses.push_back(Eigen::Matrix4d::Identity());
  fixture.observations.push_back(MakeObservation(0, fixture.poses[0], points));

  PlaneCostConfig config;
  const PlaneCostResult result = evaluatePlaneCost(
    fixture.observations,
    fixture.poses,
    1,
    config);

  EXPECT_FALSE(result.valid);
}

TEST(ScatterEigenCostTest, EvaluationIsDeterministic)
{
  const PlaneFixture fixture = MakeNoisyPlaneFixture();
  const PlaneCostResult first = EvaluateFixture(fixture, fixture.poses);
  const PlaneCostResult second = EvaluateFixture(fixture, fixture.poses);
  ASSERT_TRUE(first.valid);
  ASSERT_TRUE(second.valid);

  ASSERT_EQ(first.gradient.size(), second.gradient.size());
  ASSERT_EQ(first.hessian.rows(), second.hessian.rows());
  ASSERT_EQ(first.hessian.cols(), second.hessian.cols());

  const std::size_t gradient_bytes =
    sizeof(double) * static_cast<std::size_t>(first.gradient.size());
  const std::size_t hessian_bytes =
    sizeof(double) * static_cast<std::size_t>(first.hessian.size());

  EXPECT_EQ(
    0,
    std::memcmp(first.gradient.data(), second.gradient.data(), gradient_bytes));
  EXPECT_EQ(
    0,
    std::memcmp(first.hessian.data(), second.hessian.data(), hessian_bytes));
}

TEST(ScatterEigenCostTest, DifferentPoseHessianBlockHasCentroidCoupling)
{
  const PlaneFixture fixture = MakeNoisyPlaneFixture();
  const PlaneCostResult result = EvaluateFixture(fixture, fixture.poses);
  ASSERT_TRUE(result.valid);

  bool has_nonzero = false;
  for (int row = 0; row < 6; ++row) {
    for (int column = 6; column < 12; ++column) {
      if (std::abs(result.hessian(row, column)) > 1e-12) {
        has_nonzero = true;
      }
    }
  }

  EXPECT_TRUE(has_nonzero);
}
