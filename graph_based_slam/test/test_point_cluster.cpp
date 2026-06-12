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
#include <cstdint>
#include <cstring>
#include <random>
#include <vector>

#include "graph_based_slam/point_cluster.hpp"

namespace
{

using graphslam::map_refinement::PointCluster;
using graphslam::map_refinement::Vector6d;
using graphslam::map_refinement::clusterFirstDerivative;
using graphslam::map_refinement::clusterSecondDerivative;
using graphslam::map_refinement::expSe3;
using graphslam::map_refinement::makeCluster;
using graphslam::map_refinement::minEigenvalueOfScatter;

std::vector<Eigen::Vector3d> fixedPoints()
{
  std::vector<Eigen::Vector3d> points;
  points.push_back(Eigen::Vector3d(-1.0, 0.0, 0.25));
  points.push_back(Eigen::Vector3d(-0.3, 0.8, -0.4));
  points.push_back(Eigen::Vector3d(0.2, -0.7, 0.9));
  points.push_back(Eigen::Vector3d(1.4, 0.5, -0.2));
  points.push_back(Eigen::Vector3d(0.6, 1.1, 0.3));
  points.push_back(Eigen::Vector3d(-0.8, -1.2, -0.6));
  points.push_back(Eigen::Vector3d(1.1, -0.4, 0.7));
  return points;
}

Eigen::Matrix4d testPose()
{
  Vector6d twist = Vector6d::Zero();
  twist << 0.7, -1.2, 0.4, 0.3, -0.2, 0.5;
  return expSe3(twist);
}

bool matrix4BitwiseEqual(
  const Eigen::Matrix4d & lhs,
  const Eigen::Matrix4d & rhs)
{
  return std::memcmp(lhs.data(), rhs.data(), 16U * sizeof(double)) == 0;
}

void expectMatrix4Near(
  const Eigen::Matrix4d & actual,
  const Eigen::Matrix4d & expected,
  double tolerance)
{
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      EXPECT_NEAR(actual(row, col), expected(row, col), tolerance);
    }
  }
}

void expectMatrix3Near(
  const Eigen::Matrix3d & actual,
  const Eigen::Matrix3d & expected,
  double tolerance)
{
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      EXPECT_NEAR(actual(row, col), expected(row, col), tolerance);
    }
  }
}

void expectVector3Near(
  const Eigen::Vector3d & actual,
  const Eigen::Vector3d & expected,
  double tolerance)
{
  for (int row = 0; row < 3; ++row) {
    EXPECT_NEAR(actual(row), expected(row), tolerance);
  }
}

}  // namespace

TEST(PointClusterTest, TransformedClusterMatchesExplicitTransformedPoints)
{
  const std::vector<Eigen::Vector3d> points = fixedPoints();
  const Eigen::Matrix4d pose = testPose();

  const PointCluster local_cluster = makeCluster(points);
  const PointCluster transformed_cluster = local_cluster.transformed(pose);

  std::vector<Eigen::Vector3d> transformed_points;
  for (std::size_t i = 0; i < points.size(); ++i) {
    Eigen::Vector4d homogeneous;
    homogeneous << points[i], 1.0;
    transformed_points.push_back((pose * homogeneous).head<3>());
  }

  const PointCluster explicit_cluster = makeCluster(transformed_points);
  expectMatrix4Near(transformed_cluster.h, explicit_cluster.h, 1.0e-9);
  EXPECT_EQ(transformed_cluster.count, explicit_cluster.count);
}

TEST(PointClusterTest, ScatterAndCentroidMatchTwoPassComputation)
{
  const std::vector<Eigen::Vector3d> points = fixedPoints();
  const PointCluster cluster = makeCluster(points);

  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i < points.size(); ++i) {
    centroid += points[i];
  }
  centroid /= static_cast<double>(points.size());

  Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
  for (std::size_t i = 0; i < points.size(); ++i) {
    const Eigen::Vector3d offset = points[i] - centroid;
    scatter += offset * offset.transpose();
  }

  expectVector3Near(cluster.centroid(), centroid, 1.0e-12);
  expectMatrix3Near(cluster.scatter(), scatter, 1.0e-12);
}

TEST(PointClusterTest, MinEigenvalueOfCoplanarScatterIsNearZero)
{
  std::vector<Eigen::Vector3d> points;
  for (int x = -3; x <= 3; ++x) {
    for (int y = -2; y <= 2; ++y) {
      points.push_back(
        Eigen::Vector3d(
          static_cast<double>(x),
          static_cast<double>(y),
          0.0));
    }
  }

  const PointCluster cluster = makeCluster(points);
  EXPECT_LT(minEigenvalueOfScatter(cluster), 1.0e-18);
}

TEST(PointClusterTest, MinEigenvalueOfNoisyPlaneMatchesExpectedScatter)
{
  const double sigma = 0.01;
  std::mt19937 rng(12345U);
  std::normal_distribution<double> noise(0.0, sigma);

  std::vector<Eigen::Vector3d> points;
  for (int x = 0; x < 25; ++x) {
    for (int y = 0; y < 25; ++y) {
      points.push_back(
        Eigen::Vector3d(
          0.2 * static_cast<double>(x - 12),
          0.15 * static_cast<double>(y - 12),
          noise(rng)));
    }
  }

  const PointCluster cluster = makeCluster(points);
  const double expected = static_cast<double>(points.size()) * sigma * sigma;
  EXPECT_NEAR(minEigenvalueOfScatter(cluster), expected, 0.3 * expected);
}

TEST(PointClusterTest, MergeMatchesConcatenatedPointList)
{
  const std::vector<Eigen::Vector3d> points = fixedPoints();

  std::vector<Eigen::Vector3d> first_half;
  std::vector<Eigen::Vector3d> second_half;
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (i < points.size() / 2U) {
      first_half.push_back(points[i]);
    } else {
      second_half.push_back(points[i]);
    }
  }

  PointCluster merged = makeCluster(first_half);
  merged.merge(makeCluster(second_half));

  const PointCluster concatenated = makeCluster(points);
  expectMatrix4Near(merged.h, concatenated.h, 1.0e-12);
  EXPECT_EQ(merged.count, static_cast<std::int64_t>(points.size()));
}

TEST(PointClusterTest, FirstDerivativeMatchesCentralFiniteDifference)
{
  const std::vector<Eigen::Vector3d> points = fixedPoints();
  const PointCluster local_cluster = makeCluster(points);
  const Eigen::Matrix4d pose = testPose();
  const Eigen::Matrix4d current_h = local_cluster.transformed(pose).h;
  const double step = 1.0e-7;

  for (int r = 0; r < 6; ++r) {
    Vector6d delta = Vector6d::Zero();
    delta(r) = step;

    const Eigen::Matrix4d plus =
      local_cluster.transformed(expSe3(delta) * pose).h;
    const Eigen::Matrix4d minus =
      local_cluster.transformed(expSe3(-delta) * pose).h;
    const Eigen::Matrix4d finite_difference = (plus - minus) / (2.0 * step);
    const Eigen::Matrix4d analytic = clusterFirstDerivative(current_h, r);

    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        double scale = std::fabs(analytic(row, col));
        if (scale < 1.0) {
          scale = 1.0;
        }
        EXPECT_NEAR(
          finite_difference(row, col), analytic(row, col), 1.0e-5 * scale);
      }
    }
  }
}

TEST(PointClusterTest, SecondDerivativeIsSymmetricBitwise)
{
  const PointCluster cluster = makeCluster(fixedPoints());
  const Eigen::Matrix4d h = cluster.transformed(testPose()).h;

  for (int r = 0; r < 6; ++r) {
    for (int s = 0; s < 6; ++s) {
      const Eigen::Matrix4d first = clusterSecondDerivative(h, r, s);
      const Eigen::Matrix4d second = clusterSecondDerivative(h, s, r);
      EXPECT_TRUE(matrix4BitwiseEqual(first, second));
    }
  }
}

TEST(PointClusterTest, MakeClusterIsDeterministicBitwise)
{
  const std::vector<Eigen::Vector3d> points = fixedPoints();

  const PointCluster first = makeCluster(points);
  const PointCluster second = makeCluster(points);

  EXPECT_TRUE(matrix4BitwiseEqual(first.h, second.h));
  EXPECT_EQ(first.count, second.count);
}
