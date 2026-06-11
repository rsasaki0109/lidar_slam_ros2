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

#include "graph_based_slam/se3_lie.hpp"

namespace
{

using graphslam::map_refinement::Vector6d;
using graphslam::map_refinement::expSe3;
using graphslam::map_refinement::generator;
using graphslam::map_refinement::leftPerturb;
using graphslam::map_refinement::logSe3;

bool matrixBitwiseEqual(
  const Eigen::Matrix4d & lhs,
  const Eigen::Matrix4d & rhs)
{
  return std::memcmp(lhs.data(), rhs.data(), 16U * sizeof(double)) == 0;
}

void expectRoundTrip(const Vector6d & twist)
{
  const Vector6d round_trip = logSe3(expSe3(twist));
  EXPECT_TRUE((round_trip.isApprox(twist, 1.0e-9)));
}

}  // namespace

TEST(Se3LieTest, ExpLogRoundTripForFixedTwists)
{
  Vector6d zero = Vector6d::Zero();
  expectRoundTrip(zero);

  Vector6d pure_translation = Vector6d::Zero();
  pure_translation << 1.25, -0.5, 2.0, 0.0, 0.0, 0.0;
  expectRoundTrip(pure_translation);

  Vector6d small_rotation = Vector6d::Zero();
  small_rotation << 0.2, -0.1, 0.05, 1.0e-10, -2.0e-10, 3.0e-10;
  expectRoundTrip(small_rotation);

  Vector6d large_rotation = Vector6d::Zero();
  large_rotation << -0.3, 0.8, 1.2, 1.9, -1.1, 0.7;
  expectRoundTrip(large_rotation);
}

TEST(Se3LieTest, ExpOfZeroIsExactlyIdentity)
{
  const Eigen::Matrix4d actual = expSe3(Vector6d::Zero());
  const Eigen::Matrix4d expected = Eigen::Matrix4d::Identity();

  EXPECT_TRUE(matrixBitwiseEqual(actual, expected));
}

TEST(Se3LieTest, GeneratorMatchesFiniteDifferenceAtIdentity)
{
  const double step = 1.0e-7;

  for (int r = 0; r < 6; ++r) {
    Vector6d delta = Vector6d::Zero();
    delta(r) = step;

    const Eigen::Matrix4d finite_difference =
      (expSe3(delta) - expSe3(-delta)) / (2.0 * step);
    const Eigen::Matrix4d & analytic = generator(r);

    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        EXPECT_NEAR(finite_difference(row, col), analytic(row, col), 1.0e-6);
      }
    }
  }
}

TEST(Se3LieTest, LeftPerturbMatchesExplicitCompositionBitwise)
{
  Vector6d pose_twist = Vector6d::Zero();
  pose_twist << 1.0, -0.4, 0.7, 0.2, -0.3, 0.1;

  Vector6d delta = Vector6d::Zero();
  delta << 0.03, -0.02, 0.01, -0.04, 0.05, -0.02;

  const Eigen::Matrix4d pose = expSe3(pose_twist);
  const Eigen::Matrix4d actual = leftPerturb(delta, pose);
  const Eigen::Matrix4d expected = expSe3(delta) * pose;

  EXPECT_TRUE(matrixBitwiseEqual(actual, expected));
}

TEST(Se3LieTest, ExpRotationBlockIsOrthonormal)
{
  Vector6d twist = Vector6d::Zero();
  twist << 0.5, -1.0, 0.25, 0.7, -0.4, 0.9;

  const Eigen::Matrix3d rotation = expSe3(twist).topLeftCorner<3, 3>();
  const Eigen::Matrix3d should_be_identity =
    rotation.transpose() * rotation;

  EXPECT_TRUE((should_be_identity.isApprox(Eigen::Matrix3d::Identity(), 1.0e-12)));
}

TEST(Se3LieTest, ExpIsDeterministicBitwise)
{
  Vector6d twist = Vector6d::Zero();
  twist << -0.8, 0.6, 1.1, -0.2, 0.35, -0.45;

  const Eigen::Matrix4d first = expSe3(twist);
  const Eigen::Matrix4d second = expSe3(twist);

  EXPECT_TRUE(matrixBitwiseEqual(first, second));
}
