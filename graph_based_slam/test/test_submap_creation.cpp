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

// Characterization tests for the submap spacing decision (v0.6 Phase 1):
// the semantics below are pinned to the historical odom-input behaviour in
// tryCreateSubmap() before the extraction.

#include <gtest/gtest.h>

#include <Eigen/Core>

#include "graph_based_slam/submap_creation.hpp"

namespace graphslam
{
namespace
{

TEST(SubmapCreation, FirstValidPoseAlwaysCreates)
{
  const auto d = submap_creation::evaluate(
    Eigen::Vector3d(5.0, -2.0, 1.0), false, Eigen::Vector3d::Zero(), 1.5);
  EXPECT_TRUE(d.create);
  EXPECT_FALSE(d.jump_rejected);
  EXPECT_DOUBLE_EQ(0.0, d.distance);
}

TEST(SubmapCreation, BelowThresholdDoesNotCreate)
{
  const auto d = submap_creation::evaluate(
    Eigen::Vector3d(1.0, 0.0, 0.0), true, Eigen::Vector3d::Zero(), 1.5);
  EXPECT_FALSE(d.create);
  EXPECT_FALSE(d.jump_rejected);
  EXPECT_DOUBLE_EQ(1.0, d.distance);
}

TEST(SubmapCreation, AtThresholdCreates)
{
  // Historical comparison is `dist < threshold` -> skip, so exactly the
  // threshold distance creates a submap.
  const auto d = submap_creation::evaluate(
    Eigen::Vector3d(1.5, 0.0, 0.0), true, Eigen::Vector3d::Zero(), 1.5);
  EXPECT_TRUE(d.create);
  EXPECT_DOUBLE_EQ(1.5, d.distance);
}

TEST(SubmapCreation, JumpBeyondMaxIsRejectedNotCreated)
{
  const auto d = submap_creation::evaluate(
    Eigen::Vector3d(150.0, 0.0, 0.0), true, Eigen::Vector3d::Zero(), 1.5);
  EXPECT_FALSE(d.create);
  EXPECT_TRUE(d.jump_rejected);
  EXPECT_DOUBLE_EQ(150.0, d.distance);
}

TEST(SubmapCreation, ExactlyMaxJumpStillCreates)
{
  // Historical comparison is `dist > 100.0` -> reject, so exactly 100 m
  // passes the guard.
  const auto d = submap_creation::evaluate(
    Eigen::Vector3d(100.0, 0.0, 0.0), true, Eigen::Vector3d::Zero(), 1.5);
  EXPECT_TRUE(d.create);
  EXPECT_FALSE(d.jump_rejected);
}

TEST(SubmapCreation, DistanceIsEuclidean3d)
{
  const auto d = submap_creation::evaluate(
    Eigen::Vector3d(3.0, 4.0, 12.0), true, Eigen::Vector3d::Zero(), 1.5);
  EXPECT_TRUE(d.create);
  EXPECT_DOUBLE_EQ(13.0, d.distance);
}

TEST(SubmapCreation, CustomMaxJump)
{
  const auto d = submap_creation::evaluate(
    Eigen::Vector3d(50.0, 0.0, 0.0), true, Eigen::Vector3d::Zero(), 1.5, 40.0);
  EXPECT_FALSE(d.create);
  EXPECT_TRUE(d.jump_rejected);
}

}  // namespace
}  // namespace graphslam
