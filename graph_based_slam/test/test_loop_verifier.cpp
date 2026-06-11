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

// Characterization tests for the loop-closure verification decision logic
// extracted from searchLoopForLatest. These pin the historical semantics
// (threshold fallbacks, >= vs > boundaries, first-wins ties, ScanContext
// preference) so the extraction and the later BackendCore reuse cannot
// silently change behavior.

#include <gtest/gtest.h>

#include <cmath>

#include <Eigen/Core>  // NOLINT(build/include_order)
#include <Eigen/Geometry>  // NOLINT(build/include_order)

#include "graph_based_slam/loop_verifier.hpp"

namespace graphslam
{
namespace
{

using loop_verifier::GateConfig;
using loop_verifier::GateRejection;
using loop_verifier::LoopCandidate;
using loop_verifier::LoopCandidateResult;
using loop_verifier::RegistrationDelta;
using loop_verifier::SelectionState;

using Source = LoopCandidate::Source;

Eigen::Affine3d makePose(double x, double y, double z, double yaw_rad)
{
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  pose.translate(Eigen::Vector3d(x, y, z));
  pose.rotate(Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()));
  return pose;
}

LoopCandidateResult makeResult(int index, double fitness, Source source)
{
  LoopCandidateResult result;
  result.index = index;
  result.fitness_score = fitness;
  result.source = source;
  result.valid = true;
  return result;
}

GateConfig makeGateConfig()
{
  GateConfig config;
  config.generic_score_threshold = 1.0;
  config.scan_context_score_threshold = 0.0;
  config.max_translation_m = 5.0;
  config.max_rotation_deg = 30.0;
  config.max_translation_descriptor_m = 0.0;
  config.max_rotation_descriptor_deg = 0.0;
  return config;
}

TEST(LoopVerifierDelta, RecoversKnownRotationAndTranslation)
{
  const double yaw = 30.0 * M_PI / 180.0;
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) =
    Eigen::AngleAxisf(static_cast<float>(yaw), Eigen::Vector3f::UnitZ()).toRotationMatrix();
  transform.block<3, 1>(0, 3) = Eigen::Vector3f(1.0F, 2.0F, 2.0F);

  const auto delta = loop_verifier::computeRegistrationDelta(transform);
  EXPECT_NEAR(delta.translation_m, 3.0, 1e-6);
  EXPECT_NEAR(delta.rotation_deg, 30.0, 1e-4);
}

TEST(LoopVerifierDelta, IdentityIsZeroAndTraceIsClamped)
{
  const auto delta = loop_verifier::computeRegistrationDelta(Eigen::Matrix4f::Identity());
  EXPECT_DOUBLE_EQ(delta.translation_m, 0.0);
  // Float round-off can push the trace of a near-identity rotation above
  // 3.0; the clamp keeps acos finite instead of NaN.
  EXPECT_FALSE(std::isnan(delta.rotation_deg));
  EXPECT_DOUBLE_EQ(delta.rotation_deg, 0.0);

  Eigen::Matrix4f half_turn = Eigen::Matrix4f::Identity();
  half_turn.block<3, 3>(0, 0) =
    Eigen::AngleAxisf(static_cast<float>(M_PI), Eigen::Vector3f::UnitZ()).toRotationMatrix();
  EXPECT_NEAR(loop_verifier::computeRegistrationDelta(half_turn).rotation_deg, 180.0, 1e-3);
}

TEST(LoopVerifierGuess, DefaultIsCandidateTimesLatestInverse)
{
  const auto candidate_pose = makePose(10.0, -4.0, 1.0, 0.8);
  const auto latest_pose = makePose(-2.0, 7.0, 0.5, -1.2);
  LoopCandidate candidate;
  candidate.source = Source::DISTANCE;

  const Eigen::Matrix4f guess =
    loop_verifier::computeInitialGuess(candidate_pose, latest_pose, candidate);
  const Eigen::Matrix4f expected =
    (candidate_pose.matrix() * latest_pose.inverse().matrix()).cast<float>();
  EXPECT_TRUE(guess.isApprox(expected));
}

TEST(LoopVerifierGuess, YawHintInsertsAYawCorrection)
{
  const auto candidate_pose = makePose(3.0, 2.0, 0.0, 0.1);
  const auto latest_pose = makePose(1.0, 1.0, 0.0, 0.0);
  LoopCandidate candidate;
  candidate.source = Source::SCAN_CONTEXT;
  candidate.yaw_rad = 0.3;

  const Eigen::Matrix4f guess =
    loop_verifier::computeInitialGuess(candidate_pose, latest_pose, candidate);
  Eigen::Affine3d yaw_correction = Eigen::Affine3d::Identity();
  yaw_correction.rotate(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()));
  const Eigen::Matrix4f expected =
    (candidate_pose.matrix() * yaw_correction.matrix() *
    latest_pose.inverse().matrix()).cast<float>();
  EXPECT_TRUE(guess.isApprox(expected));

  // A yaw at or below the 1e-6 dead band falls back to the default guess.
  candidate.yaw_rad = 1e-7;
  const Eigen::Matrix4f tiny_yaw_guess =
    loop_verifier::computeInitialGuess(candidate_pose, latest_pose, candidate);
  const Eigen::Matrix4f default_guess =
    (candidate_pose.matrix() * latest_pose.inverse().matrix()).cast<float>();
  EXPECT_TRUE(tiny_yaw_guess.isApprox(default_guess));
}

TEST(LoopVerifierGuess, TriangleRelativeTransformIsChainedBetweenThePoses)
{
  const auto candidate_pose = makePose(5.0, 0.0, 0.0, 0.4);
  const auto latest_pose = makePose(0.0, 5.0, 0.0, -0.4);
  LoopCandidate candidate;
  candidate.source = Source::TRIANGLE_DESCRIPTOR;
  candidate.has_relative_transform = true;
  Eigen::Matrix4f relative = Eigen::Matrix4f::Identity();
  relative.block<3, 1>(0, 3) = Eigen::Vector3f(0.5F, -0.25F, 0.0F);
  candidate.relative_transform = relative;
  candidate.yaw_rad = 1.0;  // must be ignored when the SE(3) is present

  const Eigen::Matrix4f guess =
    loop_verifier::computeInitialGuess(candidate_pose, latest_pose, candidate);
  const Eigen::Matrix4f expected =
    (candidate_pose.matrix() * relative.cast<double>() *
    latest_pose.inverse().matrix()).cast<float>();
  EXPECT_TRUE(guess.isApprox(expected));
}

TEST(LoopVerifierGuess, RelativeTransformOnNonTriangleSourceIsIgnored)
{
  const auto candidate_pose = makePose(5.0, 0.0, 0.0, 0.4);
  const auto latest_pose = makePose(0.0, 5.0, 0.0, -0.4);
  LoopCandidate candidate;
  candidate.source = Source::BEV_DESCRIPTOR;
  candidate.has_relative_transform = true;
  Eigen::Matrix4f relative = Eigen::Matrix4f::Identity();
  relative.block<3, 1>(0, 3) = Eigen::Vector3f(9.0F, 9.0F, 9.0F);
  candidate.relative_transform = relative;

  const Eigen::Matrix4f guess =
    loop_verifier::computeInitialGuess(candidate_pose, latest_pose, candidate);
  const Eigen::Matrix4f expected =
    (candidate_pose.matrix() * latest_pose.inverse().matrix()).cast<float>();
  EXPECT_TRUE(guess.isApprox(expected));
}

TEST(LoopVerifierGuess, OnlyDistanceWithout3dBbsAlignsWithoutAGuess)
{
  EXPECT_FALSE(loop_verifier::shouldUseInitialGuess(Source::DISTANCE, false));
  EXPECT_TRUE(loop_verifier::shouldUseInitialGuess(Source::DISTANCE, true));
  EXPECT_TRUE(loop_verifier::shouldUseInitialGuess(Source::SCAN_CONTEXT, false));
  EXPECT_TRUE(loop_verifier::shouldUseInitialGuess(Source::TRIANGLE_DESCRIPTOR, false));
}

TEST(LoopVerifierGates, FitnessExactlyAtThresholdIsRejected)
{
  const auto config = makeGateConfig();
  RegistrationDelta delta;
  const auto at_threshold = loop_verifier::evaluateGates(
    Source::DISTANCE, 1.0, delta, config);
  EXPECT_EQ(at_threshold.rejection, GateRejection::FITNESS);
  EXPECT_DOUBLE_EQ(at_threshold.score_threshold, 1.0);

  const auto below = loop_verifier::evaluateGates(
    Source::DISTANCE, 0.999, delta, config);
  EXPECT_EQ(below.rejection, GateRejection::NONE);
}

TEST(LoopVerifierGates, ScanContextThresholdAppliesOnlyWhenPositive)
{
  auto config = makeGateConfig();
  config.scan_context_score_threshold = 0.5;
  RegistrationDelta delta;

  const auto sc = loop_verifier::evaluateGates(Source::SCAN_CONTEXT, 0.7, delta, config);
  EXPECT_EQ(sc.rejection, GateRejection::FITNESS);
  EXPECT_DOUBLE_EQ(sc.score_threshold, 0.5);

  // Other sources keep the generic threshold even when the SC one is set.
  const auto bev = loop_verifier::evaluateGates(Source::BEV_DESCRIPTOR, 0.7, delta, config);
  EXPECT_EQ(bev.rejection, GateRejection::NONE);

  // A non-positive SC threshold falls back to the generic one.
  config.scan_context_score_threshold = 0.0;
  const auto fallback = loop_verifier::evaluateGates(Source::SCAN_CONTEXT, 0.7, delta, config);
  EXPECT_EQ(fallback.rejection, GateRejection::NONE);
  EXPECT_DOUBLE_EQ(fallback.score_threshold, 1.0);
}

TEST(LoopVerifierGates, DescriptorCapsApplyOnlyToDescriptorSourcesAndWhenPositive)
{
  auto config = makeGateConfig();
  config.max_translation_descriptor_m = 10.0;
  config.max_rotation_descriptor_deg = 60.0;
  RegistrationDelta delta;
  delta.translation_m = 7.0;
  delta.rotation_deg = 45.0;

  // Descriptor source: the relaxed caps accept the larger correction.
  const auto sc = loop_verifier::evaluateGates(Source::SCAN_CONTEXT, 0.1, delta, config);
  EXPECT_EQ(sc.rejection, GateRejection::NONE);
  EXPECT_DOUBLE_EQ(sc.translation_cap_m, 10.0);
  EXPECT_DOUBLE_EQ(sc.rotation_cap_deg, 60.0);

  // DISTANCE always keeps the strict generic caps.
  const auto distance = loop_verifier::evaluateGates(Source::DISTANCE, 0.1, delta, config);
  EXPECT_EQ(distance.rejection, GateRejection::TRANSLATION);
  EXPECT_DOUBLE_EQ(distance.translation_cap_m, 5.0);

  // Non-positive descriptor caps fall back to the generic ones.
  config.max_translation_descriptor_m = 0.0;
  config.max_rotation_descriptor_deg = 0.0;
  const auto fallback = loop_verifier::evaluateGates(Source::SCAN_CONTEXT, 0.1, delta, config);
  EXPECT_EQ(fallback.rejection, GateRejection::TRANSLATION);
  EXPECT_DOUBLE_EQ(fallback.translation_cap_m, 5.0);
}

TEST(LoopVerifierGates, GateOrderIsFitnessThenTranslationThenRotation)
{
  const auto config = makeGateConfig();
  RegistrationDelta delta;
  delta.translation_m = 100.0;
  delta.rotation_deg = 100.0;

  const auto all_fail = loop_verifier::evaluateGates(Source::DISTANCE, 2.0, delta, config);
  EXPECT_EQ(all_fail.rejection, GateRejection::FITNESS);

  const auto trans_rot_fail = loop_verifier::evaluateGates(Source::DISTANCE, 0.1, delta, config);
  EXPECT_EQ(trans_rot_fail.rejection, GateRejection::TRANSLATION);

  delta.translation_m = 1.0;
  const auto rot_fail = loop_verifier::evaluateGates(Source::DISTANCE, 0.1, delta, config);
  EXPECT_EQ(rot_fail.rejection, GateRejection::ROTATION);
}

TEST(LoopVerifierGates, CorrectionExactlyAtTheCapPasses)
{
  const auto config = makeGateConfig();
  RegistrationDelta delta;
  delta.translation_m = 5.0;
  delta.rotation_deg = 30.0;
  const auto result = loop_verifier::evaluateGates(Source::DISTANCE, 0.1, delta, config);
  EXPECT_EQ(result.rejection, GateRejection::NONE);
}

TEST(LoopVerifierSelection, FirstArrivalWinsOnEqualFitness)
{
  SelectionState selection;
  selection.considerValid(makeResult(3, 0.5, Source::DISTANCE));
  selection.considerValid(makeResult(7, 0.5, Source::DISTANCE));
  const auto best = selection.select(false);
  ASSERT_TRUE(best.valid);
  EXPECT_EQ(best.index, 3);
}

TEST(LoopVerifierSelection, BestAttemptTracksGateRejectedCandidatesToo)
{
  SelectionState selection;
  // A converged registration whose gates rejected it: only considerConverged.
  auto rejected = makeResult(2, 0.05, Source::DISTANCE);
  rejected.valid = false;
  selection.considerConverged(rejected);

  auto accepted = makeResult(9, 0.4, Source::DISTANCE);
  selection.considerConverged(accepted);
  selection.considerValid(accepted);

  EXPECT_EQ(selection.best_attempt.index, 2);
  const auto best = selection.select(false);
  ASSERT_TRUE(best.valid);
  EXPECT_EQ(best.index, 9);
}

TEST(LoopVerifierSelection, ScanContextPreferenceOverridesBetterGenericFitness)
{
  SelectionState selection;
  selection.considerValid(makeResult(4, 0.1, Source::DISTANCE));
  selection.considerValid(makeResult(8, 0.6, Source::SCAN_CONTEXT));

  const auto unpreferred = selection.select(false);
  EXPECT_EQ(unpreferred.index, 4);

  const auto preferred = selection.select(true);
  EXPECT_EQ(preferred.index, 8);
  EXPECT_EQ(preferred.source, Source::SCAN_CONTEXT);

  // Preference without any valid ScanContext result is a no-op.
  SelectionState no_sc;
  no_sc.considerValid(makeResult(4, 0.1, Source::DISTANCE));
  EXPECT_EQ(no_sc.select(true).index, 4);
}

TEST(LoopVerifierSelection, NoValidCandidateSelectsAnInvalidResult)
{
  SelectionState selection;
  auto rejected = makeResult(2, 0.05, Source::DISTANCE);
  rejected.valid = false;
  selection.considerConverged(rejected);

  const auto best = selection.select(true);
  EXPECT_FALSE(best.valid);
  EXPECT_EQ(best.index, -1);
}

TEST(LoopVerifierEdge, RelativePoseChainsCorrectionOntoTheLatestPose)
{
  const auto candidate_pose = makePose(10.0, 0.0, 0.0, 0.0);
  const auto latest_pose = makePose(10.0, 2.0, 0.0, 0.0);

  // Identity correction: the edge is exactly candidate^-1 * latest.
  const auto identity_edge = loop_verifier::composeLoopRelativePose(
    candidate_pose, latest_pose, Eigen::Matrix4f::Identity());
  EXPECT_NEAR(identity_edge.translation().y(), 2.0, 1e-9);

  // A correction that moves the latest submap onto the candidate closes
  // the residual.
  Eigen::Matrix4f correction = Eigen::Matrix4f::Identity();
  correction(1, 3) = -2.0F;
  const auto closed_edge = loop_verifier::composeLoopRelativePose(
    candidate_pose, latest_pose, correction);
  EXPECT_NEAR(closed_edge.translation().norm(), 0.0, 1e-6);
}

}  // namespace
}  // namespace graphslam
