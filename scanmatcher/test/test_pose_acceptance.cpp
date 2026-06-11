#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#include "scanmatcher/pose_acceptance.hpp"

namespace
{
using graphslam::pose_acceptance::Config;
using graphslam::pose_acceptance::evaluate;
using graphslam::pose_acceptance::Input;
using graphslam::pose_acceptance::Outcome;
using graphslam::pose_acceptance::State;
using graphslam::pose_acceptance::TrackingState;

geometry_msgs::msg::Quaternion identityQuat()
{
  geometry_msgs::msg::Quaternion quat;
  quat.w = 1.0;
  return quat;
}

Input makeInput(
  const Eigen::Vector3d & position,
  const double stamp_sec,
  const double dt,
  const geometry_msgs::msg::Quaternion & previous_orientation)
{
  Input input;
  input.position = position;
  input.orientation = identityQuat();
  input.previous_orientation = previous_orientation;
  input.has_converged = true;
  input.fitness_score = 0.5;
  input.stamp_sec = stamp_sec;
  input.previous_stamp_sec = stamp_sec - dt;
  input.dt = dt;
  input.mapping_in_progress = false;
  return input;
}

// Drive one clean accepted scan (small motion, good fitness).
Outcome stepAccepted(const Config & config, State & state, const double stamp_sec)
{
  Input input = makeInput(
    state.previous_pose_diagnostic_position + Eigen::Vector3d(0.05, 0.0, 0.0),
    stamp_sec, 0.1, identityQuat());
  return evaluate(config, state, input);
}
}  // namespace

TEST(PoseAcceptanceTest, FirstScanAcceptsRawPoseAndInitializesState)
{
  Config config;
  State state;
  const Input input = makeInput(Eigen::Vector3d(1.0, 2.0, 3.0), 10.0, 0.0, identityQuat());

  const Outcome outcome = evaluate(config, state, input);

  EXPECT_TRUE(outcome.accepted_position.isApprox(input.position, 0.0));
  EXPECT_FALSE(outcome.hard_reject_pose_update);
  EXPECT_FALSE(outcome.soft_reject_map_update);
  EXPECT_FALSE(outcome.suppress_map_update);
  EXPECT_TRUE(outcome.warn_lines.empty());
  EXPECT_TRUE(state.previous_pose_diagnostic_valid);
  EXPECT_TRUE(state.reject_stats_initialized);
  EXPECT_EQ(state.accepted_pose_count, 1);
  EXPECT_DOUBLE_EQ(state.accepted_fitness_ema, 0.5);
  // The frame-to-frame delta needs a previous diagnostic pose.
  EXPECT_FALSE(state.last_accepted_delta_valid);
  EXPECT_EQ(state.tracking_state, TrackingState::Tracking);
}

TEST(PoseAcceptanceTest, MotionGateSoftRejectClipsThePoseAndEntersSuspect)
{
  Config config;
  State state;
  evaluate(config, state, makeInput(Eigen::Vector3d::Zero(), 1.0, 0.0, identityQuat()));

  // 1.5 m in 0.1 s: over the 8 m/s * 0.1 s = 0.8 m gate, under the 4x hard
  // multiplier (3.2 m) -> soft reject, clipped to the gate limit.
  const Input jump = makeInput(Eigen::Vector3d(1.5, 0.0, 0.0), 1.1, 0.1, identityQuat());
  const Outcome outcome = evaluate(config, state, jump);

  EXPECT_FALSE(outcome.hard_reject_pose_update);
  EXPECT_TRUE(outcome.soft_reject_map_update);
  EXPECT_TRUE(outcome.suppress_map_update);
  EXPECT_TRUE(outcome.accepted_position.isApprox(Eigen::Vector3d(0.8, 0.0, 0.0), 1e-12));
  EXPECT_EQ(state.tracking_state, TrackingState::Suspect);
  EXPECT_EQ(state.reject_map_update_cooldown_remaining, config.reject_map_update_cooldown_scans);

  ASSERT_EQ(outcome.warn_lines.size(), 3u);
  EXPECT_EQ(outcome.warn_lines[0].rfind("POSE_JUMP stamp=1.100000000 dt=0.100000", 0), 0u);
  EXPECT_EQ(outcome.warn_lines[1].rfind("POSE_REJECT stamp=1.100000000", 0), 0u);
  EXPECT_NE(outcome.warn_lines[1].find("mode=map_only"), std::string::npos);
  EXPECT_NE(outcome.warn_lines[1].find("motion_gate=true"), std::string::npos);
  EXPECT_NE(outcome.warn_lines[1].find("motion_gate_hard=false"), std::string::npos);
  EXPECT_EQ(outcome.warn_lines[2], "TRACKING_STATE suspect stamp=1.100000000");
}

TEST(PoseAcceptanceTest, NonConvergedScanHardRejectsIntoRecovery)
{
  Config config;
  State state;
  evaluate(config, state, makeInput(Eigen::Vector3d::Zero(), 1.0, 0.0, identityQuat()));

  Input bad = makeInput(Eigen::Vector3d(0.05, 0.0, 0.0), 1.1, 0.1, identityQuat());
  bad.has_converged = false;
  const Outcome outcome = evaluate(config, state, bad);

  EXPECT_TRUE(outcome.hard_reject_pose_update);
  EXPECT_FALSE(outcome.soft_reject_map_update);
  EXPECT_TRUE(outcome.suppress_map_update);
  // Hard reject falls back to the predicted pose (= previous, no delta yet).
  EXPECT_TRUE(outcome.accepted_position.isApprox(Eigen::Vector3d::Zero(), 0.0));
  EXPECT_EQ(state.tracking_state, TrackingState::Recovery);
  EXPECT_FALSE(state.last_accepted_delta_valid);
  EXPECT_FALSE(state.reject_stats_initialized);
  EXPECT_EQ(state.accepted_pose_count, 0);
  EXPECT_EQ(
    state.reject_map_update_cooldown_remaining,
    config.hard_reject_map_update_cooldown_scans);
  // reject_recovery_scans <= 1 and no mapping thread -> immediate refresh
  // request, and the historical duplicated HARD_RECOVERY warn is preserved.
  EXPECT_TRUE(outcome.request_recovery_target_refresh);
  int hard_recovery_lines = 0;
  for (const auto & line : outcome.warn_lines) {
    if (line.rfind("POSE_REJECT_HARD_RECOVERY", 0) == 0) {hard_recovery_lines += 1;}
  }
  EXPECT_EQ(hard_recovery_lines, 2);
}

TEST(PoseAcceptanceTest, InvalidFitnessEmitsTheDedicatedWarn)
{
  Config config;
  State state;
  evaluate(config, state, makeInput(Eigen::Vector3d::Zero(), 1.0, 0.0, identityQuat()));

  Input bad = makeInput(Eigen::Vector3d(0.05, 0.0, 0.0), 1.1, 0.1, identityQuat());
  bad.fitness_score = std::numeric_limits<double>::infinity();
  const Outcome outcome = evaluate(config, state, bad);

  EXPECT_TRUE(outcome.hard_reject_pose_update);
  bool found = false;
  for (const auto & line : outcome.warn_lines) {
    if (line.rfind("POSE_FITNESS_INVALID stamp=1.100000000", 0) == 0) {found = true;}
  }
  EXPECT_TRUE(found);
}

TEST(PoseAcceptanceTest, RecoveryClearsThroughSuspectToTrackingOnCleanAccepts)
{
  Config config;
  State state;
  evaluate(config, state, makeInput(Eigen::Vector3d::Zero(), 1.0, 0.0, identityQuat()));

  Input bad = makeInput(Eigen::Vector3d(0.05, 0.0, 0.0), 1.1, 0.1, identityQuat());
  bad.has_converged = false;
  evaluate(config, state, bad);
  ASSERT_EQ(state.tracking_state, TrackingState::Recovery);
  state.recovery_target_active = true;  // shell refresh result

  // recovery_clear_consecutive_accepted = 1: first clean accept -> Suspect.
  Outcome outcome = stepAccepted(config, state, 1.2);
  EXPECT_EQ(state.tracking_state, TrackingState::Suspect);
  ASSERT_FALSE(outcome.warn_lines.empty());
  EXPECT_EQ(outcome.warn_lines.back(), "TRACKING_STATE suspect stamp=1.200000000");

  // suspect_clear_consecutive_accepted = 2: two more -> Tracking.
  stepAccepted(config, state, 1.3);
  outcome = stepAccepted(config, state, 1.4);
  EXPECT_EQ(state.tracking_state, TrackingState::Tracking);
  EXPECT_FALSE(state.recovery_target_active);
  ASSERT_FALSE(outcome.warn_lines.empty());
  EXPECT_EQ(outcome.warn_lines.back(), "TRACKING_STATE tracking stamp=1.400000000");
}

TEST(PoseAcceptanceTest, SuppressMapUpdateDuringCooldownAndNonTracking)
{
  Config config;
  State state;
  evaluate(config, state, makeInput(Eigen::Vector3d::Zero(), 1.0, 0.0, identityQuat()));

  // Soft reject sets the cooldown (2 scans) and Suspect.
  evaluate(config, state, makeInput(Eigen::Vector3d(1.5, 0.0, 0.0), 1.1, 0.1, identityQuat()));
  ASSERT_EQ(state.reject_map_update_cooldown_remaining, 2);

  // Clean accept while Suspect: still suppressed (cooldown + state),
  // cooldown decrements.
  Outcome outcome = stepAccepted(config, state, 1.2);
  EXPECT_TRUE(outcome.suppress_map_update);
  EXPECT_EQ(state.reject_map_update_cooldown_remaining, 1);

  // Second clean accept returns to Tracking (suspect_clear = 2) but the
  // remaining cooldown still suppresses this scan.
  outcome = stepAccepted(config, state, 1.3);
  EXPECT_EQ(state.tracking_state, TrackingState::Tracking);
  EXPECT_TRUE(outcome.suppress_map_update);
  EXPECT_EQ(state.reject_map_update_cooldown_remaining, 0);

  // Cooldown drained and Tracking: map updates allowed again.
  outcome = stepAccepted(config, state, 1.4);
  EXPECT_FALSE(outcome.suppress_map_update);
}

TEST(PoseAcceptanceTest, NonMonotonicStampWarns)
{
  Config config;
  State state;
  evaluate(config, state, makeInput(Eigen::Vector3d::Zero(), 1.0, 0.0, identityQuat()));

  Input input = makeInput(Eigen::Vector3d(0.05, 0.0, 0.0), 0.9, -0.1, identityQuat());
  const Outcome outcome = evaluate(config, state, input);

  ASSERT_FALSE(outcome.warn_lines.empty());
  EXPECT_EQ(
    outcome.warn_lines[0],
    "POSE_STAMP_NONMONOTONIC stamp=0.900000000 prev_stamp=1.000000000 dt=-0.100000 "
    "frame=base_link");
}

TEST(PoseAcceptanceTest, AcceptedEmaFollowsTheHistoricalAlphaBlend)
{
  Config config;
  State state;
  Input first = makeInput(Eigen::Vector3d::Zero(), 1.0, 0.0, identityQuat());
  first.fitness_score = 0.5;
  evaluate(config, state, first);

  Input second = makeInput(Eigen::Vector3d(0.05, 0.0, 0.0), 1.1, 0.1, identityQuat());
  second.fitness_score = 1.0;
  evaluate(config, state, second);

  // alpha = 0.1, no cap before warmup: 0.9 * 0.5 + 0.1 * 1.0.
  EXPECT_DOUBLE_EQ(state.accepted_fitness_ema, 0.9 * 0.5 + 0.1 * 1.0);
  EXPECT_DOUBLE_EQ(state.accepted_trans_ema, 0.9 * 0.0 + 0.1 * 0.05);
  EXPECT_EQ(state.accepted_pose_count, 2);
  EXPECT_TRUE(state.last_accepted_delta_valid);
  EXPECT_TRUE(state.last_accepted_delta_position.isApprox(Eigen::Vector3d(0.05, 0.0, 0.0), 0.0));
}

TEST(PoseAcceptanceTest, SameSequenceProducesBitwiseIdenticalOutcomes)
{
  Config config;
  const std::vector<Eigen::Vector3d> positions = {
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.05, 0.01, 0.0),
    Eigen::Vector3d(1.5, 0.0, 0.0),
    Eigen::Vector3d(1.55, 0.01, 0.0),
    Eigen::Vector3d(1.6, 0.02, 0.0),
  };

  State state_a;
  State state_b;
  for (size_t i = 0; i < positions.size(); ++i) {
    const double stamp_sec = 1.0 + 0.1 * static_cast<double>(i);
    const double dt = (i == 0) ? 0.0 : 0.1;
    const Outcome a = evaluate(config, state_a,
      makeInput(positions[i], stamp_sec, dt, identityQuat()));
    const Outcome b = evaluate(config, state_b,
      makeInput(positions[i], stamp_sec, dt, identityQuat()));
    EXPECT_EQ(
      0,
      std::memcmp(
        a.accepted_position.data(), b.accepted_position.data(), sizeof(double) * 3));
    EXPECT_EQ(a.warn_lines, b.warn_lines);
    EXPECT_EQ(a.suppress_map_update, b.suppress_map_update);
  }
  EXPECT_EQ(state_a.tracking_state, state_b.tracking_state);
  EXPECT_EQ(
    0,
    std::memcmp(&state_a.accepted_fitness_ema, &state_b.accepted_fitness_ema, sizeof(double)));
}
