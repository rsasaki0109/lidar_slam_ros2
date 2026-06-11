#ifndef SCANMATCHER_POSE_ACCEPTANCE_HPP_
#define SCANMATCHER_POSE_ACCEPTANCE_HPP_

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/quaternion.hpp>

namespace graphslam
{
namespace pose_acceptance
{

enum class TrackingState
{
  Tracking,
  Suspect,
  Recovery
};

// Mirror of the ScanMatcherComponent pose-acceptance parameters (same names
// and defaults as the component members).
struct Config
{
  double diagnostic_warn_trans_jump {0.75};
  double diagnostic_warn_yaw_jump_deg {12.0};
  bool reject_nonconverged_pose_update {true};
  double reject_fitness_score {0.0};
  double reject_fitness_ratio {2.5};
  double reject_fitness_only_ratio {8.0};
  double reject_trans_only_ratio {0.0};
  int reject_trans_streak_scans {0};
  double reject_fitness_streak_ratio {0.0};
  int reject_fitness_streak_scans {0};
  double reject_hard_fitness_ratio {0.0};
  double reject_trans_jump {1.0};
  double reject_trans_jump_ratio {3.0};
  double reject_hard_trans_ratio {0.0};
  double reject_ema_alpha {0.1};
  bool motion_gate_enable {true};
  double motion_gate_max_linear_velocity {8.0};
  double motion_gate_max_yaw_rate_deg {120.0};
  double motion_gate_hard_multiplier {4.0};
  int reject_warmup_scans {20};
  int reject_map_update_cooldown_scans {2};
  int hard_reject_map_update_cooldown_scans {4};
  int reject_recovery_scans {0};
  int recovery_clear_consecutive_accepted {1};
  int suspect_clear_consecutive_accepted {2};
  double scan_period {0.1};
  std::string robot_frame_id {"base_link"};
};

// Mutable acceptance state, previously scattered across component members.
// The pose stamp itself stays in the shell (clock handling): the shell
// passes dt and the stamps as seconds.
struct State
{
  bool previous_pose_diagnostic_valid {false};
  Eigen::Vector3d previous_pose_diagnostic_position {Eigen::Vector3d::Zero()};
  double previous_pose_diagnostic_yaw {0.0};
  bool reject_stats_initialized {false};
  double accepted_fitness_ema {0.0};
  double accepted_trans_ema {0.0};
  int accepted_pose_count {0};
  int elevated_trans_streak {0};
  int elevated_fitness_streak {0};
  int consecutive_reject_count {0};
  int state_clean_consecutive_accepted {0};
  TrackingState tracking_state {TrackingState::Tracking};
  bool recovery_target_active {false};
  bool last_accepted_delta_valid {false};
  Eigen::Vector3d last_accepted_delta_position {Eigen::Vector3d::Zero()};
  tf2::Quaternion last_accepted_delta_quat {0.0, 0.0, 0.0, 1.0};
  int reject_map_update_cooldown_remaining {0};
};

// One aligned scan as seen by the acceptance logic.
struct Input
{
  Eigen::Vector3d position {Eigen::Vector3d::Zero()};
  geometry_msgs::msg::Quaternion orientation {};
  // Orientation of the previous accepted pose (current_pose_stamped before
  // this scan updates it).
  geometry_msgs::msg::Quaternion previous_orientation {};
  bool has_converged {true};
  double fitness_score {0.0};
  double stamp_sec {0.0};
  double previous_stamp_sec {0.0};
  // (stamp - previous_pose_stamp).seconds(), 0.0 on the first diagnostic
  // scan; computed by the shell from the rclcpp::Time pair so the rounding
  // matches the historical Duration::seconds() exactly.
  double dt {0.0};
  // True while the asynchronous map-update thread is running.
  bool mapping_in_progress {false};
};

struct Outcome
{
  Eigen::Vector3d accepted_position {Eigen::Vector3d::Zero()};
  geometry_msgs::msg::Quaternion accepted_quat_msg {};
  bool hard_reject_pose_update {false};
  bool soft_reject_map_update {false};
  bool suppress_map_update {false};
  // When true the shell must refresh the recovery registration target and
  // store the result in State::recovery_target_active (the decision is
  // pure, the PCL/registration side effect is not).
  bool request_recovery_target_refresh {false};
  // RCLCPP_WARN lines, byte-identical to the historical component output.
  std::vector<std::string> warn_lines;
};

inline double wrapAngleRad(double angle)
{
  while (angle > M_PI) {angle -= 2.0 * M_PI;}
  while (angle < -M_PI) {angle += 2.0 * M_PI;}
  return angle;
}

inline Eigen::Vector3d clampVectorNorm(const Eigen::Vector3d & v, double max_norm)
{
  if (max_norm <= 0.0) {
    return v;
  }
  const double norm = v.norm();
  if (norm <= max_norm || norm < 1e-9) {
    return v;
  }
  return v * (max_norm / norm);
}

inline geometry_msgs::msg::Quaternion quaternionFromRPY(double roll, double pitch, double yaw)
{
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  q.normalize();
  return tf2::toMsg(q);
}

inline std::string formatWarn(const char * fmt, ...)
{
  char buffer[1024];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  return std::string(buffer);
}

// Verbatim transplant of the publishMapAndPose acceptance block: pose-jump
// diagnostics, adaptive fitness/translation gates, motion gate, accept /
// clip / predict selection, accepted-statistics EMA, the
// Tracking/Suspect/Recovery state machine and the map-update cooldown.
inline Outcome evaluate(const Config & config, State & state, const Input & input)
{
  const Eigen::Vector3d & position = input.position;
  const geometry_msgs::msg::Quaternion & quat_msg = input.orientation;
  const bool has_converged = input.has_converged;
  const double fitness_score = input.fitness_score;
  tf2::Quaternion diag_quat_tf;
  double diag_roll, diag_pitch, diag_yaw;
  tf2::fromMsg(quat_msg, diag_quat_tf);
  tf2::Matrix3x3(diag_quat_tf).getRPY(diag_roll, diag_pitch, diag_yaw);
  double dt = 0.0;
  double trans_jump = 0.0;
  double yaw_jump_deg = 0.0;
  double fitness_ratio = 1.0;
  double trans_ratio = 1.0;
  double fitness_ref = fitness_score;
  double trans_ref = 0.0;
  bool warmup_complete = false;
  bool reject_pose_update = false;
  bool hard_reject_pose_update = false;
  bool soft_reject_map_update = false;
  bool adaptive_ratio_reject = false;
  bool fitness_only_map_reject = false;
  bool trans_only_map_reject = false;
  bool trans_streak_map_reject = false;
  bool fitness_streak_map_reject = false;
  bool hard_ratio_reject = false;
  bool motion_gate_suspect = false;
  bool motion_gate_hard = false;
  double motion_gate_trans_limit = 0.0;
  double motion_gate_yaw_limit_deg = 0.0;
  constexpr double kFitnessScoreSanityLimit = 1.0e4;
  const bool invalid_fitness_score =
    !std::isfinite(fitness_score) || fitness_score >= kFitnessScoreSanityLimit;

  Outcome outcome;

  if (state.previous_pose_diagnostic_valid) {
    dt = input.dt;
    trans_jump = (position - state.previous_pose_diagnostic_position).norm();
    yaw_jump_deg =
      std::abs(wrapAngleRad(diag_yaw - state.previous_pose_diagnostic_yaw)) * 180.0 / M_PI;

    if (dt <= 0.0) {
      outcome.warn_lines.push_back(
        formatWarn(
          "POSE_STAMP_NONMONOTONIC stamp=%.9f prev_stamp=%.9f dt=%.6f frame=%s",
          input.stamp_sec,
          input.previous_stamp_sec,
          dt,
          config.robot_frame_id.c_str()));
    }

    if (!has_converged || trans_jump >= config.diagnostic_warn_trans_jump ||
      yaw_jump_deg >= config.diagnostic_warn_yaw_jump_deg)
    {
      outcome.warn_lines.push_back(
        formatWarn(
          "POSE_JUMP stamp=%.9f dt=%.6f trans=%.6f yaw_deg=%.3f converged=%s fitness=%.6f frame=%s",
          input.stamp_sec,
          dt,
          trans_jump,
          yaw_jump_deg,
          has_converged ? "true" : "false",
          fitness_score,
          config.robot_frame_id.c_str()));
    }

    if (state.reject_stats_initialized) {
      fitness_ref = state.accepted_fitness_ema;
      trans_ref = state.accepted_trans_ema;
      if (
        !std::isfinite(fitness_ref) || fitness_ref <= 0.0 ||
        fitness_ref >= kFitnessScoreSanityLimit ||
        !std::isfinite(trans_ref) || trans_ref < 0.0)
      {
        state.reject_stats_initialized = false;
        state.accepted_fitness_ema = 0.0;
        state.accepted_trans_ema = 0.0;
        state.accepted_pose_count = 0;
        fitness_ref = fitness_score;
        trans_ref = trans_jump;
      }
    } else {
      fitness_ref = fitness_score;
      trans_ref = trans_jump;
    }

    const double fitness_ref_safe = (fitness_ref > 1e-6) ? fitness_ref : 1e-6;
    const double trans_ref_safe = (trans_ref > 1e-3) ? trans_ref : 1e-3;
    fitness_ratio = fitness_score / fitness_ref_safe;
    trans_ratio = trans_jump / trans_ref_safe;

    if (
      config.motion_gate_enable &&
      config.motion_gate_max_linear_velocity > 0.0 &&
      config.motion_gate_max_yaw_rate_deg > 0.0)
    {
      const double effective_dt = std::max(dt, config.scan_period);
      motion_gate_trans_limit = config.motion_gate_max_linear_velocity * effective_dt;
      motion_gate_yaw_limit_deg = config.motion_gate_max_yaw_rate_deg * effective_dt;
      motion_gate_suspect =
        trans_jump > motion_gate_trans_limit ||
        yaw_jump_deg > motion_gate_yaw_limit_deg;
      motion_gate_hard =
        config.motion_gate_hard_multiplier > 1.0 &&
        (
        trans_jump > motion_gate_trans_limit * config.motion_gate_hard_multiplier ||
        yaw_jump_deg > motion_gate_yaw_limit_deg * config.motion_gate_hard_multiplier);
    }

    warmup_complete = state.accepted_pose_count >= config.reject_warmup_scans;
    adaptive_ratio_reject =
      warmup_complete &&
      state.reject_stats_initialized &&
      config.reject_fitness_ratio > 0.0 &&
      config.reject_trans_jump_ratio > 0.0 &&
      fitness_ratio >= config.reject_fitness_ratio &&
      trans_ratio >= config.reject_trans_jump_ratio;
    fitness_only_map_reject =
      warmup_complete &&
      state.reject_stats_initialized &&
      config.reject_fitness_only_ratio > 0.0 &&
      fitness_ratio >= config.reject_fitness_only_ratio;
    trans_only_map_reject =
      warmup_complete &&
      state.reject_stats_initialized &&
      config.reject_trans_only_ratio > 0.0 &&
      trans_jump >= config.diagnostic_warn_trans_jump &&
      trans_ratio >= config.reject_trans_only_ratio;
    if (
      trans_jump >= config.diagnostic_warn_trans_jump &&
      trans_ratio >= config.reject_trans_jump_ratio)
    {
      state.elevated_trans_streak += 1;
    } else {
      state.elevated_trans_streak = 0;
    }
    trans_streak_map_reject =
      config.reject_trans_streak_scans > 0 &&
      state.elevated_trans_streak >= config.reject_trans_streak_scans;
    if (
      warmup_complete &&
      state.reject_stats_initialized &&
      config.reject_fitness_streak_ratio > 0.0 &&
      fitness_ratio >= config.reject_fitness_streak_ratio)
    {
      state.elevated_fitness_streak += 1;
    } else {
      state.elevated_fitness_streak = 0;
    }
    fitness_streak_map_reject =
      config.reject_fitness_streak_scans > 0 &&
      state.elevated_fitness_streak >= config.reject_fitness_streak_scans;
    hard_ratio_reject =
      warmup_complete &&
      state.reject_stats_initialized &&
      config.reject_hard_fitness_ratio > 0.0 &&
      config.reject_hard_trans_ratio > 0.0 &&
      fitness_ratio >= config.reject_hard_fitness_ratio &&
      trans_ratio >= config.reject_hard_trans_ratio;

    hard_reject_pose_update =
      invalid_fitness_score ||
      (config.reject_nonconverged_pose_update && !has_converged) ||
      (config.reject_fitness_score > 0.0 && fitness_score > config.reject_fitness_score) ||
      (
      !config.motion_gate_enable &&
      config.reject_trans_jump > 0.0 &&
      trans_jump >= config.reject_trans_jump) ||
      motion_gate_hard ||
      hard_ratio_reject;
    soft_reject_map_update =
      (adaptive_ratio_reject || fitness_only_map_reject || trans_only_map_reject ||
      trans_streak_map_reject || fitness_streak_map_reject || motion_gate_suspect) &&
      !hard_reject_pose_update;
    reject_pose_update = hard_reject_pose_update;
  }

  Eigen::Vector3d accepted_position = position;
  geometry_msgs::msg::Quaternion accepted_quat_msg = quat_msg;
  double accepted_yaw = diag_yaw;
  Eigen::Vector3d predicted_position = position;
  geometry_msgs::msg::Quaternion predicted_quat_msg = quat_msg;
  double predicted_yaw = diag_yaw;
  Eigen::Vector3d clipped_position = position;
  geometry_msgs::msg::Quaternion clipped_quat_msg = quat_msg;
  double clipped_yaw = diag_yaw;
  if (state.previous_pose_diagnostic_valid) {
    predicted_position = state.previous_pose_diagnostic_position;
    predicted_quat_msg = input.previous_orientation;
    predicted_yaw = state.previous_pose_diagnostic_yaw;
    if (state.last_accepted_delta_valid) {
      predicted_position += state.last_accepted_delta_position;
      tf2::Quaternion prev_quat_tf;
      tf2::Quaternion predicted_quat_tf;
      tf2::fromMsg(input.previous_orientation, prev_quat_tf);
      predicted_quat_tf = prev_quat_tf * state.last_accepted_delta_quat;
      predicted_quat_tf.normalize();
      predicted_quat_msg = tf2::toMsg(predicted_quat_tf);
      double predicted_roll;
      double predicted_pitch;
      tf2::Matrix3x3(predicted_quat_tf).getRPY(predicted_roll, predicted_pitch, predicted_yaw);
    }

    if (config.motion_gate_enable && motion_gate_trans_limit > 0.0 &&
      motion_gate_yaw_limit_deg > 0.0)
    {
      const Eigen::Vector3d candidate_delta = position - predicted_position;
      clipped_position =
        predicted_position + clampVectorNorm(candidate_delta, motion_gate_trans_limit);

      const double max_yaw_delta = motion_gate_yaw_limit_deg * M_PI / 180.0;
      const double candidate_yaw_delta = wrapAngleRad(diag_yaw - predicted_yaw);
      const double clipped_yaw_delta =
        std::clamp(candidate_yaw_delta, -max_yaw_delta, max_yaw_delta);
      clipped_yaw = predicted_yaw + clipped_yaw_delta;
      clipped_quat_msg = quaternionFromRPY(diag_roll, diag_pitch, clipped_yaw);
    }
  }
  if (state.previous_pose_diagnostic_valid && hard_reject_pose_update) {
    accepted_position = predicted_position;
    accepted_quat_msg = predicted_quat_msg;
    accepted_yaw = predicted_yaw;
  } else if (state.previous_pose_diagnostic_valid && soft_reject_map_update) {
    accepted_position = clipped_position;
    accepted_quat_msg = clipped_quat_msg;
    accepted_yaw = clipped_yaw;
  } else if (state.previous_pose_diagnostic_valid &&
    state.tracking_state == TrackingState::Recovery)
  {
    accepted_position = clipped_position;
    accepted_quat_msg = clipped_quat_msg;
    accepted_yaw = clipped_yaw;
  } else if (state.previous_pose_diagnostic_valid &&
    state.tracking_state == TrackingState::Suspect)
  {
    accepted_position = clipped_position;
    accepted_quat_msg = clipped_quat_msg;
    accepted_yaw = clipped_yaw;
  }
  if (state.previous_pose_diagnostic_valid &&
    (hard_reject_pose_update || soft_reject_map_update))
  {
    if (invalid_fitness_score) {
      outcome.warn_lines.push_back(
        formatWarn(
          "POSE_FITNESS_INVALID stamp=%.9f fitness=%.6f frame=%s",
          input.stamp_sec,
          fitness_score,
          config.robot_frame_id.c_str()));
    }
    outcome.warn_lines.push_back(
      formatWarn(
        "POSE_REJECT stamp=%.9f dt=%.6f trans=%.6f yaw_deg=%.3f converged=%s fitness=%.6f fitness_ref=%.6f fitness_ratio=%.3f trans_ref=%.6f trans_ratio=%.3f motion_gate=%s motion_gate_hard=%s trans_limit=%.3f yaw_limit_deg=%.3f adaptive=%s fitness_only=%s trans_only=%s trans_streak=%s fitness_streak=%s hard_ratio=%s streak_count=%d mode=%s cooldown=%d reject_nonconv=%s reject_fitness=%.3f reject_trans=%.3f frame=%s",
        input.stamp_sec,
        dt,
        trans_jump,
        yaw_jump_deg,
        has_converged ? "true" : "false",
        fitness_score,
        fitness_ref,
        fitness_ratio,
        trans_ref,
        trans_ratio,
        motion_gate_suspect ? "true" : "false",
        motion_gate_hard ? "true" : "false",
        motion_gate_trans_limit,
        motion_gate_yaw_limit_deg,
        adaptive_ratio_reject ? "true" : "false",
        fitness_only_map_reject ? "true" : "false",
        trans_only_map_reject ? "true" : "false",
        trans_streak_map_reject ? "true" : "false",
        fitness_streak_map_reject ? "true" : "false",
        hard_ratio_reject ? "true" : "false",
        state.elevated_fitness_streak,
        hard_reject_pose_update ? "hard" : "map_only",
        config.reject_map_update_cooldown_scans,
        config.reject_nonconverged_pose_update ? "true" : "false",
        config.reject_fitness_score,
        config.reject_trans_jump,
        config.robot_frame_id.c_str()));
  }

  if (!reject_pose_update && !soft_reject_map_update) {
    double alpha = config.reject_ema_alpha;
    if (alpha <= 0.0 || alpha > 1.0) {alpha = 0.1;}
    double fitness_sample = fitness_score;
    double trans_sample = trans_jump;
    if (!state.reject_stats_initialized) {
      state.accepted_fitness_ema = fitness_sample;
      state.accepted_trans_ema = trans_sample;
      state.reject_stats_initialized = true;
    } else {
      if (state.accepted_pose_count >= config.reject_warmup_scans) {
        if (config.reject_fitness_ratio > 0.0 && state.accepted_fitness_ema > 1e-6) {
          const double fitness_cap = state.accepted_fitness_ema * config.reject_fitness_ratio;
          if (fitness_sample > fitness_cap) {fitness_sample = fitness_cap;}
        }
        if (config.reject_trans_jump_ratio > 0.0 && state.accepted_trans_ema > 1e-3) {
          const double trans_cap = state.accepted_trans_ema * config.reject_trans_jump_ratio;
          if (trans_sample > trans_cap) {trans_sample = trans_cap;}
        }
      }
      state.accepted_fitness_ema =
        (1.0 - alpha) * state.accepted_fitness_ema + alpha * fitness_sample;
      state.accepted_trans_ema =
        (1.0 - alpha) * state.accepted_trans_ema + alpha * trans_sample;
    }
    state.accepted_pose_count += 1;
  }
  if (reject_pose_update || soft_reject_map_update) {
    state.consecutive_reject_count += 1;
  } else {
    state.consecutive_reject_count = 0;
  }
  if (hard_reject_pose_update) {
    state.last_accepted_delta_valid = false;
  }
  if (hard_reject_pose_update) {
    const bool activate_recovery_target =
      !input.mapping_in_progress &&
      (
      config.reject_recovery_scans <= 1 ||
      state.consecutive_reject_count >= config.reject_recovery_scans);
    if (activate_recovery_target) {
      outcome.request_recovery_target_refresh = true;
      state.consecutive_reject_count = 0;
      outcome.warn_lines.push_back(
        formatWarn(
          "POSE_REJECT_HARD_RECOVERY stamp=%.9f frame=%s",
          input.stamp_sec,
          config.robot_frame_id.c_str()));
    }
    state.reject_stats_initialized = false;
    state.accepted_fitness_ema = 0.0;
    state.accepted_trans_ema = 0.0;
    state.accepted_pose_count = 0;
    state.elevated_fitness_streak = 0;
    state.elevated_trans_streak = 0;
    state.state_clean_consecutive_accepted = 0;
    state.tracking_state = TrackingState::Recovery;
    outcome.warn_lines.push_back(
      formatWarn(
        "POSE_REJECT_HARD_RECOVERY stamp=%.9f frame=%s",
        input.stamp_sec,
        config.robot_frame_id.c_str()));
  }
  if (hard_reject_pose_update) {
    state.state_clean_consecutive_accepted = 0;
    if (state.tracking_state != TrackingState::Recovery) {
      state.tracking_state = TrackingState::Recovery;
      outcome.warn_lines.push_back(
        formatWarn("TRACKING_STATE recovery stamp=%.9f", input.stamp_sec));
    }
  } else if (soft_reject_map_update) {
    state.state_clean_consecutive_accepted = 0;
    if (state.tracking_state == TrackingState::Tracking) {
      state.tracking_state = TrackingState::Suspect;
      outcome.warn_lines.push_back(
        formatWarn("TRACKING_STATE suspect stamp=%.9f", input.stamp_sec));
    }
  } else if (state.tracking_state != TrackingState::Tracking) {
    state.state_clean_consecutive_accepted += 1;
    if (
      state.tracking_state == TrackingState::Recovery &&
      state.state_clean_consecutive_accepted >= config.recovery_clear_consecutive_accepted)
    {
      state.tracking_state = TrackingState::Suspect;
      state.state_clean_consecutive_accepted = 0;
      outcome.warn_lines.push_back(
        formatWarn("TRACKING_STATE suspect stamp=%.9f", input.stamp_sec));
    } else if (
      state.tracking_state == TrackingState::Suspect &&
      state.state_clean_consecutive_accepted >= config.suspect_clear_consecutive_accepted)
    {
      state.tracking_state = TrackingState::Tracking;
      state.state_clean_consecutive_accepted = 0;
      state.recovery_target_active = false;
      outcome.warn_lines.push_back(
        formatWarn("TRACKING_STATE tracking stamp=%.9f", input.stamp_sec));
    }
  }
  if (state.previous_pose_diagnostic_valid && !hard_reject_pose_update) {
    state.last_accepted_delta_position =
      accepted_position - state.previous_pose_diagnostic_position;
    tf2::Quaternion previous_quat_tf;
    tf2::Quaternion current_quat_tf;
    tf2::fromMsg(input.previous_orientation, previous_quat_tf);
    tf2::fromMsg(accepted_quat_msg, current_quat_tf);
    state.last_accepted_delta_quat = previous_quat_tf.inverse() * current_quat_tf;
    state.last_accepted_delta_quat.normalize();
    state.last_accepted_delta_valid = true;
  }
  state.previous_pose_diagnostic_position = accepted_position;
  state.previous_pose_diagnostic_yaw = accepted_yaw;
  state.previous_pose_diagnostic_valid = true;

  const bool reject_map_update_now = hard_reject_pose_update || soft_reject_map_update;
  const bool suppress_map_update =
    reject_map_update_now || state.reject_map_update_cooldown_remaining > 0 ||
    state.tracking_state != TrackingState::Tracking;
  if (hard_reject_pose_update) {
    state.reject_map_update_cooldown_remaining = config.hard_reject_map_update_cooldown_scans;
  } else if (soft_reject_map_update) {
    state.reject_map_update_cooldown_remaining = config.reject_map_update_cooldown_scans;
  } else if (state.reject_map_update_cooldown_remaining > 0) {
    state.reject_map_update_cooldown_remaining -= 1;
  }

  outcome.accepted_position = accepted_position;
  outcome.accepted_quat_msg = accepted_quat_msg;
  outcome.hard_reject_pose_update = hard_reject_pose_update;
  outcome.soft_reject_map_update = soft_reject_map_update;
  outcome.suppress_map_update = suppress_map_update;
  return outcome;
}

}  // namespace pose_acceptance
}  // namespace graphslam

#endif  // SCANMATCHER_POSE_ACCEPTANCE_HPP_
