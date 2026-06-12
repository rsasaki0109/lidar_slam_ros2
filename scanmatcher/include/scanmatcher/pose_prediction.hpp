#ifndef SCANMATCHER_POSE_PREDICTION_HPP_
#define SCANMATCHER_POSE_PREDICTION_HPP_

#include <algorithm>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace graphslam
{
namespace pose_prediction
{

// Mirror of the ScanMatcherComponent IMU pose-prediction parameters
// (same names and defaults as the component members).
struct ImuPredictionConfig
{
  bool use_imu {false};
  bool imu_pose_prediction_enable {true};
  double imu_pose_prediction_weight {0.0};
  double imu_pose_prediction_max_age {0.2};
  double imu_pose_prediction_max_roll_pitch_deg {12.0};
  double imu_pose_prediction_max_yaw_deg {20.0};
};

// Snapshot of the IMU state the shell holds when a cloud arrives.
// imu_age_sec is std::abs((cloud_stamp - latest_imu_stamp).seconds()),
// computed by the shell because clock handling stays a shell concern.
struct ImuObservation
{
  bool latest_imu_orientation_valid {false};
  bool cloud_imu_reference_valid {false};
  tf2::Quaternion cloud_imu_reference_quat {0.0, 0.0, 0.0, 1.0};
  tf2::Quaternion latest_imu_robot_quat {0.0, 0.0, 0.0, 1.0};
  double imu_age_sec {0.0};
};

// Constant velocity motion model: predict the next pose from the last
// frame-to-frame delta. Translation only — rotation prediction tends to
// amplify NDT oscillation.
inline Eigen::Matrix4f applyConstantVelocityPrediction(
  Eigen::Matrix4f sim_trans,
  const bool use_constant_velocity_model,
  const bool last_accepted_delta_valid,
  const bool tracking,
  const Eigen::Vector3d & last_accepted_delta_position)
{
  if (use_constant_velocity_model && last_accepted_delta_valid && tracking) {
    sim_trans.block<3, 1>(0, 3) += last_accepted_delta_position.cast<float>();
  }
  return sim_trans;
}

// Always-on IMU roll/pitch correction (gravity-constrained axes only, no
// yaw): rotate the initial guess by the clamped roll/pitch part of the IMU
// delta since the reference cloud.
inline Eigen::Matrix4f applyImuRollPitchCorrection(
  Eigen::Matrix4f sim_trans,
  const ImuPredictionConfig & config,
  const ImuObservation & imu,
  const tf2::Quaternion & current_orientation)
{
  if (
    config.use_imu && config.imu_pose_prediction_enable &&
    imu.latest_imu_orientation_valid && imu.cloud_imu_reference_valid &&
    config.imu_pose_prediction_weight > 0.0)
  {
    if (imu.imu_age_sec <= config.imu_pose_prediction_max_age) {
      tf2::Quaternion imu_delta = imu.cloud_imu_reference_quat.inverse() *
        imu.latest_imu_robot_quat;
      imu_delta.normalize();
      double imu_dr, imu_dp, imu_dy;
      tf2::Matrix3x3(imu_delta).getRPY(imu_dr, imu_dp, imu_dy);
      const double max_rp = config.imu_pose_prediction_weight * M_PI / 180.0;
      imu_dr = std::clamp(imu_dr, -max_rp, max_rp);
      imu_dp = std::clamp(imu_dp, -max_rp, max_rp);
      tf2::Quaternion rp_delta;
      rp_delta.setRPY(imu_dr, imu_dp, 0.0);
      rp_delta.normalize();
      tf2::Quaternion corrected = current_orientation * rp_delta;
      corrected.normalize();
      Eigen::Quaterniond corrected_eig(corrected.w(), corrected.x(), corrected.y(), corrected.z());
      sim_trans.block<3, 3>(0, 0) = corrected_eig.toRotationMatrix().cast<float>();
    }
  }
  return sim_trans;
}

// State-gated full IMU prediction (Suspect/Recovery only): replace the
// rotation of the initial guess with the IMU-predicted orientation, with
// roll/pitch and yaw clamped independently. state_gate_active is
// (tracking_state != Tracking || recovery_target_active).
inline Eigen::Matrix4f applyStateGatedImuPrediction(
  Eigen::Matrix4f sim_trans,
  const ImuPredictionConfig & config,
  const ImuObservation & imu,
  const tf2::Quaternion & current_orientation,
  const bool state_gate_active)
{
  if (
    config.use_imu && config.imu_pose_prediction_enable &&
    imu.latest_imu_orientation_valid && imu.cloud_imu_reference_valid &&
    state_gate_active)
  {
    if (imu.imu_age_sec <= config.imu_pose_prediction_max_age) {
      tf2::Quaternion imu_delta = imu.cloud_imu_reference_quat.inverse() *
        imu.latest_imu_robot_quat;
      imu_delta.normalize();
      double imu_delta_roll = 0.0;
      double imu_delta_pitch = 0.0;
      double imu_delta_yaw = 0.0;
      tf2::Matrix3x3(imu_delta).getRPY(imu_delta_roll, imu_delta_pitch, imu_delta_yaw);
      const double max_roll_pitch = config.imu_pose_prediction_max_roll_pitch_deg * M_PI / 180.0;
      const double max_yaw = config.imu_pose_prediction_max_yaw_deg * M_PI / 180.0;
      imu_delta_roll = std::clamp(imu_delta_roll, -max_roll_pitch, max_roll_pitch);
      imu_delta_pitch = std::clamp(imu_delta_pitch, -max_roll_pitch, max_roll_pitch);
      imu_delta_yaw = std::clamp(imu_delta_yaw, -max_yaw, max_yaw);
      tf2::Quaternion imu_delta_clamped;
      imu_delta_clamped.setRPY(imu_delta_roll, imu_delta_pitch, imu_delta_yaw);
      imu_delta_clamped.normalize();

      tf2::Quaternion predicted_quat = current_orientation * imu_delta_clamped;
      predicted_quat.normalize();
      Eigen::Quaterniond predicted_quat_eig(
        predicted_quat.w(), predicted_quat.x(), predicted_quat.y(), predicted_quat.z());
      sim_trans.block<3, 3>(0, 0) =
        predicted_quat_eig.normalized().toRotationMatrix().cast<float>();
    }
  }
  return sim_trans;
}

// IMU-predicted rotation as Euler angles matching NDT's internal convention
// (XYZ intrinsic), used as the NDT rotation-prior mean. Gating (enable
// flags, weight, age, registration method) stays with the shell.
inline Eigen::Vector3d imuPredictedRotationRpy(
  const ImuObservation & imu,
  const tf2::Quaternion & current_orientation)
{
  tf2::Quaternion imu_delta = imu.cloud_imu_reference_quat.inverse() * imu.latest_imu_robot_quat;
  imu_delta.normalize();
  tf2::Quaternion predicted_quat = current_orientation * imu_delta;
  predicted_quat.normalize();
  Eigen::Quaterniond pred_eig(predicted_quat.w(), predicted_quat.x(),
    predicted_quat.y(), predicted_quat.z());
  return pred_eig.toRotationMatrix().eulerAngles(0, 1, 2);
}

}  // namespace pose_prediction
}  // namespace graphslam

#endif  // SCANMATCHER_POSE_PREDICTION_HPP_
