#ifndef SCANMATCHER_IMU_PROCESSING_HPP_
#define SCANMATCHER_IMU_PROCESSING_HPP_

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/quaternion.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace graphslam
{
namespace imu_processing
{

inline double wrapAngleRad(double angle)
{
  while (angle > M_PI) {angle -= 2.0 * M_PI;}
  while (angle < -M_PI) {angle += 2.0 * M_PI;}
  return angle;
}

// Angular velocity / linear acceleration pair expressed in some frame.
struct ImuVectors
{
  tf2::Vector3 angular_velocity {0.0, 0.0, 0.0};
  tf2::Vector3 linear_acceleration {0.0, 0.0, 0.0};
};

// Rotate IMU angular velocity and linear acceleration from the IMU frame
// into the robot frame using the robot<-imu rotation. This is the body of
// the historical duplicated try/fallback blocks in receiveImu.
inline ImuVectors rotateImuVectorsIntoRobotFrame(
  const ImuVectors & imu_vectors,
  const tf2::Quaternion & q_robot_imu)
{
  ImuVectors robot_vectors;
  robot_vectors.angular_velocity = tf2::quatRotate(q_robot_imu, imu_vectors.angular_velocity);
  robot_vectors.linear_acceleration =
    tf2::quatRotate(q_robot_imu, imu_vectors.linear_acceleration);
  return robot_vectors;
}

// Yaw-integration state owned by the shell across IMU messages.
struct OrientationState
{
  double last_imu_time {0.0};
  double integrated_yaw {0.0};
  bool integrated_yaw_valid {false};
};

// Per-message inputs for the orientation resolution. The linear
// acceleration and angular velocity are already expressed in the robot
// frame (rotateImuVectorsIntoRobotFrame ran first when needed).
struct OrientationInput
{
  tf2::Quaternion q_world_imu {0.0, 0.0, 0.0, 1.0};
  // msg.orientation_covariance[0]; negative marks the orientation invalid
  // per REP-145 (the historical .empty() guard on the std::array is always
  // false, so the < 0.0 test is the only live condition).
  double orientation_covariance0 {0.0};
  double linear_acceleration_x {0.0};
  double linear_acceleration_y {0.0};
  double linear_acceleration_z {0.0};
  double angular_velocity_z {0.0};
  double imu_time {0.0};
  bool have_imu_tf {false};
  bool frame_differs {false};
  tf2::Quaternion q_robot_imu {0.0, 0.0, 0.0, 1.0};
};

struct OrientationResult
{
  tf2::Quaternion q_world_robot {0.0, 0.0, 0.0, 1.0};
  double roll {0.0};
  double pitch {0.0};
  double yaw {0.0};
};

// Resolve the robot orientation for one IMU message: use the reported
// orientation when valid (composing the IMU->robot frame change), else
// estimate roll/pitch from gravity and integrate yaw from the gyro.
// Verbatim transplant of the receiveImu orientation block.
inline OrientationResult resolveOrientation(
  const OrientationInput & input,
  OrientationState & state)
{
  bool orientation_valid = true;
  if (input.orientation_covariance0 < 0.0) {
    orientation_valid = false;
  }
  if (input.q_world_imu.length2() < 1e-12) {
    orientation_valid = false;
  }

  OrientationResult result;
  if (orientation_valid) {
    if (input.have_imu_tf && input.frame_differs) {
      // q_world_robot = q_world_imu * q_imu_robot
      const tf2::Quaternion q_imu_robot = input.q_robot_imu.inverse();
      result.q_world_robot = input.q_world_imu * q_imu_robot;
    } else {
      result.q_world_robot = input.q_world_imu;
    }
    tf2::Matrix3x3(result.q_world_robot).getRPY(result.roll, result.pitch, result.yaw);
    state.integrated_yaw = result.yaw;
    state.integrated_yaw_valid = true;
  } else {
    const double ax = input.linear_acceleration_x;
    const double ay = input.linear_acceleration_y;
    const double az = input.linear_acceleration_z;
    result.roll = std::atan2(ay, az);
    result.pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));
    const double dt = input.imu_time - state.last_imu_time;
    if (state.integrated_yaw_valid && dt > 0.0 && dt < 1.0) {
      state.integrated_yaw = wrapAngleRad(
        state.integrated_yaw + input.angular_velocity_z * dt);
    } else if (!state.integrated_yaw_valid) {
      state.integrated_yaw = 0.0;
      state.integrated_yaw_valid = true;
    }
    result.yaw = state.integrated_yaw;
    result.q_world_robot.setRPY(result.roll, result.pitch, result.yaw);
  }
  state.last_imu_time = input.imu_time;
  return result;
}

// Remove the gravity component from a robot-frame acceleration using the
// resolved roll/pitch. Exact expressions from receiveImu (double trig,
// float truncation at assignment).
inline Eigen::Vector3f gravityCompensatedAcceleration(
  const double linear_acceleration_x,
  const double linear_acceleration_y,
  const double linear_acceleration_z,
  const double roll,
  const double pitch)
{
  float acc_x = static_cast<float>(linear_acceleration_x) + sin(pitch) * 9.81;
  float acc_y = static_cast<float>(linear_acceleration_y) - cos(pitch) * sin(roll) * 9.81;
  float acc_z = static_cast<float>(linear_acceleration_z) - cos(pitch) * cos(roll) * 9.81;
  return Eigen::Vector3f{acc_x, acc_y, acc_z};
}

// Inputs for the post-registration complementary roll/pitch blend.
// Gating (enable flag, alpha > 0, IMU validity/age, diagnostic validity)
// stays with the shell; this is only the blend arithmetic.
struct ComplementaryBlendInput
{
  // cloud_imu_reference_quat_: IMU orientation captured at the previous cloud
  tf2::Quaternion imu_reference_quat {0.0, 0.0, 0.0, 1.0};
  tf2::Quaternion latest_imu_robot_quat {0.0, 0.0, 0.0, 1.0};
  // Accepted registration orientation for this frame
  geometry_msgs::msg::Quaternion accepted_quat_msg;
  // Rotation block of ndt_pose_ (last published orientation)
  Eigen::Matrix3f previous_published_rotation {Eigen::Matrix3f::Identity()};
  double alpha {0.0};
};

// Post-NDT complementary filter: blend roll/pitch with the IMU-predicted
// orientation for output only; yaw stays with the registration result.
// Verbatim transplant of the publishMapAndPose blend block.
inline geometry_msgs::msg::Quaternion blendComplementaryRollPitch(
  const ComplementaryBlendInput & input)
{
  tf2::Quaternion imu_delta = input.imu_reference_quat.inverse() * input.latest_imu_robot_quat;
  imu_delta.normalize();
  double imu_dr, imu_dp, imu_dy;
  tf2::Matrix3x3(imu_delta).getRPY(imu_dr, imu_dp, imu_dy);

  tf2::Quaternion ndt_quat;
  tf2::fromMsg(input.accepted_quat_msg, ndt_quat);
  double ndt_roll, ndt_pitch, ndt_yaw;
  tf2::Matrix3x3(ndt_quat).getRPY(ndt_roll, ndt_pitch, ndt_yaw);

  // Previous published rotation (ndt_pose_ stores last published RPY)
  Eigen::Quaternionf prev_q_eig(input.previous_published_rotation);
  tf2::Quaternion prev_pub_quat(prev_q_eig.x(), prev_q_eig.y(), prev_q_eig.z(), prev_q_eig.w());
  double prev_roll, prev_pitch, prev_yaw;
  tf2::Matrix3x3(prev_pub_quat).getRPY(prev_roll, prev_pitch, prev_yaw);

  double imu_pred_roll = prev_roll + imu_dr;
  double imu_pred_pitch = prev_pitch + imu_dp;

  const double a = input.alpha;
  double blended_roll = (1.0 - a) * ndt_roll + a * imu_pred_roll;
  double blended_pitch = (1.0 - a) * ndt_pitch + a * imu_pred_pitch;

  tf2::Quaternion blended_quat;
  blended_quat.setRPY(blended_roll, blended_pitch, ndt_yaw);
  blended_quat.normalize();
  return tf2::toMsg(blended_quat);
}

}  // namespace imu_processing
}  // namespace graphslam

#endif  // SCANMATCHER_IMU_PROCESSING_HPP_
