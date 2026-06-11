#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <vector>

#include "scanmatcher/imu_processing.hpp"

namespace
{
using graphslam::imu_processing::ComplementaryBlendInput;
using graphslam::imu_processing::ImuVectors;
using graphslam::imu_processing::OrientationInput;
using graphslam::imu_processing::OrientationResult;
using graphslam::imu_processing::OrientationState;

tf2::Quaternion quatFromRpy(const double roll, const double pitch, const double yaw)
{
  tf2::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);
  quat.normalize();
  return quat;
}

OrientationInput makeValidOrientationInput(const tf2::Quaternion & q_world_imu)
{
  OrientationInput input;
  input.q_world_imu = q_world_imu;
  input.orientation_covariance0 = 0.01;
  input.imu_time = 10.0;
  return input;
}

}  // namespace

TEST(ImuProcessing, RotateVectorsIdentityQuatIsNoop)
{
  ImuVectors imu_vectors;
  imu_vectors.angular_velocity = tf2::Vector3(0.1, -0.2, 0.3);
  imu_vectors.linear_acceleration = tf2::Vector3(1.0, 2.0, 9.81);

  const ImuVectors robot_vectors = graphslam::imu_processing::rotateImuVectorsIntoRobotFrame(
    imu_vectors, tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

  EXPECT_DOUBLE_EQ(robot_vectors.angular_velocity.x(), 0.1);
  EXPECT_DOUBLE_EQ(robot_vectors.angular_velocity.y(), -0.2);
  EXPECT_DOUBLE_EQ(robot_vectors.angular_velocity.z(), 0.3);
  EXPECT_DOUBLE_EQ(robot_vectors.linear_acceleration.x(), 1.0);
  EXPECT_DOUBLE_EQ(robot_vectors.linear_acceleration.y(), 2.0);
  EXPECT_DOUBLE_EQ(robot_vectors.linear_acceleration.z(), 9.81);
}

TEST(ImuProcessing, RotateVectorsMatchesHistoricalQuatRotate)
{
  // 90 degree yaw: x maps to y. Expectation computed with the exact
  // tf2::quatRotate call the historical inline block used.
  const tf2::Quaternion q_robot_imu = quatFromRpy(0.0, 0.0, M_PI / 2.0);
  ImuVectors imu_vectors;
  imu_vectors.angular_velocity = tf2::Vector3(0.4, 0.0, 0.0);
  imu_vectors.linear_acceleration = tf2::Vector3(2.0, 0.5, -1.0);

  const ImuVectors robot_vectors = graphslam::imu_processing::rotateImuVectorsIntoRobotFrame(
    imu_vectors, q_robot_imu);

  const tf2::Vector3 expected_w = tf2::quatRotate(q_robot_imu, imu_vectors.angular_velocity);
  const tf2::Vector3 expected_a = tf2::quatRotate(q_robot_imu, imu_vectors.linear_acceleration);
  EXPECT_DOUBLE_EQ(robot_vectors.angular_velocity.x(), expected_w.x());
  EXPECT_DOUBLE_EQ(robot_vectors.angular_velocity.y(), expected_w.y());
  EXPECT_DOUBLE_EQ(robot_vectors.angular_velocity.z(), expected_w.z());
  EXPECT_DOUBLE_EQ(robot_vectors.linear_acceleration.x(), expected_a.x());
  EXPECT_DOUBLE_EQ(robot_vectors.linear_acceleration.y(), expected_a.y());
  EXPECT_DOUBLE_EQ(robot_vectors.linear_acceleration.z(), expected_a.z());
  EXPECT_NEAR(robot_vectors.angular_velocity.y(), 0.4, 1e-12);
}

TEST(ImuProcessing, ValidOrientationSameFrameUsesImuQuat)
{
  OrientationState state;
  const tf2::Quaternion q_world_imu = quatFromRpy(0.05, -0.02, 1.2);
  OrientationInput input = makeValidOrientationInput(q_world_imu);

  const OrientationResult result = graphslam::imu_processing::resolveOrientation(input, state);

  double expected_roll, expected_pitch, expected_yaw;
  tf2::Matrix3x3(q_world_imu).getRPY(expected_roll, expected_pitch, expected_yaw);
  EXPECT_DOUBLE_EQ(result.roll, expected_roll);
  EXPECT_DOUBLE_EQ(result.pitch, expected_pitch);
  EXPECT_DOUBLE_EQ(result.yaw, expected_yaw);
  EXPECT_DOUBLE_EQ(result.q_world_robot.x(), q_world_imu.x());
  EXPECT_DOUBLE_EQ(result.q_world_robot.w(), q_world_imu.w());
  // A valid orientation re-seeds the integrated yaw.
  EXPECT_TRUE(state.integrated_yaw_valid);
  EXPECT_DOUBLE_EQ(state.integrated_yaw, expected_yaw);
  EXPECT_DOUBLE_EQ(state.last_imu_time, 10.0);
}

TEST(ImuProcessing, ValidOrientationComposesFrameChange)
{
  OrientationState state;
  const tf2::Quaternion q_world_imu = quatFromRpy(0.0, 0.0, 1.0);
  const tf2::Quaternion q_robot_imu = quatFromRpy(0.0, 0.0, M_PI / 2.0);

  OrientationInput input = makeValidOrientationInput(q_world_imu);
  input.have_imu_tf = true;
  input.frame_differs = true;
  input.q_robot_imu = q_robot_imu;

  const OrientationResult result = graphslam::imu_processing::resolveOrientation(input, state);

  // q_world_robot = q_world_imu * q_imu_robot (historical composition).
  const tf2::Quaternion expected = q_world_imu * q_robot_imu.inverse();
  EXPECT_DOUBLE_EQ(result.q_world_robot.x(), expected.x());
  EXPECT_DOUBLE_EQ(result.q_world_robot.y(), expected.y());
  EXPECT_DOUBLE_EQ(result.q_world_robot.z(), expected.z());
  EXPECT_DOUBLE_EQ(result.q_world_robot.w(), expected.w());

  // Without a usable TF the composition is skipped even if frames differ.
  OrientationState state_no_tf;
  OrientationInput input_no_tf = makeValidOrientationInput(q_world_imu);
  input_no_tf.have_imu_tf = false;
  input_no_tf.frame_differs = true;
  input_no_tf.q_robot_imu = q_robot_imu;
  const OrientationResult result_no_tf = graphslam::imu_processing::resolveOrientation(
    input_no_tf, state_no_tf);
  EXPECT_DOUBLE_EQ(result_no_tf.q_world_robot.x(), q_world_imu.x());
  EXPECT_DOUBLE_EQ(result_no_tf.q_world_robot.w(), q_world_imu.w());
}

TEST(ImuProcessing, NegativeCovarianceFallsBackToAccel)
{
  OrientationState state;
  state.integrated_yaw = 0.5;
  state.integrated_yaw_valid = true;
  state.last_imu_time = 9.9;

  OrientationInput input;
  input.q_world_imu = quatFromRpy(0.3, 0.2, 0.1);  // present but marked invalid
  input.orientation_covariance0 = -1.0;
  input.linear_acceleration_x = 0.5;
  input.linear_acceleration_y = 0.3;
  input.linear_acceleration_z = 9.7;
  input.angular_velocity_z = 0.2;
  input.imu_time = 10.0;

  const OrientationResult result = graphslam::imu_processing::resolveOrientation(input, state);

  const double expected_roll = std::atan2(0.3, 9.7);
  const double expected_pitch = std::atan2(-0.5, std::sqrt(0.3 * 0.3 + 9.7 * 9.7));
  const double expected_yaw = 0.5 + 0.2 * (10.0 - 9.9);
  EXPECT_DOUBLE_EQ(result.roll, expected_roll);
  EXPECT_DOUBLE_EQ(result.pitch, expected_pitch);
  EXPECT_DOUBLE_EQ(result.yaw, expected_yaw);
  EXPECT_DOUBLE_EQ(state.integrated_yaw, expected_yaw);

  const tf2::Quaternion expected_quat = quatFromRpy(expected_roll, expected_pitch, expected_yaw);
  EXPECT_NEAR(result.q_world_robot.x(), expected_quat.x(), 1e-15);
  EXPECT_NEAR(result.q_world_robot.w(), expected_quat.w(), 1e-15);
}

TEST(ImuProcessing, ZeroQuaternionFallsBackToAccel)
{
  OrientationState state;
  OrientationInput input;
  input.q_world_imu = tf2::Quaternion(0.0, 0.0, 0.0, 0.0);
  input.orientation_covariance0 = 0.0;
  input.linear_acceleration_z = 9.81;
  input.imu_time = 1.0;

  const OrientationResult result = graphslam::imu_processing::resolveOrientation(input, state);

  EXPECT_DOUBLE_EQ(result.roll, 0.0);
  EXPECT_DOUBLE_EQ(result.pitch, 0.0);
  // First invalid sample initializes the integrated yaw to zero.
  EXPECT_DOUBLE_EQ(result.yaw, 0.0);
  EXPECT_TRUE(state.integrated_yaw_valid);
}

TEST(ImuProcessing, YawIntegrationGatesOnDt)
{
  // dt <= 0: the gyro is not integrated, yaw holds.
  OrientationState state;
  state.integrated_yaw = 0.7;
  state.integrated_yaw_valid = true;
  state.last_imu_time = 10.0;

  OrientationInput input;
  input.q_world_imu = tf2::Quaternion(0.0, 0.0, 0.0, 0.0);
  input.angular_velocity_z = 1.0;
  input.linear_acceleration_z = 9.81;
  input.imu_time = 10.0;
  graphslam::imu_processing::resolveOrientation(input, state);
  EXPECT_DOUBLE_EQ(state.integrated_yaw, 0.7);

  // dt >= 1.0 (message gap): the gyro is not integrated either.
  state.last_imu_time = 10.0;
  input.imu_time = 11.5;
  graphslam::imu_processing::resolveOrientation(input, state);
  EXPECT_DOUBLE_EQ(state.integrated_yaw, 0.7);
  EXPECT_DOUBLE_EQ(state.last_imu_time, 11.5);

  // Healthy dt integrates.
  input.imu_time = 11.6;
  graphslam::imu_processing::resolveOrientation(input, state);
  EXPECT_NEAR(state.integrated_yaw, 0.7 + 1.0 * 0.1, 1e-12);
}

TEST(ImuProcessing, YawWrapsAroundPi)
{
  OrientationState state;
  state.integrated_yaw = M_PI - 0.05;
  state.integrated_yaw_valid = true;
  state.last_imu_time = 0.0;

  OrientationInput input;
  input.q_world_imu = tf2::Quaternion(0.0, 0.0, 0.0, 0.0);
  input.angular_velocity_z = 1.0;
  input.linear_acceleration_z = 9.81;
  input.imu_time = 0.1;

  const OrientationResult result = graphslam::imu_processing::resolveOrientation(input, state);

  EXPECT_NEAR(result.yaw, M_PI - 0.05 + 0.1 - 2.0 * M_PI, 1e-12);
  EXPECT_LE(result.yaw, M_PI);
}

TEST(ImuProcessing, GravityCompensationMatchesTransplantedExpressions)
{
  const double ax = 0.4;
  const double ay = -0.2;
  const double az = 9.5;
  const double roll = 0.07;
  const double pitch = -0.03;

  const Eigen::Vector3f acc = graphslam::imu_processing::gravityCompensatedAcceleration(
    ax, ay, az, roll, pitch);

  // Exact float truncation of the historical inline expressions.
  const float expected_x = static_cast<float>(ax) + sin(pitch) * 9.81;
  const float expected_y = static_cast<float>(ay) - cos(pitch) * sin(roll) * 9.81;
  const float expected_z = static_cast<float>(az) - cos(pitch) * cos(roll) * 9.81;
  EXPECT_EQ(acc.x(), expected_x);
  EXPECT_EQ(acc.y(), expected_y);
  EXPECT_EQ(acc.z(), expected_z);

  // Level pose with nominal gravity cancels to (numerically) zero z.
  const Eigen::Vector3f level = graphslam::imu_processing::gravityCompensatedAcceleration(
    0.0, 0.0, 9.81, 0.0, 0.0);
  EXPECT_NEAR(level.z(), 0.0f, 1e-6f);
}

TEST(ImuProcessing, ComplementaryBlendMatchesTransplantedFormula)
{
  ComplementaryBlendInput input;
  input.imu_reference_quat = quatFromRpy(0.01, 0.02, 0.5);
  input.latest_imu_robot_quat = quatFromRpy(0.05, -0.01, 0.52);
  const tf2::Quaternion ndt_quat = quatFromRpy(0.02, 0.01, 0.55);
  input.accepted_quat_msg = tf2::toMsg(ndt_quat);
  const Eigen::Quaternionf prev_quat(
    Eigen::AngleAxisf(0.48f, Eigen::Vector3f::UnitZ()) *
    Eigen::AngleAxisf(0.015f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(0.025f, Eigen::Vector3f::UnitX()));
  input.previous_published_rotation = prev_quat.toRotationMatrix();
  input.alpha = 0.3;

  const geometry_msgs::msg::Quaternion blended =
    graphslam::imu_processing::blendComplementaryRollPitch(input);

  // Reproduce the historical arithmetic step by step.
  tf2::Quaternion imu_delta = input.imu_reference_quat.inverse() * input.latest_imu_robot_quat;
  imu_delta.normalize();
  double imu_dr, imu_dp, imu_dy;
  tf2::Matrix3x3(imu_delta).getRPY(imu_dr, imu_dp, imu_dy);
  double ndt_roll, ndt_pitch, ndt_yaw;
  tf2::Matrix3x3(ndt_quat).getRPY(ndt_roll, ndt_pitch, ndt_yaw);
  Eigen::Quaternionf prev_q_eig(input.previous_published_rotation);
  tf2::Quaternion prev_pub_quat(prev_q_eig.x(), prev_q_eig.y(), prev_q_eig.z(), prev_q_eig.w());
  double prev_roll, prev_pitch, prev_yaw;
  tf2::Matrix3x3(prev_pub_quat).getRPY(prev_roll, prev_pitch, prev_yaw);
  const double a = input.alpha;
  tf2::Quaternion expected_quat;
  expected_quat.setRPY(
    (1.0 - a) * ndt_roll + a * (prev_roll + imu_dr),
    (1.0 - a) * ndt_pitch + a * (prev_pitch + imu_dp),
    ndt_yaw);
  expected_quat.normalize();
  const geometry_msgs::msg::Quaternion expected = tf2::toMsg(expected_quat);

  EXPECT_DOUBLE_EQ(blended.x, expected.x);
  EXPECT_DOUBLE_EQ(blended.y, expected.y);
  EXPECT_DOUBLE_EQ(blended.z, expected.z);
  EXPECT_DOUBLE_EQ(blended.w, expected.w);
}

TEST(ImuProcessing, ComplementaryBlendZeroAlphaKeepsRegistrationRotation)
{
  ComplementaryBlendInput input;
  input.imu_reference_quat = quatFromRpy(0.0, 0.0, 0.0);
  input.latest_imu_robot_quat = quatFromRpy(0.3, 0.2, 0.0);
  const tf2::Quaternion ndt_quat = quatFromRpy(0.02, 0.01, 0.55);
  input.accepted_quat_msg = tf2::toMsg(ndt_quat);
  input.previous_published_rotation = Eigen::Matrix3f::Identity();
  input.alpha = 0.0;

  const geometry_msgs::msg::Quaternion blended =
    graphslam::imu_processing::blendComplementaryRollPitch(input);

  double ndt_roll, ndt_pitch, ndt_yaw;
  tf2::Matrix3x3(ndt_quat).getRPY(ndt_roll, ndt_pitch, ndt_yaw);
  tf2::Quaternion expected_quat;
  expected_quat.setRPY(ndt_roll, ndt_pitch, ndt_yaw);
  expected_quat.normalize();
  const geometry_msgs::msg::Quaternion expected = tf2::toMsg(expected_quat);
  EXPECT_DOUBLE_EQ(blended.x, expected.x);
  EXPECT_DOUBLE_EQ(blended.y, expected.y);
  EXPECT_DOUBLE_EQ(blended.z, expected.z);
  EXPECT_DOUBLE_EQ(blended.w, expected.w);
}

TEST(ImuProcessing, SameSequenceBitwiseIdentical)
{
  const auto run_sequence = [](std::vector<OrientationResult> & results) {
      OrientationState state;
      for (int i = 0; i < 50; ++i) {
        OrientationInput input;
        if (i % 3 == 0) {
          input.q_world_imu = quatFromRpy(0.01 * i, -0.005 * i, 0.1 * i);
          input.orientation_covariance0 = 0.01;
        } else {
          input.q_world_imu = tf2::Quaternion(0.0, 0.0, 0.0, 0.0);
          input.orientation_covariance0 = -1.0;
        }
        input.linear_acceleration_x = 0.01 * i;
        input.linear_acceleration_y = -0.02 * i;
        input.linear_acceleration_z = 9.81;
        input.angular_velocity_z = 0.05 * (i % 7);
        input.imu_time = 0.1 * i;
        results.push_back(graphslam::imu_processing::resolveOrientation(input, state));
      }
    };

  std::vector<OrientationResult> first;
  std::vector<OrientationResult> second;
  run_sequence(first);
  run_sequence(second);

  ASSERT_EQ(first.size(), second.size());
  for (size_t i = 0; i < first.size(); ++i) {
    const double lhs[4] =
    {first[i].q_world_robot.x(), first[i].q_world_robot.y(), first[i].q_world_robot.z(),
      first[i].q_world_robot.w()};
    const double rhs[4] =
    {second[i].q_world_robot.x(), second[i].q_world_robot.y(), second[i].q_world_robot.z(),
      second[i].q_world_robot.w()};
    EXPECT_EQ(std::memcmp(lhs, rhs, sizeof(lhs)), 0);
    EXPECT_EQ(
      std::memcmp(&first[i].roll, &second[i].roll, sizeof(double)), 0);
    EXPECT_EQ(
      std::memcmp(&first[i].yaw, &second[i].yaw, sizeof(double)), 0);
  }
}
