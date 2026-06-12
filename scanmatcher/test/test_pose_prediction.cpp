#include <gtest/gtest.h>

#include <cmath>
#include <cstring>

#include "scanmatcher/pose_prediction.hpp"

namespace
{
using graphslam::pose_prediction::ImuObservation;
using graphslam::pose_prediction::ImuPredictionConfig;

Eigen::Matrix4f makePose(
  const Eigen::Vector3f & translation,
  const float roll_rad,
  const float pitch_rad,
  const float yaw_rad)
{
  Eigen::AngleAxisf roll_axis(roll_rad, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitch_axis(pitch_rad, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf yaw_axis(yaw_rad, Eigen::Vector3f::UnitZ());
  const Eigen::Matrix3f rotation = (yaw_axis * pitch_axis * roll_axis).toRotationMatrix();

  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  pose.block<3, 3>(0, 0) = rotation;
  pose.block<3, 1>(0, 3) = translation;
  return pose;
}

tf2::Quaternion quatFromRpy(const double roll, const double pitch, const double yaw)
{
  tf2::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);
  quat.normalize();
  return quat;
}

ImuPredictionConfig makeEnabledConfig()
{
  ImuPredictionConfig config;
  config.use_imu = true;
  config.imu_pose_prediction_enable = true;
  config.imu_pose_prediction_weight = 5.0;
  config.imu_pose_prediction_max_age = 0.2;
  config.imu_pose_prediction_max_roll_pitch_deg = 12.0;
  config.imu_pose_prediction_max_yaw_deg = 20.0;
  return config;
}

ImuObservation makeValidObservation(
  const tf2::Quaternion & reference_quat,
  const tf2::Quaternion & latest_quat,
  const double imu_age_sec)
{
  ImuObservation imu;
  imu.latest_imu_orientation_valid = true;
  imu.cloud_imu_reference_valid = true;
  imu.cloud_imu_reference_quat = reference_quat;
  imu.latest_imu_robot_quat = latest_quat;
  imu.imu_age_sec = imu_age_sec;
  return imu;
}
}  // namespace

TEST(PosePredictionConstantVelocityTest, AddsTranslationOnlyWhenAllGatesPass)
{
  const Eigen::Matrix4f pose = makePose(Eigen::Vector3f(1.0f, 2.0f, 3.0f), 0.1f, 0.2f, 0.3f);
  const Eigen::Vector3d delta(0.5, -0.25, 0.125);

  const Eigen::Matrix4f predicted =
    graphslam::pose_prediction::applyConstantVelocityPrediction(pose, true, true, true, delta);

  Eigen::Matrix4f expected = pose;
  expected.block<3, 1>(0, 3) += delta.cast<float>();
  EXPECT_TRUE(predicted.isApprox(expected, 0.0f));
  // Rotation untouched (translation-only model).
  EXPECT_TRUE((
      predicted.block<3, 3>(0, 0).isApprox(pose.block<3, 3>(0, 0), 0.0f)));
}

TEST(PosePredictionConstantVelocityTest, AnyFailedGateLeavesThePoseUnchanged)
{
  const Eigen::Matrix4f pose = makePose(Eigen::Vector3f(1.0f, 2.0f, 3.0f), 0.1f, 0.2f, 0.3f);
  const Eigen::Vector3d delta(0.5, -0.25, 0.125);

  EXPECT_TRUE(
    graphslam::pose_prediction::applyConstantVelocityPrediction(pose, false, true, true, delta)
    .isApprox(pose, 0.0f));
  EXPECT_TRUE(
    graphslam::pose_prediction::applyConstantVelocityPrediction(pose, true, false, true, delta)
    .isApprox(pose, 0.0f));
  EXPECT_TRUE(
    graphslam::pose_prediction::applyConstantVelocityPrediction(pose, true, true, false, delta)
    .isApprox(pose, 0.0f));
}

TEST(PosePredictionRollPitchTest, GatesDisableTheCorrection)
{
  const Eigen::Matrix4f pose = makePose(Eigen::Vector3f(1.0f, 0.0f, 0.0f), 0.0f, 0.0f, 0.5f);
  const tf2::Quaternion current = quatFromRpy(0.0, 0.0, 0.5);
  const ImuObservation imu =
    makeValidObservation(quatFromRpy(0.0, 0.0, 0.0), quatFromRpy(0.05, -0.03, 0.4), 0.05);

  ImuPredictionConfig config = makeEnabledConfig();
  config.imu_pose_prediction_weight = 0.0;
  EXPECT_TRUE(
    graphslam::pose_prediction::applyImuRollPitchCorrection(pose, config, imu, current)
    .isApprox(pose, 0.0f));

  config = makeEnabledConfig();
  config.use_imu = false;
  EXPECT_TRUE(
    graphslam::pose_prediction::applyImuRollPitchCorrection(pose, config, imu, current)
    .isApprox(pose, 0.0f));

  config = makeEnabledConfig();
  ImuObservation stale = imu;
  stale.imu_age_sec = config.imu_pose_prediction_max_age + 0.01;
  EXPECT_TRUE(
    graphslam::pose_prediction::applyImuRollPitchCorrection(pose, config, stale, current)
    .isApprox(pose, 0.0f));
}

TEST(PosePredictionRollPitchTest, AppliesClampedRollPitchAndIgnoresYaw)
{
  const Eigen::Matrix4f pose = makePose(Eigen::Vector3f(1.0f, 2.0f, 3.0f), 0.0f, 0.0f, 0.5f);
  const tf2::Quaternion current = quatFromRpy(0.0, 0.0, 0.5);
  // IMU delta: roll well over the +/- weight-degree cap, plus a yaw component
  // that the gravity-constrained correction must ignore.
  const ImuObservation imu =
    makeValidObservation(quatFromRpy(0.0, 0.0, 0.0), quatFromRpy(0.5, -0.02, 0.4), 0.05);
  const ImuPredictionConfig config = makeEnabledConfig();

  const Eigen::Matrix4f corrected =
    graphslam::pose_prediction::applyImuRollPitchCorrection(pose, config, imu, current);

  // Translation preserved verbatim.
  EXPECT_TRUE((corrected.block<3, 1>(0, 3).isApprox(pose.block<3, 1>(0, 3), 0.0f)));

  // Expected rotation: transplanted original formula.
  tf2::Quaternion imu_delta = imu.cloud_imu_reference_quat.inverse() * imu.latest_imu_robot_quat;
  imu_delta.normalize();
  double imu_dr, imu_dp, imu_dy;
  tf2::Matrix3x3(imu_delta).getRPY(imu_dr, imu_dp, imu_dy);
  const double max_rp = config.imu_pose_prediction_weight * M_PI / 180.0;
  imu_dr = std::clamp(imu_dr, -max_rp, max_rp);
  imu_dp = std::clamp(imu_dp, -max_rp, max_rp);
  EXPECT_DOUBLE_EQ(imu_dr, max_rp);  // the 0.5 rad roll delta hits the cap
  tf2::Quaternion rp_delta;
  rp_delta.setRPY(imu_dr, imu_dp, 0.0);
  rp_delta.normalize();
  tf2::Quaternion expected_quat = current * rp_delta;
  expected_quat.normalize();
  const Eigen::Quaterniond expected_eig(
    expected_quat.w(), expected_quat.x(), expected_quat.y(), expected_quat.z());
  EXPECT_TRUE((
      corrected.block<3, 3>(0, 0).isApprox(
      expected_eig.toRotationMatrix().cast<float>(), 0.0f)));
}

TEST(PosePredictionStateGatedTest, OnlyRunsWhenTheStateGateIsActive)
{
  const Eigen::Matrix4f pose = makePose(Eigen::Vector3f(0.0f, 0.0f, 0.0f), 0.0f, 0.0f, 0.0f);
  const tf2::Quaternion current = quatFromRpy(0.0, 0.0, 0.0);
  const ImuObservation imu =
    makeValidObservation(quatFromRpy(0.0, 0.0, 0.0), quatFromRpy(0.1, 0.05, 0.2), 0.05);
  const ImuPredictionConfig config = makeEnabledConfig();

  EXPECT_TRUE(
    graphslam::pose_prediction::applyStateGatedImuPrediction(pose, config, imu, current, false)
    .isApprox(pose, 0.0f));
  EXPECT_FALSE(
    graphslam::pose_prediction::applyStateGatedImuPrediction(pose, config, imu, current, true)
    .isApprox(pose, 0.0f));
}

TEST(PosePredictionStateGatedTest, ClampsRollPitchAndYawIndependently)
{
  const Eigen::Matrix4f pose = makePose(Eigen::Vector3f(1.0f, -2.0f, 0.5f), 0.0f, 0.0f, 1.0f);
  const tf2::Quaternion current = quatFromRpy(0.0, 0.0, 1.0);
  // Deltas beyond both caps: 30 deg roll (cap 12) and 45 deg yaw (cap 20).
  const ImuObservation imu = makeValidObservation(
    quatFromRpy(0.0, 0.0, 0.0),
    quatFromRpy(30.0 * M_PI / 180.0, 0.0, 45.0 * M_PI / 180.0),
    0.05);
  const ImuPredictionConfig config = makeEnabledConfig();

  const Eigen::Matrix4f predicted =
    graphslam::pose_prediction::applyStateGatedImuPrediction(pose, config, imu, current, true);

  // Translation preserved verbatim.
  EXPECT_TRUE((predicted.block<3, 1>(0, 3).isApprox(pose.block<3, 1>(0, 3), 0.0f)));

  // Expected rotation: transplanted original formula.
  tf2::Quaternion imu_delta = imu.cloud_imu_reference_quat.inverse() * imu.latest_imu_robot_quat;
  imu_delta.normalize();
  double dr = 0.0, dp = 0.0, dy = 0.0;
  tf2::Matrix3x3(imu_delta).getRPY(dr, dp, dy);
  const double max_rp = config.imu_pose_prediction_max_roll_pitch_deg * M_PI / 180.0;
  const double max_yaw = config.imu_pose_prediction_max_yaw_deg * M_PI / 180.0;
  dr = std::clamp(dr, -max_rp, max_rp);
  dp = std::clamp(dp, -max_rp, max_rp);
  dy = std::clamp(dy, -max_yaw, max_yaw);
  EXPECT_DOUBLE_EQ(dr, max_rp);
  EXPECT_DOUBLE_EQ(dy, max_yaw);
  tf2::Quaternion clamped;
  clamped.setRPY(dr, dp, dy);
  clamped.normalize();
  tf2::Quaternion expected_quat = current * clamped;
  expected_quat.normalize();
  const Eigen::Quaterniond expected_eig(
    expected_quat.w(), expected_quat.x(), expected_quat.y(), expected_quat.z());
  EXPECT_TRUE((
      predicted.block<3, 3>(0, 0).isApprox(
      expected_eig.normalized().toRotationMatrix().cast<float>(), 0.0f)));
}

TEST(PosePredictionNdtPriorTest, IdentityDeltaReturnsCurrentOrientationAngles)
{
  const tf2::Quaternion current = quatFromRpy(0.1, -0.2, 0.3);
  const ImuObservation imu =
    makeValidObservation(quatFromRpy(0.0, 0.0, 0.0), quatFromRpy(0.0, 0.0, 0.0), 0.0);

  const Eigen::Vector3d rpy =
    graphslam::pose_prediction::imuPredictedRotationRpy(imu, current);

  const Eigen::Quaterniond current_eig(current.w(), current.x(), current.y(), current.z());
  const Eigen::Vector3d expected = current_eig.toRotationMatrix().eulerAngles(0, 1, 2);
  EXPECT_DOUBLE_EQ(rpy.x(), expected.x());
  EXPECT_DOUBLE_EQ(rpy.y(), expected.y());
  EXPECT_DOUBLE_EQ(rpy.z(), expected.z());
}

TEST(PosePredictionDeterminismTest, SameInputsProduceBitwiseIdenticalResults)
{
  const Eigen::Matrix4f pose = makePose(Eigen::Vector3f(1.0f, 2.0f, 3.0f), 0.1f, -0.2f, 0.3f);
  const tf2::Quaternion current = quatFromRpy(0.1, -0.2, 0.3);
  const ImuObservation imu =
    makeValidObservation(quatFromRpy(0.01, 0.02, 0.03), quatFromRpy(0.05, -0.04, 0.2), 0.1);
  const ImuPredictionConfig config = makeEnabledConfig();

  const Eigen::Matrix4f first = graphslam::pose_prediction::applyStateGatedImuPrediction(
    graphslam::pose_prediction::applyImuRollPitchCorrection(
      graphslam::pose_prediction::applyConstantVelocityPrediction(
        pose, true, true, true, Eigen::Vector3d(0.5, -0.25, 0.125)),
      config, imu, current),
    config, imu, current, true);
  const Eigen::Matrix4f second = graphslam::pose_prediction::applyStateGatedImuPrediction(
    graphslam::pose_prediction::applyImuRollPitchCorrection(
      graphslam::pose_prediction::applyConstantVelocityPrediction(
        pose, true, true, true, Eigen::Vector3d(0.5, -0.25, 0.125)),
      config, imu, current),
    config, imu, current, true);

  EXPECT_EQ(0, std::memcmp(first.data(), second.data(), sizeof(float) * 16));
}
