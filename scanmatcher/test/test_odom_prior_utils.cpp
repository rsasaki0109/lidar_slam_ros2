#include <gtest/gtest.h>

#include "scanmatcher/odom_prior_utils.hpp"

namespace
{
Eigen::Matrix4f makeDelta(
  const Eigen::Vector3f & translation,
  const float roll_rad,
  const float pitch_rad,
  const float yaw_rad)
{
  Eigen::AngleAxisf roll_axis(roll_rad, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitch_axis(pitch_rad, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf yaw_axis(yaw_rad, Eigen::Vector3f::UnitZ());
  const Eigen::Matrix3f rotation = (yaw_axis * pitch_axis * roll_axis).toRotationMatrix();

  Eigen::Matrix4f delta = Eigen::Matrix4f::Identity();
  delta.block<3, 3>(0, 0) = rotation;
  delta.block<3, 1>(0, 3) = translation;
  return delta;
}

float extractYawRad(const Eigen::Matrix4f & transform)
{
  return std::atan2(transform(1, 0), transform(0, 0));
}
}  // namespace

TEST(OdomPriorUtilsTest, WeightZeroReturnsIdentity)
{
  const Eigen::Matrix4f delta = makeDelta(Eigen::Vector3f(1.0f, 2.0f, 3.0f), 0.1f, 0.2f, 0.3f);
  const Eigen::Matrix4f filtered =
    graphslam::odom_prior::filterAndBlendDelta(delta, false, false, 0.0);

  EXPECT_TRUE(filtered.isApprox(Eigen::Matrix4f::Identity(), 1e-6f));
}

TEST(OdomPriorUtilsTest, PlanarFilterRemovesZRollAndPitch)
{
  const Eigen::Matrix4f delta = makeDelta(Eigen::Vector3f(4.0f, 5.0f, 6.0f), 0.2f, -0.3f, 0.6f);
  const Eigen::Matrix4f filtered =
    graphslam::odom_prior::filterAndBlendDelta(delta, true, false, 1.0);

  EXPECT_NEAR(filtered(2, 3), 0.0f, 1e-6f);
  EXPECT_NEAR(extractYawRad(filtered), 0.6f, 1e-5f);
  EXPECT_NEAR(filtered(2, 0), 0.0f, 1e-6f);
  EXPECT_NEAR(filtered(2, 1), 0.0f, 1e-6f);
}

TEST(OdomPriorUtilsTest, TranslationOnlyKeepsIdentityRotation)
{
  const Eigen::Matrix4f delta = makeDelta(Eigen::Vector3f(3.0f, -1.0f, 2.0f), 0.4f, 0.1f, 0.7f);
  const Eigen::Matrix4f filtered =
    graphslam::odom_prior::filterAndBlendDelta(delta, false, true, 1.0);

  EXPECT_TRUE((filtered.block<3, 3>(0, 0).isApprox(Eigen::Matrix3f::Identity(), 1e-6f)));
  EXPECT_NEAR(filtered(0, 3), 3.0f, 1e-6f);
  EXPECT_NEAR(filtered(1, 3), -1.0f, 1e-6f);
  EXPECT_NEAR(filtered(2, 3), 2.0f, 1e-6f);
}

TEST(OdomPriorUtilsTest, WeightBlendsTranslationAndYaw)
{
  const Eigen::Matrix4f delta = makeDelta(Eigen::Vector3f(8.0f, 0.0f, 0.0f), 0.0f, 0.0f, 1.2f);
  const Eigen::Matrix4f filtered =
    graphslam::odom_prior::filterAndBlendDelta(delta, true, false, 0.25);

  EXPECT_NEAR(filtered(0, 3), 2.0f, 1e-6f);
  EXPECT_NEAR(filtered(1, 3), 0.0f, 1e-6f);
  EXPECT_NEAR(extractYawRad(filtered), 0.3f, 1e-4f);
}
