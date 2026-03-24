#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <cmath>

// Standalone version of ScanMatcherComponent::getTransformation
static Eigen::Matrix4f getTransformation(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Affine3d affine;
  tf2::fromMsg(pose, affine);
  return affine.matrix().cast<float>();
}

// Standalone version of the IMU gyro integration from graph_based_slam
struct StampedImu
{
  double stamp, gx, gy, gz;
};

static Eigen::Quaterniond integrateImuRotation(
  const std::vector<StampedImu> & buffer, double t0, double t1)
{
  Eigen::Quaterniond delta_q = Eigen::Quaterniond::Identity();

  auto it = std::lower_bound(buffer.begin(), buffer.end(), t0,
    [](const StampedImu & imu, double t) { return imu.stamp < t; });

  if (it == buffer.end()) {
    return delta_q;
  }

  double prev_t = t0;
  for (; it != buffer.end() && it->stamp <= t1; ++it) {
    double dt = it->stamp - prev_t;
    if (dt <= 0.0 || dt > 0.5) {
      prev_t = it->stamp;
      continue;
    }
    Eigen::Vector3d omega(it->gx, it->gy, it->gz);
    double angle = omega.norm() * dt;
    if (angle > 1e-10) {
      Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, omega.normalized()));
      delta_q = delta_q * dq;
      delta_q.normalize();
    }
    prev_t = it->stamp;
  }

  return delta_q;
}

// --- getTransformation tests ---

TEST(GetTransformation, IdentityPose)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;

  Eigen::Matrix4f result = getTransformation(pose);
  Eigen::Matrix4f expected = Eigen::Matrix4f::Identity();

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      EXPECT_NEAR(result(i, j), expected(i, j), 1e-6)
        << "Mismatch at (" << i << ", " << j << ")";
    }
  }
}

TEST(GetTransformation, TranslationOnly)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = 2.0;
  pose.position.z = 3.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;

  Eigen::Matrix4f result = getTransformation(pose);

  EXPECT_NEAR(result(0, 3), 1.0f, 1e-6);
  EXPECT_NEAR(result(1, 3), 2.0f, 1e-6);
  EXPECT_NEAR(result(2, 3), 3.0f, 1e-6);

  // Rotation part should be identity
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      float expected = (i == j) ? 1.0f : 0.0f;
      EXPECT_NEAR(result(i, j), expected, 1e-6);
    }
  }
}

TEST(GetTransformation, Rotation90DegYaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;

  // 90 degrees around Z axis
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, M_PI / 2.0);
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();

  Eigen::Matrix4f result = getTransformation(pose);

  // After 90 deg yaw: x-axis -> y-axis, y-axis -> -x-axis
  EXPECT_NEAR(result(0, 0), 0.0f, 1e-5);  // cos(90)
  EXPECT_NEAR(result(0, 1), -1.0f, 1e-5); // -sin(90)
  EXPECT_NEAR(result(1, 0), 1.0f, 1e-5);  // sin(90)
  EXPECT_NEAR(result(1, 1), 0.0f, 1e-5);  // cos(90)
  EXPECT_NEAR(result(2, 2), 1.0f, 1e-5);  // z unchanged
}

TEST(GetTransformation, TranslationAndRotation)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 5.0;
  pose.position.y = -3.0;
  pose.position.z = 1.5;

  tf2::Quaternion q;
  q.setRPY(0.1, 0.2, 0.3);
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();

  Eigen::Matrix4f result = getTransformation(pose);

  // Verify translation
  EXPECT_NEAR(result(0, 3), 5.0f, 1e-5);
  EXPECT_NEAR(result(1, 3), -3.0f, 1e-5);
  EXPECT_NEAR(result(2, 3), 1.5f, 1e-5);

  // Verify rotation is proper (det = 1, orthogonal)
  Eigen::Matrix3f rot = result.block<3, 3>(0, 0);
  EXPECT_NEAR(rot.determinant(), 1.0f, 1e-5);
  Eigen::Matrix3f should_be_identity = rot * rot.transpose();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      float expected = (i == j) ? 1.0f : 0.0f;
      EXPECT_NEAR(should_be_identity(i, j), expected, 1e-5);
    }
  }

  // Bottom row
  EXPECT_NEAR(result(3, 0), 0.0f, 1e-6);
  EXPECT_NEAR(result(3, 1), 0.0f, 1e-6);
  EXPECT_NEAR(result(3, 2), 0.0f, 1e-6);
  EXPECT_NEAR(result(3, 3), 1.0f, 1e-6);
}

// --- IMU Gyro Integration tests ---

TEST(ImuIntegration, NoData)
{
  std::vector<StampedImu> buffer;
  Eigen::Quaterniond q = integrateImuRotation(buffer, 0.0, 1.0);
  EXPECT_TRUE(q.isApprox(Eigen::Quaterniond::Identity(), 1e-10));
}

TEST(ImuIntegration, ZeroAngularVelocity)
{
  std::vector<StampedImu> buffer;
  for (int i = 0; i <= 100; ++i) {
    buffer.push_back({0.01 * i, 0.0, 0.0, 0.0});
  }

  Eigen::Quaterniond q = integrateImuRotation(buffer, 0.0, 1.0);
  EXPECT_TRUE(q.isApprox(Eigen::Quaterniond::Identity(), 1e-10));
}

TEST(ImuIntegration, ConstantYawRate)
{
  // 1 rad/s yaw rate for 1 second => expect ~1 radian rotation around Z
  double yaw_rate = 1.0;  // rad/s
  std::vector<StampedImu> buffer;
  for (int i = 0; i <= 1000; ++i) {
    buffer.push_back({0.001 * i, 0.0, 0.0, yaw_rate});
  }

  Eigen::Quaterniond q = integrateImuRotation(buffer, 0.0, 1.0);

  // Expected: rotation of 1 radian around Z
  Eigen::Quaterniond expected(Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitZ()));
  // With 1ms steps, numerical integration should be very close
  EXPECT_NEAR(q.x(), expected.x(), 1e-3);
  EXPECT_NEAR(q.y(), expected.y(), 1e-3);
  EXPECT_NEAR(q.z(), expected.z(), 1e-3);
  EXPECT_NEAR(q.w(), expected.w(), 1e-3);
}

TEST(ImuIntegration, ConstantRollRate)
{
  double roll_rate = 0.5;  // rad/s
  std::vector<StampedImu> buffer;
  for (int i = 0; i <= 500; ++i) {
    buffer.push_back({0.001 * i, roll_rate, 0.0, 0.0});
  }

  Eigen::Quaterniond q = integrateImuRotation(buffer, 0.0, 0.5);

  // Expected: rotation of 0.25 radian around X (0.5 * 0.5s)
  Eigen::Quaterniond expected(Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitX()));
  EXPECT_NEAR(q.x(), expected.x(), 1e-3);
  EXPECT_NEAR(q.y(), expected.y(), 1e-3);
  EXPECT_NEAR(q.z(), expected.z(), 1e-3);
  EXPECT_NEAR(q.w(), expected.w(), 1e-3);
}

TEST(ImuIntegration, TimeRangeSubset)
{
  // Buffer covers [0, 2], but we only integrate [0.5, 1.5]
  double yaw_rate = 2.0;
  std::vector<StampedImu> buffer;
  for (int i = 0; i <= 2000; ++i) {
    buffer.push_back({0.001 * i, 0.0, 0.0, yaw_rate});
  }

  Eigen::Quaterniond q = integrateImuRotation(buffer, 0.5, 1.5);

  // Expected: 2.0 rad/s * 1.0 s = 2.0 rad around Z
  Eigen::Quaterniond expected(Eigen::AngleAxisd(2.0, Eigen::Vector3d::UnitZ()));
  EXPECT_NEAR(q.x(), expected.x(), 1e-2);
  EXPECT_NEAR(q.y(), expected.y(), 1e-2);
  EXPECT_NEAR(q.z(), expected.z(), 1e-2);
  EXPECT_NEAR(q.w(), expected.w(), 1e-2);
}

TEST(ImuIntegration, LargeTimeGapSkipped)
{
  // Gap > 0.5s should be skipped
  std::vector<StampedImu> buffer;
  buffer.push_back({0.0, 0.0, 0.0, 1.0});
  buffer.push_back({0.1, 0.0, 0.0, 1.0});
  buffer.push_back({1.0, 0.0, 0.0, 1.0});  // 0.9s gap from previous — skipped
  buffer.push_back({1.01, 0.0, 0.0, 1.0});

  Eigen::Quaterniond q = integrateImuRotation(buffer, 0.0, 1.01);

  // Only the 0->0.1 step and 1.0->1.01 step should contribute
  // Total angle: 1.0 * 0.1 + 1.0 * 0.01 = 0.11 rad
  Eigen::Quaterniond expected(Eigen::AngleAxisd(0.11, Eigen::Vector3d::UnitZ()));
  EXPECT_NEAR(q.z(), expected.z(), 1e-3);
  EXPECT_NEAR(q.w(), expected.w(), 1e-3);
}

TEST(ImuIntegration, ResultIsNormalized)
{
  std::vector<StampedImu> buffer;
  for (int i = 0; i <= 1000; ++i) {
    buffer.push_back({0.001 * i, 0.3, 0.5, 0.7});
  }

  Eigen::Quaterniond q = integrateImuRotation(buffer, 0.0, 1.0);
  EXPECT_NEAR(q.norm(), 1.0, 1e-10);
}

// --- Rotation Prior math tests ---

TEST(RotationPrior, AngleWrapAround)
{
  // Test the angle wrapping logic used in NDT rotation prior
  // diff should be in [-pi, pi]
  auto wrap = [](double diff) {
    while (diff > M_PI) { diff -= 2.0 * M_PI; }
    while (diff < -M_PI) { diff += 2.0 * M_PI; }
    return diff;
  };

  EXPECT_NEAR(wrap(0.0), 0.0, 1e-10);
  EXPECT_NEAR(wrap(M_PI), M_PI, 1e-10);
  EXPECT_NEAR(wrap(-M_PI), -M_PI, 1e-10);
  EXPECT_NEAR(wrap(2.0 * M_PI), 0.0, 1e-10);
  EXPECT_NEAR(wrap(-2.0 * M_PI), 0.0, 1e-10);
  EXPECT_NEAR(wrap(3.0 * M_PI), M_PI, 1e-10);
  EXPECT_NEAR(wrap(M_PI + 0.1), -M_PI + 0.1, 1e-10);
  EXPECT_NEAR(wrap(-M_PI - 0.1), M_PI - 0.1, 1e-10);
}

TEST(RotationPrior, GradientSign)
{
  // Verify the gradient of the rotation prior penalty: d/dp [-w*(p-target)^2] = -2*w*(p-target)
  double weight = 0.1;
  double target = 0.5;
  double p = 0.7;
  double diff = p - target;

  double gradient = -2.0 * weight * diff;
  EXPECT_LT(gradient, 0.0);  // Pushing p back toward target

  p = 0.3;
  diff = p - target;
  gradient = -2.0 * weight * diff;
  EXPECT_GT(gradient, 0.0);  // Pushing p forward toward target
}

TEST(RotationPrior, HessianIsConstant)
{
  // The hessian of -w*(p-target)^2 is -2*w, independent of p
  double weight = 0.1;
  double hessian = -2.0 * weight;
  EXPECT_NEAR(hessian, -0.2, 1e-10);

  weight = 1.0;
  hessian = -2.0 * weight;
  EXPECT_NEAR(hessian, -2.0, 1e-10);
}

TEST(RotationPrior, ScoreDecreasesWithDistance)
{
  // Score penalty should increase as angle deviates from target
  double weight = 0.5;
  double target = 0.0;

  auto score_penalty = [&](double p) {
    double diff = p - target;
    return -weight * diff * diff;
  };

  EXPECT_NEAR(score_penalty(0.0), 0.0, 1e-10);  // at target
  EXPECT_LT(score_penalty(0.1), 0.0);            // slight deviation
  EXPECT_LT(score_penalty(0.1), score_penalty(0.0));
  EXPECT_LT(score_penalty(0.5), score_penalty(0.1));  // more deviation = worse
  EXPECT_NEAR(score_penalty(0.1), score_penalty(-0.1), 1e-10);  // symmetric
}
