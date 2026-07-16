#include <gtest/gtest.h>

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include "rko_lio/core/selective_visual_fusion.hpp"

namespace {

rko_lio::core::VisualConstraintConfidence valid_confidence() {
  return {120, 80, 1.0, 0.9, 0.2};
}

TEST(SelectiveVisualFusion, DisabledIsExactFallback) {
  const Eigen::Matrix<double, 6, 6> H =
      Eigen::Matrix<double, 6, 6>::Identity();
  const Eigen::Matrix<double, 6, 1> b =
      Eigen::Matrix<double, 6, 1>::Ones();
  const Eigen::Matrix<double, 6, 1> update =
      Eigen::Matrix<double, 6, 1>::Constant(0.01);
  const rko_lio::core::SelectiveVisualFusionConfig config;
  const auto result = rko_lio::core::fuse_visual_in_weak_directions(
      H, b, update, valid_confidence(), config);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.fused_directions, 0U);
  EXPECT_TRUE(result.H.isApprox(H, 0.0));
  EXPECT_TRUE(result.b.isApprox(b, 0.0));
}

TEST(SelectiveVisualFusion, RejectsLowConfidenceWithoutChangingSystem) {
  Eigen::Matrix<double, 6, 6> H =
      Eigen::Matrix<double, 6, 6>::Identity();
  H(0, 0) = 1.0e-8;
  const Eigen::Matrix<double, 6, 1> b =
      Eigen::Matrix<double, 6, 1>::Zero();
  rko_lio::core::SelectiveVisualFusionConfig config;
  config.enabled = true;
  auto confidence = valid_confidence();
  confidence.inliers = 10;
  const auto result = rko_lio::core::fuse_visual_in_weak_directions(
      H, b, Eigen::Matrix<double, 6, 1>::Ones(), confidence, config);
  EXPECT_FALSE(result.accepted);
  EXPECT_TRUE(result.H.isApprox(H, 0.0));
  EXPECT_TRUE(result.b.isApprox(b, 0.0));
}

TEST(SelectiveVisualFusion, AddsPriorOnlyOnWeakAxis) {
  Eigen::Matrix<double, 6, 6> H =
      Eigen::Matrix<double, 6, 6>::Identity();
  H(0, 0) = 1.0e-8;
  const Eigen::Matrix<double, 6, 1> b =
      Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> update =
      Eigen::Matrix<double, 6, 1>::Zero();
  update(0) = 0.02;
  update(1) = 0.02;
  rko_lio::core::SelectiveVisualFusionConfig config;
  config.enabled = true;
  config.max_weak_directions = 1;
  const auto result = rko_lio::core::fuse_visual_in_weak_directions(
      H, b, update, valid_confidence(), config);
  ASSERT_TRUE(result.accepted);
  EXPECT_EQ(result.fused_directions, 1U);
  EXPECT_GT(result.H(0, 0), H(0, 0));
  EXPECT_DOUBLE_EQ(result.H(1, 1), H(1, 1));
  const Eigen::Matrix<double, 6, 1> solved = result.H.ldlt().solve(-result.b);
  EXPECT_NEAR(solved(0), 0.02, 1.0e-6);
  EXPECT_NEAR(solved(1), 0.0, 1.0e-12);
}

TEST(SelectiveVisualFusion, ClampsTranslationAndRotationSeparately) {
  Eigen::Matrix<double, 6, 1> update =
      Eigen::Matrix<double, 6, 1>::Zero();
  update(0) = 2.0;
  update(3) = 1.0;
  rko_lio::core::SelectiveVisualFusionConfig config;
  config.max_translation_update_m = 0.05;
  config.max_rotation_update_rad = 0.02;
  const auto clamped = rko_lio::core::clamp_visual_update(update, config);
  EXPECT_NEAR(clamped.head<3>().norm(), 0.05, 1.0e-12);
  EXPECT_NEAR(clamped.tail<3>().norm(), 0.02, 1.0e-12);
}

}  // namespace
