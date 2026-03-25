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

#include <gtest/gtest.h>

#include "graph_based_slam/gnss_weighting.hpp"

namespace graphslam
{
namespace detail
{
namespace
{

sensor_msgs::msg::NavSatFix makeFix(
  double var_x,
  double var_y,
  double var_z,
  uint8_t covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN)
{
  sensor_msgs::msg::NavSatFix msg;
  msg.position_covariance_type = covariance_type;
  msg.position_covariance[0] = var_x;
  msg.position_covariance[4] = var_y;
  msg.position_covariance[8] = var_z;
  return msg;
}

TEST(GnssWeighting, FallsBackToFixedWeightsWithoutCovariance)
{
  GnssWeightingConfig config;
  config.base_info_weight = 2.0;
  config.vertical_weight_scale = 0.2;
  config.non_rtk_weight_scale = 1.5;
  sensor_msgs::msg::NavSatFix msg;
  msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

  const auto weights = computeGnssConstraintWeights(msg, config);

  EXPECT_FALSE(weights.covariance_valid);
  EXPECT_FALSE(weights.rtk_like);
  EXPECT_DOUBLE_EQ(weights.info_x, 3.0);
  EXPECT_DOUBLE_EQ(weights.info_y, 3.0);
  EXPECT_DOUBLE_EQ(weights.info_z, 0.6);
}

TEST(GnssWeighting, MarksLowCovarianceFixAsRtkLike)
{
  GnssWeightingConfig config;
  config.base_info_weight = 1.0;
  config.vertical_weight_scale = 0.1;
  config.covariance_min_variance_m2 = 0.01;
  config.covariance_max_variance_m2 = 25.0;
  config.rtk_fix_max_horizontal_stddev_m = 0.3;
  config.rtk_fix_weight_scale = 4.0;
  config.non_rtk_weight_scale = 1.0;

  const auto weights = computeGnssConstraintWeights(makeFix(0.01, 0.04, 0.25), config);

  EXPECT_TRUE(weights.covariance_valid);
  EXPECT_TRUE(weights.rtk_like);
  EXPECT_NEAR(weights.horizontal_stddev_m, 0.2, 1e-9);
  EXPECT_DOUBLE_EQ(weights.info_x, 400.0);
  EXPECT_DOUBLE_EQ(weights.info_y, 100.0);
  EXPECT_DOUBLE_EQ(weights.info_z, 1.6);
}

TEST(GnssWeighting, KeepsHigherCovarianceFixAsNonRtk)
{
  GnssWeightingConfig config;
  config.base_info_weight = 2.0;
  config.vertical_weight_scale = 0.2;
  config.covariance_min_variance_m2 = 0.01;
  config.covariance_max_variance_m2 = 100.0;
  config.rtk_fix_max_horizontal_stddev_m = 0.3;
  config.rtk_fix_weight_scale = 4.0;
  config.non_rtk_weight_scale = 0.5;

  const auto weights = computeGnssConstraintWeights(makeFix(4.0, 9.0, 16.0), config);

  EXPECT_TRUE(weights.covariance_valid);
  EXPECT_FALSE(weights.rtk_like);
  EXPECT_NEAR(weights.horizontal_stddev_m, 3.0, 1e-9);
  EXPECT_DOUBLE_EQ(weights.info_x, 0.25);
  EXPECT_DOUBLE_EQ(weights.info_y, 1.0 / 9.0);
  EXPECT_DOUBLE_EQ(weights.info_z, 0.0125);
}

TEST(GnssWeighting, ClampsVerySmallVarianceBeforeWeighting)
{
  GnssWeightingConfig config;
  config.base_info_weight = 1.0;
  config.vertical_weight_scale = 0.1;
  config.covariance_min_variance_m2 = 0.04;
  config.covariance_max_variance_m2 = 25.0;
  config.rtk_fix_max_horizontal_stddev_m = 0.3;
  config.rtk_fix_weight_scale = 2.0;
  config.non_rtk_weight_scale = 1.0;

  const auto weights = computeGnssConstraintWeights(makeFix(1e-6, 1e-6, 1e-6), config);

  EXPECT_TRUE(weights.covariance_valid);
  EXPECT_TRUE(weights.rtk_like);
  EXPECT_NEAR(weights.horizontal_stddev_m, 0.2, 1e-9);
  EXPECT_DOUBLE_EQ(weights.info_x, 50.0);
  EXPECT_DOUBLE_EQ(weights.info_y, 50.0);
  EXPECT_DOUBLE_EQ(weights.info_z, 5.0);
}

TEST(GnssWeighting, KeepsReasonableHeaderTimestamp)
{
  const auto resolved = resolveGnssMeasurementStamp(101.25, 100.0, 5.0);

  EXPECT_FALSE(resolved.used_fallback);
  EXPECT_DOUBLE_EQ(resolved.stamp_sec, 101.25);
}

TEST(GnssWeighting, FallsBackWhenHeaderTimestampIsZero)
{
  const auto resolved = resolveGnssMeasurementStamp(0.0, 100.0, 5.0);

  EXPECT_TRUE(resolved.used_fallback);
  EXPECT_DOUBLE_EQ(resolved.stamp_sec, 100.0);
}

TEST(GnssWeighting, FallsBackWhenHeaderTimestampSkewIsTooLarge)
{
  const auto resolved = resolveGnssMeasurementStamp(478404.12, 1656075186.17, 30.0);

  EXPECT_TRUE(resolved.used_fallback);
  EXPECT_DOUBLE_EQ(resolved.stamp_sec, 1656075186.17);
}

}  // namespace
}  // namespace detail
}  // namespace graphslam
