// Copyright 2026 Sasaki
// SPDX-License-Identifier: BSD-2-Clause

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <cmath>

#include "rko_lio/core/persistent_weak_direction.hpp"

namespace
{

using rko_lio::core::PersistentWeakDirectionConfig;
using rko_lio::core::PersistentWeakDirectionTracker;

Eigen::Matrix<double, 6, 6> informationWithWeakTranslation(const double yaw)
{
  Eigen::Matrix<double, 6, 6> information =
    Eigen::Matrix<double, 6, 6>::Zero();
  const Eigen::Rotation2Dd rotation(yaw);
  Eigen::Matrix2d translation = Eigen::Matrix2d::Zero();
  translation(0, 0) = 1.0e-8;
  translation(1, 1) = 1.0;
  information.block<2, 2>(0, 0) =
    rotation.toRotationMatrix() * translation * rotation.toRotationMatrix().transpose();
  information(2, 2) = 2.0;
  information(3, 3) = 3.0;
  information(4, 4) = 4.0;
  information(5, 5) = 5.0;
  return information;
}

PersistentWeakDirectionConfig multiscanConfig()
{
  PersistentWeakDirectionConfig config;
  config.min_consecutive_scans = 3;
  config.min_absolute_cosine = 0.98;
  config.min_translation_fraction = 0.99;
  config.require_multiscan_observability = true;
  config.observability_window_scans = 5;
  config.observability_min_scans = 3;
  config.max_aggregate_directional_information_ratio = 5.0e-4;
  return config;
}

TEST(RkoMultiscanObservability, ConfirmsDirectionWeakAcrossWholeWindow)
{
  PersistentWeakDirectionTracker tracker;
  const auto config = multiscanConfig();

  tracker.observe(informationWithWeakTranslation(0.0), 1.0e-6, 1.0e-8, config);
  tracker.observe(informationWithWeakTranslation(0.0), 1.0e-6, 1.0e-8, config);
  const auto state =
    tracker.observe(informationWithWeakTranslation(0.0), 1.0e-6, 1.0e-8, config);

  EXPECT_TRUE(state.confirmed);
  EXPECT_TRUE(state.multiscan_observability_confirmed);
  EXPECT_EQ(state.observability_window_scans, 3U);
  EXPECT_LT(state.aggregate_directional_information_ratio, 1.0e-6);
}

TEST(RkoMultiscanObservability, RejectsSlowlyRotatingInstantaneousWeakAxis)
{
  PersistentWeakDirectionTracker tracker;
  const auto config = multiscanConfig();

  tracker.observe(informationWithWeakTranslation(0.0), 1.0e-6, 1.0e-8, config);
  tracker.observe(informationWithWeakTranslation(0.1), 1.0e-6, 1.0e-8, config);
  const auto state =
    tracker.observe(informationWithWeakTranslation(0.2), 1.0e-6, 1.0e-8, config);

  EXPECT_EQ(state.consecutive_scans, 3U);
  EXPECT_FALSE(state.multiscan_observability_confirmed);
  EXPECT_FALSE(state.confirmed);
  EXPECT_GT(state.aggregate_directional_information_ratio,
            config.max_aggregate_directional_information_ratio);
}

TEST(RkoMultiscanObservability, KeepsLegacyPersistenceWhenGateIsDisabled)
{
  PersistentWeakDirectionTracker tracker;
  auto config = multiscanConfig();
  config.require_multiscan_observability = false;
  config.observability_min_scans = 5;

  tracker.observe(informationWithWeakTranslation(0.0), 1.0e-6, 1.0e-8, config);
  tracker.observe(informationWithWeakTranslation(0.1), 1.0e-6, 1.0e-8, config);
  const auto state =
    tracker.observe(informationWithWeakTranslation(0.2), 1.0e-6, 1.0e-8, config);

  EXPECT_TRUE(state.confirmed);
  EXPECT_FALSE(state.multiscan_observability_confirmed);
}

}  // namespace
