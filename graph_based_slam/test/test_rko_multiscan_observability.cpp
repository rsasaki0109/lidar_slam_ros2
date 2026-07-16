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
