// Copyright 2026 Sasaki
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "scanmatcher/registration_config.hpp"
#include "scanmatcher/scanmatcher_component.h"

#ifndef HAS_FAST_GICP
TEST(FastGicpSelector, MissingOptionalDependencyIsHardFailure)
{
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  EXPECT_FALSE(graphslam::registration_config::fastGicpAvailable());
  EXPECT_EXIT(
    {
      if (!rclcpp::ok()) {
        char ** argv = nullptr;
        rclcpp::init(0, argv);
      }
      rclcpp::NodeOptions options;
      options.parameter_overrides({rclcpp::Parameter("registration_method", "FAST_GICP")});
      auto component = std::make_shared<graphslam::ScanMatcherComponent>(options);
      (void)component;
    },
    ::testing::ExitedWithCode(1),
    "no fallback");
}

TEST(FastGicpSelector, MissingOptionalVgicpDependencyIsHardFailure)
{
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  EXPECT_EXIT(
    {
      if (!rclcpp::ok()) {
        char ** argv = nullptr;
        rclcpp::init(0, argv);
      }
      rclcpp::NodeOptions options;
      options.parameter_overrides({rclcpp::Parameter("registration_method", "FAST_VGICP")});
      auto component = std::make_shared<graphslam::ScanMatcherComponent>(options);
      (void)component;
    },
    ::testing::ExitedWithCode(1),
    "no fallback");
}
#else
TEST(FastGicpSelector, OptionalDependencyIsPresent)
{
  EXPECT_TRUE(graphslam::registration_config::fastGicpAvailable());
}
#endif

#ifndef HAS_SMALL_GICP
TEST(SmallGicpSelector, MissingOptionalDependencyIsHardFailure)
{
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  EXPECT_FALSE(graphslam::registration_config::smallGicpAvailable());
  EXPECT_EXIT(
    {
      if (!rclcpp::ok()) {
        char ** argv = nullptr;
        rclcpp::init(0, argv);
      }
      rclcpp::NodeOptions options;
      options.parameter_overrides({rclcpp::Parameter("registration_method", "SMALL_GICP")});
      auto component = std::make_shared<graphslam::ScanMatcherComponent>(options);
      (void)component;
    },
    ::testing::ExitedWithCode(1),
    "no fallback");
}

TEST(SmallGicpSelector, MissingOptionalVgicpDependencyIsHardFailure)
{
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  EXPECT_EXIT(
    {
      if (!rclcpp::ok()) {
        char ** argv = nullptr;
        rclcpp::init(0, argv);
      }
      rclcpp::NodeOptions options;
      options.parameter_overrides({rclcpp::Parameter("registration_method", "SMALL_VGICP")});
      auto component = std::make_shared<graphslam::ScanMatcherComponent>(options);
      (void)component;
    },
    ::testing::ExitedWithCode(1),
    "no fallback");
}
#else
TEST(SmallGicpSelector, OptionalDependencyIsPresent)
{
  EXPECT_TRUE(graphslam::registration_config::smallGicpAvailable());
}
#endif
