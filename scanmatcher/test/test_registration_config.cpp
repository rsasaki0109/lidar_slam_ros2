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
//    copyright notice, this list of conditions and the following disclaimer
//    in the documentation and/or other materials provided with the
//    distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>

#include <cstdint>

#include "scanmatcher/registration_config.hpp"

namespace registration = lidarslam::plugins::registration;

TEST(RegistrationConfig, FastGicpAvailabilityIsExplicitAndNeverFallbacks)
{
  EXPECT_TRUE(graphslam::registration_config::isFastGicpMethod("FAST_GICP"));
  EXPECT_TRUE(graphslam::registration_config::isFastGicpMethod("FAST_VGICP"));
  EXPECT_FALSE(graphslam::registration_config::isFastGicpMethod("NDT"));
  EXPECT_FALSE(graphslam::registration_config::isFastGicpMethod("GICP"));

#ifdef HAS_FAST_GICP
  EXPECT_TRUE(graphslam::registration_config::fastGicpAvailable());
#else
  EXPECT_FALSE(graphslam::registration_config::fastGicpAvailable());
  EXPECT_NE(
    std::string(graphslam::registration_config::fastGicpUnavailableReason()).find("no fallback"),
    std::string::npos);
#endif
}

TEST(RegistrationConfig, SmallGicpAvailabilityIsExplicitAndNeverFallbacks)
{
  EXPECT_TRUE(graphslam::registration_config::isSmallGicpMethod("SMALL_GICP"));
  EXPECT_TRUE(graphslam::registration_config::isSmallGicpMethod("SMALL_VGICP"));
  EXPECT_FALSE(graphslam::registration_config::isSmallGicpMethod("NDT"));
  EXPECT_FALSE(graphslam::registration_config::isSmallGicpMethod("GICP"));

#ifdef HAS_SMALL_GICP
  EXPECT_TRUE(graphslam::registration_config::smallGicpAvailable());
#else
  EXPECT_FALSE(graphslam::registration_config::smallGicpAvailable());
  const std::string reason =
    graphslam::registration_config::smallGicpUnavailableReason();
  EXPECT_NE(reason.find("small_gicp"), std::string::npos);
  EXPECT_NE(reason.find("SMALL_GICP/SMALL_VGICP"), std::string::npos);
  EXPECT_NE(reason.find("no fallback"), std::string::npos);
#endif
}

TEST(RegistrationConfig, MapsLegacyNdtValuesToTypedCanonicalKeys)
{
  const registration::ParameterMap parameters =
    graphslam::registration_config::makeNdtParameterMap(2.5, 0.02, 37, 0.15, 0.6, 4);

  ASSERT_EQ(parameters.size(), 7U);
  EXPECT_DOUBLE_EQ(parameters.at("resolution").asDouble(), 2.5);
  EXPECT_DOUBLE_EQ(parameters.at("transformation_epsilon").asDouble(), 0.02);
  EXPECT_EQ(parameters.at("maximum_iterations").asInteger(), std::int64_t{37});
  EXPECT_DOUBLE_EQ(parameters.at("step_size").asDouble(), 0.15);
  EXPECT_DOUBLE_EQ(parameters.at("outlier_ratio").asDouble(), 0.6);
  EXPECT_EQ(parameters.at("num_threads").asInteger(), std::int64_t{4});
  EXPECT_EQ(parameters.at("neighborhood_search_method").asString(), "DIRECT7");
  EXPECT_THROW(parameters.at("num_threads").asDouble(), std::logic_error);
}

TEST(RegistrationConfig, MapsLegacyGicpValuesAndAdaptiveMode)
{
  const registration::ParameterMap parameters =
    graphslam::registration_config::makeGicpParameterMap(4.5, true);

  ASSERT_EQ(parameters.size(), 3U);
  EXPECT_DOUBLE_EQ(parameters.at("maximum_correspondence_distance").asDouble(), 4.5);
  EXPECT_DOUBLE_EQ(parameters.at("transformation_epsilon").asDouble(), 1e-8);
  EXPECT_TRUE(parameters.at("adaptive_correspondence_threshold").asBool());
  EXPECT_THROW(parameters.at("adaptive_correspondence_threshold").asDouble(), std::logic_error);
}
