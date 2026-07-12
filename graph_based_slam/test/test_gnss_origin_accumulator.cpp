// Copyright 2026 Ryohei Sasaki
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
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

#include "graph_based_slam/gnss_origin_accumulator.hpp"

namespace graphslam
{
namespace detail
{
namespace
{

TEST(GnssOriginAccumulator, InitializesFromConsistentFixes)
{
  GnssOriginAccumulator accumulator;
  accumulator.configure(3, 20.0);

  EXPECT_FALSE(accumulator.add(35.0, 139.0, 10.0).initialized);
  EXPECT_FALSE(accumulator.add(35.00001, 139.00001, 12.0).initialized);
  const GnssOriginUpdate update = accumulator.add(35.00002, 139.00002, 14.0);

  EXPECT_TRUE(update.initialized);
  EXPECT_EQ(update.accepted_samples, 3U);
  EXPECT_NEAR(update.origin.latitude_deg, 35.00001, 1e-12);
  EXPECT_NEAR(update.origin.longitude_deg, 139.00001, 1e-12);
  EXPECT_NEAR(update.origin.altitude_m, 12.0, 1e-12);
}

TEST(GnssOriginAccumulator, RestartsAfterLargeJump)
{
  GnssOriginAccumulator accumulator;
  accumulator.configure(3, 20.0);
  accumulator.add(35.0, 139.0, 10.0);

  const GnssOriginUpdate update = accumulator.add(35.01, 139.01, 20.0);

  EXPECT_TRUE(update.reset_after_jump);
  EXPECT_FALSE(update.initialized);
  EXPECT_EQ(update.accepted_samples, 1U);
  EXPECT_GT(update.deviation_m, 20.0);
}

TEST(GnssOriginAccumulator, SupportsSingleSampleInitialization)
{
  GnssOriginAccumulator accumulator;
  accumulator.configure(1, 20.0);

  const GnssOriginUpdate update = accumulator.add(35.0, 139.0, 10.0);

  EXPECT_TRUE(update.initialized);
  EXPECT_EQ(update.accepted_samples, 1U);
  EXPECT_DOUBLE_EQ(update.origin.latitude_deg, 35.0);
  EXPECT_DOUBLE_EQ(update.origin.longitude_deg, 139.0);
  EXPECT_DOUBLE_EQ(update.origin.altitude_m, 10.0);
}

}  // namespace
}  // namespace detail
}  // namespace graphslam
