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

#include <limits>

#include <gtest/gtest.h>

#include "graph_based_slam/gnss_geometry.hpp"

namespace graphslam
{
namespace detail
{
namespace
{

TEST(GnssGeometry, ValidatesFiniteCoordinatesAndRanges)
{
  EXPECT_TRUE(isUsableGeodeticFix(35.681236, 139.767125, 12.0));
  EXPECT_FALSE(isUsableGeodeticFix(0.0, 0.0, 0.0));
  EXPECT_FALSE(isUsableGeodeticFix(91.0, 139.0, 0.0));
  EXPECT_FALSE(isUsableGeodeticFix(
    std::numeric_limits<double>::quiet_NaN(), 139.0, 0.0));
}

TEST(GnssGeometry, ComputesShortGeodeticDistanceSymmetrically)
{
  const double forward = approximateGeodeticDistanceMeters(35.0, 139.0, 35.001, 139.002);
  const double reverse = approximateGeodeticDistanceMeters(35.001, 139.002, 35.0, 139.0);
  EXPECT_NEAR(forward, 213.5, 1.0);
  EXPECT_NEAR(reverse, forward, 1e-9);
}

TEST(GnssGeometry, ConvertsOriginToZeroEnu)
{
  const GeodeticOrigin origin {35.0, 139.0, 42.0};
  EXPECT_TRUE(geodeticToEnu(35.0, 139.0, 42.0, origin).isZero(1e-12));
}

TEST(GnssGeometry, ConvertsLocalOffsetsToEastNorthUp)
{
  const GeodeticOrigin origin {35.0, 139.0, 10.0};
  const Eigen::Vector3d enu = geodeticToEnu(35.001, 139.001, 15.0, origin);
  EXPECT_NEAR(enu.x(), 91.29, 0.1);
  EXPECT_NEAR(enu.y(), 110.94, 0.1);
  EXPECT_DOUBLE_EQ(enu.z(), 5.0);
}

}  // namespace
}  // namespace detail
}  // namespace graphslam
