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

#include "graph_based_slam/adjacent_edge_auto_scale.hpp"

namespace graphslam
{
namespace detail
{
namespace
{

TEST(AdjacentEdgeAutoScaleMedian, EmptyReturnsZero)
{
  EXPECT_DOUBLE_EQ(0.0, medianChi2({}));
}

TEST(AdjacentEdgeAutoScaleMedian, SingleElement)
{
  EXPECT_DOUBLE_EQ(3.5, medianChi2({3.5}));
}

TEST(AdjacentEdgeAutoScaleMedian, OddCount)
{
  EXPECT_DOUBLE_EQ(2.0, medianChi2({5.0, 1.0, 2.0, 0.5, 7.0}));
}

TEST(AdjacentEdgeAutoScaleMedian, EvenCountAveragesTwoMiddles)
{
  EXPECT_DOUBLE_EQ(3.0, medianChi2({1.0, 2.0, 4.0, 8.0}));
}

TEST(AdjacentEdgeAutoScaleMedian, UnsortedInputOk)
{
  EXPECT_DOUBLE_EQ(50.0, medianChi2({100.0, 50.0, 25.0}));
}

TEST(AdjacentEdgeAutoScale, MedianEqualsTargetLeavesScaleUnchanged)
{
  AutoScaleConfig cfg;
  cfg.target_nis = 6.0;
  cfg.ema_alpha = 1.0;
  EXPECT_DOUBLE_EQ(1000.0, nextScale(1000.0, 6.0, cfg));
}

TEST(AdjacentEdgeAutoScale, MedianAboveTargetReducesScale)
{
  AutoScaleConfig cfg;
  cfg.target_nis = 6.0;
  cfg.ema_alpha = 1.0;
  // chi2 doubled vs target → scale should halve.
  EXPECT_DOUBLE_EQ(500.0, nextScale(1000.0, 12.0, cfg));
}

TEST(AdjacentEdgeAutoScale, MedianBelowTargetIncreasesScale)
{
  AutoScaleConfig cfg;
  cfg.target_nis = 6.0;
  cfg.ema_alpha = 1.0;
  // chi2 halved vs target → scale should double.
  EXPECT_DOUBLE_EQ(2000.0, nextScale(1000.0, 3.0, cfg));
}

TEST(AdjacentEdgeAutoScale, EmaMixesPartialMove)
{
  AutoScaleConfig cfg;
  cfg.target_nis = 6.0;
  cfg.ema_alpha = 0.5;
  // implied = 1000 * (6/12) = 500. mixed = 0.5 * 1000 + 0.5 * 500 = 750.
  EXPECT_DOUBLE_EQ(750.0, nextScale(1000.0, 12.0, cfg));
}

TEST(AdjacentEdgeAutoScale, EmaAlphaZeroFreezesScale)
{
  AutoScaleConfig cfg;
  cfg.target_nis = 6.0;
  cfg.ema_alpha = 0.0;
  EXPECT_DOUBLE_EQ(1000.0, nextScale(1000.0, 12.0, cfg));
}

TEST(AdjacentEdgeAutoScale, EmaAlphaClampedToUnit)
{
  AutoScaleConfig cfg;
  cfg.target_nis = 6.0;
  cfg.ema_alpha = 5.0;  // out-of-range value clamps to 1.0
  EXPECT_DOUBLE_EQ(500.0, nextScale(1000.0, 12.0, cfg));
}

TEST(AdjacentEdgeAutoScale, RespectsMinClamp)
{
  AutoScaleConfig cfg;
  cfg.target_nis = 6.0;
  cfg.ema_alpha = 1.0;
  cfg.min_scale = 200.0;
  cfg.max_scale = 1.0e6;
  // implied = 1000 * (6/1000) = 6 → clamps to 200.
  EXPECT_DOUBLE_EQ(200.0, nextScale(1000.0, 1000.0, cfg));
}

TEST(AdjacentEdgeAutoScale, RespectsMaxClamp)
{
  AutoScaleConfig cfg;
  cfg.target_nis = 6.0;
  cfg.ema_alpha = 1.0;
  cfg.min_scale = 1.0;
  cfg.max_scale = 5000.0;
  // implied = 1000 * (6/0.1) = 60000 → clamps to 5000.
  EXPECT_DOUBLE_EQ(5000.0, nextScale(1000.0, 0.1, cfg));
}

TEST(AdjacentEdgeAutoScale, MedianZeroLeavesScaleUnchanged)
{
  AutoScaleConfig cfg;
  cfg.target_nis = 6.0;
  cfg.ema_alpha = 0.5;
  EXPECT_DOUBLE_EQ(1000.0, nextScale(1000.0, 0.0, cfg));
}

TEST(AdjacentEdgeAutoScale, NegativeTargetLeavesScaleUnchanged)
{
  AutoScaleConfig cfg;
  cfg.target_nis = -1.0;
  cfg.ema_alpha = 0.5;
  EXPECT_DOUBLE_EQ(1000.0, nextScale(1000.0, 12.0, cfg));
}

TEST(AdjacentEdgeAutoScale, NegativeCurrentScaleLeavesUnchanged)
{
  AutoScaleConfig cfg;
  EXPECT_DOUBLE_EQ(-50.0, nextScale(-50.0, 12.0, cfg));
}

TEST(AdjacentEdgeAutoScale, RepeatedApplicationConvergesToTarget)
{
  AutoScaleConfig cfg;
  cfg.target_nis = 6.0;
  cfg.ema_alpha = 0.5;
  cfg.min_scale = 1.0;
  cfg.max_scale = 1.0e6;

  // Model: the edge residual is roughly fixed across nearby weight values, so
  // chi2 = e^T (s * I) e is approximately proportional to the scale s.
  // Picking baseline_chi2_per_unit = 0.012 makes chi2=12 at s=1000, which means
  // the equilibrium scale (where chi2 == target == 6) is s* = 6 / 0.012 = 500.
  const double baseline_chi2_per_unit = 0.012;
  double scale = 1000.0;
  for (int i = 0; i < 50; ++i) {
    const double observed_chi2 = baseline_chi2_per_unit * scale;
    scale = nextScale(scale, observed_chi2, cfg);
  }
  EXPECT_NEAR(500.0, scale, 1e-2);
}

}  // namespace
}  // namespace detail
}  // namespace graphslam
