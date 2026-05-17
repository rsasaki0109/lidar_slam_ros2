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

#include <Eigen/Core>

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

// Level 2: the same scalar helper can be driven independently for the trans
// block and the rot block of the SE(3) Information matrix. These tests model
// the runtime call pattern used in graph_based_slam_component.cpp when
// adjacent_edge_info_auto_scale_split_trans_rot_ is on.

TEST(AdjacentEdgeAutoScaleSplit, IndependentTransAndRotMoveSeparately)
{
  // Equilibrium target per block: SE(3) split with 3 DoF each → target_nis=3.
  AutoScaleConfig cfg_trans;
  cfg_trans.target_nis = 3.0;
  cfg_trans.ema_alpha = 1.0;

  AutoScaleConfig cfg_rot;
  cfg_rot.target_nis = 3.0;
  cfg_rot.ema_alpha = 1.0;

  // Trans residuals are noisier than rot residuals (median_chi2 6 vs 1.5),
  // so the trans weight should halve while the rot weight doubles in one step.
  EXPECT_DOUBLE_EQ(500.0, nextScale(1000.0, 6.0, cfg_trans));
  EXPECT_DOUBLE_EQ(2000.0, nextScale(1000.0, 1.5, cfg_rot));
}

TEST(AdjacentEdgeAutoScaleSplit, RepeatedApplicationConvergesIndependently)
{
  AutoScaleConfig cfg_trans;
  cfg_trans.target_nis = 3.0;
  cfg_trans.ema_alpha = 0.5;
  cfg_trans.min_scale = 1.0;
  cfg_trans.max_scale = 1.0e6;

  AutoScaleConfig cfg_rot;
  cfg_rot.target_nis = 3.0;
  cfg_rot.ema_alpha = 0.5;
  cfg_rot.min_scale = 1.0;
  cfg_rot.max_scale = 1.0e6;

  // Independent chi2-per-unit-scale for translation vs rotation. Equilibrium:
  // w_trans* = 3 / 0.01 = 300, w_rot* = 3 / 0.003 = 1000.
  const double trans_unit = 0.01;
  const double rot_unit = 0.003;
  double w_trans = 800.0;
  double w_rot = 200.0;
  for (int i = 0; i < 80; ++i) {
    w_trans = nextScale(w_trans, trans_unit * w_trans, cfg_trans);
    w_rot = nextScale(w_rot, rot_unit * w_rot, cfg_rot);
  }
  EXPECT_NEAR(300.0, w_trans, 1e-2);
  EXPECT_NEAR(1000.0, w_rot, 1e-2);
}

TEST(AdjacentEdgeAutoScaleSplit, ChiSquaredDecompositionMatchesBlockShape)
{
  // For a diagonal block-diag Information matrix
  //   I = block_diag(w_t * I_3, w_r * I_3)
  // the full chi^2 of e = [t; r] equals w_t ||t||^2 + w_r ||r||^2. Verify the
  // accounting the component uses against the full e^T I e expression.
  const Eigen::Matrix<double, 6, 1> err =
    (Eigen::Matrix<double, 6, 1>() << 0.2, -0.1, 0.05, 0.01, 0.0, -0.03).finished();
  const double w_t = 250.0;
  const double w_r = 800.0;
  Eigen::Matrix<double, 6, 6> info = Eigen::Matrix<double, 6, 6>::Zero();
  info.topLeftCorner<3, 3>().diagonal().setConstant(w_t);
  info.bottomRightCorner<3, 3>().diagonal().setConstant(w_r);
  const double full = err.transpose() * info * err;
  const double trans_part = w_t * err.head<3>().squaredNorm();
  const double rot_part = w_r * err.tail<3>().squaredNorm();
  EXPECT_NEAR(full, trans_part + rot_part, 1e-12);
}

}  // namespace
}  // namespace detail
}  // namespace graphslam
