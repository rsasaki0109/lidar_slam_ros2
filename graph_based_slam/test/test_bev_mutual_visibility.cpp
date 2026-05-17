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

#include "graph_based_slam/bev_mutual_visibility.hpp"
#include "graph_based_slam/submap_bev_descriptor.hpp"

namespace graphslam
{
namespace bev
{
namespace
{

constexpr int kGrid = 16;

SubmapBEVDescriptor::Descriptor makeBlankDescriptor()
{
  SubmapBEVDescriptor::Descriptor d;
  d.occupancy = Eigen::MatrixXf::Zero(kGrid, kGrid);
  d.density = Eigen::MatrixXf::Zero(kGrid, kGrid);
  d.max_height = Eigen::MatrixXf::Zero(kGrid, kGrid);
  return d;
}

// Place a "feature box" (a rectangular block of occupied cells with given
// density and height values) into a descriptor at the requested grid offsets.
void writeBlock(
  SubmapBEVDescriptor::Descriptor & d,
  int row_begin, int row_end,
  int col_begin, int col_end,
  float density_value,
  float height_value)
{
  for (int r = row_begin; r < row_end; ++r) {
    for (int c = col_begin; c < col_end; ++c) {
      if (r < 0 || r >= kGrid || c < 0 || c >= kGrid) {continue;}
      d.occupancy(r, c) = 1.0f;
      d.density(r, c) = density_value;
      d.max_height(r, c) = height_value;
    }
  }
}

TEST(BevMutualVisibility, IdenticalDescriptorsReturnZeroDistance)
{
  auto d = makeBlankDescriptor();
  writeBlock(d, 4, 8, 4, 8, 0.6f, 0.5f);
  writeBlock(d, 10, 14, 10, 14, 0.3f, 0.8f);
  const auto res = mutualVisibilityDistance(d, d);
  EXPECT_TRUE(res.valid);
  EXPECT_NEAR(0.0, res.distance, 1e-9);
  EXPECT_GT(res.overlap_ratio, 0.0);
}

TEST(BevMutualVisibility, DisjointVisibilityReturnsInvalid)
{
  auto q = makeBlankDescriptor();
  auto c = makeBlankDescriptor();
  writeBlock(q, 0, 4, 0, 4, 0.5f, 0.5f);
  writeBlock(c, 12, 16, 12, 16, 0.5f, 0.5f);
  const auto res = mutualVisibilityDistance(q, c);
  EXPECT_FALSE(res.valid);
  EXPECT_EQ(0.0, res.overlap_ratio);
  EXPECT_NEAR(1.0, res.distance, 1e-9);
}

TEST(BevMutualVisibility, EmptyDescriptorReturnsInvalid)
{
  SubmapBEVDescriptor::Descriptor q;
  SubmapBEVDescriptor::Descriptor c;
  const auto res = mutualVisibilityDistance(q, c);
  EXPECT_FALSE(res.valid);
}

TEST(BevMutualVisibility, RaisingMinOverlapForcesInvalid)
{
  auto q = makeBlankDescriptor();
  auto c = makeBlankDescriptor();
  writeBlock(q, 6, 8, 6, 8, 0.5f, 0.5f);
  writeBlock(c, 6, 8, 6, 8, 0.5f, 0.5f);
  // 4 mutually-visible cells out of 256 ≈ 0.0156 overlap ratio.
  MutualVisibilityConfig cfg;
  cfg.min_overlap_ratio = 0.10;  // requires >= 25 cells
  const auto res = mutualVisibilityDistance(q, c, cfg);
  EXPECT_FALSE(res.valid);
}

TEST(BevMutualVisibility, MaskExcludesUnobservedDifferences)
{
  // Both descriptors agree on the overlapping region (rows/cols 4..12) but
  // have completely different content in their non-overlapping observed area.
  // Mutual-visibility NCC should ignore the disagreement and report a perfect
  // score, whereas the existing cosine distance would not.
  auto q = makeBlankDescriptor();
  auto c = makeBlankDescriptor();
  writeBlock(q, 4, 12, 4, 12, 0.6f, 0.5f);  // shared block (occupied in both)
  writeBlock(c, 4, 12, 4, 12, 0.6f, 0.5f);
  // Region only observed by q (unobserved by c): completely different content.
  writeBlock(q, 0, 3, 0, 16, 0.95f, 0.9f);
  // Region only observed by c (unobserved by q).
  writeBlock(c, 13, 16, 0, 16, 0.95f, 0.9f);

  const auto fov = mutualVisibilityDistance(q, c);
  EXPECT_TRUE(fov.valid);
  EXPECT_NEAR(0.0, fov.distance, 1e-9);

  // Sanity: cosine distance does not ignore the divergent regions, so it
  // should be measurably worse than the mutual-visibility distance.
  const double cosine = SubmapBEVDescriptor::descriptorDistance(q, c);
  EXPECT_GT(cosine, fov.distance + 1e-6);
}

TEST(BevMutualVisibility, DifferentContentInsideMaskRaisesDistance)
{
  // Same visibility footprint, but the channel values disagree completely
  // inside the mask. NCC should report a non-zero distance.
  auto q = makeBlankDescriptor();
  auto c = makeBlankDescriptor();
  for (int r = 4; r < 12; ++r) {
    for (int col = 4; col < 12; ++col) {
      q.occupancy(r, col) = 1.0f;
      c.occupancy(r, col) = 1.0f;
      q.density(r, col) = (col < 8) ? 0.2f : 0.9f;
      c.density(r, col) = (col < 8) ? 0.9f : 0.2f;
      q.max_height(r, col) = (r < 8) ? 0.1f : 0.95f;
      c.max_height(r, col) = (r < 8) ? 0.95f : 0.1f;
    }
  }
  const auto res = mutualVisibilityDistance(q, c);
  EXPECT_TRUE(res.valid);
  EXPECT_GT(res.distance, 0.5);
}

TEST(BevMutualVisibility, YawSearchPicksAlignmentBin)
{
  // Block size must keep overlap_ratio above the default min_overlap of 5%,
  // i.e. >= 13 of 256 cells in a 16x16 grid.
  auto base = makeBlankDescriptor();
  writeBlock(base, 3, 9, 9, 15, 0.5f, 0.5f);
  // Density varies inside the block so density NCC is informative.
  for (int r = 3; r < 9; ++r) {
    for (int c = 9; c < 15; ++c) {
      base.density(r, c) = 0.3f + 0.1f * static_cast<float>((r + c) % 3);
    }
  }
  // Rotate the candidate by 90° about the centre.
  const SubmapBEVDescriptor::Descriptor rotated =
    SubmapBEVDescriptor::rotateDescriptor(base, M_PI / 2.0);
  const auto match = mutualVisibilityWithYawSearch(base, rotated, /*submap_id=*/7, /*yaw_bins=*/4);
  EXPECT_TRUE(match.valid);
  EXPECT_EQ(7, match.submap_id);
  EXPECT_LT(match.distance, 0.2);
  // Best yaw should correspond to the inverse rotation (the candidate is the
  // base rotated by π/2, so rotating it by 3π/2 brings it back).
  EXPECT_NE(match.yaw_bin, 0);
}

TEST(BevMutualVisibility, YawSearchInvalidWhenNoOverlap)
{
  auto q = makeBlankDescriptor();
  auto c = makeBlankDescriptor();
  writeBlock(q, 0, 1, 0, 1, 0.5f, 0.5f);
  writeBlock(c, 15, 16, 15, 16, 0.5f, 0.5f);
  const auto match = mutualVisibilityWithYawSearch(q, c, /*submap_id=*/3, /*yaw_bins=*/8);
  EXPECT_FALSE(match.valid);
  EXPECT_NEAR(1.0, match.distance, 1e-9);
}

}  // namespace
}  // namespace bev
}  // namespace graphslam
