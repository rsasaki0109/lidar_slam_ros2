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

// FOV-aware BEV descriptor distance for solid-state LiDAR loop closure.
//
// The bundled ``SubmapBEVDescriptor::descriptorDistance`` uses cosine distance
// over the flattened occupancy / density / max_height stack. That works well
// on 360-degree spinning LiDAR because both submaps observe roughly the same
// FOV, so unobserved cells (= 0) mean the same thing on both sides. On
// non-360 solid-state LiDAR (e.g. Livox MID-360) two submaps captured from
// different poses can have very different observed footprints, and the cosine
// distance lumps "the sensor never saw this cell" together with "the sensor
// saw this cell and it was empty". That biases place recognition.
//
// This header implements the FOV-aware fix recommended in the GPT pro 先生
// roadmap: build a mutual-visibility mask = (query observed) AND (candidate
// observed), then compute zero-mean normalized cross-correlation over the
// masked cells per channel and average across channels. Cells outside the
// mask do not contribute, so unobserved area no longer biases the score.

#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

#include "graph_based_slam/submap_bev_descriptor.hpp"

namespace graphslam
{
namespace bev
{

struct MutualVisibilityResult
{
  // 1 - average NCC across channels, clamped to [0, 2]. 0 = perfect match,
  // 1 = uncorrelated, 2 = inverse-correlated (rare in practice).
  double distance {1.0};
  // Fraction of cells inside the mutual-visibility mask.
  double overlap_ratio {0.0};
  // False when the overlap was below ``min_overlap_ratio``; ``distance``
  // is then forced to 1.0 so the caller treats it as no match.
  bool valid {false};
};

struct MutualVisibilityConfig
{
  // Minimum fraction of grid cells that must lie inside the mutual-visibility
  // mask for the score to be considered valid. With grids of 40x40 the default
  // 0.05 corresponds to ~80 mutually-observed cells.
  double min_overlap_ratio {0.05};
  // Cell-value epsilon used to convert occupancy floats into the visibility
  // mask. occupancy is written as 0.0f or 1.0f by SubmapBEVDescriptor.
  float occupancy_eps {0.5f};
};

namespace detail
{

inline Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> visibilityMask(
  const Eigen::MatrixXf & occupancy, float occupancy_eps)
{
  return  occupancy.array() > occupancy_eps;
}

// Compute zero-mean normalized cross-correlation of two grids, restricted to
// cells where ``mask`` is true. Returns {ncc, informative}.
//
// Edge cases:
//   - fewer than 2 masked cells: {0.0, false} (cannot compute)
//   - both sides constant within the mask and means agree: {+1.0, true}
//     (the channel is consistent across the mutually-visible region; it does
//     carry information, namely "no disagreement")
//   - both sides constant but means disagree: {-1.0, true}
//     (the channel is a hard disagreement)
//   - exactly one side constant while the other varies: {0.0, false}
//     (correlation is undefined; caller should skip this channel)
//   - otherwise: clamped Pearson correlation, {ncc, true}.
inline std::pair<double, bool> maskedNCC(
  const Eigen::MatrixXf & a,
  const Eigen::MatrixXf & b,
  const Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> & mask)
{
  const int rows = a.rows();
  const int cols = a.cols();
  if (rows != b.rows() || cols != b.cols() || rows != mask.rows() || cols != mask.cols()) {
    return {0.0, false};
  }
  double sum_a = 0.0;
  double sum_b = 0.0;
  int n = 0;
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      if (!mask(r, c)) {continue;}
      sum_a += a(r, c);
      sum_b += b(r, c);
      ++n;
    }
  }
  if (n < 2) {
    return {0.0, false};
  }
  const double mean_a = sum_a / n;
  const double mean_b = sum_b / n;

  double var_a = 0.0;
  double var_b = 0.0;
  double cov = 0.0;
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      if (!mask(r, c)) {continue;}
      const double da = a(r, c) - mean_a;
      const double db = b(r, c) - mean_b;
      var_a += da * da;
      var_b += db * db;
      cov += da * db;
    }
  }
  constexpr double kVarEps = 1e-12;
  constexpr double kMeanEps = 1e-6;
  const bool a_constant = var_a < kVarEps;
  const bool b_constant = var_b < kVarEps;
  if (a_constant && b_constant) {
    return {(std::abs(mean_a - mean_b) < kMeanEps) ? 1.0 : -1.0, true};
  }
  if (a_constant || b_constant) {
    return {0.0, false};
  }
  const double ncc = cov / std::sqrt(var_a * var_b);
  return {std::max(-1.0, std::min(1.0, ncc)), true};
}

}  // namespace detail

// Score a single (query, candidate) descriptor pair with mutual-visibility NCC.
// Both descriptors must share the same grid resolution.
inline MutualVisibilityResult mutualVisibilityDistance(
  const SubmapBEVDescriptor::Descriptor & query,
  const SubmapBEVDescriptor::Descriptor & candidate,
  const MutualVisibilityConfig & cfg = MutualVisibilityConfig())
{
  MutualVisibilityResult result;
  if (query.occupancy.rows() != candidate.occupancy.rows() ||
    query.occupancy.cols() != candidate.occupancy.cols() ||
    query.occupancy.rows() == 0 || query.occupancy.cols() == 0)
  {
    return result;
  }
  const auto mask_q = detail::visibilityMask(query.occupancy, cfg.occupancy_eps);
  const auto mask_c = detail::visibilityMask(candidate.occupancy, cfg.occupancy_eps);
  const auto mask = mask_q && mask_c;

  const int total = static_cast<int>(mask.size());
  const int overlap = static_cast<int>(mask.count());
  result.overlap_ratio = total > 0 ? static_cast<double>(overlap) / total : 0.0;

  const double min_overlap = std::max(0.0, cfg.min_overlap_ratio);
  if (result.overlap_ratio < min_overlap || overlap < 2) {
    return result;
  }

  const auto pair_o = detail::maskedNCC(query.occupancy, candidate.occupancy, mask);
  const auto pair_d = detail::maskedNCC(query.density, candidate.density, mask);
  const auto pair_h = detail::maskedNCC(query.max_height, candidate.max_height, mask);
  std::vector<double> nccs;
  nccs.reserve(3);
  if (pair_o.second) {nccs.push_back(pair_o.first);}
  if (pair_d.second) {nccs.push_back(pair_d.first);}
  if (pair_h.second) {nccs.push_back(pair_h.first);}
  if (nccs.empty()) {
    // No channel produced an informative NCC. The mutual-visibility mask
    // itself overlaps, so treat that as a structural match.
    result.distance = 0.0;
    result.valid = true;
    return result;
  }
  const double avg_ncc =
    std::accumulate(nccs.begin(), nccs.end(), 0.0) / static_cast<double>(nccs.size());
  result.distance = std::max(0.0, std::min(2.0, 1.0 - avg_ncc));
  result.valid = true;
  return result;
}

// Yaw-search variant. Rotates ``candidate`` through ``yaw_bins`` evenly-spaced
// orientations, evaluates mutual-visibility NCC at each, and returns the best
// match (lowest distance). Mirrors SubmapBEVDescriptor::distanceWithAlignment
// but uses FOV-aware scoring underneath.
struct YawAlignedMatch
{
  int submap_id {-1};
  double distance {1.0};
  double overlap_ratio {0.0};
  int yaw_bin {0};
  double yaw_rad {0.0};
  bool valid {false};
};

inline YawAlignedMatch mutualVisibilityWithYawSearch(
  const SubmapBEVDescriptor::Descriptor & query,
  const SubmapBEVDescriptor::Descriptor & candidate,
  int submap_id,
  int yaw_bins = SubmapBEVDescriptor::DEFAULT_YAW_BINS,
  const MutualVisibilityConfig & cfg = MutualVisibilityConfig())
{
  YawAlignedMatch best;
  best.submap_id = submap_id;
  best.distance = std::numeric_limits<double>::max();
  if (yaw_bins < 1) {
    yaw_bins = 1;
  }
  for (int bin = 0; bin < yaw_bins; ++bin) {
    const double yaw_rad = static_cast<double>(bin) * 2.0 * M_PI /
      static_cast<double>(yaw_bins);
    const SubmapBEVDescriptor::Descriptor rotated =
      SubmapBEVDescriptor::rotateDescriptor(candidate, yaw_rad);
    const MutualVisibilityResult res = mutualVisibilityDistance(query, rotated, cfg);
    if (!res.valid) {continue;}
    if (res.distance < best.distance) {
      best.distance = res.distance;
      best.overlap_ratio = res.overlap_ratio;
      best.yaw_bin = bin;
      best.yaw_rad = yaw_rad;
      best.valid = true;
    }
  }
  if (!best.valid) {
    best.distance = 1.0;
  }
  return best;
}

}  // namespace bev
}  // namespace graphslam
