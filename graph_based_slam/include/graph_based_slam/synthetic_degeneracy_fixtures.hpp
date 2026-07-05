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

#ifndef GRAPH_BASED_SLAM__SYNTHETIC_DEGENERACY_FIXTURES_HPP_
#define GRAPH_BASED_SLAM__SYNTHETIC_DEGENERACY_FIXTURES_HPP_

// Deterministic synthetic degenerate-scenario fixtures + point-to-plane
// Gauss-Newton (H, b) assembly, built for docs/roadmap/v0.8.md Phase 0
// ("freeze metrics"). These are the oracle fixtures Phase 1's planned
// `localizability_analysis.hpp` (eigenvalue/condition-number classification
// per Zhang, Kaess, Singh, "On Degeneracy of Optimization-based State
// Estimation Problems", ICRA 2016; direction categories per Tuna et al.,
// "X-ICP", arXiv:2306.08258 / IEEE T-RO 2024) will be unit-tested against.
//
// Clean-room: only the published point-to-plane ICP residual/Jacobian
//   r_i = n_i^T (R p_i + t - q_i),   J_i = [ n_i^T , (R p_i x n_i)^T ]
// (the standard SE(3) point-to-plane linearization about a perturbation
// R ~= I + skew(theta), twist order [translation, rotation], exactly as
// used by graph_based_slam/plane_ba.hpp's se3_lie.hpp convention) is used
// here; no GPL reference implementation was read for this file, and no
// upstream RKO-LIO source was consulted.
//
// Three scenarios (docs/roadmap/v0.8.md Phase 0 / §4.3 / §6):
//
//   - corridor: parallel side walls (+/- y normal) + floor/ceiling
//     (+/- z normal), extruded along x with no x-normal texture anywhere.
//     No correspondence's plane normal has an x component, and the moment
//     arm (p x n) never produces an x-translation Jacobian entry either
//     (see the derivation below) -- so H's row/column for pure
//     x-translation (the along-corridor axis) is *exactly* zero, an exact
//     algebraic degeneracy proof, not merely "small". This matches the v0.8
//     motivating evidence (docs/roadmap/v0.8.md §0): exp07's corridor drift
//     is along the corridor's long axis specifically, not a rotational
//     (yaw) degeneracy.
//   - box: the corridor cross-section plus two end walls (+/- x normal),
//     i.e. a fully closed six-plane room. Every one of the six directions
//     now has a directly- or moment-arm-constrained Jacobian column ->
//     well-conditioned in all six directions.
//   - single_plane: a floor only. Only tz, rx, ry receive any constraint;
//     tx, ty, and rz (yaw about the plane normal) are exactly zero columns
//     -> three simultaneously non-observable directions, the
//     "do-not-move" multi-directional degenerate case (the same shape of
//     hazard the v0.7 Phase 2 plane-BA gauge fixing was built to respect).
//
// Jacobian derivation used above (identity pose, R=I): for a wall/plane
// with fixed unit normal n and surface point p = (px, py, pz),
//   J_t     = n                      (translation block, always n itself)
//   J_theta = p x n                  (rotation block, the moment arm)
// A corridor wall at y=+w/2 with inward normal n=(0,-1,0) gives
// J = [0,-1,0, pz,0,-px] (no px-independent trace of tx anywhere); the
// floor/ceiling normals (0,0,+-1) give J = [0,0,+-1, +-py,-+px,0] -- neither
// family ever has a nonzero *tx* entry, so summing J J^T over any number of
// such correspondences leaves the tx row/column exactly zero. Adding end
// walls with normal (+-1,0,0) contributes J = [+-1,0,0, 0,+-pz,-+py], which
// is what promotes the corridor to the fully-observable box.
//
// Determinism: every fixture point lattice is a fixed nested loop (no
// randomness in point placement, no floating-point-order-dependent
// reductions); the optional along-normal "wall thickness" noise uses a
// std::mt19937 seeded with a fixed, fixture-local constant, drawn in a
// fixed per-sample order. Noise is applied strictly along each
// correspondence's own plane normal, so it perturbs the ideal-vs-noisy
// point offset (and therefore `b`) but leaves the moment arm p x n --
// and therefore every eigenvalue of H -- unaffected. Eigen decompositions
// (`computeEigenSignature`) sort eigenvalues ascending and canonicalize
// each eigenvector's sign by its largest-magnitude component, the same
// discipline as `scatter_eigen_cost.hpp`'s `canonicalNormal`, so repeated
// runs are bitwise identical.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Core>  // NOLINT(build/include_order)
#include <Eigen/Eigenvalues>  // NOLINT(build/include_order)

namespace graphslam
{
namespace degeneracy
{

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

/// One point-to-plane correspondence in the local (sensor) frame: a surface
/// point `point` and its unit plane normal `normal`. `planar_offset` is the
/// signed along-normal noise already baked into `point` (so the noise-free
/// ideal surface point is `point - planar_offset * normal`); it only feeds
/// the residual `b`, never the Jacobian.
struct PlaneCorrespondence
{
  Eigen::Vector3d point {Eigen::Vector3d::Zero()};
  Eigen::Vector3d normal {Eigen::Vector3d::Zero()};
  double planar_offset {0.0};
};

struct DegeneracyFixture
{
  std::string name;
  std::vector<PlaneCorrespondence> correspondences;
};

struct CorridorFixtureConfig
{
  double length {20.0};            // along x, sampled over [0, length]
  double width {2.0};               // side walls at y = +/- width/2
  double height {2.5};              // floor z=0, ceiling z=height
  double grid_step {0.5};           // sample spacing along every axis
  double wall_noise_sigma {0.0};    // along-normal thickness noise; 0 = exact
  std::uint32_t seed {20260705u};
};

struct BoxFixtureConfig
{
  double length {20.0};
  double width {2.0};
  double height {2.5};
  double grid_step {0.5};
  double wall_noise_sigma {0.0};
  std::uint32_t seed {20260705u};
};

struct SinglePlaneFixtureConfig
{
  double extent_x {20.0};           // floor spans x in [0, extent_x]
  double extent_y {2.0};            // floor spans y in [-extent_y/2, extent_y/2]
  double grid_step {0.5};
  double wall_noise_sigma {0.0};
  std::uint32_t seed {20260705u};
};

namespace detail
{

// Deterministic per-sample noise: a fixed-seed engine drawn strictly in
// ascending sample-index order, so regenerating a fixture always reproduces
// the identical noise sequence bitwise.
inline std::vector<double> deterministicNoise(
  std::size_t count, double sigma, std::uint32_t seed)
{
  std::vector<double> noise(count, 0.0);
  if (sigma <= 0.0) {
    return noise;
  }
  std::mt19937 rng(seed);
  std::normal_distribution<double> dist(0.0, sigma);
  for (std::size_t i = 0; i < count; ++i) {
    noise[i] = dist(rng);
  }
  return noise;
}

inline void appendCorrespondences(
  std::vector<PlaneCorrespondence> * out,
  const std::vector<Eigen::Vector3d> & points,
  const std::vector<Eigen::Vector3d> & normals,
  double noise_sigma,
  std::uint32_t seed)
{
  const std::vector<double> noise = deterministicNoise(points.size(), noise_sigma, seed);
  out->reserve(out->size() + points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    PlaneCorrespondence correspondence;
    correspondence.point = points[i] + noise[i] * normals[i];
    correspondence.normal = normals[i];
    correspondence.planar_offset = noise[i];
    out->push_back(correspondence);
  }
}

}  // namespace detail

/// Long straight corridor: parallel side walls + floor/ceiling, no texture
/// along the corridor axis. Exact along-corridor (x) translation
/// degeneracy (see file header derivation).
inline DegeneracyFixture makeCorridorFixture(const CorridorFixtureConfig & config)
{
  DegeneracyFixture fixture;
  fixture.name = "corridor";

  std::vector<Eigen::Vector3d> points;
  std::vector<Eigen::Vector3d> normals;

  const double half_width = 0.5 * config.width;
  for (double x = 0.0; x <= config.length + 1e-9; x += config.grid_step) {
    // Side walls, normals pointing into the corridor interior.
    for (double z = 0.0; z <= config.height + 1e-9; z += config.grid_step) {
      points.emplace_back(x, half_width, z);
      normals.emplace_back(0.0, -1.0, 0.0);
      points.emplace_back(x, -half_width, z);
      normals.emplace_back(0.0, 1.0, 0.0);
    }
    // Floor (z=0) / ceiling (z=height).
    for (double y = -half_width; y <= half_width + 1e-9; y += config.grid_step) {
      points.emplace_back(x, y, 0.0);
      normals.emplace_back(0.0, 0.0, 1.0);
      points.emplace_back(x, y, config.height);
      normals.emplace_back(0.0, 0.0, -1.0);
    }
  }

  detail::appendCorrespondences(
    &fixture.correspondences, points, normals, config.wall_noise_sigma, config.seed);
  return fixture;
}

/// Closed six-plane room: the corridor cross-section plus two end walls
/// (x=0, x=length). Fully observable in all six directions.
inline DegeneracyFixture makeBoxFixture(const BoxFixtureConfig & config)
{
  CorridorFixtureConfig corridor_config;
  corridor_config.length = config.length;
  corridor_config.width = config.width;
  corridor_config.height = config.height;
  corridor_config.grid_step = config.grid_step;
  corridor_config.wall_noise_sigma = config.wall_noise_sigma;
  corridor_config.seed = config.seed;
  DegeneracyFixture fixture = makeCorridorFixture(corridor_config);
  fixture.name = "box";

  std::vector<Eigen::Vector3d> points;
  std::vector<Eigen::Vector3d> normals;
  const double half_width = 0.5 * config.width;
  for (double y = -half_width; y <= half_width + 1e-9; y += config.grid_step) {
    for (double z = 0.0; z <= config.height + 1e-9; z += config.grid_step) {
      points.emplace_back(0.0, y, z);
      normals.emplace_back(1.0, 0.0, 0.0);
      points.emplace_back(config.length, y, z);
      normals.emplace_back(-1.0, 0.0, 0.0);
    }
  }
  // Distinct seed offset from the corridor draw so the end-wall noise
  // sequence is independent of (but still fully determined by) the
  // corridor's own noise sequence.
  detail::appendCorrespondences(
    &fixture.correspondences, points, normals, config.wall_noise_sigma, config.seed + 1u);
  return fixture;
}

/// Single ground plane / open field: non-observable in translation-x,
/// translation-y, and yaw (rotation about the plane normal) simultaneously.
inline DegeneracyFixture makeSinglePlaneFixture(const SinglePlaneFixtureConfig & config)
{
  DegeneracyFixture fixture;
  fixture.name = "single_plane";

  std::vector<Eigen::Vector3d> points;
  std::vector<Eigen::Vector3d> normals;
  const double half_y = 0.5 * config.extent_y;
  for (double x = 0.0; x <= config.extent_x + 1e-9; x += config.grid_step) {
    for (double y = -half_y; y <= half_y + 1e-9; y += config.grid_step) {
      points.emplace_back(x, y, 0.0);
      normals.emplace_back(0.0, 0.0, 1.0);
    }
  }

  detail::appendCorrespondences(
    &fixture.correspondences, points, normals, config.wall_noise_sigma, config.seed);
  return fixture;
}

/// Point-to-plane Gauss-Newton (H, b) accumulated over a fixture's
/// correspondences, linearized at the given pose (identity by default).
struct GaussNewtonSystem
{
  Matrix6d h {Matrix6d::Zero()};
  Vector6d b {Vector6d::Zero()};
  int residual_count {0};
};

inline GaussNewtonSystem buildGaussNewtonSystem(
  const std::vector<PlaneCorrespondence> & correspondences,
  const Eigen::Matrix3d & rotation = Eigen::Matrix3d::Identity(),
  const Eigen::Vector3d & translation = Eigen::Vector3d::Zero())
{
  GaussNewtonSystem system;
  for (const PlaneCorrespondence & correspondence : correspondences) {
    const Eigen::Vector3d rotated_point = rotation * correspondence.point;
    const Eigen::Vector3d ideal_point =
      correspondence.point - correspondence.planar_offset * correspondence.normal;

    Vector6d jacobian;
    jacobian.head<3>() = correspondence.normal;
    jacobian.tail<3>() = rotated_point.cross(correspondence.normal);

    const double residual =
      correspondence.normal.dot(rotated_point + translation - ideal_point);

    system.h.noalias() += jacobian * jacobian.transpose();
    system.b.noalias() += jacobian * residual;
    ++system.residual_count;
  }
  return system;
}

/// Ascending eigenvalues + sign-canonicalized eigenvectors of a symmetric
/// 6x6 Gauss-Newton Hessian: the Phase 1 detector oracle representation.
struct EigenSignature
{
  Vector6d values {Vector6d::Zero()};
  Matrix6d vectors {Matrix6d::Zero()};
};

inline EigenSignature computeEigenSignature(const Matrix6d & h)
{
  // Symmetrize defensively: callers accumulate H as sum(J J^T), which is
  // exactly symmetric up to floating-point summation order, but
  // SelfAdjointEigenSolver only reads the lower triangle by default -- make
  // that explicit rather than relying on the caller's construction.
  const Matrix6d symmetric = 0.5 * (h + h.transpose());
  Eigen::SelfAdjointEigenSolver<Matrix6d> solver(symmetric);
  EigenSignature signature;
  signature.values = solver.eigenvalues();
  signature.vectors = solver.eigenvectors();
  for (int col = 0; col < 6; ++col) {
    int largest_index = 0;
    double largest_abs = std::abs(signature.vectors(0, col));
    for (int row = 1; row < 6; ++row) {
      const double component_abs = std::abs(signature.vectors(row, col));
      if (component_abs > largest_abs) {
        largest_abs = component_abs;
        largest_index = row;
      }
    }
    if (signature.vectors(largest_index, col) < 0.0) {
      signature.vectors.col(col) = -signature.vectors.col(col);
    }
  }
  return signature;
}

}  // namespace degeneracy
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__SYNTHETIC_DEGENERACY_FIXTURES_HPP_
