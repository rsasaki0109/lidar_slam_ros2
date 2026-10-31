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

#ifndef GRAPH_BASED_SLAM__LOOP_EDGE_INFORMATION_HPP_
#define GRAPH_BASED_SLAM__LOOP_EDGE_INFORMATION_HPP_

// Loop edge information from the loop registration's Gauss-Newton Hessian
// (robust loops, bench_report/data/grandtour/loops_robust_2026-10-29/plan.txt).
// The tangent is a right perturbation X exp([w; v]) with the rotation first,
// as in small_gicp, whose Hessian H of a registration T is the one of
// T exp(delta).

#include <cmath>

#include <Eigen/Core>  // NOLINT(build/include_order)
#include <Eigen/Eigenvalues>  // NOLINT(build/include_order)
#include <Eigen/Geometry>  // NOLINT(build/include_order)

namespace graphslam
{
namespace loop_information
{

using Matrix6d = Eigen::Matrix<double, 6, 6>;

inline Eigen::Matrix3d skew(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(),
    v.z(), 0.0, -v.x(),
    -v.y(), v.x(), 0.0;
  return m;
}

// Ad_g with g exp(xi) g^-1 = exp(Ad_g xi) for xi = [w; v].
inline Matrix6d adjoint(const Eigen::Isometry3d & g)
{
  Matrix6d a = Matrix6d::Zero();
  const Eigen::Matrix3d r = g.rotation();
  a.topLeftCorner<3, 3>() = r;
  a.bottomRightCorner<3, 3>() = r;
  a.bottomLeftCorner<3, 3>() = skew(g.translation()) * r;
  return a;
}

// Inverse of a symmetric positive definite 6x6 matrix (a covariance or an
// information); false if it is not finite or not positive definite.
inline bool invertPositiveDefinite(const Matrix6d & m, Matrix6d * inverse)
{
  if (inverse == nullptr || !m.allFinite()) {
    return false;
  }
  const Eigen::SelfAdjointEigenSolver<Matrix6d> solver(0.5 * (m + m.transpose()));
  if (solver.info() != Eigen::Success || !(solver.eigenvalues().minCoeff() > 0.0)) {
    return false;
  }
  *inverse = solver.eigenvectors() * solver.eigenvalues().cwiseInverse().asDiagonal() *
    solver.eigenvectors().transpose();
  return inverse->allFinite();
}

// The loop's relative pose is Z = C^-1 T L (candidate pose C, latest submap
// pose L, registration T of the latest cloud in the world frame), so
// Z(delta) = C^-1 T exp(delta) L = Z exp(Ad_{L^-1} delta): with delta = Ad_L e
// the information of the edge's tangent e is Ad_L^T H Ad_L.
inline Matrix6d relativePoseInformation(
  const Eigen::Isometry3d & latest_pose, const Matrix6d & registration_hessian)
{
  const Matrix6d a = adjoint(latest_pose);
  return a.transpose() * registration_hessian * a;
}

// The same edge with its two submaps swapped (Z^-1): e' = -Ad_Z e.
inline Matrix6d inverseEdgeInformation(
  const Eigen::Isometry3d & relative_pose, const Matrix6d & information)
{
  const Matrix6d a = adjoint(relative_pose.inverse());
  return a.transpose() * information * a;
}

// In g2o EdgeSE3's error order [t; q.vec] (q.vec = w / 2 for a small error),
// with the translation block's and the rotation block's mean eigenvalue set
// to the constant loop information (trans, rot): the Hessian gives only the
// shape (which directions the registration constrains, and how they couple),
// the constants keep the weight. Zero if the information is not usable.
inline Matrix6d g2oShapedInformation(
  const Matrix6d & information, double constant_trans, double constant_rot)
{
  // [w; v] = J [t; q]: w = 2 q, v = t.
  Matrix6d j = Matrix6d::Zero();
  j.block<3, 3>(0, 3) = 2.0 * Eigen::Matrix3d::Identity();
  j.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity();
  const Matrix6d g = j.transpose() * information * j;
  const double mean_trans = g.topLeftCorner<3, 3>().trace() / 3.0;
  const double mean_rot = g.bottomRightCorner<3, 3>().trace() / 3.0;
  if (!g.allFinite() || !(mean_trans > 0.0) || !(mean_rot > 0.0) ||
    !(constant_trans > 0.0) || !(constant_rot > 0.0))
  {
    return Matrix6d::Zero();
  }
  Eigen::Matrix<double, 6, 1> d;
  d.head<3>().setConstant(std::sqrt(constant_trans / mean_trans));
  d.tail<3>().setConstant(std::sqrt(constant_rot / mean_rot));
  const Matrix6d shaped = d.asDiagonal() * g * d.asDiagonal();
  return 0.5 * (shaped + shaped.transpose());
}

}  // namespace loop_information
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__LOOP_EDGE_INFORMATION_HPP_
