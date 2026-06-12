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

#ifndef GRAPH_BASED_SLAM__POINT_CLUSTER_HPP_
#define GRAPH_BASED_SLAM__POINT_CLUSTER_HPP_

#include <cassert>
#include <cstdint>
#include <vector>

#include <Eigen/Core>  // NOLINT(build/include_order)
#include <Eigen/Eigenvalues>  // NOLINT(build/include_order)

#include "graph_based_slam/se3_lie.hpp"

namespace graphslam
{
namespace map_refinement
{

/**
 * @brief Homogeneous point-cluster coordinates for the plane BA.
 *
 * Encodes all raw points of one (feature, pose) pair as a symmetric 4x4
 * matrix so cost, gradient, and Hessian work over observing poses instead of
 * raw points. The eliminated plane cost is E_f = lambda_min(scatter), where
 * scatter is the unnormalized centroid scatter matrix.
 */
struct PointCluster
{
  Eigen::Matrix4d h {Eigen::Matrix4d::Zero()};  // sum [x;1][x;1]^T
  std::int64_t count {0};

  void add(const Eigen::Vector3d & p)
  {
    Eigen::Vector4d homogeneous;
    homogeneous << p, 1.0;
    h += homogeneous * homogeneous.transpose();
    ++count;
  }

  void merge(const PointCluster & other)
  {
    h += other.h;
    count += other.count;
  }

  PointCluster transformed(const Eigen::Matrix4d & pose) const
  {
    PointCluster result;
    result.h = pose * h * pose.transpose();
    result.count = count;
    return result;
  }

  Eigen::Matrix3d q() const
  {
    return h.topLeftCorner<3, 3>();
  }

  Eigen::Vector3d s() const
  {
    return h.topRightCorner<3, 1>();
  }

  double n() const
  {
    return h(3, 3);
  }

  Eigen::Vector3d centroid() const
  {
    assert(n() > 0.0);
    return s() / n();
  }

  Eigen::Matrix3d scatter() const
  {
    assert(n() > 0.0);
    const Eigen::Vector3d sum = s();
    return q() - (sum * sum.transpose()) / n();
  }

  Eigen::Matrix3d covariance() const
  {
    assert(n() > 0.0);
    return scatter() / n();
  }
};

/**
 * @brief Returns lambda_min(scatter), not lambda_min(covariance).
 *
 * The v0.7 design keeps the raw scatter form internally; normalized robust
 * loss layers divide by n separately.
 */
inline double minEigenvalueOfScatter(const PointCluster & cluster)
{
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cluster.scatter());
  assert(solver.info() == Eigen::Success);
  return solver.eigenvalues()(0);
}

inline PointCluster makeCluster(const std::vector<Eigen::Vector3d> & points)
{
  PointCluster cluster;
  for (std::size_t i = 0; i < points.size(); ++i) {
    cluster.add(points[i]);
  }
  return cluster;
}

inline Eigen::Matrix4d clusterFirstDerivative(
  const Eigen::Matrix4d & h,
  int r)
{
  const Eigen::Matrix4d & g_r = generator(r);
  return g_r * h + h * g_r.transpose();
}

inline Eigen::Matrix4d clusterSecondDerivative(
  const Eigen::Matrix4d & h,
  int r,
  int s)
{
  const int first = r <= s ? r : s;
  const int second = r <= s ? s : r;
  const Eigen::Matrix4d & g_r = generator(first);
  const Eigen::Matrix4d & g_s = generator(second);
  const Eigen::Matrix4d p_rs = 0.5 * (g_r * g_s + g_s * g_r);

  return p_rs * h + h * p_rs.transpose() +
         g_r * h * g_s.transpose() + g_s * h * g_r.transpose();
}

}  // namespace map_refinement
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__POINT_CLUSTER_HPP_
