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

#ifndef GRAPH_BASED_SLAM__SE3_LIE_HPP_
#define GRAPH_BASED_SLAM__SE3_LIE_HPP_

#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>

#include <vector>

#include <Eigen/Core>  // NOLINT(build/include_order)

namespace graphslam
{
namespace map_refinement
{

using Vector6d = Eigen::Matrix<double, 6, 1>;

/**
 * @brief SE(3) utilities for the v0.7 plane BA.
 *
 * Math foundation for docs/roadmap/v0.7.md Phase 2 and
 * docs/research/map-refinement-clean-room-design.md. Twists use the order
 * [translation, rotation]. Pose increments are left perturbations:
 * T(delta) = Exp(delta) * T.
 */
namespace detail
{

constexpr double kSmallAngle = 1.0e-9;

inline Eigen::Vector3d vee(const Eigen::Matrix3d & matrix)
{
  Eigen::Vector3d vector;
  vector << matrix(2, 1), matrix(0, 2), matrix(1, 0);
  return vector;
}

}  // namespace detail

inline Eigen::Matrix3d skew(const Eigen::Vector3d & vector)
{
  Eigen::Matrix3d matrix;
  matrix << 0.0, -vector.z(), vector.y(),
    vector.z(), 0.0, -vector.x(),
    -vector.y(), vector.x(), 0.0;
  return matrix;
}

namespace detail
{

inline std::array<Eigen::Matrix4d, 6> makeGenerators()
{
  std::array<Eigen::Matrix4d, 6> generators;
  for (std::size_t i = 0; i < generators.size(); ++i) {
    generators[i].setZero();
  }

  generators[0](0, 3) = 1.0;
  generators[1](1, 3) = 1.0;
  generators[2](2, 3) = 1.0;

  generators[3].topLeftCorner<3, 3>() = skew(Eigen::Vector3d::UnitX());
  generators[4].topLeftCorner<3, 3>() = skew(Eigen::Vector3d::UnitY());
  generators[5].topLeftCorner<3, 3>() = skew(Eigen::Vector3d::UnitZ());

  return generators;
}

}  // namespace detail

inline Eigen::Matrix4d expSe3(const Vector6d & twist)
{
  const Eigen::Vector3d rho = twist.head<3>();
  const Eigen::Vector3d phi = twist.tail<3>();
  const double theta_squared = phi.squaredNorm();
  const double theta = std::sqrt(theta_squared);

  const Eigen::Matrix3d phi_hat = skew(phi);
  const Eigen::Matrix3d phi_hat_squared = phi_hat * phi_hat;

  double a = 0.0;
  double b = 0.0;
  double c = 0.0;

  if (theta < detail::kSmallAngle) {
    const double theta_fourth = theta_squared * theta_squared;
    a = 1.0 - theta_squared / 6.0 + theta_fourth / 120.0;
    b = 0.5 - theta_squared / 24.0 + theta_fourth / 720.0;
    c = 1.0 / 6.0 - theta_squared / 120.0 + theta_fourth / 5040.0;
  } else {
    a = std::sin(theta) / theta;
    b = (1.0 - std::cos(theta)) / theta_squared;
    c = (theta - std::sin(theta)) / (theta_squared * theta);
  }

  const Eigen::Matrix3d identity3 = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d rotation =
    identity3 + a * phi_hat + b * phi_hat_squared;
  const Eigen::Matrix3d v_matrix =
    identity3 + b * phi_hat + c * phi_hat_squared;

  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.topLeftCorner<3, 3>() = rotation;
  transform.topRightCorner<3, 1>() = v_matrix * rho;
  return transform;
}

inline Vector6d logSe3(const Eigen::Matrix4d & transform)
{
  const Eigen::Matrix3d rotation = transform.topLeftCorner<3, 3>();
  const Eigen::Vector3d translation = transform.topRightCorner<3, 1>();

  double cos_theta = 0.5 * (rotation.trace() - 1.0);
  if (cos_theta > 1.0) {
    cos_theta = 1.0;
  } else if (cos_theta < -1.0) {
    cos_theta = -1.0;
  }

  const double theta = std::acos(cos_theta);

  Eigen::Matrix3d phi_hat_from_rotation;
  if (theta < detail::kSmallAngle) {
    phi_hat_from_rotation = 0.5 * (rotation - rotation.transpose());
  } else {
    const double sin_theta = std::sin(theta);
    phi_hat_from_rotation =
      (theta / (2.0 * sin_theta)) * (rotation - rotation.transpose());
  }

  const Eigen::Vector3d phi = detail::vee(phi_hat_from_rotation);
  const Eigen::Matrix3d phi_hat = skew(phi);
  const Eigen::Matrix3d phi_hat_squared = phi_hat * phi_hat;
  const double phi_theta_squared = phi.squaredNorm();
  const double phi_theta = std::sqrt(phi_theta_squared);

  Eigen::Matrix3d v_inverse;
  if (phi_theta < detail::kSmallAngle) {
    v_inverse = Eigen::Matrix3d::Identity() - 0.5 * phi_hat +
      (1.0 / 12.0) * phi_hat_squared;
  } else {
    const double coefficient =
      (1.0 / phi_theta_squared) -
      ((1.0 + std::cos(phi_theta)) /
      (2.0 * phi_theta * std::sin(phi_theta)));
    v_inverse = Eigen::Matrix3d::Identity() - 0.5 * phi_hat +
      coefficient * phi_hat_squared;
  }

  Vector6d twist;
  twist.head<3>() = v_inverse * translation;
  twist.tail<3>() = phi;
  return twist;
}

inline const Eigen::Matrix4d & generator(int r)
{
  static const std::array<Eigen::Matrix4d, 6> generators =
    detail::makeGenerators();

  assert(r >= 0);
  assert(r < static_cast<int>(generators.size()));

  return generators[static_cast<std::size_t>(r)];
}

inline Eigen::Matrix4d leftPerturb(
  const Vector6d & delta,
  const Eigen::Matrix4d & pose)
{
  return expSe3(delta) * pose;
}

}  // namespace map_refinement
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__SE3_LIE_HPP_
