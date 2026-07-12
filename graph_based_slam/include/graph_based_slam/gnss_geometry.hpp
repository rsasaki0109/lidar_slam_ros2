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

#ifndef GRAPH_BASED_SLAM__GNSS_GEOMETRY_HPP_
#define GRAPH_BASED_SLAM__GNSS_GEOMETRY_HPP_

#include <cmath>

#include <Eigen/Core>  // NOLINT(build/include_order)

namespace graphslam
{
namespace detail
{

struct GeodeticOrigin
{
  double latitude_deg {0.0};
  double longitude_deg {0.0};
  double altitude_m {0.0};
};

inline bool isUsableGeodeticFix(double latitude_deg, double longitude_deg, double altitude_m)
{
  if (!std::isfinite(latitude_deg) || !std::isfinite(longitude_deg) ||
    !std::isfinite(altitude_m))
  {
    return false;
  }
  if (latitude_deg < -90.0 || latitude_deg > 90.0) {
    return false;
  }
  if (longitude_deg < -180.0 || longitude_deg > 180.0) {
    return false;
  }
  return std::abs(latitude_deg) >= 1e-6 || std::abs(longitude_deg) >= 1e-6;
}

inline double approximateGeodeticDistanceMeters(
  double latitude_0_deg, double longitude_0_deg,
  double latitude_1_deg, double longitude_1_deg)
{
  constexpr double kEarthRadiusM = 6378137.0;
  constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;
  const double latitude_0_rad = latitude_0_deg * kDegreesToRadians;
  const double latitude_1_rad = latitude_1_deg * kDegreesToRadians;
  const double delta_latitude = latitude_1_rad - latitude_0_rad;
  const double delta_longitude =
    (longitude_1_deg - longitude_0_deg) * kDegreesToRadians;
  const double x =
    delta_longitude * std::cos((latitude_0_rad + latitude_1_rad) * 0.5);
  return std::hypot(x, delta_latitude) * kEarthRadiusM;
}

inline Eigen::Vector3d geodeticToEnu(
  double latitude_deg, double longitude_deg, double altitude_m,
  const GeodeticOrigin & origin)
{
  constexpr double kSemiMajorAxisM = 6378137.0;
  constexpr double kFlattening = 1.0 / 298.257223563;
  constexpr double kEccentricitySquared =
    2.0 * kFlattening - kFlattening * kFlattening;
  constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;

  const double origin_latitude_rad = origin.latitude_deg * kDegreesToRadians;
  const double delta_latitude =
    (latitude_deg - origin.latitude_deg) * kDegreesToRadians;
  const double delta_longitude =
    (longitude_deg - origin.longitude_deg) * kDegreesToRadians;
  const double sin_origin_latitude = std::sin(origin_latitude_rad);
  const double curvature = 1.0 -
    kEccentricitySquared * sin_origin_latitude * sin_origin_latitude;
  const double prime_vertical_radius = kSemiMajorAxisM / std::sqrt(curvature);
  const double meridian_radius =
    kSemiMajorAxisM * (1.0 - kEccentricitySquared) / std::pow(curvature, 1.5);

  return Eigen::Vector3d(
    delta_longitude * prime_vertical_radius * std::cos(origin_latitude_rad),
    delta_latitude * meridian_radius,
    altitude_m - origin.altitude_m);
}

}  // namespace detail
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__GNSS_GEOMETRY_HPP_
