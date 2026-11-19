// Copyright 2026 lidarslam_ros2 contributors
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
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the
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

#ifndef GRAPH_BASED_SLAM__MAP_CHANGE_EVIDENCE_HPP_
#define GRAPH_BASED_SLAM__MAP_CHANGE_EVIDENCE_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace graphslam::map_change
{

constexpr double voxel_m = 0.15;
constexpr double ray_margin_m = 0.30;
using Point = std::array<double, 3>;
using Voxel = std::array<int, 3>;

inline bool usable(const Point & point)
{
  return std::all_of(point.begin(), point.end(), [](double v) {
           return std::isfinite(v) && std::abs(v) < 1e6;
         });
}

inline Voxel voxel(const Point & point)
{
  return {static_cast<int>(std::floor(point[0] / voxel_m)),
    static_cast<int>(std::floor(point[1] / voxel_m)),
    static_cast<int>(std::floor(point[2] / voxel_m))};
}

struct VoxelHash
{
  std::size_t operator()(const Voxel & key) const
  {
    return (std::uint64_t(static_cast<std::uint32_t>(key[0])) * 73856093u) ^
           (std::uint64_t(static_cast<std::uint32_t>(key[1])) * 19349663u) ^
           (std::uint64_t(static_cast<std::uint32_t>(key[2])) * 83492791u);
  }
};

inline double squaredDistance(const Point & a, const Point & b)
{
  const double x = a[0] - b[0];
  const double y = a[1] - b[1];
  const double z = a[2] - b[2];
  return x * x + y * y + z * z;
}

// Visit world-grid cells strictly before the endpoint margin. At a corner or
// edge, advance all tied axes so zero-length intersections do not count.
template<typename Visit>
void traceFree(const Point & origin, const Point & hit, Visit visit)
{
  const double length = std::sqrt(squaredDistance(origin, hit));
  if (length <= ray_margin_m) {
    return;
  }
  const double limit = 1.0 - ray_margin_m / length;
  Voxel cell = voxel(origin);
  const Voxel endpoint = voxel(hit);
  Voxel step{};
  Point crossing{}, stride{};
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const double component = hit[axis] - origin[axis];
    if (component == 0.0) {
      crossing[axis] = std::numeric_limits<double>::infinity();
      continue;
    }
    step[axis] = component > 0.0 ? 1 : -1;
    const int boundary = cell[axis] + (component > 0.0 ? 1 : 0);
    crossing[axis] = (boundary * voxel_m - origin[axis]) / component;
    stride[axis] = voxel_m / std::abs(component);
  }
  double entered = 0.0;
  while (entered < limit && cell != endpoint) {
    const double left = *std::min_element(crossing.begin(), crossing.end());
    if (left > entered) {
      visit(cell);
    }
    if (left >= limit) {
      break;
    }
    for (std::size_t axis = 0; axis < 3; ++axis) {
      if (crossing[axis] <= left + 1e-12) {
        cell[axis] += step[axis];
        crossing[axis] += stride[axis];
      }
    }
    entered = left;
  }
}

}  // namespace graphslam::map_change
#endif  // GRAPH_BASED_SLAM__MAP_CHANGE_EVIDENCE_HPP_
