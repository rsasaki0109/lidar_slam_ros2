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

#ifndef GRAPH_BASED_SLAM__SUBMAP_CREATION_HPP_
#define GRAPH_BASED_SLAM__SUBMAP_CREATION_HPP_

#include <Eigen/Core>

namespace graphslam
{
namespace submap_creation
{

struct Decision
{
  // Append a new submap at this pose.
  bool create {false};
  // The pose moved implausibly far since the last submap and was rejected.
  bool jump_rejected {false};
  // Distance from the previous submap position (0 for the very first submap).
  double distance {0.0};
};

// The submap spacing decision extracted verbatim from the odom-input path:
// the first valid pose always creates a submap; afterwards a pose creates one
// when it has moved at least `distance_threshold` since the previous submap,
// unless the step exceeds `max_jump` (teleport guard), which rejects the pose
// without consuming it as the new reference.
inline Decision evaluate(
  const Eigen::Vector3d & pos, bool last_position_valid,
  const Eigen::Vector3d & last_position, double distance_threshold,
  double max_jump = 100.0)
{
  Decision decision;
  if (!last_position_valid) {
    decision.create = true;
    return decision;
  }
  const double dist = (pos - last_position).norm();
  decision.distance = dist;
  if (dist < distance_threshold) {
    return decision;
  }
  if (dist > max_jump) {
    decision.jump_rejected = true;
    return decision;
  }
  decision.create = true;
  return decision;
}

}  // namespace submap_creation
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__SUBMAP_CREATION_HPP_
