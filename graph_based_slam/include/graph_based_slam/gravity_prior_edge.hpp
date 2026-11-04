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

#ifndef GRAPH_BASED_SLAM__GRAVITY_PRIOR_EDGE_HPP_
#define GRAPH_BASED_SLAM__GRAVITY_PRIOR_EDGE_HPP_

// A LiDAR-inertial frontend measures gravity, so each submap's roll and pitch
// in its (gravity-aligned) odometry frame are absolute observations. The
// adjacent edges only carry relative rotations, and weakly, so position
// anchors such as GNSS can tilt the graph to fit them (and an extrapolated
// tilt turns into height error after the last anchor). This unary factor
// keeps a node's "up" direction, expressed in the body frame, at the
// odometry's: error = R^T z_world - up_body. Rotation about the vertical
// (yaw) leaves the error unchanged, so it stays free.

#include <istream>
#include <ostream>

#include <Eigen/Core>  // NOLINT(build/include_order)

#include "g2o/core/base_unary_edge.h"
#include "g2o/types/slam3d/vertex_se3.h"

namespace graphslam
{
namespace pose_graph
{

class GravityPriorEdge : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void computeError() override
  {
    const auto * vertex = static_cast<const g2o::VertexSE3 *>(_vertices[0]);
    _error = vertex->estimate().linear().transpose() * Eigen::Vector3d::UnitZ() - _measurement;
  }

  bool read(std::istream & input) override
  {
    input >> _measurement.x() >> _measurement.y() >> _measurement.z();
    for (int row = 0; row < 3; ++row) {
      for (int column = row; column < 3; ++column) {
        input >> information()(row, column);
        if (row != column) {
          information()(column, row) = information()(row, column);
        }
      }
    }
    return static_cast<bool>(input);
  }

  bool write(std::ostream & output) const override
  {
    output << _measurement.transpose();
    for (int row = 0; row < 3; ++row) {
      for (int column = row; column < 3; ++column) {
        output << ' ' << information()(row, column);
      }
    }
    return static_cast<bool>(output);
  }
};

}  // namespace pose_graph
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__GRAVITY_PRIOR_EDGE_HPP_
