// Copyright 2026 Sasaki
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef GRAPH_BASED_SLAM__PLANE_REVISIT_EDGE_HPP_
#define GRAPH_BASED_SLAM__PLANE_REVISIT_EDGE_HPP_

#include <istream>
#include <ostream>

#include <Eigen/Core>  // NOLINT(build/include_order)
#include <Eigen/Geometry>  // NOLINT(build/include_order)

#include "g2o/core/base_binary_edge.h"
#include "g2o/types/slam3d/vertex_se3.h"

namespace graphslam
{
namespace pose_graph
{

struct LocalPlaneObservation
{
  Eigen::Vector3d normal {Eigen::Vector3d::UnitZ()};
  double offset {0.0};
  int support_points {0};
};

struct PlaneRevisitMeasurement
{
  LocalPlaneObservation from;
  LocalPlaneObservation to;
};

inline LocalPlaneObservation transformPlaneToWorld(
  const LocalPlaneObservation & local,
  const Eigen::Isometry3d & pose)
{
  LocalPlaneObservation world = local;
  world.normal = pose.rotation() * local.normal;
  world.normal.normalize();
  world.offset = local.offset - world.normal.dot(pose.translation());
  return world;
}

/// Binary plane-landmark re-observation factor.
///
/// The cross product contributes two effective normal-alignment constraints;
/// the signed offset contributes translation along the plane normal. Motion
/// tangent to the plane and rotation about its normal remain unconstrained,
/// matching the physical observability of one plane.
class PlaneRevisitEdge : public g2o::BaseBinaryEdge<
    4, PlaneRevisitMeasurement, g2o::VertexSE3, g2o::VertexSE3>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void computeError() override
  {
    const auto * from_vertex = static_cast<const g2o::VertexSE3 *>(_vertices[0]);
    const auto * to_vertex = static_cast<const g2o::VertexSE3 *>(_vertices[1]);
    const LocalPlaneObservation from_world =
      transformPlaneToWorld(_measurement.from, from_vertex->estimate());
    LocalPlaneObservation to_world =
      transformPlaneToWorld(_measurement.to, to_vertex->estimate());
    if (from_world.normal.dot(to_world.normal) < 0.0) {
      to_world.normal = -to_world.normal;
      to_world.offset = -to_world.offset;
    }
    _error.head<3>() = from_world.normal.cross(to_world.normal);
    _error(3) = from_world.offset - to_world.offset;
  }

  bool read(std::istream & input) override
  {
    input >> _measurement.from.normal.x() >> _measurement.from.normal.y() >>
    _measurement.from.normal.z() >> _measurement.from.offset >>
    _measurement.from.support_points >> _measurement.to.normal.x() >>
    _measurement.to.normal.y() >> _measurement.to.normal.z() >>
    _measurement.to.offset >> _measurement.to.support_points;
    for (int row = 0; row < 4; ++row) {
      for (int column = row; column < 4; ++column) {
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
    output << _measurement.from.normal.transpose() << ' ' <<
      _measurement.from.offset << ' ' << _measurement.from.support_points << ' ' <<
      _measurement.to.normal.transpose() << ' ' << _measurement.to.offset << ' ' <<
      _measurement.to.support_points;
    for (int row = 0; row < 4; ++row) {
      for (int column = row; column < 4; ++column) {
        output << ' ' << information()(row, column);
      }
    }
    return static_cast<bool>(output);
  }
};

}  // namespace pose_graph
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__PLANE_REVISIT_EDGE_HPP_
