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

// Robust kernel selection for loop closure edges in graph_based_slam.
//
// The bundled default is g2o::RobustKernelHuber, which clips the loop edge
// influence above a threshold but never drives it to zero. For solid-state
// LiDAR / aggressive false-loop regimes, Dynamic Covariance Scaling (DCS,
// Agarwal et al. 2013) is preferred because it analytically down-weights
// outlier loops toward zero without introducing extra optimisation
// variables. Cauchy is offered as a smoother middle ground.
//
// The parsing helpers live here so they are unit-testable without pulling
// in g2o headers. The factory ``makeLoopEdgeKernel`` is included only when
// g2o's RobustKernel headers are visible to keep this header lightweight
// for tests.

#ifndef GRAPH_BASED_SLAM__LOOP_EDGE_ROBUSTIFIER_HPP_
#define GRAPH_BASED_SLAM__LOOP_EDGE_ROBUSTIFIER_HPP_

#include <algorithm>
#include <cctype>
#include <string>

#if defined(GRAPH_BASED_SLAM_WITH_G2O)
#include <g2o/core/robust_kernel_impl.h>
#endif

namespace graphslam
{
namespace robust
{

enum class LoopEdgeKernelType
{
  Huber,
  DCS,
  Cauchy,
};

inline std::string toLowerAscii(const std::string & input)
{
  std::string out;
  out.reserve(input.size());
  for (char ch : input) {
    out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
  }
  return out;
}

// Parse a ROS-parameter string into the matching kernel enum. Unknown values
// fall back to Huber, matching the historical default.
inline LoopEdgeKernelType parseLoopEdgeKernelType(const std::string & raw)
{
  const std::string lower = toLowerAscii(raw);
  if (lower == "dcs") {return LoopEdgeKernelType::DCS;}
  if (lower == "cauchy") {return LoopEdgeKernelType::Cauchy;}
  return LoopEdgeKernelType::Huber;
}

// Reverse lookup used for logging and effective-parameter dumps.
inline const char * loopEdgeKernelTypeName(LoopEdgeKernelType type)
{
  switch (type) {
    case LoopEdgeKernelType::DCS:
      return "dcs";
    case LoopEdgeKernelType::Cauchy:
      return "cauchy";
    default:
      return "huber";
  }
}

#if defined(GRAPH_BASED_SLAM_WITH_G2O)
// Factory wrapper around g2o's robust kernels. The returned pointer is owned
// by whatever object it is later attached to (typically a g2o::OptimizableGraph
// edge via ``setRobustKernel``).
inline g2o::RobustKernel * makeLoopEdgeKernel(LoopEdgeKernelType type, double delta)
{
  g2o::RobustKernel * kernel = nullptr;
  switch (type) {
    case LoopEdgeKernelType::DCS:
      kernel = new g2o::RobustKernelDCS();
      break;
    case LoopEdgeKernelType::Cauchy:
      kernel = new g2o::RobustKernelCauchy();
      break;
    default:
      kernel = new g2o::RobustKernelHuber();
      break;
  }
  kernel->setDelta(delta);
  return kernel;
}
#endif  // GRAPH_BASED_SLAM_WITH_G2O

}  // namespace robust
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__LOOP_EDGE_ROBUSTIFIER_HPP_
