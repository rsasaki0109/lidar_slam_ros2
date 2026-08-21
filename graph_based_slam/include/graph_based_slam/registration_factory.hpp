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

#ifndef GRAPH_BASED_SLAM__REGISTRATION_FACTORY_HPP_
#define GRAPH_BASED_SLAM__REGISTRATION_FACTORY_HPP_

// The legacy GICP construction retained only for the explicit offline/live
// compatibility bridge.  NDT construction belongs to the typed host resolver
// and must not have a second direct PCL factory path.

#include <pcl/registration/registration.h>  // NOLINT(build/include_order)
#include <pclomp/gicp_omp.h>  // NOLINT(build/include_order)

#include <pclomp/gicp_omp_impl.hpp>

namespace graphslam
{
namespace backend_core
{

inline boost::shared_ptr<pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>>
makeLegacyGicpRegistration()
{
  boost::shared_ptr<pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>>
  gicp(new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
  gicp->setMaxCorrespondenceDistance(30);
  gicp->setMaximumIterations(100);
  gicp->setTransformationEpsilon(1e-8);
  gicp->setEuclideanFitnessEpsilon(1e-6);
  gicp->setRANSACIterations(0);
  return gicp;
}

}  // namespace backend_core
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__REGISTRATION_FACTORY_HPP_
