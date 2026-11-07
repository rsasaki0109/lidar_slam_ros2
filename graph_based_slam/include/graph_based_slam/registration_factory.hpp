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

// The loop-closure registration construction shared by the ROS component
// and the offline deterministic runner, so both verify candidates with
// byte-identical registration settings (docs/roadmap/v0.6.md, Phase 2).

#include <pcl/memory.h>  // NOLINT(build/include_order)
#include <pcl/registration/registration.h>  // NOLINT(build/include_order)
#include <pclomp/gicp_omp.h>  // NOLINT(build/include_order)
#include <pclomp/ndt_omp.h>  // NOLINT(build/include_order)

#include <cstddef>
#include <list>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <pcl/search/kdtree.h>  // NOLINT(build/include_order)
#include <pclomp/gicp_omp_impl.hpp>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>

#ifdef GRAPH_BASED_SLAM_HAVE_SMALL_GICP
#include <small_gicp/pcl/pcl_registration.hpp>  // NOLINT(build/include_order)
#include <small_gicp/pcl/pcl_registration_impl.hpp>  // NOLINT(build/include_order)
#endif

namespace graphslam
{
namespace backend_core
{

#ifdef GRAPH_BASED_SLAM_HAVE_SMALL_GICP
// small_gicp's PCL registration that keeps the target state of its last few targets.
//
// Loop search registers the queries of consecutive submaps against the same few target
// aggregates: TargetCloudCache hands out the same cloud object while the target's submaps keep
// their poses. small_gicp rebuilds its KD tree and target covariances, and PCL the KD tree of
// getFitnessScore(), whenever the target changes, so alternating between candidates paid for
// them on almost every registration. This keeps them for the last kTargetCapacity targets, keyed
// by the cloud object. An entry owns its cloud, so the address cannot be reused while the entry
// lives, and the kept trees and covariances are the ones a rebuild gives: results do not change.
// As small_gicp and PCL already assume for a repeated target pointer, a target cloud must not
// change in place after it is set.
template<typename PointT>
class TargetCachingSmallGicp : public small_gicp::RegistrationPCL<PointT, PointT>
{
public:
  using Base = small_gicp::RegistrationPCL<PointT, PointT>;
  using PclRegistration = pcl::Registration<PointT, PointT, float>;
  using PointCloudTargetConstPtr = typename Base::PointCloudTargetConstPtr;
  // Four targets hold nearly every repeat; a query tries up to three candidates (REPORT ch. 51).
  static constexpr std::size_t kTargetCapacity = 4U;

  void setInputTarget(const PointCloudTargetConstPtr & cloud) override
  {
    if (this->target_ == cloud) {
      return;
    }
    keepCurrentTarget();
    for (auto it = kept_.begin(); it != kept_.end(); ++it) {
      if (it->cloud == cloud) {
        PclRegistration::setInputTarget(cloud);
        this->target_tree_ = it->target_tree;
        this->target_covs_ = it->target_covs;
        this->target_voxelmap_.reset();
        this->tree_ = it->fitness_tree;
        this->target_cloud_updated_ = false;
        kept_.splice(kept_.begin(), kept_, it);
        return;
      }
    }
    Base::setInputTarget(cloud);
    // A new tree object: initCompute() builds it for this target and a kept tree stays as it is.
    this->tree_.reset(new pcl::search::KdTree<PointT>);
  }

  std::size_t keptTargetCount() const {return kept_.size();}

private:
  struct KeptTarget
  {
    PointCloudTargetConstPtr cloud;
    std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<PointT>>> target_tree;
    std::vector<Eigen::Matrix4d> target_covs;
    typename PclRegistration::KdTreePtr fitness_tree;
  };

  // Keeps the current target once an alignment has built all of its state.
  void keepCurrentTarget()
  {
    if (!this->target_ || this->target_cloud_updated_ || !this->target_tree_ ||
      this->target_covs_.size() != this->target_->size())
    {
      return;
    }
    for (auto it = kept_.begin(); it != kept_.end(); ++it) {
      if (it->cloud == this->target_) {
        kept_.splice(kept_.begin(), kept_, it);
        return;
      }
    }
    kept_.push_front(
      KeptTarget{this->target_, this->target_tree_, std::move(this->target_covs_), this->tree_});
    this->target_covs_.clear();
    if (kept_.size() > kTargetCapacity) {
      kept_.pop_back();
    }
  }

  std::list<KeptTarget> kept_;
};
#endif

// Returns nullptr for an unknown method; the caller decides how to fail.
inline pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr
makeLoopRegistration(
  const std::string & method, double ndt_resolution, int ndt_num_threads,
  double gicp_max_correspondence_distance = 30.0, double gicp_transformation_epsilon = 1e-8)
{
  if (method == "NDT") {
    auto ndt = pcl::make_shared<
      pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>>();
    ndt->setMaximumIterations(100);
    ndt->setResolution(ndt_resolution);
    ndt->setTransformationEpsilon(0.01);
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    if (ndt_num_threads > 0) {ndt->setNumThreads(ndt_num_threads);}
    return ndt;
  }
  if (method == "GICP") {
    auto gicp = pcl::make_shared<
      pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>>();
    gicp->setMaxCorrespondenceDistance(gicp_max_correspondence_distance);
    gicp->setMaximumIterations(100);
    gicp->setTransformationEpsilon(gicp_transformation_epsilon);
    gicp->setEuclideanFitnessEpsilon(1e-6);
    gicp->setRANSACIterations(0);
    return gicp;
  }
#ifdef GRAPH_BASED_SLAM_HAVE_SMALL_GICP
  // small_gicp's GICP (Gauss-Newton). Its epsilon is a step size: it stops when an update moves
  // less than gicp_transformation_epsilon in translation (m) and in rotation (rad); an
  // alignment that has not stopped within 100 iterations is reported as not converged.
  // ndt_num_threads is its thread count as for NDT (one thread: a fixed reduction order).
  if (method == "SMALL_GICP") {
    auto gicp = pcl::make_shared<TargetCachingSmallGicp<pcl::PointXYZI>>();
    gicp->setRegistrationType("GICP");
    gicp->setMaxCorrespondenceDistance(gicp_max_correspondence_distance);
    gicp->setMaximumIterations(100);
    gicp->setTransformationEpsilon(gicp_transformation_epsilon);
    gicp->setRotationEpsilon(gicp_transformation_epsilon);
    if (ndt_num_threads > 0) {gicp->setNumThreads(ndt_num_threads);}
    return gicp;
  }
#endif
  return nullptr;
}

// Legacy host-built GICP used by the offline runner's compatibility bridge.
// Kept separate so the numeric defaults exactly match the historical PCL
// construction that the plugin API documents.
inline pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr
makeLegacyGicpRegistration(
  double max_correspondence_distance = 30.0, double transformation_epsilon = 1e-8)
{
  auto gicp = pcl::make_shared<
    pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>>();
  gicp->setMaxCorrespondenceDistance(max_correspondence_distance);
  gicp->setMaximumIterations(100);
  gicp->setTransformationEpsilon(transformation_epsilon);
  gicp->setEuclideanFitnessEpsilon(1e-6);
  gicp->setRANSACIterations(0);
  return gicp;
}

}  // namespace backend_core
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__REGISTRATION_FACTORY_HPP_
