#ifndef REGISTRATION_FACTORY_HPP_
#define REGISTRATION_FACTORY_HPP_

#include <string>
#include <memory>
#include <stdexcept>

#include <boost/make_shared.hpp>

#include <pcl/point_types.h>
#include <pcl/registration/registration.h>

#include <pclomp/ndt_omp.h>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp.h>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>
#include <pclomp/gicp_omp.h>
#include <pclomp/gicp_omp_impl.hpp>

namespace graphslam
{

struct RegistrationParams
{
  std::string method = "NDT";
  double ndt_resolution = 5.0;
  int ndt_num_threads = 0;
  double gicp_corr_dist_threshold = 5.0;

  // Optional: for loop closure (graph_based_slam)
  int max_iterations = 0;  // 0 means use default
  double euclidean_fitness_epsilon = 0.0;  // 0 means use default
  int ransac_iterations = -1;  // -1 means use default
};

class RegistrationFactory
{
public:
  using PointT = pcl::PointXYZI;
  using RegistrationPtr = boost::shared_ptr<pcl::Registration<PointT, PointT>>;

  static RegistrationPtr create(const RegistrationParams& params)
  {
    if (params.method == "NDT") {
      return createNDT(params);
    } else if (params.method == "GICP") {
      return createGICP(params);
    } else {
      throw std::invalid_argument("Invalid registration method: " + params.method);
    }
  }

private:
  static RegistrationPtr createNDT(const RegistrationParams& params)
  {
    auto ndt = boost::make_shared<pclomp::NormalDistributionsTransform<PointT, PointT>>();
    ndt->setResolution(params.ndt_resolution);
    ndt->setTransformationEpsilon(0.01);
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    if (params.ndt_num_threads > 0) {
      ndt->setNumThreads(params.ndt_num_threads);
    }
    if (params.max_iterations > 0) {
      ndt->setMaximumIterations(params.max_iterations);
    }
    return ndt;
  }

  static RegistrationPtr createGICP(const RegistrationParams& params)
  {
    auto gicp = boost::make_shared<pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>>();
    gicp->setMaxCorrespondenceDistance(params.gicp_corr_dist_threshold);
    gicp->setTransformationEpsilon(1e-8);
    if (params.max_iterations > 0) {
      gicp->setMaximumIterations(params.max_iterations);
    }
    if (params.euclidean_fitness_epsilon > 0) {
      gicp->setEuclideanFitnessEpsilon(params.euclidean_fitness_epsilon);
    }
    if (params.ransac_iterations >= 0) {
      gicp->setRANSACIterations(params.ransac_iterations);
    }
    return gicp;
  }
};

}  // namespace graphslam

#endif  // REGISTRATION_FACTORY_HPP_
