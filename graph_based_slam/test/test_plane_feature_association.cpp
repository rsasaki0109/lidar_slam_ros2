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

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <random>
#include <vector>

#include <Eigen/Core>  // NOLINT(build/include_order)
#include <Eigen/Geometry>  // NOLINT(build/include_order)

#include "graph_based_slam/plane_ba.hpp"
#include "graph_based_slam/plane_feature_association.hpp"

namespace
{

namespace map_refinement = graphslam::map_refinement;
namespace plane_extraction = graphslam::plane_extraction;

Eigen::Matrix4d makePose(
  const double yaw,
  const Eigen::Vector3d & translation)
{
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  pose.block<3, 3>(0, 0) =
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  pose.block<3, 1>(0, 3) = translation;
  return pose;
}

Eigen::Vector3d transformPoint(
  const Eigen::Matrix4d & pose,
  const Eigen::Vector3d & point)
{
  const Eigen::Vector4d homogeneous(
    point.x(),
    point.y(),
    point.z(),
    1.0);
  return (pose * homogeneous).head<3>();
}

Eigen::Vector3d inverseTransformPoint(
  const Eigen::Matrix4d & pose,
  const Eigen::Vector3d & point)
{
  const Eigen::Matrix3d rotation = pose.block<3, 3>(0, 0);
  const Eigen::Vector3d translation = pose.block<3, 1>(0, 3);
  return rotation.transpose() * (point - translation);
}

std::vector<Eigen::Vector3d> toLocalCloud(
  const std::vector<Eigen::Vector3d> & world_points,
  const Eigen::Matrix4d & pose)
{
  std::vector<Eigen::Vector3d> local_points;
  local_points.reserve(world_points.size());
  for (std::size_t i = 0; i < world_points.size(); ++i) {
    local_points.push_back(inverseTransformPoint(pose, world_points[i]));
  }
  return local_points;
}

std::vector<Eigen::Vector3d> makeFloorPoints(
  const int side_count,
  const double spacing,
  const double z_sigma,
  std::mt19937 * rng)
{
  std::normal_distribution<double> noise(0.0, z_sigma);
  std::vector<Eigen::Vector3d> points;
  points.reserve(static_cast<std::size_t>(side_count * side_count));

  for (int y_index = 0; y_index < side_count; ++y_index) {
    for (int x_index = 0; x_index < side_count; ++x_index) {
      points.push_back(
        Eigen::Vector3d(
          0.25 + spacing * static_cast<double>(x_index),
          0.25 + spacing * static_cast<double>(y_index),
          noise(*rng)));
    }
  }
  return points;
}

std::vector<Eigen::Vector3d> makeXWallPoints(
  const double x,
  const int side_count,
  const double spacing)
{
  std::vector<Eigen::Vector3d> points;
  points.reserve(static_cast<std::size_t>(side_count * side_count));

  for (int z_index = 0; z_index < side_count; ++z_index) {
    for (int y_index = 0; y_index < side_count; ++y_index) {
      points.push_back(
        Eigen::Vector3d(
          x,
          0.25 + spacing * static_cast<double>(y_index),
          0.25 + spacing * static_cast<double>(z_index)));
    }
  }
  return points;
}

std::vector<Eigen::Vector3d> makeYWallPoints(
  const double y,
  const int side_count,
  const double spacing)
{
  std::vector<Eigen::Vector3d> points;
  points.reserve(static_cast<std::size_t>(side_count * side_count));

  for (int z_index = 0; z_index < side_count; ++z_index) {
    for (int x_index = 0; x_index < side_count; ++x_index) {
      points.push_back(
        Eigen::Vector3d(
          0.25 + spacing * static_cast<double>(x_index),
          y,
          0.25 + spacing * static_cast<double>(z_index)));
    }
  }
  return points;
}

void appendPoints(
  const std::vector<Eigen::Vector3d> & source,
  std::vector<Eigen::Vector3d> * target)
{
  target->insert(target->end(), source.begin(), source.end());
}

std::vector<Eigen::Vector3d> buildWorldCloud(
  const std::vector<std::vector<Eigen::Vector3d>> & local_clouds,
  const std::vector<Eigen::Matrix4d> & poses)
{
  std::vector<Eigen::Vector3d> world_cloud;
  for (std::size_t pose_index = 0; pose_index < local_clouds.size(); ++pose_index) {
    for (std::size_t point_index = 0; point_index < local_clouds[pose_index].size();
      ++point_index)
    {
      world_cloud.push_back(
        transformPoint(
          poses[pose_index],
          local_clouds[pose_index][point_index]));
    }
  }
  return world_cloud;
}

plane_extraction::PlaneExtractionConfig makeExtractionConfig()
{
  plane_extraction::PlaneExtractionConfig config;
  config.root_voxel_size = 8.0;
  config.max_octree_depth = 0;
  config.min_points_per_plane = 20;
  config.max_plane_thickness = 0.04;
  config.min_planarity_ratio = 4.0;
  config.enable_quarter_test = false;
  return config;
}

map_refinement::AssociationConfig makeAssociationConfig()
{
  map_refinement::AssociationConfig config;
  config.extraction = makeExtractionConfig();
  config.min_observing_poses = 2;
  config.min_points_per_observation = 5;
  return config;
}

Eigen::Vector3d clusterCentroid(
  const map_refinement::PointCluster & cluster)
{
  const double count = cluster.h(3, 3);
  if (count <= 0.0) {
    return Eigen::Vector3d::Zero();
  }
  return cluster.h.block<3, 1>(0, 3) / count;
}

double relativeTranslationError(
  const std::vector<Eigen::Matrix4d> & poses,
  const std::vector<Eigen::Matrix4d> & ground_truth)
{
  const Eigen::Matrix4d relative = poses[0].inverse() * poses[1];
  const Eigen::Matrix4d relative_ground_truth = ground_truth[0].inverse() * ground_truth[1];
  return (
    relative.block<3, 1>(0, 3) -
    relative_ground_truth.block<3, 1>(0, 3)).norm();
}

}  // namespace

TEST(PlaneFeatureAssociation, SharedNoisyFloorCreatesTwoPoseFeatures)
{
  std::mt19937 rng(7);
  const std::vector<Eigen::Vector3d> floor_points =
    makeFloorPoints(12, 0.12, 0.004, &rng);

  std::vector<Eigen::Matrix4d> poses;
  poses.push_back(makePose(0.0, Eigen::Vector3d::Zero()));
  poses.push_back(makePose(0.25, Eigen::Vector3d(0.4, -0.15, 0.08)));

  std::vector<std::vector<Eigen::Vector3d>> local_clouds;
  local_clouds.push_back(toLocalCloud(floor_points, poses[0]));
  local_clouds.push_back(toLocalCloud(floor_points, poses[1]));

  const map_refinement::AssociationConfig config = makeAssociationConfig();
  const map_refinement::AssociationResult association =
    map_refinement::associatePlaneFeatures(local_clouds, poses, config);

  ASSERT_FALSE(association.features.empty());
  for (std::size_t i = 0; i < association.features.size(); ++i) {
    EXPECT_EQ(2U, association.features[i].observations.size());
  }

  plane_extraction::PlaneExtractionConfig extraction_config = config.extraction;
  extraction_config.collect_point_indices = true;
  const plane_extraction::PlaneExtractionResult extraction =
    plane_extraction::extractPlanarPatches(
    buildWorldCloud(local_clouds, poses),
    extraction_config);

  ASSERT_GE(extraction.patches.size(), association.features.size());
  for (std::size_t feature_index = 0; feature_index < association.features.size();
    ++feature_index)
  {
    const Eigen::Vector3d patch_centroid = extraction.patches[feature_index].centroid;
    const map_refinement::PlaneFeature & feature = association.features[feature_index];

    for (std::size_t observation_index = 0; observation_index < feature.observations.size();
      ++observation_index)
    {
      const map_refinement::PlaneFeatureObservation & observation =
        feature.observations[observation_index];
      const Eigen::Vector3d world_centroid =
        transformPoint(
          poses[static_cast<std::size_t>(observation.pose_index)],
          clusterCentroid(observation.local_cluster));
      EXPECT_LT((world_centroid - patch_centroid).norm(), 0.1);
    }
  }
}

TEST(PlaneFeatureAssociation, SinglePosePatchIsFiltered)
{
  std::vector<Eigen::Matrix4d> poses;
  poses.push_back(Eigen::Matrix4d::Identity());
  poses.push_back(Eigen::Matrix4d::Identity());

  std::vector<std::vector<Eigen::Vector3d>> local_clouds(2);
  local_clouds[0] = makeXWallPoints(0.0, 8, 0.12);

  map_refinement::AssociationConfig config = makeAssociationConfig();
  config.min_observing_poses = 2;

  const map_refinement::AssociationResult association =
    map_refinement::associatePlaneFeatures(local_clouds, poses, config);

  EXPECT_GT(association.patches_total, association.patches_used);
  EXPECT_TRUE(association.features.empty());
}

TEST(PlaneFeatureAssociation, AssociatedFeaturesImprovePlaneBaPose)
{
  std::vector<Eigen::Vector3d> world_points;
  appendPoints(makeFloorPoints(8, 0.2, 0.0, new std::mt19937(1)), &world_points);
  appendPoints(makeXWallPoints(4.0, 8, 0.2), &world_points);
  appendPoints(makeYWallPoints(4.0, 8, 0.2), &world_points);

  std::vector<Eigen::Matrix4d> ground_truth;
  ground_truth.push_back(Eigen::Matrix4d::Identity());
  ground_truth.push_back(makePose(0.18, Eigen::Vector3d(0.25, -0.18, 0.12)));

  std::vector<std::vector<Eigen::Vector3d>> local_clouds;
  local_clouds.push_back(toLocalCloud(world_points, ground_truth[0]));
  local_clouds.push_back(toLocalCloud(world_points, ground_truth[1]));

  std::vector<Eigen::Matrix4d> initial_poses = ground_truth;
  initial_poses[1](0, 3) += 0.03;

  map_refinement::AssociationConfig association_config = makeAssociationConfig();
  association_config.extraction.root_voxel_size = 3.0;
  association_config.extraction.max_plane_thickness = 0.08;

  const map_refinement::AssociationResult association =
    map_refinement::associatePlaneFeatures(
      local_clouds,
      initial_poses,
      association_config);
  ASSERT_GE(association.features.size(), 3U);

  map_refinement::PlaneBaConfig ba_config;
  ba_config.prior_translation_sigma = 0.0;
  ba_config.max_iterations = 30;

  const double initial_error = relativeTranslationError(initial_poses, ground_truth);
  const map_refinement::PlaneBaResult ba_result =
    map_refinement::solvePlaneBa(association.features, initial_poses, ba_config);
  const std::vector<Eigen::Matrix4d> & refined_poses = ba_result.poses;
  ASSERT_EQ(initial_poses.size(), refined_poses.size());

  const double refined_error = relativeTranslationError(refined_poses, ground_truth);
  EXPECT_LT(refined_error, initial_error);
  EXPECT_LT(refined_error, 0.5 * initial_error);
}

TEST(PlaneFeatureAssociation, MinPointsPerObservationDropsSparseObservations)
{
  std::vector<Eigen::Vector3d> dense_floor;
  std::mt19937 rng(11);
  dense_floor = makeFloorPoints(7, 0.15, 0.0, &rng);

  std::vector<Eigen::Vector3d> sparse_floor;
  sparse_floor.push_back(dense_floor[0]);
  sparse_floor.push_back(dense_floor[1]);
  sparse_floor.push_back(dense_floor[2]);

  std::vector<Eigen::Matrix4d> poses;
  poses.push_back(Eigen::Matrix4d::Identity());
  poses.push_back(Eigen::Matrix4d::Identity());

  std::vector<std::vector<Eigen::Vector3d>> local_clouds;
  local_clouds.push_back(dense_floor);
  local_clouds.push_back(sparse_floor);

  map_refinement::AssociationConfig config = makeAssociationConfig();
  config.min_observing_poses = 1;
  config.min_points_per_observation = 5;

  const map_refinement::AssociationResult association =
    map_refinement::associatePlaneFeatures(local_clouds, poses, config);

  ASSERT_FALSE(association.features.empty());
  for (std::size_t i = 0; i < association.features.size(); ++i) {
    ASSERT_EQ(1U, association.features[i].observations.size());
    EXPECT_EQ(0, association.features[i].observations[0].pose_index);
    EXPECT_NEAR(49.0, association.features[i].observations[0].local_cluster.h(3, 3), 1.0e-9);
  }
}

TEST(PlaneFeatureAssociation, DeterministicFirstFeatureClusterH)
{
  std::mt19937 rng(13);
  const std::vector<Eigen::Vector3d> floor_points =
    makeFloorPoints(9, 0.14, 0.002, &rng);

  std::vector<Eigen::Matrix4d> poses;
  poses.push_back(Eigen::Matrix4d::Identity());
  poses.push_back(makePose(-0.2, Eigen::Vector3d(0.3, 0.1, 0.05)));

  std::vector<std::vector<Eigen::Vector3d>> local_clouds;
  local_clouds.push_back(toLocalCloud(floor_points, poses[0]));
  local_clouds.push_back(toLocalCloud(floor_points, poses[1]));

  const map_refinement::AssociationConfig config = makeAssociationConfig();
  const map_refinement::AssociationResult first =
    map_refinement::associatePlaneFeatures(local_clouds, poses, config);
  const map_refinement::AssociationResult second =
    map_refinement::associatePlaneFeatures(local_clouds, poses, config);

  ASSERT_FALSE(first.features.empty());
  ASSERT_FALSE(second.features.empty());
  ASSERT_FALSE(first.features[0].observations.empty());
  ASSERT_FALSE(second.features[0].observations.empty());
  EXPECT_EQ(first.features.size(), second.features.size());

  const map_refinement::PointCluster & first_cluster =
    first.features[0].observations[0].local_cluster;
  const map_refinement::PointCluster & second_cluster =
    second.features[0].observations[0].local_cluster;
  EXPECT_EQ(
    0,
    std::memcmp(first_cluster.h.data(), second_cluster.h.data(), sizeof(double) * 16U));
}

TEST(PlaneFeatureAssociation, ExtractorCollectPointIndicesDoesNotChangeAcceptance)
{
  std::mt19937 rng(17);
  const std::vector<Eigen::Vector3d> floor_points =
    makeFloorPoints(10, 0.1, 0.003, &rng);

  plane_extraction::PlaneExtractionConfig without_indices = makeExtractionConfig();
  plane_extraction::PlaneExtractionConfig with_indices = without_indices;
  with_indices.collect_point_indices = true;

  const plane_extraction::PlaneExtractionResult without_result =
    plane_extraction::extractPlanarPatches(floor_points, without_indices);
  const plane_extraction::PlaneExtractionResult with_result =
    plane_extraction::extractPlanarPatches(floor_points, with_indices);

  EXPECT_TRUE(without_result.patch_point_indices.empty());
  ASSERT_EQ(without_result.patches.size(), with_result.patches.size());
  ASSERT_EQ(with_result.patches.size(), with_result.patch_point_indices.size());

  for (std::size_t i = 0; i < without_result.patches.size(); ++i) {
    EXPECT_EQ(without_result.patches[i].point_count, with_result.patches[i].point_count);
  }
}
