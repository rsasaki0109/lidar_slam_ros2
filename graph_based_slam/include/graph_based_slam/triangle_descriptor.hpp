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

// STD/BTC-style triangle descriptor primitives, implemented from scratch
// under BSD-2 so the default workflow can include them.
//
// The pipeline (in this header):
//   1. extractKeypoints: dispatcher that picks one of the modes below.
//      - extractKeypointsBEV: take a submap point cloud, project to BEV, pick
//        the local-maximum cells in max_height as stable keypoints. Works on
//        spinning 360° LiDAR with wide vertical FOV (e.g. OS1 outdoors).
//      - extractKeypointsEdge3D: PCA eigenvalue ratio on radius-r neighborhoods
//        picks edge-like supports (column edges, wall corners). Survives in
//        narrow-FOV (MID-360) and indoor scenes where BEV max-height collapses.
//   2. buildTriangles: enumerate all 3-tuples of keypoints, drop those whose
//      edge lengths fall outside [min_edge_m, max_edge_m] or that are
//      near-collinear, and store the edges sorted ascending so the triangle
//      is a yaw / rotation invariant descriptor.
//   3. estimateRigidFromTriangle: SVD / Umeyama on the 3-point
//      correspondence to recover the SE(3) bringing one triangle onto the
//      other; this is the "geometric verification" step used downstream.
//
// Matching, hashing, and the actual TriangleDatabase live in follow-up
// patches; this header keeps the testable primitives self-contained.

#ifndef GRAPH_BASED_SLAM__TRIANGLE_DESCRIPTOR_HPP_
#define GRAPH_BASED_SLAM__TRIANGLE_DESCRIPTOR_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace graphslam
{
namespace triangle
{

struct Keypoint
{
  Eigen::Vector3f position {Eigen::Vector3f::Zero()};
  // BEV max-height minus the neighborhood floor (BEV mode),
  // or PCA edgeness (λ2 - λ1) / λ2 (EDGE_3D mode); higher = more salient.
  float salience {0.0f};
};

// Keypoint extraction strategy. BEV_MAX_HEIGHT was the original outdoor-only
// extractor; EDGE_3D adds PCA-based edge keypoints that survive in narrow-FOV
// LiDARs (MID-360) and indoor scenes where BEV max-height collapses.
enum class KeypointMode
{
  BEV_MAX_HEIGHT,
  EDGE_3D,
};

struct KeypointExtractionConfig
{
  KeypointMode mode {KeypointMode::BEV_MAX_HEIGHT};
  // ----- BEV_MAX_HEIGHT params -----
  // Side length of the BEV window centred on the submap origin.
  double grid_size_m {60.0};
  // Cells per side; cell_size = grid_size_m / grid_cells.
  int grid_cells {60};
  // Radius (in cells) used for local-max neighborhood comparison.
  int neighborhood_radius_cells {2};
  // Minimum salience (m) to keep a keypoint.
  float min_salience_m {0.3f};
  // ----- EDGE_3D params -----
  // Voxel downsample size before PCA; trades repeatability for cost.
  float edge_voxel_size_m {0.4f};
  // Radius (m) for the PCA neighborhood used to compute eigenvalues.
  float edge_neighbor_radius_m {1.0f};
  // Minimum neighbor count; points with sparser support are skipped.
  int edge_min_neighbors {6};
  // Minimum PCA edgeness (λ2 - λ1) / λ2 to accept a candidate.
  float edge_min_edgeness {0.5f};
  // Suppression radius (m) for non-maximum suppression of edgeness.
  float edge_nms_radius_m {2.0f};
  // ----- common -----
  // Cap on returned keypoints; we keep the highest-salience ones.
  int max_keypoints {80};
};

struct TriangleDescriptor
{
  // Edge lengths sorted ascending. edges[0] <= edges[1] <= edges[2].
  std::array<float, 3> edges {{0.0f, 0.0f, 0.0f}};
  // Indices into the keypoint list this triangle was built from. The order
  // matches the edges array: the vertex opposite ``edges[k]`` is
  // ``keypoint_ids[k]``; vertices a, b, c are ids [0], [1], [2] but the
  // physical edges connecting them are sorted before they are written here.
  std::array<int, 3> keypoint_ids {{-1, -1, -1}};
};

struct TriangleBuildConfig
{
  // Lower edge bound (m); shorter triangles are dropped because tiny
  // baselines are unstable under noise.
  float min_edge_m {2.0f};
  // Upper edge bound (m); longer triangles tend to bridge dynamic objects.
  float max_edge_m {50.0f};
  // Reject triangles whose smallest angle is below this threshold (deg);
  // near-collinear configurations carry almost no orientation information.
  float min_angle_deg {5.0f};
  // Cap on number of triangles returned, sorted by largest edge first to
  // bias toward globally informative baselines.
  int max_triangles {5000};
};

// --------------------------- helpers ---------------------------

namespace detail
{

inline float edgeLength(const Eigen::Vector3f & a, const Eigen::Vector3f & b)
{
  return (a - b).norm();
}

// Triangle smallest angle from edge lengths via the law of cosines.
inline float smallestAngleDeg(float l1, float l2, float l3)
{
  // l3 is the longest edge; opposite angle is the largest. The smallest
  // angle is opposite the shortest edge (l1).
  const float num = l2 * l2 + l3 * l3 - l1 * l1;
  const float den = 2.0f * l2 * l3;
  if (den <= 1e-9f) {return 0.0f;}
  const float cos_alpha = std::max(-1.0f, std::min(1.0f, num / den));
  return std::acos(cos_alpha) * 180.0f / static_cast<float>(M_PI);
}

}  // namespace detail

// --------------------------- keypoint extraction ---------------------------

inline std::vector<Keypoint> extractKeypointsBEV(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  const KeypointExtractionConfig & cfg)
{
  std::vector<Keypoint> result;
  if (cloud.empty() || cfg.grid_cells <= 0 || cfg.grid_size_m <= 0.0) {
    return result;
  }
  const int gc = cfg.grid_cells;
  const float half = static_cast<float>(cfg.grid_size_m * 0.5);
  const float cell = static_cast<float>(cfg.grid_size_m / static_cast<double>(gc));

  Eigen::MatrixXf max_height = Eigen::MatrixXf::Constant(
    gc, gc, -std::numeric_limits<float>::infinity());
  // Remember the (x, y) of the highest contributing point per cell so the
  // emitted keypoint sits at the actual support and not at the cell centre.
  Eigen::MatrixXf best_x = Eigen::MatrixXf::Zero(gc, gc);
  Eigen::MatrixXf best_y = Eigen::MatrixXf::Zero(gc, gc);

  for (const auto & p : cloud.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {continue;}
    if (std::abs(p.x) > half || std::abs(p.y) > half) {continue;}
    const int ix = static_cast<int>(std::floor((p.x + half) / cell));
    const int iy = static_cast<int>(std::floor((p.y + half) / cell));
    if (ix < 0 || ix >= gc || iy < 0 || iy >= gc) {continue;}
    if (p.z > max_height(iy, ix)) {
      max_height(iy, ix) = p.z;
      best_x(iy, ix) = p.x;
      best_y(iy, ix) = p.y;
    }
  }

  const int r = std::max(1, cfg.neighborhood_radius_cells);
  std::vector<Keypoint> candidates;
  candidates.reserve(static_cast<std::size_t>(gc) * static_cast<std::size_t>(gc));

  for (int iy = 0; iy < gc; ++iy) {
    for (int ix = 0; ix < gc; ++ix) {
      const float h = max_height(iy, ix);
      if (!std::isfinite(h)) {continue;}
      // Local-max test on the (2r+1)x(2r+1) window.
      float floor_h = std::numeric_limits<float>::infinity();
      bool is_max = true;
      int neighbor_count = 0;
      for (int dy = -r; dy <= r && is_max; ++dy) {
        const int ny = iy + dy;
        if (ny < 0 || ny >= gc) {continue;}
        for (int dx = -r; dx <= r; ++dx) {
          const int nx = ix + dx;
          if (nx < 0 || nx >= gc) {continue;}
          if (nx == ix && ny == iy) {continue;}
          const float hn = max_height(ny, nx);
          if (!std::isfinite(hn)) {continue;}
          ++neighbor_count;
          if (hn > h) {is_max = false; break;}
          if (hn < floor_h) {floor_h = hn;}
        }
      }
      if (!is_max || neighbor_count == 0) {continue;}
      const float salience = std::isfinite(floor_h) ? (h - floor_h) : 0.0f;
      if (salience < cfg.min_salience_m) {continue;}
      Keypoint kp;
      kp.position = Eigen::Vector3f(best_x(iy, ix), best_y(iy, ix), h);
      kp.salience = salience;
      candidates.push_back(kp);
    }
  }

  std::sort(
    candidates.begin(), candidates.end(),
    [](const Keypoint & a, const Keypoint & b) {return a.salience > b.salience;});

  const int keep = std::min<int>(cfg.max_keypoints, static_cast<int>(candidates.size()));
  result.assign(candidates.begin(), candidates.begin() + keep);
  return result;
}

// --------------------------- edge-3D keypoint extraction ---------------------------

// PCA-based edge keypoint extractor. For each voxel-downsampled point, compute
// the covariance of its radius-r neighbors and pick points where the largest
// eigenvalue dominates the middle one — geometrically these are linear /
// edge-like supports (column edges, wall corners, door frames) that survive in
// narrow-FOV LiDAR and indoor scenes where BEV max-height collapses.
inline std::vector<Keypoint> extractKeypointsEdge3D(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  const KeypointExtractionConfig & cfg)
{
  std::vector<Keypoint> result;
  if (cloud.empty()) {return result;}

  // Voxel downsample so the PCA cost is bounded regardless of submap density.
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr src(new pcl::PointCloud<pcl::PointXYZI>(cloud));
  if (cfg.edge_voxel_size_m > 0.0f) {
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(src);
    vg.setLeafSize(cfg.edge_voxel_size_m, cfg.edge_voxel_size_m, cfg.edge_voxel_size_m);
    vg.filter(*downsampled);
  } else {
    downsampled = src;
  }
  if (downsampled->size() < static_cast<std::size_t>(std::max(3, cfg.edge_min_neighbors))) {
    return result;
  }

  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud(downsampled);

  struct Candidate
  {
    int idx;
    float edgeness;
    Eigen::Vector3f position;
  };
  std::vector<Candidate> candidates;
  candidates.reserve(downsampled->size());

  const float radius = std::max(0.05f, cfg.edge_neighbor_radius_m);
  std::vector<int> nn_idx;
  std::vector<float> nn_sq;

  for (std::size_t i = 0; i < downsampled->size(); ++i) {
    const auto & p = downsampled->points[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {continue;}
    nn_idx.clear();
    nn_sq.clear();
    const int found = kdtree.radiusSearch(static_cast<int>(i), radius, nn_idx, nn_sq);
    if (found < cfg.edge_min_neighbors) {continue;}

    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (int idx : nn_idx) {
      const auto & q = downsampled->points[idx];
      centroid += Eigen::Vector3f(q.x, q.y, q.z);
    }
    centroid /= static_cast<float>(nn_idx.size());

    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (int idx : nn_idx) {
      const auto & q = downsampled->points[idx];
      Eigen::Vector3f d(q.x - centroid.x(), q.y - centroid.y(), q.z - centroid.z());
      cov += d * d.transpose();
    }
    cov /= static_cast<float>(nn_idx.size());

    // SelfAdjointEigenSolver returns eigenvalues in ascending order.
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov, Eigen::EigenvaluesOnly);
    if (es.info() != Eigen::Success) {continue;}
    const Eigen::Vector3f ev = es.eigenvalues();
    const float lam0 = std::max(0.0f, ev(0));
    const float lam1 = std::max(0.0f, ev(1));
    const float lam2 = std::max(0.0f, ev(2));
    if (lam2 <= 1e-9f) {continue;}
    const float edgeness = (lam2 - lam1) / lam2;
    if (edgeness < cfg.edge_min_edgeness) {continue;}

    Candidate c;
    c.idx = static_cast<int>(i);
    c.edgeness = edgeness;
    c.position = Eigen::Vector3f(p.x, p.y, p.z);
    candidates.push_back(c);
  }

  std::sort(
    candidates.begin(), candidates.end(),
    [](const Candidate & a, const Candidate & b) {return a.edgeness > b.edgeness;});

  // Non-maximum suppression in 3D: drop candidates that fall within
  // edge_nms_radius_m of an already-accepted higher-edgeness keypoint.
  const float nms_r = std::max(0.0f, cfg.edge_nms_radius_m);
  const float nms_r2 = nms_r * nms_r;
  std::vector<Keypoint> kept;
  kept.reserve(candidates.size());
  for (const auto & c : candidates) {
    bool suppressed = false;
    if (nms_r > 0.0f) {
      for (const auto & k : kept) {
        if ((k.position - c.position).squaredNorm() < nms_r2) {
          suppressed = true;
          break;
        }
      }
    }
    if (suppressed) {continue;}
    Keypoint kp;
    kp.position = c.position;
    kp.salience = c.edgeness;
    kept.push_back(kp);
    if (cfg.max_keypoints > 0 && static_cast<int>(kept.size()) >= cfg.max_keypoints) {break;}
  }

  result = std::move(kept);
  return result;
}

// Dispatcher: pick the keypoint extractor based on ``cfg.mode``. New code
// should call this rather than the mode-specific entry points; the BEV and
// EDGE_3D functions stay public for tests and ablation tooling.
inline std::vector<Keypoint> extractKeypoints(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  const KeypointExtractionConfig & cfg)
{
  switch (cfg.mode) {
    case KeypointMode::EDGE_3D:
      return extractKeypointsEdge3D(cloud, cfg);
    case KeypointMode::BEV_MAX_HEIGHT:
    default:
      return extractKeypointsBEV(cloud, cfg);
  }
}

// --------------------------- triangle enumeration ---------------------------

inline std::vector<TriangleDescriptor> buildTriangles(
  const std::vector<Keypoint> & keypoints,
  const TriangleBuildConfig & cfg)
{
  std::vector<TriangleDescriptor> all;
  const int n = static_cast<int>(keypoints.size());
  if (n < 3) {return all;}
  all.reserve(static_cast<std::size_t>(n) * static_cast<std::size_t>(n));

  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      const float l_ij = detail::edgeLength(keypoints[i].position, keypoints[j].position);
      if (l_ij < cfg.min_edge_m || l_ij > cfg.max_edge_m) {continue;}
      for (int k = j + 1; k < n; ++k) {
        const float l_jk = detail::edgeLength(keypoints[j].position, keypoints[k].position);
        if (l_jk < cfg.min_edge_m || l_jk > cfg.max_edge_m) {continue;}
        const float l_ik = detail::edgeLength(keypoints[i].position, keypoints[k].position);
        if (l_ik < cfg.min_edge_m || l_ik > cfg.max_edge_m) {continue;}

        // Sort edges ascending. Track which keypoint is opposite each edge so
        // downstream code can recover the original physical correspondence.
        // Edge ij is opposite vertex k; edge jk opposite i; edge ik opposite j.
        struct EdgeRef
        {
          float length;
          int opposite_kp;
        };
        std::array<EdgeRef, 3> e = {{
          {l_jk, i}, {l_ik, j}, {l_ij, k},
        }};
        std::sort(
          e.begin(), e.end(),
          [](const EdgeRef & a, const EdgeRef & b) {return a.length < b.length;});

        const float ang = detail::smallestAngleDeg(e[0].length, e[1].length, e[2].length);
        if (ang < cfg.min_angle_deg) {continue;}

        TriangleDescriptor t;
        t.edges = {{e[0].length, e[1].length, e[2].length}};
        t.keypoint_ids = {{e[0].opposite_kp, e[1].opposite_kp, e[2].opposite_kp}};
        all.push_back(t);
      }
    }
  }

  // Keep the most informative triangles (largest baseline first) and cap.
  std::sort(
    all.begin(), all.end(),
    [](const TriangleDescriptor & a, const TriangleDescriptor & b) {
      return a.edges[2] > b.edges[2];
    });
  if (cfg.max_triangles > 0 && static_cast<int>(all.size()) > cfg.max_triangles) {
    all.resize(static_cast<std::size_t>(cfg.max_triangles));
  }
  return all;
}

// --------------------------- 3-point rigid SE(3) ---------------------------

// Estimate the SE(3) transform T such that T * src[i] ≈ dst[i] for i in 0..2,
// using closed-form Umeyama (3D analogue of Kabsch with translation centring).
// Returns identity when the source points are degenerate (collinear / nearly
// coincident).
inline Eigen::Matrix4f estimateRigidFromTriangle(
  const std::array<Eigen::Vector3f, 3> & src,
  const std::array<Eigen::Vector3f, 3> & dst)
{
  const Eigen::Vector3f src_centroid = (src[0] + src[1] + src[2]) / 3.0f;
  const Eigen::Vector3f dst_centroid = (dst[0] + dst[1] + dst[2]) / 3.0f;
  Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
  for (int i = 0; i < 3; ++i) {
    H += (src[i] - src_centroid) * (dst[i] - dst_centroid).transpose();
  }
  Eigen::JacobiSVD<Eigen::Matrix3f> svd(
    H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3f R = svd.matrixV() * svd.matrixU().transpose();
  if (R.determinant() < 0.0f) {
    Eigen::Matrix3f V = svd.matrixV();
    V.col(2) *= -1.0f;
    R = V * svd.matrixU().transpose();
  }
  const Eigen::Vector3f t = dst_centroid - R * src_centroid;
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = t;
  return T;
}

// N-point variant of estimateRigidFromTriangle. Used to refine the winning
// 3-point RANSAC hypothesis after we know which triangle pairs agree on the
// SE(3): pooling the 3*N inlier point correspondences and running one final
// Umeyama least-squares reduces translation noise by √N relative to the
// single-triangle estimate. Returns identity if N < 3 or sizes mismatch.
inline Eigen::Matrix4f estimateRigidFromCorrespondences(
  const std::vector<Eigen::Vector3f> & src,
  const std::vector<Eigen::Vector3f> & dst)
{
  if (src.size() != dst.size() || src.size() < 3) {
    return Eigen::Matrix4f::Identity();
  }
  const std::size_t n = src.size();
  Eigen::Vector3f src_centroid = Eigen::Vector3f::Zero();
  Eigen::Vector3f dst_centroid = Eigen::Vector3f::Zero();
  for (std::size_t i = 0; i < n; ++i) {
    src_centroid += src[i];
    dst_centroid += dst[i];
  }
  src_centroid /= static_cast<float>(n);
  dst_centroid /= static_cast<float>(n);
  Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
  for (std::size_t i = 0; i < n; ++i) {
    H += (src[i] - src_centroid) * (dst[i] - dst_centroid).transpose();
  }
  Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3f R = svd.matrixV() * svd.matrixU().transpose();
  if (R.determinant() < 0.0f) {
    Eigen::Matrix3f V = svd.matrixV();
    V.col(2) *= -1.0f;
    R = V * svd.matrixU().transpose();
  }
  const Eigen::Vector3f t = dst_centroid - R * src_centroid;
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = t;
  return T;
}

}  // namespace triangle
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__TRIANGLE_DESCRIPTOR_HPP_
