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

#ifndef GRAPH_BASED_SLAM__GRAPH_BASED_SLAM_COMPONENT_H_
#define GRAPH_BASED_SLAM__GRAPH_BASED_SLAM_COMPONENT_H_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define GS_GBS_EXPORT __attribute__ ((dllexport))
    #define GS_GBS_IMPORT __attribute__ ((dllimport))
  #else
    #define GS_GBS_EXPORT __declspec(dllexport)
    #define GS_GBS_IMPORT __declspec(dllimport)
  #endif
  #ifdef GS_GBS_BUILDING_DLL
    #define GS_GBS_PUBLIC GS_GBS_EXPORT
  #else
    #define GS_GBS_PUBLIC GS_GBS_IMPORT
  #endif
  #define GS_GBS_PUBLIC_TYPE GS_GBS_PUBLIC
  #define GS_GBS_LOCAL
#else
  #define GS_GBS_EXPORT __attribute__ ((visibility("default")))
  #define GS_GBS_IMPORT
  #if __GNUC__ >= 4
    #define GS_GBS_PUBLIC __attribute__ ((visibility("default")))
    #define GS_GBS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define GS_GBS_PUBLIC
    #define GS_GBS_LOCAL
  #endif
  #define GS_GBS_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

#include <pcl/point_types.h>  // NOLINT(build/include_order)
#include <pcl/io/pcd_io.h>  // NOLINT(build/include_order)
#include <pcl/registration/gicp.h>  // NOLINT(build/include_order)
#include <pcl/registration/ndt.h>  // NOLINT(build/include_order)
#include <pcl_conversions/pcl_conversions.h>  // NOLINT(build/include_order)
#include <pclomp/gicp_omp.h>  // NOLINT(build/include_order)
#include <pclomp/ndt_omp.h>  // NOLINT(build/include_order)
#include <pclomp/voxel_grid_covariance_omp.h>  // NOLINT(build/include_order)
#include <tf2_ros/buffer.h>  // NOLINT(build/include_order)
#include <tf2_ros/transform_broadcaster.h>  // NOLINT(build/include_order)
#include <tf2_ros/transform_listener.h>  // NOLINT(build/include_order)

#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <lidarslam_msgs/msg/map_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pclomp/gicp_omp_impl.hpp>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/edge_se3_pointxyz.h"
#include "g2o/types/slam3d/parameter_se3_offset.h"
#include "g2o/types/slam3d/se3quat.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "graph_based_slam/gnss_weighting.hpp"
#include "graph_based_slam/scan_context.hpp"
#include "graph_based_slam/solid_descriptor.hpp"
#include "graph_based_slam/submap_bev_descriptor.hpp"
#include "graph_based_slam/three_d_bbs_loop_verifier.hpp"
#include "graph_based_slam/triangle_descriptor_database.hpp"

namespace graphslam
{
  class GraphBasedSlamComponent: public rclcpp::Node  // NOLINT(runtime/indentation_namespace)
  {
public:
    GS_GBS_PUBLIC
    explicit GraphBasedSlamComponent(const rclcpp::NodeOptions & options);

private:
    std::mutex mtx_;

    rclcpp::Clock clock_;
    tf2_ros::Buffer tfbuffer_;
    tf2_ros::TransformListener listener_;
    tf2_ros::TransformBroadcaster broadcaster_;

    boost::shared_ptr < pcl::Registration < pcl::PointXYZI, pcl::PointXYZI >> registration_;
    pcl::VoxelGrid < pcl::PointXYZI > voxelgrid_;

    lidarslam_msgs::msg::MapArray map_array_msg_;
    rclcpp::Subscription < lidarslam_msgs::msg::MapArray > ::SharedPtr map_array_sub_;
    rclcpp::Publisher < lidarslam_msgs::msg::MapArray > ::SharedPtr modified_map_array_pub_;
    rclcpp::Publisher < nav_msgs::msg::Path > ::SharedPtr modified_path_pub_;
    rclcpp::Publisher < sensor_msgs::msg::PointCloud2 > ::SharedPtr modified_map_pub_;
    rclcpp::TimerBase::SharedPtr loop_detect_timer_;
    rclcpp::Service < std_srvs::srv::Empty > ::SharedPtr map_save_srv_;

    struct LoopEdge
    {
      std::pair < int, int > pair_id;
      Eigen::Isometry3d relative_pose;
      double fitness_score {0.0};
    };
    using LoopEdges = std::vector < LoopEdge >;
    using MapSaveRequestHeader = std::shared_ptr < rmw_request_id_t >;
    using MapSaveRequest = std::shared_ptr < std_srvs::srv::Empty::Request >;
    using MapSaveResponse = std::shared_ptr < std_srvs::srv::Empty::Response >;

    void initializePubSub();
    void handleMapSaveRequest(
      const MapSaveRequestHeader request_header,
      const MapSaveRequest request,
      const MapSaveResponse response);
    void searchLoop();
    // Per-query loop search body, factored out of searchLoop() so the scheduler
    // can run it for one (default) or many (deterministic mode) query submaps.
    void searchLoopForLatest(
      const lidarslam_msgs::msg::MapArray & map_array_msg,
      LoopEdges & loop_edges,
      int num_submaps,
      int latest_idx);
    bool snapshotGraphState(
      lidarslam_msgs::msg::MapArray & map_array_msg,
      LoopEdges & loop_edges,
      bool consume_map_update);
    void snapshotLoopEdges(LoopEdges & loop_edges);
    bool upsertLoopEdge(const LoopEdge & loop_edge);
    void doPoseAdjustment(
      lidarslam_msgs::msg::MapArray map_array_msg,
      const LoopEdges & loop_edges,
      bool do_save_map);
    void publishMapAndPose();

    // loop search parameter
    int loop_detection_period_;
    double threshold_loop_closure_score_;
    double scan_context_loop_closure_score_threshold_ {-1.0};
    double distance_loop_closure_;
    double range_of_searching_loop_closure_;
    int search_submap_num_;
    int max_loop_candidate_count_ {3};
    int loop_edge_dedup_index_window_ {8};
    double loop_max_translation_delta_ {15.0};
    double loop_max_rotation_delta_deg_ {45.0};
    // Per-source overrides for descriptor-based candidates (TRIANGLE,
    // SCAN_CONTEXT, BEV, SOLID). When positive, replace the generic caps
    // above for those sources only — DISTANCE keeps the strict default.
    // -1.0 = disabled / fall back to the generic cap.
    double loop_max_translation_delta_descriptor_ {-1.0};
    double loop_max_rotation_delta_deg_descriptor_ {-1.0};
    // Deterministic loop scheduling (opt-in, v0.4 D1). When false (default),
    // searchLoop queries only the single latest submap per timer tick — the
    // historical wall-clock-driven behaviour, whose (query, db) pair set depends
    // on timer batching rather than the map. When true, searchLoop catches up
    // over every submap index not yet used as a query, so the set of loop-search
    // queries is a pure function of the map regardless of tick timing.
    bool deterministic_loop_scheduling_ {false};
    int last_searched_submap_idx_ {-1};

    // pose graph optimization parameter
    int num_adjacent_pose_cnstraints_;
    bool use_save_map_in_loop_ {true};
    double adjacent_edge_info_weight_ {1000.0};
    double loop_edge_info_weight_ {100.0};
    double loop_edge_robust_kernel_delta_ {1.0};
    std::string loop_edge_robust_kernel_type_ {"huber"};

    // Auto-scaling for adjacent_edge_info_weight (Level 1: NIS median tracking).
    // When enabled, the post-optimisation chi-squared of adjacent edges is
    // monitored and adjacent_edge_info_weight_ is mixed toward
    // current * target_nis / median_chi2 via EMA, clamped to [min, max].
    bool adjacent_edge_info_auto_scale_ {false};
    double adjacent_edge_info_auto_scale_target_nis_ {6.0};
    double adjacent_edge_info_auto_scale_ema_alpha_ {0.3};
    double adjacent_edge_info_auto_scale_min_ {1.0};
    double adjacent_edge_info_auto_scale_max_ {1.0e6};
    // Level 2: split the adjacent edge Information matrix into translation /
    // rotation blocks (block-diag with weights w_trans, w_rot on I_3 each) so
    // the auto-scaler can balance translation residuals and rotation residuals
    // independently. When split mode is off the legacy single-scalar shape is
    // used. Targets default to 3.0 (3 DoF per block, vs 6 for the unified
    // mode); the EMA / min / max defaults are shared with Level 1.
    bool adjacent_edge_info_auto_scale_split_trans_rot_ {false};
    double adjacent_edge_info_weight_trans_ {-1.0};
    double adjacent_edge_info_weight_rot_ {-1.0};
    double adjacent_edge_info_auto_scale_target_nis_trans_ {3.0};
    double adjacent_edge_info_auto_scale_target_nis_rot_ {3.0};

    bool initial_map_array_received_ {false};
    bool is_map_array_updated_ {false};
    int previous_submaps_num_ {0};

    LoopEdges loop_edges_;

    bool debug_flag_ {false};

    // Scan Context loop detection
    bool use_scan_context_ {false};
    double scan_context_threshold_ {0.3};
    bool prefer_scan_context_candidates_ {false};
    ScanContext::Database scan_context_db_;
    bool use_bev_descriptor_ {false};
    double bev_descriptor_threshold_ {0.20};
    double bev_descriptor_grid_size_m_ {80.0};
    int bev_descriptor_grid_cells_ {40};
    int bev_descriptor_yaw_bins_ {24};
    int bev_descriptor_sequence_window_ {0};
    double bev_descriptor_sequence_threshold_ {-1.0};
    double bev_descriptor_pose_consistency_threshold_m_ {-1.0};
    double bev_descriptor_max_euclidean_distance_m_ {-1.0};
    double bev_descriptor_rerank_weight_m_ {100.0};
    // FOV-aware (mutual-visibility) distance for the BEV descriptor. Default
    // off so the cosine-distance baseline stays unchanged on 360° LiDAR.
    bool bev_use_mutual_visibility_ {false};
    double bev_mutual_visibility_min_overlap_ratio_ {0.05};
    double bev_mutual_visibility_occupancy_eps_ {0.5};
    SubmapBEVDescriptor::Database bev_descriptor_db_;
    // Triangle (STD/BTC-style) descriptor place-recognition path. Built on the
    // BSD-2 primitives in graph_based_slam/triangle_descriptor*. Default off
    // so the existing default workflow stays unchanged.
    bool use_triangle_descriptor_ {false};
    // Tuned 2026-05-18 on NTU VIRAL tnp_01 ablation v4. The earlier loose
    // defaults (60 cells, 0.3 m salience, 80 keypoints, 1.0 m edge bin)
    // produced false-positive vote buckets where one stale submap collected
    // every triangle match; this triggered randomly-rotating SE(3) outputs
    // that NDT could not refine. The tighter values below caused triangle
    // to vote across distinct submap ids (32 / 40 / 17 / 9) and produced
    // the first triangle-sourced accepted loop closure (32 <-> 95, 0.49 m
    // / 1.06 deg correction).
    double triangle_descriptor_grid_size_m_ {60.0};
    int triangle_descriptor_grid_cells_ {100};
    int triangle_descriptor_max_keypoints_ {40};
    double triangle_descriptor_min_salience_m_ {0.8};
    double triangle_descriptor_min_edge_m_ {2.0};
    double triangle_descriptor_max_edge_m_ {50.0};
    int triangle_descriptor_max_triangles_ {3000};
    double triangle_descriptor_edge_bin_m_ {0.5};
    // Quad-hash 4th-point feature bin (m). 0 = disabled (legacy 3-edge hash).
    // When > 0, the bucket key also includes the quantized distance from the
    // triangle centroid to the nearest non-vertex keypoint, which makes the
    // hash 4-dim and rejects wrong-but-agreeing triangle pairs in repeated
    // geometry (corridor / parking-row / parallel column rows).
    double triangle_descriptor_quad_feature_bin_m_ {0.0};
    // Keypoint extractor mode. "bev_max_height" is the original outdoor-only
    // extractor; "edge_3d" enables PCA-edgeness keypoints that survive in
    // narrow-FOV / indoor scenes (MID-360, Newer College math_hard) where
    // BEV max-height keypoint repeatability collapses.
    std::string triangle_descriptor_keypoint_mode_ {"bev_max_height"};
    double triangle_descriptor_edge_voxel_size_m_ {0.4};
    double triangle_descriptor_edge_neighbor_radius_m_ {1.0};
    int triangle_descriptor_edge_min_neighbors_ {6};
    double triangle_descriptor_edge_min_edgeness_ {0.5};
    double triangle_descriptor_edge_nms_radius_m_ {2.0};
    // 5-inlier floor would have killed the only accepted loop in v4 (id=32
    // emitted with 4 inliers), so settle on 4 as the compromise between
    // recall and noise. Votes can stay loose because the tighter keypoint
    // / hash params suppress most false buckets on their own.
    int triangle_descriptor_min_votes_ {6};
    int triangle_descriptor_min_inliers_ {4};
    // Companion to min_inliers expressed as inliers / eval_n. Zero disables.
    // Lets the operator combine a low absolute count with a meaningful
    // relative-density floor (e.g. 4 inliers / max_pairs 64 = 6% vs the
    // same 4 inliers / max_pairs 20 = 20%).
    double triangle_descriptor_min_inlier_ratio_ {0.0};
    // Cap on triangle pairs evaluated inside the RANSAC consensus check.
    // Lower numbers make min_inlier_ratio more informative; default 64 keeps
    // the previous behaviour.
    int triangle_descriptor_max_pairs_ {64};
    // 4-point consensus: after the 3-point RANSAC picks a winning SE(3),
    // optionally project every query keypoint by that transform and require
    // this many to fall within `fourth_point_max_distance_m` of some
    // database keypoint in the chosen submap. Three points uniquely
    // determine SE(3), so even a strong 3-point consensus can be fooled by
    // repeated structure; the 4-point gate adds an independent constraint.
    // Default 0 disables the gate.
    int triangle_descriptor_min_4th_point_agreements_ {0};
    double triangle_descriptor_fourth_point_max_distance_m_ {2.0};
    // After the 3-point RANSAC picks the winning SE(3), re-estimate it by
    // pooling the 3 * N_inliers point correspondences and running a single
    // N-point Umeyama least-squares. Reduces translation noise by √N versus
    // keeping the single 3-point hypothesis.
    bool triangle_descriptor_refine_se3_with_all_inliers_ {false};
    // Diagnostic-only: when true, run accumulateVotes (and submap_id selection)
    // but skip the RANSAC findLoopCandidate inner loop. Used to isolate
    // "executor scheduling cost of enabling triangle pipeline" from
    // "RANSAC compute cost" when investigating APE drift on tuned configs.
    // Default false (production).
    bool triangle_descriptor_skip_ransac_ {false};
    double triangle_descriptor_inlier_translation_m_ {2.0};
    double triangle_descriptor_inlier_rotation_deg_ {5.0};
    int triangle_descriptor_exclude_recent_ {4};
    // Cross-verification: when both use_triangle_descriptor and
    // use_bev_descriptor are true, gate the triangle candidate by also
    // requiring the BEV mutual-visibility distance to clear an upper bound.
    // Helps filter false positives caused by repeated geometry (corridors,
    // facades) at the cost of triangle-only recall.
    bool triangle_verify_with_bev_ {false};
    double triangle_verify_bev_max_distance_ {0.30};
    graphslam::triangle::TriangleDatabase triangle_descriptor_db_;
    struct TrianglePerSubmap
    {
      std::vector < graphslam::triangle::Keypoint > keypoints;
      std::vector < graphslam::triangle::TriangleDescriptor > triangles;
    };
    std::vector < TrianglePerSubmap > triangle_descriptor_per_submap_;
    int triangle_descriptor_next_submap_idx_ {0};
    bool use_solid_descriptor_ {false};
    double solid_descriptor_min_similarity_ {0.70};
    int solid_descriptor_sequence_window_ {0};
    double solid_descriptor_sequence_min_similarity_ {-1.0};
    double solid_descriptor_pose_consistency_threshold_m_ {-1.0};
    double solid_descriptor_max_euclidean_distance_m_ {-1.0};
    SolidDescriptor::Database solid_descriptor_db_;
    bool use_3d_bbs_for_scan_context_ {false};
    double three_d_bbs_min_level_res_ {1.0};
    int three_d_bbs_max_level_ {3};
    double three_d_bbs_score_threshold_percentage_ {0.25};
    int three_d_bbs_timeout_msec_ {50};
    int three_d_bbs_num_threads_ {0};
    double three_d_bbs_voxel_leaf_size_ {1.0};
    int three_d_bbs_source_submap_num_ {2};
    int three_d_bbs_target_submap_radius_ {1};
    double three_d_bbs_translation_search_margin_m_ {15.0};
    double three_d_bbs_roll_pitch_search_deg_ {10.0};
    double three_d_bbs_yaw_search_deg_ {180.0};
    ThreeDBBSLoopVerifier three_d_bbs_loop_verifier_;

    bool use_dynamic_object_filter_ {false};
    double dynamic_object_filter_voxel_size_ {0.3};
    int dynamic_object_filter_min_observations_ {2};
    int dynamic_object_filter_temporal_window_ {5};
    double dynamic_object_filter_max_range_from_sensor_m_ {30.0};

    // PCD disk cache for memory-efficient submap storage
    std::string pcd_cache_dir_;
    bool use_pcd_cache_ {false};
    void saveSubmapToPCD(
      int idx,
      const pcl::PointCloud < pcl::PointXYZI > ::Ptr & cloud);
    pcl::PointCloud < pcl::PointXYZI > ::Ptr loadSubmapFromPCD(int idx);

    // Autoware-compatible grid-divided PCD map output
    std::string map_save_dir_ {"."};
    double map_grid_size_x_ {20.0};
    double map_grid_size_y_ {20.0};
    double map_leaf_size_ {0.2};
    void saveGridDividedMap(
      const pcl::PointCloud < pcl::PointXYZI > ::Ptr & map);

    // Direct odometry + cloud input mode (for LIO frontends)
    bool use_odom_input_ {false};
    double submap_distance_threshold_ {1.5};
    rclcpp::Subscription < nav_msgs::msg::Odometry > ::SharedPtr odom_sub_;
    rclcpp::Subscription < sensor_msgs::msg::PointCloud2 > ::SharedPtr cloud_sub_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
    Eigen::Vector3d last_submap_position_ {0, 0, 0};
    bool last_submap_position_valid_ {false};
    double accumulated_distance_ {0.0};
    void receiveOdometry(const nav_msgs::msg::Odometry & msg);
    void receiveCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void tryCreateSubmap();
    nav_msgs::msg::Odometry latest_odom_;
    bool latest_odom_valid_ {false};
    rclcpp::Time latest_cloud_stamp_ {0, 0, RCL_ROS_TIME};

    // GNSS constraints for georeferenced mapping
    bool use_gnss_ {false};
    std::string gnss_topic_ {"/gnss/fix"};
    double gnss_info_weight_ {1.0};
    bool gnss_use_covariance_weighting_ {true};
    double gnss_covariance_min_variance_m2_ {0.01};
    double gnss_covariance_max_variance_m2_ {25.0};
    double gnss_rtk_fix_max_horizontal_stddev_m_ {0.3};
    double gnss_rtk_fix_weight_scale_ {3.0};
    double gnss_non_rtk_weight_scale_ {1.0};
    double gnss_header_stamp_max_skew_sec_ {30.0};
    int gnss_origin_min_samples_ {3};
    double gnss_origin_consistency_threshold_m_ {20.0};
    rclcpp::Subscription < sensor_msgs::msg::NavSatFix > ::SharedPtr gnss_sub_;
    struct GnssEnu
    {
      double stamp;
      double x;
      double y;
      double z;  // ENU coordinates relative to origin
      double info_x;
      double info_y;
      double info_z;
      bool covariance_valid;
      bool rtk_like;
      double horizontal_stddev_m;
    };
    struct GnssOriginSample
    {
      double lat;
      double lon;
      double alt;
    };
    std::vector < GnssEnu > gnss_buffer_;
    std::vector < GnssOriginSample > gnss_origin_candidates_;
    std::mutex gnss_mtx_;
    bool gnss_origin_set_ {false};
    double gnss_origin_lat_ {0.0};
    double gnss_origin_lon_ {0.0};
    double gnss_origin_alt_ {0.0};
    void receiveNavSatFix(const sensor_msgs::msg::NavSatFix & msg);
    bool isUsableGnssFix(const sensor_msgs::msg::NavSatFix & msg) const;
    void tryInitializeGnssOrigin(double lat, double lon, double alt);
    double approximateGeodeticDistanceMeters(
      double lat0,
      double lon0,
      double lat1,
      double lon1) const;
    Eigen::Vector3d geodeticToEnu(double lat, double lon, double alt) const;

    // IMU preintegration
    bool use_imu_preintegration_ {false};
    double imu_rotation_info_roll_pitch_ {100.0};
    double imu_rotation_info_yaw_ {10.0};
    rclcpp::Subscription < sensor_msgs::msg::Imu > ::SharedPtr imu_sub_;
    struct StampedImu
    {
      double stamp;
      double ax;
      double ay;
      double az;
      double gx;
      double gy;
      double gz;
      double qx;
      double qy;
      double qz;
      double qw;
    };
    std::vector < StampedImu > imu_buffer_;
    std::mutex imu_mtx_;
    static constexpr size_t kMaxImuBufferSize = 50000;
    void receiveImu(const sensor_msgs::msg::Imu & msg);
    Eigen::Quaterniond integrateImuRotation(double t0, double t1) const;
  };
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__GRAPH_BASED_SLAM_COMPONENT_H_
