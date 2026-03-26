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

    // pose graph optimization parameter
    int num_adjacent_pose_cnstraints_;
    bool use_save_map_in_loop_ {true};
    double adjacent_edge_info_weight_ {1000.0};
    double loop_edge_info_weight_ {100.0};
    double loop_edge_robust_kernel_delta_ {1.0};

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
    SubmapBEVDescriptor::Database bev_descriptor_db_;
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
