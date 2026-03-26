#ifndef GS_SM_COMPONENT_H_INCLUDED
#define GS_SM_COMPONENT_H_INCLUDED

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define GS_SM_EXPORT __attribute__ ((dllexport))
    #define GS_SM_IMPORT __attribute__ ((dllimport))
  #else
    #define GS_SM_EXPORT __declspec(dllexport)
    #define GS_SM_IMPORT __declspec(dllimport)
  #endif
  #ifdef GS_SM_BUILDING_DLL
    #define GS_SM_PUBLIC GS_SM_EXPORT
  #else
    #define GS_SM_PUBLIC GS_SM_IMPORT
  #endif
  #define GS_SM_PUBLIC_TYPE GS_SM_PUBLIC
  #define GS_SM_LOCAL
#else
  #define GS_SM_EXPORT __attribute__ ((visibility("default")))
  #define GS_SM_IMPORT
  #if __GNUC__ >= 4
    #define GS_SM_PUBLIC __attribute__ ((visibility("default")))
    #define GS_SM_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define GS_SM_PUBLIC
    #define GS_SM_LOCAL
  #endif
  #define GS_SM_PUBLIC_TYPE
#endif

#if __cplusplus
} // extern "C"
#endif

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <lidarslam_msgs/msg/map_array.hpp>
#include "scanmatcher/lidar_undistortion.hpp"
#include "scanmatcher/voxel_hash_map.hpp"

#include <pclomp/ndt_omp.h>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp.h>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>
#include <pclomp/gicp_omp.h>
#include <pclomp/gicp_omp_impl.hpp>
#ifdef HAS_FAST_GICP
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>
#endif
#ifdef HAS_SMALL_GICP
#include <small_gicp/pcl/pcl_registration.hpp>
#endif

#include <mutex>
#include <thread>
#include <future>

#include <pcl_conversions/pcl_conversions.h>

namespace graphslam
{
  class ScanMatcherComponent: public rclcpp::Node
  {
public:
    GS_SM_PUBLIC
    explicit ScanMatcherComponent(const rclcpp::NodeOptions & options);

private:
    enum class TrackingState
    {
      Tracking,
      Suspect,
      Recovery
    };

    rclcpp::Clock clock_;
    tf2_ros::Buffer tfbuffer_;
    tf2_ros::TransformListener listener_;
    tf2_ros::TransformBroadcaster broadcaster_;

    std::string global_frame_id_;
    std::string robot_frame_id_;
    std::string odom_frame_id_;

    boost::shared_ptr<pcl::Registration < pcl::PointXYZI, pcl::PointXYZI >> registration_;

    rclcpp::Subscription < geometry_msgs::msg::PoseStamped > ::SharedPtr initial_pose_sub_;
    rclcpp::Subscription < sensor_msgs::msg::Imu > ::SharedPtr imu_sub_;
    rclcpp::Subscription < sensor_msgs::msg::PointCloud2 > ::SharedPtr input_cloud_sub_;

    std::mutex mtx_;
    pcl::PointCloud < pcl::PointXYZI > targeted_cloud_;
    rclcpp::Time last_map_time_;
    bool mapping_flag_ {false};
    bool is_map_updated_ {false};
    std::thread mapping_thread_;
    std::packaged_task < void() > mapping_task_;
    std::future < void > mapping_future_;

    geometry_msgs::msg::PoseStamped current_pose_stamped_;
    lidarslam_msgs::msg::MapArray map_array_msg_;
    nav_msgs::msg::Path path_;
    rclcpp::Publisher < geometry_msgs::msg::PoseStamped > ::SharedPtr pose_pub_;
    rclcpp::Publisher < sensor_msgs::msg::PointCloud2 > ::SharedPtr map_pub_;
    rclcpp::Publisher < lidarslam_msgs::msg::MapArray > ::SharedPtr map_array_pub_;
    rclcpp::Publisher < nav_msgs::msg::Path > ::SharedPtr path_pub_;

    void initializePubSub();
    bool initializeMap(const pcl::PointCloud <pcl::PointXYZI>::Ptr & cloud_ptr, const std_msgs::msg::Header & header);
    void receiveCloud(
      const pcl::PointCloud < pcl::PointXYZI> ::ConstPtr & input_cloud_ptr,
      const rclcpp::Time stamp);
    void receiveImu(const sensor_msgs::msg::Imu imu_msg);
    void publishMapAndPose(
      const pcl::PointCloud < pcl::PointXYZI > ::ConstPtr & cloud_ptr,
      const Eigen::Matrix4f final_transformation,
      const rclcpp::Time stamp
    );
    bool reserveDebugCloudDumpFrame(int * frame_index);
    void dumpDebugCloudStage(
      const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud_ptr,
      const std::vector<float> * point_times,
      const rclcpp::Time stamp,
      int frame_index,
      const std::string & stage);
    Eigen::Matrix4f getTransformation(const geometry_msgs::msg::Pose pose);
    void publishMap(const lidarslam_msgs::msg::MapArray & map_array_msg, const std::string & map_frame_id);
    void updateMap(
      const pcl::PointCloud < pcl::PointXYZI > ::ConstPtr cloud_ptr,
      const Eigen::Matrix4f final_transformation,
      const geometry_msgs::msg::PoseStamped current_pose_stamped
    );
    bool refreshRegistrationTargetFromTargetedCloud();
    geometry_msgs::msg::TransformStamped calculateMaptoOdomTransform(
      const geometry_msgs::msg::TransformStamped &base_to_map_msg,
      const rclcpp::Time stamp
    );
    const char * trackingStateName(TrackingState state) const;

    bool initial_pose_received_ {false};
    bool initial_cloud_received_ {false};

    // setting parameter
    std::string registration_method_;
    double trans_for_mapupdate_;
    double vg_size_for_input_;
    double vg_size_for_map_;
    int min_points_for_scan_ {100};
    bool use_min_max_filter_ {false};
    double scan_min_range_ {0.1};
    double scan_max_range_ {100.0};
    double map_publish_period_;
    int num_targeted_cloud_;
    int num_recovery_targeted_cloud_ {40};
    bool use_spatial_local_map_ {false};
    double spatial_local_map_radius_ {30.0};
    bool use_voxel_hash_map_ {false};
    double voxel_hash_map_voxel_size_ {1.0};
    double voxel_hash_map_max_distance_ {100.0};
    int voxel_hash_map_max_points_per_voxel_ {20};
    std::unique_ptr<VoxelHashMapPCL> voxel_hash_map_;
    bool adaptive_correspondence_threshold_ {false};
    double adaptive_corr_dist_ema_ {0.0};
    double adaptive_corr_dist_ema_alpha_ {0.1};
    double adaptive_corr_dist_multiplier_ {3.0};
    bool async_map_update_ {true};
    int async_map_update_warmup_submaps_ {1};
    int recovery_clear_consecutive_accepted_ {1};
    int suspect_clear_consecutive_accepted_ {2};

    bool set_initial_pose_ {false};
    bool publish_tf_ {true};
    bool use_odom_ {false};
    bool odom_prior_planar_ {false};
    bool odom_prior_translation_only_ {false};
    bool odom_prior_suspect_recovery_only_ {false};
    double odom_prior_weight_ {1.0};
    bool use_imu_ {false};
    bool imu_translation_deskew_ {true};
    bool imu_rotation_deskew_use_orientation_ {true};
    bool imu_pose_prediction_enable_ {true};
    double imu_pose_prediction_max_age_ {0.2};
    double imu_pose_prediction_max_roll_pitch_deg_ {12.0};
    double imu_pose_prediction_max_yaw_deg_ {20.0};
    double imu_pose_prediction_weight_ {0.0};
    bool imu_complementary_enable_ {false};
    double imu_complementary_alpha_ {0.0};
    Eigen::Matrix4f ndt_pose_ {Eigen::Matrix4f::Identity()};
    bool ndt_pose_valid_ {false};
    bool imu_ndt_prior_enable_ {false};
    double imu_ndt_prior_weight_ {0.0};
    bool imu_ndt_prior_roll_pitch_only_ {true};
    bool imu_z_prior_enable_ {false};
    double imu_z_prior_weight_ {0.0};
    bool use_constant_velocity_model_ {false};
    bool debug_flag_ {false};
    std::string debug_cloud_dump_dir_;
    int debug_cloud_dump_max_frames_ {0};
    int debug_cloud_dump_frame_count_ {0};
    int cloud_queue_depth_ {5};
    double ndt_transformation_epsilon_ {0.01};
    int ndt_max_iterations_ {35};
    double ndt_outlier_ratio_ {0.55};
    double diagnostic_warn_trans_jump_ {0.75};
    double diagnostic_warn_yaw_jump_deg_ {12.0};
    bool reject_nonconverged_pose_update_ {true};
    double reject_fitness_score_ {0.0};
    double reject_fitness_ratio_ {2.5};
    double reject_fitness_only_ratio_ {8.0};
    double reject_trans_only_ratio_ {0.0};
    int reject_trans_streak_scans_ {0};
    double reject_fitness_streak_ratio_ {0.0};
    double reject_hard_fitness_ratio_ {0.0};
    double reject_trans_jump_ {1.0};
    double reject_trans_jump_ratio_ {3.0};
    double reject_hard_trans_ratio_ {0.0};
    double reject_ema_alpha_ {0.1};
    bool motion_gate_enable_ {true};
    double motion_gate_max_linear_velocity_ {8.0};
    double motion_gate_max_yaw_rate_deg_ {120.0};
    double motion_gate_hard_multiplier_ {4.0};
    int reject_warmup_scans_ {20};
    int reject_map_update_cooldown_scans_ {2};
    int hard_reject_map_update_cooldown_scans_ {4};
    int reject_map_update_cooldown_remaining_ {0};
    int reject_fitness_streak_scans_ {0};
    int elevated_fitness_streak_ {0};
    int elevated_trans_streak_ {0};
    int reject_recovery_scans_ {0};
    int consecutive_reject_count_ {0};
    double accepted_fitness_ema_ {0.0};
    double accepted_trans_ema_ {0.0};
    int accepted_pose_count_ {0};
    bool reject_stats_initialized_ {false};
    rclcpp::Time previous_pose_stamp_ {0, 0, RCL_ROS_TIME};
    rclcpp::Time last_cloud_stamp_ {0, 0, RCL_ROS_TIME};
    bool last_cloud_stamp_valid_ {false};
    Eigen::Vector3d previous_pose_diagnostic_position_ {Eigen::Vector3d::Zero()};
    double previous_pose_diagnostic_yaw_ {0.0};
    bool previous_pose_diagnostic_valid_ {false};
    Eigen::Vector3d last_accepted_delta_position_ {Eigen::Vector3d::Zero()};
    tf2::Quaternion last_accepted_delta_quat_ {0.0, 0.0, 0.0, 1.0};
    bool last_accepted_delta_valid_ {false};
    TrackingState tracking_state_ {TrackingState::Tracking};
    int state_clean_consecutive_accepted_ {0};
    bool recovery_target_active_ {false};

    // map
    Eigen::Vector3d previous_position_;
    double trans_;
    double latest_distance_ {0};

    // initial_pose
    double initial_pose_x_;
    double initial_pose_y_;
    double initial_pose_z_;
    double initial_pose_qx_;
    double initial_pose_qy_;
    double initial_pose_qz_;
    double initial_pose_qw_;

    // odom
    Eigen::Matrix4f previous_odom_mat_ {Eigen::Matrix4f::Identity()};
    bool previous_odom_valid_ {false};

    // imu
    double scan_period_ {0.1};
    double last_imu_time_ {0.0};
    double imu_integrated_yaw_ {0.0};
    bool imu_integrated_yaw_valid_ {false};
    tf2::Quaternion latest_imu_robot_quat_ {0.0, 0.0, 0.0, 1.0};
    rclcpp::Time latest_imu_stamp_ {0, 0, RCL_ROS_TIME};
    bool latest_imu_orientation_valid_ {false};
    tf2::Quaternion cloud_imu_reference_quat_ {0.0, 0.0, 0.0, 1.0};
    rclcpp::Time cloud_imu_reference_stamp_ {0, 0, RCL_ROS_TIME};
    bool cloud_imu_reference_valid_ {false};
    LidarUndistortion lidar_undistortion_;

  };
} // namespace graphslam

#endif  //GS_SM_COMPONENT_H_INCLUDED
