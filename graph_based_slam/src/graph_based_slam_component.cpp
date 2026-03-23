#include "graph_based_slam/graph_based_slam_component.h"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>

using namespace std::chrono_literals;

namespace graphslam
{
GraphBasedSlamComponent::GraphBasedSlamComponent(const rclcpp::NodeOptions & options)
: Node("graph_based_slam", options),
  clock_(RCL_ROS_TIME),
  tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
  listener_(tfbuffer_),
  broadcaster_(this)
{
  RCLCPP_INFO(get_logger(), "initialization start");
  std::string registration_method;
  double voxel_leaf_size;
  double ndt_resolution;
  int ndt_num_threads;

  declare_parameter("registration_method", "NDT");
  get_parameter("registration_method", registration_method);
  declare_parameter("voxel_leaf_size", 0.2);
  get_parameter("voxel_leaf_size", voxel_leaf_size);
  declare_parameter("ndt_resolution", 5.0);
  get_parameter("ndt_resolution", ndt_resolution);
  declare_parameter("ndt_num_threads", 0);
  get_parameter("ndt_num_threads", ndt_num_threads);
  declare_parameter("loop_detection_period", 1000);
  get_parameter("loop_detection_period", loop_detection_period_);
  declare_parameter("threshold_loop_closure_score", 1.0);
  get_parameter("threshold_loop_closure_score", threshold_loop_closure_score_);
  declare_parameter("distance_loop_closure", 20.0);
  get_parameter("distance_loop_closure", distance_loop_closure_);
  declare_parameter("range_of_searching_loop_closure", 20.0);
  get_parameter("range_of_searching_loop_closure", range_of_searching_loop_closure_);
  declare_parameter("search_submap_num", 3);
  get_parameter("search_submap_num", search_submap_num_);
  declare_parameter("num_adjacent_pose_cnstraints", 5);
  get_parameter("num_adjacent_pose_cnstraints", num_adjacent_pose_cnstraints_);
  declare_parameter("use_save_map_in_loop", true);
  get_parameter("use_save_map_in_loop", use_save_map_in_loop_);
  declare_parameter("debug_flag", false);
  get_parameter("debug_flag", debug_flag_);
  declare_parameter("adjacent_edge_info_weight", 1000.0);
  get_parameter("adjacent_edge_info_weight", adjacent_edge_info_weight_);
  declare_parameter("use_scan_context", false);
  get_parameter("use_scan_context", use_scan_context_);
  declare_parameter("use_pcd_cache", false);
  get_parameter("use_pcd_cache", use_pcd_cache_);
  declare_parameter("pcd_cache_dir", std::string("/tmp/graph_slam_pcd_cache"));
  get_parameter("pcd_cache_dir", pcd_cache_dir_);
  if (use_pcd_cache_) {
    std::filesystem::create_directories(pcd_cache_dir_);
    std::cout << "pcd_cache_dir:" << pcd_cache_dir_ << std::endl;
  }
  declare_parameter("scan_context_threshold", 0.3);
  get_parameter("scan_context_threshold", scan_context_threshold_);
  declare_parameter("map_save_dir", std::string("."));
  get_parameter("map_save_dir", map_save_dir_);
  declare_parameter("map_grid_size_x", 20.0);
  get_parameter("map_grid_size_x", map_grid_size_x_);
  declare_parameter("map_grid_size_y", 20.0);
  get_parameter("map_grid_size_y", map_grid_size_y_);
  declare_parameter("map_leaf_size", 0.2);
  get_parameter("map_leaf_size", map_leaf_size_);
  declare_parameter("use_gnss", false);
  get_parameter("use_gnss", use_gnss_);
  declare_parameter("gnss_info_weight", 1.0);
  get_parameter("gnss_info_weight", gnss_info_weight_);
  declare_parameter("use_imu_preintegration", false);
  get_parameter("use_imu_preintegration", use_imu_preintegration_);
  declare_parameter("imu_rotation_info_roll_pitch", 100.0);
  get_parameter("imu_rotation_info_roll_pitch", imu_rotation_info_roll_pitch_);
  declare_parameter("imu_rotation_info_yaw", 10.0);
  get_parameter("imu_rotation_info_yaw", imu_rotation_info_yaw_);

  std::cout << "registration_method:" << registration_method << std::endl;
  std::cout << "voxel_leaf_size[m]:" << voxel_leaf_size << std::endl;
  std::cout << "ndt_resolution[m]:" << ndt_resolution << std::endl;
  std::cout << "ndt_num_threads:" << ndt_num_threads << std::endl;
  std::cout << "loop_detection_period[Hz]:" << loop_detection_period_ << std::endl;
  std::cout << "threshold_loop_closure_score:" << threshold_loop_closure_score_ << std::endl;
  std::cout << "distance_loop_closure[m]:" << distance_loop_closure_ << std::endl;
  std::cout << "range_of_searching_loop_closure[m]:" << range_of_searching_loop_closure_ <<
    std::endl;
  std::cout << "search_submap_num:" << search_submap_num_ << std::endl;
  std::cout << "num_adjacent_pose_cnstraints:" << num_adjacent_pose_cnstraints_ << std::endl;
  std::cout << "use_save_map_in_loop:" << std::boolalpha << use_save_map_in_loop_ << std::endl;
  std::cout << "debug_flag:" << std::boolalpha << debug_flag_ << std::endl;
  std::cout << "use_scan_context:" << std::boolalpha << use_scan_context_ << std::endl;
  if (use_scan_context_) {
    std::cout << "scan_context_threshold:" << scan_context_threshold_ << std::endl;
  }
  declare_parameter("use_odom_input", false);
  get_parameter("use_odom_input", use_odom_input_);
  declare_parameter("submap_distance_threshold", 1.5);
  get_parameter("submap_distance_threshold", submap_distance_threshold_);
  std::cout << "use_odom_input:" << std::boolalpha << use_odom_input_ << std::endl;
  if (use_odom_input_) {
    std::cout << "submap_distance_threshold[m]:" << submap_distance_threshold_ << std::endl;
  }
  std::cout << "use_imu_preintegration:" << std::boolalpha << use_imu_preintegration_ << std::endl;
  if (use_imu_preintegration_) {
    std::cout << "imu_rotation_info_roll_pitch:" << imu_rotation_info_roll_pitch_ << std::endl;
    std::cout << "imu_rotation_info_yaw:" << imu_rotation_info_yaw_ << std::endl;
  }
  std::cout << "------------------" << std::endl;

  voxelgrid_.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);

  if (registration_method == "NDT") {
	  boost::shared_ptr<pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>>
      ndt(new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt->setMaximumIterations(100);
    ndt->setResolution(ndt_resolution);
    ndt->setTransformationEpsilon(0.01);
    // ndt->setTransformationEpsilon(1e-6);
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    if (ndt_num_threads > 0) {ndt->setNumThreads(ndt_num_threads);}
    registration_ = ndt;
  } else if (registration_method == "GICP") {
	  boost::shared_ptr<pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>>
      gicp(new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    gicp->setMaxCorrespondenceDistance(30);
    gicp->setMaximumIterations(100);
    //gicp->setCorrespondenceRandomness(20);
    gicp->setTransformationEpsilon(1e-8);
    gicp->setEuclideanFitnessEpsilon(1e-6);
    gicp->setRANSACIterations(0);
    registration_ = gicp;
  } else {
    RCLCPP_ERROR(get_logger(), "invalid registration_method");
    exit(1);
  }

  initializePubSub();

  auto map_save_callback =
    [this](const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      const std::shared_ptr<std_srvs::srv::Empty::Response> response) -> void
    {
      std::cout << "Received an request to save the map" << std::endl;
      if (initial_map_array_received_ == false) {
        std::cout << "initial map is not received" << std::endl;
        return;
      }
      doPoseAdjustment(map_array_msg_, true);
    };

  map_save_srv_ = create_service<std_srvs::srv::Empty>("map_save", map_save_callback);

}

void GraphBasedSlamComponent::initializePubSub()
{
  RCLCPP_INFO(get_logger(), "initialize Publishers and Subscribers");

  auto map_array_callback =
    [this](const typename lidarslam_msgs::msg::MapArray::SharedPtr msg_ptr) -> void
    {
      std::lock_guard<std::mutex> lock(mtx_);
      map_array_msg_ = *msg_ptr;
      // Save new submaps to PCD and clear cloud from memory
      if (use_pcd_cache_) {
        for (int i = 0; i < static_cast<int>(map_array_msg_.submaps.size()); i++) {
          auto& sub = map_array_msg_.submaps[i];
          if (sub.cloud.data.size() > 0) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(sub.cloud, *cloud);
            if (cloud->size() > 0) {
              saveSubmapToPCD(i, cloud);
              sub.cloud = sensor_msgs::msg::PointCloud2();  // Free memory
            }
          }
        }
      }
      initial_map_array_received_ = true;
      is_map_array_updated_ = true;
    };

  map_array_sub_ =
    create_subscription<lidarslam_msgs::msg::MapArray>(
    "map_array", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(), map_array_callback);

  if (use_odom_input_) {
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom_input", 10,
      std::bind(&GraphBasedSlamComponent::receiveOdometry, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud_input", rclcpp::SensorDataQoS(),
      std::bind(&GraphBasedSlamComponent::receiveCloud, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Direct odom+cloud input mode enabled");
  }

  std::chrono::milliseconds period(loop_detection_period_);
  loop_detect_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&GraphBasedSlamComponent::searchLoop, this)
  );

  modified_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "modified_map",
    rclcpp::QoS(10));

  modified_map_array_pub_ = create_publisher<lidarslam_msgs::msg::MapArray>(
    "modified_map_array", rclcpp::QoS(10));

  modified_path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "modified_path",
    rclcpp::QoS(10));

  if (use_imu_preintegration_) {
    auto imu_callback =
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) -> void
      {
        receiveImu(*msg);
      };
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu", rclcpp::SensorDataQoS(), imu_callback);
    RCLCPP_INFO(get_logger(), "IMU preintegration enabled, subscribed to /imu");
  }

  if (use_gnss_) {
    gnss_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gnss/fix", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) { receiveNavSatFix(*msg); });
    RCLCPP_INFO(get_logger(), "GNSS constraints enabled, subscribed to /gnss/fix");
  }

  RCLCPP_INFO(get_logger(), "initialization end");

}

void GraphBasedSlamComponent::searchLoop()
{

  if (initial_map_array_received_ == false) {return;}
  if (is_map_array_updated_ == false) {return;}
  if (map_array_msg_.cloud_coordinate != map_array_msg_.LOCAL) {
    RCLCPP_WARN(get_logger(), "cloud_coordinate should be local, but it's not local.");
  }
  is_map_array_updated_ = false;

  lidarslam_msgs::msg::MapArray map_array_msg = map_array_msg_;
  std::lock_guard<std::mutex> lock(mtx_);
  int num_submaps = map_array_msg.submaps.size();

  if(debug_flag_)
  {
    RCLCPP_INFO(get_logger(), "searching Loop, num_submaps:%d", num_submaps);
  }

  // Update Scan Context database for new submaps (one at a time)
  if (use_scan_context_ && scan_context_db_.size() < num_submaps) {
    int idx = num_submaps - 1;
    if (scan_context_db_.size() <= idx) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
      if (use_pcd_cache_) {
        cloud = loadSubmapFromPCD(idx);
      } else {
        cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(map_array_msg.submaps[idx].cloud, *cloud);
      }
      scan_context_db_.add(ScanContext::computeDescriptor(cloud));
    }
  }

  double min_fitness_score = std::numeric_limits<double>::max();
  double distance_min_fitness_score = 0;
  bool is_candidate = false;

  lidarslam_msgs::msg::SubMap latest_submap;
  latest_submap = map_array_msg.submaps[num_submaps - 1];

  // Aggregate latest N submaps as source (improves matching quality)
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_latest_submap_cloud_ptr(
    new pcl::PointCloud<pcl::PointXYZI>);
  for (int k = 0; k < search_submap_num_ && (num_submaps - 1 - k) >= 0; k++) {
    int src_idx = num_submaps - 1 - k;
    auto& src_submap = map_array_msg.submaps[src_idx];
    pcl::PointCloud<pcl::PointXYZI>::Ptr src_cloud;
    if (use_pcd_cache_) {
      src_cloud = loadSubmapFromPCD(src_idx);
    } else {
      src_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(src_submap.cloud, *src_cloud);
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_src(new pcl::PointCloud<pcl::PointXYZI>);
    Eigen::Affine3d src_affine;
    tf2::fromMsg(src_submap.pose, src_affine);
    pcl::transformPointCloud(*src_cloud, *transformed_src, src_affine.matrix().cast<float>());
    *transformed_latest_submap_cloud_ptr += *transformed_src;
  }
  // Downsample source
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_source(new pcl::PointCloud<pcl::PointXYZI>);
  voxelgrid_.setInputCloud(transformed_latest_submap_cloud_ptr);
  voxelgrid_.filter(*filtered_source);
  registration_->setInputSource(filtered_source);
  double latest_moving_distance = latest_submap.distance;
  Eigen::Vector3d latest_submap_pos{
    latest_submap.pose.position.x,
    latest_submap.pose.position.y,
    latest_submap.pose.position.z};
  int id_min = 0;
  double min_dist = std::numeric_limits<double>::max();
  lidarslam_msgs::msg::SubMap min_submap;

  // Scan Context-based candidate selection (if enabled)
  if (use_scan_context_ && scan_context_db_.size() > ScanContext::EXCLUDE_RECENT) {
    auto [sc_idx, sc_dist] = scan_context_db_.query(
      scan_context_db_.descriptors.back(),
      ScanContext::NUM_CANDIDATES,
      ScanContext::EXCLUDE_RECENT,
      scan_context_threshold_);

    if (sc_idx >= 0 && sc_idx < num_submaps) {
      is_candidate = true;
      id_min = sc_idx;
      min_submap = map_array_msg.submaps[sc_idx];
      std::cout << "ScanContext loop candidate: id=" << sc_idx
                << " sc_dist=" << sc_dist << std::endl;
    }
  }

  // Distance-based candidate selection (fallback or when Scan Context disabled)
  if (!is_candidate) {
    for (int i = 0; i < num_submaps; i++) {
      auto submap = map_array_msg.submaps[i];
      Eigen::Vector3d submap_pos{submap.pose.position.x, submap.pose.position.y,
        submap.pose.position.z};
      double dist = (latest_submap_pos - submap_pos).norm();
      if (latest_moving_distance - submap.distance > distance_loop_closure_ &&
        dist < range_of_searching_loop_closure_)
      {
        is_candidate = true;
        if (dist < min_dist) {
        id_min = i;
        min_dist = dist;
        min_submap = submap;
        }
      }
    }
  }  // end distance-based fallback

  if (is_candidate) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr submap_clouds_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    for (int j = 0; j <= 2 * search_submap_num_; ++j) {
      if (id_min + j - search_submap_num_ < 0) {continue;}
      int near_idx = id_min + j - search_submap_num_;
      auto near_submap = map_array_msg.submaps[near_idx];
      pcl::PointCloud<pcl::PointXYZI>::Ptr submap_cloud_ptr;
      if (use_pcd_cache_) {
        submap_cloud_ptr = loadSubmapFromPCD(near_idx);
      } else {
        submap_cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(near_submap.cloud, *submap_cloud_ptr);
      }
      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_submap_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZI>);
      Eigen::Affine3d affine;
      tf2::fromMsg(near_submap.pose, affine);
      pcl::transformPointCloud(
        *submap_cloud_ptr, *transformed_submap_cloud_ptr,
        affine.matrix().cast<float>());
      *submap_clouds_ptr += *transformed_submap_cloud_ptr;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_clouds_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    voxelgrid_.setInputCloud(submap_clouds_ptr);
    voxelgrid_.filter(*filtered_clouds_ptr);
    registration_->setInputTarget(filtered_clouds_ptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    registration_->align(*output_cloud_ptr);
    double fitness_score = registration_->getFitnessScore();

    if (fitness_score < threshold_loop_closure_score_) {

      Eigen::Affine3d init_affine;
      tf2::fromMsg(latest_submap.pose, init_affine);
      Eigen::Affine3d submap_affine;
      tf2::fromMsg(min_submap.pose, submap_affine);

      LoopEdge loop_edge;
      loop_edge.pair_id = std::pair<int, int>(id_min, num_submaps - 1);
      Eigen::Isometry3d from = Eigen::Isometry3d(submap_affine.matrix());
      Eigen::Isometry3d to = Eigen::Isometry3d(
        registration_->getFinalTransformation().cast<double>() * init_affine.matrix());

      loop_edge.relative_pose = Eigen::Isometry3d(from.inverse() * to);
      loop_edges_.push_back(loop_edge);

      std::cout << "---" << std::endl;
      std::cout << "PoseAdjustment distance:" << min_submap.distance << ", score:" << fitness_score << std::endl;
      std::cout << "id_loop_point 1:" << id_min << " id_loop_point 2:" << num_submaps - 1 << std::endl;
      std::cout << "final transformation:" << std::endl;
      std::cout << registration_->getFinalTransformation() << std::endl;
      doPoseAdjustment(map_array_msg, use_save_map_in_loop_);

      return;
    }
    std::cout << "min_submap_distance:" << min_submap.distance << " min_fitness_score:" << fitness_score << std::endl;
  }
}

void GraphBasedSlamComponent::doPoseAdjustment(
  lidarslam_msgs::msg::MapArray map_array_msg,
  bool do_save_map)
{

  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linear_solver =
    std::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
  g2o::OptimizationAlgorithmLevenberg * solver = new g2o::OptimizationAlgorithmLevenberg(
    std::make_unique<g2o::BlockSolver_6_3>(std::move(linear_solver)));

  optimizer.setAlgorithm(solver);

  int submaps_size = map_array_msg.submaps.size();
  Eigen::Matrix<double, 6, 6> info_mat = Eigen::Matrix<double, 6, 6>::Identity() * adjacent_edge_info_weight_;
  for (int i = 0; i < submaps_size; i++) {
    Eigen::Affine3d affine;
    Eigen::fromMsg(map_array_msg.submaps[i].pose, affine);
    Eigen::Isometry3d pose(affine.matrix());

    g2o::VertexSE3 * vertex_se3 = new g2o::VertexSE3();
    vertex_se3->setId(i);
    vertex_se3->setEstimate(pose);
    if (i == 0) {vertex_se3->setFixed(true);}
    optimizer.addVertex(vertex_se3);

    if (i > num_adjacent_pose_cnstraints_) {
      for (int j = 0; j < num_adjacent_pose_cnstraints_; j++) {
        Eigen::Affine3d pre_affine;
        Eigen::fromMsg(
          map_array_msg.submaps[i - num_adjacent_pose_cnstraints_ + j].pose,
          pre_affine);
        Eigen::Isometry3d pre_pose(pre_affine.matrix());
        Eigen::Isometry3d relative_pose = pre_pose.inverse() * pose;
        g2o::EdgeSE3 * edge_se3 = new g2o::EdgeSE3();
        edge_se3->setMeasurement(relative_pose);
        edge_se3->setInformation(info_mat);
        edge_se3->vertices()[0] = optimizer.vertex(i - num_adjacent_pose_cnstraints_ + j);
        edge_se3->vertices()[1] = optimizer.vertex(i);
        optimizer.addEdge(edge_se3);
      }
    }

  }
  /* IMU rotation constraint edges */
  if (use_imu_preintegration_ && submaps_size > 1) {
    std::lock_guard<std::mutex> imu_lock(imu_mtx_);
    int imu_edges_added = 0;
    for (int i = 1; i < submaps_size; i++) {
      double t0 = rclcpp::Time(map_array_msg.submaps[i - 1].header.stamp).seconds();
      double t1 = rclcpp::Time(map_array_msg.submaps[i].header.stamp).seconds();
      if (t1 <= t0 || t1 - t0 > 30.0) { continue; }

      Eigen::Quaterniond imu_delta_q = integrateImuRotation(t0, t1);
      if (imu_delta_q.isApprox(Eigen::Quaterniond::Identity(), 1e-8)) { continue; }

      // Build relative pose measurement: translation from odometry, rotation from IMU
      Eigen::Affine3d affine_prev, affine_curr;
      Eigen::fromMsg(map_array_msg.submaps[i - 1].pose, affine_prev);
      Eigen::fromMsg(map_array_msg.submaps[i].pose, affine_curr);
      Eigen::Isometry3d odom_prev(affine_prev.matrix());
      Eigen::Isometry3d odom_curr(affine_curr.matrix());
      Eigen::Isometry3d odom_relative = odom_prev.inverse() * odom_curr;

      // Replace rotation with IMU-integrated rotation
      Eigen::Isometry3d imu_relative = Eigen::Isometry3d::Identity();
      imu_relative.linear() = imu_delta_q.toRotationMatrix();
      imu_relative.translation() = odom_relative.translation();

      g2o::EdgeSE3 * edge_se3 = new g2o::EdgeSE3();
      edge_se3->setMeasurement(imu_relative);

      // Information matrix: high for roll/pitch rotation, moderate for yaw, zero for translation
      Eigen::Matrix<double, 6, 6> imu_info = Eigen::Matrix<double, 6, 6>::Zero();
      // g2o EdgeSE3 information: [rot(3) | trans(3)] order
      imu_info(0, 0) = imu_rotation_info_roll_pitch_;  // roll
      imu_info(1, 1) = imu_rotation_info_roll_pitch_;  // pitch
      imu_info(2, 2) = imu_rotation_info_yaw_;         // yaw
      // translation: zero weight (don't trust IMU double integration)
      edge_se3->setInformation(imu_info);

      edge_se3->vertices()[0] = optimizer.vertex(i - 1);
      edge_se3->vertices()[1] = optimizer.vertex(i);
      optimizer.addEdge(edge_se3);
      imu_edges_added++;
    }
    if (debug_flag_) {
      RCLCPP_INFO(get_logger(), "Added %d IMU rotation constraint edges", imu_edges_added);
    }
  }

  /* loop edge */
  Eigen::Matrix<double, 6, 6> loop_info_mat = Eigen::Matrix<double, 6, 6>::Identity();
  for (auto loop_edge : loop_edges_) {
    g2o::EdgeSE3 * edge_se3 = new g2o::EdgeSE3();
    edge_se3->setMeasurement(loop_edge.relative_pose);
    edge_se3->setInformation(loop_info_mat);
    edge_se3->vertices()[0] = optimizer.vertex(loop_edge.pair_id.first);
    edge_se3->vertices()[1] = optimizer.vertex(loop_edge.pair_id.second);
    optimizer.addEdge(edge_se3);
  }

  /* GNSS position constraints */
  if (use_gnss_ && gnss_origin_set_) {
    std::lock_guard<std::mutex> gnss_lock(gnss_mtx_);
    int gnss_edges_added = 0;
    // GNSS info: position only (translation), no rotation constraint
    Eigen::Matrix<double, 6, 6> gnss_info = Eigen::Matrix<double, 6, 6>::Zero();
    gnss_info(3, 3) = gnss_info_weight_;  // x
    gnss_info(4, 4) = gnss_info_weight_;  // y
    gnss_info(5, 5) = gnss_info_weight_ * 0.1;  // z (less reliable for GNSS)

    for (int i = 0; i < submaps_size; i++) {
      double submap_time = rclcpp::Time(map_array_msg.submaps[i].header.stamp).seconds();
      // Find nearest GNSS measurement
      double best_dt = std::numeric_limits<double>::max();
      GnssEnu best_gnss;
      bool found = false;
      for (const auto& g : gnss_buffer_) {
        double dt = std::abs(g.stamp - submap_time);
        if (dt < best_dt) {
          best_dt = dt;
          best_gnss = g;
          found = true;
        }
      }
      if (!found || best_dt > 1.0) continue;  // Skip if no GNSS within 1 second

      // Create unary-like constraint: edge from vertex i to a fixed GNSS position
      // Use EdgeSE3 with vertex 0 = fixed GNSS pose, vertex 1 = submap
      int gnss_vertex_id = submaps_size + gnss_edges_added;
      g2o::VertexSE3 * gnss_vertex = new g2o::VertexSE3();
      gnss_vertex->setId(gnss_vertex_id);
      Eigen::Isometry3d gnss_pose = Eigen::Isometry3d::Identity();
      gnss_pose.translation() = Eigen::Vector3d(best_gnss.x, best_gnss.y, best_gnss.z);
      gnss_vertex->setEstimate(gnss_pose);
      gnss_vertex->setFixed(true);
      optimizer.addVertex(gnss_vertex);

      g2o::EdgeSE3 * edge = new g2o::EdgeSE3();
      edge->setMeasurement(Eigen::Isometry3d::Identity());
      edge->setInformation(gnss_info);
      edge->vertices()[0] = gnss_vertex;
      edge->vertices()[1] = optimizer.vertex(i);
      optimizer.addEdge(edge);
      gnss_edges_added++;
    }
    if (debug_flag_) {
      RCLCPP_INFO(get_logger(), "Added %d GNSS position constraint edges", gnss_edges_added);
    }
  }

  optimizer.initializeOptimization();
  optimizer.optimize(10);
  optimizer.save("pose_graph.g2o");

  /* modified_map publish */
  std::cout << "modified_map publish" << std::endl;
  lidarslam_msgs::msg::MapArray modified_map_array_msg;
  modified_map_array_msg.header = map_array_msg.header;
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  for (int i = 0; i < submaps_size; i++) {
    g2o::VertexSE3 * vertex_se3 = static_cast<g2o::VertexSE3 *>(optimizer.vertex(i));
    Eigen::Affine3d se3 = vertex_se3->estimate();
    geometry_msgs::msg::Pose pose = tf2::toMsg(se3);

    /* map */
    Eigen::Affine3d previous_affine;
    tf2::fromMsg(map_array_msg.submaps[i].pose, previous_affine);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr;
    if (use_pcd_cache_) {
      cloud_ptr = loadSubmapFromPCD(i);
    } else {
      cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(map_array_msg.submaps[i].cloud, *cloud_ptr);
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(
      new pcl::PointCloud<pcl::PointXYZI>());

    pcl::transformPointCloud(*cloud_ptr, *transformed_cloud_ptr, se3.matrix().cast<float>());
    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*transformed_cloud_ptr, *cloud_msg_ptr);
    *map_ptr += *transformed_cloud_ptr;

    /* submap */
    lidarslam_msgs::msg::SubMap submap;
    submap.header = map_array_msg.submaps[i].header;
    submap.pose = pose;
    submap.cloud = *cloud_msg_ptr;
    modified_map_array_msg.submaps.push_back(submap);

    /* path */
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = submap.header;
    pose_stamped.pose = submap.pose;
    path.poses.push_back(pose_stamped);

  }

  modified_map_array_pub_->publish(modified_map_array_msg);
  modified_path_pub_->publish(path);

  sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*map_ptr, *map_msg_ptr);
  map_msg_ptr->header.frame_id = "map";
  modified_map_pub_->publish(*map_msg_ptr);
  if (do_save_map) {
    saveGridDividedMap(map_ptr);
  }

}

void GraphBasedSlamComponent::receiveNavSatFix(const sensor_msgs::msg::NavSatFix & msg)
{
  if (msg.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
    return;  // No valid fix
  }
  // Reject obviously invalid coordinates
  if (std::abs(msg.latitude) < 1e-6 && std::abs(msg.longitude) < 1e-6) {
    return;
  }

  std::lock_guard<std::mutex> lock(gnss_mtx_);

  if (!gnss_origin_set_) {
    gnss_origin_lat_ = msg.latitude;
    gnss_origin_lon_ = msg.longitude;
    gnss_origin_alt_ = msg.altitude;
    gnss_origin_set_ = true;
    RCLCPP_INFO(get_logger(), "GNSS origin set: lat=%.8f, lon=%.8f, alt=%.2f",
      gnss_origin_lat_, gnss_origin_lon_, gnss_origin_alt_);
  }

  Eigen::Vector3d enu = geodeticToEnu(msg.latitude, msg.longitude, msg.altitude);
  GnssEnu g;
  g.stamp = rclcpp::Time(msg.header.stamp).seconds();
  g.x = enu.x();
  g.y = enu.y();
  g.z = enu.z();
  gnss_buffer_.push_back(g);

  // Limit buffer size
  if (gnss_buffer_.size() > 100000) {
    gnss_buffer_.erase(gnss_buffer_.begin(), gnss_buffer_.begin() + 25000);
  }
}

Eigen::Vector3d GraphBasedSlamComponent::geodeticToEnu(
  double lat, double lon, double alt) const
{
  // WGS84 parameters
  constexpr double a = 6378137.0;              // semi-major axis [m]
  constexpr double f = 1.0 / 298.257223563;    // flattening
  constexpr double e2 = 2 * f - f * f;         // eccentricity squared

  auto toRad = [](double deg) { return deg * M_PI / 180.0; };

  double lat0 = toRad(gnss_origin_lat_);
  double lon0 = toRad(gnss_origin_lon_);
  double lat1 = toRad(lat);
  double lon1 = toRad(lon);

  double dlat = lat1 - lat0;
  double dlon = lon1 - lon0;
  double dalt = alt - gnss_origin_alt_;

  double sin_lat0 = std::sin(lat0);
  double N = a / std::sqrt(1.0 - e2 * sin_lat0 * sin_lat0);
  double M = a * (1.0 - e2) / std::pow(1.0 - e2 * sin_lat0 * sin_lat0, 1.5);

  // ENU: East = dlon * N * cos(lat), North = dlat * M, Up = dalt
  double east = dlon * N * std::cos(lat0);
  double north = dlat * M;
  double up = dalt;

  return Eigen::Vector3d(east, north, up);
}

void GraphBasedSlamComponent::receiveImu(const sensor_msgs::msg::Imu & msg)
{
  std::lock_guard<std::mutex> lock(imu_mtx_);
  StampedImu imu;
  imu.stamp = rclcpp::Time(msg.header.stamp).seconds();
  imu.gx = msg.angular_velocity.x;
  imu.gy = msg.angular_velocity.y;
  imu.gz = msg.angular_velocity.z;
  imu.ax = msg.linear_acceleration.x;
  imu.ay = msg.linear_acceleration.y;
  imu.az = msg.linear_acceleration.z;
  imu.qx = msg.orientation.x;
  imu.qy = msg.orientation.y;
  imu.qz = msg.orientation.z;
  imu.qw = msg.orientation.w;
  imu_buffer_.push_back(imu);
  if (imu_buffer_.size() > kMaxImuBufferSize) {
    imu_buffer_.erase(imu_buffer_.begin(), imu_buffer_.begin() + kMaxImuBufferSize / 4);
  }
}

Eigen::Quaterniond GraphBasedSlamComponent::integrateImuRotation(double t0, double t1) const
{
  // Integrate gyroscope measurements between t0 and t1
  Eigen::Quaterniond delta_q = Eigen::Quaterniond::Identity();

  // Find first IMU sample >= t0
  auto it = std::lower_bound(imu_buffer_.begin(), imu_buffer_.end(), t0,
    [](const StampedImu & imu, double t) { return imu.stamp < t; });

  if (it == imu_buffer_.end()) {
    return delta_q;  // no data
  }

  double prev_t = t0;
  for (; it != imu_buffer_.end() && it->stamp <= t1; ++it) {
    double dt = it->stamp - prev_t;
    if (dt <= 0.0 || dt > 0.5) {
      prev_t = it->stamp;
      continue;
    }
    // Small angle quaternion integration
    Eigen::Vector3d omega(it->gx, it->gy, it->gz);
    double angle = omega.norm() * dt;
    if (angle > 1e-10) {
      Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, omega.normalized()));
      delta_q = delta_q * dq;
      delta_q.normalize();
    }
    prev_t = it->stamp;
  }

  return delta_q;
}

void GraphBasedSlamComponent::receiveCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (debug_flag_ && !latest_cloud_) {
    RCLCPP_INFO(get_logger(), "First cloud received, %zu bytes", msg->data.size());
  }
  latest_cloud_ = msg;
  latest_cloud_stamp_ = rclcpp::Time(msg->header.stamp);
  // When cloud arrives, try to create submap with latest odom
  tryCreateSubmap();
}

void GraphBasedSlamComponent::receiveOdometry(const nav_msgs::msg::Odometry & msg)
{
  // Buffer latest odom
  Eigen::Vector3d pos(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
  if (!std::isfinite(pos.x()) || !std::isfinite(pos.y()) || !std::isfinite(pos.z())) {
    return;
  }
  if (debug_flag_ && !latest_odom_valid_) {
    RCLCPP_INFO(get_logger(), "First odom received: (%.2f, %.2f, %.2f)", pos.x(), pos.y(), pos.z());
  }
  latest_odom_ = msg;
  latest_odom_valid_ = true;
}

void GraphBasedSlamComponent::tryCreateSubmap()
{
  if (!latest_odom_valid_ || !latest_cloud_) return;

  Eigen::Vector3d pos(
    latest_odom_.pose.pose.position.x,
    latest_odom_.pose.pose.position.y,
    latest_odom_.pose.pose.position.z);

  // Check distance threshold
  if (last_submap_position_valid_) {
    double dist = (pos - last_submap_position_).norm();
    if (dist < submap_distance_threshold_) return;
    if (dist > 100.0) return;
    accumulated_distance_ += dist;
  }
  last_submap_position_ = pos;
  last_submap_position_valid_ = true;

  // Create SubMap (use "map" frame for SLAM output regardless of odom frame)
  lidarslam_msgs::msg::SubMap submap;
  submap.header.stamp = latest_odom_.header.stamp;
  submap.header.frame_id = "map";
  submap.distance = accumulated_distance_;
  submap.pose = latest_odom_.pose.pose;
  submap.cloud = *latest_cloud_;
  submap.cloud.header.frame_id = latest_odom_.child_frame_id;

  int n;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    map_array_msg_.header.stamp = latest_odom_.header.stamp;
    map_array_msg_.header.frame_id = "map";
    map_array_msg_.submaps.push_back(submap);
    n = map_array_msg_.submaps.size();

    // Save to PCD and clear cloud from memory
    if (use_pcd_cache_) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(submap.cloud, *cloud);
      saveSubmapToPCD(n - 1, cloud);
      // Clear cloud data from memory (keep pose and metadata)
      map_array_msg_.submaps.back().cloud = sensor_msgs::msg::PointCloud2();
    }

    initial_map_array_received_ = true;
    is_map_array_updated_ = true;
  }

  if (n % 50 == 0) {
    RCLCPP_INFO(get_logger(), "Odom input: %d submaps, distance: %.1fm", n, accumulated_distance_);
  }
}

void GraphBasedSlamComponent::saveSubmapToPCD(int idx, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
  std::string path = pcd_cache_dir_ + "/submap_" + std::to_string(idx) + ".pcd";
  pcl::io::savePCDFileBinaryCompressed(path, *cloud);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr GraphBasedSlamComponent::loadSubmapFromPCD(int idx)
{
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  std::string path = pcd_cache_dir_ + "/submap_" + std::to_string(idx) + ".pcd";
  if (pcl::io::loadPCDFile(path, *cloud) == -1) {
    RCLCPP_WARN(get_logger(), "Failed to load PCD: %s", path.c_str());
  }
  return cloud;
}

void GraphBasedSlamComponent::saveGridDividedMap(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& map)
{
  if (map->empty()) {
    std::cout << "Map is empty, skipping save." << std::endl;
    return;
  }

  // Create output directory
  std::string out_dir = map_save_dir_ + "/pointcloud_map";
  std::filesystem::create_directories(out_dir);

  // Downsample the map
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setInputCloud(map);
  vg.setLeafSize(map_leaf_size_, map_leaf_size_, map_leaf_size_);
  vg.filter(*downsampled);

  std::cout << "Map points: " << map->size() << " -> " << downsampled->size()
            << " (leaf=" << map_leaf_size_ << "m)" << std::endl;

  // Compute bounding box
  pcl::PointXYZI min_pt, max_pt;
  pcl::getMinMax3D(*downsampled, min_pt, max_pt);

  // Compute grid bounds (align to grid)
  double x_min = std::floor(min_pt.x / map_grid_size_x_) * map_grid_size_x_;
  double y_min = std::floor(min_pt.y / map_grid_size_y_) * map_grid_size_y_;
  double x_max = std::ceil(max_pt.x / map_grid_size_x_) * map_grid_size_x_;
  double y_max = std::ceil(max_pt.y / map_grid_size_y_) * map_grid_size_y_;

  int nx = static_cast<int>((x_max - x_min) / map_grid_size_x_);
  int ny = static_cast<int>((y_max - y_min) / map_grid_size_y_);
  if (nx <= 0) nx = 1;
  if (ny <= 0) ny = 1;

  // Assign points to grid cells
  std::map<std::pair<int, int>, pcl::PointCloud<pcl::PointXYZI>::Ptr> grid_cells;
  for (const auto& pt : downsampled->points) {
    int gx = static_cast<int>(std::floor((pt.x - x_min) / map_grid_size_x_));
    int gy = static_cast<int>(std::floor((pt.y - y_min) / map_grid_size_y_));
    auto key = std::make_pair(gx, gy);
    if (grid_cells.find(key) == grid_cells.end()) {
      grid_cells[key] = pcl::PointCloud<pcl::PointXYZI>::Ptr(
        new pcl::PointCloud<pcl::PointXYZI>);
    }
    grid_cells[key]->push_back(pt);
  }

  // Save each grid cell as PCD and build metadata
  // Format: Autoware pointcloud_map_loader expects:
  //   x_resolution: 20.0
  //   y_resolution: 20.0
  //   filename.pcd: [x, y]   (lower-left corner of grid cell)
  std::ofstream meta(out_dir + "/pointcloud_map_metadata.yaml");
  meta << std::fixed;
  meta << "x_resolution: " << std::setprecision(1) << map_grid_size_x_ << std::endl;
  meta << "y_resolution: " << std::setprecision(1) << map_grid_size_y_ << std::endl;

  int saved = 0;
  for (auto& [key, cloud] : grid_cells) {
    if (cloud->empty()) continue;
    double cell_x = x_min + key.first * map_grid_size_x_;
    double cell_y = y_min + key.second * map_grid_size_y_;

    std::ostringstream filename;
    filename << std::fixed << std::setprecision(0)
             << cell_x << "_" << cell_y << ".pcd";
    std::string filepath = out_dir + "/" + filename.str();
    pcl::io::savePCDFileBinaryCompressed(filepath, *cloud);

    meta << filename.str() << ": [" << std::setprecision(1)
         << cell_x << ", " << cell_y << "]" << std::endl;
    saved++;
  }

  meta.close();

  // Also save the full map as a single PCD for convenience
  pcl::io::savePCDFileBinaryCompressed(map_save_dir_ + "/map.pcd", *downsampled);

  std::cout << "Saved grid-divided map: " << saved << " cells ("
            << map_grid_size_x_ << "x" << map_grid_size_y_ << "m) to " << out_dir
            << std::endl;
  std::cout << "Total points: " << downsampled->size() << std::endl;
  std::cout << "Metadata: " << out_dir << "/pointcloud_map_metadata.yaml" << std::endl;

  // Save GNSS origin for Autoware's map_projection_loader
  if (gnss_origin_set_) {
    std::string proj_file = map_save_dir_ + "/map_projector_info.yaml";
    std::ofstream proj(proj_file);
    proj << std::fixed << std::setprecision(10);
    proj << "projector_type: local" << std::endl;
    proj << "vertical_datum: WGS84" << std::endl;
    proj << "map_origin:" << std::endl;
    proj << "  latitude: " << gnss_origin_lat_ << std::endl;
    proj << "  longitude: " << gnss_origin_lon_ << std::endl;
    proj << "  altitude: " << std::setprecision(3) << gnss_origin_alt_ << std::endl;
    proj.close();
    std::cout << "GNSS origin saved: " << proj_file << std::endl;
  }
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(graphslam::GraphBasedSlamComponent)
