#include "graph_based_slam/graph_based_slam_component.h"
#include <chrono>

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

  declare_parameter("global_frame_id", "map");
  get_parameter("global_frame_id", global_frame_id_);
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
  declare_parameter("num_adjacent_pose_constraints", 5);
  get_parameter("num_adjacent_pose_constraints", num_adjacent_pose_constraints_);
  declare_parameter("use_save_map_in_loop", true);
  get_parameter("use_save_map_in_loop", use_save_map_in_loop_);
  declare_parameter("debug_flag", false);
  get_parameter("debug_flag", debug_flag_);

  RCLCPP_INFO(get_logger(), "global_frame_id: %s", global_frame_id_.c_str());
  RCLCPP_INFO(get_logger(), "registration_method: %s", registration_method.c_str());
  RCLCPP_INFO(get_logger(), "voxel_leaf_size[m]: %f", voxel_leaf_size);
  RCLCPP_INFO(get_logger(), "ndt_resolution[m]: %f", ndt_resolution);
  RCLCPP_INFO(get_logger(), "ndt_num_threads: %d", ndt_num_threads);
  RCLCPP_INFO(get_logger(), "loop_detection_period[ms]: %d", loop_detection_period_);
  RCLCPP_INFO(get_logger(), "threshold_loop_closure_score: %f", threshold_loop_closure_score_);
  RCLCPP_INFO(get_logger(), "distance_loop_closure[m]: %f", distance_loop_closure_);
  RCLCPP_INFO(get_logger(), "range_of_searching_loop_closure[m]: %f", range_of_searching_loop_closure_);
  RCLCPP_INFO(get_logger(), "search_submap_num: %d", search_submap_num_);
  RCLCPP_INFO(get_logger(), "num_adjacent_pose_constraints: %d", num_adjacent_pose_constraints_);
  RCLCPP_INFO(get_logger(), "use_save_map_in_loop: %s", use_save_map_in_loop_ ? "true" : "false");
  RCLCPP_INFO(get_logger(), "debug_flag: %s", debug_flag_ ? "true" : "false");

  if (voxel_leaf_size <= 0.0) {
    throw std::invalid_argument("voxel_leaf_size must be positive");
  }
  voxelgrid_.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);

  // Create registration using factory pattern
  RegistrationParams reg_params;
  reg_params.method = registration_method;
  reg_params.ndt_resolution = ndt_resolution;
  reg_params.ndt_num_threads = ndt_num_threads;
  reg_params.gicp_corr_dist_threshold = loop_closure_defaults::kGicpCorrDistThreshold;
  reg_params.max_iterations = loop_closure_defaults::kMaxIterations;
  reg_params.euclidean_fitness_epsilon = loop_closure_defaults::kEuclideanFitnessEpsilon;
  reg_params.ransac_iterations = loop_closure_defaults::kRansacIterations;

  try {
    registration_ = RegistrationFactory::create(reg_params);
  } catch (const std::invalid_argument& e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    throw;
  }

  initializePubSub();

  auto map_save_callback =
    [this](const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      const std::shared_ptr<std_srvs::srv::Empty::Response> response) -> void
    {
      (void)request_header;
      (void)request;
      (void)response;
      RCLCPP_INFO(get_logger(), "Received a request to save the map");
      if (initial_map_array_received_ == false) {
        RCLCPP_WARN(get_logger(), "initial map is not received");
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
      std::scoped_lock lock(mtx_);
      map_array_msg_ = *msg_ptr;
      initial_map_array_received_ = true;
      is_map_array_updated_ = true;
    };

  map_array_sub_ =
    create_subscription<lidarslam_msgs::msg::MapArray>(
    "map_array", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(), map_array_callback);

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

  RCLCPP_INFO(get_logger(), "initialization end");

}

void GraphBasedSlamComponent::searchLoop()
{

  if (initial_map_array_received_ == false) {return;}
  if (is_map_array_updated_ == false) {return;}

  lidarslam_msgs::msg::MapArray map_array_msg;
  {
    std::scoped_lock lock(mtx_);
    if (map_array_msg_.cloud_coordinate != map_array_msg_.LOCAL) {
      RCLCPP_WARN(get_logger(), "cloud_coordinate should be local, but it's not local.");
    }
    is_map_array_updated_ = false;
    map_array_msg = map_array_msg_;
  }
  int num_submaps = map_array_msg.submaps.size();

  if(debug_flag_)
  {
    RCLCPP_INFO(get_logger(), "searching Loop, num_submaps:%d", num_submaps);
  }

  double min_fitness_score = std::numeric_limits<double>::max();
  double distance_min_fitness_score = 0;
  bool is_candidate = false;

  lidarslam_msgs::msg::SubMap latest_submap;
  latest_submap = map_array_msg.submaps[num_submaps - 1];
  Eigen::Affine3d latest_submap_affine;
  tf2::fromMsg(latest_submap.pose, latest_submap_affine);
  pcl::PointCloud<pcl::PointXYZI>::Ptr latest_submap_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(latest_submap.cloud, *latest_submap_cloud_ptr);
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_latest_submap_cloud_ptr(
    new pcl::PointCloud<pcl::PointXYZI>);
  Eigen::Affine3d latest_affine;
  tf2::fromMsg(latest_submap.pose, latest_affine);
  pcl::transformPointCloud(
    *latest_submap_cloud_ptr, *transformed_latest_submap_cloud_ptr,
    latest_affine.matrix().cast<float>());
  registration_->setInputSource(transformed_latest_submap_cloud_ptr);
  double latest_moving_distance = latest_submap.distance;
  Eigen::Vector3d latest_submap_pos{
    latest_submap.pose.position.x,
    latest_submap.pose.position.y,
    latest_submap.pose.position.z};
  int id_min = 0;
  double min_dist = std::numeric_limits<double>::max();
  lidarslam_msgs::msg::SubMap min_submap;
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

  if (is_candidate) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr submap_clouds_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    for (int j = 0; j <= 2 * search_submap_num_; ++j) {
      if (id_min + j - search_submap_num_ < 0) {continue;}
      auto near_submap = map_array_msg.submaps[id_min + j - search_submap_num_];
      pcl::PointCloud<pcl::PointXYZI>::Ptr submap_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(near_submap.cloud, *submap_cloud_ptr);
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

      RCLCPP_INFO(get_logger(), "---");
      RCLCPP_INFO(get_logger(), "PoseAdjustment distance: %f, score: %f", min_submap.distance, fitness_score);
      RCLCPP_INFO(get_logger(), "id_loop_point 1: %d, id_loop_point 2: %d", id_min, num_submaps - 1);
      RCLCPP_INFO(get_logger(), "Loop closure detected, performing pose adjustment");
      doPoseAdjustment(map_array_msg, use_save_map_in_loop_);

      return;
    }
    RCLCPP_DEBUG(get_logger(), "min_submap_distance: %f, min_fitness_score: %f", min_submap.distance, fitness_score);
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
  Eigen::Matrix<double, 6, 6> info_mat = Eigen::Matrix<double, 6, 6>::Identity();
  for (int i = 0; i < submaps_size; i++) {
    Eigen::Affine3d affine;
    Eigen::fromMsg(map_array_msg.submaps[i].pose, affine);
    Eigen::Isometry3d pose(affine.matrix());

    g2o::VertexSE3 * vertex_se3 = new g2o::VertexSE3();
    vertex_se3->setId(i);
    vertex_se3->setEstimate(pose);
    if (i == 0) {vertex_se3->setFixed(true);}
    optimizer.addVertex(vertex_se3);

    if (i > num_adjacent_pose_constraints_) {
      for (int j = 0; j < num_adjacent_pose_constraints_; j++) {
        Eigen::Affine3d pre_affine;
        Eigen::fromMsg(
          map_array_msg.submaps[i - num_adjacent_pose_constraints_ + j].pose,
          pre_affine);
        Eigen::Isometry3d pre_pose(pre_affine.matrix());
        Eigen::Isometry3d relative_pose = pre_pose.inverse() * pose;
        g2o::EdgeSE3 * edge_se3 = new g2o::EdgeSE3();
        edge_se3->setMeasurement(relative_pose);
        edge_se3->setInformation(info_mat);
        edge_se3->vertices()[0] = optimizer.vertex(i - num_adjacent_pose_constraints_ + j);
        edge_se3->vertices()[1] = optimizer.vertex(i);
        optimizer.addEdge(edge_se3);
      }
    }

  }
  /* loop edge */
  for (auto loop_edge : loop_edges_) {
    g2o::EdgeSE3 * edge_se3 = new g2o::EdgeSE3();
    edge_se3->setMeasurement(loop_edge.relative_pose);
    edge_se3->setInformation(info_mat);
    edge_se3->vertices()[0] = optimizer.vertex(loop_edge.pair_id.first);
    edge_se3->vertices()[1] = optimizer.vertex(loop_edge.pair_id.second);
    optimizer.addEdge(edge_se3);
  }

  optimizer.initializeOptimization();
  optimizer.optimize(10);
  optimizer.save("pose_graph.g2o");

  /* modified_map publish */
  RCLCPP_INFO(get_logger(), "Publishing modified map");
  lidarslam_msgs::msg::MapArray modified_map_array_msg;
  modified_map_array_msg.header = map_array_msg.header;
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_id_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  for (int i = 0; i < submaps_size; i++) {
    g2o::VertexSE3 * vertex_se3 = static_cast<g2o::VertexSE3 *>(optimizer.vertex(i));
    Eigen::Affine3d se3 = vertex_se3->estimate();
    geometry_msgs::msg::Pose pose = tf2::toMsg(se3);

    /* map */
    Eigen::Affine3d previous_affine;
    tf2::fromMsg(map_array_msg.submaps[i].pose, previous_affine);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(
      new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(map_array_msg.submaps[i].cloud, *cloud_ptr);

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
  map_msg_ptr->header.frame_id = global_frame_id_;
  modified_map_pub_->publish(*map_msg_ptr);
  if (do_save_map) {pcl::io::savePCDFileASCII("map.pcd", *map_ptr);} // too heavy

}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(graphslam::GraphBasedSlamComponent)
