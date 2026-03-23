"""Launch RKO-LIO offline_node + graph_based_slam together.

RKO-LIO provides LiDAR-inertial odometry; graph_based_slam adds loop-closure
on top via odom_input / cloud_input mode.  Defaults are tuned for the
Newer College dataset (OS1-64 + VN100).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("lidarslam")
    main_param_dir_default = os.path.join(pkg_share, "param", "lidarslam.yaml")
    rviz_config_default = os.path.join(pkg_share, "rviz", "mapping.rviz")

    return LaunchDescription(
        [
            # ── shared ────────────────────────────────────────────────
            DeclareLaunchArgument(
                "main_param_dir",
                default_value=main_param_dir_default,
                description="Full path to the lidarslam parameter YAML (graph_based_slam section).",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time (/clock).",
            ),
            DeclareLaunchArgument(
                "global_frame_id",
                default_value="map",
                description="Global frame id.",
            ),
            DeclareLaunchArgument(
                "save_dir",
                default_value=".",
                description="Directory for graph_based_slam outputs (pose_graph.g2o / map.pcd).",
            ),

            # ── RKO-LIO parameters ───────────────────────────────────
            DeclareLaunchArgument(
                "bag_path",
                default_value="",
                description="Path to the rosbag to play (required for offline_node).",
            ),
            DeclareLaunchArgument(
                "lidar_topic",
                default_value="/os_cloud_node/points",
                description="LiDAR point-cloud topic.",
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value="/os_cloud_node/imu",
                description="IMU topic.",
            ),
            DeclareLaunchArgument(
                "base_frame",
                default_value="os_sensor",
                description="Robot base frame (odometry is expressed in this frame).",
            ),
            DeclareLaunchArgument(
                "odom_frame",
                default_value="odom",
                description="Odometry parent frame.",
            ),
            DeclareLaunchArgument(
                "lidar_frame",
                default_value="",
                description="LiDAR frame id (empty = read from message header).",
            ),
            DeclareLaunchArgument(
                "imu_frame",
                default_value="",
                description="IMU frame id (empty = read from message header).",
            ),
            DeclareLaunchArgument(
                "publish_odom_tf",
                default_value="true",
                description="Let RKO-LIO broadcast odom->base_frame TF.",
            ),
            DeclareLaunchArgument(
                "deskew",
                default_value="true",
                description="Enable point-cloud deskewing.",
            ),
            DeclareLaunchArgument(
                "voxel_size",
                default_value="1.0",
                description="Local-map voxel size (m).",
            ),
            DeclareLaunchArgument(
                "max_range",
                default_value="100.0",
                description="Max valid LiDAR range (m).",
            ),
            DeclareLaunchArgument(
                "min_range",
                default_value="1.0",
                description="Min valid LiDAR range (m).",
            ),
            DeclareLaunchArgument(
                "initialization_phase",
                default_value="false",
                description="Use IMU data between first two frames to initialise bias/orientation.",
            ),
            DeclareLaunchArgument(
                "skip_to_time",
                default_value="0.0",
                description="Skip to this timestamp in the bag (seconds).",
            ),
            DeclareLaunchArgument(
                "dump_results",
                default_value="false",
                description="Dump RKO-LIO trajectory to disk on exit.",
            ),
            DeclareLaunchArgument(
                "results_dir",
                default_value="results",
                description="Output directory for RKO-LIO results.",
            ),
            DeclareLaunchArgument(
                "run_name",
                default_value="rko_lio_run",
                description="Run name tag for RKO-LIO results.",
            ),

            # ── graph_based_slam parameters ───────────────────────────
            DeclareLaunchArgument(
                "adjacent_edge_info_weight",
                default_value="1000.0",
                description="Information weight for adjacent edges in the pose graph.",
            ),
            DeclareLaunchArgument(
                "use_scan_context",
                default_value="true",
                description="Enable ScanContext-based loop detection.",
            ),
            DeclareLaunchArgument(
                "use_pcd_cache",
                default_value="true",
                description="Cache point-cloud data on disk to reduce memory.",
            ),
            DeclareLaunchArgument(
                "threshold_loop_closure_score",
                default_value="3.0",
                description="NDT fitness threshold for accepting a loop closure.",
            ),
            DeclareLaunchArgument(
                "distance_loop_closure",
                default_value="100.0",
                description="Max distance (m) to search for loop candidates.",
            ),

            # ── static TF (e.g. os_sensor -> os_imu) ─────────────────
            DeclareLaunchArgument(
                "publish_static_tf",
                default_value="true",
                description="Publish a static TF between two configurable frames.",
            ),
            DeclareLaunchArgument(
                "static_tf_parent",
                default_value="os_sensor",
                description="Parent frame for the static TF.",
            ),
            DeclareLaunchArgument(
                "static_tf_child",
                default_value="os_imu",
                description="Child frame for the static TF.",
            ),
            DeclareLaunchArgument("static_tf_x", default_value="0.006253"),
            DeclareLaunchArgument("static_tf_y", default_value="-0.011775"),
            DeclareLaunchArgument("static_tf_z", default_value="0.007645"),
            DeclareLaunchArgument("static_tf_qx", default_value="0"),
            DeclareLaunchArgument("static_tf_qy", default_value="0"),
            DeclareLaunchArgument("static_tf_qz", default_value="0"),
            DeclareLaunchArgument("static_tf_qw", default_value="1"),

            # ── rviz ──────────────────────────────────────────────────
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Start RViz.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=rviz_config_default,
                description="Full path to the RViz config file.",
            ),

            # ============================================================
            # Nodes
            # ============================================================

            # 1) RKO-LIO offline_node
            Node(
                package="rko_lio",
                executable="offline_node",
                name="rko_lio_offline_node",
                parameters=[
                    {
                        "bag_path": LaunchConfiguration("bag_path"),
                        "lidar_topic": LaunchConfiguration("lidar_topic"),
                        "imu_topic": LaunchConfiguration("imu_topic"),
                        "base_frame": LaunchConfiguration("base_frame"),
                        "odom_frame": LaunchConfiguration("odom_frame"),
                        "lidar_frame": LaunchConfiguration("lidar_frame"),
                        "imu_frame": LaunchConfiguration("imu_frame"),
                        "publish_odom_tf": LaunchConfiguration("publish_odom_tf"),
                        "deskew": LaunchConfiguration("deskew"),
                        "voxel_size": LaunchConfiguration("voxel_size"),
                        "max_range": LaunchConfiguration("max_range"),
                        "min_range": LaunchConfiguration("min_range"),
                        "initialization_phase": LaunchConfiguration("initialization_phase"),
                        "skip_to_time": LaunchConfiguration("skip_to_time"),
                        "publish_deskewed_scan": True,
                        "dump_results": LaunchConfiguration("dump_results"),
                        "results_dir": LaunchConfiguration("results_dir"),
                        "run_name": LaunchConfiguration("run_name"),
                    },
                ],
                output="screen",
                emulate_tty=True,
            ),

            # 2) graph_based_slam (odom_input mode)
            Node(
                package="graph_based_slam",
                executable="graph_based_slam_node",
                parameters=[
                    LaunchConfiguration("main_param_dir"),
                    {
                        "global_frame_id": LaunchConfiguration("global_frame_id"),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "use_odom_input": True,
                        "adjacent_edge_info_weight": LaunchConfiguration(
                            "adjacent_edge_info_weight"
                        ),
                        "use_scan_context": LaunchConfiguration("use_scan_context"),
                        "use_pcd_cache": LaunchConfiguration("use_pcd_cache"),
                        "threshold_loop_closure_score": LaunchConfiguration(
                            "threshold_loop_closure_score"
                        ),
                        "distance_loop_closure": LaunchConfiguration(
                            "distance_loop_closure"
                        ),
                        "save_pose_graph_path": PathJoinSubstitution(
                            [LaunchConfiguration("save_dir"), "pose_graph.g2o"]
                        ),
                        "save_map_path": PathJoinSubstitution(
                            [LaunchConfiguration("save_dir"), "map.pcd"]
                        ),
                    },
                ],
                remappings=[
                    ("odom_input", "/rko_lio/odometry"),
                    ("cloud_input", "/rko_lio/frame"),
                ],
                output="screen",
            ),

            # 3) Static TF (os_sensor -> os_imu by default)
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    LaunchConfiguration("static_tf_x"),
                    LaunchConfiguration("static_tf_y"),
                    LaunchConfiguration("static_tf_z"),
                    LaunchConfiguration("static_tf_qx"),
                    LaunchConfiguration("static_tf_qy"),
                    LaunchConfiguration("static_tf_qz"),
                    LaunchConfiguration("static_tf_qw"),
                    LaunchConfiguration("static_tf_parent"),
                    LaunchConfiguration("static_tf_child"),
                ],
                condition=IfCondition(LaunchConfiguration("publish_static_tf")),
            ),

            # 4) RViz (optional)
            Node(
                package="rviz2",
                executable="rviz2",
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
                arguments=["-d", LaunchConfiguration("rviz_config")],
                condition=IfCondition(LaunchConfiguration("use_rviz")),
                output="screen",
            ),
        ]
    )
