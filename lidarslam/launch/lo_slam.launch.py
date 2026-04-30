"""Launch scanmatcher + graph_based_slam in LiDAR-only (LO) configuration."""

# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """LO launch: same graph as lidarslam.launch.py, default LO parameters."""
    pkg_share = get_package_share_directory('lidarslam')
    main_param_dir_default = os.path.join(pkg_share, 'param', 'lidarslam_lo.yaml')
    rviz_config_default = os.path.join(pkg_share, 'rviz', 'mapping.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'main_param_dir',
            default_value=main_param_dir_default,
            description=(
                'Parameter YAML (scan_matcher + graph_based_slam); '
                'default is lidarslam_lo.yaml.'
            ),
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time (/clock). Required for rosbag playback.',
        ),
        DeclareLaunchArgument(
            'global_frame_id',
            default_value='map',
            description='Global frame id.',
        ),
        DeclareLaunchArgument(
            'robot_frame_id',
            default_value='base_link',
            description='Robot base frame id.',
        ),
        DeclareLaunchArgument(
            'odom_frame_id',
            default_value='odom',
            description='Odometry frame id (when scanmatcher use_odom:=true).',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config_default,
            description='RViz config path.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Start RViz.',
        ),
        DeclareLaunchArgument(
            'use_graph_based_slam',
            default_value='true',
            description='Start graph_based_slam backend.',
        ),
        DeclareLaunchArgument(
            'input_cloud',
            default_value='/points_raw',
            description='Input PointCloud2 topic.',
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu',
            description='IMU topic name (unused when scanmatcher use_imu:=false).',
        ),
        DeclareLaunchArgument(
            'gnss_topic',
            default_value='/gnss/fix',
            description='NavSatFix for graph_based_slam when use_gnss:=true.',
        ),
        DeclareLaunchArgument(
            'save_dir',
            default_value='.',
            description='Backend output directory (map, g2o).',
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value=LaunchConfiguration('robot_frame_id'),
            description='Parent frame for optional static lidar TF.',
        ),
        DeclareLaunchArgument(
            'lidar_frame',
            default_value='lidar',
            description='Child frame for optional static lidar TF.',
        ),
        DeclareLaunchArgument(
            'use_odom_input',
            default_value='false',
            description='graph_based_slam external odometry (RKO-LIO) mode.',
        ),
        DeclareLaunchArgument(
            'publish_static_tf',
            default_value='true',
            description='Publish static TF base_frame ->  lidar_frame.',
        ),
        DeclareLaunchArgument('static_tf_x', default_value='0'),
        DeclareLaunchArgument('static_tf_y', default_value='0'),
        DeclareLaunchArgument('static_tf_z', default_value='0'),
        DeclareLaunchArgument('static_tf_qx', default_value='0'),
        DeclareLaunchArgument('static_tf_qy', default_value='0'),
        DeclareLaunchArgument('static_tf_qz', default_value='0'),
        DeclareLaunchArgument('static_tf_qw', default_value='1'),
        Node(
            package='scanmatcher',
            executable='scanmatcher_node',
            parameters=[
                LaunchConfiguration('main_param_dir'),
                {
                    'global_frame_id': LaunchConfiguration('global_frame_id'),
                    'robot_frame_id': LaunchConfiguration('robot_frame_id'),
                    'odom_frame_id': LaunchConfiguration('odom_frame_id'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                },
            ],
            remappings=[
                ('/input_cloud', LaunchConfiguration('input_cloud')),
                ('/imu', LaunchConfiguration('imu_topic')),
            ],
            output='screen',
        ),
        Node(
            package='graph_based_slam',
            executable='graph_based_slam_node',
            parameters=[
                LaunchConfiguration('main_param_dir'),
                {
                    'global_frame_id': LaunchConfiguration('global_frame_id'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'use_odom_input': LaunchConfiguration('use_odom_input'),
                    'gnss_topic': LaunchConfiguration('gnss_topic'),
                    'map_save_dir': LaunchConfiguration('save_dir'),
                    'save_pose_graph_path': PathJoinSubstitution([
                        LaunchConfiguration('save_dir'),
                        'pose_graph.g2o',
                    ]),
                    'save_map_path': PathJoinSubstitution([
                        LaunchConfiguration('save_dir'),
                        'map.pcd',
                    ]),
                },
            ],
            remappings=[
                ('/imu', LaunchConfiguration('imu_topic')),
            ],
            condition=IfCondition(LaunchConfiguration('use_graph_based_slam')),
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                LaunchConfiguration('static_tf_x'),
                LaunchConfiguration('static_tf_y'),
                LaunchConfiguration('static_tf_z'),
                LaunchConfiguration('static_tf_qx'),
                LaunchConfiguration('static_tf_qy'),
                LaunchConfiguration('static_tf_qz'),
                LaunchConfiguration('static_tf_qw'),
                LaunchConfiguration('base_frame'),
                LaunchConfiguration('lidar_frame'),
            ],
            condition=IfCondition(LaunchConfiguration('publish_static_tf')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=['-d', LaunchConfiguration('rviz_config')],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            output='screen',
        ),
    ])
