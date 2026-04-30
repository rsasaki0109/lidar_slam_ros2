"""Launch small_gicp ICP/GICP odometry + graph_based_slam backend."""

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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _parse_bool_arg(value: str) -> bool:
    return value.strip().lower() in ('true', '1', 'yes', 'on')


def _add_optional_param_override(overrides, context, arg_name, parser):
    value = LaunchConfiguration(arg_name).perform(context)
    if value == '':
        return
    overrides[arg_name] = parser(value)


def create_small_gicp_node(context, *args, **kwargs):
    del args
    del kwargs

    parameters = []
    parameters.append(
        {
            'odom_frame': LaunchConfiguration('odom_frame_id'),
            'lidar_frame': LaunchConfiguration('robot_frame_id'),
            'publish_tf': LaunchConfiguration('publish_tf'),
        }
    )

    param_file = LaunchConfiguration('small_gicp_param_file').perform(context)
    if param_file:
        parameters.append(param_file)

    overrides = {}
    _add_optional_param_override(overrides, context, 'downsampling_resolution', float)
    _add_optional_param_override(overrides, context, 'voxel_resolution', float)
    _add_optional_param_override(
        overrides, context, 'max_correspondence_distance', float
    )
    _add_optional_param_override(overrides, context, 'num_neighbors', int)
    _add_optional_param_override(overrides, context, 'num_threads', int)
    _add_optional_param_override(overrides, context, 'max_range', float)
    _add_optional_param_override(overrides, context, 'min_range', float)
    _add_optional_param_override(overrides, context, 'min_motion_threshold', float)
    _add_optional_param_override(overrides, context, 'use_gicp', _parse_bool_arg)
    if overrides:
        parameters.append(overrides)

    return [
        Node(
            package='scanmatcher',
            executable='small_gicp_odom_node',
            namespace='small_gicp',
            parameters=parameters,
            remappings=[
                ('pointcloud', LaunchConfiguration('input_cloud')),
            ],
            output='screen',
        )
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('lidarslam')
    main_param_dir_default = os.path.join(pkg_share, 'param', 'lidarslam_lo.yaml')
    rviz_config_default = os.path.join(pkg_share, 'rviz', 'mapping.rviz')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'main_param_dir',
                default_value=main_param_dir_default,
                description=(
                    'Parameter YAML for graph_based_slam '
                    '(and optional scan_matcher block).'
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
                description='Robot/LiDAR frame id (child of odom TF).',
            ),
            DeclareLaunchArgument(
                'odom_frame_id',
                default_value='odom',
                description='Odometry frame id.',
            ),
            DeclareLaunchArgument(
                'input_cloud',
                default_value='/points_raw',
                description='Input point cloud topic (sensor_msgs/PointCloud2).',
            ),
            DeclareLaunchArgument(
                'gnss_topic',
                default_value='/gnss/fix',
                description='NavSatFix topic for graph_based_slam when use_gnss:=true.',
            ),
            DeclareLaunchArgument(
                'save_dir',
                default_value='.',
                description='Directory for backend outputs (pose_graph.g2o/map.pcd).',
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
                description='Start the graph_based_slam backend node.',
            ),
            DeclareLaunchArgument(
                'publish_tf',
                default_value='true',
                description='Publish odom->robot TF from the frontend.',
            ),
            DeclareLaunchArgument(
                'small_gicp_param_file',
                default_value='',
                description='Optional YAML file for small_gicp_odom_node parameters.',
            ),
            # Optional overrides (empty = keep defaults)
            DeclareLaunchArgument('downsampling_resolution', default_value=''),
            DeclareLaunchArgument('voxel_resolution', default_value=''),
            DeclareLaunchArgument('max_correspondence_distance', default_value=''),
            DeclareLaunchArgument('num_neighbors', default_value=''),
            DeclareLaunchArgument('num_threads', default_value=''),
            DeclareLaunchArgument('max_range', default_value=''),
            DeclareLaunchArgument('min_range', default_value=''),
            DeclareLaunchArgument('min_motion_threshold', default_value=''),
            DeclareLaunchArgument('use_gicp', default_value=''),
            DeclareLaunchArgument(
                'publish_static_tf',
                default_value='false',
                description=(
                    'Publish an identity static TF from base_frame to lidar_frame '
                    '(rarely needed here).'
                ),
            ),
            DeclareLaunchArgument(
                'base_frame', default_value=LaunchConfiguration('robot_frame_id')
            ),
            DeclareLaunchArgument(
                'lidar_frame', default_value=LaunchConfiguration('robot_frame_id')
            ),
            DeclareLaunchArgument('static_tf_x', default_value='0'),
            DeclareLaunchArgument('static_tf_y', default_value='0'),
            DeclareLaunchArgument('static_tf_z', default_value='0'),
            DeclareLaunchArgument('static_tf_qx', default_value='0'),
            DeclareLaunchArgument('static_tf_qy', default_value='0'),
            DeclareLaunchArgument('static_tf_qz', default_value='0'),
            DeclareLaunchArgument('static_tf_qw', default_value='1'),
            OpaqueFunction(function=create_small_gicp_node),
            Node(
                package='graph_based_slam',
                executable='graph_based_slam_node',
                parameters=[
                    LaunchConfiguration('main_param_dir'),
                    {
                        'global_frame_id': LaunchConfiguration('global_frame_id'),
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'use_odom_input': True,
                        'gnss_topic': LaunchConfiguration('gnss_topic'),
                        'map_save_dir': LaunchConfiguration('save_dir'),
                        'save_pose_graph_path': PathJoinSubstitution(
                            [LaunchConfiguration('save_dir'), 'pose_graph.g2o']
                        ),
                        'save_map_path': PathJoinSubstitution(
                            [LaunchConfiguration('save_dir'), 'map.pcd']
                        ),
                    },
                ],
                remappings=[
                    ('odom_input', '/small_gicp/odom'),
                    ('cloud_input', LaunchConfiguration('input_cloud')),
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
        ]
    )
