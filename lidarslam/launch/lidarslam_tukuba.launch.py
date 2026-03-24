"""Launch the Tukuba demo configuration."""

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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create the Tukuba demo launch description."""
    pkg_share = get_package_share_directory('lidarslam')
    main_param_dir_default = os.path.join(
        pkg_share,
        'param',
        'lidarslam_tukuba.yaml',
    )
    rviz_param_dir_default = os.path.join(
        pkg_share,
        'rviz',
        'mapping_tukuba.rviz',
    )

    main_param_dir = LaunchConfiguration('main_param_dir')
    rviz_param_dir = LaunchConfiguration('rviz_param_dir')

    return LaunchDescription([
        DeclareLaunchArgument(
            'main_param_dir',
            default_value=main_param_dir_default,
            description='Full path to the main parameter file to load.',
        ),
        DeclareLaunchArgument(
            'rviz_param_dir',
            default_value=rviz_param_dir_default,
            description='Full path to the RViz config file to load.',
        ),
        Node(
            package='scanmatcher',
            executable='scanmatcher_node',
            parameters=[main_param_dir],
            remappings=[('/input_cloud', '/points_raw')],
            output='screen',
        ),
        Node(
            package='graph_based_slam',
            executable='graph_based_slam_node',
            parameters=[main_param_dir],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'velodyne'],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_param_dir],
        ),
    ])
