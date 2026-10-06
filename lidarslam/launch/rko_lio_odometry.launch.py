"""Run RKO-LIO online odometry, publishing odom -> the LiDAR frame."""

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

# Defaults fit a Livox MID-360 on its own (no robot TF tree): the odometry
# tracks the sensor frame, the IMU and LiDAR extrinsics are identity, the odom
# frame is levelled with gravity at startup, and IMU acceleration published in
# g (as Livox drivers do) is detected and converted to m/s^2.
# lidar_localization_ros2's quickstart picks this odometry up when localizing
# on a lidar_slam_ros2 map.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


DEFAULT_RKO_PARAM_FILE = 'rko_lio_mid360.yaml'


def load_rko_params(path):
    """Read an RKO-LIO parameter file, flat or in the ros__parameters layout."""
    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}
    for value in data.values():
        if isinstance(value, dict) and 'ros__parameters' in value:
            return dict(value['ros__parameters'])
    return dict(data)


def create_online_node(context, *args, **kwargs):
    """Create the online node from the parameter file and launch arguments."""
    del args
    del kwargs
    param_file = LaunchConfiguration('rko_param_file').perform(context)
    if not param_file:
        param_file = os.path.join(
            get_package_share_directory('lidarslam'), 'param', DEFAULT_RKO_PARAM_FILE
        )
    parameters = load_rko_params(param_file)
    for name in (
        'lidar_topic', 'imu_topic', 'base_frame', 'odom_frame', 'imu_acceleration_unit',
    ):
        parameters[name] = LaunchConfiguration(name).perform(context)
    parameters['use_sim_time'] = (
        LaunchConfiguration('use_sim_time').perform(context).strip().lower()
        in ('true', '1', 'yes', 'on')
    )
    return [
        Node(
            package='rko_lio',
            executable='online_node',
            name='rko_lio',
            parameters=[parameters],
            output='screen',
            emulate_tty=True,
        ),
    ]


def generate_launch_description():
    """Declare MID-360 defaults; override them for other sensors or mounts."""
    return LaunchDescription([
        DeclareLaunchArgument('lidar_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        DeclareLaunchArgument(
            'base_frame',
            default_value='livox_frame',
            description='Frame the odometry tracks (odom -> base_frame).',
        ),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument(
            'imu_acceleration_unit',
            default_value='auto',
            description='mps2, g, or auto (g when the first sample is below half of g).',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Set true when replaying a bag with --clock.',
        ),
        DeclareLaunchArgument(
            'rko_param_file',
            default_value='',
            description='RKO-LIO parameters (extrinsics, initialization); default '
            f'param/{DEFAULT_RKO_PARAM_FILE}.',
        ),
        OpaqueFunction(function=create_online_node),
    ])
