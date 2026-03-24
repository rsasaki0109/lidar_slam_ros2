"""Launch graph_based_slam with a configurable parameter file."""

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
import launch
import launch_ros.actions


def generate_launch_description():
    """Create the graph_based_slam launch description."""
    graphbasedslam_param_dir = launch.substitutions.LaunchConfiguration(
        'graphbasedslam_param_dir',
        default=os.path.join(
            get_package_share_directory('graph_based_slam'),
            'param',
            'graphbasedslam.yaml',
        ),
    )

    graphbasedslam = launch_ros.actions.Node(
        package='graph_based_slam',
        executable='graph_based_slam_node',
        parameters=[graphbasedslam_param_dir],
        output='screen',
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'graphbasedslam_param_dir',
            default_value=graphbasedslam_param_dir,
            description='Full path to graphbasedslam parameter file to load',
        ),
        graphbasedslam,
    ])
