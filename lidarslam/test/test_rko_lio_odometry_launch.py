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

"""Tests for the MID-360 RKO-LIO online odometry launch."""

from __future__ import annotations

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions


REPO_ROOT = Path(__file__).resolve().parents[2]
LAUNCH_PATH = REPO_ROOT / 'lidarslam' / 'launch' / 'rko_lio_odometry.launch.py'
PARAM_PATH = REPO_ROOT / 'lidarslam' / 'param' / 'rko_lio_mid360.yaml'


def _load_launch_module():
    spec = importlib.util.spec_from_file_location('rko_lio_odometry_launch', LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _default_context(module):
    context = LaunchContext()
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            context.launch_configurations[entity.name] = perform_substitutions(
                context, entity.default_value
            )
    return context


def test_param_file_may_be_flat_or_ros_layout(tmp_path):
    module = _load_launch_module()
    flat = tmp_path / 'flat.yaml'
    flat.write_text('initialization_phase: true\n', encoding='utf-8')
    wrapped = tmp_path / 'wrapped.yaml'
    wrapped.write_text(
        '/**:\n  ros__parameters:\n    initialization_phase: true\n', encoding='utf-8'
    )
    assert module.load_rko_params(flat) == {'initialization_phase': True}
    assert module.load_rko_params(wrapped) == {'initialization_phase': True}


def test_defaults_track_a_standalone_mid360(monkeypatch):
    module = _load_launch_module()
    created = []
    monkeypatch.setattr(module, 'Node', lambda **kwargs: created.append(kwargs))
    context = _default_context(module)
    assert context.launch_configurations['rko_param_file'] == ''
    # The installed share holds the same file; read the source copy here.
    context.launch_configurations['rko_param_file'] = str(PARAM_PATH)

    module.create_online_node(context)

    (node,) = created
    assert node['executable'] == 'online_node'
    (parameters,) = node['parameters']
    identity = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
    assert parameters['extrinsic_lidar2base_quat_xyzw_xyz'] == identity
    assert parameters['extrinsic_imu2base_quat_xyzw_xyz'] == identity
    assert parameters['initialization_phase'] is True
    assert parameters['lidar_topic'] == '/livox/lidar'
    assert parameters['imu_topic'] == '/livox/imu'
    assert parameters['base_frame'] == 'livox_frame'
    assert parameters['odom_frame'] == 'odom'
    assert parameters['use_sim_time'] is False

    context.launch_configurations['use_sim_time'] = 'true'
    context.launch_configurations['base_frame'] = 'base_link'
    created.clear()
    module.create_online_node(context)
    (parameters,) = created[0]['parameters']
    assert parameters['use_sim_time'] is True
    assert parameters['base_frame'] == 'base_link'
