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

"""Unit tests for MID-360 robot recording helpers."""

from __future__ import annotations

import importlib
from pathlib import Path
import sys

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_DIR = REPO_ROOT / 'scripts'


def _load_record_module():
    if str(SCRIPTS_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPTS_DIR))
    return importlib.import_module('mid360_robot_record_tools')


def _load_robot_module():
    if str(SCRIPTS_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPTS_DIR))
    return importlib.import_module('mid360_robot_tools')


def _profile(tmp_path: Path):
    robot_module = _load_robot_module()
    profile_path = tmp_path / 'profile.yaml'
    profile_path.write_text(
        yaml.safe_dump({
            'robot_name': 'record_robot',
            'base_frame': 'base_link',
            'lidar_frame': 'livox_frame',
            'imu_frame': 'livox_frame',
            'expected_pointcloud_topic': '/livox/lidar',
            'expected_imu_topic': '/livox/imu',
        }),
        encoding='utf-8',
    )
    return robot_module.RobotProfileLoader().load(profile_path)


def test_record_planner_builds_rosbag_command(tmp_path: Path):
    module = _load_record_module()
    profile = _profile(tmp_path)

    plan = module.Mid360RobotRecordPlanner().build_plan(
        profile,
        module.RecordOptions(
            bag_root=tmp_path / 'bags',
            run_id='field test 01',
            duration_sec='30',
            extra_topics=('/diagnostics', '/tf'),
            storage_id='sqlite3',
            max_cache_size='104857600',
        ),
    )

    assert plan.run_id == 'field_test_01'
    assert plan.bag_path == tmp_path / 'bags' / 'field_test_01'
    assert plan.topics == ('/livox/lidar', '/livox/imu', '/tf', '/tf_static', '/diagnostics')
    assert plan.command[:6] == (
        'timeout',
        '30',
        'ros2',
        'bag',
        'record',
        '-o',
    )
    assert '--storage' in plan.command
    assert '--max-cache-size' in plan.command
    assert '/livox/lidar' in plan.command
    assert '/livox/imu' in plan.command


def test_record_manifest_writer_persists_plan_and_profile_snapshot(tmp_path: Path):
    module = _load_record_module()
    profile = _profile(tmp_path)
    plan = module.Mid360RobotRecordPlanner().build_plan(
        profile,
        module.RecordOptions(bag_root=tmp_path / 'bags', run_id='record_run'),
    )

    paths = module.Mid360RecordManifestWriter().write(profile, plan)

    assert paths['json'].is_file()
    assert paths['markdown'].is_file()
    assert paths['profile'].is_file()
    assert '/livox/lidar' in paths['json'].read_text(encoding='utf-8')
    assert 'ros2 bag record' in paths['markdown'].read_text(encoding='utf-8')
    assert 'record_robot' in paths['profile'].read_text(encoding='utf-8')


def test_record_planner_requires_profile_topics(tmp_path: Path):
    module = _load_record_module()
    robot_module = _load_robot_module()
    profile = robot_module.RobotProfile(
        robot_name='missing_topics',
        frames=robot_module.RobotFrames(),
    )

    try:
        module.Mid360RobotRecordPlanner().build_plan(
            profile,
            module.RecordOptions(bag_root=tmp_path / 'bags'),
        )
    except ValueError as exc:
        assert 'expected_pointcloud_topic' in str(exc)
    else:
        raise AssertionError('expected missing profile topic to fail')
