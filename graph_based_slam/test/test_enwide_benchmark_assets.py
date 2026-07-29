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

"""Validate the preregistered ENWIDE SOTA benchmark assets."""

from __future__ import annotations

import subprocess
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
PROFILE = (
    ROOT / 'configs' / 'slam_benchmark_profiles'
    / 'degenerate_lio_sota_v1.yaml'
)
DOWNLOADER = ROOT / 'scripts' / 'download_enwide.sh'
RUNNER = ROOT / 'scripts' / 'run_enwide_sota_benchmark.sh'
RKO_CONFIG = (
    ROOT / 'configs' / 'enwide' / 'rko_lio_os0_degenerate_sota_v1.yaml'
)


def _profile():
    return yaml.safe_load(PROFILE.read_text())['degenerate_lio_sota_profile']


def test_enwide_profile_preregisters_no_external_velocity_track():
    profile = _profile()
    assert profile['status'] == 'preregistered_report_only'
    assert profile['claim_policy']['sota_claim_allowed'] is False
    assert profile['track']['required_modalities'] == [
        'lidar_geometry', 'lidar_intensity', 'imu',
    ]
    assert set(profile['track']['forbidden_modalities']) == {
        'radar', 'wheel_odometry', 'gnss', 'camera',
    }
    assert profile['execution_contract']['parameter_policy'] == (
        'one_frozen_parameter_set'
    )
    assert (
        profile['execution_contract']['minimum_matched_ground_truth_fraction']
        == 0.98
    )
    assert profile['execution_contract']['candidate_config'] == str(
        RKO_CONFIG.relative_to(ROOT)
    )


def test_enwide_profile_freezes_tunnel_inputs_and_published_reference():
    datasets = _profile()['datasets']
    assert set(datasets) == {'enwide_tunnel_s', 'enwide_tunnel_d'}
    assert datasets['enwide_tunnel_s']['expected_bag_bytes'] == 14983936757
    assert datasets['enwide_tunnel_d']['expected_bag_bytes'] == 7485669675
    assert datasets['enwide_tunnel_s']['published_coin_lio'] == {
        'ate_rmse_m': 0.743,
        'rte_percent': 1.60,
    }
    assert datasets['enwide_tunnel_d']['published_coin_lio'] == {
        'ate_rmse_m': 0.487,
        'rte_percent': 1.59,
    }
    assert all(not dataset['tuning_allowed'] for dataset in datasets.values())


def test_enwide_rival_revisions_are_full_commit_hashes():
    for rival in _profile()['rivals'].values():
        revision = rival['revision']
        assert len(revision) == 40
        int(revision, 16)


def test_enwide_downloader_help_is_offline_and_documents_safe_modes():
    completed = subprocess.run(
        ['bash', str(DOWNLOADER), '--help'],
        check=True,
        capture_output=True,
        text=True,
    )
    assert '--metadata-only' in completed.stdout
    assert '--convert' in completed.stdout
    assert 'tunnel_s|tunnel_d|all' in completed.stdout
    text = DOWNLOADER.read_text()
    assert '.ros2-convert.XXXXXX' in text
    assert "'version': importlib.metadata.version('rosbags')" in text


def test_enwide_rko_config_uses_official_os0_extrinsic_and_no_external_velocity():
    config = yaml.safe_load(RKO_CONFIG.read_text())
    assert config['extrinsic_lidar2base_quat_xyzw_xyz'] == [
        0.0, 0.0, 1.0, 0.0, -0.006253, 0.011775, 0.028525,
    ]
    assert config['lidar_timestamps.force_relative'] is True
    assert config['intensity_constraint'] is True
    assert config['radar_velocity_fusion'] is False
    assert config['radar_velocity_continuous_fusion'] is False


def test_enwide_runner_exposes_only_dataset_output_and_repetition_options():
    completed = subprocess.run(
        ['bash', str(RUNNER), '--help'],
        check=True,
        capture_output=True,
        text=True,
    )
    assert '--sequence-dir' in completed.stdout
    assert '--output-dir' in completed.stdout
    assert '--runs' in completed.stdout
    for forbidden_option in (
            '--lidar-topic', '--imu-topic', '--rko-param', '--segment-length'):
        assert forbidden_option not in completed.stdout
    text = RUNNER.read_text()
    assert 'd5793a58dd8bd6743b5173172cd2ae2086d44e03' in text
    assert '--completion-end-margin-secs 1.0' in text
    assert '--skip-map-save' in text
    assert "'sota_claim_allowed': False" in text
    assert 'export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-87}"' in text
