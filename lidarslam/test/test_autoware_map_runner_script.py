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

"""Regression tests for the one-shot Autoware map runner script."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'run_autoware_map_from_bag.py'
BEGINNER_SCRIPT_PATH = REPO_ROOT / 'scripts' / 'run_autoware_map_beginner.sh'


def _load_module():
    spec = importlib.util.spec_from_file_location('run_autoware_map_from_bag', SCRIPT_PATH)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_metadata(tmp_path: Path, bag_name: str, topics: list[tuple[str, str, int]]) -> Path:
    bag_dir = tmp_path / bag_name
    bag_dir.mkdir()
    metadata = {
        'rosbag2_bagfile_information': {
            'duration': {'nanoseconds': 2_000_000_000},
            'message_count': sum(count for _, _, count in topics),
            'topics_with_message_count': [
                {
                    'topic_metadata': {
                        'name': name,
                        'type': msg_type,
                        'serialization_format': 'cdr',
                        'offered_qos_profiles': '',
                    },
                    'message_count': count,
                }
                for name, msg_type, count in topics
            ],
        },
    }
    (bag_dir / 'metadata.yaml').write_text(yaml.safe_dump(metadata), encoding='utf-8')
    return bag_dir


def test_runner_script_supports_profiles_and_viewers():
    script = SCRIPT_PATH.read_text(encoding='utf-8')

    assert 'preflight_autoware_map_bag.py' in script
    assert 'diagnose_autoware_map_run.py' in script
    assert 'rko_lio_graph_public_path' in script
    assert 'rko_lio_graph_mid360_preset' in script
    assert 'pointcloud_gnss_smoke' in script
    assert 'packet_applanix_smoke' in script
    assert '--viewer' in script
    assert 'verify_autoware_map.py' in script
    assert 'Next steps:' in script
    assert 'run_graph_slam_pointcloud_map_in_autoware_foxglove.sh' in script
    assert 'run_graph_slam_pointcloud_map_in_autoware.sh' in script
    assert '--dry-run' in script


def test_dogfood_script_can_skip_viewer():
    script = (REPO_ROOT / 'scripts' / 'run_rko_lio_graph_autoware_dogfood.sh').read_text(
        encoding='utf-8'
    )

    assert '--skip-viewer' in script
    assert 'if [[ "$SKIP_VIEWER" == "false" ]]; then' in script


def test_beginner_wrapper_exposes_simple_viewer_flags():
    script = BEGINNER_SCRIPT_PATH.read_text(encoding='utf-8')

    assert 'run_autoware_map_from_bag.py' in script
    assert '--foxglove' in script
    assert '--autoware' in script
    assert '--no-viewer' in script
    assert '--dry-run' in script


def test_runner_prefers_mid360_preset_for_livox_bag(tmp_path: Path):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        'mid360_demo_bag',
        [
            ('/livox/lidar', 'sensor_msgs/msg/PointCloud2', 20),
            ('/livox/imu', 'sensor_msgs/msg/Imu', 180),
        ],
    )

    plan = module.build_execution_plan(
        bag_path=bag_dir,
        profile_id=None,
        output_dir=tmp_path / 'out',
        verify_map=True,
    )

    assert plan['profile_id'] == 'rko_lio_graph_mid360_preset'
    assert 'lidarslam_mid360_rko_graph.yaml' in ' '.join(plan['command'])
