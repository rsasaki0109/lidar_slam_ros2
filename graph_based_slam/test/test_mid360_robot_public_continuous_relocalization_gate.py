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

"""Tests for public MID-360 continuous RKO-LIO relocalization gate."""

from __future__ import annotations

import json
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'scripts'
sys.path.insert(0, str(SCRIPT_DIR))

from mid360_robot_public_continuous_relocalization_gate import (  # noqa: E402
    CONTINUOUS_RELOCALIZATION_GATE_JSON,
    CONTINUOUS_RELOCALIZATION_GATE_MARKDOWN,
    ContinuousRelocalizationGate,
    ContinuousRelocalizationGateOptions,
    DEFAULT_PUBLIC_LOOP_END_STAMP_SEC,
    DEFAULT_PUBLIC_LOOP_START_STAMP_SEC,
)


def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding='utf-8')


def _write_minimal_map(
    run_dir: Path,
    *,
    tum_poses: int = 1200,
    closed_public_endpoint: bool = True,
) -> None:
    pointcloud_map = run_dir / 'pointcloud_map'
    pointcloud_map.mkdir(parents=True, exist_ok=True)
    (run_dir / 'map_projector_info.yaml').write_text(
        'projector_type: local\n',
        encoding='utf-8',
    )
    (pointcloud_map / 'pointcloud_map_metadata.yaml').write_text(
        'x_resolution: 20\n'
        'y_resolution: 20\n'
        '0_0.pcd: [0, 0]\n',
        encoding='utf-8',
    )
    (pointcloud_map / '0_0.pcd').write_text(
        '# .PCD v0.7\n'
        'VERSION 0.7\n'
        'FIELDS x y z\n'
        'SIZE 4 4 4\n'
        'TYPE F F F\n'
        'COUNT 1 1 1\n'
        'WIDTH 1\n'
        'HEIGHT 1\n'
        'POINTS 1\n'
        'DATA ascii\n'
        '1.0 2.0 0.0\n',
        encoding='ascii',
    )
    tum = run_dir / 'continuous_tum_0.txt'
    duration = DEFAULT_PUBLIC_LOOP_END_STAMP_SEC - DEFAULT_PUBLIC_LOOP_START_STAMP_SEC
    denom = max(1, tum_poses - 1)
    lines = []
    for index in range(tum_poses):
        stamp = DEFAULT_PUBLIC_LOOP_START_STAMP_SEC + duration * index / denom
        x = float(index % 100)
        if index == 0:
            x = 0.0
        elif index == tum_poses - 1:
            x = 1.0 if closed_public_endpoint else 100.0
        lines.append(f'{stamp:.6f} {x:.3f} 0 0 0 0 0 1\n')
    tum.write_text(''.join(lines), encoding='ascii')


def _kidnap_config() -> dict:
    return {
        'max_scan_delta_sec': 10000.0,
        'enable_kidnap_relocalization': True,
        'reset_on_registration_failure': True,
        'recovery_min_failures': 1,
        'relocalize_after_scan_gap': False,
        'relocalization_min_correspondences': 20,
        'relocalization_min_inlier_ratio': 0.08,
        'relocalization_max_mean_error': 1.5,
        'relocalization_max_correspondance_distance': 2.5,
        'relocalization_yaw_samples': 36,
        'relocalization_pose_stride': 5,
        'relocalization_min_pose_separation': 30,
        'relocalization_max_iterations': 20,
    }


def _write_run_config(run_dir: Path, config: dict | None = None) -> None:
    config_dir = run_dir / 'continuous_0'
    config_dir.mkdir(parents=True, exist_ok=True)
    _write_json(config_dir / 'config.json', {'config': config or _kidnap_config()})


def _write_tracked_config(path: Path, config: dict | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = config or _kidnap_config()
    path.write_text(
        ''.join(f'{key}: {str(value).lower() if isinstance(value, bool) else value}\n'
                for key, value in payload.items()),
        encoding='utf-8',
    )


def _write_loop_alignment(path: Path) -> None:
    _write_json(
        path,
        {
            'status': 'PASS',
            'trajectory': {'poses': 1200, 'path_length_m': 700.0},
            'nearest_revisit': {'distance_m': 0.16},
            'loop_candidates': [
                {'distance_m': 0.16, 'start_index': 100, 'end_index': 900},
                {'distance_m': 0.20, 'start_index': 101, 'end_index': 901},
            ],
            'counts': {'fail': 0},
        },
    )


def _write_slam_log(run_dir: Path, *, relocalized: bool = True) -> None:
    lines = [
        '[graph_based_slam]: searching Loop, num_submaps:289\n',
        'RKO LIO Offline Node took 87.3 seconds.\n',
    ]
    if relocalized:
        lines.extend([
            'Kidnap relocalization matched 23/23 keypoints, mean error 0.167 m.\n',
            'Kidnap recovery accepted scan at 1693923000s via global relocalization.\n',
        ])
    (run_dir / 'slam.launch.log').write_text(''.join(lines), encoding='utf-8')


def test_continuous_relocalization_gate_passes_with_required_evidence(tmp_path: Path):
    run_dir = tmp_path / 'run'
    tracked = tmp_path / 'tracked.yaml'
    loop = run_dir / 'mid360_robot_loop_alignment.json'
    _write_minimal_map(run_dir)
    _write_run_config(run_dir)
    _write_tracked_config(tracked)
    _write_loop_alignment(loop)
    _write_slam_log(run_dir)

    report = ContinuousRelocalizationGate().build_report(
        ContinuousRelocalizationGateOptions(
            run_dir=run_dir,
            output_dir=tmp_path / 'gate',
            loop_alignment_json=loop,
            tracked_rko_config=tracked,
            min_rko_poses=1000,
            min_trajectory_duration_sec=500.0,
            min_relocalization_events=1,
        )
    )

    assert report['status'] == 'PASS'
    assert report['completion_ready'] is True
    assert report['counts']['fail'] == 0
    assert (tmp_path / 'gate' / CONTINUOUS_RELOCALIZATION_GATE_JSON).is_file()
    assert (tmp_path / 'gate' / CONTINUOUS_RELOCALIZATION_GATE_MARKDOWN).is_file()


def test_continuous_relocalization_gate_fails_without_relocalization_event(tmp_path: Path):
    run_dir = tmp_path / 'run'
    tracked = tmp_path / 'tracked.yaml'
    loop = run_dir / 'mid360_robot_loop_alignment.json'
    _write_minimal_map(run_dir)
    _write_run_config(run_dir)
    _write_tracked_config(tracked)
    _write_loop_alignment(loop)
    _write_slam_log(run_dir, relocalized=False)

    report = ContinuousRelocalizationGate().build_report(
        ContinuousRelocalizationGateOptions(
            run_dir=run_dir,
            output_dir=tmp_path / 'gate',
            loop_alignment_json=loop,
            tracked_rko_config=tracked,
        )
    )

    assert report['status'] == 'FAIL'
    assert any(
        check['id'] == 'kidnap_relocalization_event_present'
        and check['status'] == 'FAIL'
        for check in report['checks']
    )


def test_continuous_relocalization_gate_fails_when_public_endpoint_not_closed(tmp_path: Path):
    run_dir = tmp_path / 'run'
    tracked = tmp_path / 'tracked.yaml'
    loop = run_dir / 'mid360_robot_loop_alignment.json'
    _write_minimal_map(run_dir, closed_public_endpoint=False)
    _write_run_config(run_dir)
    _write_tracked_config(tracked)
    _write_loop_alignment(loop)
    _write_slam_log(run_dir)

    report = ContinuousRelocalizationGate().build_report(
        ContinuousRelocalizationGateOptions(
            run_dir=run_dir,
            output_dir=tmp_path / 'gate',
            loop_alignment_json=loop,
            tracked_rko_config=tracked,
        )
    )

    assert report['status'] == 'FAIL'
    assert any(
        check['id'] == 'public_loop_endpoint_relocalized'
        and check['status'] == 'FAIL'
        for check in report['checks']
    )


def test_continuous_relocalization_gate_fails_when_run_config_differs(tmp_path: Path):
    run_dir = tmp_path / 'run'
    tracked = tmp_path / 'tracked.yaml'
    loop = run_dir / 'mid360_robot_loop_alignment.json'
    run_config = _kidnap_config()
    run_config['relocalization_yaw_samples'] = 12
    _write_minimal_map(run_dir)
    _write_run_config(run_dir, run_config)
    _write_tracked_config(tracked)
    _write_loop_alignment(loop)
    _write_slam_log(run_dir)

    report = ContinuousRelocalizationGate().build_report(
        ContinuousRelocalizationGateOptions(
            run_dir=run_dir,
            output_dir=tmp_path / 'gate',
            loop_alignment_json=loop,
            tracked_rko_config=tracked,
        )
    )

    assert report['status'] == 'FAIL'
    assert any(
        check['id'] == 'tracked_kidnap_config_matches_run'
        and check['status'] == 'FAIL'
        for check in report['checks']
    )
