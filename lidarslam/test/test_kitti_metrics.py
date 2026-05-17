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

"""Regression tests for the KITTI drift metric helper."""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'kitti_metrics.py'


def _load_module():
    spec = importlib.util.spec_from_file_location('kitti_metrics', SCRIPT_PATH)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _identity_quat() -> np.ndarray:
    return np.array([0.0, 0.0, 0.0, 1.0])


def _straight_line_tum(length_m: float = 1000.0, step_m: float = 1.0) -> np.ndarray:
    """Build a TUM array along +x at 1 m spacing with identity rotations."""
    n = int(length_m / step_m) + 1
    ts = np.arange(n) * 0.1
    xs = np.arange(n) * step_m
    rows = []
    for i in range(n):
        rows.append([ts[i], xs[i], 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    return np.array(rows)


def _curved_path_tum(radius_m: float = 200.0, total_arc_m: float = 1000.0,
                     step_m: float = 1.0) -> np.ndarray:
    """Build an arc on a circle (xy plane); yaw stays aligned with +x tangent."""
    n = int(total_arc_m / step_m) + 1
    ts = np.arange(n) * 0.1
    rows = []
    for i in range(n):
        s = i * step_m
        theta = s / radius_m
        x = radius_m * math.sin(theta)
        y = radius_m * (1.0 - math.cos(theta))
        # quaternion for yaw rotation by theta about +z
        qz = math.sin(theta * 0.5)
        qw = math.cos(theta * 0.5)
        rows.append([ts[i], x, y, 0.0, 0.0, 0.0, qz, qw])
    return np.array(rows)


def test_perfect_estimate_yields_zero_drift():
    module = _load_module()
    tum = _straight_line_tum(length_m=1000.0)
    poses = module.tum_to_poses(tum)
    metrics = module.compute_kitti_errors(poses, poses.copy())
    assert metrics['t_rel_percent_avg'] == pytest.approx(0.0, abs=1e-9)
    assert metrics['r_rel_deg_per_m_avg'] == pytest.approx(0.0, abs=1e-9)
    assert metrics['pairs_total'] > 0


def test_global_se3_transform_does_not_change_drift():
    """KITTI drift is invariant to a global SE(3) shift of the estimate."""
    module = _load_module()
    tum = _curved_path_tum()
    gt = module.tum_to_poses(tum)
    rot = np.array(
        [
            [math.cos(0.3), -math.sin(0.3), 0.0, 5.0],
            [math.sin(0.3), math.cos(0.3), 0.0, -2.0],
            [0.0, 0.0, 1.0, 1.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    est = np.einsum('ij,njk->nik', rot, gt)
    metrics = module.compute_kitti_errors(gt, est)
    assert metrics['t_rel_percent_avg'] == pytest.approx(0.0, abs=1e-7)
    assert metrics['r_rel_deg_per_m_avg'] == pytest.approx(0.0, abs=1e-7)


def test_constant_translational_drift_rate_is_reported_correctly():
    """A 1% along-track over-shoot must show up as ~1% t_rel."""
    module = _load_module()
    gt_tum = _straight_line_tum(length_m=1000.0, step_m=1.0)
    est_tum = gt_tum.copy()
    est_tum[:, 1] = est_tum[:, 1] * 1.01  # stretch x by +1%
    gt_poses = module.tum_to_poses(gt_tum)
    est_poses = module.tum_to_poses(est_tum)
    metrics = module.compute_kitti_errors(gt_poses, est_poses)
    assert metrics['t_rel_percent_avg'] == pytest.approx(1.0, rel=1e-3)


def test_per_length_breakdown_includes_all_requested_lengths():
    module = _load_module()
    tum = _curved_path_tum(radius_m=200.0, total_arc_m=1000.0)
    gt = module.tum_to_poses(tum)
    metrics = module.compute_kitti_errors(
        gt, gt.copy(), lengths_m=(100, 200, 400, 800)
    )
    assert set(metrics['per_length'].keys()) == {100, 200, 400, 800}
    for L, stats in metrics['per_length'].items():
        assert stats['pairs'] > 0, f'no pairs for L={L}'


def test_returns_none_when_no_window_fits():
    """If the trajectory is shorter than the smallest window, no pairs."""
    module = _load_module()
    tum = _straight_line_tum(length_m=50.0, step_m=1.0)
    poses = module.tum_to_poses(tum)
    metrics = module.compute_kitti_errors(poses, poses, lengths_m=(100, 200))
    assert metrics['t_rel_percent_avg'] is None
    assert metrics['r_rel_deg_per_m_avg'] is None
    assert metrics['pairs_total'] == 0


def test_pose_shape_mismatch_raises():
    module = _load_module()
    a = np.tile(np.eye(4), (10, 1, 1))
    b = np.tile(np.eye(4), (5, 1, 1))
    with pytest.raises(ValueError, match='shape mismatch'):
        module.compute_kitti_errors(a, b)


def test_quat_to_R_normalises_input():
    """Non-unit quaternions should still produce a proper rotation matrix."""
    module = _load_module()
    R = module._quat_to_R(np.array([0.0, 0.0, 2.0, 2.0]))  # not unit norm
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-9)
    assert np.isclose(np.linalg.det(R), 1.0, atol=1e-9)


def test_cli_writes_json(tmp_path: Path, capsys: pytest.CaptureFixture[str]):
    """End-to-end smoke: TUM in → JSON metrics out via the CLI entrypoint."""
    module = _load_module()
    gt = _straight_line_tum(length_m=1000.0)
    est = gt.copy()
    est[:, 1] = est[:, 1] * 1.02  # +2% along-track drift
    gt_path = tmp_path / 'gt.tum'
    est_path = tmp_path / 'est.tum'
    out_json = tmp_path / 'out.json'
    np.savetxt(gt_path, gt, fmt='%.6f')
    np.savetxt(est_path, est, fmt='%.6f')

    rc = module.main(
        [
            '--gt', str(gt_path),
            '--est', str(est_path),
            '--out-json', str(out_json),
            '--label', 'unit_test',
        ]
    )
    assert rc == 0
    import json
    data = json.loads(out_json.read_text())
    assert data['label'] == 'unit_test'
    assert data['t_rel_percent_avg'] == pytest.approx(2.0, rel=1e-3)
    assert data['frames'] == len(gt)
