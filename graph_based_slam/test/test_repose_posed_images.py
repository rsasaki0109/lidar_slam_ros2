# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Tests for re-posing one fixed image set across repeated trajectories."""

import importlib.util
import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'repose_posed_images.py'
SPEC = importlib.util.spec_from_file_location('repose_posed_images', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_repose_preserves_images_stamps_and_interpolates_body_pose(tmp_path):
    images = tmp_path / 'source' / 'images'
    images.mkdir(parents=True)
    (images / 'frame.png').write_bytes(b'image')
    template = tmp_path / 'source' / 'transforms.json'
    template.write_text(json.dumps({
        'camera_model': 'OPENCV', 'w': 10, 'h': 8,
        'fl_x': 5, 'fl_y': 6, 'cx': 4, 'cy': 3,
        'k1': 0, 'k2': 0, 'p1': 0, 'p2': 0, 'k3': 0,
        'frames': [{'file_path': 'images/frame.png', 'stamp': 0.5,
                    'transform_matrix': np.eye(4).tolist()}],
    }))
    trajectory = tmp_path / 'trajectory.tum'
    trajectory.write_text(
        '0 0 0 0 0 0 0 1\n'
        '1 2 0 0 0 0 0 1\n')
    extrinsic = tmp_path / 'extrinsic.json'
    extrinsic.write_text(json.dumps({'matrix': np.eye(4).tolist()}))
    output = tmp_path / 'output' / 'transforms.json'

    result = MODULE.repose(template, trajectory, extrinsic, output)

    document = json.loads(output.read_text())
    assert result['frames'] == 1
    assert document['frames'][0]['stamp'] == 0.5
    assert Path(document['frames'][0]['file_path']).is_absolute()
    np.testing.assert_allclose(
        np.asarray(document['frames'][0]['transform_matrix'])[:3, 3],
        [1, 0, 0])
