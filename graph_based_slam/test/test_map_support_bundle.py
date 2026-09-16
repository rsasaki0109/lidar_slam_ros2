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

"""Tests for privacy-safe map support bundles."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import tarfile


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'create_map_support_bundle.py'


def _load_module():
    spec = importlib.util.spec_from_file_location('create_map_support_bundle', SCRIPT)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_bundle_redacts_paths_and_omits_large_artifacts(tmp_path: Path):
    module = _load_module()
    run_dir = tmp_path / 'private-user' / 'run'
    run_dir.mkdir(parents=True)
    private_path = f'{run_dir}/dataset/secret_bag'
    (run_dir / 'autoware_map_diagnosis.md').write_text(
        f'run: {run_dir}\nbag: {private_path}\n', encoding='utf-8'
    )
    (run_dir / 'map.pcd').write_bytes(b'point cloud must not be shared')
    (run_dir / 'rosbag.db3').write_bytes(b'bag must not be shared')
    output = tmp_path / 'support.tar.gz'

    module.create_bundle(run_dir, output)

    with tarfile.open(output, 'r:gz') as archive:
        names = archive.getnames()
        diagnosis = archive.extractfile('support/autoware_map_diagnosis.md').read().decode()
        manifest = json.loads(
            archive.extractfile('support/support_bundle_manifest.json').read()
        )
    assert 'support/map.pcd' not in names
    assert 'support/rosbag.db3' not in names
    assert str(run_dir) not in diagnosis
    assert '${RUN_DIR}' in diagnosis
    assert manifest['privacy']['bags_maps_and_pointclouds_included'] is False


def test_bundle_rejects_empty_run(tmp_path: Path):
    module = _load_module()
    run_dir = tmp_path / 'empty'
    run_dir.mkdir()
    try:
        module.create_bundle(run_dir, tmp_path / 'support.tar.gz')
    except ValueError as exc:
        assert 'no supported diagnostics' in str(exc)
    else:
        raise AssertionError('empty run should be rejected')
