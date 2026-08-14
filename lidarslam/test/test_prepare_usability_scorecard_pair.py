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

"""Tests for paired fail-closed usability scorecard worksheet preparation."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'prepare_usability_scorecard_pair.py'
SPEC = importlib.util.spec_from_file_location(
    'prepare_usability_scorecard_pair_test',
    SCRIPT,
)
assert SPEC is not None and SPEC.loader is not None
PREPARE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PREPARE)


def _args(output_dir: Path, **overrides: str | None) -> list[str]:
    values = {
        '--lidarslam-version': '0.9.1',
        '--lidarslam-revision-kind': 'git-commit',
        '--lidarslam-revision': 'a' * 40,
        '--lidarslam-documentation-url': 'https://example.test/lidarslam',
        '--lidarslam-trial-id': 'lidarslam-pair-trial-a',
        '--glim-version': '1.0.0',
        '--glim-revision-kind': 'release-tag',
        '--glim-revision': 'v1.0.0',
        '--glim-documentation-url': 'https://example.test/glim',
        '--glim-trial-id': 'glim-pair-trial-a',
        '--cohort-id': 'external-paired-operator-a',
        '--comparison-pair-id': 'paired-jazzy-machine-class-a',
        '--input-id': 'fixed-demo-v1',
        '--ros-distro': 'jazzy',
        '--os-family': 'ubuntu-24.04',
        '--architecture': 'x86_64',
        '--hardware-class': 'eight-core-32gib-x86_64',
        '--machine-fingerprint-sha256': 'b' * 64,
        '--output-dir': str(output_dir),
    }
    values.update(overrides)
    result = []
    for key, value in values.items():
        if value is None:
            continue
        result.extend((key, value))
    return result


def _records(output_dir: Path) -> dict[str, dict]:
    return {
        path.stem: json.loads(path.read_text(encoding='utf-8'))
        for path in output_dir.glob('*.json')
    }


def test_pair_has_shared_metadata_and_opposite_order(tmp_path, capsys):
    """One command prepares two independently valid incomplete worksheets."""
    assert PREPARE.main(_args(tmp_path)) == 0
    manifest = json.loads(capsys.readouterr().out)
    records = _records(tmp_path)

    assert manifest['status'] == 'PREPARED_INCOMPLETE'
    assert manifest['remote_mutations_performed'] is False
    assert set(records) == {'lidarslam-pair-trial-a', 'glim-pair-trial-a'}
    lidarslam = records['lidarslam-pair-trial-a']
    glim = records['glim-pair-trial-a']
    assert lidarslam['product']['id'] == 'lidarslam_ros2'
    assert glim['product']['id'] == 'glim'
    assert lidarslam['operator']['product_order'] == 'first'
    assert glim['operator']['product_order'] == 'second'
    assert lidarslam['environment']['comparison_pair_id'] == (
        glim['environment']['comparison_pair_id']
    )
    assert lidarslam['environment']['machine_fingerprint_sha256'] == (
        glim['environment']['machine_fingerprint_sha256']
    )
    assert all(
        task['outcome']['status'] == 'FAIL'
        for record in (lidarslam, glim)
        for task in record['tasks']
    )


def test_per_product_fingerprints_are_supported(tmp_path, capsys):
    """Different hosts work without repeating the pair metadata."""
    args = _args(
        tmp_path,
        **{
            '--machine-fingerprint-sha256': None,
            '--lidarslam-machine-fingerprint-sha256': 'c' * 64,
            '--glim-machine-fingerprint-sha256': 'd' * 64,
        },
    )
    args = [value for value in args if value != 'None']
    assert PREPARE.main(args) == 0
    capsys.readouterr()
    records = _records(tmp_path)
    assert records['lidarslam-pair-trial-a']['environment'][
        'machine_fingerprint_sha256'] == 'c' * 64
    assert records['glim-pair-trial-a']['environment'][
        'machine_fingerprint_sha256'] == 'd' * 64


def test_existing_destination_refuses_pair_without_overwriting(tmp_path,
                                                               capsys):
    """An existing destination blocks both writes."""
    existing = tmp_path / 'glim-pair-trial-a.json'
    existing.write_text('keep\n', encoding='utf-8')
    assert PREPARE.main(_args(tmp_path)) == 2
    assert not (tmp_path / 'lidarslam-pair-trial-a.json').exists()
    assert existing.read_text(encoding='utf-8') == 'keep\n'
    assert 'refusing to overwrite' in capsys.readouterr().err


def test_missing_fingerprint_fails_closed(tmp_path, capsys):
    """A pair cannot be prepared without an identity for each host."""
    args = _args(tmp_path, **{'--machine-fingerprint-sha256': None})
    args = [value for value in args if value != 'None']
    assert PREPARE.main(args) == 2
    assert 'fingerprint' in capsys.readouterr().err
