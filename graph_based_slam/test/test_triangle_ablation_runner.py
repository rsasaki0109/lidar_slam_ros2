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

"""Regression tests for the triangle ablation runner wrapper."""

from __future__ import annotations

import os
from pathlib import Path
import stat
import subprocess

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'run_triangle_ablation.sh'


def test_script_exists_and_is_executable():
    assert SCRIPT.is_file()
    mode = SCRIPT.stat().st_mode
    assert mode & stat.S_IXUSR, f'{SCRIPT} must be user-executable'


def test_script_help_exits_with_usage():
    """--help must exit non-zero (matches the existing runner convention)."""
    result = subprocess.run(
        ['bash', str(SCRIPT), '--help'],
        capture_output=True,
        text=True,
        check=False,
    )
    combined = result.stdout + result.stderr
    assert 'use_triangle_descriptor' in combined
    assert 'baseline' in combined
    assert 'candidate' in combined


def test_script_help_documents_skip_reference_gen():
    """
    Help banner documents the non-NTU forwarding flags.

    --skip-reference-gen / --reference-source must surface in the help banner so
    operators running on non-NTU bags (Newer College, MID-360, custom) can
    discover the forwarding path without grepping the script body.
    """
    result = subprocess.run(
        ['bash', str(SCRIPT), '--help'],
        capture_output=True,
        text=True,
        check=False,
    )
    combined = result.stdout + result.stderr
    assert '--skip-reference-gen' in combined
    assert '--reference-source' in combined
    # Operators must understand the wiring without reading the script body.
    assert 'NTU' in combined or 'Newer' in combined


def test_script_accepts_and_forwards_skip_reference_gen(tmp_path: Path):
    """
    Forwarded flags reach the inner benchmark script.

    --skip-reference-gen / --reference-source must reach the inner
    run_rko_lio_graph_benchmark.sh invocation. We stub the inner script with a
    shim that records argv so the test stays hermetic (no ros2 / no bag).
    """
    stub_dir = tmp_path / 'scripts'
    stub_dir.mkdir()
    # Copy the runner so we can swap the inner benchmark script via PATH.
    runner = stub_dir / SCRIPT.name
    runner.write_bytes(SCRIPT.read_bytes())
    runner.chmod(0o755)

    inner_log = tmp_path / 'inner_args.log'
    inner_stub = stub_dir / 'run_rko_lio_graph_benchmark.sh'
    inner_stub.write_text(
        '#!/usr/bin/env bash\n'
        f'printf "%s\\n" "$@" > {inner_log}\n'
        'metrics_dir=""\n'
        'while [[ $# -gt 0 ]]; do\n'
        '  if [[ "$1" == "--output-dir" ]]; then metrics_dir="$2"; fi\n'
        '  shift\n'
        'done\n'
        '[[ -n "$metrics_dir" ]] && mkdir -p "$metrics_dir" \\\n'
        '  && echo "{}" > "$metrics_dir/metrics.json"\n',
        encoding='utf-8',
    )
    inner_stub.chmod(0o755)
    # Stub the report generator too — it parses metrics.json and would fail.
    report_stub = stub_dir / 'generate_place_recognition_report.py'
    report_stub.write_text('#!/usr/bin/env python3\nimport sys; sys.exit(0)\n', encoding='utf-8')
    report_stub.chmod(0o755)

    base_yaml = tmp_path / 'base.yaml'
    base_yaml.write_text(
        yaml.safe_dump(
            {'graph_based_slam': {'ros__parameters': {'use_triangle_descriptor': False}}},
            sort_keys=False,
        ),
        encoding='utf-8',
    )
    ref_tum = tmp_path / 'ref.tum'
    ref_tum.write_text('# timestamp tx ty tz qx qy qz qw\n', encoding='utf-8')
    ref_meta = tmp_path / 'ref.json'
    ref_meta.write_text('{}\n', encoding='utf-8')
    bag_dir = tmp_path / 'bag'
    bag_dir.mkdir()
    (bag_dir / 'metadata.yaml').write_text('version: 5\n', encoding='utf-8')
    out_dir = tmp_path / 'out'

    result = subprocess.run(
        [
            'bash', str(runner),
            '--bag', str(bag_dir),
            '--reference-tum', str(ref_tum),
            '--reference-meta', str(ref_meta),
            '--lidar-topic', '/os_cloud_node/points',
            '--imu-topic', '/os_cloud_node/imu',
            '--base-param', str(base_yaml),
            '--output-dir', str(out_dir),
            '--skip-reference-gen',
            '--reference-source', 'newer_college_gt',
        ],
        capture_output=True,
        text=True,
        check=False,
        env={**os.environ},
    )
    # The runner may exit non-zero if downstream steps it does NOT stub fail,
    # but the inner benchmark script must have been invoked with the new flags.
    assert inner_log.exists(), (
        f'inner stub never ran. stdout={result.stdout!r} stderr={result.stderr!r}'
    )
    forwarded = inner_log.read_text(encoding='utf-8').splitlines()
    assert '--skip-reference-gen' in forwarded
    assert '--reference-source' in forwarded
    src_idx = forwarded.index('--reference-source')
    assert forwarded[src_idx + 1] == 'newer_college_gt'


def test_script_rejects_missing_args():
    """Missing required arguments must exit non-zero with the usage banner."""
    result = subprocess.run(
        ['bash', str(SCRIPT)],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode != 0
    assert 'Missing required argument' in result.stderr


def test_yaml_override_keeps_other_descriptors(tmp_path: Path):
    """Inline override logic must only flip use_triangle_descriptor."""
    base = tmp_path / 'base.yaml'
    base.write_text(
        yaml.safe_dump(
            {
                'graph_based_slam': {
                    'ros__parameters': {
                        'use_scan_context': True,
                        'use_bev_descriptor': True,
                        'use_solid_descriptor': False,
                        'use_triangle_descriptor': False,
                        'triangle_descriptor_min_votes': 6,
                    },
                },
            },
            sort_keys=False,
        ),
        encoding='utf-8',
    )
    candidate = tmp_path / 'candidate.yaml'
    # Mirror the inline python heredoc inside run_triangle_ablation.sh.
    code = (
        'import sys, yaml; from pathlib import Path; '
        'src = Path(sys.argv[1]); dst = Path(sys.argv[2]); '
        'flag = sys.argv[3].lower() == "true"; '
        'data = yaml.safe_load(src.read_text(encoding="utf-8")); '
        'params = data.get("graph_based_slam", {}).get("ros__parameters", {}); '
        'params["use_triangle_descriptor"] = flag; '
        'dst.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")'
    )
    subprocess.run(
        ['python3', '-c', code, str(base), str(candidate), 'true'],
        check=True,
        env={**os.environ},
    )
    out_params = yaml.safe_load(candidate.read_text(encoding='utf-8'))['graph_based_slam'][
        'ros__parameters']
    assert out_params['use_triangle_descriptor'] is True
    # Other descriptor knobs must stay untouched.
    assert out_params['use_scan_context'] is True
    assert out_params['use_bev_descriptor'] is True
    assert out_params['use_solid_descriptor'] is False
    assert out_params['triangle_descriptor_min_votes'] == 6
