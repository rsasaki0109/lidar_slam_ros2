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

"""CLI contract tests for HILTI overlap-gate cross-validation."""

from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts/run_hilti_overlap_crossval.sh'


def _fixture(tmp_path: Path, sequence: str = 'exp01') -> tuple[Path, Path, Path]:
    dataset = tmp_path / 'datasets'
    bag = dataset / f'{sequence}_ros2'
    bag.mkdir(parents=True)
    (bag / 'metadata.yaml').write_text('version: 9\n', encoding='utf-8')
    slug = {
        'exp01': 'exp01_construction_ground_level',
        'exp07': 'exp07_long_corridor',
    }[sequence]
    (dataset / f'{slug}_gt.txt').write_text(
        '1.0 0 0 0 0 0 0 1\n2.0 0 1 0 0 0 0 1\n', encoding='utf-8')
    setup = tmp_path / 'setup.bash'
    setup.write_text('', encoding='utf-8')
    return dataset, tmp_path / 'evidence', setup


def _run(*args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ['bash', str(SCRIPT), *args], cwd=ROOT, capture_output=True,
        text=True, check=False)


def test_dry_run_prints_capture_and_matched_gate_commands(tmp_path: Path):
    dataset, output, setup = _fixture(tmp_path)
    result = _run(
        '--sequence', 'exp01', '--dataset-root', str(dataset),
        '--output-root', str(output), '--setup', str(setup),
        '--runs', '1', '--dry-run')

    assert result.returncode == 0, result.stderr
    assert 'record_backend_input.sh' in result.stdout
    assert 'run_rko_lio_graph_benchmark.sh' in result.stdout
    assert 'loop_min_overlap_ratio:=0.0' in result.stdout
    assert 'loop_min_overlap_ratio:=0.76' in result.stdout
    assert 'loop_min_overlap_ratio_large_correction:=0.70' in result.stdout
    assert '--ape-interpolate' in result.stdout
    assert 'capture_params.yaml' in result.stdout
    assert not output.exists()


def test_offline_only_requires_a_complete_frozen_backend(tmp_path: Path):
    dataset, output, setup = _fixture(tmp_path)
    result = _run(
        '--sequence', 'exp01', '--dataset-root', str(dataset),
        '--output-root', str(output), '--setup', str(setup),
        '--offline-only', '--dry-run')

    assert result.returncode == 2
    assert 'frozen backend input not found' in result.stderr


def test_record_and_offline_modes_are_mutually_exclusive(tmp_path: Path):
    dataset, output, setup = _fixture(tmp_path)
    result = _run(
        '--sequence', 'exp01', '--dataset-root', str(dataset),
        '--output-root', str(output), '--setup', str(setup),
        '--record-only', '--offline-only', '--dry-run')

    assert result.returncode == 2
    assert 'mutually exclusive' in result.stderr


def test_script_uses_external_ssd_default_and_refuses_incomplete_capture():
    source = SCRIPT.read_text(encoding='utf-8')

    assert '/media/sasaki/aiueo/benchmarks/' in source
    assert 'incomplete backend directory exists' in source
    assert 'source output exists without backend input' in source
    assert "graph['distance_loop_closure'] = 1.0e12" in source
    assert "graph['debug_flag'] = False" in source
    assert 'edges_removed_by_gate' in source
    assert 'comparison.json' in source
