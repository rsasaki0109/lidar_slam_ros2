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
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
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

"""Cgroup-v2 event and process-RSS evidence contract tests."""

import importlib.util
import json
from pathlib import Path
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'container_memory_evidence', ROOT / 'scripts' / 'container_memory_evidence.py')
EVIDENCE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(EVIDENCE)


def _write_cgroup(root: Path, *, oom: int = 0):
    (root / 'cgroup.controllers').write_text('memory\n')
    (root / 'memory.peak').write_text('200')
    (root / 'memory.current').write_text('100')
    (root / 'memory.max').write_text('max')
    (root / 'memory.events').write_text(
        f'oom {oom}\n' f'oom_kill {oom}\n' 'high 0\n')
    (root / 'memory.events.local').write_text(
        f'oom {oom}\n' f'oom_kill {oom}\n' 'high 0\n')
    pressure = 'some avg10=0.00 avg60=0.00 avg300=0.00 total=1\n'
    pressure += 'full avg10=0.00 avg60=0.00 avg300=0.00 total=0\n'
    (root / 'memory.pressure').write_text(pressure)


def _sampler():
    return {
        'schema_version': 1,
        'measurement_version': 'm6a7-container-process-rss-v1',
        'measurement_scope': 'container_pid_namespace_proc_status',
        'primary_metric': 'aggregate_process_tree_peak_rss_bytes',
        'primary_metric_definition':
            'sum_of_per_process_vmrss_peaks_shared_pages_may_be_recounted',
        'status': 'pass', 'atomic': True, 'sampler_excluded': True,
        'sampler_pid': 321, 'first_sample_monotonic_ns': 1,
        'last_sample_monotonic_ns': 2,
        'sample_count': 2, 'sample_errors': 0, 'pid_race_skips': 0,
        'missed_intervals': 0, 'interval_jitter_percent': 0.0,
        'thresholds': {'min_samples': 2, 'max_errors': 0,
                       'max_race_skips': 0, 'max_jitter_percent': 10.0},
        'peak': {'vmrss_bytes': 123, 'rss_anon_bytes': 100,
                 'rss_file_bytes': 20, 'rss_shmem_bytes': 3,
                 'process_count': 1},
        'aggregate_process_tree_peak_rss_bytes': 123,
    }


def _write_shell_fixture_scripts(tmp_path):
    sampler = tmp_path / 'fake_sampler.py'
    sampler.write_text(
        """#!/usr/bin/env python3
import argparse
import json
from pathlib import Path
import signal
import time

stop = False

def handle(_signum, _frame):
    global stop
    stop = True

signal.signal(signal.SIGTERM, handle)
signal.signal(signal.SIGINT, handle)
parser = argparse.ArgumentParser()
parser.add_argument('--output', type=Path, required=True)
args, _ = parser.parse_known_args()
while not stop:
    time.sleep(0.01)
args.output.parent.mkdir(parents=True, exist_ok=True)
part = args.output.with_name(args.output.name + '.part')
part.write_text(json.dumps({'status': 'pass'}) + '\\n')
part.replace(args.output)
""", encoding='utf-8')
    sampler.chmod(0o755)
    memory = tmp_path / 'fake_memory.py'
    memory.write_text(
        """#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('mode')
parser.add_argument('--output', type=Path)
parser.add_argument('--part', type=Path)
parser.add_argument('--process-status', default='0')
parser.add_argument('--sampler-stop-status', default='0')
args, _ = parser.parse_known_args()
if args.mode == 'baseline':
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text('{}\\n')
else:
    value = {
        'status': 'pass' if args.sampler_stop_status == '0' else 'invalid',
        'atomic': True,
        'process_exit_status': int(args.process_status),
        'sampler_stop_status': int(args.sampler_stop_status),
    }
    args.part.write_text(json.dumps(value) + '\\n')
    args.part.replace(args.output)
""", encoding='utf-8')
    memory.chmod(0o755)
    return sampler, memory


def _run_shell_trap_fixture(tmp_path, action):
    sampler, memory = _write_shell_fixture_scripts(tmp_path)
    output = tmp_path / 'out'
    output.mkdir()
    helper = ROOT / 'scripts' / 'container_memory_evidence.sh'
    command = f"""set -uo pipefail
export OUT_DIR={output}
export M6A7_SAMPLER_SCRIPT={sampler}
export M6A7_MEMORY_SCRIPT={memory}
export M6A7_SAMPLER_STOP_TIMEOUT_SECS=2
source {helper}
trap 'm6a5_container_exit_trap "$?"' EXIT
m6a5_install_container_signal_traps
m6a7_start_process_rss_sampler
sleep 0.1
{action}
"""
    result = subprocess.run(
        ['bash', '-c', command], capture_output=True, text=True, timeout=5)
    evidence = json.loads((output / 'container_memory.json').read_text())
    return result, output, evidence


@pytest.mark.parametrize(
    ('action', 'expected_status'),
    [('exit 17', 17), ('kill -TERM $$', 143), ('kill -INT $$', 130)])
def test_exit_and_signal_traps_finalize_sampler_without_manual_stop(
        tmp_path, action, expected_status):
    result, output, evidence = _run_shell_trap_fixture(tmp_path, action)
    assert result.returncode == expected_status
    assert evidence['status'] == 'pass'
    assert evidence['process_exit_status'] == expected_status
    assert evidence['sampler_stop_status'] == 0
    assert (output / 'container_process_rss.json').is_file()
    assert not (output / 'container_process_rss.json.part').exists()
    assert not (output / 'container_memory.json.part').exists()


def test_memory_finalization_is_idempotent_after_explicit_and_exit_traps(
        tmp_path):
    sampler, memory = _write_shell_fixture_scripts(tmp_path)
    output = tmp_path / 'out'
    output.mkdir()
    helper = ROOT / 'scripts' / 'container_memory_evidence.sh'
    command = f"""set -uo pipefail
export OUT_DIR={output}
export M6A7_SAMPLER_SCRIPT={sampler}
export M6A7_MEMORY_SCRIPT={memory}
source {helper}
trap 'm6a5_container_exit_trap "$?"' EXIT
m6a5_install_container_signal_traps
m6a7_start_process_rss_sampler
sleep 0.1
m6a5_write_container_memory_evidence 0
first=$?
m6a5_write_container_memory_evidence 0
second=$?
printf '%s %s\\n' "$first" "$second"
"""
    result = subprocess.run(
        ['bash', '-c', command], capture_output=True, text=True, timeout=5)
    assert result.returncode == 0
    assert result.stdout.strip() == '0 0'
    assert (output / 'container_memory.json').is_file()


def test_missing_sampler_is_recorded_invalid_by_exit_finalization(tmp_path):
    _, memory = _write_shell_fixture_scripts(tmp_path)
    output = tmp_path / 'out'
    output.mkdir()
    helper = ROOT / 'scripts' / 'container_memory_evidence.sh'
    command = f"""set -uo pipefail
export OUT_DIR={output}
export M6A7_SAMPLER_SCRIPT={tmp_path / 'does-not-exist.py'}
export M6A7_MEMORY_SCRIPT={memory}
source {helper}
trap 'm6a5_container_exit_trap "$?"' EXIT
m6a5_install_container_signal_traps
m6a7_start_process_rss_sampler || true
exit 0
"""
    result = subprocess.run(
        ['bash', '-c', command], capture_output=True, text=True, timeout=5)
    assert result.returncode == 0
    evidence = json.loads((output / 'container_memory.json').read_text())
    assert evidence['status'] == 'invalid'
    assert evidence['sampler_stop_status'] != 0


def test_final_evidence_accepts_unlimited_cgroup_and_records_pressure_delta(tmp_path):
    cgroup = tmp_path / 'cgroup'
    cgroup.mkdir()
    _write_cgroup(cgroup)
    baseline = tmp_path / 'baseline.json'
    EVIDENCE.capture_baseline(baseline, cgroup)
    (cgroup / 'memory.peak').write_text('300')
    (cgroup / 'memory.current').write_text('150')
    (cgroup / 'memory.events').write_text('oom 0\noom_kill 0\nhigh 1\n')
    (cgroup / 'memory.events.local').write_text('oom 0\noom_kill 0\nhigh 1\n')
    sampler = tmp_path / 'container_process_rss.json'
    sampler.write_text(json.dumps(_sampler()))
    output = tmp_path / 'container_memory.json'
    part = tmp_path / 'container_memory.json.part'
    value = EVIDENCE.write_final(
        part, output, root=cgroup, baseline_path=baseline,
        sampler_path=sampler, timestamp='now', process_status='0',
        sampler_stop_status='0', cgroup_path='/fixture',
        proc_self_cgroup='0::/fixture', readability_status='pass',
        readability_reason='output_tree_chmod_a+rX')
    assert output.is_file() and not part.exists()
    assert value['status'] == 'pass'
    assert value['memory_max_unlimited'] is True
    assert value['primary_metric'] == 'aggregate_process_tree_peak_rss_bytes'
    assert value['primary_metric_definition'] == (
        'sum_of_per_process_vmrss_peaks_shared_pages_may_be_recounted')
    assert value['process_tree_peak_rss_bytes'] == 123
    assert value['cgroup_events']['memory_events']['delta']['high'] == 1


def test_final_evidence_rejects_oom_delta_and_malformed_pressure(tmp_path):
    cgroup = tmp_path / 'cgroup'
    cgroup.mkdir()
    _write_cgroup(cgroup)
    baseline = tmp_path / 'baseline.json'
    EVIDENCE.capture_baseline(baseline, cgroup)
    (cgroup / 'memory.events').write_text('oom 1\noom_kill 1\nhigh 0\n')
    (cgroup / 'memory.events.local').write_text('oom 1\noom_kill 1\nhigh 0\n')
    (cgroup / 'memory.pressure').write_text('malformed\n')
    sampler = tmp_path / 'container_process_rss.json'
    sampler.write_text(json.dumps(_sampler()))
    output = tmp_path / 'container_memory.json'
    value = EVIDENCE.write_final(
        output.with_suffix('.json.part'), output, root=cgroup,
        baseline_path=baseline, sampler_path=sampler, timestamp='now',
        process_status='0', sampler_stop_status='0', cgroup_path='/fixture',
        proc_self_cgroup='0::/fixture', readability_status='pass',
        readability_reason='output_tree_chmod_a+rX')
    assert value['status'] == 'invalid'
    assert value['cgroup_events']['oom_free'] is False
