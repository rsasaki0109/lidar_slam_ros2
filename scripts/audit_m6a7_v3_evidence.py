#!/usr/bin/env python3
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

"""Fail-closed, read-only audit for the M6a7 v3 sampler evidence."""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
from typing import Any


EXPECTED_PREREG_SHA256 = (
    '84b104ab7a71d2c128dd5f9b2c5f1047942e225d7ccfddf507d768384014c63f')
EXPECTED_IMAGE = (
    'm6a3c-lidarslam-ours@sha256:'
    '18198c17627459e96c574b1bf3093064c9c092f4fc2f89594b7e4b14705288bd')
EXPECTED_CPUSET = '0-7'
EXPECTED_CHECKSUM = 1744134064215079676
EXPECTED_INTERVAL = 0.25
EXPECTED_SCHEDULER_NICE = 10
REQUIRED_RUN_FILES = (
    'mode.txt', 'docker_exit_status.txt', 'docker_client_time.txt',
    'checksum.txt', 'workload_time.json', 'host_started_utc.txt',
    'host_finished_utc.txt')


class AuditError(ValueError):
    """Raised for malformed or incomplete audit input."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def output_tree_hash(path: Path) -> str:
    """Hash files using the output-tree-v1 path plus file-digest contract."""
    if not path.is_dir():
        raise AuditError(f'evidence tree is missing: {path}')
    files = sorted(item for item in path.rglob('*') if item.is_file())
    if not files:
        raise AuditError(f'evidence tree is empty: {path}')
    digest = hashlib.sha256()
    for item in files:
        digest.update(item.relative_to(path).as_posix().encode('utf-8'))
        digest.update(b'\0')
        digest.update(hashlib.sha256(item.read_bytes()).digest())
    return digest.hexdigest()


def parse_host_exit(report: str) -> int:
    """Parse one GNU ``time`` Exit status, tolerating leading whitespace."""
    matches = []
    for line in report.splitlines():
        stripped = line.strip()
        if stripped.startswith('Exit status:'):
            prefix, value = stripped.split(':', 1)
            if prefix != 'Exit status' or not re.fullmatch(r'-?[0-9]+', value.strip()):
                raise AuditError('malformed GNU time Exit status')
            matches.append(int(value.strip()))
    if len(matches) != 1:
        raise AuditError('GNU time Exit status is missing or duplicated')
    return matches[0]


def parse_mode(run_name: str, mode_text: str) -> str:
    """Validate the exact run mode; do not infer ``off`` from a suffix."""
    expected = run_name.rsplit('_', 1)[-1]
    if expected not in {'off', 'on'}:
        raise AuditError(f'invalid run name mode: {run_name}')
    mode = mode_text.strip()
    if mode not in {'off', 'on'} or mode != expected:
        raise AuditError(f'mode mismatch for {run_name}')
    return mode


def expected_schedule(pair_count: int = 20) -> list[str]:
    if pair_count < 1:
        raise AuditError('pair count must be positive')
    return [
        f'pair{pair:02d}_{mode}'
        for pair in range(1, pair_count + 1)
        for mode in (('off', 'on') if pair % 2 else ('on', 'off'))]


def validate_schedule_names(run_root: Path, pair_count: int = 20) -> list[str]:
    """Reject missing, duplicate, or extra attempt directories."""
    expected = expected_schedule(pair_count)
    actual = sorted(item.name for item in run_root.iterdir() if item.is_dir())
    if actual != sorted(expected):
        raise AuditError(
            f'run directory set mismatch: expected {expected!r}, got {actual!r}')
    if len(actual) != len(set(actual)):
        raise AuditError('duplicate run directory name')
    return expected


def _parse_timestamp(path: Path) -> tuple[str, int]:
    try:
        text = path.read_text(encoding='utf-8').strip()
        value = dt.datetime.fromisoformat(text)
    except (OSError, ValueError) as error:
        raise AuditError(f'invalid timestamp: {path}') from error
    if value.tzinfo is None:
        raise AuditError(f'timestamp lacks timezone: {path}')
    return text, int(value.timestamp() * 1_000_000_000)


def _read_integer(path: Path) -> int:
    try:
        return int(path.read_text(encoding='utf-8').strip())
    except (OSError, ValueError) as error:
        raise AuditError(f'invalid integer file: {path}') from error


def _command_fields(path: Path) -> dict[str, Any]:
    try:
        text = path.read_text(encoding='utf-8')
    except OSError as error:
        raise AuditError(f'cannot read host time report: {path}') from error
    lines = [line for line in text.splitlines()
             if 'Command being timed:' in line]
    if len(lines) != 1:
        raise AuditError(f'GNU time command line missing or duplicated: {path}')
    line = lines[0]
    images = re.findall(r' ([^ ]+@sha256:[0-9a-f]{64}) -c ', line)
    cpusets = re.findall(r'--cpuset-cpus ([^ ]+)', line)
    forbidden = re.findall(r'(?i)(ground[_-]?truth|scorer|ape)', line)
    return {
        'command_sha256': hashlib.sha256(line.encode('utf-8')).hexdigest(),
        'image': images[0] if len(images) == 1 else None,
        'cpuset': cpusets[0] if len(cpusets) == 1 else None,
        'network_none': '--network none' in line,
        'external_network': '--network host' in line or '--net=host' in line,
        'memory_cap': bool(re.search(r'--memory(?:=| )', line)),
        'forbidden_gt_or_scorer_tokens': forbidden,
    }


def audit_runs(run_root: Path) -> dict[str, Any]:
    schedule = validate_schedule_names(run_root)
    rows: list[dict[str, Any]] = []
    for index, name in enumerate(schedule, start=1):
        path = run_root / name
        missing = [
            filename for filename in REQUIRED_RUN_FILES
            if not (path / filename).is_file()]
        if missing:
            raise AuditError(f'{name} missing files: {missing!r}')
        mode = parse_mode(name, (path / 'mode.txt').read_text(encoding='utf-8'))
        docker_exit = _read_integer(path / 'docker_exit_status.txt')
        host_exit = parse_host_exit((path / 'docker_client_time.txt').read_text(
            encoding='utf-8'))
        checksum = _read_integer(path / 'checksum.txt')
        workload = json.loads((path / 'workload_time.json').read_text(
            encoding='utf-8'))
        started, started_ns = _parse_timestamp(path / 'host_started_utc.txt')
        finished, finished_ns = _parse_timestamp(path / 'host_finished_utc.txt')
        command = _command_fields(path / 'docker_client_time.txt')
        parts = [item.relative_to(path).as_posix()
                 for item in path.rglob('*') if item.name.endswith('.part')]
        if parts:
            raise AuditError(f'{name} contains .part files: {parts!r}')
        if (docker_exit != 0 or host_exit != 0 or checksum != EXPECTED_CHECKSUM or
                workload.get('checksum') != EXPECTED_CHECKSUM or
                workload.get('worker_count') != 8 or
                workload.get('work_items_per_worker') != 8_000_000 or
                finished_ns < started_ns or command['image'] != EXPECTED_IMAGE or
                command['cpuset'] != EXPECTED_CPUSET or
                not command['network_none'] or command['external_network'] or
                command['memory_cap'] or command['forbidden_gt_or_scorer_tokens']):
            raise AuditError(f'{name} run contract mismatch')
        rows.append({
            'schedule_index': index, 'directory': name, 'mode': mode,
            'docker_exit_status': docker_exit,
            'host_time_exit_status': host_exit, 'checksum': checksum,
            'workload_checksum': workload.get('checksum'),
            'worker_count': workload.get('worker_count'),
            'work_items_per_worker': workload.get('work_items_per_worker'),
            'started_utc': started, 'finished_utc': finished,
            'command': command})
    observed = [row['directory'] for row in sorted(
        rows, key=lambda row: row['started_utc'])]
    if observed != schedule:
        raise AuditError('host start order differs from AB/BA schedule')
    if len({row['started_utc'] for row in rows}) != len(rows):
        raise AuditError('duplicate host start timestamps')
    return {
        'expected_order': schedule, 'observed_order': observed,
        'pairs': 20, 'runs': len(rows), 'rows': rows,
        'part_files': [], 'all_complete': len(rows) == 40,
    }


def preregistration_binding(run_root: Path, source: Path) -> dict[str, Any]:
    """Bind source and run-root metadata without changing the raw run root."""
    metadata = run_root / 'preregistration.json'
    if not source.is_file() or not metadata.is_file():
        raise AuditError('preregistration source or run metadata is missing')
    source_sha = sha256_file(source)
    metadata_sha = sha256_file(metadata)
    if source_sha != EXPECTED_PREREG_SHA256 or metadata_sha != source_sha:
        raise AuditError('preregistration SHA binding mismatch')
    try:
        value = json.loads(source.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as error:
        raise AuditError('preregistration JSON is invalid') from error
    if value.get('schema_version') != 3 or value.get('status') != \
            'preregistered_not_measured':
        raise AuditError('preregistration state is not frozen-before-run')
    marker = run_root / 'COMPLETE.marker'
    marker_sidecar = run_root / 'COMPLETE.marker.sha256'
    if not marker.is_file() or not marker_sidecar.is_file():
        raise AuditError('completion marker or sidecar is missing')
    if marker_sidecar.read_text(encoding='utf-8').split()[0] != sha256_file(marker):
        raise AuditError('completion marker sidecar mismatch')
    return {
        'source_path': str(source), 'run_metadata_path': str(metadata),
        'sha256': source_sha, 'run_metadata_sha256': metadata_sha,
        'binding_kind': 'run_root_preregistration_copy',
        'marker_path': str(marker), 'marker_sha256': sha256_file(marker),
        'marker_sidecar_path': str(marker_sidecar),
        'marker_sidecar_verified': True,
    }


def _memory_contract(value: dict[str, Any]) -> bool:
    return value.get('rss_status') == 'pass' and value.get('status') == 'pass' and \
        value.get('memory_max_raw') == 'max' and \
        value.get('memory_max_unlimited') is True and \
        value.get('oom_free') is True and \
        value.get('interval_requested_seconds') == EXPECTED_INTERVAL and \
        value.get('scheduler_nice') == EXPECTED_SCHEDULER_NICE and \
        value.get('sampler_stop_status') == 0 and \
        value.get('output_readability', {}).get('status') == 'pass'


def _audit_summary(summary_path: Path, run_root: Path) -> dict[str, Any]:
    try:
        summary = json.loads(summary_path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as error:
        raise AuditError('normalized summary is unreadable') from error
    sidecar = summary_path.with_suffix('.sha256')
    if not sidecar.is_file():
        sidecar = summary_path.with_name(summary_path.name + '.sha256')
    if not sidecar.is_file() or sidecar.read_text().split()[0] != sha256_file(summary_path):
        raise AuditError('normalized summary sidecar mismatch')
    if summary.get('overall_status') != 'PASS' or \
            summary.get('gt_accessed') is not False or \
            summary.get('scorer_accessed') is not False or \
            summary.get('frozen_performance_bags_accessed') is not False or \
            summary.get('campaign4_started') is not False:
        raise AuditError('normalized summary safety flags are not PASS')
    overhead = summary.get('overhead', {})
    gate = overhead.get('gate', {})
    if overhead.get('root') != str(run_root) or overhead.get('marker_count') != 40 or \
            overhead.get('missing_markers') != [] or \
            gate.get('median_abs_pass') is not True or \
            gate.get('bootstrap_95_upper_pass') is not True:
        raise AuditError('normalized overhead summary is incomplete')
    memory_records: list[dict[str, Any]] = []
    signal = summary.get('signal_smoke', {})
    if signal.get('docker_remnants') != 'none' or \
            {row.get('name') for row in signal.get('rows', [])} != \
            {'nonzero', 'term', 'int'}:
        raise AuditError('signal lineage is incomplete')
    for row in signal['rows']:
        if row.get('exit_match') is not True or \
                row.get('actual_exit') != row.get('expected_exit') or \
                not _memory_contract(row.get('memory', {})):
            raise AuditError(f"signal lineage failed: {row.get('name')}")
        memory_records.append(row['memory'])
    allocation = summary.get('allocation_cache_separation', {})
    if allocation.get('allocation_pass') is not True or \
            allocation.get('cache_separation_pass') is not True or \
            allocation.get('alloc128_process_rss_delta_bytes', 0) < 104857600:
        raise AuditError('allocation/cache lineage failed')
    for name in ('baseline', 'alloc128', 'cache512'):
        if not _memory_contract(allocation.get(name, {})):
            raise AuditError(f'allocation lineage failed: {name}')
        memory_records.append(allocation[name])
    smoke = summary.get('wrapper_smoke', {})
    if smoke.get('all_pass') is not True or smoke.get('docker_remnants') != 'none' or \
            {row.get('system') for row in smoke.get('rows', [])} != \
            {'ours', 'glim', 'fast'}:
        raise AuditError('wrapper lineage is incomplete')
    for row in smoke['rows']:
        contract = row.get('contract', {})
        if row.get('exit_status') != 0 or contract.get('status') != 'pass' or \
                contract.get('gt_mounted') is not False or \
                contract.get('performance_run') is not False or \
                (row.get('system') == 'fast' and
                 contract.get('loopback_only') is not True) or \
                not _memory_contract(row.get('memory', {})):
            raise AuditError(f"wrapper lineage failed: {row.get('system')}")
        memory_records.append(row['memory'])
    for record in memory_records:
        path = Path(record.get('path', ''))
        if not path.is_dir() or any(item.name.endswith('.part')
                                    for item in path.rglob('*')):
            raise AuditError(f'evidence tree is incomplete: {path}')
        for item in path.rglob('*'):
            if item.is_file() and item.stat().st_mode & 0o444 == 0:
                raise AuditError(f'evidence file is unreadable: {item}')
    return {
        'summary_path': str(summary_path),
        'summary_sha256': sha256_file(summary_path),
        'summary_sidecar_verified': True,
        'overhead_gate': gate,
        'signal_exit_statuses': {
            row['name']: row['actual_exit'] for row in signal['rows']},
        'allocation_cache': {
            'alloc128_rss_delta_bytes': allocation[
                'alloc128_process_rss_delta_bytes'],
            'cache512_rss_delta_bytes': allocation[
                'cache512_process_rss_delta_bytes']},
        'wrapper_systems': sorted(row['system'] for row in smoke['rows']),
        'gt_accessed': False, 'scorer_accessed': False,
    }


def _machine_leaks() -> dict[str, bool]:
    docker = subprocess.run(['docker', 'ps', '-aq'], capture_output=True,
                            text=True, check=False)
    ps = subprocess.run(['ps', '-eo', 'pid=,args='], capture_output=True,
                        text=True, check=False)
    sampler = [line for line in ps.stdout.splitlines()
               if 'sample_container_process_rss.py' in line]
    return {
        'docker_empty': docker.returncode == 0 and not docker.stdout.strip(),
        'sampler_empty': not sampler,
    }


def _write_atomic(path: Path, value: dict[str, Any]) -> str:
    if path.exists() or path.with_name(path.name + '.part').exists():
        raise AuditError(f'refusing to overwrite {path}')
    part = path.with_name(path.name + '.part')
    part.write_text(json.dumps(value, indent=2, sort_keys=True) + '\n',
                    encoding='utf-8')
    os.replace(part, path)
    return sha256_file(path)


def _write_sidecar(path: Path, digest: str) -> None:
    sidecar = path.with_name(path.name + '.sha256')
    if sidecar.exists() or sidecar.with_name(sidecar.name + '.part').exists():
        raise AuditError(f'refusing to overwrite {sidecar}')
    part = sidecar.with_name(sidecar.name + '.part')
    part.write_text(f'{digest}  {path.name}\n', encoding='utf-8')
    os.replace(part, sidecar)


def audit(*, run_root: Path, source: Path, summary: Path,
          output_root: Path) -> tuple[Path, Path, str, str]:
    if output_root.exists():
        raise AuditError(f'output root already exists: {output_root}')
    output_root.mkdir(parents=True)
    binding = preregistration_binding(run_root, source)
    runs = audit_runs(run_root)
    lineage = _audit_summary(summary, run_root)
    leaks = _machine_leaks()
    if not all(leaks.values()):
        raise AuditError(f'live leak check failed: {leaks}')
    audit_value = {
        'schema_version': 1,
        'audit_kind': 'm6a7_v3_final_run_and_lineage_audit',
        'status': 'PASS', 'read_only_inputs': True,
        'gt_accessed': False, 'scorer_accessed': False,
        'campaign4_started': False, 'preregistration': binding,
        'schedule': runs, 'lineage': lineage, 'machine_checks': leaks,
    }
    audit_path = output_root / 'v3_run_audit.json'
    audit_sha = _write_atomic(audit_path, audit_value)
    _write_sidecar(audit_path, audit_sha)
    receipt = {
        'schema_version': 1,
        'receipt_kind': 'm6a7_v3_final_normalized_receipt',
        'status': 'PASS', 'gt_accessed': False, 'scorer_accessed': False,
        'campaign4_started': False, 'audit_path': str(audit_path),
        'audit_sha256': audit_sha, 'preregistration': binding,
        'lineage': lineage, 'schedule': {
            'pairs': 20, 'runs': 40, 'order': 'AB_BA_alternating',
            'all_complete': True},
        'contract': {
            'cpuset': EXPECTED_CPUSET, 'image': EXPECTED_IMAGE,
            'network': 'none', 'memory_max': 'max',
            'sampler_interval_ms': 250,
            'sampler_scheduler_nice': EXPECTED_SCHEDULER_NICE,
            'docker_client_rss_comparable': False,
            'aggregate_rss_definition':
                'sum_of_per_process_vmrss_peaks_shared_pages_may_be_recounted'},
    }
    receipt_path = output_root / 'm6a7_v3_final_receipt.json'
    receipt_sha = _write_atomic(receipt_path, receipt)
    _write_sidecar(receipt_path, receipt_sha)
    return audit_path, receipt_path, audit_sha, receipt_sha


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-root', type=Path, required=True)
    parser.add_argument('--source-prereg', type=Path, required=True)
    parser.add_argument('--summary', type=Path, required=True)
    parser.add_argument('--output-root', type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        audit_path, receipt_path, audit_sha, receipt_sha = audit(
            run_root=args.run_root, source=args.source_prereg,
            summary=args.summary, output_root=args.output_root)
    except (AuditError, OSError, json.JSONDecodeError) as error:
        print(f'audit failed closed: {error}', file=__import__('sys').stderr)
        return 1
    print(json.dumps({
        'status': 'PASS', 'audit_path': str(audit_path),
        'audit_sha256': audit_sha, 'receipt_path': str(receipt_path),
        'receipt_sha256': receipt_sha}, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
