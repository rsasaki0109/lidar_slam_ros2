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

"""Stdlib cgroup-v2 evidence writer used inside benchmark containers."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
from typing import Any


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    if path.exists():
        raise ValueError(f'final evidence already exists: {path}')
    part = path.with_name(path.name + '.part')
    if part.exists():
        raise ValueError(f'evidence part already exists: {part}')
    part.parent.mkdir(parents=True, exist_ok=True)
    part.write_text(json.dumps(value, indent=2, sort_keys=True) + '\n',
                    encoding='utf-8')
    part.replace(path)


def _read(path: Path) -> str | None:
    try:
        return path.read_text(encoding='utf-8')
    except (OSError, UnicodeError):
        return None


def _snapshot_value(root: Path, name: str) -> dict[str, Any]:
    raw = _read(root / name)
    return {'available': raw is not None, 'raw': raw}


def capture_baseline(output: Path, root: Path) -> None:
    atomic_json(output, {
        'schema_version': 1,
        'measurement_version': 'm6a7-cgroup-events-v1',
        'timestamp_utc': datetime.now(timezone.utc).isoformat(),
        'memory_events': _snapshot_value(root, 'memory.events'),
        'memory_events_local': _snapshot_value(root, 'memory.events.local'),
        'memory_pressure': _snapshot_value(root, 'memory.pressure'),
    })


def _parse_counters(raw: str | None) -> dict[str, int] | None:
    if not isinstance(raw, str):
        return None
    result: dict[str, int] = {}
    for line in raw.splitlines():
        fields = line.split()
        if len(fields) != 2 or not re.fullmatch(r'[a-z_]+', fields[0]) or \
                not re.fullmatch(r'[0-9]+', fields[1]):
            return None
        result[fields[0]] = int(fields[1])
    return result or None


def _parse_pressure(raw: str | None) -> dict[str, dict[str, int | float]] | None:
    if not isinstance(raw, str):
        return None
    result: dict[str, dict[str, int | float]] = {}
    for line in raw.splitlines():
        fields = line.split()
        if not fields or fields[0] not in ('some', 'full'):
            return None
        values: dict[str, int | float] = {}
        for field in fields[1:]:
            key, separator, value = field.partition('=')
            if not separator or key not in ('avg10', 'avg60', 'avg300', 'total'):
                return None
            try:
                number = float(value)
            except ValueError:
                return None
            if not math.isfinite(number) or number < 0:
                return None
            values[key] = int(number) if key == 'total' else number
        if set(values) != {'avg10', 'avg60', 'avg300', 'total'}:
            return None
        result[fields[0]] = values
    return result if set(result) == {'some', 'full'} else None


def _delta(before: Any, after: Any) -> tuple[Any, bool]:
    if not isinstance(before, dict) or not isinstance(after, dict) or \
            set(before) != set(after):
        return None, False
    result: dict[str, Any] = {}
    for key in before:
        if isinstance(before[key], dict):
            value, valid = _delta(before[key], after[key])
        else:
            value = after[key] - before[key]
            valid = value >= 0
        result[key] = value
        if not valid:
            return result, False
    return result, True


def _event_record(root: Path, baseline: dict[str, Any] | None,
                  key: str, filename: str, parser) -> tuple[dict[str, Any], bool]:
    old_doc = baseline.get(key) if isinstance(baseline, dict) else None
    old_raw = old_doc.get('raw') if isinstance(old_doc, dict) else None
    old = parser(old_raw)
    new_raw = _read(root / filename)
    new = parser(new_raw)
    difference, valid = _delta(old, new)
    record = {
        'baseline': old, 'final': new, 'delta': difference,
        'file': f'/sys/fs/cgroup/{filename}',
    }
    return record, bool(valid and old is not None and new is not None)


def _load_json(path: Path) -> dict[str, Any] | None:
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def _memory_values(root: Path) -> tuple[dict[str, Any], bool]:
    peak_raw = (_read(root / 'memory.peak') or '').strip()
    current_raw = (_read(root / 'memory.current') or '').strip()
    max_raw = (_read(root / 'memory.max') or '').strip()
    peak = int(peak_raw) if peak_raw.isdigit() else None
    current = int(current_raw) if current_raw.isdigit() else None
    maximum = int(max_raw) if max_raw.isdigit() else None
    valid = peak is not None and current is not None and peak >= current and \
        (max_raw == 'max' or (maximum is not None and maximum > 0 and
                              current <= maximum))
    return {
        'memory_peak_raw': peak_raw,
        'memory_current_raw': current_raw,
        'memory_max_raw': max_raw,
        'container_cgroup_peak_bytes': peak,
        'container_cgroup_total_peak_bytes': peak,
        'memory_current_bytes': current,
        'memory_max_bytes': maximum,
        'memory_max_unlimited': max_raw == 'max',
    }, bool(valid)


def _sampler_valid(sampler: dict[str, Any] | None) -> tuple[int | None, bool]:
    peak = sampler.get('peak') if isinstance(sampler, dict) else None
    vmrss = peak.get('vmrss_bytes') if isinstance(peak, dict) else None
    thresholds = sampler.get('thresholds') if isinstance(sampler, dict) else None
    errors = sampler.get('sample_errors') if isinstance(sampler, dict) else None
    races = sampler.get('pid_race_skips') if isinstance(sampler, dict) else None
    jitter = sampler.get('interval_jitter_percent') if isinstance(sampler, dict) else None
    threshold_valid = isinstance(thresholds, dict) and \
        not isinstance(thresholds.get('min_samples'), bool) and \
        isinstance(thresholds.get('min_samples'), int) and \
        thresholds.get('min_samples') >= 1 and \
        not isinstance(thresholds.get('max_errors'), bool) and \
        isinstance(thresholds.get('max_errors'), int) and \
        thresholds.get('max_errors') >= 0 and \
        not isinstance(thresholds.get('max_race_skips'), bool) and \
        isinstance(thresholds.get('max_race_skips'), int) and \
        thresholds.get('max_race_skips') >= 0 and \
        isinstance(thresholds.get('max_jitter_percent'), (int, float)) and \
        math.isfinite(float(thresholds.get('max_jitter_percent'))) and \
        thresholds.get('max_jitter_percent') >= 0
    aggregate = sampler.get('aggregate_process_tree_peak_rss_bytes') \
        if isinstance(sampler, dict) else None
    metric_valid = isinstance(sampler, dict) and \
        sampler.get('primary_metric') == \
        'aggregate_process_tree_peak_rss_bytes' and \
        sampler.get('primary_metric_definition') == \
        'sum_of_per_process_vmrss_peaks_shared_pages_may_be_recounted' and \
        isinstance(aggregate, int) and not isinstance(aggregate, bool) and \
        aggregate >= 0 and aggregate == vmrss
    valid = isinstance(sampler, dict) and metric_valid and \
        sampler.get('measurement_version') == 'm6a7-container-process-rss-v1' and \
        sampler.get('measurement_scope') == 'container_pid_namespace_proc_status' and \
        sampler.get('status') == 'pass' and sampler.get('atomic') is True and \
        isinstance(vmrss, int) and vmrss >= 0 and \
        isinstance(sampler.get('sample_count'), int) and \
        sampler.get('sample_count') >= 2 and sampler.get('sample_count') >= \
        (thresholds.get('min_samples') if isinstance(thresholds, dict) else 2) and \
        isinstance(errors, int) and errors >= 0 and \
        isinstance(races, int) and races >= 0 and \
        isinstance(jitter, (int, float)) and math.isfinite(float(jitter)) and \
        jitter >= 0 and sampler.get('missed_intervals') == 0 and \
        threshold_valid and errors <= thresholds['max_errors'] and \
        races <= thresholds['max_race_skips'] and \
        jitter <= thresholds['max_jitter_percent']
    return vmrss if isinstance(vmrss, int) else None, bool(valid)


def write_final(part: Path, final: Path, *, root: Path, baseline_path: Path,
                sampler_path: Path, timestamp: str, process_status: str,
                sampler_stop_status: str, cgroup_path: str,
                proc_self_cgroup: str, readability_status: str,
                readability_reason: str) -> dict[str, Any]:
    if final.exists():
        raise ValueError(f'final evidence already exists: {final}')
    baseline = _load_json(baseline_path)
    sampler = _load_json(sampler_path)
    event_data: dict[str, Any] = {}
    events_valid = baseline is not None
    for key, filename, parser in (
            ('memory_events', 'memory.events', _parse_counters),
            ('memory_events_local', 'memory.events.local', _parse_counters),
            ('memory_pressure', 'memory.pressure', _parse_pressure)):
        record, valid = _event_record(root, baseline, key, filename, parser)
        event_data[key] = record
        events_valid = events_valid and valid
    oom_delta: dict[str, int] = {}
    for key in ('memory_events', 'memory_events_local'):
        delta = event_data[key].get('delta')
        if isinstance(delta, dict):
            for name in ('oom', 'oom_kill'):
                value = delta.get(name, 0)
                oom_delta[f'{key}.{name}'] = value
                if value != 0:
                    events_valid = False
    memory, memory_valid = _memory_values(root)
    process_peak, sampler_valid = _sampler_valid(sampler)
    status = 'pass' if memory_valid and sampler_valid and events_valid and \
        readability_status == 'pass' and sampler_stop_status == '0' else 'invalid'
    reasons = []
    if not memory_valid:
        reasons.append('cgroup_memory_invalid')
    if not sampler_valid:
        reasons.append('process_rss_sampler_invalid')
    if not events_valid:
        reasons.append('cgroup_events_or_pressure_invalid')
    if readability_status != 'pass':
        reasons.append(readability_reason)
    if sampler_stop_status != '0':
        reasons.append('sampler_exit_nonzero')
    data: dict[str, Any] = {
        'schema_version': 2,
        'measurement_version': 'm6a7-container-memory-v2',
        'status': status,
        'status_reason': '' if status == 'pass' else ','.join(reasons),
        'measurement_scope': 'container_cgroup_v2_with_pid_rss',
        'children_included': True,
        'timestamp_utc': timestamp,
        'process_exit_status': int(process_status) if process_status.isdigit() else None,
        'cgroup_version': 2 if (root / 'cgroup.controllers').is_file() else 0,
        'cgroup_mount': '/sys/fs/cgroup', 'cgroup_path': cgroup_path,
        'proc_self_cgroup': proc_self_cgroup,
        'memory_files': {'peak': '/sys/fs/cgroup/memory.peak',
                         'current': '/sys/fs/cgroup/memory.current',
                         'max': '/sys/fs/cgroup/memory.max'},
        **memory,
        # This is a sum of each process's VmRSS peak. Shared pages are counted
        # once per process, so it is intentionally not a cgroup-unique total.
        'primary_metric': 'aggregate_process_tree_peak_rss_bytes',
        'primary_metric_definition':
            'sum_of_per_process_vmrss_peaks_shared_pages_may_be_recounted',
        'aggregate_process_tree_peak_rss_bytes': process_peak,
        'process_tree_peak_rss_bytes': process_peak,
        'process_rss_metric_definition':
            'sum_of_per_process_vmrss_peaks_shared_pages_may_be_recounted',
        'process_rss_evidence_path': '/out/container_process_rss.json',
        'process_rss_evidence': sampler,
        'cgroup_events': {
            'status': 'pass' if events_valid else 'invalid',
            **event_data, 'oom_delta': oom_delta,
            'oom_free': not any(value != 0 for value in oom_delta.values()),
        },
        'sampler_stop_status': int(sampler_stop_status)
        if sampler_stop_status.isdigit() else None,
        'output_readability': {'status': readability_status,
                               'reason': readability_reason,
                               'scope': 'OUT_DIR_only'},
        'atomic': True,
    }
    atomic_json(part, data)
    # atomic_json refuses an existing final and writes its argument as a final
    # path; use os.replace here to keep the part/final protocol explicit.
    part.replace(final)
    return data


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest='mode', required=True)
    baseline = sub.add_parser('baseline')
    baseline.add_argument('--output', type=Path, required=True)
    baseline.add_argument('--cgroup-root', type=Path, default=Path('/sys/fs/cgroup'))
    final = sub.add_parser('final')
    final.add_argument('--part', type=Path, required=True)
    final.add_argument('--output', type=Path, required=True)
    final.add_argument('--baseline', type=Path, required=True)
    final.add_argument('--sampler', type=Path, required=True)
    final.add_argument('--cgroup-root', type=Path, default=Path('/sys/fs/cgroup'))
    final.add_argument('--process-status', default='0')
    final.add_argument('--sampler-stop-status', default='0')
    final.add_argument('--cgroup-path', default='')
    final.add_argument('--proc-self-cgroup', default='')
    final.add_argument('--readability-status', default='pass')
    final.add_argument('--readability-reason', default='output_tree_chmod_a+rX')
    args = parser.parse_args(argv)
    if args.mode == 'baseline':
        capture_baseline(args.output, args.cgroup_root)
        return 0
    write_final(args.part, args.output, root=args.cgroup_root,
                baseline_path=args.baseline, sampler_path=args.sampler,
                timestamp=datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'),
                process_status=args.process_status,
                sampler_stop_status=args.sampler_stop_status,
                cgroup_path=args.cgroup_path,
                proc_self_cgroup=args.proc_self_cgroup,
                readability_status=args.readability_status,
                readability_reason=args.readability_reason)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
