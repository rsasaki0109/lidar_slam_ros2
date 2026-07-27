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

"""Tests for the bounded product map soak harness."""

import argparse
import importlib.util
import json
from pathlib import Path

import jsonschema
import pytest


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'run_map_soak',
    ROOT / 'scripts/run_map_soak.py',
)
SOAK = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SOAK)


def _args(tmp_path: Path, **overrides) -> argparse.Namespace:
    bag = tmp_path / 'bag'
    bag.mkdir(exist_ok=True)
    (bag / 'metadata.yaml').write_text(
        'rosbag2_bagfile_information: {}\n',
        encoding='utf-8',
    )
    cli = tmp_path / 'lidarslam-map'
    cli.write_text('#!/bin/sh\n', encoding='utf-8')
    cli.chmod(0o755)
    values = {
        'bag': str(bag),
        'output_root': str(tmp_path / 'soak'),
        'soak_profile': 'one-hour',
        'hardware_label': 'ci-amd64-test-host',
        'map_profile': 'mid360_livox_smoke',
        'cli': str(cli),
        'max_peak_rss_mib': 512.0,
        'max_output_gib': 1.0,
        'max_dropped_inputs': 0,
        'min_free_space_gib': 0.01,
    }
    values.update(overrides)
    return argparse.Namespace(**values)


def test_profiles_are_exactly_one_and_eight_hours():
    assert SOAK.PROFILE_DURATIONS == {
        'one-hour': 3600.0,
        'eight-hour': 28800.0,
    }


def test_parse_gnu_time_reports_wall_rss_and_exit():
    report = SOAK.parse_gnu_time(
        'Elapsed (wall clock) time (h:mm:ss or m:ss): 1:02:03.50\n'
        'Maximum resident set size (kbytes): 262144\n'
        'Exit status: 7\n'
    )

    assert report == {
        'wall_time_sec': 3723.5,
        'peak_rss_mib': 256.0,
        'exit_code': 7,
    }


def test_parse_gnu_time_rejects_incomplete_evidence():
    with pytest.raises(ValueError, match='peak_rss'):
        SOAK.parse_gnu_time(
            'Elapsed (wall clock) time (h:mm:ss or m:ss): 2:00.00\n'
            'Exit status: 0\n'
        )


def test_drop_counters_are_explicit_and_conservative():
    counts = SOAK.count_dropped_inputs(
        'Message Filter dropping message because its queue is full\n'
        'Dropping scan during kidnap recovery\n'
        'unrelated warning\n'
    )

    assert counts == {
        'message_filter': 1,
        'scan_drop': 1,
        'queue_overflow': 1,
    }


def test_successful_soak_writes_schema_valid_threshold_evidence(tmp_path):
    args = _args(tmp_path)
    ticks = iter([0.0, 0.0, 3600.0, 3600.0])

    def fake_iteration(command, log_path, time_path):
        run_dir = Path(command[command.index('--output-dir') + 1])
        run_dir.mkdir(parents=True)
        (run_dir / 'map.pcd').write_bytes(b'x' * 1024)
        log_path.write_text('', encoding='utf-8')
        time_path.write_text('fake GNU time evidence\n', encoding='utf-8')
        return (
            0,
            {'wall_time_sec': 30.0, 'peak_rss_mib': 128.0, 'exit_code': 0},
            {'message_filter': 0, 'scan_drop': 0, 'queue_overflow': 0},
        )

    exit_code, report = SOAK.run_soak(
        args,
        clock=lambda: next(ticks),
        run_iteration=fake_iteration,
    )

    assert exit_code == 0
    assert report['status'] == 'passed'
    assert report['metrics']['iterations_completed'] == 1
    assert report['metrics']['wall_time_sec'] == 3600.0
    assert all(report['checks'].values())
    saved = json.loads((Path(args.output_root) / SOAK.REPORT_NAME).read_text())
    schema = json.loads(
        (ROOT / 'docs/schemas/soak-report-v1.schema.json').read_text()
    )
    jsonschema.Draft7Validator.check_schema(schema)
    jsonschema.validate(saved, schema)


def test_failed_iteration_stops_and_preserves_failed_report(tmp_path):
    args = _args(tmp_path)
    ticks = iter([0.0, 0.0, 12.0])

    def fake_iteration(command, log_path, time_path):
        log_path.write_text('Dropping scan: queue full\n', encoding='utf-8')
        time_path.write_text('fake GNU time evidence\n', encoding='utf-8')
        return (
            9,
            {'wall_time_sec': 12.0, 'peak_rss_mib': 64.0, 'exit_code': 9},
            {'message_filter': 0, 'scan_drop': 1, 'queue_overflow': 1},
        )

    exit_code, report = SOAK.run_soak(
        args,
        clock=lambda: next(ticks),
        run_iteration=fake_iteration,
    )

    assert exit_code == 1
    assert report['status'] == 'failed'
    assert report['metrics']['iterations_failed'] == 1
    assert report['metrics']['dropped_inputs_total'] == 2
    assert report['checks']['target_duration_reached'] is False
    assert report['checks']['all_iterations_succeeded'] is False
    assert (Path(args.output_root) / SOAK.REPORT_NAME).is_file()


def test_telemetry_failure_becomes_terminal_evidence(tmp_path):
    args = _args(tmp_path)
    ticks = iter([0.0, 0.0, 3.0])

    def broken_iteration(command, log_path, time_path):
        raise ValueError('GNU time report lacks fields')

    exit_code, report = SOAK.run_soak(
        args,
        clock=lambda: next(ticks),
        run_iteration=broken_iteration,
    )

    assert exit_code == 1
    assert report['status'] == 'failed'
    assert report['completed_at'] is not None
    assert 'lacks fields' in report['last_error']
    saved = json.loads((Path(args.output_root) / SOAK.REPORT_NAME).read_text())
    assert saved['status'] == 'failed'
    assert saved['last_error'] == report['last_error']


def test_resource_budget_failure_stops_before_another_iteration(tmp_path):
    args = _args(tmp_path, max_peak_rss_mib=100.0)
    ticks = iter([0.0, 0.0, 10.0])
    calls = []

    def over_budget_iteration(command, log_path, time_path):
        calls.append(command)
        return (
            0,
            {'wall_time_sec': 10.0, 'peak_rss_mib': 101.0, 'exit_code': 0},
            {'message_filter': 0, 'scan_drop': 0, 'queue_overflow': 0},
        )

    exit_code, report = SOAK.run_soak(
        args,
        clock=lambda: next(ticks),
        run_iteration=over_budget_iteration,
    )

    assert exit_code == 1
    assert len(calls) == 1
    assert report['checks']['peak_rss_within_budget'] is False
    assert report['last_error'].endswith('failed peak_rss_within_budget')


def test_existing_output_is_never_overwritten(tmp_path):
    args = _args(tmp_path)
    output_root = Path(args.output_root)
    output_root.mkdir()
    marker = output_root / 'owned'
    marker.write_text('keep', encoding='utf-8')

    with pytest.raises(ValueError, match='already exists'):
        SOAK.run_soak(args)

    assert marker.read_text(encoding='utf-8') == 'keep'
