#!/usr/bin/env python3
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

"""Convert GNU time -v output into the shared runtime evidence contract."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml


def elapsed_seconds(value: str) -> float:
    """Parse GNU time h:mm:ss or m:ss elapsed values."""
    fields = value.strip().split(':')
    if len(fields) == 2:
        return float(fields[0]) * 60.0 + float(fields[1])
    if len(fields) == 3:
        return float(fields[0]) * 3600.0 + float(fields[1]) * 60.0 + float(fields[2])
    raise ValueError(f'invalid elapsed time {value!r}')


def parse_gnu_time(text: str, bag_duration_sec: float,
                   repetitions: int = 1) -> dict:
    """Extract wall time, peak RSS, CPU load, and processing/sensor ratio."""
    if bag_duration_sec <= 0.0:
        raise ValueError('bag duration must be positive')
    if repetitions < 1:
        raise ValueError('repetitions must be positive')
    elapsed_key = 'Elapsed (wall clock) time (h:mm:ss or m:ss)'
    known_keys = (elapsed_key, 'Maximum resident set size (kbytes)',
                  'Percent of CPU this job got', 'Exit status')
    fields = {}
    for line in text.splitlines():
        stripped = line.strip()
        for key in known_keys:
            prefix = key + ':'
            if stripped.startswith(prefix):
                fields[key] = stripped[len(prefix):].strip()
                break
    if elapsed_key not in fields or 'Maximum resident set size (kbytes)' not in fields:
        raise ValueError('GNU time report lacks elapsed time or maximum RSS')
    total_wall = elapsed_seconds(fields[elapsed_key])
    wall = total_wall / repetitions
    cpu = fields.get('Percent of CPU this job got', '').rstrip('%')
    return {
        'wall_time_sec': wall,
        'total_wall_time_sec': total_wall,
        'repetitions': repetitions,
        'bag_duration_sec': float(bag_duration_sec),
        'realtime_factor': wall / bag_duration_sec,
        'peak_rss_mb': float(fields['Maximum resident set size (kbytes)']) / 1024.0,
        'cpu_percent': float(cpu) if cpu else None,
        'process_exit_status': int(fields.get('Exit status', '0')),
        'source': 'GNU time -v around the timed benchmark stage',
    }


def read_rosbag2_duration(metadata_path: Path) -> float:
    """Read the recorded duration from a rosbag2 metadata.yaml file."""
    metadata = yaml.safe_load(metadata_path.read_text())
    try:
        nanoseconds = metadata['rosbag2_bagfile_information']['duration']['nanoseconds']
        duration = float(nanoseconds) / 1e9
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(
            f'{metadata_path} lacks rosbag2 duration.nanoseconds') from exc
    if duration <= 0.0:
        raise ValueError('bag duration must be positive')
    return duration


def main() -> int:
    """Convert one GNU time report into normalized JSON evidence."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--time-file', type=Path, required=True)
    duration = parser.add_mutually_exclusive_group(required=True)
    duration.add_argument('--bag-duration-sec', type=float)
    duration.add_argument(
        '--bag-metadata', type=Path,
        help='rosbag2 metadata.yaml; duration is read automatically')
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument(
        '--repetitions', type=int, default=1,
        help='identical timed runs; wall time is normalized per run')
    args = parser.parse_args()
    bag_duration_sec = (
        read_rosbag2_duration(args.bag_metadata)
        if args.bag_metadata is not None else args.bag_duration_sec)
    report = parse_gnu_time(
        args.time_file.read_text(), bag_duration_sec, args.repetitions)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
