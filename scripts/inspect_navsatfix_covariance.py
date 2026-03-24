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

"""Inspect NavSatFix covariance quality in a ROS 2 bag."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
import sys
from typing import Iterable

try:
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
except ImportError:  # pragma: no cover
    AnyReader = None
    Stores = None
    get_typestore = None


COVARIANCE_TYPE_UNKNOWN = 0


class NavSatFixRecord:
    """Minimal NavSatFix fields needed for covariance inspection."""

    __slots__ = (
        'stamp_sec',
        'latitude',
        'longitude',
        'altitude',
        'status',
        'covariance_type',
        'position_covariance',
    )

    def __init__(
        self,
        *,
        stamp_sec: float,
        latitude: float,
        longitude: float,
        altitude: float,
        status: int,
        covariance_type: int,
        position_covariance: tuple[float, float, float, float, float, float, float, float, float],
    ) -> None:
        self.stamp_sec = stamp_sec
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.status = status
        self.covariance_type = covariance_type
        self.position_covariance = position_covariance


def _usable_fix_reason(record: NavSatFixRecord) -> str | None:
    if not (
        math.isfinite(record.latitude)
        and math.isfinite(record.longitude)
        and math.isfinite(record.altitude)
    ):
        return 'non_finite'
    if record.latitude < -90.0 or record.latitude > 90.0:
        return 'latitude_out_of_range'
    if record.longitude < -180.0 or record.longitude > 180.0:
        return 'longitude_out_of_range'
    if abs(record.latitude) < 1e-6 and abs(record.longitude) < 1e-6:
        return 'zero_origin'
    return None


def _has_known_covariance(record: NavSatFixRecord) -> bool:
    if record.covariance_type == COVARIANCE_TYPE_UNKNOWN:
        return False
    var_x = record.position_covariance[0]
    var_y = record.position_covariance[4]
    var_z = record.position_covariance[8]
    return (
        math.isfinite(var_x)
        and math.isfinite(var_y)
        and math.isfinite(var_z)
        and var_x > 0.0
        and var_y > 0.0
        and var_z > 0.0
    )


def _clamp_variance(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def _percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    index = round((len(values) - 1) * fraction)
    return values[index]


def summarize_navsatfix_records(
    records: Iterable[NavSatFixRecord],
    *,
    rtk_threshold_m: float = 0.3,
    min_variance_m2: float = 0.01,
    max_variance_m2: float = 25.0,
) -> dict[str, object]:
    """Summarize usable/invalid fixes and covariance-driven RTK-like classes."""
    total_messages = 0
    usable_fixes = 0
    invalid_reason_counts: dict[str, int] = {}
    status_counts: dict[str, int] = {}
    covariance_type_counts: dict[str, int] = {}
    covariance_known = 0
    covariance_unknown = 0
    usable_covariance_known = 0
    usable_covariance_unknown = 0
    rtk_like = 0
    non_rtk = 0
    horizontal_stddevs: list[float] = []

    for record in records:
        total_messages += 1
        status_key = str(record.status)
        status_counts[status_key] = status_counts.get(status_key, 0) + 1
        covariance_key = str(record.covariance_type)
        covariance_type_counts[covariance_key] = covariance_type_counts.get(covariance_key, 0) + 1

        covariance_valid = _has_known_covariance(record)
        if covariance_valid:
            covariance_known += 1
        else:
            covariance_unknown += 1

        invalid_reason = _usable_fix_reason(record)
        if invalid_reason is not None:
            invalid_reason_counts[invalid_reason] = invalid_reason_counts.get(invalid_reason, 0) + 1
            continue

        usable_fixes += 1
        if not covariance_valid:
            usable_covariance_unknown += 1
            continue

        usable_covariance_known += 1
        var_x = _clamp_variance(record.position_covariance[0], min_variance_m2, max_variance_m2)
        var_y = _clamp_variance(record.position_covariance[4], min_variance_m2, max_variance_m2)
        horizontal_stddev = math.sqrt(max(var_x, var_y))
        horizontal_stddevs.append(horizontal_stddev)
        if horizontal_stddev <= rtk_threshold_m:
            rtk_like += 1
        else:
            non_rtk += 1

    horizontal_stddevs.sort()
    summary: dict[str, object] = {
        'total_messages': total_messages,
        'usable_fixes': usable_fixes,
        'invalid_fixes': total_messages - usable_fixes,
        'invalid_reason_counts': invalid_reason_counts,
        'covariance_known': covariance_known,
        'covariance_unknown': covariance_unknown,
        'usable_covariance_known': usable_covariance_known,
        'usable_covariance_unknown': usable_covariance_unknown,
        'rtk_threshold_m': rtk_threshold_m,
        'rtk_like': rtk_like,
        'non_rtk': non_rtk,
        'status_counts': status_counts,
        'covariance_type_counts': covariance_type_counts,
    }
    if horizontal_stddevs:
        summary['horizontal_stddev_m'] = {
            'min': horizontal_stddevs[0],
            'p50': statistics.median(horizontal_stddevs),
            'p90': _percentile(horizontal_stddevs, 0.90),
            'p95': _percentile(horizontal_stddevs, 0.95),
            'max': horizontal_stddevs[-1],
        }
    else:
        summary['horizontal_stddev_m'] = None
    return summary


def _iter_bag_records(bag_path: Path, topic: str) -> Iterable[NavSatFixRecord]:
    if AnyReader is None or Stores is None or get_typestore is None:
        raise RuntimeError('rosbags is not installed')

    with AnyReader(
        [bag_path],
        default_typestore=get_typestore(Stores.LATEST),
    ) as reader:
        connections = [conn for conn in reader.connections if conn.topic == topic]
        if not connections:
            raise RuntimeError(f'topic not found in bag: {topic}')

        for conn, _, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, conn.msgtype)
            stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            covariance = tuple(float(value) for value in msg.position_covariance)
            yield NavSatFixRecord(
                stamp_sec=stamp_sec,
                latitude=float(msg.latitude),
                longitude=float(msg.longitude),
                altitude=float(msg.altitude),
                status=int(msg.status.status),
                covariance_type=int(msg.position_covariance_type),
                position_covariance=covariance,
            )


def _format_summary(summary: dict[str, object], *, bag_path: Path, topic: str) -> str:
    lines = [
        f'bag: {bag_path}',
        f'topic: {topic}',
        f'total_messages: {summary["total_messages"]}',
        f'usable_fixes: {summary["usable_fixes"]}',
        f'invalid_fixes: {summary["invalid_fixes"]}',
        f'covariance_known: {summary["covariance_known"]}',
        f'covariance_unknown: {summary["covariance_unknown"]}',
        f'usable_covariance_known: {summary["usable_covariance_known"]}',
        f'usable_covariance_unknown: {summary["usable_covariance_unknown"]}',
        f'rtk_like_threshold_m: {summary["rtk_threshold_m"]:.3f}',
        f'rtk_like: {summary["rtk_like"]}',
        f'non_rtk: {summary["non_rtk"]}',
        f'invalid_reason_counts: {json.dumps(summary["invalid_reason_counts"], sort_keys=True)}',
        f'status_counts: {json.dumps(summary["status_counts"], sort_keys=True)}',
        f'covariance_type_counts: {json.dumps(summary["covariance_type_counts"], sort_keys=True)}',
    ]
    horizontal_stddev = summary['horizontal_stddev_m']
    if horizontal_stddev is None:
        lines.append('horizontal_stddev_m: null')
    else:
        lines.append(f'horizontal_stddev_m: {json.dumps(horizontal_stddev, sort_keys=True)}')
    return '\n'.join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description='Inspect NavSatFix covariance quality and RTK-like counts in a ROS 2 bag.',
    )
    parser.add_argument('bag_path', type=Path, help='Path to a rosbag2 directory.')
    parser.add_argument(
        '--topic',
        default='/gnss/fix',
        help='NavSatFix topic to inspect (default: /gnss/fix).',
    )
    parser.add_argument(
        '--rtk-threshold-m',
        type=float,
        default=0.3,
        help='Horizontal stddev threshold used to classify RTK-like fixes.',
    )
    parser.add_argument(
        '--min-variance-m2',
        type=float,
        default=0.01,
        help='Lower variance clamp used by graph_based_slam GNSS weighting.',
    )
    parser.add_argument(
        '--max-variance-m2',
        type=float,
        default=25.0,
        help='Upper variance clamp used by graph_based_slam GNSS weighting.',
    )
    parser.add_argument(
        '--json',
        action='store_true',
        help='Emit the summary as JSON instead of plain text.',
    )
    args = parser.parse_args(argv)

    if args.rtk_threshold_m <= 0.0:
        parser.error('--rtk-threshold-m must be positive')
    if args.min_variance_m2 <= 0.0:
        parser.error('--min-variance-m2 must be positive')
    if args.max_variance_m2 < args.min_variance_m2:
        parser.error('--max-variance-m2 must be >= --min-variance-m2')
    if not args.bag_path.is_dir():
        parser.error(f'bag path does not exist: {args.bag_path}')

    try:
        summary = summarize_navsatfix_records(
            _iter_bag_records(args.bag_path, args.topic),
            rtk_threshold_m=args.rtk_threshold_m,
            min_variance_m2=args.min_variance_m2,
            max_variance_m2=args.max_variance_m2,
        )
    except RuntimeError as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 1

    if args.json:
        payload = {
            'bag_path': str(args.bag_path),
            'topic': args.topic,
            'summary': summary,
        }
        print(json.dumps(payload, indent=2, sort_keys=True))
    else:
        print(_format_summary(summary, bag_path=args.bag_path, topic=args.topic))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
