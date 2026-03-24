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

"""Inspect Applanix GSOF50 GNSS quality in a ROS 2 bag."""

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
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
except ImportError:  # pragma: no cover
    AnyReader = None
    Stores = None
    get_typestore = None
    get_types_from_msg = None


GNSS_STATUS_NAMES = {
    0: 'FIX_NOT_AVAILABLE',
    1: 'GNSS_SPS_MODE',
    2: 'DIFFERENTIAL_GPS_SPS',
    3: 'GPS_PPS_MODE',
    4: 'FIXED_RTK_MODE',
    5: 'FLOAT_RTK',
    6: 'DIRECT_GEOREFERENCING_MODE',
    7: 'GNSS_UNKNOWN',
}

IMU_ALIGNMENT_NAMES = {
    0: 'GPS_ONLY',
    1: 'COARSE_LEVELING',
    2: 'DEGRADED',
    3: 'ALIGNED',
    4: 'FULL_NAV',
    5: 'IMU_UNKNOWN',
}


class ApplanixGsof50Record:
    """Minimal GSOF50 fields needed for quality inspection."""

    __slots__ = (
        'stamp_sec',
        'gnss_status',
        'imu_alignment',
        'pos_rms_north_m',
        'pos_rms_east_m',
        'heading_rms_deg',
    )

    def __init__(
        self,
        *,
        stamp_sec: float,
        gnss_status: int,
        imu_alignment: int,
        pos_rms_north_m: float,
        pos_rms_east_m: float,
        heading_rms_deg: float,
    ) -> None:
        self.stamp_sec = stamp_sec
        self.gnss_status = gnss_status
        self.imu_alignment = imu_alignment
        self.pos_rms_north_m = pos_rms_north_m
        self.pos_rms_east_m = pos_rms_east_m
        self.heading_rms_deg = heading_rms_deg


def _percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    index = round((len(values) - 1) * fraction)
    return values[index]


def _summarize_distribution(values: list[float]) -> dict[str, float] | None:
    if not values:
        return None
    ordered = sorted(values)
    return {
        'min': ordered[0],
        'p50': statistics.median(ordered),
        'p90': _percentile(ordered, 0.90),
        'p95': _percentile(ordered, 0.95),
        'max': ordered[-1],
    }


def summarize_applanix_gsof50_records(
    records: Iterable[ApplanixGsof50Record],
) -> dict[str, object]:
    """Summarize GNSS mode counts and RMS quality from GSOF50 messages."""
    total_messages = 0
    gnss_mode_counts: dict[str, int] = {}
    imu_alignment_counts: dict[str, int] = {}
    horizontal_rms_values: list[float] = []
    heading_rms_values: list[float] = []

    for record in records:
        total_messages += 1
        gnss_name = GNSS_STATUS_NAMES.get(record.gnss_status, str(record.gnss_status))
        gnss_mode_counts[gnss_name] = gnss_mode_counts.get(gnss_name, 0) + 1
        imu_name = IMU_ALIGNMENT_NAMES.get(record.imu_alignment, str(record.imu_alignment))
        imu_alignment_counts[imu_name] = imu_alignment_counts.get(imu_name, 0) + 1

        north = record.pos_rms_north_m
        east = record.pos_rms_east_m
        if math.isfinite(north) and math.isfinite(east):
            horizontal_rms_values.append(math.hypot(north, east))

        if math.isfinite(record.heading_rms_deg):
            heading_rms_values.append(record.heading_rms_deg)

    return {
        'total_messages': total_messages,
        'gnss_mode_counts': gnss_mode_counts,
        'imu_alignment_counts': imu_alignment_counts,
        'horizontal_rms_m': _summarize_distribution(horizontal_rms_values),
        'heading_rms_deg': _summarize_distribution(heading_rms_values),
    }


def _default_applanix_msg_dirs(repo_root: Path) -> list[Path]:
    return [
        repo_root / 'Thirdparty' / 'applanix' / 'applanix_msgs' / 'msg',
        repo_root / 'applanix_msgs' / 'msg',
        Path('/tmp/applanix/applanix_msgs/msg'),
    ]


def _resolve_applanix_msg_dir(requested: Path | None, repo_root: Path) -> Path:
    candidates = [requested] if requested is not None else _default_applanix_msg_dirs(repo_root)
    for candidate in candidates:
        if candidate is not None and candidate.is_dir():
            return candidate
    raise RuntimeError(
        'could not find applanix_msgs message definitions; pass '
        '--applanix-msg-dir or clone https://github.com/autowarefoundation/applanix.git',
    )


def _load_applanix_typestore(msg_dir: Path):
    if AnyReader is None or Stores is None or get_typestore is None or get_types_from_msg is None:
        raise RuntimeError('rosbags is not installed')

    typestore = get_typestore(Stores.LATEST)
    for path in sorted(msg_dir.glob('*.msg')):
        text = path.read_text(encoding='utf-8')
        msg_name = f'applanix_msgs/msg/{path.stem}'
        typestore.register(get_types_from_msg(text, msg_name))
    return typestore


def _iter_bag_records(bag_path: Path, topic: str, msg_dir: Path) -> Iterable[ApplanixGsof50Record]:
    typestore = _load_applanix_typestore(msg_dir)
    with AnyReader([bag_path], default_typestore=typestore) as reader:
        connections = [conn for conn in reader.connections if conn.topic == topic]
        if not connections:
            raise RuntimeError(f'topic not found in bag: {topic}')

        for conn, _, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, conn.msgtype)
            stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            yield ApplanixGsof50Record(
                stamp_sec=stamp_sec,
                gnss_status=int(msg.status.gnss),
                imu_alignment=int(msg.status.imu_alignment),
                pos_rms_north_m=float(msg.pos_rms_error.north),
                pos_rms_east_m=float(msg.pos_rms_error.east),
                heading_rms_deg=float(msg.attitude_rms_error_heading),
            )


def _format_summary(summary: dict[str, object], *, bag_path: Path, topic: str, msg_dir: Path) -> str:
    lines = [
        f'bag: {bag_path}',
        f'topic: {topic}',
        f'applanix_msg_dir: {msg_dir}',
        f'total_messages: {summary["total_messages"]}',
        f'gnss_mode_counts: {json.dumps(summary["gnss_mode_counts"], sort_keys=True)}',
        f'imu_alignment_counts: {json.dumps(summary["imu_alignment_counts"], sort_keys=True)}',
        f'horizontal_rms_m: {json.dumps(summary["horizontal_rms_m"], sort_keys=True)}',
        f'heading_rms_deg: {json.dumps(summary["heading_rms_deg"], sort_keys=True)}',
    ]
    return '\n'.join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description='Inspect Applanix GSOF50 GNSS quality in a ROS 2 bag.',
    )
    parser.add_argument('bag_path', type=Path, help='Path to a rosbag2 directory.')
    parser.add_argument(
        '--topic',
        default='/lvx_client/gsof/ins_solution_rms_50',
        help='GSOF50 topic to inspect (default: /lvx_client/gsof/ins_solution_rms_50).',
    )
    parser.add_argument(
        '--applanix-msg-dir',
        type=Path,
        default=None,
        help='Path to applanix_msgs/msg containing *.msg definitions.',
    )
    parser.add_argument(
        '--json',
        action='store_true',
        help='Emit the summary as JSON instead of plain text.',
    )
    args = parser.parse_args(argv)

    if not args.bag_path.is_dir():
        parser.error(f'bag path does not exist: {args.bag_path}')

    repo_root = Path(__file__).resolve().parents[1]
    try:
        msg_dir = _resolve_applanix_msg_dir(args.applanix_msg_dir, repo_root)
        summary = summarize_applanix_gsof50_records(
            _iter_bag_records(args.bag_path, args.topic, msg_dir),
        )
    except RuntimeError as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 1

    if args.json:
        payload = {
            'bag_path': str(args.bag_path),
            'topic': args.topic,
            'applanix_msg_dir': str(msg_dir),
            'summary': summary,
        }
        print(json.dumps(payload, indent=2, sort_keys=True))
    else:
        print(_format_summary(summary, bag_path=args.bag_path, topic=args.topic, msg_dir=msg_dir))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
