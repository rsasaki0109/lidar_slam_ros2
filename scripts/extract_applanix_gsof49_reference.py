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

"""Extract an Applanix GSOF49 trajectory into TUM format."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def import_rosbags_modules():
    """Import rosbags lazily so helper tests stay lightweight."""
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError(
            'rosbags is required to extract Applanix references from rosbag2',
        ) from exc
    return AnyReader, Stores, get_typestore, get_types_from_msg


def _default_applanix_msg_dirs(repo_root: Path) -> list[Path]:
    return [
        repo_root / 'Thirdparty' / 'applanix' / 'applanix_msgs' / 'msg',
        repo_root / 'applanix_msgs' / 'msg',
        Path('/tmp/applanix/applanix_msgs/msg'),
    ]


def resolve_applanix_msg_dir(requested: Path | None, repo_root: Path) -> Path:
    """Locate applanix_msgs message definitions."""
    candidates = [requested] if requested is not None else _default_applanix_msg_dirs(repo_root)
    for candidate in candidates:
        if candidate is not None and candidate.is_dir():
            return candidate
    raise RuntimeError(
        'could not find applanix_msgs message definitions; pass '
        '--applanix-msg-dir or clone https://github.com/autowarefoundation/applanix.git',
    )


def load_typestore_with_applanix(msg_dir: Path):
    """Build a typestore that knows both standard ROS 2 and applanix_msgs."""
    _, Stores, get_typestore, get_types_from_msg = import_rosbags_modules()
    typestore = get_typestore(Stores.LATEST)
    for path in sorted(msg_dir.glob('*.msg')):
        text = path.read_text(encoding='utf-8')
        msg_name = f'applanix_msgs/msg/{path.stem}'
        typestore.register(get_types_from_msg(text, msg_name))
    return typestore


def lla_to_ecef(latitude_deg: float, longitude_deg: float, altitude_m: float) -> tuple[float, float, float]:
    """Convert WGS84 LLA into ECEF."""
    lat = math.radians(latitude_deg)
    lon = math.radians(longitude_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    radius = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (radius + altitude_m) * cos_lat * cos_lon
    y = (radius + altitude_m) * cos_lat * sin_lon
    z = (radius * (1.0 - WGS84_E2) + altitude_m) * sin_lat
    return x, y, z


def ecef_to_enu(
    x: float,
    y: float,
    z: float,
    origin_latitude_deg: float,
    origin_longitude_deg: float,
    origin_altitude_m: float,
) -> tuple[float, float, float]:
    """Convert ECEF into local ENU relative to the origin LLA."""
    ox, oy, oz = lla_to_ecef(
        origin_latitude_deg,
        origin_longitude_deg,
        origin_altitude_m,
    )
    dx = x - ox
    dy = y - oy
    dz = z - oz

    lat0 = math.radians(origin_latitude_deg)
    lon0 = math.radians(origin_longitude_deg)
    sin_lat0 = math.sin(lat0)
    cos_lat0 = math.cos(lat0)
    sin_lon0 = math.sin(lon0)
    cos_lon0 = math.cos(lon0)

    east = -sin_lon0 * dx + cos_lon0 * dy
    north = (
        -sin_lat0 * cos_lon0 * dx
        - sin_lat0 * sin_lon0 * dy
        + cos_lat0 * dz
    )
    up = (
        cos_lat0 * cos_lon0 * dx
        + cos_lat0 * sin_lon0 * dy
        + sin_lat0 * dz
    )
    return east, north, up


def lla_to_enu(
    latitude_deg: float,
    longitude_deg: float,
    altitude_m: float,
    origin_latitude_deg: float,
    origin_longitude_deg: float,
    origin_altitude_m: float,
) -> tuple[float, float, float]:
    """Convert WGS84 LLA into local ENU relative to the origin LLA."""
    return ecef_to_enu(
        *lla_to_ecef(latitude_deg, longitude_deg, altitude_m),
        origin_latitude_deg,
        origin_longitude_deg,
        origin_altitude_m,
    )


def heading_deg_to_enu_yaw_deg(heading_deg: float) -> float:
    """Convert north-clockwise heading into ENU yaw."""
    yaw_deg = 90.0 - heading_deg
    while yaw_deg <= -180.0:
        yaw_deg += 360.0
    while yaw_deg > 180.0:
        yaw_deg -= 360.0
    return yaw_deg


def rpy_deg_to_quaternion(
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
) -> tuple[float, float, float, float]:
    """Convert roll/pitch/yaw degrees into an XYZW quaternion."""
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def is_usable_fix(
    *,
    latitude_deg: float,
    longitude_deg: float,
    altitude_m: float,
    imu_alignment: int,
    gnss_status: int,
    aligned_value: int,
    fix_not_available_value: int,
    gnss_unknown_value: int,
) -> tuple[bool, str]:
    """Return whether a GSOF49 sample is usable as a reference pose."""
    if not math.isfinite(latitude_deg) or not math.isfinite(longitude_deg):
        return False, 'non_finite_lla'
    if abs(latitude_deg) < 1e-9 and abs(longitude_deg) < 1e-9:
        return False, 'zero_origin'
    if not math.isfinite(altitude_m):
        return False, 'non_finite_altitude'
    if imu_alignment < aligned_value:
        return False, 'imu_not_aligned'
    if gnss_status == fix_not_available_value:
        return False, 'fix_not_available'
    if gnss_status == gnss_unknown_value:
        return False, 'gnss_unknown'
    return True, 'ok'


def parse_args() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description='Extract an Applanix GSOF49 reference trajectory into TUM format.',
    )
    parser.add_argument('--input', required=True, help='Input rosbag2 directory.')
    parser.add_argument('--output', required=True, help='Output TUM path.')
    parser.add_argument(
        '--topic',
        default='/lvx_client/gsof/ins_solution_49',
        help='Applanix NavigationSolutionGsof49 topic.',
    )
    parser.add_argument(
        '--applanix-msg-dir',
        type=Path,
        default=None,
        help='Path to applanix_msgs/msg containing *.msg definitions.',
    )
    parser.add_argument(
        '--meta-out',
        default='',
        help='Optional JSON summary path.',
    )
    return parser.parse_args()


def main() -> int:
    """Extract a TUM trajectory from GSOF49 messages."""
    args = parse_args()
    input_path = Path(args.input).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve()
    meta_out = (
        Path(args.meta_out).expanduser().resolve()
        if args.meta_out else None
    )
    if not input_path.is_dir():
        raise SystemExit(f'input bag not found: {input_path}')

    repo_root = Path(__file__).resolve().parents[1]
    msg_dir = resolve_applanix_msg_dir(args.applanix_msg_dir, repo_root)
    AnyReader, _, _, _ = import_rosbags_modules()
    typestore = load_typestore_with_applanix(msg_dir)

    rows: list[str] = []
    total_messages = 0
    kept_messages = 0
    skipped_counts: dict[str, int] = {}
    origin_lla: tuple[float, float, float] | None = None

    with AnyReader([input_path], default_typestore=typestore) as reader:
        conn = next((c for c in reader.connections if c.topic == args.topic), None)
        if conn is None:
            raise SystemExit(f'topic not found: {args.topic}')

        for _, stamp_ns, raw in reader.messages(connections=[conn]):
            total_messages += 1
            msg = reader.deserialize(raw, conn.msgtype)
            status = getattr(msg, 'status', None)
            lla = getattr(msg, 'lla', None)
            if status is None or lla is None:
                skipped_counts['missing_status_or_lla'] = (
                    skipped_counts.get('missing_status_or_lla', 0) + 1
                )
                continue

            usable, reason = is_usable_fix(
                latitude_deg=float(lla.latitude),
                longitude_deg=float(lla.longitude),
                altitude_m=float(lla.altitude),
                imu_alignment=int(status.imu_alignment),
                gnss_status=int(status.gnss),
                aligned_value=int(status.ALIGNED),
                fix_not_available_value=int(status.FIX_NOT_AVAILABLE),
                gnss_unknown_value=int(status.GNSS_UNKNOWN),
            )
            if not usable:
                skipped_counts[reason] = skipped_counts.get(reason, 0) + 1
                continue

            if origin_lla is None:
                origin_lla = (
                    float(lla.latitude),
                    float(lla.longitude),
                    float(lla.altitude),
                )

            x, y, z = lla_to_enu(
                float(lla.latitude),
                float(lla.longitude),
                float(lla.altitude),
                origin_lla[0],
                origin_lla[1],
                origin_lla[2],
            )
            yaw_deg = heading_deg_to_enu_yaw_deg(float(msg.heading))
            qx, qy, qz, qw = rpy_deg_to_quaternion(
                float(msg.roll),
                float(msg.pitch),
                yaw_deg,
            )
            timestamp = float(stamp_ns) * 1e-9
            rows.append(
                f'{timestamp:.9f} '
                f'{x:.9f} {y:.9f} {z:.9f} '
                f'{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}',
            )
            kept_messages += 1

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        '\n'.join(rows) + ('\n' if rows else ''),
        encoding='utf-8',
    )

    summary: dict[str, Any] = {
        'input_bag': str(input_path),
        'topic': args.topic,
        'output_tum': str(output_path),
        'applanix_msg_dir': str(msg_dir),
        'total_messages': total_messages,
        'kept_messages': kept_messages,
        'skipped_counts': skipped_counts,
    }
    if origin_lla is not None:
        summary['origin_lla'] = {
            'latitude': origin_lla[0],
            'longitude': origin_lla[1],
            'altitude': origin_lla[2],
        }

    if meta_out is not None:
        meta_out.parent.mkdir(parents=True, exist_ok=True)
        meta_out.write_text(
            json.dumps(summary, indent=2, sort_keys=True) + '\n',
            encoding='utf-8',
        )

    print(json.dumps(summary, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
