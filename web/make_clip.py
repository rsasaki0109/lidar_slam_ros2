#!/usr/bin/env python3
"""Write a compact LiDAR + IMU clip (LSWC2) for the browser demo from a rosbag2.

Layout, little-endian, gzip-compressed as a whole:
  b"LSWC2\\0\\0\\0", uint32 json_len, json header (UTF-8)
  IMU: n_imu x (int32 sec, uint32 nsec, 6 x float64: ax ay az wx wy wz)
  scans: n_scans x (int32 sec, uint32 nsec, uint32 n, then x[n], y[n], z[n], dt[n])
    x, y, z: centimetres, dt: point time after the header stamp in 2-microsecond units,
    each column stored as uint16 zigzag deltas (the first value against 0).

The scans keep every --keep-every-th point of the cloud. The point times come from the
cloud's float64 "timestamp" field (absolute nanoseconds, as Livox MID-360 drivers publish).
"""

import argparse
import gzip
import json
import struct
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


def zigzag_deltas(values: np.ndarray) -> bytes:
    deltas = np.diff(values.astype(np.int64), prepend=0)
    if np.abs(deltas).max(initial=0) >= 32768:
        raise ValueError('a delta does not fit in int16')
    deltas = deltas.astype(np.int32)
    return ((deltas << 1) ^ (deltas >> 31)).astype('<u2').tobytes()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('bag')
    parser.add_argument('out', help='output .lswc2.gz')
    parser.add_argument('--lidar-topic', default='/livox/lidar')
    parser.add_argument('--imu-topic', default='/livox/imu')
    parser.add_argument('--start', type=float, default=0.0,
                        help='seconds after the first message')
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--keep-every', type=int, default=4, help='keep every Nth point')
    parser.add_argument('--meta', default='{}', help='JSON merged into the header')
    args = parser.parse_args()

    imu_rows = []
    scans = []
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    with AnyReader([Path(args.bag)], default_typestore=typestore) as reader:
        connections = [c for c in reader.connections
                       if c.topic in (args.lidar_topic, args.imu_topic)]
        first = None
        for connection, receive_time, raw in reader.messages(connections=connections):
            first = receive_time if first is None else first
            relative = (receive_time - first) * 1e-9
            if relative < args.start:
                continue
            if relative > args.start + args.duration:
                break
            msg = reader.deserialize(raw, connection.msgtype)
            stamp = msg.header.stamp
            if connection.topic == args.imu_topic:
                a, w = msg.linear_acceleration, msg.angular_velocity
                imu_rows.append(struct.pack('<iI6d', stamp.sec, stamp.nanosec,
                                            a.x, a.y, a.z, w.x, w.y, w.z))
                continue
            fields = {f.name: f for f in msg.fields}
            dtype = np.dtype({
                'names': ['x', 'y', 'z', 'timestamp'],
                'formats': ['<f4', '<f4', '<f4', '<f8'],
                'offsets': [fields[name].offset for name in ('x', 'y', 'z', 'timestamp')],
                'itemsize': msg.point_step,
            })
            points = np.frombuffer(msg.data, dtype=dtype)[::args.keep_every]
            xyz = np.stack([points['x'], points['y'], points['z']], axis=1)
            keep = np.isfinite(xyz).all(axis=1) & (np.abs(xyz) < 320.0).all(axis=1)
            points, xyz = points[keep], xyz[keep]
            header_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            dt = np.clip(np.round((points['timestamp'] - header_ns) / 2000.0), 0, 65535)
            centimetres = np.round(xyz * 100.0).astype(np.int64)
            scans.append(struct.pack('<iII', stamp.sec, stamp.nanosec, len(points))
                         + zigzag_deltas(centimetres[:, 0]) + zigzag_deltas(centimetres[:, 1])
                         + zigzag_deltas(centimetres[:, 2]) + zigzag_deltas(dt))

    header = {
        'format': 'LSWC2', 'n_imu': len(imu_rows), 'n_scans': len(scans),
        'xyz_scale_m': 0.01, 'time_unit_s': 2e-6, 'keep_every': args.keep_every,
        'start_s': args.start, 'duration_s': args.duration,
    }
    header.update(json.loads(args.meta))
    blob = json.dumps(header, sort_keys=True).encode()
    raw = (b'LSWC2\0\0\0' + struct.pack('<I', len(blob)) + blob
           + b''.join(imu_rows) + b''.join(scans))
    Path(args.out).write_bytes(gzip.compress(raw, compresslevel=9, mtime=0))
    print(json.dumps(header, sort_keys=True), len(raw), Path(args.out).stat().st_size)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
