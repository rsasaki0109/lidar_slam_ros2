#!/usr/bin/env python3
"""Convert one sequence of the LUH long-term indoor dataset to a compact ROS 2 bag, streamed.

The dataset (data.uni-hannover.de, "A Multi-Scenario Dataset for Long-Term Indoor Localization
and Pedestrian Behavior Analysis in Dynamic Environments", CC BY 4.0) gives each sequence as one
ROS 1 bag of 5-19 GB with an Ouster OS1-128, its IMU and a Hesai Pandar64. This reads the bag
front to back from a file or over HTTP (resuming with Range requests), never storing it, and
writes a ROS 2 bag (zstd-compressed messages) with:

  /ouster/imu     sensor_msgs/Imu, unchanged
  /ouster/points  sensor_msgs/PointCloud2 with x, y, z, intensity (float32) and t (uint32, ns
                  after the header stamp); zero-range returns dropped; with --column-step N only
                  every N-th of the 2048 columns is kept (2: the 1024-column mode)
  /tf_static      unchanged, when the bag has it

Other topics (the Hesai) are skipped. docs/roadmap/long-term-mapping-2026-11.md uses the result.

  convert_luh_longterm_bag.py SOURCE OUT_DIR [--column-step 2] [--expect-md5 HEX]

SOURCE is a path or an https URL of sequence<i>.bag. --expect-md5 checks the whole stream against
the dataset's md5sums.txt.
"""

import argparse
import bz2
import hashlib
import struct
import sys
import time
import urllib.request
from pathlib import Path

import lz4.frame
import numpy as np
from rosbags.rosbag2 import CompressionFormat, CompressionMode, Writer
from rosbags.typesys import Stores, get_typestore

MAGIC = b'#ROSBAG V2.0\n'
OP_MSG, OP_CONN, OP_CHUNK = 0x02, 0x07, 0x05
POINTS, IMU, TF_STATIC = '/ouster/points', '/ouster/imu', '/tf_static'
KEEP = {POINTS: 'sensor_msgs/msg/PointCloud2', IMU: 'sensor_msgs/msg/Imu',
        TF_STATIC: 'tf2_msgs/msg/TFMessage'}
# PointField datatypes
FLOAT32, UINT32 = 7, 6


class Stream:
    """Sequential reader over a file or an HTTP URL, resuming the download on errors."""

    def __init__(self, source: str, retries: int = 20):
        self.source, self.retries, self.offset = source, retries, 0
        self.size = None
        self.md5 = hashlib.md5()
        self._open()

    def _open(self):
        if self.source.startswith('https://'):
            request = urllib.request.Request(
                self.source, headers={'Range': f'bytes={self.offset}-'})
            self.handle = urllib.request.urlopen(request, timeout=120)
            # Content-Range: bytes START-END/TOTAL gives the whole size on every (re)open.
            content_range = self.handle.headers.get('Content-Range', '')
            if '/' in content_range and content_range.rsplit('/', 1)[1].isdigit():
                self.size = int(content_range.rsplit('/', 1)[1])
            elif self.offset == 0 and self.handle.headers.get('Content-Length'):
                self.size = int(self.handle.headers['Content-Length'])
        else:
            self.handle = open(self.source, 'rb')
            self.handle.seek(self.offset)

    def read(self, n: int) -> bytes:
        parts, remaining, failures = [], n, 0
        while remaining:
            try:
                part = self.handle.read(min(remaining, 1 << 22))
                if not part and self.size is not None and self.offset < self.size:
                    # The server closed the connection early: resume from here.
                    raise ConnectionError(f'connection closed at {self.offset} of {self.size}')
            except OSError as error:
                part, failures = b'', failures + 1
                if failures > self.retries:
                    raise
                print(f'  read error at {self.offset}: {error}; resuming', file=sys.stderr)
                time.sleep(min(60, 2 ** failures))
                self._open()
                continue
            if not part:
                if not parts and remaining == n:
                    return b''
                raise EOFError(f'stream ended at {self.offset}, {remaining} bytes short')
            parts.append(part)
            self.md5.update(part)
            self.offset += len(part)
            remaining -= len(part)
        return b''.join(parts)


def header_fields(buf: bytes) -> dict:
    out, i = {}, 0
    while i < len(buf):
        (n,) = struct.unpack_from('<i', buf, i)
        key, _, value = buf[i + 4:i + 4 + n].partition(b'=')
        out[key.decode()] = value
        i += 4 + n
    return out


def read_record(stream: Stream):
    raw = stream.read(4)
    if not raw:
        return None
    (header_len,) = struct.unpack('<i', raw)
    header = header_fields(stream.read(header_len))
    (data_len,) = struct.unpack('<i', stream.read(4))
    return header, stream.read(data_len)


def chunk_records(buf: bytes):
    i = 0
    while i < len(buf):
        (header_len,) = struct.unpack_from('<i', buf, i)
        header = header_fields(buf[i + 4:i + 4 + header_len])
        (data_len,) = struct.unpack_from('<i', buf, i + 4 + header_len)
        yield header, buf[i + 8 + header_len:i + 8 + header_len + data_len]
        i += 8 + header_len + data_len


def compact_cloud(msg, typestore, column_step: int):
    """x, y, z, intensity, t of the returns with a range; every column_step-th column."""
    offsets = {f.name: f.offset for f in msg.fields}
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, msg.point_step)
    rows = rows[:, ::column_step]
    flat = rows.reshape(-1, msg.point_step)

    def column(name, dtype):
        return flat[:, offsets[name]:offsets[name] + 4].copy().view(dtype)[:, 0]

    x, y, z = column('x', '<f4'), column('y', '<f4'), column('z', '<f4')
    keep = (x != 0) | (y != 0) | (z != 0)
    out = np.zeros(int(keep.sum()), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                                           ('intensity', '<f4'), ('t', '<u4')])
    out['x'], out['y'], out['z'] = x[keep], y[keep], z[keep]
    out['intensity'] = column('intensity', '<f4')[keep]
    out['t'] = column('t', '<u4')[keep]
    field = typestore.types['sensor_msgs/msg/PointField']
    fields = [field(name=n, offset=o, datatype=d, count=1)
              for n, o, d in (('x', 0, FLOAT32), ('y', 4, FLOAT32), ('z', 8, FLOAT32),
                              ('intensity', 12, FLOAT32), ('t', 16, UINT32))]
    cloud = typestore.types['sensor_msgs/msg/PointCloud2']
    return cloud(header=msg.header, height=1, width=len(out), fields=fields,
                 is_bigendian=False, point_step=20, row_step=20 * len(out),
                 data=np.frombuffer(out.tobytes(), dtype=np.uint8), is_dense=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('source')
    parser.add_argument('out_dir', type=Path)
    parser.add_argument('--column-step', type=int, default=2)
    parser.add_argument('--expect-md5')
    args = parser.parse_args()

    ros1 = get_typestore(Stores.ROS1_NOETIC)
    ros2 = get_typestore(Stores.ROS2_HUMBLE)
    stream = Stream(args.source)
    if stream.read(len(MAGIC)) != MAGIC:
        raise SystemExit('not a ROS 1 bag (v2.0)')
    writer = Writer(args.out_dir, version=8)
    writer.set_compression(CompressionMode.MESSAGE, CompressionFormat.ZSTD)
    writer.open()
    topics, outputs, counts = {}, {}, {}
    started, last_report = time.monotonic(), 0.0
    try:
        while (record := read_record(stream)) is not None:
            header, data = record
            if header['op'][0] != OP_CHUNK:
                continue
            compression = header['compression'].decode()
            if compression == 'bz2':
                data = bz2.decompress(data)
            elif compression == 'lz4':
                data = lz4.frame.decompress(data)
            for inner, payload in chunk_records(data):
                op = inner['op'][0]
                if op == OP_CONN:
                    conn = struct.unpack('<i', inner['conn'])[0]
                    topics[conn] = inner['topic'].decode()
                    topic = topics[conn]
                    if topic in KEEP and topic not in outputs:
                        outputs[topic] = writer.add_connection(topic, KEEP[topic], typestore=ros2)
                elif op == OP_MSG:
                    topic = topics.get(struct.unpack('<i', inner['conn'])[0])
                    if topic not in outputs:
                        continue
                    sec, nsec = struct.unpack('<II', inner['time'])
                    stamp = sec * 1_000_000_000 + nsec
                    if topic == POINTS:
                        msg = ros1.deserialize_ros1(payload, KEEP[topic])
                        cloud = compact_cloud(msg, ros2, args.column_step)
                        cdr = ros2.serialize_cdr(cloud, KEEP[topic])
                    else:
                        # The ROS 2 types read the ROS 1 message's fields by name (the header's
                        # seq is not among them).
                        cdr = ros2.serialize_cdr(ros1.deserialize_ros1(payload, KEEP[topic]),
                                                 KEEP[topic])
                    writer.write(outputs[topic], stamp, cdr)
                    counts[topic] = counts.get(topic, 0) + 1
            now = time.monotonic()
            if now - last_report > 60:
                last_report = now
                print(f'  {stream.offset / 1e9:.2f} GB read, {counts}, {now - started:.0f} s',
                      file=sys.stderr, flush=True)
    finally:
        writer.close()
    print({'bytes_read': stream.offset, 'md5': stream.md5.hexdigest(), 'messages': counts,
           'column_step': args.column_step, 'seconds': round(time.monotonic() - started)})
    if args.expect_md5 and stream.md5.hexdigest() != args.expect_md5:
        print(f'md5 mismatch: expected {args.expect_md5}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
