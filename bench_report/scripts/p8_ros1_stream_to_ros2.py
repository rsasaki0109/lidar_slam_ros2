#!/usr/bin/env python3
"""p5_ros1_stream_to_ros2.py for the second blind run's datasets (Ouster clouds, IMU in a second bag).

Copies a LiDAR and an IMU topic of a ROS 1 bag read as a stream into a ROS 2 bag (mcap), messages in time order.

usage: producer | p8_ros1_stream_to_ros2.py - OUT_BAG --lidar TOPIC --imu TOPIC [--imu-bag FILE]
           [--lidar-stamp-offset SEC] [--keep x,y,z,intensity,t,ring] [--drop-zero] [--max-lidar N] [--summary OUT.json]

--imu-bag FILE         the IMU topic is read from this (stored) ROS 1 bag instead of the stream (MCD: one bag per
                       sensor); its messages are written between the clouds in stamp order.
--lidar-stamp-offset S added to every cloud's header stamp (MCD stamps the end of the sweep while the per-point `t`
                       counts from its start, see the dataset's user manual; -0.1 gives the usual start-of-sweep stamp).
--keep F1,F2,...       only these point fields are stored, packed without padding in this order (same data types).
--drop-zero            points with x = y = z = 0 (no return) are not stored; the cloud becomes unorganized
                       (height 1, is_dense true).
--max-lidar N          stop after N clouds (pipeline checks).
--lidar-alt TOPIC      a second name the LiDAR topic may have in the stream (PASTEL: some bags hold /os0_128/points,
                       others /os0_128/points_filtered); its clouds are written under the --lidar name. The summary
                       lists the source topics seen; a stream that holds both must be converted again without this.

Every message is written with its header stamp as the bag time. The summary lists every topic of the stream with its
type and message count, the sha256 of the bytes read, and what was written.
Run with `python3 -I`: the bag is untrusted input.
"""
import argparse
import json
import os
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import p5_ros1_stream_to_ros2 as p5  # noqa: E402

SIZE = {1: 1, 2: 1, 3: 2, 4: 2, 5: 4, 6: 4, 7: 4, 8: 8}  # sensor_msgs/PointField datatype -> bytes


class Converter(p5.Converter):
    def __init__(self, out, a):
        super().__init__(out, a.lidar, a.imu, None)
        self.a = a
        self.keep = a.keep.split(',') if a.keep else None
        self.seen = {}  # ROS 1 connection id -> [topic, type, count]
        self.pending_imu = []  # (stamp, serialized) from --imu-bag, sorted
        self.imu_i = 0
        self.imu_from_stream = a.imu_bag is None
        self.layout = None
        self.lidar_sources = set()

    def connection(self, header, data):
        cid = struct.unpack('<I', header['conn'])[0]
        if cid not in self.seen:
            self.seen[cid] = [header['topic'].decode(), p5.fields_of(data)['type'].decode(), 0]
        if header['topic'].decode() in (self.lidar, self.a.lidar_alt):
            self.lidar_sources.add(header['topic'].decode())
            header = dict(header, topic=self.lidar.encode())
        if header['topic'].decode() == self.imu and not self.imu_from_stream:
            return
        super().connection(header, data)

    def cloud(self, m):
        PF = self.dst.types['sensor_msgs/msg/PointField']
        n, step = m.width * m.height, m.point_step
        if m.row_step != m.width * step or m.is_bigendian:
            raise ValueError('padded rows or big-endian clouds are not supported')
        rows = np.asarray(m.data, dtype=np.uint8)[:n * step].reshape(n, step)
        if self.layout is None:
            self.layout = {'point_step': step, 'height': m.height, 'width': m.width, 'frame_id': m.header.frame_id,
                           'fields': [[f.name, f.offset, f.datatype, f.count] for f in m.fields]}
        fields = list(m.fields)
        if self.a.drop_zero:
            by = {f.name: f for f in fields}
            xyz = np.concatenate([rows[:, by[k].offset:by[k].offset + 4] for k in 'xyz'], axis=1)
            rows = rows[xyz.view('<f4').reshape(n, 3).any(axis=1)]
        if self.keep:
            by = {f.name: f for f in fields}
            parts, out_fields, off = [], [], 0
            for k in self.keep:
                f = by[k]
                size = SIZE[f.datatype] * f.count
                parts.append(rows[:, f.offset:f.offset + size])
                out_fields.append(PF(name=k, offset=off, datatype=f.datatype, count=f.count))
                off += size
            rows, step = np.concatenate(parts, axis=1), off
        else:
            out_fields = [PF(name=f.name, offset=f.offset, datatype=f.datatype, count=f.count) for f in fields]
        height, width = (1, len(rows)) if self.a.drop_zero else (m.height, m.width)
        st = self.stats[self.lidar]
        st['points_in'] = st.get('points_in', 0) + n
        st['points_out'] = st.get('points_out', 0) + len(rows)
        h = self.header2(m.header)
        if self.a.lidar_stamp_offset:
            ns = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec + round(self.a.lidar_stamp_offset * 1e9)
            h = self.dst.types['std_msgs/msg/Header'](
                stamp=self.dst.types['builtin_interfaces/msg/Time'](sec=ns // 1_000_000_000,
                                                                    nanosec=ns % 1_000_000_000),
                frame_id=m.header.frame_id)
        return self.dst.types['sensor_msgs/msg/PointCloud2'](
            header=h, height=height, width=width, fields=out_fields, is_bigendian=False, point_step=step,
            row_step=step * width, data=np.ascontiguousarray(rows).ravel(),
            is_dense=True if self.a.drop_zero else m.is_dense)

    def write(self, topic, stamp, raw):
        self.writer.write(self.out_conn[topic], stamp, raw)
        st = self.stats[topic]
        st['messages'] += 1
        st['first_stamp'] = min(st.get('first_stamp', stamp), stamp)
        st['last_stamp'] = max(st.get('last_stamp', stamp), stamp)

    def flush_imu(self, until):
        while self.imu_i < len(self.pending_imu) and self.pending_imu[self.imu_i][0] <= until:
            self.write(self.imu, *self.pending_imu[self.imu_i])
            self.imu_i += 1

    def message(self, header, data):
        cid = struct.unpack('<I', header['conn'])[0]
        self.seen[cid][2] += 1
        if cid not in self.conns:
            return
        topic, name = self.conns[cid]
        m = self.src.deserialize_ros1(data, name)
        out = self.cloud(m) if topic == self.lidar else self.imu_msg(m)
        stamp = out.header.stamp.sec * 1_000_000_000 + out.header.stamp.nanosec
        if topic == self.lidar:
            self.flush_imu(stamp)
        self.write(topic, stamp, self.dst.serialize_cdr(out, name))

    def read_imu_bag(self, path):
        """All messages of the IMU topic of a stored ROS 1 bag, as (stamp, CDR), sorted by stamp."""
        own_conns, self.conns = self.conns, {}
        out = []

        def take(h, d):
            cid = struct.unpack('<I', h['conn'])[0]
            if cid in self.conns:
                m = self.imu_msg(self.src.deserialize_ros1(d, self.conns[cid][1]))
                out.append((m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec,
                            self.dst.serialize_cdr(m, self.conns[cid][1])))

        self.imu_from_stream = True
        with open(path, 'rb') as f:
            for kind, h, d in bag_records(p5.Stream(f)):
                if kind == 'connection':
                    if h['topic'].decode() == self.imu:
                        p5.Converter.connection(self, h, d)
                else:
                    take(h, d)
        self.imu_from_stream = False
        self.conns = own_conns
        out.sort(key=lambda x: x[0])
        self.pending_imu = out


def bag_records(s):
    """('connection' | 'message', header, data) of a ROS 1 bag stream, chunks decompressed."""
    if s.read(len(p5.MAGIC)) != p5.MAGIC:
        sys.exit('not a ROS bag 2.0')
    while True:
        raw = s.read(4)
        if len(raw) < 4:
            return
        (hl,) = struct.unpack('<I', raw)
        header = p5.fields_of(s.read(hl))
        (dl,) = struct.unpack('<I', s.read(4))
        op = header['op'][0]
        if op == p5.OP_CHUNK:
            for h, d in p5.records(p5.decompress(header['compression'], s.read(dl))):
                if h['op'][0] == p5.OP_CONNECTION:
                    yield 'connection', h, d
                elif h['op'][0] == p5.OP_MSG:
                    yield 'message', h, d
        elif op == p5.OP_CONNECTION:
            yield 'connection', header, s.read(dl)
        else:
            s.read(dl)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('out')
    ap.add_argument('--lidar', required=True)
    ap.add_argument('--imu', required=True)
    ap.add_argument('--imu-bag')
    ap.add_argument('--lidar-alt')
    ap.add_argument('--lidar-stamp-offset', type=float, default=0.0)
    ap.add_argument('--keep')
    ap.add_argument('--drop-zero', action='store_true')
    ap.add_argument('--max-lidar', type=int)
    ap.add_argument('--summary')
    a = ap.parse_args()
    conv = Converter(a.out, a)
    if a.imu_bag:
        conv.read_imu_bag(a.imu_bag)
    s = p5.Stream(sys.stdin.buffer if a.bag == '-' else open(a.bag, 'rb'))
    complete = True
    for kind, h, d in bag_records(s):
        if kind == 'connection':
            conv.connection(h, d)
        else:
            conv.message(h, d)
            if a.max_lidar and conv.stats[a.lidar]['messages'] >= a.max_lidar:
                complete = False
                break
    if complete:
        s.drain()
        conv.flush_imu(1 << 62)
    conv.close()
    summary = {'bytes_read': s.count, 'sha256': s.sha.hexdigest() if complete else None, 'complete': complete,
               'written': conv.stats, 'lidar_layout': conv.layout, 'lidar_source_topics': sorted(conv.lidar_sources),
               'stream_topics': sorted([t, ty, c] for t, ty, c in conv.seen.values())}
    text = json.dumps(summary, indent=1)
    print(text)
    if a.summary:
        with open(a.summary, 'w') as f:
            f.write(text + '\n')


if __name__ == '__main__':
    main()
