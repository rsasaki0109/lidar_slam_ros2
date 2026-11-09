#!/usr/bin/env python3
"""Copy a LiDAR and an IMU topic of a ROS 1 bag, read as a stream, into a ROS 2 bag (mcap), without the rest.

For holdout bags larger than the free disk (HILTI 2022 exp16: 15.5 GB): the bag is piped from the download and never
stored. Only sensor_msgs/PointCloud2 and sensor_msgs/Imu are copied; every other topic (the cameras) is skipped.

usage: curl -sSL URL | p5_ros1_stream_to_ros2.py - OUT_BAG --lidar /hesai/pandar --imu /alphasense/imu
                                                [--time-field time] [--summary OUT.json]

--time-field NAME appends a float32 field NAME to every point: the point's `timestamp` (float64, s) minus the cloud's
header stamp, for readers that need a per-point time relative to the header (FAST-LIO2's Velodyne input). The original
fields stay in front, so a reader that takes the first time-like field still reads `timestamp`. The summary holds the
sha256 of the bytes read, so the stream can be checked against the dataset's published hash.

Run with `python3 -I` from a directory other than the data's: the bag is untrusted input.
"""
import argparse
import bz2
import hashlib
import json
import struct
import sys

import numpy as np
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import get_types_from_msg, get_typestore, Stores

MAGIC = b'#ROSBAG V2.0\n'
OP_MSG, OP_BAG_HEADER, OP_INDEX, OP_CHUNK, OP_CHUNK_INFO, OP_CONNECTION = 0x02, 0x03, 0x04, 0x05, 0x06, 0x07


class Stream:
    """Exact reads from a byte stream, hashing every byte read."""

    def __init__(self, f):
        self.f, self.sha, self.count = f, hashlib.sha256(), 0

    def read(self, n):
        parts, left = [], n
        while left:
            b = self.f.read(min(left, 1 << 22))
            if not b:
                break
            parts.append(b)
            left -= len(b)
        out = b''.join(parts)
        self.sha.update(out)
        self.count += len(out)
        return out

    def drain(self):
        while True:
            b = self.f.read(1 << 22)
            if not b:
                return
            self.sha.update(b)
            self.count += len(b)


def fields_of(header):
    out, i = {}, 0
    while i < len(header):
        (n,) = struct.unpack_from('<I', header, i)
        name, _, value = header[i + 4:i + 4 + n].partition(b'=')
        out[name.decode()] = value
        i += 4 + n
    return out


def records(buf):
    """Records of an in-memory buffer (a chunk's content)."""
    i = 0
    while i < len(buf):
        (hl,) = struct.unpack_from('<I', buf, i)
        header = buf[i + 4:i + 4 + hl]
        (dl,) = struct.unpack_from('<I', buf, i + 4 + hl)
        data = buf[i + 8 + hl:i + 8 + hl + dl]
        i += 8 + hl + dl
        yield fields_of(header), data


def decompress(kind, data):
    if kind == b'none':
        return data
    if kind == b'bz2':
        return bz2.decompress(data)
    if kind == b'lz4':
        import lz4.frame
        return lz4.frame.decompress(data)
    raise ValueError(f'unknown chunk compression {kind!r}')


class Converter:
    def __init__(self, out, lidar, imu, time_field):
        self.lidar, self.imu, self.time_field = lidar, imu, time_field
        self.src = get_typestore(Stores.ROS1_NOETIC)
        self.dst = get_typestore(Stores.ROS2_JAZZY)
        self.writer = Writer(out, version=9, storage_plugin=StoragePlugin.MCAP)
        self.writer.open()
        self.conns = {}  # ROS 1 connection id -> (topic, ROS 2 connection) for the copied topics
        self.out_conn = {}
        self.stats = {lidar: {'messages': 0}, imu: {'messages': 0}}

    def connection(self, header, data):
        cid = struct.unpack('<I', header['conn'])[0]
        topic = header['topic'].decode()
        if topic not in (self.lidar, self.imu) or cid in self.conns:
            return
        info = fields_of(data)
        msgtype = info['type'].decode()
        want = 'sensor_msgs/PointCloud2' if topic == self.lidar else 'sensor_msgs/Imu'
        if msgtype != want:
            raise ValueError(f'{topic}: type {msgtype}, expected {want}')
        name = msgtype.replace('/', '/msg/')
        self.src.register(get_types_from_msg(info['message_definition'].decode(), name))
        if topic not in self.out_conn:
            self.out_conn[topic] = self.writer.add_connection(topic, name, typestore=self.dst)
        self.conns[cid] = (topic, name)

    def header2(self, h):
        return self.dst.types['std_msgs/msg/Header'](
            stamp=self.dst.types['builtin_interfaces/msg/Time'](sec=h.stamp.sec, nanosec=h.stamp.nanosec),
            frame_id=h.frame_id)

    def cloud(self, m):
        PF = self.dst.types['sensor_msgs/msg/PointField']
        fields = [PF(name=f.name, offset=f.offset, datatype=f.datatype, count=f.count) for f in m.fields]
        data, step = np.asarray(m.data, dtype=np.uint8), m.point_step
        if self.time_field:
            n = m.width * m.height
            ts = next(f for f in m.fields if f.name == 'timestamp')
            if ts.datatype != 8 or m.is_bigendian:  # float64, little endian
                raise ValueError('timestamp field is not little-endian float64')
            rows = data[:n * step].reshape(n, step) if m.row_step == m.width * step else None
            if rows is None:
                raise ValueError('padded rows are not supported')
            t = rows[:, ts.offset:ts.offset + 8].copy().view('<f8').ravel()
            rel = (t - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)).astype('<f4')
            data = np.concatenate([rows, rel.view(np.uint8).reshape(n, 4)], axis=1).ravel()
            fields.append(PF(name=self.time_field, offset=step, datatype=7, count=1))
            step += 4
            st = self.stats[self.lidar]
            st['rel_min'] = min(st.get('rel_min', np.inf), float(rel.min()))
            st['rel_max'] = max(st.get('rel_max', -np.inf), float(rel.max()))
        return self.dst.types['sensor_msgs/msg/PointCloud2'](
            header=self.header2(m.header), height=m.height, width=m.width, fields=fields,
            is_bigendian=m.is_bigendian, point_step=step, row_step=step * m.width, data=data, is_dense=m.is_dense)

    def imu_msg(self, m):
        T = self.dst.types
        v3 = T['geometry_msgs/msg/Vector3']
        return T['sensor_msgs/msg/Imu'](
            header=self.header2(m.header),
            orientation=T['geometry_msgs/msg/Quaternion'](
                x=m.orientation.x, y=m.orientation.y, z=m.orientation.z, w=m.orientation.w),
            orientation_covariance=m.orientation_covariance,
            angular_velocity=v3(x=m.angular_velocity.x, y=m.angular_velocity.y, z=m.angular_velocity.z),
            angular_velocity_covariance=m.angular_velocity_covariance,
            linear_acceleration=v3(x=m.linear_acceleration.x, y=m.linear_acceleration.y, z=m.linear_acceleration.z),
            linear_acceleration_covariance=m.linear_acceleration_covariance)

    def message(self, header, data):
        cid = struct.unpack('<I', header['conn'])[0]
        if cid not in self.conns:
            return
        topic, name = self.conns[cid]
        sec, nsec = struct.unpack('<II', header['time'])
        m = self.src.deserialize_ros1(data, name)
        out = self.cloud(m) if topic == self.lidar else self.imu_msg(m)
        stamp = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
        self.writer.write(self.out_conn[topic], stamp, self.dst.serialize_cdr(out, name))
        st = self.stats[topic]
        st['messages'] += 1
        st['first_stamp'] = min(st.get('first_stamp', stamp), stamp)
        st['last_stamp'] = max(st.get('last_stamp', stamp), stamp)
        st['bag_time_minus_stamp_max_s'] = max(st.get('bag_time_minus_stamp_max_s', -1e9),
                                               sec + nsec * 1e-9 - stamp * 1e-9)

    def close(self):
        self.writer.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag', help='ROS 1 bag file, or - for standard input')
    ap.add_argument('out', help='ROS 2 bag directory to create (mcap)')
    ap.add_argument('--lidar', required=True)
    ap.add_argument('--imu', required=True)
    ap.add_argument('--time-field')
    ap.add_argument('--summary')
    a = ap.parse_args()
    s = Stream(sys.stdin.buffer if a.bag == '-' else open(a.bag, 'rb'))
    if s.read(len(MAGIC)) != MAGIC:
        sys.exit('not a ROS bag 2.0')
    conv = Converter(a.out, a.lidar, a.imu, a.time_field)
    chunks = 0
    while True:
        raw = s.read(4)
        if len(raw) < 4:
            break
        (hl,) = struct.unpack('<I', raw)
        header = fields_of(s.read(hl))
        (dl,) = struct.unpack('<I', s.read(4))
        op = header['op'][0]
        if op == OP_CHUNK:
            chunks += 1
            for h, d in records(decompress(header['compression'], s.read(dl))):
                if h['op'][0] == OP_CONNECTION:
                    conv.connection(h, d)
                elif h['op'][0] == OP_MSG:
                    conv.message(h, d)
        elif op == OP_CONNECTION:
            conv.connection(header, s.read(dl))
        else:  # bag header, index data, chunk info: skipped
            s.read(dl)
    s.drain()
    conv.close()
    summary = {'bytes_read': s.count, 'sha256': s.sha.hexdigest(), 'chunks': chunks, 'topics': conv.stats}
    text = json.dumps(summary, indent=1)
    print(text)
    if a.summary:
        with open(a.summary, 'w') as f:
            f.write(text + '\n')


if __name__ == '__main__':
    main()
