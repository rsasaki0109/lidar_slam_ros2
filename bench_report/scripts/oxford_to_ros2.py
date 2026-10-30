#!/usr/bin/env python3
"""Oxford Spires raw LiDAR/IMU -> ROS 2 bag (sqlite3), without storing the 4 GB cloud archive.

Reads the zip central directory of raw/lidar-clouds.zip over HTTP range requests, then fetches every member
(one binary PCD per scan: x y z _ intensity _ timestamp ring _) the same way, checks its CRC-32, and writes
  /hesai/pandar                (sensor_msgs/PointCloud2, frame pandar): x y z intensity t, all float32, where t
                               is seconds since the header stamp, the earliest point time of the scan;
  /alphasense_driver_ros/imu   (sensor_msgs/Imu, frame imu_sensor_frame) from raw/imu.csv.
Every message is stored at the time it would arrive from a driver: a scan at its last point time, an IMU
sample at its stamp. Only LiDAR and IMU are read; no ground truth.
Usage: oxford_to_ros2.py <zip_url> <imu.csv> <out_bag_dir> [workers]
"""
import concurrent.futures
import csv
import os
import struct
import subprocess
import sys
import zlib

import numpy as np
import rclpy.serialization
import rosbag2_py
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Imu, PointCloud2, PointField

URL, IMU_CSV, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
WORKERS = int(sys.argv[4]) if len(sys.argv) > 4 else 8


def fetch(start, end):
    for _ in range(5):
        r = subprocess.run(['curl', '-sS', '-L', '--fail', '-m', '300', '-r', f'{start}-{end}', URL],
                           capture_output=True)
        if r.returncode == 0 and len(r.stdout) == end - start + 1:
            return r.stdout
    raise RuntimeError(f'range {start}-{end} failed: {r.stderr[-300:]}')


def archive_size():
    r = subprocess.run(['curl', '-sS', '-L', '-I', '-m', '120', URL], capture_output=True, text=True, check=True)
    sizes = [int(line.split(':', 1)[1]) for line in r.stdout.splitlines() if line.lower().startswith('content-length')]
    return sizes[-1]


def central_directory(size):
    tail = fetch(size - 65536, size - 1)
    i = tail.rfind(b'PK\x05\x06')
    _, _, _, _, entries, cd_size, cd_off, _ = struct.unpack('<IHHHHIIH', tail[i:i + 22])
    if 0xFFFFFFFF in (cd_size, cd_off) or entries == 0xFFFF:
        # ZIP64 (an archive past 4 GiB): the locator just before the end record gives the ZIP64 end record
        j = tail.rfind(b'PK\x06\x07', 0, i)
        z64_off = struct.unpack('<Q', tail[j + 8:j + 16])[0]
        z64 = fetch(z64_off, z64_off + 55)
        assert z64[:4] == b'PK\x06\x06', z64[:4]
        entries, cd_size, cd_off = struct.unpack('<QQQ', z64[32:56])
    cd = fetch(cd_off, cd_off + cd_size - 1)
    members, pos = [], 0
    while pos + 46 <= len(cd) and cd[pos:pos + 4] == b'PK\x01\x02':
        method = struct.unpack('<H', cd[pos + 10:pos + 12])[0]
        crc, csize, usize = struct.unpack('<III', cd[pos + 16:pos + 28])
        n, m, k = struct.unpack('<HHH', cd[pos + 28:pos + 34])
        offset = struct.unpack('<I', cd[pos + 42:pos + 46])[0]
        name = cd[pos + 46:pos + 46 + n].decode()
        # ZIP64 extra field (id 1): the 64-bit sizes and offset, in this order, of the fields set to 0xFFFFFFFF
        extra, e = cd[pos + 46 + n:pos + 46 + n + m], 0
        while e + 4 <= len(extra):
            eid, elen = struct.unpack('<HH', extra[e:e + 4])
            if eid == 1:
                vals, v = extra[e + 4:e + 4 + elen], 0
                if usize == 0xFFFFFFFF:
                    usize, v = struct.unpack('<Q', vals[v:v + 8])[0], v + 8
                if csize == 0xFFFFFFFF:
                    csize, v = struct.unpack('<Q', vals[v:v + 8])[0], v + 8
                if offset == 0xFFFFFFFF:
                    offset = struct.unpack('<Q', vals[v:v + 8])[0]
            e += 4 + elen
        if name.endswith('.pcd'):
            members.append((name, method, crc, csize, usize, offset))
        pos += 46 + n + m + k
    assert len(members) == entries - 1, (len(members), entries)
    return sorted(members, key=lambda e: e[0])


def load_member(member):
    name, method, crc, csize, usize, offset = member
    blob = fetch(offset, offset + 30 + 512 + csize - 1)
    n, m = struct.unpack('<HH', blob[26:30])
    start = 30 + n + m
    if start + csize > len(blob):
        blob = fetch(offset, offset + start + csize - 1)
    data = blob[start:start + csize]
    raw = zlib.decompress(data, -15) if method == 8 else data
    if len(raw) != usize or (zlib.crc32(raw) & 0xFFFFFFFF) != crc:
        raise RuntimeError(f'{name}: size or CRC-32 mismatch')
    return name, raw


def parse_pcd(raw):
    end = raw.index(b'\nDATA ') + 1
    header = {}
    for line in raw[:end].decode('latin1').splitlines():
        if line and not line.startswith('#'):
            key, *vals = line.split()
            header[key] = vals
    data_line_end = raw.index(b'\n', end)
    assert raw[end:data_line_end].split()[1] == b'binary', raw[end:data_line_end]
    kinds = {('F', '4'): 'f4', ('F', '8'): 'f8', ('U', '1'): 'u1', ('U', '2'): 'u2', ('U', '4'): 'u4'}
    fields, offset = [], 0
    for idx, (name, size, typ, count) in enumerate(zip(header['FIELDS'], header['SIZE'], header['TYPE'],
                                                      header['COUNT'])):
        dt = kinds[(typ, size)]
        label = name if name != '_' else f'_pad{idx}'
        fields.append((label, dt if count == '1' else (dt, int(count))))
    dtype = np.dtype(fields)
    points = int(header['POINTS'][0])
    return np.frombuffer(raw, dtype=dtype, count=points, offset=data_line_end + 1)


def stamp(ns):
    return Time(sec=int(ns // 1_000_000_000), nanosec=int(ns % 1_000_000_000))


def cloud_message(pcd):
    t_ns = np.round(pcd['timestamp'] * 1e9).astype(np.int64)
    t0 = int(t_ns.min())
    out = np.empty(len(pcd), dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('intensity', 'f4'), ('t', 'f4')])
    out['x'], out['y'], out['z'], out['intensity'] = pcd['x'], pcd['y'], pcd['z'], pcd['intensity']
    out['t'] = ((t_ns - t0) * 1e-9).astype(np.float32)
    msg = PointCloud2()
    msg.header.stamp = stamp(t0)
    msg.header.frame_id = 'pandar'
    msg.height, msg.width = 1, len(out)
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(('x', 'y', 'z', 'intensity', 't'))]
    msg.is_bigendian, msg.point_step = False, 20
    msg.row_step, msg.is_dense = 20 * len(out), True
    msg.data = out.tobytes()
    return msg, int(t_ns.max())


def imu_messages():
    with open(IMU_CSV) as f:
        for row in csv.DictReader(f):
            ns = int(row['secs']) * 1_000_000_000 + int(row['nsecs'])
            msg = Imu()
            msg.header.stamp = stamp(ns)
            msg.header.frame_id = 'imu_sensor_frame'
            msg.orientation_covariance[0] = -1.0
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = (
                float(row['acc_x']), float(row['acc_y']), float(row['acc_z']))
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (
                float(row['ang_vel_x']), float(row['ang_vel_y']), float(row['ang_vel_z']))
            yield ns, msg


def main():
    size = archive_size()
    members = central_directory(size)
    if os.environ.get('LIMIT'):
        members = members[:int(os.environ['LIMIT'])]
    print(f'archive {size} bytes, {len(members)} scans', flush=True)
    writer = rosbag2_py.SequentialWriter()
    # BAG_STORAGE=mcap writes a zstd-chunked MCAP (about 0.75x the sqlite3 size, read transparently).
    if os.environ.get('BAG_STORAGE') == 'mcap':
        storage = rosbag2_py.StorageOptions(uri=OUT, storage_id='mcap', storage_preset_profile='zstd_fast')
    else:
        storage = rosbag2_py.StorageOptions(uri=OUT, storage_id='sqlite3')
    writer.open(storage, rosbag2_py.ConverterOptions('', ''))
    for topic, kind in (('/hesai/pandar', 'sensor_msgs/msg/PointCloud2'),
                        ('/alphasense_driver_ros/imu', 'sensor_msgs/msg/Imu')):
        writer.create_topic(rosbag2_py.TopicMetadata(id=0, name=topic, type=kind, serialization_format='cdr'))
    imu = imu_messages()
    pending = next(imu, None)
    written_imu = written_scans = points = 0
    last_scan_end = -1
    with concurrent.futures.ThreadPoolExecutor(WORKERS) as pool:
        for begin in range(0, len(members), 4 * WORKERS):
            for name, raw in pool.map(load_member, members[begin:begin + 4 * WORKERS]):
                msg, end_ns = cloud_message(parse_pcd(raw))
                assert end_ns > last_scan_end, name
                last_scan_end = end_ns
                while pending is not None and pending[0] <= end_ns:
                    writer.write('/alphasense_driver_ros/imu', rclpy.serialization.serialize_message(pending[1]),
                                 pending[0])
                    written_imu += 1
                    pending = next(imu, None)
                writer.write('/hesai/pandar', rclpy.serialization.serialize_message(msg), end_ns)
                written_scans += 1
                points += msg.width
            print(f'{written_scans}/{len(members)} scans, {written_imu} imu', flush=True)
    while pending is not None:
        writer.write('/alphasense_driver_ros/imu', rclpy.serialization.serialize_message(pending[1]), pending[0])
        written_imu += 1
        pending = next(imu, None)
    del writer
    print(f'done: {written_scans} scans, {points} points, {written_imu} imu samples', flush=True)


if __name__ == '__main__':
    main()
