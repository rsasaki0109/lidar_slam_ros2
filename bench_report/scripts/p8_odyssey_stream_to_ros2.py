#!/usr/bin/env python3
"""Odyssey (arXiv 2512.14428) drive, read as a tar stream, into a ROS 2 bag (mcap) with the LiDAR and the IMU only.

usage: curl -sSL URL | p8_odyssey_stream_to_ros2.py - OUT_BAG [--imu-file FILE] [--max-lidar N] [--summary OUT.json]
       curl -sSL URL | p8_odyssey_stream_to_ros2.py - --extract-imu FILE

A drive is one tar.gz: <seq>/ouster/<ns>.bin (one revolution: float32 x, y, z, t, reflectivity, near_ir per point,
128 x 2048 in beam order, NaN where there is no return; the file name is the revolution's Unix time in ns and t the ns
since its start), <seq>/m300/imu.txt (300 Hz, one line per sample: ns, orientation, angular velocity and linear
acceleration with variances), <seq>/refsys/ (the reference: never read here) and <seq>/metadata/ (never read here).
Only the names of the members are listed; the content of every member but ouster/*.bin and m300/imu.txt is skipped.

Written: /ouster/points (sensor_msgs/PointCloud2, header stamp = the file name, points without a return dropped,
fields x, y, z, t (uint32 ns), ring (uint16, the row of the 128 x 2048 grid), packed: 18 bytes a point, the least that
lets a 221 s drive fit this VM's /dev/shm; reflectivity and near_ir are not copied)
and /m300/imu (sensor_msgs/Imu, orientation not copied), in stamp order. The archive's members come in no time order,
so the kept points of each revolution are spooled to OUT_BAG.spool/ and the bag is written from there at the end
(each spool file is deleted as it is written). --imu-file gives the IMU file from a first pass with --extract-imu
(needed with --max-lidar when the IMU file comes after the clouds).

Run with `python3 -I` from a directory other than the data's: the archive is untrusted input. Nothing is extracted.
"""
import argparse
import hashlib
import io
import json
import os
import re
import sys
import tarfile

import numpy as np
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import Stores, get_typestore

LIDAR, IMU = '/ouster/points', '/m300/imu'
ROWS = 128
POINT = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('t', '<u4'), ('ring', '<u2')])  # 18 bytes


class Hashed:
    def __init__(self, f):
        self.f, self.sha, self.count = f, hashlib.sha256(), 0

    def read(self, n=-1):
        b = self.f.read(n)
        self.sha.update(b)
        self.count += len(b)
        return b


def imu_rows(data):
    a = np.loadtxt(io.BytesIO(data), dtype=np.float64, ndmin=2)
    if a.shape[1] != 20:
        raise ValueError(f'm300/imu.txt has {a.shape[1]} columns, expected 20')
    # the stamp is read again as an integer: float64 does not hold ns since 1970 exactly
    stamps = np.array([int(line.split(None, 1)[0]) for line in data.splitlines() if line.strip()], dtype=np.int64)
    order = np.argsort(stamps, kind='stable')
    return stamps[order], a[order, 8:11], a[order, 14:17]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('tar')
    ap.add_argument('out', nargs='?')
    ap.add_argument('--imu-file')
    ap.add_argument('--extract-imu')
    ap.add_argument('--max-lidar', type=int)
    ap.add_argument('--summary')
    a = ap.parse_args()
    src = Hashed(sys.stdin.buffer if a.tar == '-' else open(a.tar, 'rb'))
    tar = tarfile.open(fileobj=src, mode='r|*')
    T = get_typestore(Stores.ROS2_JAZZY).types
    store = get_typestore(Stores.ROS2_JAZZY)
    imu = imu_rows(open(a.imu_file, 'rb').read()) if a.imu_file else None
    writer = conn = None
    if a.out:
        writer = Writer(a.out, version=9, storage_plugin=StoragePlugin.MCAP)
        writer.open()
        conn = {LIDAR: writer.add_connection(LIDAR, 'sensor_msgs/msg/PointCloud2', typestore=store),
                IMU: writer.add_connection(IMU, 'sensor_msgs/msg/Imu', typestore=store)}
    stats = {LIDAR: {'messages': 0, 'points_in': 0, 'points_out': 0}, IMU: {'messages': 0}}
    members, other, imu_i, complete, layout, spooled = {}, [], 0, True, None, []
    spool = (a.out.rstrip('/') + '.spool') if a.out else None
    if spool:
        os.makedirs(spool)
    fields = [T['sensor_msgs/msg/PointField'](name=n, offset=POINT.fields[n][1], datatype=d, count=1)
              for n, d in (('x', 7), ('y', 7), ('z', 7), ('t', 6), ('ring', 4))]

    def header(ns, frame):
        return T['std_msgs/msg/Header'](stamp=T['builtin_interfaces/msg/Time'](sec=int(ns // 10**9),
                                                                              nanosec=int(ns % 10**9)), frame_id=frame)

    def write(topic, ns, msg, name):
        writer.write(conn[topic], int(ns), store.serialize_cdr(msg, name))
        st = stats[topic]
        st['messages'] += 1
        st.setdefault('first_stamp', int(ns))
        st['last_stamp'] = int(ns)

    def flush_imu(until):
        nonlocal imu_i
        stamps, gyro, acc = imu
        v3, cov = T['geometry_msgs/msg/Vector3'], np.zeros(9)
        while imu_i < len(stamps) and stamps[imu_i] <= until:
            g, c = gyro[imu_i], acc[imu_i]
            write(IMU, stamps[imu_i], T['sensor_msgs/msg/Imu'](
                header=header(stamps[imu_i], 'imu'),
                orientation=T['geometry_msgs/msg/Quaternion'](x=0.0, y=0.0, z=0.0, w=1.0), orientation_covariance=cov,
                angular_velocity=v3(x=g[0], y=g[1], z=g[2]), angular_velocity_covariance=cov,
                linear_acceleration=v3(x=c[0], y=c[1], z=c[2]), linear_acceleration_covariance=cov),
                'sensor_msgs/msg/Imu')
            imu_i += 1

    for m in tar:
        parts = m.name.strip('./').split('/')
        folder = parts[1] if len(parts) > 2 else '(top)'
        members[folder] = members.get(folder, 0) + (1 if m.isfile() else 0)
        if m.isfile() and folder != 'ouster':
            other.append([m.name, m.size])
        if not m.isfile():
            continue
        if folder == 'm300' and parts[-1] == 'imu.txt' and imu is None:
            data = tar.extractfile(m).read()
            if a.extract_imu:
                open(a.extract_imu, 'wb').write(data)
                complete = False
                break
            imu = imu_rows(data)
        elif folder == 'ouster' and re.fullmatch(r'\d+\.bin', parts[-1]) and writer:
            ns = int(parts[-1][:-4])
            p = np.frombuffer(tar.extractfile(m).read(), dtype='<f4').reshape(-1, 6)
            n = len(p)
            if n % ROWS:
                sys.exit(f'{m.name}: {n} points, not a multiple of {ROWS}')
            if layout is None:
                layout = {'points': n, 'columns': n // ROWS, 't_max_ns': float(np.nanmax(p[:, 3]))}
            keep = np.isfinite(p[:, :3]).all(axis=1)
            out = np.empty(int(keep.sum()), dtype=POINT)
            out['x'], out['y'], out['z'] = p[keep, 0], p[keep, 1], p[keep, 2]
            out['t'] = np.rint(p[keep, 3]).astype(np.uint32)
            out['ring'] = (np.nonzero(keep)[0] // (n // ROWS)).astype(np.uint16)
            # the archive's members are in no time order and the IMU file may come last: spool, then write in order
            out.tofile(os.path.join(spool, f'{ns}.pts'))
            spooled.append(ns)
            stats[LIDAR]['points_in'] += n
            if a.max_lidar and len(spooled) >= a.max_lidar:
                complete = False
                break
    if complete:
        while src.read(1 << 22):
            pass
    if writer:
        if imu is None:
            sys.exit('no m300/imu.txt before the end of the stream (with --max-lidar: give --imu-file)')
        for ns in sorted(spooled):
            path = os.path.join(spool, f'{ns}.pts')
            out = np.fromfile(path, dtype=POINT)
            os.remove(path)
            flush_imu(ns)
            write(LIDAR, ns, T['sensor_msgs/msg/PointCloud2'](
                header=header(ns, 'os_lidar'), height=1, width=len(out), fields=fields, is_bigendian=False,
                point_step=POINT.itemsize, row_step=POINT.itemsize * len(out), data=out.view(np.uint8).ravel(),
                is_dense=True), 'sensor_msgs/msg/PointCloud2')
            stats[LIDAR]['points_out'] += len(out)
        os.rmdir(spool)
        if complete:
            flush_imu(1 << 62)
    if writer:
        writer.close()
    summary = {'bytes_read': src.count, 'sha256': src.sha.hexdigest() if complete else None, 'complete': complete,
               'written': stats, 'lidar_layout': layout, 'members_per_folder': members, 'other_members': other}
    text = json.dumps(summary, indent=1)
    print(text)
    if a.summary:
        open(a.summary, 'w').write(text + '\n')


if __name__ == '__main__':
    main()
