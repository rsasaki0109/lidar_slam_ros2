#!/usr/bin/env python3
"""Rebuild graph_based_slam's map.pcd (the 40 submap scans, one per 1.5 m, at their odometry poses,
VoxelGrid 0.1 m centroids) from a bag of /rko_lio/odometry + /rko_lio/frame, with the two placements of
frame_check.py: A = T_odom_base * p (as the backend does), B = T_odom_base * T_base_lidar * p.
Usage: build_maps_ab.py <bag_dir> <out_dir>
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import frame_check as fc  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from sensor_msgs.msg import PointCloud2  # noqa: E402
from sensor_msgs_py import point_cloud2  # noqa: E402


def voxel_centroids(p, leaf):
    keys = np.floor(p / leaf).astype(np.int64)
    _, inv, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    inv = inv.reshape(-1)
    sums = np.zeros((counts.size, 3))
    np.add.at(sums, inv, p)
    return sums / counts[:, None]


def write_pcd(path, pts):
    pts = pts.astype(np.float32)
    data = np.zeros((pts.shape[0], 4), dtype=np.float32)
    data[:, :3] = pts
    header = ('# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z intensity\n'
              'SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n'
              f'WIDTH {pts.shape[0]}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {pts.shape[0]}\nDATA binary\n')
    with open(path, 'wb') as f:
        f.write(header.encode())
        f.write(data.tobytes())


def main():
    bag, out = sys.argv[1], sys.argv[2]
    os.makedirs(out, exist_ok=True)
    odoms = {}
    for _, data, _ in fc.reader(bag, ['/rko_lio/odometry']):
        m = deserialize_message(data, Odometry)
        p, o = m.pose.pose.position, m.pose.pose.orientation
        odoms[fc.stamp(m.header)] = fc.se3(fc.rot(o.x, o.y, o.z, o.w), [p.x, p.y, p.z])
    clouds = {}
    for _, data, _ in fc.reader(bag, ['/rko_lio/frame']):
        m = deserialize_message(data, PointCloud2)
        clouds[fc.stamp(m.header)] = m
    chosen, last = [], None
    for s in sorted(t for t in odoms if t in clouds):
        pos = odoms[s][:3, 3]
        if last is None or np.linalg.norm(pos - last) >= 1.5:
            chosen.append(s)
            last = pos
    T_bl = fc.se3(fc.rot(fc.QX, fc.QY, fc.QZ, fc.QW), [fc.TX, fc.TY, fc.TZ])
    for name, extr in (('A', np.eye(4)), ('B', T_bl)):
        parts = []
        for s in chosen:
            pts = point_cloud2.read_points_numpy(clouds[s], field_names=('x', 'y', 'z'), skip_nans=True)
            parts.append(fc.place(odoms[s] @ extr, np.asarray(pts, dtype=np.float64).reshape(-1, 3)))
        merged = voxel_centroids(np.concatenate(parts), 0.1)
        write_pcd(os.path.join(out, f'map_{name}.pcd'), merged)
        print(name, len(chosen), 'scans', merged.shape[0], 'points')


if __name__ == '__main__':
    main()
