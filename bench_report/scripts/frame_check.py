#!/usr/bin/env python3
"""Which frame are the /rko_lio/frame points in?

Reads a bag with /rko_lio/odometry and /rko_lio/frame, picks the submap scans with graph_based_slam's
1.5 m rule, and measures how well scan pairs overlap when placed as
  A: T_odom_base * p                  (what graph_based_slam does today)
  B: T_odom_base * T_base_lidar * p   (points taken as LiDAR-frame points)
Usage: frame_check.py <bag_dir> <out.txt> [loop pairs as q:c,...]
"""
import sys

import numpy as np
from scipy.spatial import cKDTree

import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

# extrinsic_lidar2base_quat_xyzw_xyz of configs/hilti2022/rko_lio_hilti2022_pandar_competitive_v2.yaml
QX, QY, QZ, QW, TX, TY, TZ = 0.7071068, -0.7071068, 0.0, 0.0, -0.001, -0.00855, 0.055


def rot(x, y, z, w):
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def se3(r, t):
    m = np.eye(4)
    m[:3, :3] = r
    m[:3, 3] = t
    return m


def reader(bag, topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    while r.has_next():
        yield r.read_next()


def stamp(h):
    return h.stamp.sec * 1_000_000_000 + h.stamp.nanosec


def voxel(p, leaf=0.2):
    _, idx = np.unique(np.floor(p / leaf).astype(np.int64), axis=0, return_index=True)
    return p[np.sort(idx)]


def place(T, pts):
    return pts @ T[:3, :3].T + T[:3, 3]


def overlap(a, b):
    d, _ = cKDTree(a).query(b, k=1, distance_upper_bound=5.0)
    d = d[np.isfinite(d)]
    if d.size == 0:
        return float('nan'), 0.0, 0.0
    return float(np.median(d)), float(np.mean(d < 0.1)), float(np.mean(d < 0.3))


def main():
    bag, out = sys.argv[1], sys.argv[2]
    pairs = [tuple(int(v) for v in s.split(':')) for s in sys.argv[3].split(',')] if len(sys.argv) > 3 else []
    odoms = {}
    for _, data, _ in reader(bag, ['/rko_lio/odometry']):
        m = deserialize_message(data, Odometry)
        p, o = m.pose.pose.position, m.pose.pose.orientation
        odoms[stamp(m.header)] = se3(rot(o.x, o.y, o.z, o.w), [p.x, p.y, p.z])
    cloud_stamps = set()
    for _, data, _ in reader(bag, ['/rko_lio/frame']):
        cloud_stamps.add(stamp(deserialize_message(data, PointCloud2).header))
    paired = sorted(s for s in odoms if s in cloud_stamps)
    chosen, last = [], None
    for s in paired:
        pos = odoms[s][:3, 3]
        if last is None or np.linalg.norm(pos - last) >= 1.5:
            chosen.append(s)
            last = pos
    want = set(chosen)
    clouds = {}
    for _, data, _ in reader(bag, ['/rko_lio/frame']):
        m = deserialize_message(data, PointCloud2)
        s = stamp(m.header)
        if s in want:
            pts = point_cloud2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True)
            clouds[s] = np.asarray(pts, dtype=np.float64).reshape(-1, 3)
    T_bl = se3(rot(QX, QY, QZ, QW), [TX, TY, TZ])
    lines = [f'odometry {len(odoms)}, clouds {len(cloud_stamps)}, paired {len(paired)}, submaps {len(chosen)}',
             'pair | A: median NN m, <0.1 m, <0.3 m | B: median NN m, <0.1 m, <0.3 m']
    world = {'A': {}, 'B': {}}
    for i, s in enumerate(chosen):
        world['A'][i] = voxel(place(odoms[s], clouds[s]))
        world['B'][i] = voxel(place(odoms[s] @ T_bl, clouds[s]))
    todo = [(i, i + 1) for i in range(len(chosen) - 1)] + [(c, q) for q, c in pairs]
    summary = {'A': [], 'B': []}
    for i, j in todo:
        if i >= len(chosen) or j >= len(chosen):
            continue
        row = []
        for k in ('A', 'B'):
            med, f1, f3 = overlap(world[k][i], world[k][j])
            row.append(f'{med:.3f}, {f1:.2f}, {f3:.2f}')
            if j == i + 1:
                summary[k].append(med)
        tag = 'consecutive' if j == i + 1 else 'loop candidate'
        lines.append(f'{i}-{j} ({tag}) | {row[0]} | {row[1]}')
    for k in ('A', 'B'):
        lines.append(f'{k}: median over consecutive pairs of the median NN distance = {np.median(summary[k]):.3f} m')
    open(out, 'w').write('\n'.join(lines) + '\n')
    print('\n'.join(lines[:2] + lines[-2:]))


if __name__ == '__main__':
    main()
