#!/usr/bin/env python3
"""Estimate a KITTI Velodyne's elevation offset from its scans alone (no ground truth).

Plan: data/kitti/front_diag_2026-11-01/plan_elevation.txt. A beam pointing delta lower than its calibration makes flat
ground look like a cone falling away from the sensor: z measured = z true - r tan(delta). On every STRIDE-th scan, the
ground along the car (|y| < 3 m, 5 m < |x| < 40 m, z within [-2.2, -1.2] m) is fitted by z = a + c x + b |x| with three
rounds of keeping the points within 0.15 m; b is the cone (|x| is even in the driving direction), c the tilt. A scan
counts if at least 300 points remain on each side. Prints delta = -atan(median b) in degrees and the spread.
Usage (ROS sourced): p2_kitti_elevation_offset.py <bag dir> [stride]
"""
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2


def scan_slope(pts):
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    m = (np.abs(y) < 3.0) & (np.abs(x) > 5.0) & (np.abs(x) < 40.0) & (z > -2.2) & (z < -1.2)
    x, z = x[m], z[m]
    keep = np.ones(len(x), bool)
    coef = None
    for _ in range(3):
        if keep.sum() < 20:
            return None
        A = np.c_[np.ones(keep.sum()), x[keep], np.abs(x[keep])]
        coef, *_ = np.linalg.lstsq(A, z[keep], rcond=None)
        res = z - np.c_[np.ones(len(x)), x, np.abs(x)] @ coef
        keep = np.abs(res) < 0.15
    if (keep & (x > 0)).sum() < 300 or (keep & (x < 0)).sum() < 300:
        return None
    return coef[2]


def main():
    bag = sys.argv[1]
    stride = int(sys.argv[2]) if len(sys.argv) > 2 else 5
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/kitti/velodyne/points']))
    slopes, i = [], -1
    while r.has_next():
        _, data, _ = r.read_next()
        i += 1
        if i % stride:
            continue
        m = deserialize_message(data, PointCloud2)
        pts = np.frombuffer(m.data, dtype=np.float32).reshape(-1, m.point_step // 4)[:, :3].astype(np.float64)
        b = scan_slope(pts[np.isfinite(pts).all(1)])
        if b is not None:
            slopes.append(b)
    s = np.array(slopes)
    delta = -np.degrees(np.arctan(np.median(s)))
    q = -np.degrees(np.arctan(np.percentile(s, [75, 25])))
    print(f'scans used {len(s)} of {i + 1} (every {stride}th); median slope {np.median(s):+.6f}; '
          f'delta {delta:+.4f} deg (interquartile {q[0]:+.4f} .. {q[1]:+.4f}); mean-based {-np.degrees(np.arctan(s.mean())):+.4f} deg')


if __name__ == '__main__':
    main()
