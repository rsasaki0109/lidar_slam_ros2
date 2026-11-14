#!/usr/bin/env python3
"""GT-free scene statistics of a ROS 2 bag's LiDAR scans, for the choice between the two operating points
(bench_report/data/phase1_global/adaptive_point/plan.txt).

usage: p10_scan_stats.py BAG LIDAR_TOPIC [N=20] [STEP=25]  ->  one CSV line on stdout

Reads N scans, every STEP-th from the first. Per scan, over the points with 0.7 m < range < 100 m: the count, the
median and the 90 % range, and the number of occupied voxels at 1.5 m (operating point B's keypoints) and at 0.5625 m
(A's). Prints the medians over the scans:
scans,points,median_range,p90_range,voxels_1.5,voxels_0.5625
Run with `python3 -I` from another directory when the bag is untrusted input.
"""
import statistics
import sys

import numpy as np
from rosbags.highlevel import AnyReader
from pathlib import Path

DT = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}


def xyz(msg):
    off = {f.name: (f.offset, f.datatype) for f in msg.fields}
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(-1, msg.point_step)
    cols = []
    for k in 'xyz':
        o, t = off[k]
        d = np.dtype(DT[t]).newbyteorder('>' if msg.is_bigendian else '<')
        cols.append(raw[:, o:o + d.itemsize].copy().view(d)[:, 0].astype(np.float64))
    return np.stack(cols, 1)


def main():
    bag, topic = sys.argv[1:3]
    n = int(sys.argv[3]) if len(sys.argv) > 3 else 20
    step = int(sys.argv[4]) if len(sys.argv) > 4 else 25
    rows = []
    with AnyReader([Path(bag)]) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        for i, (conn, _, raw) in enumerate(reader.messages(connections=conns)):
            if i % step:
                continue
            p = xyz(reader.deserialize(raw, conn.msgtype))
            r = np.linalg.norm(p, axis=1)
            p, r = p[(r > 0.7) & (r < 100.0) & np.isfinite(r)], r[(r > 0.7) & (r < 100.0) & np.isfinite(r)]
            if len(p) == 0:
                continue
            vox = [len(np.unique(np.floor(p / v).astype(np.int64), axis=0)) for v in (1.5, 0.5625)]
            rows.append((len(p), float(np.median(r)), float(np.percentile(r, 90)), vox[0], vox[1]))
            if len(rows) >= n:
                break
    if not rows:
        sys.exit('no scans')
    m = [statistics.median(c) for c in zip(*rows)]
    print(f'{len(rows)},{m[0]:.0f},{m[1]:.2f},{m[2]:.2f},{m[3]:.0f},{m[4]:.0f}')


if __name__ == '__main__':
    main()
