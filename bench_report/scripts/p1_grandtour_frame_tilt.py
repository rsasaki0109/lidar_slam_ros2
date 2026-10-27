#!/usr/bin/env python3
"""
Tilt of a GrandTour trajectory's world frame from gravity (chapter 34).

The IMU's specific force (raw, gravity-inclusive) is rotated into the world frame with the trajectory's attitude
(T_world_imu = T_world_prism T_imu_prism^-1, slerp at the IMU stamps) and averaged over eight equal parts of the
mission; over tens of seconds the motion averages out and the mean points world-up, so its angle from +z is the frame's
tilt. Uses no reference.
Usage: p1_grandtour_frame_tilt.py <estimate.tum> <imu zarr group dir> <extrinsics.txt>
"""
import sys

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

import yaml
import zarr


def main():
    est = np.loadtxt(sys.argv[1])
    z = zarr.open(sys.argv[2], mode='r')
    imu = np.column_stack([z['timestamp'][:].astype(float), z['ang_vel'][:], z['lin_acc'][:]])
    q = yaml.safe_load(open(sys.argv[3]))['T_imu_prism_quat_xyzw_xyz']
    R_world_imu = Rotation.from_quat(est[:, 4:8]) * Rotation.from_quat(q[:4]).inv()
    t = est[:, 0]
    imu = imu[(imu[:, 0] >= t[0]) & (imu[:, 0] <= t[-1])]
    world_force = Slerp(t, R_world_imu)(imu[:, 0]).apply(imu[:, 4:7])
    edges = np.linspace(imu[0, 0], imu[-1, 0], 9)
    for a, b in zip(edges[:-1], edges[1:]):
        m = world_force[(imu[:, 0] >= a) & (imu[:, 0] < b)].mean(0)
        tilt = np.degrees(np.arctan2(np.hypot(m[0], m[1]), m[2]))
        print(f'{a - imu[0, 0]:6.0f}-{b - imu[0, 0]:4.0f} s  mean specific force {np.round(m, 3)}  tilt {tilt:5.2f} deg')


if __name__ == '__main__':
    main()
