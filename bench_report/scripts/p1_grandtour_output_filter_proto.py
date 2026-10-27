#!/usr/bin/env python3
"""
Offline prototype of the causal output position filter (chapter 34, screen_hs_f_2026-10-27/plan.txt).

Takes a GrandTour trajectory (T_world_prism) and the mission's IMU, and runs the filter that rko_lio's
output_position_filter.hpp implements: state = world position, velocity and acceleration bias of the IMU; each IMU
sample's specific force, rotated into the world frame with the trajectory's attitude (slerp between scans) and less
gravity, drives it; each scan's position is a measurement. The filtered IMU positions are moved back to the prism.
Gravity: (0, 0, 9.8107) after a static start (rko_lio's world); with --moving the mission's mean specific force stands
in for the re-estimated gravity of a moving start (it uses the whole mission, so it is not causal: a prototype only).
Prints the ATE without reference spikes and the band split (p1_grandtour_bands.py) for the raw and filtered outputs.
For development missions only (reads the reference).
Usage: p1_grandtour_output_filter_proto.py <estimate.tum> <imu zarr group dir> <extrinsics.txt> <mission> [--moving]
"""
import os
import sys
import tempfile

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

import yaml
import zarr

sys.path.insert(0, '/home/user/lidar_slam_ros2/bench_report/scripts')
from p1_grandtour_bands import bands  # noqa: E402
from p1_grandtour_time_offset import ate  # noqa: E402

SETTINGS = ((0.02, 0.001, 1e-4), (0.02, 0.001, 1e-3), (0.02, 0.003, 1e-3), (0.01, 0.003, 1e-3))


def run_filter(t, p_imu, imu, world_force, gravity, sig_r, q_a, q_b):
    x = np.zeros(9)
    x[:3] = p_imu[0]
    P = np.diag([sig_r ** 2] * 3 + [0.5 ** 2] * 3 + [0.05 ** 2] * 3)
    H = np.zeros((3, 9))
    H[:, :3] = np.eye(3)
    out = np.zeros_like(p_imu)
    out[0] = p_imu[0]
    j = 0
    for i in range(1, len(t)):
        t_prev = t[i - 1]
        while j < len(imu) and imu[j, 0] <= t[i]:
            dt = imu[j, 0] - t_prev
            if dt > 0:
                a = world_force[j] - gravity - x[6:9]
                F = np.eye(9)
                F[:3, 3:6] = np.eye(3) * dt
                F[:3, 6:9] = -0.5 * np.eye(3) * dt * dt
                F[3:6, 6:9] = -np.eye(3) * dt
                x[:3] += x[3:6] * dt + 0.5 * a * dt * dt
                x[3:6] += a * dt
                Q = np.zeros((9, 9))
                Q[:3, :3] = np.eye(3) * q_a * dt ** 3 / 3
                Q[:3, 3:6] = Q[3:6, :3] = np.eye(3) * q_a * dt ** 2 / 2
                Q[3:6, 3:6] = np.eye(3) * q_a * dt
                Q[6:9, 6:9] = np.eye(3) * q_b * dt
                P = F @ P @ F.T + Q
            t_prev = imu[j, 0]
            j += 1
        if t[i] > t_prev:
            x[:3] += x[3:6] * (t[i] - t_prev)
        K = P @ H.T @ np.linalg.inv(H @ P @ H.T + np.eye(3) * sig_r ** 2)
        x = x + K @ (p_imu[i] - x[:3])
        P = P - K @ H @ P
        out[i] = x[:3]
    return out


def main():
    est = np.loadtxt(sys.argv[1])
    z = zarr.open(sys.argv[2], mode='r')
    imu = np.column_stack([z['timestamp'][:].astype(float), z['ang_vel'][:], z['lin_acc'][:]])
    q = yaml.safe_load(open(sys.argv[3]))['T_imu_prism_quat_xyzw_xyz']
    short = sys.argv[4]
    moving = '--moving' in sys.argv[5:]
    t_imu_prism = np.array(q[4:])
    ref = zarr.open(f'/home/user/data/grandtour/reference_dev/{short}/prism_position', mode='r')
    rt = ref['timestamp'][:].astype(float)
    rp = ref['point'][:]
    t = est[:, 0]
    R_world_imu = Rotation.from_quat(est[:, 4:8]) * Rotation.from_quat(q[:4]).inv()
    p_imu = est[:, 1:4] - R_world_imu.apply(t_imu_prism)
    imu = imu[(imu[:, 0] >= t[0]) & (imu[:, 0] <= t[-1])]
    world_force = Slerp(t, R_world_imu)(imu[:, 0]).apply(imu[:, 4:7])
    gravity = world_force.mean(0) if moving else np.array([0.0, 0.0, 9.8107])

    def report(name, e):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, 'est.tum')
            np.savetxt(path, e)
            b = bands(path, short)
        print(f'{name:34s} ATE {ate(e, rt, rp, 0.0) * 1000:5.1f} mm  <1s {b[0]:4.1f}({b[1]:4.1f}) '
              f'1-10s {b[2]:4.1f}({b[3]:4.1f}) slow {b[4]:4.1f}', flush=True)

    report('raw', est)
    for sig_r, q_a, q_b in SETTINGS:
        filtered = est.copy()
        filtered[:, 1:4] = run_filter(t, p_imu, imu, world_force, gravity, sig_r, q_a, q_b) + \
            R_world_imu.apply(t_imu_prism)
        report(f'sigma {sig_r} q_a {q_a} q_b {q_b}', filtered)


if __name__ == '__main__':
    main()
