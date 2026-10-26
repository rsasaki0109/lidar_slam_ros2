#!/usr/bin/env python3
"""
Chapter 30.8: what the gravity window would measure if every orientation were the ground truth.

For each window (length --window s, every --step s) the raw accelerometer samples are rotated into the world frame
with the ground-truth orientation of the IMU at their time and averaged, as RKO-LIO's gravity window does with its
own orientations. With a perfect, unbiased accelerometer the mean points straight up; any tilt left is what the
accelerometer itself contributes (bias, misalignment). The azimuth of that tilt is compared with the heading.
Usage: p1_gravity_window_check.py <bag> <imu_topic> <gt_tum> <T_imu_gtframe 4x4 .npy or 'identity'> <out_csv>
       [--window 40] [--step 5] [--accel-scale 1.0]
The ground truth must give the pose of its body frame in a gravity-aligned world frame.
"""
import argparse
import csv

import numpy as np
import rclpy.serialization
import rosbag2_py
from sensor_msgs.msg import Imu


def quat_to_R(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def read_imu(bag, topic):
    reader = rosbag2_py.SequentialReader()
    storage = 'mcap' if any(p.endswith('.mcap') for p in __import__('os').listdir(bag)) else 'sqlite3'
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=storage), rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    t, a = [], []
    while reader.has_next():
        _, data, _ = reader.read_next()
        msg = rclpy.serialization.deserialize_message(data, Imu)
        t.append(msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec)
        a.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
    return np.array(t), np.array(a)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('imu_topic')
    ap.add_argument('gt')
    ap.add_argument('t_imu_gt')
    ap.add_argument('out')
    ap.add_argument('--window', type=float, default=40.0)
    ap.add_argument('--step', type=float, default=5.0)
    ap.add_argument('--accel-scale', type=float, default=1.0, help='e.g. 9.80665 for an IMU in g')
    args = ap.parse_args()

    t_imu, acc = read_imu(args.bag, args.imu_topic)
    acc = acc * args.accel_scale
    gt = np.loadtxt(args.gt)
    T_imu_gt = np.eye(4) if args.t_imu_gt == 'identity' else np.load(args.t_imu_gt)
    R_gt_imu = T_imu_gt[:3, :3].T  # rotation of the IMU frame expressed in the GT body frame
    keep = (t_imu >= gt[0, 0]) & (t_imu <= gt[-1, 0])
    t_imu, acc = t_imu[keep], acc[keep]
    idx = np.clip(np.searchsorted(gt[:, 0], t_imu), 1, len(gt) - 1)
    near = np.where(np.abs(gt[idx - 1, 0] - t_imu) < np.abs(gt[idx, 0] - t_imu), idx - 1, idx)
    ok = np.abs(gt[near, 0] - t_imu) < 0.1
    t_imu, acc, near = t_imu[ok], acc[ok], near[near >= 0][ok]
    R_world_body = np.array([quat_to_R(q) for q in gt[:, 4:8]])
    world_acc = np.einsum('nij,jk,nk->ni', R_world_body[near], R_gt_imu, acc)
    heading_vec = R_world_body[near][:, :, 0]  # body x axis in the world
    # A constant accelerometer bias in the IMU frame: the mean of the measurement minus the gravity the IMU should
    # feel at its ground-truth attitude (the motion averages out over the whole sequence).
    R_world_imu = np.einsum('nij,jk->nik', R_world_body[near], R_gt_imu)
    up = np.array([0.0, 0.0, 9.80665])
    bias = (acc - np.einsum('nji,j->ni', R_world_imu, up)).mean(0)
    world_acc_unbiased = np.einsum('nij,nj->ni', R_world_imu, acc - bias)

    def tilt_of(m):
        return np.degrees(np.arccos(np.clip(m[2] / np.linalg.norm(m), -1, 1)))

    rows = []
    start = t_imu[0]
    while start + args.window <= t_imu[-1]:
        sel = (t_imu >= start) & (t_imu < start + args.window)
        m = world_acc[sel].mean(0)
        azimuth = np.degrees(np.arctan2(m[1], m[0]))
        h = heading_vec[sel].mean(0)
        heading = np.degrees(np.arctan2(h[1], h[0]))
        rows.append([start - t_imu[0], tilt_of(m), azimuth, heading, np.linalg.norm(m),
                     tilt_of(world_acc_unbiased[sel].mean(0))])
        start += args.step
    with open(args.out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['window_start_s', 'tilt_deg', 'tilt_azimuth_deg', 'mean_heading_deg', 'mean_norm',
                    'tilt_without_bias_deg'])
        for r in rows:
            w.writerow([f'{v:.5f}' for v in r])
    r = np.array(rows)
    rel = np.radians((r[:, 2] - r[:, 3] + 180) % 360 - 180)
    horizontal = np.degrees(np.arctan2(np.linalg.norm(bias[:2]), 9.80665))
    print(f'windows {len(rows)}  tilt median {np.median(r[:, 1]):.3f} deg  p90 {np.percentile(r[:, 1], 90):.3f}  '
          f'max {r[:, 1].max():.3f}  (relevel threshold 0.29 deg)')
    print(f'azimuth-heading resultant {abs(np.mean(np.exp(1j * rel))):.2f}  world-azimuth resultant '
          f'{abs(np.mean(np.exp(1j * np.radians(r[:, 2])))):.2f}')
    print(f'IMU-frame bias {np.round(bias, 4)} m/s^2 (x,y part = {horizontal:.3f} deg of tilt)  '
          f'tilt without it: median {np.median(r[:, 5]):.3f} deg  max {r[:, 5].max():.3f}')


if __name__ == '__main__':
    main()
