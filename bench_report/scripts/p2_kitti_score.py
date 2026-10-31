#!/usr/bin/env python3
"""Score a trajectory on a KITTI Odometry training sequence made from its raw drive (kitti_raw_to_ros2.py).

The estimate is a TUM trajectory of the base frame (the OXTS IMU), stamped at the scans' header stamps. Both it and
the ground truth (dataset/poses/<seq>.txt, the left camera's poses) are put in the Velodyne frame:
  ground truth  T_w_velo = Tr^-1 T_w_cam0 Tr           (Tr: velodyne -> cam0, from the sequence's calib.txt)
  estimate      T_w_velo = T_w_base T_base_velo        (T_base_velo: extrinsic_lidar2base_quat_xyzw_xyz)
and matched frame by frame through the scans' header stamps (<bag>.frames.txt). A frame the estimate does not have is
left out and counted.
Prints one line: matched frames, APE RMSE (m, SE(3) Umeyama alignment of the positions), APE max (m), KITTI t_rel (%)
and r_rel (deg/m) over 100..800 m (scripts/kitti_metrics.py).
Usage: p2_kitti_score.py <est.tum> <poses.txt> <calib.txt> <frames.txt> <qx,qy,qz,qw,x,y,z> [out.json]
"""
import json
import sys

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, '/home/user/lidar_slam_ros2/scripts')
from kitti_metrics import compute_kitti_errors  # noqa: E402


def se3(q_xyzw, t):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(q_xyzw).as_matrix()
    T[:3, 3] = t
    return T


def main():
    est_path, poses_path, calib_path, frames_path, ext = sys.argv[1:6]
    out_json = sys.argv[6] if len(sys.argv) > 6 else None
    e = [float(v) for v in ext.split(',')]
    T_base_velo = se3(e[:4], e[4:])
    tr = next(line for line in open(calib_path) if line.startswith('Tr:'))
    Tr = np.eye(4)
    Tr[:3, :] = np.array([float(v) for v in tr.split()[1:]]).reshape(3, 4)
    gt_cam = np.loadtxt(poses_path).reshape(-1, 3, 4)
    frames = np.loadtxt(frames_path, dtype=np.int64)
    stamp_of = {int(f): int(t) for f, t, _ in frames}
    est = np.loadtxt(est_path)
    est_ns = np.round(est[:, 0] * 1e9).astype(np.int64)
    gt, es, missing = [], [], 0
    for f in range(len(gt_cam)):
        if f not in stamp_of:
            missing += 1
            continue
        j = np.searchsorted(est_ns, stamp_of[f])
        cand = [k for k in (j - 1, j) if 0 <= k < len(est_ns) and abs(est_ns[k] - stamp_of[f]) <= 1_000_000]
        if not cand:
            missing += 1
            continue
        k = cand[0]
        T_cam = np.eye(4)
        T_cam[:3, :] = gt_cam[f]
        gt.append(np.linalg.inv(Tr) @ T_cam @ Tr)
        es.append(se3(est[k, 4:8], est[k, 1:4]) @ T_base_velo)
    gt, es = np.array(gt), np.array(es)
    # SE(3) Umeyama alignment of the estimate's positions onto the ground truth's
    P, Q = es[:, :3, 3], gt[:, :3, 3]
    mp, mq = P.mean(0), Q.mean(0)
    U, _, Vt = np.linalg.svd((Q - mq).T @ (P - mp))
    D = np.eye(3)
    D[2, 2] = np.sign(np.linalg.det(U @ Vt))
    R = U @ D @ Vt
    err = np.linalg.norm((P - mp) @ R.T + mq - Q, axis=1)
    kitti = compute_kitti_errors(gt, es)
    res = dict(matched=len(gt), missing=missing, ape_rmse=float(np.sqrt(np.mean(err ** 2))), ape_max=float(err.max()),
               t_rel_percent=kitti['t_rel_percent_avg'], r_rel_deg_per_m=kitti['r_rel_deg_per_m_avg'])
    if out_json:
        json.dump(dict(res, per_length=kitti['per_length'], ape_per_frame=err.tolist()), open(out_json, 'w'))
    print(f"{res['matched']},{res['missing']},{res['ape_rmse']:.4f},{res['ape_max']:.4f},"
          f"{res['t_rel_percent']:.4f},{res['r_rel_deg_per_m']:.6f}")


if __name__ == '__main__':
    main()
