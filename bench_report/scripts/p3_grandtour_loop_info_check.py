#!/usr/bin/env python3
"""
Does the loop registration's Hessian point the right way? (Phase 3, robust loops, item 2.) For each loop edge of a
backend run with loop_edge_information.csv (SMALL_GICP), the translation covariance the Hessian predicts, in the first
submap's frame (R_Z Sigma_vv R_Z^T, Sigma the inverse of the [w; v] information), against the edge's translation error
from the reference (p3_grandtour_loop_errors.py). The Hessian's scale is not physical (small_gicp's covariances are
regularised shapes), so only its shape is read: the share of the squared error along each predicted principal
direction, against the share that direction has of the predicted variance.
For development missions only (reads the reference).
Usage: p3_grandtour_loop_info_check.py <mission> <backend dir> <frontend front_tum_0.txt> <extrinsics.txt>
Prints per scored edge: i, j, predicted sigma (unscaled, mm) and error (mm) along the strongest / middle / weakest
direction; then the totals.
"""
import sys

import numpy as np

sys.path.insert(0, '/home/user/lidar_slam_ros2/bench_report/scripts')
from p1_grandtour_to_ros2 import matrix  # noqa: E402
from p3_grandtour_loop_errors import edge_errors  # noqa: E402


def main():
    mission, backend, front_path, ext_path = sys.argv[1:5]
    info = {}
    for row in np.loadtxt(f'{backend}/loop_edge_information.csv', delimiter=',', skiprows=1, ndmin=2):
        if row[2] > 0:
            info[(int(row[0]), int(row[1]))] = row[3:39].reshape(6, 6)
    quat = {(int(e[0]), int(e[1])): e[6:10]
            for e in np.loadtxt(f'{backend}/loop_edges.csv', delimiter=',', skiprows=1, ndmin=2)}
    err_share, var_share = np.zeros(3), np.zeros(3)
    for i, j, _, _, length, _, loop_err, _ in edge_errors(mission, backend, front_path, ext_path):
        if loop_err is None or (i, j) not in info:
            continue
        sigma = np.linalg.inv(info[(i, j)])
        rz = matrix([0.0, 0.0, 0.0], quat[(i, j)])[:3, :3]
        cov = rz @ sigma[3:, 3:] @ rz.T
        lam, vec = np.linalg.eigh(cov)  # ascending: strongest direction first
        along = vec.T @ loop_err
        err_share += along ** 2 / max(loop_err @ loop_err, 1e-12)
        var_share += lam / lam.sum()
        print(f'{mission},{i},{j},length {length:.2f} m,sigma ' + ' '.join(f'{1e3 * np.sqrt(v):.3f}' for v in lam) +
              ' mm,error ' + ' '.join(f'{1e3 * abs(a):.1f}' for a in along) + f' mm,weakest axis {np.round(vec[:, 2], 2)}')
    if var_share.sum() > 0:
        n = var_share.sum()
        print(f'{mission},mean share of the squared error along strongest / middle / weakest: '
              + ' '.join(f'{v / n:.2f}' for v in err_share) + '; predicted variance share: '
              + ' '.join(f'{v / n:.2f}' for v in var_share))


if __name__ == '__main__':
    main()
