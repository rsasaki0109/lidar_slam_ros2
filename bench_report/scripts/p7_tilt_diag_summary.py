#!/usr/bin/env python3
"""Summary of rko_lio's per-scan attitude diagnostic (RKO_LIO_TILT_DIAG; report chapter 60.4).

usage: p7_tilt_diag_summary.py DIAG [DIAG ...]

DIAG: one line per scan, t, the registration's correction of the mean-rate prediction (rotation vector, world x y z),
the same for the sample-wise gyro integral, the Schur complement of the rotation block of the final ICP system
(xx yy zz xy xz yz, world frame, translation marginalized), the translation block's diagonal, and the mean gyro rate.
Prints, per file, the spread and the mean of the roll/pitch correction (deg per scan), how often it exceeds 0.05 deg,
and the tilt information (smallest eigenvalue of the xy block of the Schur complement) against the yaw information.
"""
import sys

import numpy as np

COLS = ['t', 'mx', 'my', 'mz', 'sx', 'sy', 'sz', 'Sxx', 'Syy', 'Szz', 'Sxy', 'Sxz', 'Syz', 'Hx', 'Hy', 'Hz', 'rate']


def load(path):
    a = np.loadtxt(path, ndmin=2)
    return {c: a[:, i] for i, c in enumerate(COLS)}


def tilt_information(d):
    """Smallest eigenvalue of the 2x2 roll/pitch block of the Schur complement, and its share of the yaw one."""
    a, b, c = d['Sxx'], d['Sxy'], d['Syy']
    lam_min = 0.5 * (a + c) - np.sqrt(0.25 * (a - c) ** 2 + b ** 2)
    return lam_min, lam_min / d['Szz']


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    print(f'{"file":40s} {"scans":>6s} {"rms_m":>7s} {"rms_s":>7s} {"mean_m":>7s} {"mean_s":>7s} '
          f'{">.05_m":>7s} {">.05_s":>7s} {"tilt_inf":>9s} {"tilt/yaw":>9s}')
    for path in sys.argv[1:]:
        d = load(path)
        deg = 180 / np.pi
        m = np.stack([d['mx'], d['my']], 1) * deg
        s = np.stack([d['sx'], d['sy']], 1) * deg
        ok = np.isfinite(s).all(1)
        lam, share = tilt_information(d)
        name = '/'.join(path.split('/')[-2:])
        print(f'{name:40s} {len(m):6d} {np.sqrt((m ** 2).sum(1).mean()):7.3f} '
              f'{np.sqrt((s[ok] ** 2).sum(1).mean()):7.3f} {np.linalg.norm(m.mean(0)):7.4f} '
              f'{np.linalg.norm(s[ok].mean(0)):7.4f} {np.mean(np.linalg.norm(m, axis=1) > 0.05):7.3f} '
              f'{np.mean(np.linalg.norm(s[ok], axis=1) > 0.05):7.3f} {np.median(lam):9.3f} {np.median(share):9.4f}')


if __name__ == '__main__':
    main()
