"""Position APE (rigid Umeyama, no scale) of a TUM trajectory against an LUH GT file."""
import sys
import numpy as np


def load_gt(path):
    rows = np.loadtxt(path, skiprows=1)
    stamps = rows[:, 0]
    # stamps are written with one decimal digit fused: 17410941326 -> 1741094132.6
    t = stamps / 10.0 if stamps[0] > 1e10 else stamps
    return t, rows[:, 1:4]


def umeyama(src, dst):
    ms, md = src.mean(0), dst.mean(0)
    u, s, vt = np.linalg.svd((dst - md).T @ (src - ms))
    d = np.eye(3)
    d[2, 2] = np.sign(np.linalg.det(u @ vt))
    r = u @ d @ vt
    return r, md - r @ ms


def main(est_path, gt_path):
    est = np.loadtxt(est_path)
    tg, pg = load_gt(gt_path)
    idx = np.searchsorted(tg, est[:, 0])
    pairs = []
    for i, j in enumerate(idx):
        near = [k for k in (j - 1, j) if 0 <= k < len(tg)]
        best = min(near, key=lambda k: abs(tg[k] - est[i, 0]))
        if abs(tg[best] - est[i, 0]) <= 0.05:
            pairs.append((i, best))
    a = est[[p[0] for p in pairs], 1:4]
    b = pg[[p[1] for p in pairs]]
    r, t = umeyama(a, b)
    err = np.linalg.norm((a @ r.T + t) - b, axis=1)
    path = np.linalg.norm(np.diff(b, axis=0), axis=1).sum()
    yaw = np.degrees(np.arctan2(r[1, 0], r[0, 0]))
    print(f'{est_path}: pairs {len(pairs)}/{len(est)} rmse {np.sqrt((err**2).mean()):.4f} '
          f'median {np.median(err):.4f} max {err.max():.4f} path {path:.1f} m yaw {yaw:.1f}')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
