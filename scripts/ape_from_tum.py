#!/usr/bin/env python3

import argparse
import bisect
import math
import statistics
from pathlib import Path


def read_tum(path: Path) -> list[tuple[float, tuple[float, float, float]]]:
    poses: list[tuple[float, tuple[float, float, float]]] = []
    with path.open("r", encoding="utf-8", errors="replace") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            try:
                ts = float(parts[0])
                xyz = (float(parts[1]), float(parts[2]), float(parts[3]))
            except Exception:
                continue
            poses.append((ts, xyz))
    poses.sort(key=lambda item: item[0])
    return poses


def associate(
    ref: list[tuple[float, tuple[float, float, float]]],
    est: list[tuple[float, tuple[float, float, float]]],
    max_diff: float,
) -> tuple[list[tuple[float, float, float]], list[tuple[float, float, float]]]:
    est_times = [ts for ts, _ in est]
    est_xyz = [xyz for _, xyz in est]
    ref_xyz_matched: list[tuple[float, float, float]] = []
    est_xyz_matched: list[tuple[float, float, float]] = []

    for ref_ts, ref_xyz in ref:
        idx = bisect.bisect_left(est_times, ref_ts)
        candidates = []
        if idx < len(est_times):
            candidates.append(idx)
        if idx > 0:
            candidates.append(idx - 1)
        best_idx = None
        best_dt = None
        for cand in candidates:
            dt = abs(est_times[cand] - ref_ts)
            if dt <= max_diff and (best_dt is None or dt < best_dt):
                best_idx = cand
                best_dt = dt
        if best_idx is None:
            continue
        ref_xyz_matched.append(ref_xyz)
        est_xyz_matched.append(est_xyz[best_idx])

    return ref_xyz_matched, est_xyz_matched


def align_first_pose(
    ref_xyz: list[tuple[float, float, float]],
    est_xyz: list[tuple[float, float, float]],
) -> list[tuple[float, float, float]]:
    dx = ref_xyz[0][0] - est_xyz[0][0]
    dy = ref_xyz[0][1] - est_xyz[0][1]
    dz = ref_xyz[0][2] - est_xyz[0][2]
    return [(x + dx, y + dy, z + dz) for x, y, z in est_xyz]


def try_align_umeyama(
    ref_xyz: list[tuple[float, float, float]],
    est_xyz: list[tuple[float, float, float]],
) -> tuple[str, list[tuple[float, float, float]]]:
    try:
        import numpy as np
    except Exception:
        return "first_pose", align_first_pose(ref_xyz, est_xyz)

    ref = np.asarray(ref_xyz, dtype=float)
    est = np.asarray(est_xyz, dtype=float)
    if ref.shape[0] < 3:
        return "first_pose", align_first_pose(ref_xyz, est_xyz)

    mu_ref = ref.mean(axis=0)
    mu_est = est.mean(axis=0)
    ref_centered = ref - mu_ref
    est_centered = est - mu_est
    cov = est_centered.T @ ref_centered / ref.shape[0]

    try:
        u, _, vt = np.linalg.svd(cov)
    except Exception:
        return "first_pose", align_first_pose(ref_xyz, est_xyz)

    rot = vt.T @ u.T
    if np.linalg.det(rot) < 0:
        vt[-1, :] *= -1
        rot = vt.T @ u.T
    trans = mu_ref - rot @ mu_est
    aligned = (rot @ est.T).T + trans
    return "se3_umeyama", [tuple(row.tolist()) for row in aligned]


def calc_errors(
    ref_xyz: list[tuple[float, float, float]],
    est_xyz: list[tuple[float, float, float]],
) -> list[float]:
    out: list[float] = []
    for (rx, ry, rz), (ex, ey, ez) in zip(ref_xyz, est_xyz):
        out.append(math.sqrt((rx - ex) ** 2 + (ry - ey) ** 2 + (rz - ez) ** 2))
    return out


def write_report(path: Path, errors: list[float], pairs: int, alignment: str) -> None:
    rmse = math.sqrt(sum(e * e for e in errors) / len(errors))
    mean = statistics.fmean(errors)
    median = statistics.median(errors)
    std = statistics.pstdev(errors) if len(errors) > 1 else 0.0
    min_v = min(errors)
    max_v = max(errors)

    lines = [
        "APE translation (m)",
        f"pairs: {pairs}",
        f"alignment: {alignment}",
        f"rmse: {rmse}",
        f"mean: {mean}",
        f"median: {median}",
        f"std: {std}",
        f"min: {min_v}",
        f"max: {max_v}",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser(description="Compute APE from two TUM trajectories.")
    ap.add_argument("--ref", required=True, help="Reference TUM trajectory")
    ap.add_argument("--est", required=True, help="Estimated TUM trajectory")
    ap.add_argument("--out", required=True, help="Output report path")
    ap.add_argument("--max-time-diff", type=float, default=0.05, help="Max timestamp association gap in seconds")
    args = ap.parse_args()

    ref = read_tum(Path(args.ref).expanduser().resolve())
    est = read_tum(Path(args.est).expanduser().resolve())
    if not ref or not est:
        return 1

    ref_xyz, est_xyz = associate(ref, est, args.max_time_diff)
    if len(ref_xyz) < 2:
        return 1

    alignment, aligned_est = try_align_umeyama(ref_xyz, est_xyz)
    errors = calc_errors(ref_xyz, aligned_est)
    write_report(Path(args.out).expanduser().resolve(), errors, len(errors), alignment)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
