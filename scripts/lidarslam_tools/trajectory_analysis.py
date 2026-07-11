"""Trajectory association, alignment, and error-series helpers."""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass
class Pose:
    t: float
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


def read_tum(path: Path) -> list[Pose]:
    poses: list[Pose] = []
    if not path.is_file():
        return poses
    with path.open("r", encoding="utf-8", errors="replace") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            try:
                poses.append(
                    Pose(
                        t=float(parts[0]),
                        x=float(parts[1]),
                        y=float(parts[2]),
                        z=float(parts[3]),
                        qx=float(parts[4]),
                        qy=float(parts[5]),
                        qz=float(parts[6]),
                        qw=float(parts[7]),
                    )
                )
            except Exception:
                continue
    poses.sort(key=lambda pose: pose.t)
    return poses


def associate_poses(ref: list[Pose], est: list[Pose], max_diff: float = 0.05) -> tuple[list[Pose], list[Pose]]:
    est_times = [pose.t for pose in est]
    ref_out: list[Pose] = []
    est_out: list[Pose] = []
    for ref_pose in ref:
        idx = bisect.bisect_left(est_times, ref_pose.t)
        candidates = []
        if idx < len(est_times):
            candidates.append(idx)
        if idx > 0:
            candidates.append(idx - 1)
        best_idx = None
        best_dt = None
        for cand in candidates:
            dt = abs(est_times[cand] - ref_pose.t)
            if dt <= max_diff and (best_dt is None or dt < best_dt):
                best_idx = cand
                best_dt = dt
        if best_idx is None:
            continue
        ref_out.append(ref_pose)
        est_out.append(est[best_idx])
    return ref_out, est_out


def quaternion_to_matrix(qx: float, qy: float, qz: float, qw: float) -> tuple[tuple[float, float, float], ...]:
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm == 0.0:
        return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    qx /= norm
    qy /= norm
    qz /= norm
    qw /= norm
    return (
        (
            1.0 - 2.0 * (qy * qy + qz * qz),
            2.0 * (qx * qy - qz * qw),
            2.0 * (qx * qz + qy * qw),
        ),
        (
            2.0 * (qx * qy + qz * qw),
            1.0 - 2.0 * (qx * qx + qz * qz),
            2.0 * (qy * qz - qx * qw),
        ),
        (
            2.0 * (qx * qz - qy * qw),
            2.0 * (qy * qz + qx * qw),
            1.0 - 2.0 * (qx * qx + qy * qy),
        ),
    )


def matmul_vec(rot: tuple[tuple[float, float, float], ...], vec: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = vec
    return (
        rot[0][0] * x + rot[0][1] * y + rot[0][2] * z,
        rot[1][0] * x + rot[1][1] * y + rot[1][2] * z,
        rot[2][0] * x + rot[2][1] * y + rot[2][2] * z,
    )


def matmul_mat(
    a: tuple[tuple[float, float, float], ...],
    b: tuple[tuple[float, float, float], ...],
) -> tuple[tuple[float, float, float], ...]:
    return (
        (
            a[0][0] * b[0][0] + a[0][1] * b[1][0] + a[0][2] * b[2][0],
            a[0][0] * b[0][1] + a[0][1] * b[1][1] + a[0][2] * b[2][1],
            a[0][0] * b[0][2] + a[0][1] * b[1][2] + a[0][2] * b[2][2],
        ),
        (
            a[1][0] * b[0][0] + a[1][1] * b[1][0] + a[1][2] * b[2][0],
            a[1][0] * b[0][1] + a[1][1] * b[1][1] + a[1][2] * b[2][1],
            a[1][0] * b[0][2] + a[1][1] * b[1][2] + a[1][2] * b[2][2],
        ),
        (
            a[2][0] * b[0][0] + a[2][1] * b[1][0] + a[2][2] * b[2][0],
            a[2][0] * b[0][1] + a[2][1] * b[1][1] + a[2][2] * b[2][1],
            a[2][0] * b[0][2] + a[2][1] * b[1][2] + a[2][2] * b[2][2],
        ),
    )


def rotation_to_rpy_deg(rot: tuple[tuple[float, float, float], ...]) -> tuple[float, float, float]:
    sy = math.sqrt(rot[0][0] * rot[0][0] + rot[1][0] * rot[1][0])
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(rot[2][1], rot[2][2])
        pitch = math.atan2(-rot[2][0], sy)
        yaw = math.atan2(rot[1][0], rot[0][0])
    else:
        roll = math.atan2(-rot[1][2], rot[1][1])
        pitch = math.atan2(-rot[2][0], sy)
        yaw = 0.0
    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


def unwrap_degrees(values: list[float]) -> list[float]:
    if not values:
        return []
    out = [values[0]]
    for value in values[1:]:
        adjusted = value
        prev = out[-1]
        while adjusted - prev > 180.0:
            adjusted -= 360.0
        while adjusted - prev < -180.0:
            adjusted += 360.0
        out.append(adjusted)
    return out


def estimate_alignment(
    ref_poses: list[Pose],
    est_poses: list[Pose],
) -> tuple[str, tuple[tuple[float, float, float], ...], tuple[float, float, float]]:
    identity = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    if len(ref_poses) < 2 or len(est_poses) < 2:
        dx = ref_poses[0].x - est_poses[0].x
        dy = ref_poses[0].y - est_poses[0].y
        dz = ref_poses[0].z - est_poses[0].z
        return "first_pose", identity, (dx, dy, dz)

    try:
        import numpy as np
    except Exception:
        dx = ref_poses[0].x - est_poses[0].x
        dy = ref_poses[0].y - est_poses[0].y
        dz = ref_poses[0].z - est_poses[0].z
        return "first_pose", identity, (dx, dy, dz)

    ref_xyz = np.asarray([(pose.x, pose.y, pose.z) for pose in ref_poses], dtype=float)
    est_xyz = np.asarray([(pose.x, pose.y, pose.z) for pose in est_poses], dtype=float)
    if ref_xyz.shape[0] < 3:
        dx = ref_poses[0].x - est_poses[0].x
        dy = ref_poses[0].y - est_poses[0].y
        dz = ref_poses[0].z - est_poses[0].z
        return "first_pose", identity, (dx, dy, dz)

    mu_ref = ref_xyz.mean(axis=0)
    mu_est = est_xyz.mean(axis=0)
    cov = (est_xyz - mu_est).T @ (ref_xyz - mu_ref) / ref_xyz.shape[0]
    try:
        u, _, vt = np.linalg.svd(cov)
    except Exception:
        dx = ref_poses[0].x - est_poses[0].x
        dy = ref_poses[0].y - est_poses[0].y
        dz = ref_poses[0].z - est_poses[0].z
        return "first_pose", identity, (dx, dy, dz)

    rot_np = vt.T @ u.T
    if np.linalg.det(rot_np) < 0:
        vt[-1, :] *= -1
        rot_np = vt.T @ u.T
    trans_np = mu_ref - rot_np @ mu_est
    rot = tuple(tuple(float(rot_np[r, c]) for c in range(3)) for r in range(3))
    trans = tuple(float(trans_np[i]) for i in range(3))
    return "se3_umeyama", rot, trans


def build_aligned_series(rec: Any) -> dict[str, Any] | None:
    if rec.lid_tum_path is None or rec.glim_traj_path is None:
        return None
    ref_all = read_tum(rec.glim_traj_path)
    est_all = read_tum(rec.lid_tum_path)
    if not ref_all or not est_all:
        return None

    ref, est = associate_poses(ref_all, est_all)
    if len(ref) < 2:
        return None

    alignment_name, rot_align, trans_align = estimate_alignment(ref, est)
    times: list[float] = []
    ref_xyz = {"x": [], "y": [], "z": []}
    est_xyz = {"x": [], "y": [], "z": []}
    ref_rpy = {"roll": [], "pitch": [], "yaw": []}
    est_rpy = {"roll": [], "pitch": [], "yaw": []}

    t0 = ref[0].t
    for ref_pose, est_pose in zip(ref, est):
        times.append(ref_pose.t - t0)

        ref_xyz["x"].append(ref_pose.x)
        ref_xyz["y"].append(ref_pose.y)
        ref_xyz["z"].append(ref_pose.z)

        aligned_xyz = matmul_vec(rot_align, (est_pose.x, est_pose.y, est_pose.z))
        est_xyz["x"].append(aligned_xyz[0] + trans_align[0])
        est_xyz["y"].append(aligned_xyz[1] + trans_align[1])
        est_xyz["z"].append(aligned_xyz[2] + trans_align[2])

        ref_rot = quaternion_to_matrix(ref_pose.qx, ref_pose.qy, ref_pose.qz, ref_pose.qw)
        est_rot = quaternion_to_matrix(est_pose.qx, est_pose.qy, est_pose.qz, est_pose.qw)
        aligned_rot = matmul_mat(rot_align, est_rot)
        ref_roll, ref_pitch, ref_yaw = rotation_to_rpy_deg(ref_rot)
        est_roll, est_pitch, est_yaw = rotation_to_rpy_deg(aligned_rot)
        ref_rpy["roll"].append(ref_roll)
        ref_rpy["pitch"].append(ref_pitch)
        ref_rpy["yaw"].append(ref_yaw)
        est_rpy["roll"].append(est_roll)
        est_rpy["pitch"].append(est_pitch)
        est_rpy["yaw"].append(est_yaw)

    for key in ("roll", "pitch", "yaw"):
        ref_rpy[key] = unwrap_degrees(ref_rpy[key])
        est_rpy[key] = unwrap_degrees(est_rpy[key])

    err_xyz = {
        axis: [est_value - ref_value for est_value, ref_value in zip(est_xyz[axis], ref_xyz[axis])]
        for axis in ("x", "y", "z")
    }
    err_rpy = {
        axis: [est_value - ref_value for est_value, ref_value in zip(est_rpy[axis], ref_rpy[axis])]
        for axis in ("roll", "pitch", "yaw")
    }
    err_norm = [
        math.sqrt(
            err_xyz["x"][idx] * err_xyz["x"][idx]
            + err_xyz["y"][idx] * err_xyz["y"][idx]
            + err_xyz["z"][idx] * err_xyz["z"][idx]
        )
        for idx in range(len(times))
    ]
    peak_idx = max(range(len(err_norm)), key=err_norm.__getitem__)

    return {
        "alignment": alignment_name,
        "pairs": len(times),
        "times": times,
        "ref_xyz": ref_xyz,
        "est_xyz": est_xyz,
        "ref_rpy": ref_rpy,
        "est_rpy": est_rpy,
        "err_xyz": err_xyz,
        "err_rpy": err_rpy,
        "err_norm": err_norm,
        "peak_idx": peak_idx,
        "peak_time": times[peak_idx],
        "peak_error": err_norm[peak_idx],
    }


