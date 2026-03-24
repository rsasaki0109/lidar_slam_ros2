#!/usr/bin/env python3

from __future__ import annotations

import argparse
import bisect
import html
import json
import math
import re
import statistics
from collections import defaultdict
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any


LIDAR_COLOR = "#bc4b2f"
GLIM_COLOR = "#24573d"
GRID_COLOR = "rgba(32, 24, 21, 0.12)"


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


@dataclass
class RunRecord:
    group: str
    run: str
    metrics_path: Path
    bag: str
    bag_name: str
    ape_rmse: float | None
    ape_median: float | None
    ape_max: float | None
    lid_ok: bool
    glim_ok: bool
    lid_rtf: float | None
    glim_rtf: float | None
    lid_wall: float | None
    glim_wall: float | None
    reference_kind: str
    reference_source: str
    param_name: str
    lid_tum_path: Path | None
    glim_traj_path: Path | None
    mtime: float


def as_bool(value: Any) -> bool | None:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"1", "true", "yes", "on"}:
            return True
        if lowered in {"0", "false", "no", "off"}:
            return False
    return None


def as_float(value: Any) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except Exception:
        return None


def infer_reference_kind(source: Any, explicit_kind: Any) -> str:
    if explicit_kind:
        return str(explicit_kind)
    lowered = str(source or "").strip().lower()
    if "gt" in lowered or "ground_truth" in lowered:
        return "ground_truth"
    if "glim" in lowered or "cross" in lowered:
        return "cross_validation"
    return "-"


def fmt_float(value: float | None, digits: int = 3, suffix: str = "") -> str:
    if value is None or math.isnan(value):
        return "-"
    return f"{value:.{digits}f}{suffix}"


def fmt_ratio(num: int, den: int) -> str:
    if den == 0:
        return "0/0"
    pct = (100.0 * num) / den
    return f"{num}/{den} ({pct:.0f}%)"


def median(values: list[float]) -> float | None:
    if not values:
        return None
    return float(statistics.median(values))


def slugify(text: str) -> str:
    lowered = text.lower().strip()
    lowered = re.sub(r"[^a-z0-9]+", "-", lowered)
    return lowered.strip("-") or "plot"


def resolve_artifact_path(value: str | None, repo_root: Path) -> Path | None:
    if not value:
        return None
    path = Path(str(value)).expanduser()
    if not path.is_absolute():
        path = (repo_root / path).resolve()
    return path


def load_record(metrics_path: Path, output_root: Path, repo_root: Path) -> RunRecord | None:
    try:
        with metrics_path.open("r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception:
        return None

    rel = metrics_path.relative_to(output_root)
    parts = rel.parts
    if len(parts) < 2:
        return None
    group = parts[0]
    group_root = output_root / group
    rel_parent = metrics_path.parent.relative_to(group_root)
    run = rel_parent.as_posix() if str(rel_parent) != "." else group

    evo = data.get("evo") or {}
    ape = evo.get("ape") if isinstance(evo, dict) else None
    lid = data.get("lidarslam") or {}
    glim = data.get("glim") or {}
    reference = data.get("reference") or {}

    bag_path = str(data.get("bag_path") or "")
    bag_name = Path(bag_path).name if bag_path else "-"
    param_path = str(lid.get("param_path") or "")
    param_name = Path(param_path).name if param_path else "auto"

    reference_source = str(
        (
            reference.get("source")
            if isinstance(reference, dict) else None
        )
        or glim.get("reference_source")
        or "-"
    )
    reference_kind = infer_reference_kind(
        reference_source,
        reference.get("kind") if isinstance(reference, dict) else "",
    )

    return RunRecord(
        group=group,
        run=run,
        metrics_path=metrics_path,
        bag=bag_path,
        bag_name=bag_name,
        ape_rmse=as_float(ape.get("rmse")) if isinstance(ape, dict) else None,
        ape_median=as_float(ape.get("median")) if isinstance(ape, dict) else None,
        ape_max=as_float(ape.get("max")) if isinstance(ape, dict) else None,
        lid_ok=as_bool(lid.get("success")) is True,
        glim_ok=as_bool(glim.get("success")) is True,
        lid_rtf=as_float(lid.get("rtf")),
        glim_rtf=as_float(glim.get("rtf")),
        lid_wall=as_float(lid.get("wall_sec")),
        glim_wall=as_float(glim.get("wall_sec")),
        reference_kind=reference_kind,
        reference_source=reference_source,
        param_name=param_name,
        lid_tum_path=resolve_artifact_path(lid.get("tum_path"), repo_root),
        glim_traj_path=resolve_artifact_path(glim.get("traj_path"), repo_root),
        mtime=metrics_path.stat().st_mtime,
    )


def summarize_group(group: str, records: list[RunRecord], output_root: Path) -> dict[str, Any]:
    apes = [rec.ape_rmse for rec in records if rec.ape_rmse is not None]
    group_root = output_root / group
    return {
        "group": group,
        "records": sorted(records, key=lambda rec: (rec.ape_rmse is None, rec.ape_rmse or 999.0, rec.run)),
        "count": len(records),
        "best_ape": min(apes) if apes else None,
        "median_ape": median(apes),
        "lid_success": sum(1 for rec in records if rec.lid_ok),
        "glim_success": sum(1 for rec in records if rec.glim_ok),
        "good_runs": sum(1 for rec in records if run_quality(rec)[0] == "GOOD"),
        "unstable_runs": sum(1 for rec in records if run_quality(rec)[0] == "UNSTABLE"),
        "bad_runs": sum(1 for rec in records if run_quality(rec)[0] == "BAD"),
        "latest_mtime": max(rec.mtime for rec in records),
        "latest_iso": datetime.fromtimestamp(max(rec.mtime for rec in records)).strftime("%Y-%m-%d %H:%M:%S"),
        "summary_md": (group_root / "summary.md") if (group_root / "summary.md").is_file() else None,
        "summary_csv": (group_root / "summary.csv") if (group_root / "summary.csv").is_file() else None,
    }


def metric_link(path: Path, output_root: Path) -> str:
    return html.escape(path.relative_to(output_root).as_posix())


def badge(text: str, tone: str) -> str:
    return f"<span class='badge badge-{tone}'>{html.escape(text)}</span>"


def ape_spike_ratio(rec: RunRecord) -> float | None:
    if rec.ape_max is None or rec.ape_median is None:
        return None
    return rec.ape_max / max(rec.ape_median, 1e-3)


def is_spiky_run(rec: RunRecord) -> bool:
    ratio = ape_spike_ratio(rec)
    if ratio is None or rec.ape_max is None or rec.ape_median is None:
        return False
    if rec.ape_max >= 0.5 and ratio >= 8.0:
        return True
    return (rec.ape_max - rec.ape_median) >= 0.1 and ratio >= 4.0


def stability_markup(rec: RunRecord) -> str:
    ratio = ape_spike_ratio(rec)
    if ratio is None:
        return "<span class='muted'>-</span>"
    if is_spiky_run(rec):
        digits = 0 if ratio >= 10.0 else 1
        return badge(f"SPIKY x{ratio:.{digits}f}", "warn")
    return badge("STABLE", "good")


def run_quality(rec: RunRecord) -> tuple[str, str]:
    if not rec.lid_ok:
        return ("BAD", "bad")
    if rec.ape_rmse is None:
        return ("NO_APE", "warn")
    if rec.ape_rmse >= 0.10:
        return ("BAD", "bad")
    if rec.ape_max is not None and rec.ape_max >= 0.50:
        return ("BAD", "bad")
    if is_spiky_run(rec):
        return ("UNSTABLE", "warn")
    if rec.ape_rmse <= 0.03:
        return ("GOOD", "good")
    return ("UNSTABLE", "warn")


def quality_markup(rec: RunRecord) -> str:
    label, tone = run_quality(rec)
    return badge(label, tone)


def collect_log_alerts(run_dir: Path, limit: int = 6) -> list[dict[str, Any]]:
    patterns = [
        ("fatal", "bad", re.compile(r"(fatal|traceback|segmentation fault|core dumped|terminate called)", re.IGNORECASE)),
        ("pose_jump", "warn", re.compile(r"(POSE_JUMP|POSE_REJECT|POSE_STAMP_NONMONOTONIC)", re.IGNORECASE)),
        ("tf", "warn", re.compile(r"(TF_OLD_DATA|from the past for frame|extrapolation.+past)", re.IGNORECASE)),
        ("timeout", "warn", re.compile(r"(timeout|timed out|rc=124|exit status 124)", re.IGNORECASE)),
        ("ndt", "warn", re.compile(r"(ndt.+(warn|fail|error|degener|conver)|degeneracy|no transform available)", re.IGNORECASE)),
        ("drop", "warn", re.compile(r"(drop(ped)?|queue full|skipping message|discard)", re.IGNORECASE)),
        ("numeric", "bad", re.compile(r"(\bnan\b|\binf\b|invalid value)", re.IGNORECASE)),
    ]
    alerts: list[dict[str, Any]] = []
    seen: set[tuple[str, str]] = set()
    for log_path in sorted(run_dir.glob("*.log")):
        try:
            with log_path.open("r", encoding="utf-8", errors="replace") as f:
                for lineno, raw in enumerate(f, start=1):
                    line = " ".join(raw.strip().split())
                    if not line:
                        continue
                    for label, tone, pattern in patterns:
                        if not pattern.search(line):
                            continue
                        snippet = line[:220]
                        key = (label, snippet)
                        if key in seen:
                            break
                        seen.add(key)
                        alerts.append(
                            {
                                "label": label.upper(),
                                "tone": tone,
                                "file": log_path.name,
                                "line": lineno,
                                "snippet": snippet,
                            }
                        )
                        break
                    if len(alerts) >= limit:
                        return alerts
        except Exception:
            continue
    return alerts


def render_log_alerts(rec: RunRecord) -> str:
    alerts = collect_log_alerts(rec.metrics_path.parent)
    if not alerts:
        return "<p class='plot-meta'>no obvious TF / timeout / NDT warnings found in local logs.</p>"
    cards = []
    for alert in alerts:
        cards.append(
            "<div class='alert-card'>"
            f"{badge(str(alert['label']), str(alert['tone']))}"
            f"<span class='alert-src'>{html.escape(str(alert['file']))}:{alert['line']}</span>"
            f"<p>{html.escape(str(alert['snippet']))}</p>"
            "</div>"
        )
    return "<div class='alert-grid'>" + "".join(cards) + "</div>"


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


def build_aligned_series(rec: RunRecord) -> dict[str, Any] | None:
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


def svg_header(width: int, height: int) -> str:
    return (
        f"<svg viewBox='0 0 {width} {height}' width='100%' height='100%' "
        "xmlns='http://www.w3.org/2000/svg' preserveAspectRatio='xMidYMid meet'>"
    )


def line_chart_svg(
    times: list[float],
    ref_vals: list[float],
    est_vals: list[float],
    title: str,
    y_unit: str,
    width: int = 360,
    height: int = 180,
) -> str:
    if len(times) < 2:
        return "<div class='plot-empty'>not enough points</div>"

    left, right, top, bottom = 42, 14, 22, 26
    plot_w = width - left - right
    plot_h = height - top - bottom
    x_min = min(times)
    x_max = max(times)
    y_values = ref_vals + est_vals
    y_min = min(y_values)
    y_max = max(y_values)
    if math.isclose(x_min, x_max):
        x_max = x_min + 1.0
    if math.isclose(y_min, y_max):
        y_min -= 1.0
        y_max += 1.0
    pad = (y_max - y_min) * 0.08
    y_min -= pad
    y_max += pad

    def map_x(value: float) -> float:
        return left + ((value - x_min) / (x_max - x_min)) * plot_w

    def map_y(value: float) -> float:
        return top + plot_h - ((value - y_min) / (y_max - y_min)) * plot_h

    def polyline(timeseries: list[float], values: list[float], color: str) -> str:
        points = " ".join(f"{map_x(tx):.2f},{map_y(v):.2f}" for tx, v in zip(timeseries, values))
        return f"<polyline fill='none' stroke='{color}' stroke-width='2.1' points='{points}' />"

    grid = []
    for idx in range(5):
        frac = idx / 4.0
        y = top + frac * plot_h
        value = y_max - frac * (y_max - y_min)
        grid.append(
            f"<line x1='{left}' y1='{y:.2f}' x2='{left + plot_w}' y2='{y:.2f}' stroke='{GRID_COLOR}' stroke-width='1' />"
            f"<text x='{left - 6}' y='{y + 4:.2f}' text-anchor='end' class='axis'>{value:.2f}</text>"
        )
    grid.append(
        f"<text x='{left + plot_w}' y='{height - 6}' text-anchor='end' class='axis'>{x_max:.1f}s</text>"
    )
    grid.append(
        f"<text x='{left}' y='{height - 6}' text-anchor='start' class='axis'>{x_min:.1f}s</text>"
    )
    zero_line = ""
    if y_min <= 0.0 <= y_max:
        y0 = map_y(0.0)
        zero_line = (
            f"<line x1='{left}' y1='{y0:.2f}' x2='{left + plot_w}' y2='{y0:.2f}' "
            "stroke='rgba(32,24,21,0.24)' stroke-dasharray='4 4' stroke-width='1' />"
        )

    return (
        svg_header(width, height)
        + f"<text x='{left}' y='14' class='plot-title'>{html.escape(title)}</text>"
        + f"<text x='{width - right}' y='14' text-anchor='end' class='axis'>{html.escape(y_unit)}</text>"
        + "".join(grid)
        + zero_line
        + polyline(times, ref_vals, GLIM_COLOR)
        + polyline(times, est_vals, LIDAR_COLOR)
        + "</svg>"
    )


def diff_chart_svg(
    times: list[float],
    vals: list[float],
    title: str,
    y_unit: str,
    width: int = 360,
    height: int = 180,
    color: str = LIDAR_COLOR,
) -> str:
    if len(times) < 2 or len(vals) < 2:
        return "<div class='plot-empty'>not enough points</div>"

    left, right, top, bottom = 42, 14, 22, 26
    plot_w = width - left - right
    plot_h = height - top - bottom
    x_min = min(times)
    x_max = max(times)
    y_min = min(vals)
    y_max = max(vals)
    if math.isclose(x_min, x_max):
        x_max = x_min + 1.0
    if math.isclose(y_min, y_max):
        y_min -= 1.0
        y_max += 1.0
    pad = (y_max - y_min) * 0.08
    y_min -= pad
    y_max += pad

    def map_x(value: float) -> float:
        return left + ((value - x_min) / (x_max - x_min)) * plot_w

    def map_y(value: float) -> float:
        return top + plot_h - ((value - y_min) / (y_max - y_min)) * plot_h

    points = " ".join(f"{map_x(tx):.2f},{map_y(v):.2f}" for tx, v in zip(times, vals))
    grid = []
    for idx in range(5):
        frac = idx / 4.0
        y = top + frac * plot_h
        value = y_max - frac * (y_max - y_min)
        grid.append(
            f"<line x1='{left}' y1='{y:.2f}' x2='{left + plot_w}' y2='{y:.2f}' stroke='{GRID_COLOR}' stroke-width='1' />"
            f"<text x='{left - 6}' y='{y + 4:.2f}' text-anchor='end' class='axis'>{value:.2f}</text>"
        )
    grid.append(
        f"<text x='{left + plot_w}' y='{height - 6}' text-anchor='end' class='axis'>{x_max:.1f}s</text>"
    )
    grid.append(
        f"<text x='{left}' y='{height - 6}' text-anchor='start' class='axis'>{x_min:.1f}s</text>"
    )
    zero_line = ""
    if y_min <= 0.0 <= y_max:
        y0 = map_y(0.0)
        zero_line = (
            f"<line x1='{left}' y1='{y0:.2f}' x2='{left + plot_w}' y2='{y0:.2f}' "
            "stroke='rgba(32,24,21,0.24)' stroke-dasharray='4 4' stroke-width='1' />"
        )

    return (
        svg_header(width, height)
        + f"<text x='{left}' y='14' class='plot-title'>{html.escape(title)}</text>"
        + f"<text x='{width - right}' y='14' text-anchor='end' class='axis'>{html.escape(y_unit)}</text>"
        + "".join(grid)
        + zero_line
        + f"<polyline fill='none' stroke='{color}' stroke-width='2.1' points='{points}' />"
        + "</svg>"
    )


def xy_chart_svg(
    ref_x: list[float],
    ref_y: list[float],
    est_x: list[float],
    est_y: list[float],
    width: int = 720,
    height: int = 420,
) -> str:
    if len(ref_x) < 2 or len(est_x) < 2:
        return "<div class='plot-empty'>not enough points</div>"

    left, right, top, bottom = 42, 20, 20, 32
    plot_w = width - left - right
    plot_h = height - top - bottom
    x_all = ref_x + est_x
    y_all = ref_y + est_y
    x_min = min(x_all)
    x_max = max(x_all)
    y_min = min(y_all)
    y_max = max(y_all)
    span = max(x_max - x_min, y_max - y_min, 1e-6)
    x_mid = (x_min + x_max) * 0.5
    y_mid = (y_min + y_max) * 0.5
    x_min = x_mid - span * 0.55
    x_max = x_mid + span * 0.55
    y_min = y_mid - span * 0.55
    y_max = y_mid + span * 0.55

    def map_x(value: float) -> float:
        return left + ((value - x_min) / (x_max - x_min)) * plot_w

    def map_y(value: float) -> float:
        return top + plot_h - ((value - y_min) / (y_max - y_min)) * plot_h

    def polyline(xs: list[float], ys: list[float], color: str) -> str:
        points = " ".join(f"{map_x(px):.2f},{map_y(py):.2f}" for px, py in zip(xs, ys))
        return f"<polyline fill='none' stroke='{color}' stroke-width='2.2' points='{points}' />"

    grid = []
    for idx in range(5):
        frac = idx / 4.0
        x = left + frac * plot_w
        y = top + frac * plot_h
        grid.append(f"<line x1='{x:.2f}' y1='{top}' x2='{x:.2f}' y2='{top + plot_h}' stroke='{GRID_COLOR}' stroke-width='1' />")
        grid.append(f"<line x1='{left}' y1='{y:.2f}' x2='{left + plot_w}' y2='{y:.2f}' stroke='{GRID_COLOR}' stroke-width='1' />")
    start_ref = f"<circle cx='{map_x(ref_x[0]):.2f}' cy='{map_y(ref_y[0]):.2f}' r='4.5' fill='{GLIM_COLOR}' />"
    start_est = f"<circle cx='{map_x(est_x[0]):.2f}' cy='{map_y(est_y[0]):.2f}' r='4.5' fill='{LIDAR_COLOR}' />"
    return (
        svg_header(width, height)
        + f"<text x='{left}' y='14' class='plot-title'>XY Trajectory Overlay</text>"
        + f"<text x='{left}' y='{height - 8}' class='axis'>x [{x_min:.2f}, {x_max:.2f}] m</text>"
        + f"<text x='{width - right}' y='{height - 8}' text-anchor='end' class='axis'>y [{y_min:.2f}, {y_max:.2f}] m</text>"
        + "".join(grid)
        + polyline(ref_x, ref_y, GLIM_COLOR)
        + polyline(est_x, est_y, LIDAR_COLOR)
        + start_ref
        + start_est
        + "</svg>"
    )


def plotly_3d_chart(
    series: dict[str, Any],
    plot_id: str,
) -> str:
    data = [
        {
            "type": "scatter3d",
            "mode": "lines",
            "name": "GLIM",
            "x": series["ref_xyz"]["x"],
            "y": series["ref_xyz"]["y"],
            "z": series["ref_xyz"]["z"],
            "line": {"color": GLIM_COLOR, "width": 4},
        },
        {
            "type": "scatter3d",
            "mode": "lines",
            "name": "lidarslam",
            "x": series["est_xyz"]["x"],
            "y": series["est_xyz"]["y"],
            "z": series["est_xyz"]["z"],
            "line": {"color": LIDAR_COLOR, "width": 4},
        },
    ]
    layout = {
        "margin": {"l": 0, "r": 0, "t": 34, "b": 0},
        "paper_bgcolor": "rgba(255,255,255,0)",
        "plot_bgcolor": "rgba(255,255,255,0)",
        "legend": {
            "orientation": "h",
            "x": 0.0,
            "y": 1.08,
            "bgcolor": "rgba(255,255,255,0.0)",
        },
        "scene": {
            "xaxis": {"title": "X [m]", "gridcolor": GRID_COLOR, "zerolinecolor": GRID_COLOR},
            "yaxis": {"title": "Y [m]", "gridcolor": GRID_COLOR, "zerolinecolor": GRID_COLOR},
            "zaxis": {"title": "Z [m]", "gridcolor": GRID_COLOR, "zerolinecolor": GRID_COLOR},
            "aspectmode": "data",
            "dragmode": "orbit",
        },
    }
    config = {"responsive": True, "displaylogo": False, "scrollZoom": True}
    div_id = f"{plot_id}-3d"
    return (
        f"<div class='plot-card plotly-card'><div id='{div_id}' class='plotly-3d'></div></div>"
        f"<script>Plotly.newPlot({json.dumps(div_id)}, {json.dumps(data)}, {json.dumps(layout)}, {json.dumps(config)});</script>"
    )


def plot_bundle(rec: RunRecord, output_root: Path, open_default: bool) -> str:
    series = build_aligned_series(rec)
    plot_id = slugify(f"{rec.group}-{rec.run}")
    if series is None:
        return (
            f"<details class='plot-detail' id='{plot_id}'{' open' if open_default else ''}>"
            f"<summary>{html.escape(rec.run)} plots</summary>"
            "<p class='muted'>trajectory artifacts are missing for this run.</p>"
            "</details>"
        )

    xy_svg = xy_chart_svg(
        series["ref_xyz"]["x"],
        series["ref_xyz"]["y"],
        series["est_xyz"]["x"],
        series["est_xyz"]["y"],
    )
    xyz_3d_plot = plotly_3d_chart(series, plot_id)
    axis_plots = []
    for axis in ("x", "y", "z"):
        axis_plots.append(
            line_chart_svg(
                series["times"],
                series["ref_xyz"][axis],
                series["est_xyz"][axis],
                f"{axis.upper()} over Time",
                "m",
            )
        )
    for axis, label in (("roll", "Roll"), ("pitch", "Pitch"), ("yaw", "Yaw")):
        axis_plots.append(
            line_chart_svg(
                series["times"],
                series["ref_rpy"][axis],
                series["est_rpy"][axis],
                f"{label} over Time",
                "deg",
            )
        )

    error_plots = [
        diff_chart_svg(
            series["times"],
            series["err_norm"],
            "Position Error Norm",
            "m",
            color=GLIM_COLOR,
        )
    ]
    for axis in ("x", "y", "z"):
        error_plots.append(
            diff_chart_svg(
                series["times"],
                series["err_xyz"][axis],
                f"{axis.upper()} Error vs GLIM",
                "m",
            )
        )
    for axis, label in (("roll", "Roll"), ("pitch", "Pitch"), ("yaw", "Yaw")):
        error_plots.append(
            diff_chart_svg(
                series["times"],
                series["err_rpy"][axis],
                f"{label} Error vs GLIM",
                "deg",
            )
        )

    window_radius = min(max(len(series["times"]) // 12, 20), 80)
    peak_start = max(0, series["peak_idx"] - window_radius)
    peak_stop = min(len(series["times"]), series["peak_idx"] + window_radius + 1)
    zoom_xy_svg = xy_chart_svg(
        series["ref_xyz"]["x"][peak_start:peak_stop],
        series["ref_xyz"]["y"][peak_start:peak_stop],
        series["est_xyz"]["x"][peak_start:peak_stop],
        series["est_xyz"]["y"][peak_start:peak_stop],
        width=520,
        height=280,
    )
    zoom_err_svg = diff_chart_svg(
        series["times"][peak_start:peak_stop],
        series["err_norm"][peak_start:peak_stop],
        f"Position Error near Peak ({series['peak_time']:.1f}s)",
        "m",
        width=520,
        height=280,
        color=GLIM_COLOR,
    )

    spike_ratio = ape_spike_ratio(rec)
    spike_text = f" · spike x{spike_ratio:.1f}" if spike_ratio is not None else ""
    detail_line = (
        f"<p class='plot-meta'>pairs {series['pairs']} · alignment {html.escape(series['alignment'])} · "
        f"APE rmse {fmt_float(rec.ape_rmse, 3, ' m')} · "
        f"median {fmt_float(rec.ape_median, 3, ' m')} · "
        f"max {fmt_float(rec.ape_max, 3, ' m')}{spike_text} · "
        f"<a href='{metric_link(rec.metrics_path, output_root)}'>metrics.json</a></p>"
    )
    return (
        f"<details class='plot-detail' id='{plot_id}'{' open' if open_default else ''}>"
        f"<summary>{html.escape(rec.run)} trajectory plots</summary>"
        "<div class='legend'>"
        f"<span><i style='background:{GLIM_COLOR}'></i>GLIM</span>"
        f"<span><i style='background:{LIDAR_COLOR}'></i>lidarslam</span>"
        "</div>"
        + detail_line
        + "<p class='plot-subhead'>Interactive 3D XYZ</p>"
        + "<div class='plot-grid'>"
        + xyz_3d_plot
        + "</div>"
        + "<p class='plot-subhead'>XY Overlay</p>"
        + f"<div class='xy-wrap'>{xy_svg}</div>"
        + "<p class='plot-subhead'>XYZRPY Time Series</p>"
        + "<div class='plot-grid'>"
        + "".join(f"<div class='plot-card'>{svg}</div>" for svg in axis_plots)
        + "</div>"
        + "<p class='plot-subhead'>Error vs GLIM</p>"
        + "<div class='plot-grid'>"
        + "".join(f"<div class='plot-card'>{svg}</div>" for svg in error_plots)
        + "</div>"
        + "<p class='plot-subhead'>Peak Window</p>"
        + f"<p class='plot-meta'>largest translation error {fmt_float(series['peak_error'], 3, ' m')} at t={series['peak_time']:.2f}s</p>"
        + "<div class='plot-grid'>"
        + f"<div class='plot-card'>{zoom_xy_svg}</div>"
        + f"<div class='plot-card'>{zoom_err_svg}</div>"
        + "</div>"
        + "<p class='plot-subhead'>Observed Warnings</p>"
        + render_log_alerts(rec)
        + "</details>"
    )


def run_row(rec: RunRecord, output_root: Path) -> str:
    ape_rmse = fmt_float(rec.ape_rmse, 3, " m")
    ape_median = fmt_float(rec.ape_median, 3, " m")
    ape_max = fmt_float(rec.ape_max, 3, " m")
    ape_width = 0.0
    if rec.ape_rmse is not None:
        ape_width = min(100.0, (rec.ape_rmse / 0.05) * 100.0)
    anchor = slugify(f"{rec.group}-{rec.run}")
    return (
        "<tr>"
        f"<td><a href='#{anchor}'>{html.escape(rec.run)}</a><div class='mini-link'><a href='{metric_link(rec.metrics_path, output_root)}'>metrics.json</a></div></td>"
        f"<td>{html.escape(rec.bag_name)}</td>"
        f"<td>{quality_markup(rec)}</td>"
        f"<td class='metric-cell'><span>{ape_rmse}</span><div class='bar'><span style='width:{ape_width:.1f}%'></span></div></td>"
        f"<td>{ape_median}</td>"
        f"<td>{ape_max}</td>"
        f"<td>{stability_markup(rec)}</td>"
        f"<td>{fmt_float(rec.lid_rtf, 3)}</td>"
        f"<td>{fmt_float(rec.glim_rtf, 3)}</td>"
        f"<td>{badge('OK' if rec.lid_ok else 'FAIL', 'good' if rec.lid_ok else 'bad')}</td>"
        f"<td>{badge('OK' if rec.glim_ok else 'FAIL', 'good' if rec.glim_ok else 'bad')}</td>"
        f"<td>{html.escape(rec.reference_kind)}</td>"
        f"<td>{html.escape(rec.reference_source)}</td>"
        f"<td>{html.escape(rec.param_name)}</td>"
        "</tr>"
    )


def section(summary: dict[str, Any], output_root: Path) -> str:
    records = summary["records"]
    links = []
    if summary["summary_md"] is not None:
        links.append(f"<a href='{metric_link(summary['summary_md'], output_root)}'>summary.md</a>")
    if summary["summary_csv"] is not None:
        links.append(f"<a href='{metric_link(summary['summary_csv'], output_root)}'>summary.csv</a>")
    links_html = " ".join(links) if links else "<span class='muted'>no aggregate files</span>"
    rows = "\n".join(run_row(rec, output_root) for rec in records)
    plots = "\n".join(plot_bundle(rec, output_root, open_default=(idx == 0)) for idx, rec in enumerate(records))
    return f"""
    <section class="panel">
      <div class="panel-head">
        <div>
          <p class="eyebrow">Experiment Group</p>
          <h2>{html.escape(summary['group'])}</h2>
        </div>
        <div class="meta">
          <span>{links_html}</span>
          <span class="muted">updated {html.escape(summary['latest_iso'])}</span>
        </div>
      </div>
      <div class="stats">
        <div class="stat"><span class="label">Runs</span><strong>{summary['count']}</strong></div>
        <div class="stat"><span class="label">Best APE</span><strong>{fmt_float(summary['best_ape'], 3, ' m')}</strong></div>
        <div class="stat"><span class="label">Median APE</span><strong>{fmt_float(summary['median_ape'], 3, ' m')}</strong></div>
        <div class="stat"><span class="label">Good</span><strong>{summary['good_runs']}</strong></div>
        <div class="stat"><span class="label">Unstable</span><strong>{summary['unstable_runs']}</strong></div>
        <div class="stat"><span class="label">Bad</span><strong>{summary['bad_runs']}</strong></div>
        <div class="stat"><span class="label">lidarslam</span><strong>{fmt_ratio(summary['lid_success'], summary['count'])}</strong></div>
        <div class="stat"><span class="label">GLIM</span><strong>{fmt_ratio(summary['glim_success'], summary['count'])}</strong></div>
      </div>
      <div class="table-wrap">
        <table>
          <thead>
            <tr>
              <th>Run</th>
              <th>Bag</th>
              <th>Quality</th>
              <th>APE RMSE</th>
              <th>APE Med</th>
              <th>APE Max</th>
              <th>Stability</th>
              <th>Lidar RTF</th>
              <th>GLIM RTF</th>
              <th>Lidar</th>
              <th>GLIM</th>
              <th>Ref Kind</th>
              <th>Ref Src</th>
              <th>Params</th>
            </tr>
          </thead>
          <tbody>
            {rows}
          </tbody>
        </table>
      </div>
      <div class="plot-stack">
        {plots}
      </div>
    </section>
    """


def build_page(output_root: Path, groups: list[dict[str, Any]]) -> str:
    highlights = {}
    for prefix in ("ape_cycle_", "crossbag_ape_", "tight_tune_"):
        matches = [group for group in groups if group["group"].startswith(prefix)]
        if matches:
            highlights[prefix] = max(matches, key=lambda group: group["latest_mtime"])

    all_apes = [
        rec.ape_rmse
        for group in groups
        for rec in group["records"]
        if rec.ape_rmse is not None
    ]
    fresh_glim = sum(
        1
        for group in groups
        for rec in group["records"]
        if rec.glim_ok and rec.reference_source == "fresh"
    )
    total_runs = sum(group["count"] for group in groups)
    best_run = None
    for group in groups:
        for rec in group["records"]:
            if rec.ape_rmse is None:
                continue
            if best_run is None or rec.ape_rmse < best_run.ape_rmse:
                best_run = rec

    hero_cards = [
        ("Best APE", fmt_float(best_run.ape_rmse if best_run else None, 3, " m"), best_run.run if best_run else "-"),
        ("Median APE", fmt_float(median([v for v in all_apes if v is not None]), 3, " m"), f"{len(all_apes)} runs"),
        ("Fresh GLIM", str(fresh_glim), f"{total_runs} total runs"),
    ]

    spotlight = []
    labels = {
        "ape_cycle_": "Latest Auto Cycle",
        "crossbag_ape_": "Cross-Bag Check",
        "tight_tune_": "Tuned Param Check",
    }
    for prefix, label in labels.items():
        summary = highlights.get(prefix)
        if summary is None:
            continue
        spotlight.append(
            f"""
            <article class="spotlight">
              <p class="eyebrow">{label}</p>
              <h3>{html.escape(summary['group'])}</h3>
              <p class="spot-value">{fmt_float(summary['best_ape'], 3, ' m')}</p>
              <p class="muted">median {fmt_float(summary['median_ape'], 3, ' m')} · GLIM {fmt_ratio(summary['glim_success'], summary['count'])}</p>
            </article>
            """
        )

    sections_html = "\n".join(section(group, output_root) for group in groups[:8])
    generated_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>SLAM Experiment Report</title>
  <script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>
  <style>
    :root {{
      --bg: #f3ede1;
      --ink: #201815;
      --muted: #6d6058;
      --panel: rgba(255, 251, 245, 0.82);
      --line: rgba(32, 24, 21, 0.12);
      --accent: {LIDAR_COLOR};
      --good: {GLIM_COLOR};
      --good-bg: #d7efdf;
      --warn: #8b5b0c;
      --warn-bg: #f6e3bb;
      --bad: #8d2f24;
      --bad-bg: #f7d8d3;
      --shadow: 0 20px 60px rgba(62, 42, 24, 0.12);
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      color: var(--ink);
      background:
        radial-gradient(circle at top left, rgba(188, 75, 47, 0.18), transparent 30%),
        radial-gradient(circle at top right, rgba(36, 87, 61, 0.12), transparent 25%),
        linear-gradient(180deg, #f7f2e9 0%, var(--bg) 100%);
      font-family: "Iowan Old Style", "Palatino Linotype", "Book Antiqua", Palatino, "Noto Serif JP", serif;
      line-height: 1.45;
    }}
    a {{ color: var(--accent); text-decoration: none; }}
    a:hover {{ text-decoration: underline; }}
    code {{
      padding: 0.1rem 0.35rem;
      border-radius: 999px;
      background: rgba(255, 255, 255, 0.66);
      font-family: "IBM Plex Mono", "SFMono-Regular", Consolas, monospace;
      font-size: 0.9em;
    }}
    .page {{
      width: min(1240px, calc(100% - 32px));
      margin: 0 auto;
      padding: 32px 0 56px;
    }}
    .hero {{
      padding: 28px;
      border: 1px solid var(--line);
      border-radius: 28px;
      background: linear-gradient(145deg, rgba(255,255,255,0.72), rgba(255,248,240,0.86));
      box-shadow: var(--shadow);
    }}
    .eyebrow {{
      margin: 0 0 10px;
      color: var(--muted);
      text-transform: uppercase;
      letter-spacing: 0.14em;
      font-size: 0.72rem;
    }}
    h1, h2, h3 {{
      margin: 0;
      font-weight: 700;
    }}
    h1 {{
      font-size: clamp(2rem, 4vw, 3.6rem);
      line-height: 0.98;
      max-width: 10ch;
    }}
    .sub {{
      margin: 14px 0 0;
      color: var(--muted);
      max-width: 72ch;
    }}
    .hero-grid {{
      display: grid;
      grid-template-columns: 1.2fr 1fr;
      gap: 20px;
      margin-top: 26px;
    }}
    .hero-cards, .spotlights {{
      display: grid;
      gap: 16px;
    }}
    .hero-cards {{
      grid-template-columns: repeat(3, minmax(0, 1fr));
    }}
    .card, .spotlight, .panel {{
      border: 1px solid var(--line);
      border-radius: 24px;
      background: rgba(255, 252, 247, 0.84);
      backdrop-filter: blur(14px);
    }}
    .card {{
      padding: 18px;
    }}
    .card strong {{
      display: block;
      font-size: 1.5rem;
      margin-bottom: 4px;
    }}
    .card span, .muted {{
      color: var(--muted);
    }}
    .spotlights {{
      grid-template-columns: repeat(3, minmax(0, 1fr));
    }}
    .spotlight {{
      padding: 18px;
    }}
    .spot-value {{
      font-size: 1.9rem;
      margin: 14px 0 6px;
      color: var(--accent);
    }}
    .panel {{
      margin-top: 24px;
      padding: 22px;
      box-shadow: var(--shadow);
    }}
    .panel-head {{
      display: flex;
      gap: 16px;
      justify-content: space-between;
      align-items: start;
      margin-bottom: 18px;
    }}
    .panel-head h2 {{
      font-size: 1.5rem;
    }}
    .meta {{
      display: flex;
      flex-wrap: wrap;
      justify-content: end;
      gap: 12px;
      font-size: 0.92rem;
    }}
    .stats {{
      display: grid;
      grid-template-columns: repeat(7, minmax(0, 1fr));
      gap: 12px;
      margin-bottom: 18px;
    }}
    .stat {{
      padding: 14px 16px;
      border: 1px solid var(--line);
      border-radius: 18px;
      background: rgba(255,255,255,0.45);
    }}
    .label {{
      display: block;
      color: var(--muted);
      font-size: 0.82rem;
      margin-bottom: 6px;
    }}
    .stat strong {{
      font-size: 1.05rem;
    }}
    .table-wrap {{
      overflow-x: auto;
    }}
    table {{
      width: 100%;
      border-collapse: collapse;
      min-width: 1180px;
      font-family: "IBM Plex Mono", "SFMono-Regular", Consolas, monospace;
      font-size: 0.86rem;
    }}
    th, td {{
      text-align: left;
      padding: 12px 10px;
      border-bottom: 1px solid var(--line);
      vertical-align: middle;
    }}
    th {{
      color: var(--muted);
      font-weight: 600;
      font-size: 0.78rem;
      text-transform: uppercase;
      letter-spacing: 0.08em;
    }}
    .badge {{
      display: inline-flex;
      align-items: center;
      border-radius: 999px;
      padding: 4px 9px;
      font-size: 0.74rem;
      font-weight: 700;
      letter-spacing: 0.06em;
      text-transform: uppercase;
    }}
    .badge-good {{
      color: var(--good);
      background: var(--good-bg);
    }}
    .badge-bad {{
      color: var(--bad);
      background: var(--bad-bg);
    }}
    .badge-warn {{
      color: var(--warn);
      background: var(--warn-bg);
    }}
    .metric-cell {{
      min-width: 170px;
    }}
    .bar {{
      height: 6px;
      margin-top: 6px;
      border-radius: 999px;
      background: rgba(188, 75, 47, 0.1);
      overflow: hidden;
    }}
    .bar span {{
      display: block;
      height: 100%;
      border-radius: 999px;
      background: linear-gradient(90deg, #d17d64, var(--accent));
    }}
    .mini-link {{
      margin-top: 4px;
      font-size: 0.75rem;
    }}
    .plot-stack {{
      display: grid;
      gap: 14px;
      margin-top: 18px;
    }}
    .plot-detail {{
      border: 1px solid var(--line);
      border-radius: 20px;
      background: rgba(255,255,255,0.5);
      overflow: hidden;
    }}
    .plot-detail summary {{
      cursor: pointer;
      list-style: none;
      padding: 16px 18px;
      font-weight: 700;
    }}
    .plot-detail summary::-webkit-details-marker {{
      display: none;
    }}
    .plot-detail[open] summary {{
      border-bottom: 1px solid var(--line);
      background: rgba(255,255,255,0.35);
    }}
    .legend {{
      display: flex;
      gap: 14px;
      padding: 14px 18px 0;
      color: var(--muted);
      font-size: 0.88rem;
    }}
    .legend span {{
      display: inline-flex;
      align-items: center;
      gap: 8px;
    }}
    .legend i {{
      width: 12px;
      height: 12px;
      border-radius: 999px;
      display: inline-block;
    }}
    .plot-meta {{
      margin: 8px 18px 0;
      color: var(--muted);
      font-size: 0.9rem;
    }}
    .plot-subhead {{
      margin: 14px 18px 0;
      color: var(--muted);
      font-size: 0.78rem;
      font-weight: 700;
      letter-spacing: 0.08em;
      text-transform: uppercase;
      font-family: "IBM Plex Mono", "SFMono-Regular", Consolas, monospace;
    }}
    .alert-grid {{
      display: grid;
      gap: 10px;
      padding: 12px 18px 18px;
    }}
    .alert-card {{
      border: 1px solid var(--line);
      border-radius: 14px;
      background: rgba(255,255,255,0.72);
      padding: 12px 14px;
    }}
    .alert-src {{
      margin-left: 8px;
      color: var(--muted);
      font-size: 0.82rem;
      font-family: "IBM Plex Mono", "SFMono-Regular", Consolas, monospace;
    }}
    .alert-card p {{
      margin: 8px 0 0;
      color: var(--ink);
      font-family: "IBM Plex Mono", "SFMono-Regular", Consolas, monospace;
      font-size: 0.82rem;
      line-height: 1.45;
    }}
    .xy-wrap {{
      padding: 10px 18px 0;
    }}
    .plot-grid {{
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 12px;
      padding: 12px 18px 18px;
    }}
    .plot-card {{
      border: 1px solid var(--line);
      border-radius: 16px;
      background: rgba(255,255,255,0.78);
      overflow: hidden;
      min-height: 196px;
    }}
    .plotly-card {{
      min-height: 420px;
    }}
    .plotly-3d {{
      width: 100%;
      height: 420px;
    }}
    .plot-card svg, .xy-wrap svg {{
      display: block;
    }}
    .plot-title {{
      fill: var(--ink);
      font-size: 12px;
      font-weight: 700;
      font-family: "IBM Plex Mono", "SFMono-Regular", Consolas, monospace;
    }}
    .axis {{
      fill: var(--muted);
      font-size: 10px;
      font-family: "IBM Plex Mono", "SFMono-Regular", Consolas, monospace;
    }}
    .plot-empty {{
      padding: 18px;
      color: var(--muted);
      font-family: "IBM Plex Mono", "SFMono-Regular", Consolas, monospace;
    }}
    footer {{
      margin-top: 18px;
      color: var(--muted);
      font-size: 0.9rem;
    }}
    @media (max-width: 920px) {{
      .hero-grid, .hero-cards, .spotlights, .stats, .plot-grid {{
        grid-template-columns: 1fr;
      }}
      .panel-head {{
        flex-direction: column;
      }}
      .meta {{
        justify-content: start;
      }}
    }}
  </style>
</head>
<body>
  <div class="page">
    <section class="hero">
      <p class="eyebrow">SLAM Experiment Report</p>
      <h1>Trajectory overlays and axis-by-axis drift, side by side.</h1>
      <p class="sub">Generated from <code>output/**/metrics.json</code> and the paired trajectory files. Each run includes an interactive <code>3D XYZ</code> trajectory view, an <code>XY</code> overlay, <code>X/Y/Z/Roll/Pitch/Yaw</code> time-series plots, signed error traces against GLIM, an auto-extracted peak window, and local log warnings so bad runs are easier to classify and debug from one page.</p>
      <div class="hero-grid">
        <div class="hero-cards">
          {''.join(f"<article class='card'><span>{html.escape(title)}</span><strong>{html.escape(value)}</strong><span>{html.escape(note)}</span></article>" for title, value, note in hero_cards)}
        </div>
        <div class="spotlights">
          {''.join(spotlight)}
        </div>
      </div>
    </section>
    {sections_html}
    <footer>Generated at {html.escape(generated_at)} from {len(groups)} experiment groups.</footer>
  </div>
</body>
</html>
"""


def main() -> int:
    ap = argparse.ArgumentParser(description="Generate a static HTML report from benchmark metrics.")
    ap.add_argument("--root", default="output", help="Output root containing metrics.json files")
    ap.add_argument("--out", default="output/latest_report.html", help="HTML output path")
    args = ap.parse_args()

    output_root = Path(args.root).expanduser().resolve()
    repo_root = output_root.parent
    out_path = Path(args.out).expanduser().resolve()
    metrics_paths = sorted(output_root.rglob("metrics.json"))
    records = []
    for metrics_path in metrics_paths:
        record = load_record(metrics_path, output_root, repo_root)
        if record is not None:
            records.append(record)
    if not records:
        raise SystemExit("no metrics.json found")

    grouped: dict[str, list[RunRecord]] = defaultdict(list)
    for record in records:
        grouped[record.group].append(record)

    groups = [summarize_group(group, recs, output_root) for group, recs in grouped.items()]
    groups.sort(key=lambda item: item["latest_mtime"], reverse=True)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(build_page(output_root, groups), encoding="utf-8")
    print(out_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
