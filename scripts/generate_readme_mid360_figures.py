#!/usr/bin/env python3
from __future__ import annotations

import bisect
import math
from pathlib import Path
from xml.sax.saxutils import escape

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "output"
IMAGE_DIR = ROOT / "lidarslam" / "images"

XY_OUT = IMAGE_DIR / "mid360_glim_compare_xy.svg"
ERR_OUT = IMAGE_DIR / "mid360_glim_compare_error.svg"


def find_latest_any(patterns: list[str]) -> Path | None:
    candidates = []
    for pattern in patterns:
        candidates.extend(OUTPUT.glob(pattern))
    candidates = [path for path in candidates if path.exists()]
    if not candidates:
        return None
    return max(candidates, key=lambda path: path.stat().st_mtime)


def load_tum(path: Path) -> list[dict[str, float]]:
    rows = []
    for line in path.read_text().splitlines():
        parts = line.strip().split()
        if len(parts) < 8:
            continue
        t, x, y, z, qx, qy, qz, qw = map(float, parts[:8])
        rows.append(
            {
                "t": t,
                "x": x,
                "y": y,
                "z": z,
                "qx": qx,
                "qy": qy,
                "qz": qz,
                "qw": qw,
            }
        )
    return rows


def path_length(rows: list[dict[str, float]]) -> float:
    total = 0.0
    for a, b in zip(rows, rows[1:]):
        total += math.dist((a["x"], a["y"], a["z"]), (b["x"], b["y"], b["z"]))
    return total


def match_rows(
    ref_rows: list[dict[str, float]],
    est_rows: list[dict[str, float]],
    tolerance: float,
) -> list[tuple[dict[str, float], dict[str, float]]]:
    est_times = [row["t"] for row in est_rows]
    pairs = []
    for ref in ref_rows:
        idx = bisect.bisect_left(est_times, ref["t"])
        candidates = []
        if idx < len(est_rows):
            candidates.append(est_rows[idx])
        if idx > 0:
            candidates.append(est_rows[idx - 1])
        best = None
        best_dt = None
        for cand in candidates:
            dt = abs(cand["t"] - ref["t"])
            if best is None or dt < best_dt:
                best = cand
                best_dt = dt
        if best is not None and best_dt is not None and best_dt <= tolerance:
            pairs.append((ref, best))
    return pairs


def rigid_align(
    pairs: list[tuple[dict[str, float], dict[str, float]]],
) -> tuple[np.ndarray, np.ndarray]:
    ref = np.array([[a["x"], a["y"], a["z"]] for a, _ in pairs], dtype=float)
    est = np.array([[b["x"], b["y"], b["z"]] for _, b in pairs], dtype=float)
    ref_centroid = ref.mean(axis=0)
    est_centroid = est.mean(axis=0)
    w = (est - est_centroid).T @ (ref - ref_centroid)
    u, _, vt = np.linalg.svd(w)
    rot = vt.T @ u.T
    if np.linalg.det(rot) < 0:
        vt[-1, :] *= -1.0
        rot = vt.T @ u.T
    trans = ref_centroid - rot @ est_centroid
    return rot, trans


def apply_alignment(
    rows: list[dict[str, float]], rot: np.ndarray, trans: np.ndarray
) -> list[dict[str, float]]:
    aligned = []
    for row in rows:
        pos = np.array([row["x"], row["y"], row["z"]], dtype=float)
        pos_aligned = rot @ pos + trans
        aligned.append(
            {
                **row,
                "x": float(pos_aligned[0]),
                "y": float(pos_aligned[1]),
                "z": float(pos_aligned[2]),
            }
        )
    return aligned


def ticks(min_v: float, max_v: float, count: int = 6) -> list[float]:
    if math.isclose(min_v, max_v):
        return [min_v]
    step = (max_v - min_v) / max(count - 1, 1)
    return [min_v + step * i for i in range(count)]


def fmt_meter(value: float) -> str:
    return f"{value:.1f}"


def line(points: list[tuple[float, float]]) -> str:
    return " ".join(f"{x:.2f},{y:.2f}" for x, y in points)


def build_xy_svg(
    glim_rows: list[dict[str, float]],
    lid_rows: list[dict[str, float]],
    summary: dict[str, float],
) -> str:
    width = 1080
    height = 640
    left = 96
    right = 24
    top = 116
    bottom = 72
    plot_w = width - left - right
    plot_h = height - top - bottom

    all_x = [row["x"] for row in glim_rows + lid_rows]
    all_y = [row["y"] for row in glim_rows + lid_rows]
    min_x = min(all_x)
    max_x = max(all_x)
    min_y = min(all_y)
    max_y = max(all_y)
    pad_x = max((max_x - min_x) * 0.04, 1.0)
    pad_y = max((max_y - min_y) * 0.04, 1.0)
    min_x -= pad_x
    max_x += pad_x
    min_y -= pad_y
    max_y += pad_y

    span_x = max_x - min_x
    span_y = max_y - min_y
    scale = min(plot_w / span_x, plot_h / span_y)
    draw_w = span_x * scale
    draw_h = span_y * scale
    x_offset = left + (plot_w - draw_w) / 2.0
    y_offset = top + (plot_h - draw_h) / 2.0

    def project(row: dict[str, float]) -> tuple[float, float]:
        px = x_offset + (row["x"] - min_x) * scale
        py = y_offset + draw_h - (row["y"] - min_y) * scale
        return px, py

    glim_poly = line([project(row) for row in glim_rows])
    lid_poly = line([project(row) for row in lid_rows])

    x_ticks = ticks(min_x, max_x)
    y_ticks = ticks(min_y, max_y)

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" width="{width}" height="{height}">',
        '<rect width="100%" height="100%" fill="#f6f8fb"/>',
        '<text x="28" y="38" font-family="Arial, sans-serif" font-size="28" font-weight="700" fill="#13202b">GLIM MID360 sample: XY trajectory overlay</text>',
        '<text x="28" y="66" font-family="Arial, sans-serif" font-size="16" fill="#516679">Rigid alignment applied. The plot uses a fixed aspect ratio to preserve geometry.</text>',
        f'<rect x="{left}" y="{top}" width="{plot_w}" height="{plot_h}" fill="#ffffff" stroke="#d8e3ef"/>',
    ]

    for value in x_ticks:
        x = x_offset + (value - min_x) * scale
        parts.append(
            f'<line x1="{x:.2f}" y1="{top}" x2="{x:.2f}" y2="{top + plot_h}" stroke="#e9eef4" stroke-width="1"/>'
        )
        parts.append(
            f'<text x="{x:.2f}" y="{height - 28}" text-anchor="middle" font-family="Arial, sans-serif" font-size="13" fill="#607487">{escape(fmt_meter(value))}</text>'
        )
    for value in y_ticks:
        y = y_offset + draw_h - (value - min_y) * scale
        parts.append(
            f'<line x1="{left}" y1="{y:.2f}" x2="{left + plot_w}" y2="{y:.2f}" stroke="#e9eef4" stroke-width="1"/>'
        )
        parts.append(
            f'<text x="84" y="{y + 4:.2f}" text-anchor="end" font-family="Arial, sans-serif" font-size="13" fill="#607487">{escape(fmt_meter(value))}</text>'
        )

    parts.extend(
        [
            f'<polyline points="{glim_poly}" fill="none" stroke="#0b6bcb" stroke-width="3.0" vector-effect="non-scaling-stroke"/>',
            f'<polyline points="{lid_poly}" fill="none" stroke="#bc4b2f" stroke-width="2.6" vector-effect="non-scaling-stroke"/>',
        ]
    )

    for row, color, label in [
        (glim_rows[0], "#0b6bcb", "GLIM start"),
        (glim_rows[-1], "#0b6bcb", "GLIM end"),
        (lid_rows[0], "#bc4b2f", "lidarslam start"),
        (lid_rows[-1], "#bc4b2f", "lidarslam end"),
    ]:
        x, y = project(row)
        parts.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="5.5" fill="{color}" stroke="#ffffff" stroke-width="2"/>')

    parts.extend(
        [
            f'<text x="{left + plot_w / 2:.2f}" y="{height - 10}" text-anchor="middle" font-family="Arial, sans-serif" font-size="15" fill="#425568">X [m]</text>',
            f'<text x="22" y="{top + plot_h / 2:.2f}" transform="rotate(-90 22 {top + plot_h / 2:.2f})" text-anchor="middle" font-family="Arial, sans-serif" font-size="15" fill="#425568">Y [m]</text>',
            '<rect x="760" y="18" width="292" height="82" rx="10" fill="#ffffff" stroke="#d8e3ef"/>',
            '<line x1="780" y1="42" x2="816" y2="42" stroke="#0b6bcb" stroke-width="4"/>',
            '<text x="826" y="47" font-family="Arial, sans-serif" font-size="15" fill="#0b6bcb">GLIM reference</text>',
            '<line x1="780" y1="68" x2="816" y2="68" stroke="#bc4b2f" stroke-width="4"/>',
            '<text x="826" y="73" font-family="Arial, sans-serif" font-size="15" fill="#bc4b2f">lidarslam aligned</text>',
            f'<text x="780" y="92" font-family="Arial, sans-serif" font-size="13" fill="#425568">RMSE {summary["rmse"]:.3f} m, median {summary["median"]:.3f} m, max {summary["max"]:.3f} m</text>',
            f'<text x="780" y="108" font-family="Arial, sans-serif" font-size="13" fill="#425568">Path lengths: GLIM {summary["glim_path"]:.2f} m, lidarslam {summary["lid_path"]:.2f} m</text>',
        ]
    )
    parts.append("</svg>")
    return "\n".join(parts)


def build_error_svg(
    errors: list[tuple[float, float]],
    summary: dict[str, float],
) -> str:
    width = 1080
    height = 460
    left = 88
    right = 24
    top = 92
    bottom = 68
    plot_w = width - left - right
    plot_h = height - top - bottom

    min_t = 0.0
    max_t = max(t for t, _ in errors)
    min_e = 0.0
    max_e = max(e for _, e in errors)
    y_max = max_e * 1.08

    def project(t: float, e: float) -> tuple[float, float]:
        x = left + (t - min_t) / max(max_t - min_t, 1e-9) * plot_w
        y = top + plot_h - (e - min_e) / max(y_max - min_e, 1e-9) * plot_h
        return x, y

    err_poly = line([project(t, e) for t, e in errors])
    x_ticks = ticks(min_t, max_t)
    y_ticks = ticks(min_e, y_max)

    rmse_y = project(0.0, summary["rmse"])[1]
    median_y = project(0.0, summary["median"])[1]

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" width="{width}" height="{height}">',
        '<rect width="100%" height="100%" fill="#f6f8fb"/>',
        '<text x="28" y="38" font-family="Arial, sans-serif" font-size="28" font-weight="700" fill="#13202b">GLIM MID360 sample: position error after rigid alignment</text>',
        '<text x="28" y="66" font-family="Arial, sans-serif" font-size="16" fill="#516679">Lower is better. Error is the 3D distance between time-matched poses after SE(3) alignment.</text>',
        f'<rect x="{left}" y="{top}" width="{plot_w}" height="{plot_h}" fill="#ffffff" stroke="#d8e3ef"/>',
    ]

    for value in x_ticks:
        x = left + (value - min_t) / max(max_t - min_t, 1e-9) * plot_w
        parts.append(
            f'<line x1="{x:.2f}" y1="{top}" x2="{x:.2f}" y2="{top + plot_h}" stroke="#e9eef4" stroke-width="1"/>'
        )
        parts.append(
            f'<text x="{x:.2f}" y="{height - 26}" text-anchor="middle" font-family="Arial, sans-serif" font-size="13" fill="#607487">{value:.0f}</text>'
        )
    for value in y_ticks:
        y = top + plot_h - (value - min_e) / max(y_max - min_e, 1e-9) * plot_h
        parts.append(
            f'<line x1="{left}" y1="{y:.2f}" x2="{left + plot_w}" y2="{y:.2f}" stroke="#e9eef4" stroke-width="1"/>'
        )
        parts.append(
            f'<text x="76" y="{y + 4:.2f}" text-anchor="end" font-family="Arial, sans-serif" font-size="13" fill="#607487">{value:.1f}</text>'
        )

    parts.extend(
        [
            f'<line x1="{left}" y1="{rmse_y:.2f}" x2="{left + plot_w}" y2="{rmse_y:.2f}" stroke="#0ea5a3" stroke-dasharray="8 6" stroke-width="2"/>',
            f'<line x1="{left}" y1="{median_y:.2f}" x2="{left + plot_w}" y2="{median_y:.2f}" stroke="#f59e0b" stroke-dasharray="8 6" stroke-width="2"/>',
            f'<polyline points="{err_poly}" fill="none" stroke="#7a3cff" stroke-width="2.4" vector-effect="non-scaling-stroke"/>',
            f'<text x="{left + plot_w / 2:.2f}" y="{height - 10}" text-anchor="middle" font-family="Arial, sans-serif" font-size="15" fill="#425568">Time [s]</text>',
            f'<text x="22" y="{top + plot_h / 2:.2f}" transform="rotate(-90 22 {top + plot_h / 2:.2f})" text-anchor="middle" font-family="Arial, sans-serif" font-size="15" fill="#425568">Position error [m]</text>',
            '<rect x="714" y="18" width="338" height="88" rx="10" fill="#ffffff" stroke="#d8e3ef"/>',
            '<line x1="734" y1="42" x2="770" y2="42" stroke="#7a3cff" stroke-width="4"/>',
            '<text x="780" y="47" font-family="Arial, sans-serif" font-size="15" fill="#7a3cff">error trace</text>',
            '<line x1="734" y1="68" x2="770" y2="68" stroke="#0ea5a3" stroke-dasharray="8 6" stroke-width="3"/>',
            f'<text x="780" y="73" font-family="Arial, sans-serif" font-size="15" fill="#0ea5a3">RMSE {summary["rmse"]:.3f} m</text>',
            '<line x1="734" y1="94" x2="770" y2="94" stroke="#f59e0b" stroke-dasharray="8 6" stroke-width="3"/>',
            f'<text x="780" y="99" font-family="Arial, sans-serif" font-size="15" fill="#f59e0b">median {summary["median"]:.3f} m, max {summary["max"]:.3f} m</text>',
        ]
    )
    parts.append("</svg>")
    return "\n".join(parts)


def main() -> None:
    glim_dir = find_latest_any(["glim_mid360_sample_*"])
    lid_dir = find_latest_any(
        [
            "lidarslam_mid360_auto_*",
            "lidarslam_mid360_noimu_nograph_fix_*",
            "lidarslam_mid360_noimu_*",
            "lidarslam_mid360_clean_*",
        ]
    )
    if glim_dir is None or lid_dir is None:
        raise SystemExit("required MID360 runs not found")

    glim_rows = load_tum(glim_dir / "dump" / "traj_lidar.txt")
    lid_rows = load_tum(lid_dir / "traj_lidarslam.tum")
    if not glim_rows or not lid_rows:
        raise SystemExit("trajectory file missing")

    pairs = match_rows(glim_rows, lid_rows, tolerance=0.05)
    if len(pairs) < 10:
        pairs = match_rows(glim_rows, lid_rows, tolerance=0.15)
    if len(pairs) < 10:
        raise SystemExit("not enough matched poses")

    rot, trans = rigid_align(pairs)
    lid_aligned = apply_alignment(lid_rows, rot, trans)
    aligned_pairs = match_rows(glim_rows, lid_aligned, tolerance=0.05)
    if len(aligned_pairs) < 10:
        aligned_pairs = match_rows(glim_rows, lid_aligned, tolerance=0.15)

    errors = []
    for ref, est in aligned_pairs:
        errors.append(
            (
                ref["t"] - aligned_pairs[0][0]["t"],
                math.dist((ref["x"], ref["y"], ref["z"]), (est["x"], est["y"], est["z"])),
            )
        )

    err_values = [value for _, value in errors]
    summary = {
        "glim_path": path_length(glim_rows),
        "lid_path": path_length(lid_rows),
        "rmse": math.sqrt(sum(v * v for v in err_values) / len(err_values)),
        "median": sorted(err_values)[len(err_values) // 2],
        "max": max(err_values),
    }

    IMAGE_DIR.mkdir(parents=True, exist_ok=True)
    XY_OUT.write_text(build_xy_svg(glim_rows, lid_aligned, summary), encoding="utf-8")
    ERR_OUT.write_text(build_error_svg(errors, summary), encoding="utf-8")
    print(XY_OUT)
    print(ERR_OUT)


if __name__ == "__main__":
    main()
