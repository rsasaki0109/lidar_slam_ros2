#!/usr/bin/env python3

"""Generate synthetic benchmark fixtures for CI and report tooling."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def write_tum(path: Path, *, x_offset: float = 0.0, y_bias: float = 0.0) -> None:
    """Write a tiny TUM trajectory with a deterministic offset."""
    poses = []
    for idx in range(6):
        t = float(idx + 1)
        x = x_offset + idx * 0.5
        y = y_bias + idx * 0.05
        z = 0.0
        poses.append(f"{t:.1f} {x:.6f} {y:.6f} {z:.6f} 0 0 0 1")
    path.write_text("\n".join(poses) + "\n", encoding="utf-8")


def write_ape_report(path: Path, *, rmse: float, median: float, max_error: float) -> None:
    """Write a compact evo-style APE text report."""
    lines = [
        "APE translation (m)",
        "pairs: 6",
        "alignment: se3_umeyama",
        f"rmse: {rmse}",
        f"mean: {rmse * 0.92}",
        f"median: {median}",
        "std: 0.010",
        "min: 0.001",
        f"max: {max_error}",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_log(path: Path, *, run_name: str) -> None:
    """Write a minimal log file so the HTML report has local artifacts."""
    path.write_text(
        "\n".join(
            [
                f"[INFO] synthetic benchmark fixture for {run_name}",
                "[INFO] loop closure completed",
                "[INFO] map export completed",
            ]
        ) + "\n",
        encoding="utf-8",
    )


def write_metrics(
    run_dir: Path,
    *,
    bag_name: str,
    reference_tum: Path,
    ape_rmse: float,
    ape_median: float,
    ape_max: float,
    lid_rtf: float,
    lid_wall: float,
) -> None:
    """Write one synthetic metrics.json tree with paired artifacts."""
    run_dir.mkdir(parents=True, exist_ok=True)

    corrected_tum = run_dir / "traj_corrected.tum"
    raw_tum = run_dir / "traj_raw.tum"
    ape_report = run_dir / "ape_corrected_vs_gt.txt"
    raw_ape_report = run_dir / "ape_raw_vs_gt.txt"
    log_path = run_dir / "synthetic_run.log"

    write_tum(corrected_tum, x_offset=ape_rmse * 0.1, y_bias=ape_median * 0.1)
    write_tum(raw_tum, x_offset=ape_max * 0.05, y_bias=ape_rmse * 0.1)
    write_ape_report(
        ape_report,
        rmse=ape_rmse,
        median=ape_median,
        max_error=ape_max,
    )
    write_ape_report(
        raw_ape_report,
        rmse=ape_rmse * 1.2,
        median=ape_median * 1.2,
        max_error=max(ape_max, ape_rmse * 1.3),
    )
    write_log(log_path, run_name=run_dir.name)

    metrics: dict[str, Any] = {
        "out_dir": str(run_dir),
        "bag_path": f"/synthetic/{bag_name}",
        "bag_duration_sec": 12.0,
        "points_topic": "/points",
        "frames": {
            "points_frame_id": "os_sensor",
            "robot_frame_id": "base_link",
            "global_frame_id": "map",
            "odom_frame_id": "odom",
        },
        "reference": {
            "source": "synthetic_gt",
            "tum_path": str(reference_tum),
        },
        "lidarslam": {
            "success": True,
            "rtf": lid_rtf,
            "wall_sec": lid_wall,
            "param_path": "lidarslam/param/lidarslam.yaml",
            "tum_path": str(corrected_tum),
            "log_path": str(log_path),
        },
        "glim": {
            "available": False,
            "success": False,
            "reference_source": "synthetic_gt",
            "traj_path": str(reference_tum),
        },
        "evo": {
            "ape": {
                "rmse": ape_rmse,
                "median": ape_median,
                "max": ape_max,
                "path": str(ape_report),
            },
            "raw_ape": {
                "rmse": ape_rmse * 1.2,
                "median": ape_median * 1.2,
                "max": max(ape_max, ape_rmse * 1.3),
                "path": str(raw_ape_report),
            },
        },
    }
    (run_dir / "metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n",
        encoding="utf-8",
    )


def generate_fixture(root: Path, profile: str) -> list[Path]:
    """Create a synthetic benchmark root compatible with report tooling."""
    root.mkdir(parents=True, exist_ok=True)
    reference_dir = root / "references"
    reference_dir.mkdir(parents=True, exist_ok=True)
    reference_tum = reference_dir / "synthetic_reference.tum"
    write_tum(reference_tum)

    suite_root = root / "ci_fixture"
    if profile == "passing":
        specs = [
            ("run_best", "synthetic_good_bag", 0.060, 0.040, 0.120, 0.42, 28.0),
            ("run_ok", "synthetic_ok_bag", 0.090, 0.060, 0.180, 0.37, 30.0),
        ]
    else:
        specs = [
            ("run_best", "synthetic_good_bag", 0.060, 0.040, 0.120, 0.42, 28.0),
            ("run_bad", "synthetic_bad_bag", 0.180, 0.120, 0.420, 0.35, 31.0),
        ]

    outputs: list[Path] = []
    for run_name, bag_name, ape_rmse, ape_median, ape_max, lid_rtf, lid_wall in specs:
        run_dir = suite_root / run_name
        write_metrics(
            run_dir,
            bag_name=bag_name,
            reference_tum=reference_tum,
            ape_rmse=ape_rmse,
            ape_median=ape_median,
            ape_max=ape_max,
            lid_rtf=lid_rtf,
            lid_wall=lid_wall,
        )
        outputs.append(run_dir / "metrics.json")
    return outputs


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate synthetic benchmark metrics for CI/report smoke tests.",
    )
    parser.add_argument(
        "--root",
        required=True,
        help="Output root where the synthetic benchmark tree will be created.",
    )
    parser.add_argument(
        "--profile",
        choices=["passing", "failing"],
        default="passing",
        help="Fixture profile to emit.",
    )
    args = parser.parse_args()

    root = Path(args.root).expanduser().resolve()
    outputs = generate_fixture(root, args.profile)
    print(f"fixture_root: {root}")
    for output in outputs:
        print(f"metrics_json: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
