#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import math
import os
import re
import shutil
import subprocess
from pathlib import Path


REPO_ROOT = Path("/workspace/ai_coding_ws/ros2")
BASE_CFG = REPO_ROOT / "output/glim_ntu_tnp01_vn100_ext_globalnoopt_cfg"
BAG = REPO_ROOT / "demo_data/ntu_viral/tnp_01_points_restamped_vn100_rosbag2"
REF = REPO_ROOT / "output/ntu_viral_tnp01_gt_leica.tum"
APPLY_OFFSET = REPO_ROOT / "scripts/apply_tum_frame_offset.py"
APE = REPO_ROOT / "scripts/ape_from_tum.py"
OUT_ROOT = REPO_ROOT / "output"

PRISM_OFFSET = (-0.243656, -0.012288, -0.328095)


def load_json(path: Path) -> dict:
    text = path.read_text(encoding="utf-8", errors="replace")
    text = re.sub(r"/\*\*.*?\*/", "", text, flags=re.S)
    lines = []
    for line in text.splitlines():
        if "//" in line:
            line = line.split("//", 1)[0]
        line = line.rstrip()
        if line.strip():
            lines.append(line)
    text = "\n".join(lines)
    text = re.sub(r",(?=\s*[}\]])", "", text)
    return json.loads(text)


def save_json(path: Path, obj: dict) -> None:
    path.write_text(json.dumps(obj, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def parse_metric(path: Path) -> dict[str, float]:
    vals: dict[str, float] = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        key = key.strip()
        value = value.strip()
        try:
            vals[key] = float(value)
        except ValueError:
            continue
    return vals


def path_length(tum_path: Path) -> float:
    pts = []
    with tum_path.open("r", encoding="utf-8", errors="replace") as f:
        for line in f:
            if not line.strip():
                continue
            parts = line.split()
            pts.append(tuple(map(float, parts[1:4])))
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


def make_config(
    dst: Path,
    imu_time_offset: float,
    tx: float,
    ty: float,
    tz: float,
    preprocess_threads: int | None = None,
    odom_threads: int | None = None,
) -> None:
    if dst.exists():
        shutil.rmtree(dst)
    shutil.copytree(BASE_CFG, dst)
    ros = load_json(dst / "config_ros.json")
    ros["glim_ros"]["imu_topic"] = "/imu/imu"
    ros["glim_ros"]["imu_topics"] = ["/imu/imu"]
    ros["glim_ros"]["points_topic"] = "/os1_cloud_node1/points"
    ros["glim_ros"]["points_topics"] = ["/os1_cloud_node1/points"]
    ros["glim_ros"]["imu_time_offset"] = imu_time_offset
    ros["glim_ros"]["acc_scale"] = 0.0
    save_json(dst / "config_ros.json", ros)

    sensors = load_json(dst / "config_sensors.json")
    sensors["sensors"]["T_lidar_imu"] = [tx, ty, tz, 0.0, 0.0, 0.0, 1.0]
    sensors["sensors"]["ring_field"] = "ring"
    sensors["sensors"]["autoconf_perpoint_times"] = True
    sensors["sensors"]["autoconf_prefer_frame_time"] = False
    sensors["sensors"]["perpoint_relative_time"] = True
    sensors["sensors"]["perpoint_time_scale"] = 1e-9
    save_json(dst / "config_sensors.json", sensors)

    preprocess = load_json(dst / "config_preprocess.json")
    if preprocess_threads is not None:
        preprocess["preprocess"]["num_threads"] = preprocess_threads
    save_json(dst / "config_preprocess.json", preprocess)

    odom_gpu = load_json(dst / "config_odometry_gpu.json")
    if odom_threads is not None:
        odom_gpu["odometry_estimation"]["num_threads"] = odom_threads
    save_json(dst / "config_odometry_gpu.json", odom_gpu)

    odom_cpu = load_json(dst / "config_odometry_cpu.json")
    if odom_threads is not None:
        odom_cpu["odometry_estimation"]["num_threads"] = odom_threads
    save_json(dst / "config_odometry_cpu.json", odom_cpu)

    gm = load_json(dst / "config_global_mapping_gpu.json")
    gm["global_mapping"]["enable_optimization"] = False
    save_json(dst / "config_global_mapping_gpu.json", gm)


def run_case(case_dir: Path, cfg_dir: Path, omp_threads: int | None = None) -> dict:
    dump_dir = case_dir / "dump"
    log_path = case_dir / "glim.log"
    traj = dump_dir / "traj_lidar.txt"
    prism_traj = case_dir / "traj_lidar_prism.tum"
    ape_out = case_dir / "ape_vs_leica_prism.txt"

    if dump_dir.exists():
        shutil.rmtree(dump_dir)
    dump_dir.mkdir(parents=True, exist_ok=True)

    cmd = [
        "docker",
        "run",
        "--rm",
        "--gpus",
        "all",
    ]
    if omp_threads is not None:
        cmd += ["-e", f"OMP_NUM_THREADS={omp_threads}"]
    cmd += [
        "-v",
        f"{cfg_dir}:/config:ro",
        "-v",
        f"{BAG}:/bag:ro",
        "-v",
        f"{dump_dir}:/tmp/dump",
        "koide3/glim_ros2:jazzy_cuda12.5",
        "bash",
        "-lc",
        "source /opt/ros/jazzy/setup.bash && "
        "source /root/ros2_ws/install/setup.bash && "
        "ros2 run glim_ros glim_rosbag /bag "
        "--ros-args -p config_path:=/config -p auto_quit:=true -p dump_path:=/tmp/dump",
    ]
    with log_path.open("w", encoding="utf-8") as log:
        proc = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, cwd=REPO_ROOT)
    if proc.returncode != 0 or not traj.exists():
        return {
            "ok": False,
            "returncode": proc.returncode,
            "traj_exists": traj.exists(),
            "log_path": str(log_path),
        }

    subprocess.run(
        [
            "python3",
            str(APPLY_OFFSET),
            "--in",
            str(traj),
            "--out",
            str(prism_traj),
            "--tx",
            str(PRISM_OFFSET[0]),
            "--ty",
            str(PRISM_OFFSET[1]),
            "--tz",
            str(PRISM_OFFSET[2]),
        ],
        check=True,
        cwd=REPO_ROOT,
    )
    subprocess.run(
        [
            "python3",
            str(APE),
            "--ref",
            str(REF),
            "--est",
            str(prism_traj),
            "--out",
            str(ape_out),
        ],
        check=True,
        cwd=REPO_ROOT,
    )
    metrics = parse_metric(ape_out)
    return {
        "ok": True,
        "returncode": proc.returncode,
        "rmse": metrics.get("rmse"),
        "median": metrics.get("median"),
        "max": metrics.get("max"),
        "pairs": metrics.get("pairs"),
        "path_length": path_length(traj),
        "traj_path": str(traj),
        "ape_path": str(ape_out),
        "log_path": str(log_path),
    }


def candidate_grid() -> list[dict]:
    # Focused search around the authors' FAST-LIO setting.
    base_t = (-0.050, 0.000, 0.055)
    offsets = [0.10, 0.12, 0.05]
    tx_candidates = [base_t[0] - 0.02, base_t[0], base_t[0] + 0.02]
    tz_candidates = [base_t[2] - 0.02, base_t[2], base_t[2] + 0.02]
    out = []
    for dt in offsets:
        for tx in tx_candidates:
            for tz in tz_candidates:
                out.append(
                    {
                        "imu_time_offset": dt,
                        "tx": round(tx, 6),
                        "ty": 0.0,
                        "tz": round(tz, 6),
                    },
                )
    return out


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--out-dir",
        default=str(OUT_ROOT / "glim_tnp01_vn100_sweep_20260311"),
        help="Output directory",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=0,
        help="Run only the first N candidates",
    )
    parser.add_argument(
        "--resume",
        action="store_true",
        help="Skip cases that already have completed APE results in out-dir.",
    )
    parser.add_argument(
        "--preprocess-threads",
        type=int,
        default=0,
        help="Override config_preprocess.json num_threads when > 0.",
    )
    parser.add_argument(
        "--odom-threads",
        type=int,
        default=0,
        help="Override config_odometry_(gpu|cpu).json num_threads when > 0.",
    )
    parser.add_argument(
        "--omp-threads",
        type=int,
        default=0,
        help="Set OMP_NUM_THREADS inside the GLIM container when > 0.",
    )
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    candidates = candidate_grid()
    if args.limit > 0:
        candidates = candidates[: args.limit]

    summary_path = out_dir / "summary.json"
    if args.resume and summary_path.exists():
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
    else:
        summary = []
    completed = {row["case"] for row in summary if row.get("ok")}
    for idx, cand in enumerate(candidates, start=1):
        tag = (
            f"case_{idx:02d}_toff{cand['imu_time_offset']:+.2f}_"
            f"tx{cand['tx']:+.3f}_tz{cand['tz']:+.3f}"
        ).replace("+", "p").replace("-", "m")
        if args.resume and tag in completed:
            continue
        case_dir = out_dir / tag
        cfg_dir = case_dir / "config"
        case_dir.mkdir(parents=True, exist_ok=True)
        make_config(
            cfg_dir,
            cand["imu_time_offset"],
            cand["tx"],
            cand["ty"],
            cand["tz"],
            preprocess_threads=args.preprocess_threads if args.preprocess_threads > 0 else None,
            odom_threads=args.odom_threads if args.odom_threads > 0 else None,
        )
        result = run_case(case_dir, cfg_dir, omp_threads=args.omp_threads if args.omp_threads > 0 else None)
        rec = {**cand, **result, "case": tag}
        summary.append(rec)
        print(json.dumps(rec, ensure_ascii=True))
        summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    ok = [r for r in summary if r.get("ok") and r.get("rmse") is not None]
    if ok:
        best = min(ok, key=lambda r: r["rmse"])
        print("best", json.dumps(best, ensure_ascii=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
