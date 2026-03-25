#!/usr/bin/env python3

import argparse
import csv
import json
import statistics
from pathlib import Path
from typing import Any


def _fmt_float(v: Any, digits: int = 3) -> str:
    if v is None:
        return ""
    try:
        return f"{float(v):.{digits}f}"
    except Exception:
        return ""


def _fmt_int(v: Any) -> str:
    if v is None:
        return ""
    try:
        return str(int(v))
    except Exception:
        return ""


def _as_bool(v: Any):
    if isinstance(v, bool):
        return v
    if isinstance(v, str):
        vv = v.strip().lower()
        if vv in ("true", "1", "yes", "y", "on"):
            return True
        if vv in ("false", "0", "no", "n", "off"):
            return False
    return None


def _as_float(v: Any) -> float | None:
    if v is None:
        return None
    try:
        return float(v)
    except Exception:
        return None


def _load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _infer_reference_kind(reference_source: Any, explicit_kind: Any) -> str:
    if explicit_kind:
        return str(explicit_kind)
    lowered = str(reference_source or "").strip().lower()
    if "gt" in lowered or "ground_truth" in lowered:
        return "ground_truth"
    if "glim" in lowered or "cross" in lowered:
        return "cross_validation"
    return ""


def _median(vals: list[float]) -> float | None:
    vals = [v for v in vals if v is not None]
    if not vals:
        return None
    return float(statistics.median(vals))


def _write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _primary_sort_key(primary: str, rec: dict[str, Any]) -> tuple[int, float]:
    missing = bool(rec.get("primary_missing"))
    raw = rec.get("primary_raw")
    raw_f = _as_float(raw)
    return (
        1 if missing or raw_f is None else 0,
        float("inf") if raw_f is None else raw_f,
    )


def _primary_value_text(primary: str, rec: dict[str, Any]) -> str:
    if primary == "lid_rtf":
        return rec.get("lid_rtf", "")
    if primary == "glim_rtf":
        return rec.get("glim_rtf", "")
    if primary == "wall_sec":
        return rec.get("lid_wall_s", "")
    return rec.get("ape_rmse_m", "")


def main() -> int:
    script_dir = Path(__file__).resolve().parent
    repo_root = script_dir.parent

    ap = argparse.ArgumentParser(description="Summarize lidarslam_ros2 benchmark runs (success/RTF/APE).")
    ap.add_argument(
        "--root",
        default=str(repo_root / "output"),
        help="Root output directory (default: ./output under the repo)",
    )
    ap.add_argument("--write-md", default="", help="Write markdown summary to this file")
    ap.add_argument("--write-csv", default="", help="Write CSV summary to this file")
    ap.add_argument(
        "--primary",
        default="ape",
        choices=["ape", "lid_rtf", "glim_rtf", "wall_sec"],
        help="Primary objective for ranking (default: ape)",
    )
    ap.add_argument(
        "--ape-threshold",
        type=float,
        default=None,
        help="Optional APE rmse threshold (m) for pass/fail flag",
    )
    ap.add_argument(
        "--fail-on-ape-threshold",
        action="store_true",
        help=(
            "Return a non-zero exit code when any run is missing APE or "
            "exceeds --ape-threshold"
        ),
    )
    ap.add_argument(
        "--ape-threshold-reference-kind",
        default="all",
        choices=["all", "ground_truth", "cross_validation", "unknown"],
        help=(
            "Limit threshold pass/fail checks to runs whose reference kind matches "
            "this value (default: all)"
        ),
    )
    args = ap.parse_args()

    root = Path(args.root).expanduser().resolve()
    metrics_paths = sorted(root.rglob("metrics.json"))

    if not metrics_paths:
        print(f"no metrics.json found under: {root}")
        return 1

    runs: list[dict[str, Any]] = []
    for p in metrics_paths:
        try:
            runs.append(_load_json(p))
        except Exception as e:
            print(f"warn: failed to read {p}: {e}")

    if not runs:
        print("no readable metrics.json found")
        return 1

    lid_ok = 0
    glim_ok = 0
    lid_rtfs: list[float] = []
    glim_rtfs: list[float] = []
    ape_rmses: list[float] = []

    records: list[dict[str, Any]] = []
    for r in runs:
        out_dir = r.get("out_dir") or ""
        bag_path = r.get("bag_path") or ""
        bag_name = Path(bag_path).name if bag_path else ""
        dur = r.get("bag_duration_sec")

        frames = r.get("frames") or {}
        points_topic = r.get("points_topic") or ""
        points_frame = (frames.get("points_frame_id") or "") if isinstance(frames, dict) else ""

        lid = r.get("lidarslam") or {}
        lid_success = _as_bool(lid.get("success")) if isinstance(lid, dict) else None
        lid_wall = lid.get("wall_sec") if isinstance(lid, dict) else None
        lid_rtf = lid.get("rtf") if isinstance(lid, dict) else None

        glim = r.get("glim") or {}
        glim_avail = _as_bool(glim.get("available")) if isinstance(glim, dict) else None
        glim_success = _as_bool(glim.get("success")) if isinstance(glim, dict) else None
        glim_wall = glim.get("wall_sec") if isinstance(glim, dict) else None
        glim_rtf = glim.get("rtf") if isinstance(glim, dict) else None
        reference = r.get("reference") or {}
        reference_source = (
            (reference.get("source") or "")
            if isinstance(reference, dict) else ""
        )
        reference_kind = _infer_reference_kind(
            reference_source,
            (reference.get("kind") if isinstance(reference, dict) else ""),
        )
        glim_ref = (
            reference_source
            or ((glim.get("reference_source") or "") if isinstance(glim, dict) else "")
        )

        evo = r.get("evo") or {}
        ape = evo.get("ape") if isinstance(evo, dict) else None
        ape_rmse = (ape.get("rmse") if isinstance(ape, dict) else None) if ape is not None else None

        if lid_success is True:
            lid_ok += 1
        if glim_success is True:
            glim_ok += 1

        try:
            if lid_success is True and lid_rtf is not None:
                lid_rtfs.append(float(lid_rtf))
        except Exception:
            pass
        try:
            if glim_success is True and glim_rtf is not None:
                glim_rtfs.append(float(glim_rtf))
        except Exception:
            pass
        try:
            if lid_success is True and ape_rmse is not None:
                ape_rmses.append(float(ape_rmse))
        except Exception:
            pass

        ape_raw = _as_float(ape_rmse)
        lid_raw = _as_float(lid_rtf)
        glim_raw = _as_float(glim_rtf)
        lid_wall_raw = _as_float(lid_wall)

        if args.primary == "ape":
            primary_raw = ape_raw
            primary_missing = ape_raw is None
        elif args.primary == "lid_rtf":
            primary_raw = lid_raw
            primary_missing = lid_raw is None or lid_success is not True
        elif args.primary == "glim_rtf":
            primary_raw = glim_raw
            primary_missing = glim_raw is None or glim_success is not True
        elif args.primary == "wall_sec":
            primary_raw = lid_wall_raw
            primary_missing = lid_wall_raw is None or lid_success is not True
        else:
            primary_raw = ape_raw
            primary_missing = ape_raw is None

        ape_ok = ""
        if args.ape_threshold is not None and ape_raw is not None:
            ape_ok = "true" if ape_raw <= args.ape_threshold else "false"

        records.append(
            {
                "run": Path(out_dir).name if out_dir else "",
                "bag": bag_name,
                "bag_s": _fmt_float(dur, 1),
                "points_topic": points_topic,
                "points_frame": points_frame,
                "lid_ok": str(lid_success).lower() if lid_success is not None else "",
                "lid_rtf": _fmt_float(lid_rtf),
                "lid_wall_s": _fmt_float(lid_wall, 2),
                "glim_avail": "true" if glim_avail is True else ("false" if glim_avail is False else ""),
                "glim_ok": str(glim_success).lower() if glim_success is not None else "",
                "ape_ref_kind": reference_kind,
                "ape_ref_src": glim_ref,
                "glim_rtf": _fmt_float(glim_rtf),
                "glim_wall_s": _fmt_float(glim_wall, 2),
                "ape_rmse_m": _fmt_float(ape_raw),
                "ape_ok": ape_ok,
                "primary_raw": primary_raw,
                "primary_missing": primary_missing,
            }
        )

    records_sorted = sorted(records, key=lambda rec: _primary_sort_key(args.primary, rec))
    for idx, rec in enumerate(records_sorted, start=1):
        rec["primary_rank"] = str(idx)

    total = len(records)
    lid_rate = (100.0 * lid_ok / total) if total else 0.0
    glim_rate = (100.0 * glim_ok / total) if total else 0.0

    lid_med = _median(lid_rtfs)
    glim_med = _median(glim_rtfs)
    ape_med = _median(ape_rmses)

    if args.primary == "ape" and records_sorted:
        best = records_sorted[0]
        best_note = (
            f"{best['run']} ({best.get('ape_rmse_m')}m)"
            if best and not bool(best.get("primary_missing"))
            else "N/A"
        )
    elif records_sorted:
        best_note = f"{records_sorted[0]['run']} ({_primary_value_text(args.primary, records_sorted[0])})"
    else:
        best_note = "N/A"

    pass_count = 0
    threshold_records = records
    if args.ape_threshold_reference_kind != "all":
        threshold_records = [
            rec for rec in records
            if rec.get("ape_ref_kind") == args.ape_threshold_reference_kind
        ]
    if args.ape_threshold is not None:
        for rec in threshold_records:
            v = _as_float(rec.get("ape_rmse_m"))
            if v is not None and v <= args.ape_threshold:
                pass_count += 1

    header = [
        "run",
        "primary_rank",
        "bag",
        "bag_s",
        "points_topic",
        "points_frame",
        "lid_ok",
        "lid_rtf",
        "lid_wall_s",
        "glim_avail",
        "glim_ok",
        "ape_ref_kind",
        "ape_ref_src",
        "glim_rtf",
        "glim_wall_s",
        "ape_rmse_m",
        "ape_ok",
    ]

    md_lines: list[str] = []
    md_lines.append(f"# Benchmark summary (primary={args.primary})")
    md_lines.append("")
    md_lines.append(f"- runs: {total}")
    md_lines.append(f"- lidarslam success: {lid_ok}/{total} ({lid_rate:.1f}%)")
    md_lines.append(f"- lidarslam median RTF (success-only): {_fmt_float(lid_med)}")
    md_lines.append(f"- GLIM success: {glim_ok}/{total} ({glim_rate:.1f}%)")
    md_lines.append(f"- GLIM median RTF (success-only): {_fmt_float(glim_med)}")
    if args.primary == "ape":
        md_lines.append(f"- APE-primary best: {best_note}")
    else:
        md_lines.append(f"- primary_best: {best_note}")
    if ape_med is not None:
        md_lines.append(f"- APE RMSE median vs selected reference (m): {_fmt_float(ape_med)} (n={len(ape_rmses)})")
    else:
        md_lines.append("- APE RMSE: (not available; install evo to enable)")
    if args.ape_threshold is not None:
        md_lines.append(
            f"- APE threshold <= {args.ape_threshold}m "
            f"(reference_kind={args.ape_threshold_reference_kind}): "
            f"{pass_count}/{len(threshold_records)}"
        )

    md_lines.append("")
    md_lines.append("| " + " | ".join(header) + " |")
    md_lines.append("| " + " | ".join(["---"] * len(header)) + " |")
    for rec in records_sorted:
        row = [
            rec["run"],
            rec.get("primary_rank", ""),
            rec["bag"],
            rec["bag_s"],
            rec["points_topic"],
            rec["points_frame"],
            rec["lid_ok"],
            rec["lid_rtf"],
            rec["lid_wall_s"],
            rec["glim_avail"],
            rec["glim_ok"],
            rec["ape_ref_kind"],
            rec["ape_ref_src"],
            rec["glim_rtf"],
            rec["glim_wall_s"],
            rec["ape_rmse_m"],
            rec["ape_ok"],
        ]
        md_lines.append("| " + " | ".join(row) + " |")
    md = "\n".join(md_lines) + "\n"

    print(md, end="")

    if args.write_md:
        _write_text(Path(args.write_md).expanduser().resolve(), md)
    if args.write_csv:
        out_csv = Path(args.write_csv).expanduser().resolve()
        out_csv.parent.mkdir(parents=True, exist_ok=True)
        with out_csv.open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(header)
            for rec in records_sorted:
                w.writerow(
                    [
                        rec["run"],
                        rec.get("primary_rank", ""),
                        rec["bag"],
                        rec["bag_s"],
                        rec["points_topic"],
                        rec["points_frame"],
                        rec["lid_ok"],
                        rec["lid_rtf"],
                        rec["lid_wall_s"],
                        rec["glim_avail"],
                        rec["glim_ok"],
                        rec["ape_ref_kind"],
                        rec["ape_ref_src"],
                        rec["glim_rtf"],
                        rec["glim_wall_s"],
                        rec["ape_rmse_m"],
                        rec["ape_ok"],
                    ]
                )

    if args.fail_on_ape_threshold:
        if args.ape_threshold is None:
            print("error: --fail-on-ape-threshold requires --ape-threshold")
            return 1

        failing_runs = [
            rec["run"]
            for rec in records_sorted
            if (
                args.ape_threshold_reference_kind == "all"
                or rec.get("ape_ref_kind") == args.ape_threshold_reference_kind
            )
            if rec.get("ape_ok") != "true"
        ]
        if failing_runs:
            print(
                "error: APE threshold failed for runs: "
                + ", ".join(failing_runs),
            )
            return 2

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
