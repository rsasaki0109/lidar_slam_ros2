"""Benchmark record loading, aggregation, formatting, and quality rules."""

from __future__ import annotations

import html
import json
import math
import re
import statistics
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any


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
