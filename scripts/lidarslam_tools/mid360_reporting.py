"""Shared report primitives for MID-360 command-line workflows."""

from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .serialization import payload_to_json


def utc_timestamp() -> str:
    """Return a consistent machine-readable report timestamp."""
    return datetime.now(timezone.utc).isoformat()


def write_report(output_dir: Path, json_name: str, markdown_name: str,
                 payload: dict[str, Any], markdown: str) -> dict[str, Path]:
    """Write the canonical JSON and Markdown report pair."""
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / json_name
    markdown_path = output_dir / markdown_name
    json_path.write_text(payload_to_json(payload) + '\n', encoding='utf-8')
    markdown_path.write_text(markdown + '\n', encoding='utf-8')
    return {'json': json_path, 'markdown': markdown_path}


def status_from_checks(checks: list[dict[str, Any]]) -> str:
    """Reduce check severities to the repository PASS/WARN/FAIL status."""
    statuses = {check['status'] for check in checks}
    if 'fail' in statuses:
        return 'FAIL'
    if 'warn' in statuses:
        return 'WARN'
    return 'PASS'


def count_checks(checks: list[dict[str, Any]]) -> dict[str, int]:
    """Count checks by severity using a stable output schema."""
    return {
        status: sum(check['status'] == status for check in checks)
        for status in ('ok', 'warn', 'fail')
    }


def append_bag_diagnostics(lines: list[str], diagnostics: dict[str, Any]) -> None:
    """Append the shared bag-diagnostics Markdown section body."""
    if not diagnostics:
        lines.append('- missing')
        return
    topics = diagnostics.get('topics') or {}
    for key in ('pointcloud', 'imu'):
        topic = topics.get(key) or {}
        rate_hz = topic.get('metadata_rate_hz')
        frame_ids = topic.get('sampled_frame_ids') or []
        rate_text = f'{float(rate_hz):.2f}' if isinstance(rate_hz, (int, float)) else 'unknown'
        frames_text = ', '.join(frame_ids) if frame_ids else 'not sampled'
        lines.append(f'- {key}_metadata_rate_hz: `{rate_text}`')
        lines.append(f'- {key}_sampled_frame_ids: `{frames_text}`')
    sample_reader = diagnostics.get('sample_reader') or {}
    lines.append(f"- sample_reader_available: `{sample_reader.get('available')}`")
