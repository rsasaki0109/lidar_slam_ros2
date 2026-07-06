#!/usr/bin/env python3
"""Summarize a per-scan degeneracy diagnostics CSV into a report-only summary."""

# v0.8 Phase 1 (docs/roadmap/v0.8.md §5): consumes the per-scan CSV written by
# graph_based_slam (degeneracy_diagnostics_csv_path parameter, live component
# or graph_slam_offline_runner) and reproduces the same scan-level summary the
# C++ map-bundle degeneracy report computes (degeneracy_report_summary.hpp):
# category counts/rates over the scans with diagnostics available, plus the
# longest contiguous not-well-conditioned interval. Report-only by design --
# this script never fails a gate on classification content, only on unusable
# input (missing/empty/malformed CSV).

from __future__ import annotations

import argparse
import csv
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path

EXPECTED_COLUMNS = 54
CATEGORY_ORDER = ('WELL_CONDITIONED', 'DEGENERATE', 'NON_OBSERVABLE')


@dataclass
class WorstInterval:
    """Longest contiguous run of not-fully-well-conditioned scans."""

    valid: bool = False
    start_stamp_sec: float = 0.0
    end_stamp_sec: float = 0.0
    length_scans: int = 0
    category: str = 'DEGENERATE'


@dataclass
class Summary:
    """Scan-level degeneracy summary mirroring degeneracy_report_summary.hpp."""

    total_scans: int = 0
    diagnostics_available_scans: int = 0
    well_conditioned_scans: int = 0
    degenerate_scans: int = 0
    non_observable_scans: int = 0
    worst_interval: WorstInterval = field(default_factory=WorstInterval)

    def ratios(self) -> dict:
        """Return availability and per-category rates as a dict."""
        available = self.diagnostics_available_scans
        return {
            'diagnostics_available_ratio': (
                self.diagnostics_available_scans / self.total_scans if self.total_scans else 0.0
            ),
            'well_conditioned_ratio': (
                self.well_conditioned_scans / available if available else 0.0
            ),
            'degenerate_ratio': self.degenerate_scans / available if available else 0.0,
            'non_observable_ratio': self.non_observable_scans / available if available else 0.0,
        }


def worst_category(row: dict) -> str:
    """Return the most ill-constrained category present in one CSV row."""
    if int(row['non_observable_count']) > 0:
        return 'NON_OBSERVABLE'
    if int(row['degenerate_count']) > 0:
        return 'DEGENERATE'
    return 'WELL_CONDITIONED'


def _pick_worse(current: WorstInterval, candidate: WorstInterval) -> WorstInterval:
    """Return the longer interval; ties prefer NON_OBSERVABLE, then first seen."""
    if not candidate.valid:
        return current
    if not current.valid:
        return candidate
    if candidate.length_scans > current.length_scans:
        return candidate
    if (candidate.length_scans == current.length_scans and
            candidate.category == 'NON_OBSERVABLE' and current.category != 'NON_OBSERVABLE'):
        return candidate
    return current


def summarize_rows(rows: list[dict]) -> Summary:
    """Fold parsed CSV rows into a Summary, mirroring the C++ accumulator."""
    summary = Summary()
    current: WorstInterval | None = None

    def close_run() -> None:
        nonlocal current
        if current is not None:
            summary.worst_interval = _pick_worse(summary.worst_interval, current)
            current = None

    for row in rows:
        summary.total_scans += 1
        if row['diagnostics_available'] != '1':
            close_run()
            continue
        summary.diagnostics_available_scans += 1
        worst = worst_category(row)
        stamp = float(row['stamp_sec'])
        if worst == 'WELL_CONDITIONED':
            summary.well_conditioned_scans += 1
            close_run()
            continue
        if worst == 'DEGENERATE':
            summary.degenerate_scans += 1
        else:
            summary.non_observable_scans += 1
        if current is None:
            current = WorstInterval(
                valid=True, start_stamp_sec=stamp, end_stamp_sec=stamp,
                length_scans=0, category=worst)
        elif worst == 'NON_OBSERVABLE':
            current.category = 'NON_OBSERVABLE'
        current.end_stamp_sec = stamp
        current.length_scans += 1

    close_run()
    return summary


def load_rows(csv_path: Path) -> list[dict]:
    """Read and validate the diagnostics CSV, returning its data rows."""
    with csv_path.open('r', encoding='utf-8', newline='') as handle:
        reader = csv.DictReader(handle)
        column_count = 0 if reader.fieldnames is None else len(reader.fieldnames)
        if column_count != EXPECTED_COLUMNS:
            raise ValueError(
                f'unexpected CSV header ({column_count} columns,'
                f' expected {EXPECTED_COLUMNS}): {csv_path}')
        return list(reader)


def summary_as_dict(summary: Summary) -> dict:
    """Serialize a Summary (counts, ratios, worst interval) to a plain dict."""
    ratios = summary.ratios()
    data = {
        'total_scans': summary.total_scans,
        'diagnostics_available_scans': summary.diagnostics_available_scans,
        'well_conditioned_scans': summary.well_conditioned_scans,
        'degenerate_scans': summary.degenerate_scans,
        'non_observable_scans': summary.non_observable_scans,
    }
    data.update({key: round(value, 6) for key, value in ratios.items()})
    interval = {'valid': summary.worst_interval.valid}
    if summary.worst_interval.valid:
        interval.update({
            'category': summary.worst_interval.category,
            'start_stamp_sec': round(summary.worst_interval.start_stamp_sec, 6),
            'end_stamp_sec': round(summary.worst_interval.end_stamp_sec, 6),
            'length_scans': summary.worst_interval.length_scans,
        })
    data['worst_interval'] = interval
    return data


def render_markdown(data: dict, csv_path: Path) -> str:
    """Render the summary dict as a small Markdown report."""
    lines = [
        '# Degeneracy diagnostics summary (report-only)',
        '',
        f'- source_csv: `{csv_path}`',
        f'- total_scans: {data["total_scans"]}',
        f'- diagnostics_available_scans: {data["diagnostics_available_scans"]}'
        f' ({100.0 * data["diagnostics_available_ratio"]:.1f}%)',
        '',
        '| category (scan-level worst) | scans | rate of available |',
        '|---|---:|---:|',
    ]
    for name, count_key, ratio_key in (
            ('WELL_CONDITIONED', 'well_conditioned_scans', 'well_conditioned_ratio'),
            ('DEGENERATE', 'degenerate_scans', 'degenerate_ratio'),
            ('NON_OBSERVABLE', 'non_observable_scans', 'non_observable_ratio')):
        lines.append(f'| {name} | {data[count_key]} | {100.0 * data[ratio_key]:.1f}% |')
    lines.append('')
    interval = data['worst_interval']
    if interval['valid']:
        lines.append(
            f'- worst interval: {interval["length_scans"]} scans'
            f' ({interval["category"]}), t = {interval["start_stamp_sec"]:.3f}'
            f' .. {interval["end_stamp_sec"]:.3f} s')
    else:
        lines.append('- worst interval: none (no not-well-conditioned run observed)')
    lines.append('')
    return '\n'.join(lines)


def main(argv: list[str] | None = None) -> int:
    """Run the CLI."""
    parser = argparse.ArgumentParser(
        description='Summarize a per-scan degeneracy diagnostics CSV (report-only).')
    parser.add_argument('--csv', required=True, help='diagnostics CSV path')
    parser.add_argument('--write-md', default='', help='optional Markdown output path')
    parser.add_argument('--write-json', default='', help='optional JSON output path')
    args = parser.parse_args(argv)

    csv_path = Path(args.csv)
    if not csv_path.is_file():
        print(f'error: CSV not found: {csv_path}', file=sys.stderr)
        return 1
    try:
        rows = load_rows(csv_path)
    except ValueError as error:
        print(f'error: {error}', file=sys.stderr)
        return 1
    if not rows:
        print(f'error: CSV has no data rows: {csv_path}', file=sys.stderr)
        return 1

    data = summary_as_dict(summarize_rows(rows))
    markdown = render_markdown(data, csv_path)
    print(markdown)
    if args.write_md:
        Path(args.write_md).write_text(markdown, encoding='utf-8')
    if args.write_json:
        Path(args.write_json).write_text(
            json.dumps(data, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    return 0


if __name__ == '__main__':
    sys.exit(main())
