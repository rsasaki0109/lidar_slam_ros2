#!/usr/bin/env python3
"""Aggregate per-sequence KITTI metric JSONs into a single markdown report.

Each input JSON is the output of ``scripts/kitti_metrics.py``: at minimum it
must carry ``t_rel_percent_avg`` and ``r_rel_deg_per_m_avg``. The aggregator
groups results by sequence id (and optionally by estimator label so several
estimators can be compared side by side) and prints / writes a markdown table.

Usage:
  python3 scripts/aggregate_kitti_metrics.py \\
      --input ours::output/kitti/seq00/metrics.json \\
      --input ours::output/kitti/seq05/metrics.json \\
      --input kiss::output/kitti/kiss_icp/seq00.json \\
      --out-md output/kitti_report.md

The ``label::path`` form is preferred when more than one estimator is being
compared. When the prefix is omitted the file's own ``label`` is used; if both
are missing the label defaults to ``run``.

Sequence ids are picked up from the JSON's ``sequence`` key when present and
otherwise inferred from the path with a regex (``seq(\\d+)`` or
``_(\\d{2})_``).
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Any


SEQ_REGEXES = (
    re.compile(r'seq[_-]?(\d{2})'),
    re.compile(r'_(\d{2})_(?:rko|small|lo|graph|bench|kitti)'),
    re.compile(r'_(\d{2})(?:\.|_|$)'),
)


def _infer_sequence(payload: dict[str, Any], path: Path) -> str:
    seq = payload.get('sequence')
    if isinstance(seq, (str, int)) and str(seq).strip():
        return str(seq).strip().zfill(2)
    text = path.as_posix()
    for rx in SEQ_REGEXES:
        m = rx.search(text)
        if m:
            return m.group(1).zfill(2)
    return ''


def _parse_input(spec: str) -> tuple[str, Path]:
    """Split a ``label::path`` argument into (label, path). Label may be empty."""
    if '::' in spec:
        label, path = spec.split('::', 1)
        return label.strip(), Path(path).expanduser()
    return '', Path(spec).expanduser()


def load_metric_files(specs: list[str]) -> list[dict[str, Any]]:
    """Load each metric JSON and normalise into a flat record dict."""
    records: list[dict[str, Any]] = []
    for spec in specs:
        label_override, path = _parse_input(spec)
        if not path.exists():
            raise FileNotFoundError(f'metric file does not exist: {path}')
        payload = json.loads(path.read_text(encoding='utf-8'))
        label = label_override or str(payload.get('label') or '') or 'run'
        records.append(
            {
                'label': label,
                'path': str(path),
                'sequence': _infer_sequence(payload, path),
                't_rel_percent_avg': payload.get('t_rel_percent_avg'),
                'r_rel_deg_per_m_avg': payload.get('r_rel_deg_per_m_avg'),
                'pairs_total': payload.get('pairs_total'),
                'frames': payload.get('frames'),
            }
        )
    return records


def _fmt(v: object, digits: int = 3) -> str:
    if v is None:
        return ''
    try:
        return f'{float(v):.{digits}f}'
    except Exception:
        return str(v)


def render_markdown(records: list[dict[str, Any]]) -> str:
    """Render aggregated records as a markdown report."""
    if not records:
        return '# KITTI Odometry aggregate report\n\nNo input metrics.\n'

    labels = sorted({r['label'] for r in records})
    sequences = sorted({r['sequence'] for r in records if r['sequence']})

    lines: list[str] = []
    lines.append('# KITTI Odometry aggregate report')
    lines.append('')
    lines.append(f'- inputs: {len(records)}')
    lines.append(f'- estimators: {", ".join(labels) if labels else "(unlabeled)"}')
    lines.append(f'- sequences: {", ".join(sequences) if sequences else "(unknown)"}')
    lines.append('')

    if len(labels) > 1 and sequences:
        # Side-by-side comparison: one row per sequence, one column per estimator.
        per_seq: dict[str, dict[str, dict[str, Any]]] = {seq: {} for seq in sequences}
        for r in records:
            seq = r['sequence']
            if not seq:
                continue
            per_seq[seq][r['label']] = r

        header = ['sequence'] + [f'{lbl} t_rel%' for lbl in labels] \
            + [f'{lbl} r_rel°/m' for lbl in labels]
        lines.append('## Per-sequence comparison')
        lines.append('')
        lines.append('| ' + ' | '.join(header) + ' |')
        lines.append('| ' + ' | '.join(['---'] * len(header)) + ' |')
        for seq in sequences:
            row = [seq]
            for lbl in labels:
                r = per_seq[seq].get(lbl)
                row.append(_fmt(r['t_rel_percent_avg']) if r else '')
            for lbl in labels:
                r = per_seq[seq].get(lbl)
                row.append(_fmt(r['r_rel_deg_per_m_avg']) if r else '')
            lines.append('| ' + ' | '.join(row) + ' |')
        lines.append('')

        lines.append('## Aggregate per estimator')
        lines.append('')
        agg_header = ['estimator', 'sequences', 'avg_t_rel%', 'avg_r_rel°/m']
        lines.append('| ' + ' | '.join(agg_header) + ' |')
        lines.append('| ' + ' | '.join(['---'] * len(agg_header)) + ' |')
        for lbl in labels:
            sub = [r for r in records if r['label'] == lbl]
            t_vals = [r['t_rel_percent_avg'] for r in sub if r['t_rel_percent_avg'] is not None]
            r_vals = [r['r_rel_deg_per_m_avg'] for r in sub if r['r_rel_deg_per_m_avg'] is not None]
            avg_t = sum(t_vals) / len(t_vals) if t_vals else None
            avg_r = sum(r_vals) / len(r_vals) if r_vals else None
            lines.append(
                '| ' + ' | '.join([lbl, str(len(sub)), _fmt(avg_t), _fmt(avg_r)]) + ' |'
            )
        lines.append('')
    else:
        lines.append('## Per-run metrics')
        lines.append('')
        header = ['estimator', 'sequence', 't_rel_%', 'r_rel_deg/m', 'pairs', 'frames']
        lines.append('| ' + ' | '.join(header) + ' |')
        lines.append('| ' + ' | '.join(['---'] * len(header)) + ' |')
        for r in records:
            lines.append(
                '| '
                + ' | '.join(
                    [
                        r['label'],
                        r['sequence'] or '',
                        _fmt(r['t_rel_percent_avg']),
                        _fmt(r['r_rel_deg_per_m_avg']),
                        str(r.get('pairs_total') or ''),
                        str(r.get('frames') or ''),
                    ]
                )
                + ' |'
            )
        lines.append('')

        t_vals = [r['t_rel_percent_avg'] for r in records if r['t_rel_percent_avg'] is not None]
        r_vals = [r['r_rel_deg_per_m_avg'] for r in records if r['r_rel_deg_per_m_avg'] is not None]
        if t_vals:
            lines.append(f'- average t_rel: {_fmt(sum(t_vals) / len(t_vals))}%')
        if r_vals:
            lines.append(f'- average r_rel: {_fmt(sum(r_vals) / len(r_vals))} deg/m')

    return '\n'.join(lines) + '\n'


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description='Aggregate KITTI metric JSONs into a markdown report.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument(
        '--input',
        action='append',
        default=[],
        help='Per-run KITTI metric JSON. Optional "label::path" prefix.',
    )
    ap.add_argument('--out-md', type=Path, default=None, help='Write report to this path')
    args = ap.parse_args(argv)

    if not args.input:
        ap.error('at least one --input is required')

    records = load_metric_files(args.input)
    report = render_markdown(records)
    print(report, end='')

    if args.out_md:
        args.out_md.parent.mkdir(parents=True, exist_ok=True)
        args.out_md.write_text(report, encoding='utf-8')
        print(f'wrote {args.out_md}')

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
