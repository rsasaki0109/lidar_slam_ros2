#!/usr/bin/env python3
"""Aggregate backend replay variance runs into a small markdown report."""

from __future__ import annotations

import argparse
import hashlib
import re
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


RMSE_RE = re.compile(
    r'^\s*rmse:\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)'
)


@dataclass
class RunSummary:
    name: str
    rmse: Optional[float]
    edges: set[tuple[int, int]]
    edge_hash: str
    log_loop_edges: Optional[int]


def run_sort_key(path: Path) -> tuple[int, str]:
    match = re.search(r'(\d+)$', path.name)
    if match:
        return int(match.group(1)), path.name
    return sys.maxsize, path.name


def read_rmse(path: Path) -> Optional[float]:
    if not path.exists():
        return None

    for line in path.read_text(errors='replace').splitlines():
        match = RMSE_RE.match(line)
        if match:
            return float(match.group(1))

    return None


def read_edges(path: Path) -> set[tuple[int, int]]:
    edges: set[tuple[int, int]] = set()
    if not path.exists():
        return edges

    for line in path.read_text(errors='replace').splitlines():
        parts = line.split()
        if len(parts) < 2:
            continue

        try:
            i = int(parts[0])
            j = int(parts[1])
        except ValueError:
            continue

        edges.add((min(i, j), max(i, j)))

    return edges


def edge_set_hash(edges: set[tuple[int, int]]) -> str:
    payload = '\n'.join(f'{i} {j}' for i, j in sorted(edges))
    return hashlib.sha256(payload.encode('utf-8')).hexdigest()[:12]


def count_backend_loop_lines(path: Path) -> Optional[int]:
    if not path.exists():
        return None

    return sum(
        1 for line in path.read_text(errors='replace').splitlines()
        if 'Loop edge' in line
    )


def collect_runs(bench_dir: Path) -> list[RunSummary]:
    summaries: list[RunSummary] = []

    for run_dir in sorted(bench_dir.glob('replay_run*'), key=run_sort_key):
        if not run_dir.is_dir():
            continue

        edges = read_edges(run_dir / 'loop_edges.txt')
        summaries.append(
            RunSummary(
                name=run_dir.name,
                rmse=read_rmse(run_dir / 'ape.txt'),
                edges=edges,
                edge_hash=edge_set_hash(edges),
                log_loop_edges=count_backend_loop_lines(run_dir / 'backend.log'),
            )
        )

    return summaries


def format_rmse(value: Optional[float]) -> str:
    if value is None:
        return 'NA'
    return f'{value:.9g}'


def format_sigma(values: list[float]) -> str:
    if not values:
        return 'nan'
    if len(values) == 1:
        return '0'
    return f'{statistics.stdev(values):.9g}'


def format_pair_list(edges: set[tuple[int, int]]) -> str:
    if not edges:
        return '(none)'
    return ', '.join(f'{i} {j}' for i, j in sorted(edges))


def print_report(summaries: list[RunSummary]) -> None:
    print('| run | ape_rmse | n_loop_edges | edge_set_hash |')
    print('| --- | ---: | ---: | --- |')

    for summary in summaries:
        print(
            f'| {summary.name} | {format_rmse(summary.rmse)} | '
            f'{len(summary.edges)} | `{summary.edge_hash}` |'
        )

    rmse_values = [summary.rmse for summary in summaries if summary.rmse is not None]
    print()
    print(f'ape_sigma: {format_sigma(rmse_values)}')

    if not summaries:
        print('edge_sets_identical: true')
        return

    base_edges = summaries[0].edges
    identical = all(summary.edges == base_edges for summary in summaries[1:])
    print(f'edge_sets_identical: {str(identical).lower()}')

    if identical:
        return

    print()
    print('symmetric_differences_vs_run1:')
    for summary in summaries[1:]:
        diff = base_edges ^ summary.edges
        print(f'- {summary.name}: {format_pair_list(diff)}')


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Aggregate backend replay variance outputs.'
    )
    parser.add_argument(
        '--bench-dir',
        required=True,
        type=Path,
        help='Directory containing replay_run*/ subdirectories.',
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    summaries = collect_runs(args.bench_dir)
    print_report(summaries)
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except Exception as exc:  # Reporting tool: never fail the sweep.
        print(f'aggregate_backend_replay.py: ERROR: {exc}', file=sys.stderr)
        raise SystemExit(0)
