#!/usr/bin/env python3
"""Summarize packet IMU deskew validation runs from real open-data benchmarks."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


def _load_metrics(path: Path) -> dict:
    return json.loads(path.read_text(encoding='utf-8'))


def _get_case_metrics(root: Path) -> list[dict]:
    cases: list[dict] = []
    for case_dir in sorted(path for path in root.iterdir() if path.is_dir()):
        no_imu_path = case_dir / 'no_imu' / 'metrics.json'
        imu_path = case_dir / 'imu' / 'metrics.json'
        if not no_imu_path.is_file() or not imu_path.is_file():
            continue
        no_imu = _load_metrics(no_imu_path)
        imu = _load_metrics(imu_path)
        cases.append(
            {
                'label': case_dir.name,
                'no_imu_path': str(no_imu_path),
                'imu_path': str(imu_path),
                'no_imu': no_imu,
                'imu': imu,
            }
        )
    return cases


def _path_coverage(metrics: dict) -> float | None:
    cross_validation = metrics.get('cross_validation', {})
    estimated = cross_validation.get('estimated_path_length_m')
    reference = cross_validation.get('reference_path_length_m')
    if not isinstance(estimated, (float, int)) or not isinstance(reference, (float, int)):
        return None
    if reference <= 0.0:
        return None
    return float(estimated) / float(reference)


def _round(value: float | None, digits: int = 3) -> str:
    if value is None:
        return 'n/a'
    return f'{value:.{digits}f}'


def _summarize_case(case: dict, args: argparse.Namespace) -> dict:
    no_imu_metrics = case['no_imu']
    imu_metrics = case['imu']
    no_imu_rmse = no_imu_metrics['evo']['ape']['rmse']
    imu_rmse = imu_metrics['evo']['ape']['rmse']
    no_imu_matched = no_imu_metrics['cross_validation']['matched_poses']
    imu_matched = imu_metrics['cross_validation']['matched_poses']
    no_imu_coverage = _path_coverage(no_imu_metrics)
    imu_coverage = _path_coverage(imu_metrics)
    rmse_ratio = None
    if no_imu_rmse > 0.0:
        rmse_ratio = float(imu_rmse) / float(no_imu_rmse)
    matched_ratio = None
    if no_imu_matched > 0:
        matched_ratio = float(imu_matched) / float(no_imu_matched)

    checks = {
        'no_imu_path_coverage_ok': (
            no_imu_coverage is not None and no_imu_coverage >= args.min_path_coverage
        ),
        'imu_path_coverage_ok': (
            imu_coverage is not None and imu_coverage >= args.min_path_coverage
        ),
        'imu_rmse_regression_ok': (
            rmse_ratio is not None and rmse_ratio <= args.max_rmse_regression_ratio
        ),
        'imu_matched_pose_ratio_ok': (
            matched_ratio is not None and matched_ratio >= args.min_matched_pose_ratio
        ),
    }
    overall_ok = all(checks.values())
    return {
        'label': case['label'],
        'no_imu_path': case['no_imu_path'],
        'imu_path': case['imu_path'],
        'no_imu_rmse_m': float(no_imu_rmse),
        'imu_rmse_m': float(imu_rmse),
        'rmse_ratio': rmse_ratio,
        'no_imu_matched_poses': int(no_imu_matched),
        'imu_matched_poses': int(imu_matched),
        'matched_pose_ratio': matched_ratio,
        'no_imu_path_coverage': no_imu_coverage,
        'imu_path_coverage': imu_coverage,
        'checks': checks,
        'overall_ok': overall_ok,
    }


def _write_markdown(path: Path, results: list[dict], args: argparse.Namespace) -> None:
    lines = [
        '# Packet IMU Deskew Validation',
        '',
        'Acceptance criteria:',
        f'- path coverage for `no_imu` and `imu` >= `{args.min_path_coverage:.2f}`',
        f'- `imu_rmse / no_imu_rmse` <= `{args.max_rmse_regression_ratio:.2f}`',
        f'- `imu_matched_poses / no_imu_matched_poses` >= `{args.min_matched_pose_ratio:.2f}`',
        '',
        '| case | no-IMU RMSE (m) | IMU RMSE (m) | RMSE ratio | no-IMU coverage | IMU coverage | no-IMU matched | IMU matched | matched ratio | status |',
        '| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |',
    ]
    for result in results:
        lines.append(
            '| '
            + result['label']
            + ' | '
            + _round(result['no_imu_rmse_m'])
            + ' | '
            + _round(result['imu_rmse_m'])
            + ' | '
            + _round(result['rmse_ratio'])
            + ' | '
            + _round(result['no_imu_path_coverage'])
            + ' | '
            + _round(result['imu_path_coverage'])
            + ' | '
            + str(result['no_imu_matched_poses'])
            + ' | '
            + str(result['imu_matched_poses'])
            + ' | '
            + _round(result['matched_pose_ratio'])
            + ' | '
            + ('PASS' if result['overall_ok'] else 'FAIL')
            + ' |'
        )
    failing = [result['label'] for result in results if not result['overall_ok']]
    lines.extend(['', 'Overall: ' + ('PASS' if not failing else 'FAIL')])
    if failing:
        lines.append(f'- failing cases: {", ".join(failing)}')
    path.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', required=True, help='Matrix root dir containing <case>/{no_imu,imu}/metrics.json')
    parser.add_argument('--write-md', required=True, help='Output markdown report path')
    parser.add_argument('--write-json', required=True, help='Output JSON summary path')
    parser.add_argument('--min-path-coverage', type=float, default=0.95)
    parser.add_argument('--max-rmse-regression-ratio', type=float, default=1.10)
    parser.add_argument('--min-matched-pose-ratio', type=float, default=0.80)
    args = parser.parse_args()

    root = Path(args.root).expanduser().resolve()
    write_md = Path(args.write_md).expanduser().resolve()
    write_json = Path(args.write_json).expanduser().resolve()
    if not root.is_dir():
        print(f'error: root not found: {root}')
        return 1

    cases = _get_case_metrics(root)
    if not cases:
        print(f'error: no complete no_imu/imu case pairs found under {root}')
        return 1

    results = [_summarize_case(case, args) for case in cases]
    payload = {
        'root': str(root),
        'thresholds': {
            'min_path_coverage': args.min_path_coverage,
            'max_rmse_regression_ratio': args.max_rmse_regression_ratio,
            'min_matched_pose_ratio': args.min_matched_pose_ratio,
        },
        'results': results,
    }
    write_md.parent.mkdir(parents=True, exist_ok=True)
    write_json.parent.mkdir(parents=True, exist_ok=True)
    _write_markdown(write_md, results, args)
    write_json.write_text(json.dumps(payload, indent=2), encoding='utf-8')

    failing = [result['label'] for result in results if not result['overall_ok']]
    print(f'cases: {len(results)}')
    print(f'write_md: {write_md}')
    print(f'write_json: {write_json}')
    if failing:
        print(f'error: packet IMU deskew validation failed for: {", ".join(failing)}')
        return 2
    best = min(results, key=lambda item: item['imu_rmse_m'])
    print(
        'best_imu_case: '
        f"{best['label']} ({best['imu_rmse_m']:.3f}m, "
        f"coverage={best['imu_path_coverage']:.3f})"
    )
    return 0


if __name__ == '__main__':
    sys.exit(main())
