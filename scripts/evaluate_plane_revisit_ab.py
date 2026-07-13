#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Gate a plane-revisit OFF/ON run across trajectory, map and BIM quality."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml


def _read_ape(path: Path) -> dict[str, float]:
    values: dict[str, float] = {}
    for line in path.read_text().splitlines():
        key, separator, value = line.partition(':')
        if separator and key in {'pairs', 'rmse', 'mean', 'max'}:
            values[key] = float(value.strip())
    if 'rmse' not in values:
        raise ValueError(f'{path}: APE rmse is missing')
    return values


def _read_yaml_section(path: Path, section: str) -> dict:
    document = yaml.safe_load(path.read_text())
    if not isinstance(document, dict) or not isinstance(document.get(section), dict):
        raise ValueError(f'{path}: YAML section {section!r} is missing')
    return document[section]


def _relative_percent(before: float, after: float) -> float:
    return 0.0 if before == 0.0 else 100.0 * (after - before) / abs(before)


def evaluate(args: argparse.Namespace) -> dict:
    off_ape = _read_ape(args.off_ape)
    on_ape = _read_ape(args.on_ape)
    off_map = _read_yaml_section(args.off_map_quality, 'map_quality_report')
    on_map = _read_yaml_section(args.on_map_quality, 'map_quality_report')
    plane = _read_yaml_section(args.on_plane_report, 'plane_revisit')
    off_bim = json.loads(args.off_bim.read_text())
    on_bim = json.loads(args.on_bim.read_text())

    off_plane = off_map['plane_metrics']
    on_plane = on_map['plane_metrics']
    off_fit = off_bim['element_fit']
    on_fit = on_bim['element_fit']
    distribution_floor = (
        float(off_fit['distribution_ratio']['mean']) - args.distribution_tolerance)

    checks = {
        'constraints_present': int(plane['constraints']) > 0,
        'trajectory_rmse_improved': on_ape['rmse'] < off_ape['rmse'],
        'map_thickness_mean_improved': (
            on_plane['thickness_rms_mean_m'] <= off_plane['thickness_rms_mean_m']),
        'map_thickness_p95_improved': (
            on_plane['thickness_rms_p95_m'] <= off_plane['thickness_rms_p95_m']),
        'map_coverage_improved': (
            on_plane['planar_coverage'] >= off_plane['planar_coverage']),
        'bim_coverage_improved': (
            on_fit['coverage_ratio']['mean'] >= off_fit['coverage_ratio']['mean']),
        'bim_distance_rmse_improved': (
            on_fit['distance_rmse_m']['mean'] <= off_fit['distance_rmse_m']['mean']),
        'bim_distance_p95_improved': (
            on_fit['distance_p95_m']['mean'] <= off_fit['distance_p95_m']['mean']),
        'bim_distribution_non_regressed': (
            on_fit['distribution_ratio']['mean'] >= distribution_floor),
    }
    metrics = {
        'trajectory_rmse_m': {
            'off': off_ape['rmse'], 'on': on_ape['rmse'],
            'change_percent': _relative_percent(off_ape['rmse'], on_ape['rmse']),
            'pairs': int(on_ape.get('pairs', 0)),
        },
        'map_thickness_mean_m': {
            'off': off_plane['thickness_rms_mean_m'],
            'on': on_plane['thickness_rms_mean_m'],
            'change_percent': _relative_percent(
                off_plane['thickness_rms_mean_m'], on_plane['thickness_rms_mean_m']),
        },
        'map_thickness_p95_m': {
            'off': off_plane['thickness_rms_p95_m'],
            'on': on_plane['thickness_rms_p95_m'],
            'change_percent': _relative_percent(
                off_plane['thickness_rms_p95_m'], on_plane['thickness_rms_p95_m']),
        },
        'map_planar_coverage': {
            'off': off_plane['planar_coverage'], 'on': on_plane['planar_coverage'],
            'change_percent': _relative_percent(
                off_plane['planar_coverage'], on_plane['planar_coverage']),
        },
        'bim_coverage_mean': {
            'off': off_fit['coverage_ratio']['mean'],
            'on': on_fit['coverage_ratio']['mean'],
            'change_percent': _relative_percent(
                off_fit['coverage_ratio']['mean'], on_fit['coverage_ratio']['mean']),
        },
        'bim_distance_rmse_mean_m': {
            'off': off_fit['distance_rmse_m']['mean'],
            'on': on_fit['distance_rmse_m']['mean'],
            'change_percent': _relative_percent(
                off_fit['distance_rmse_m']['mean'], on_fit['distance_rmse_m']['mean']),
        },
        'bim_distance_p95_mean_m': {
            'off': off_fit['distance_p95_m']['mean'],
            'on': on_fit['distance_p95_m']['mean'],
            'change_percent': _relative_percent(
                off_fit['distance_p95_m']['mean'], on_fit['distance_p95_m']['mean']),
        },
        'bim_distribution_mean': {
            'off': off_fit['distribution_ratio']['mean'],
            'on': on_fit['distribution_ratio']['mean'],
            'change_percent': _relative_percent(
                off_fit['distribution_ratio']['mean'],
                on_fit['distribution_ratio']['mean']),
            'absolute_tolerance': args.distribution_tolerance,
        },
    }
    return {
        'schema_version': 1,
        'passed': all(checks.values()),
        'checks': checks,
        'plane_revisit': {
            key: plane.get(key) for key in (
                'candidate_constraints', 'constraints',
                'constraints_rejected_initial_residual', 'chi2_before', 'chi2_after')
        },
        'metrics': metrics,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--off-ape', type=Path, required=True)
    parser.add_argument('--on-ape', type=Path, required=True)
    parser.add_argument('--off-map-quality', type=Path, required=True)
    parser.add_argument('--on-map-quality', type=Path, required=True)
    parser.add_argument('--off-bim', type=Path, required=True)
    parser.add_argument('--on-bim', type=Path, required=True)
    parser.add_argument('--on-plane-report', type=Path, required=True)
    parser.add_argument('--distribution-tolerance', type=float, default=0.001)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = evaluate(args)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n')
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
