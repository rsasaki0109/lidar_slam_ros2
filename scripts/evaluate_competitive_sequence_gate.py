#!/usr/bin/env python3
"""Evaluate one frozen-sequence head-to-head competitive SLAM gate."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROFILE = ROOT / 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml'


def nested(document: dict[str, Any], path: str) -> Any:
    value: Any = document
    for key in path.split('.'):
        if not isinstance(value, dict) or key not in value:
            raise ValueError(f'missing result field: {path}')
        value = value[key]
    return value


def finite(document: dict[str, Any], path: str) -> float:
    value = nested(document, path)
    if isinstance(value, bool) or not math.isfinite(float(value)):
        raise ValueError(f'non-finite result field: {path}')
    return float(value)


def evaluate(ours: dict[str, Any], rival: dict[str, Any],
             contract: dict[str, Any]) -> dict[str, Any]:
    policy = contract['win_policy']
    checks: dict[str, dict[str, Any]] = {}

    def check(name: str, passed: bool, evidence: Any) -> None:
        checks[name] = {'pass': bool(passed), 'evidence': evidence}

    identity_fields = ('sequence', 'track', 'input_manifest_sha256',
                       'reference_sha256', 'calibration_sha256', 'machine_id')
    identity = {field: (ours.get(field), rival.get(field))
                for field in identity_fields}
    check('identical_evaluation_contract', all(a == b and a not in (None, '')
          for a, b in identity.values()), identity)
    check('excluded_capabilities_enforced',
          ours.get('excluded_capabilities') == contract['excluded_capabilities'],
          ours.get('excluded_capabilities'))

    required_repetitions = int(contract['repetitions'])
    ours_repetitions = int(nested(ours, 'repetitions.valid'))
    rival_repetitions = int(nested(rival, 'repetitions.valid'))
    check('three_valid_repetitions',
          ours_repetitions >= required_repetitions and
          rival_repetitions >= required_repetitions,
          {'required': required_repetitions, 'ours': ours_repetitions,
           'rival': rival_repetitions})
    ours_failures = int(nested(ours, 'repetitions.failures'))
    rival_failures = int(nested(rival, 'repetitions.failures'))
    check('no_incomplete_or_catastrophic_runs',
          ours_failures == 0 and rival_failures == 0,
          {'ours': ours_failures, 'rival': rival_failures})

    ours_ape = finite(ours, 'trajectory.ape_rmse_median_m')
    rival_ape = finite(rival, 'trajectory.ape_rmse_median_m')
    improvement = 100.0 * (rival_ape - ours_ape) / rival_ape
    check('primary_accuracy_improvement',
          improvement >= float(policy['minimum_primary_improvement_percent']),
          {'ours_m': ours_ape, 'rival_m': rival_ape,
           'improvement_percent': improvement,
           'required_percent': policy['minimum_primary_improvement_percent']})

    ours_rtf = finite(ours, 'runtime.processing_rtf_median')
    check('realtime', ours_rtf <= float(policy['maximum_realtime_factor']),
          {'ours': ours_rtf, 'maximum': policy['maximum_realtime_factor']})
    ours_rss = finite(ours, 'runtime.peak_rss_max_mb')
    rival_rss = finite(rival, 'runtime.peak_rss_max_mb')
    rss_ratio = ours_rss / rival_rss
    check('peak_rss', rss_ratio <= float(policy['maximum_peak_rss_ratio_to_rival']),
          {'ours_mb': ours_rss, 'rival_mb': rival_rss, 'ratio': rss_ratio,
           'maximum_ratio': policy['maximum_peak_rss_ratio_to_rival']})

    map_tolerance = float(policy['maximum_mapping_regression_percent']) / 100.0
    ours_map_runs = int(nested(ours, 'mapping.valid_repetitions'))
    rival_map_runs = int(nested(rival, 'mapping.valid_repetitions'))
    ours_meaningful = int(nested(ours, 'mapping.meaningful_repetitions'))
    rival_meaningful = int(nested(rival, 'mapping.meaningful_repetitions'))
    ours_map_valid = bool(nested(ours, 'mapping.aggregation_valid'))
    rival_map_valid = bool(nested(rival, 'mapping.aggregation_valid'))
    map_evidence = {
        'required_repetitions': required_repetitions,
        'ours': {'valid_repetitions': ours_map_runs,
                 'meaningful_repetitions': ours_meaningful,
                 'aggregation_valid': ours_map_valid},
        'rival': {'valid_repetitions': rival_map_runs,
                  'meaningful_repetitions': rival_meaningful,
                  'aggregation_valid': rival_map_valid},
    }
    if ours_map_runs < required_repetitions or rival_map_runs < required_repetitions:
        map_evidence['reason'] = 'missing repeated map evidence'
        check('mapping_non_regression', False, map_evidence)
    elif not ours_map_valid or ours_meaningful < required_repetitions:
        map_evidence['reason'] = 'ours has non-meaningful map evidence'
        check('mapping_non_regression', False, map_evidence)
    elif not rival_map_valid or rival_meaningful < required_repetitions:
        map_evidence['reason'] = (
            'rival map evidence is complete but fails meaningful-quality threshold')
        check('mapping_non_regression', True, map_evidence)
    else:
        ours_mean = finite(ours, 'mapping.plane_thickness_mean_worst_m')
        rival_mean = finite(rival, 'mapping.plane_thickness_mean_worst_m')
        ours_p95 = finite(ours, 'mapping.plane_thickness_p95_worst_m')
        rival_p95 = finite(rival, 'mapping.plane_thickness_p95_worst_m')
        ours_coverage = finite(ours, 'mapping.planar_coverage_worst')
        rival_coverage = finite(rival, 'mapping.planar_coverage_worst')
        map_rows = {
            'mean_thickness': ours_mean <= rival_mean * (1.0 + map_tolerance),
            'p95_thickness': ours_p95 <= rival_p95 * (1.0 + map_tolerance),
            'planar_coverage': (
                ours_coverage >= rival_coverage * (1.0 - map_tolerance)),
        }
        map_evidence.update({
            'checks': map_rows, 'tolerance_percent': 100.0 * map_tolerance,
            'ours_metrics': {'mean': ours_mean, 'p95': ours_p95,
                             'coverage': ours_coverage},
            'rival_metrics': {'mean': rival_mean, 'p95': rival_p95,
                              'coverage': rival_coverage},
        })
        check('mapping_non_regression', all(map_rows.values()), map_evidence)
    false_loops = int(nested(ours, 'loop_closure.verified_false_edges'))
    check('zero_verified_false_loops', false_loops == 0, false_loops)

    if ours.get('track') == 'fast_livo2_lidar_imu_visual':
        colour_tolerance = float(
            policy['maximum_visual_colour_regression_percent']) / 100.0
        ours_rgb = finite(ours, 'visual.heldout_rgb_l2_median')
        rival_rgb = finite(rival, 'visual.heldout_rgb_l2_median')
        ours_inlier = finite(ours, 'visual.heldout_rgb_inlier_20')
        rival_inlier = finite(rival, 'visual.heldout_rgb_inlier_20')
        rows = {
            'rgb_l2_median': ours_rgb <= rival_rgb * (1.0 + colour_tolerance),
            'rgb_inlier_20': ours_inlier >= rival_inlier * (1.0 - colour_tolerance),
        }
        check('visual_colour_non_regression', all(rows.values()), {
            'checks': rows, 'tolerance_percent': 100.0 * colour_tolerance,
            'ours': {'median': ours_rgb, 'inlier': ours_inlier},
            'rival': {'median': rival_rgb, 'inlier': rival_inlier}})

    return {'schema_version': 1, 'sequence': ours.get('sequence'),
            'track': ours.get('track'), 'pass': all(
                row['pass'] for row in checks.values()), 'checks': checks}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ours', type=Path, required=True)
    parser.add_argument('--rival', type=Path, required=True)
    parser.add_argument('--profile', type=Path, default=DEFAULT_PROFILE)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    ours = json.loads(args.ours.read_text())
    rival = json.loads(args.rival.read_text())
    contract = yaml.safe_load(args.profile.read_text())['competitive_slam_profile']
    result = evaluate(ours, rival, contract)
    if args.output.exists():
        raise ValueError(f'refusing to overwrite: {args.output}')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n')
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result['pass'] else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError,
            yaml.YAMLError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        sys.exit(2)
