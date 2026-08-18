#!/usr/bin/env python3
"""Require every frozen holdout to pass both competitive SLAM tracks."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROFILE = ROOT / 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml'
REQUIRED_TRACKS = {
    'glim_cpu_lidar_imu', 'fast_livo2_lidar_imu_visual'}


def evaluate(gates: list[dict[str, Any]], contract: dict[str, Any]) -> dict[str, Any]:
    slots = contract['datasets']['holdout_slots']
    assigned = {
        slot['sequence'] for slot in slots.values()
        if slot.get('status') in {'assigned_inputs_pending_hash', 'frozen'}}
    by_pair = {(gate.get('sequence'), gate.get('track')): gate for gate in gates}
    expected = {(sequence, track) for sequence in assigned
                for track in REQUIRED_TRACKS}
    missing = sorted(expected.difference(by_pair))
    unexpected = sorted(set(by_pair).difference(expected))
    failed = sorted(pair for pair in expected
                    if pair in by_pair and not by_pair[pair].get('pass'))
    minimum = int(contract['win_policy']['minimum_holdout_wins'])
    passed_sequences = sorted(
        sequence for sequence in assigned
        if all(by_pair.get((sequence, track), {}).get('pass')
               for track in REQUIRED_TRACKS))
    checks = {
        'all_holdout_inputs_frozen': all(
            slot.get('status') == 'frozen' for slot in slots.values()),
        'all_expected_track_gates_present': not missing and not unexpected,
        'all_track_gates_pass': not missing and not failed,
        'minimum_complete_holdout_wins': len(passed_sequences) >= minimum,
    }
    return {
        'schema_version': 1, 'pass': all(checks.values()), 'checks': checks,
        'assigned_sequences': sorted(assigned),
        'required_tracks': sorted(REQUIRED_TRACKS),
        'expected_gate_count': len(expected), 'provided_gate_count': len(gates),
        'missing_gates': missing, 'unexpected_gates': unexpected,
        'failed_gates': failed, 'complete_holdout_wins': passed_sequences,
        'minimum_complete_holdout_wins': minimum,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gate', type=Path, action='append', required=True)
    parser.add_argument('--profile', type=Path, default=DEFAULT_PROFILE)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    gates = [json.loads(path.read_text()) for path in args.gate]
    contract = yaml.safe_load(args.profile.read_text())['competitive_slam_profile']
    result = evaluate(gates, contract)
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
