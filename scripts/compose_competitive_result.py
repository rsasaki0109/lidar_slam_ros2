#!/usr/bin/env python3
"""Compose runner artifacts into the strict competitive sequence-gate schema."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import statistics
import sys
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROFILE = ROOT / 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml'


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _runtime_value(run: dict[str, Any], key: str) -> Any:
    runtime = run.get('runtime') or {}
    return runtime.get(key)


def _completion(run: dict[str, Any]) -> tuple[bool, int | None]:
    completion = run.get('completion') or run
    complete = bool(completion.get('trajectory_complete'))
    status = completion.get('process_exit_status')
    return complete, None if status is None else int(status)


def compose(*, system: str, track: str, manifest_path: Path,
            reference_path: Path, machine_path: Path,
            trajectory_path: Path, map_path: Path,
            profile_path: Path = DEFAULT_PROFILE,
            processing_path: Path | None = None,
            loop_path: Path | None = None,
            visual_path: Path | None = None) -> dict[str, Any]:
    contract = yaml.safe_load(profile_path.read_text())['competitive_slam_profile']
    if track not in contract['tracks']:
        raise ValueError(f'unknown competition track: {track}')
    manifest = json.loads(manifest_path.read_text())
    if manifest.get('status') != 'frozen':
        raise ValueError('input manifest is not frozen')
    machine = json.loads(machine_path.read_text())
    machine_id = machine.get('machine_id')
    if not isinstance(machine_id, str) or len(machine_id) != 64:
        raise ValueError('machine artifact has no SHA-256 machine_id')
    trajectory = json.loads(trajectory_path.read_text())
    mapping_source = json.loads(map_path.read_text())
    aggregate = trajectory['aggregate']
    runs = trajectory['runs']
    valid_repetitions = int(trajectory['valid_repetitions'])
    failures = sum(
        not complete or status != 0
        for complete, status in (_completion(run) for run in runs))

    rss_values = [
        float(value) for run in runs
        for value in [_runtime_value(run, 'peak_rss_mb')]
        if value is not None]
    peak_rss = aggregate.get('peak_rss_max_mb')
    if peak_rss is None:
        if not rss_values:
            raise ValueError('trajectory summary has no peak RSS evidence')
        peak_rss = max(rss_values)
    runtime: dict[str, Any] = {'peak_rss_max_mb': float(peak_rss)}

    if processing_path is not None:
        processing = json.loads(processing_path.read_text())
        if processing.get('valid_processing_rtf_evidence'):
            runtime['processing_rtf_median'] = float(
                processing['processing_rtf_upper_bound_median'])
    else:
        rtf_values = [
            float(value) for run in runs
            for value in [_runtime_value(run, 'processing_realtime_factor')]
            if value is not None]
        if len(rtf_values) == valid_repetitions:
            runtime['processing_rtf_median'] = statistics.median(rtf_values)

    map_valid_repetitions = int(mapping_source['valid_repetitions'])
    map_valid = bool(mapping_source.get(
        'aggregation_valid', mapping_source.get('aggregate') is not None))
    meaningful = int(mapping_source.get(
        'meaningful_repetitions',
        map_valid_repetitions if map_valid else 0))
    mapping: dict[str, Any] = {
        'aggregation_valid': map_valid,
        'valid_repetitions': map_valid_repetitions,
        'meaningful_repetitions': meaningful,
    }
    map_aggregate = mapping_source.get('aggregate')
    if map_valid:
        if not isinstance(map_aggregate, dict):
            raise ValueError('valid map summary has no aggregate')
        mapping.update({
            'plane_thickness_mean_worst_m': float(
                map_aggregate['plane_thickness_mean_worst_m']),
            'plane_thickness_p95_worst_m': float(
                map_aggregate['plane_thickness_p95_worst_m']),
            'planar_coverage_worst': float(
                map_aggregate['planar_coverage_worst']),
        })

    result: dict[str, Any] = {
        'schema_version': 1,
        'system': system,
        'sequence': manifest['sequence'],
        'track': track,
        'input_manifest_sha256': sha256(manifest_path),
        'reference_sha256': sha256(reference_path),
        'calibration_sha256': manifest['hashes']['calibration_archive_sha256'],
        'machine_id': machine_id,
        'excluded_capabilities': contract['excluded_capabilities'],
        'repetitions': {'valid': valid_repetitions, 'failures': failures},
        'trajectory': {
            'ape_rmse_median_m': float(aggregate['ape_rmse_median_m'])},
        'runtime': runtime,
        'mapping': mapping,
        'provenance': {
            'profile_sha256': sha256(profile_path),
            'machine_artifact_sha256': sha256(machine_path),
            'trajectory_summary_sha256': sha256(trajectory_path),
            'map_summary_sha256': sha256(map_path),
        },
    }
    if processing_path is not None:
        result['provenance']['processing_validation_sha256'] = sha256(
            processing_path)
    if loop_path is not None:
        loop_source = json.loads(loop_path.read_text())
        loop = loop_source.get('loop_closure', loop_source)
        result['loop_closure'] = {
            'verified_false_edges': int(loop['verified_false_edges'])}
        result['provenance']['loop_report_sha256'] = sha256(loop_path)
    if visual_path is not None:
        visual_source = json.loads(visual_path.read_text())
        visual = visual_source.get('visual', visual_source.get(
            'aggregate', visual_source))
        result['visual'] = {
            'heldout_rgb_l2_median': float(visual['heldout_rgb_l2_median']),
            'heldout_rgb_inlier_20': float(visual['heldout_rgb_inlier_20']),
        }
        result['provenance']['visual_report_sha256'] = sha256(visual_path)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--system', required=True)
    parser.add_argument('--track', required=True)
    parser.add_argument('--input-manifest', type=Path, required=True)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--machine', type=Path, required=True)
    parser.add_argument('--trajectory-summary', type=Path, required=True)
    parser.add_argument('--map-summary', type=Path, required=True)
    parser.add_argument('--processing-validation', type=Path)
    parser.add_argument('--loop-report', type=Path)
    parser.add_argument('--visual-report', type=Path)
    parser.add_argument('--profile', type=Path, default=DEFAULT_PROFILE)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError(f'refusing to overwrite: {args.output}')
    result = compose(
        system=args.system, track=args.track,
        manifest_path=args.input_manifest, reference_path=args.reference,
        machine_path=args.machine, trajectory_path=args.trajectory_summary,
        map_path=args.map_summary, profile_path=args.profile,
        processing_path=args.processing_validation, loop_path=args.loop_report,
        visual_path=args.visual_report)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n')
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError,
            yaml.YAMLError) as error:
        print(f'error: {error}', file=sys.stderr)
        sys.exit(2)
