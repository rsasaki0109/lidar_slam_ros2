#!/usr/bin/env python3
"""Validate and hash one frozen HILTI 2022 competitive holdout input."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROFILE = ROOT / 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml'


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def sha256_tree(path: Path) -> str:
    digest = hashlib.sha256()
    for candidate in sorted(item for item in path.rglob('*') if item.is_file()):
        digest.update(candidate.relative_to(path).as_posix().encode())
        digest.update(b'\0')
        with candidate.open('rb') as stream:
            for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
                digest.update(block)
    return digest.hexdigest()


def find_slot(profile: dict[str, Any], sequence: str) -> tuple[str, dict[str, Any]]:
    slots = profile['competitive_slam_profile']['datasets']['holdout_slots']
    matches = [(name, slot) for name, slot in slots.items()
               if slot.get('sequence') == sequence]
    if len(matches) != 1:
        raise ValueError(f'expected exactly one holdout slot for {sequence}')
    return matches[0]


def slug_from_url(url: str) -> str:
    filename = url.rsplit('/', 1)[-1]
    if not filename.endswith('.bag'):
        raise ValueError(f'bag_url does not end in .bag: {url}')
    return filename[:-4]


def read_raw_hash(raw_bag: Path, sidecar: Path, expected_bytes: int) -> str:
    if raw_bag.exists():
        actual_bytes = raw_bag.stat().st_size
        if actual_bytes != expected_bytes:
            raise ValueError(
                f'raw bag byte count mismatch: expected {expected_bytes}, got {actual_bytes}')
        actual_hash = sha256_file(raw_bag)
        if sidecar.exists() and sidecar.read_text().strip() != actual_hash:
            raise ValueError('raw bag hash sidecar does not match the raw bag')
        return actual_hash
    if not sidecar.is_file():
        raise ValueError(f'raw bag and hash sidecar are both missing: {raw_bag}')
    value = sidecar.read_text().strip()
    if len(value) != 64:
        raise ValueError(f'invalid raw bag SHA-256 sidecar: {sidecar}')
    int(value, 16)
    return value


def metadata_summary(path: Path) -> dict[str, Any]:
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    info = document['rosbag2_bagfile_information']
    topics = sorted(
        row['topic_metadata']['name']
        for row in info['topics_with_message_count'])
    return {
        'storage_identifier': info['storage_identifier'],
        'duration_nanoseconds': info['duration']['nanoseconds'],
        'starting_time_nanoseconds': info['starting_time']['nanoseconds_since_epoch'],
        'message_count': info['message_count'],
        'topics': topics,
    }


def build_manifest(profile_path: Path, dataset_dir: Path,
                   sequence: str) -> dict[str, Any]:
    profile_document = yaml.safe_load(profile_path.read_text(encoding='utf-8'))
    slot_name, slot = find_slot(profile_document, sequence)
    slug = slug_from_url(slot['bag_url'])
    raw_bag = dataset_dir / f'{slug}.bag'
    raw_sidecar = dataset_dir / f'{sequence}_raw_bag.sha256'
    ros2_bag = dataset_dir / f'{sequence}_ros2'
    metadata = ros2_bag / 'metadata.yaml'
    gt = dataset_dir / f'{slug}_gt.txt'
    calibration = dataset_dir / 'calibration_files.zip'
    for required in (metadata, gt, calibration):
        if not required.is_file():
            raise ValueError(f'required input is missing: {required}')
    summary = metadata_summary(metadata)
    required_topics = {'/hesai/pandar', '/alphasense/imu'}
    missing = required_topics.difference(summary['topics'])
    if missing:
        raise ValueError(f'converted bag lacks required topics: {sorted(missing)}')
    ground_truth_hash = sha256_file(gt)
    calibration_hash = sha256_file(calibration)
    for key, actual in (
            ('ground_truth_sha256', ground_truth_hash),
            ('calibration_archive_sha256', calibration_hash)):
        expected = slot.get(key)
        if expected is not None and expected != actual:
            raise ValueError(
                f'{key} differs from profile: expected {expected}, got {actual}')
    return {
        'schema_version': 1,
        'profile': profile_document['competitive_slam_profile']['name'],
        'slot': slot_name,
        'sequence': sequence,
        'dataset': slot['dataset'],
        'status': 'frozen',
        'source': {
            'bag_url': slot['bag_url'],
            'ground_truth_url': slot['ground_truth_url'],
            'raw_bag_expected_bytes': slot['bag_expected_bytes'],
        },
        'hashes': {
            'raw_rosbag1_sha256': read_raw_hash(
                raw_bag, raw_sidecar, slot['bag_expected_bytes']),
            'canonical_rosbag2_tree_sha256': sha256_tree(ros2_bag),
            'ground_truth_sha256': ground_truth_hash,
            'calibration_archive_sha256': calibration_hash,
        },
        'canonical_rosbag2': summary,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sequence', required=True)
    parser.add_argument('--dataset-dir', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--profile', default=DEFAULT_PROFILE, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError(f'refusing to overwrite frozen manifest: {args.output}')
    manifest = build_manifest(args.profile.resolve(), args.dataset_dir.resolve(),
                              args.sequence)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n',
                           encoding='utf-8')
    print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        sys.exit(2)
