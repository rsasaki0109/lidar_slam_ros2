#!/usr/bin/env python3
"""Plan and freeze the preregistered HILTI fresh-holdout inputs.

The default ``plan`` action is read-only and performs no network or dataset
access.  ``download`` must be selected explicitly; it downloads into a
per-sequence staging directory, verifies size and hashes, and atomically
publishes a ``downloaded_hashed`` manifest.  ``finalize`` is a separate,
reviewable step which binds a converted ROS 2 tree and an independently
generated semantic-equivalence report before publishing ``frozen_unopened``.

Ground truth is never parsed by this tool.  It is treated as an opaque byte
stream: only the expected byte count and SHA-256 are recorded.  The final
state is intentionally not written back to the selection receipt or the
competitive profile; an operator must review the manifests and update those
files in a separate change.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
from typing import Any, Iterable
from urllib.parse import urlparse
import uuid

import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SELECTION = ROOT / (
    'configs/slam_benchmark_profiles/fresh_holdout_selection_2026-08.yaml')
SOURCE_REVISION_RE = re.compile(r'^[0-9a-f]{40}$')
SHA256_RE = re.compile(r'^[0-9a-f]{64}$')
GIT_BLOB_RE = re.compile(r'^[0-9a-f]{40}$')
OFFICIAL_HOST = 'huggingface.co'
MANIFEST_SCHEMA_VERSION = 1
SEMANTIC_HASH_KIND = 'rosbag_semantic_message_hash_v1'
INPUT_HASH_KIND = 'canonical_ros_input_manifest_sha256_v1'
CALIBRATION_HASH_KIND = 'canonical_calibration_tree_sha256_v1'

# This is the common HILTI/Phasma ROS 2 input contract.  The converted bag
# must expose these topics with these ROS 2 types.  Other metadata topics are
# retained in the tree hash but cannot replace a required sensor stream.
CANONICAL_TOPIC_CONTRACT = (
    ('/hesai/pandar', 'sensor_msgs/msg/PointCloud2', 'lidar'),
    ('/alphasense/imu', 'sensor_msgs/msg/Imu', 'imu'),
    ('/alphasense/cam0/image_raw', 'sensor_msgs/msg/Image', 'camera0'),
    ('/alphasense/cam1/image_raw', 'sensor_msgs/msg/Image', 'camera1'),
    ('/alphasense/cam2/image_raw', 'sensor_msgs/msg/Image', 'camera2'),
    ('/alphasense/cam3/image_raw', 'sensor_msgs/msg/Image', 'camera3'),
    ('/alphasense/cam4/image_raw', 'sensor_msgs/msg/Image', 'camera4'),
)


def _canonical_json(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(',', ':'),
                      ensure_ascii=True).encode('utf-8')


def canonical_json_sha256(value: Any) -> str:
    return hashlib.sha256(_canonical_json(value)).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def sha256_tree(path: Path) -> str:
    digest = hashlib.sha256()
    for candidate in sorted(item for item in path.rglob('*') if item.is_file()):
        relative = candidate.relative_to(path).as_posix()
        digest.update(relative.encode('utf-8'))
        digest.update(b'\0')
        with candidate.open('rb') as stream:
            for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
                digest.update(block)
    return digest.hexdigest()


def git_blob_sha1(path: Path) -> str:
    size = path.stat().st_size
    digest = hashlib.sha1()  # noqa: S324 - Git blob identity is SHA-1 by contract.
    digest.update(f'blob {size}\0'.encode('ascii'))
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _require_sha(value: Any, label: str) -> str:
    if not isinstance(value, str) or SHA256_RE.fullmatch(value) is None:
        raise ValueError(f'{label} must be a 64-hex SHA-256')
    return value.lower()


def _require_git_blob(value: Any, label: str) -> str:
    if not isinstance(value, str) or GIT_BLOB_RE.fullmatch(value) is None:
        raise ValueError(f'{label} must be a 40-hex Git blob OID')
    return value.lower()


def _require_relative_path(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value or Path(value).is_absolute():
        raise ValueError(f'{label} must be a relative path')
    path = Path(value)
    if '..' in path.parts or '.' in path.parts:
        raise ValueError(f'{label} must not contain . or .. components')
    return path.as_posix()


def _official_file_url(repository: str, revision: str, relative: str) -> str:
    parsed = urlparse(repository)
    if parsed.scheme != 'https' or parsed.netloc != OFFICIAL_HOST:
        raise ValueError('official repository must be an HTTPS Hugging Face URL')
    if not parsed.path.startswith('/datasets/'):
        raise ValueError('official repository must point to a Hugging Face dataset')
    return (f'{repository.rstrip("/")}/resolve/{revision}/{relative}'
            '?download=true')


def _blob_to_resolve_url(url: Any, label: str) -> str:
    if not isinstance(url, str):
        raise ValueError(f'{label} URL is missing')
    parsed = urlparse(url)
    if parsed.scheme != 'https' or parsed.netloc != OFFICIAL_HOST:
        raise ValueError(f'{label} URL must use official Hugging Face HTTPS')
    pieces = parsed.path.strip('/').split('/')
    try:
        blob_index = pieces.index('blob')
    except ValueError as exc:
        raise ValueError(f'{label} URL must be a Hugging Face blob URL') from exc
    if blob_index == 0 or len(pieces) <= blob_index + 2:
        raise ValueError(f'{label} URL has no revision or file path')
    revision = pieces[blob_index + 1]
    if SOURCE_REVISION_RE.fullmatch(revision) is None:
        raise ValueError(f'{label} URL revision is not a full commit')
    repository = '/' + '/'.join(pieces[:blob_index])
    relative = '/'.join(pieces[blob_index + 2:])
    return (f'https://{OFFICIAL_HOST}{repository}/resolve/{revision}/'
            f'{relative}?download=true')


def _load_selection(path: Path) -> dict[str, Any]:
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(document, dict) or document.get('receipt_kind') != (
            'm5b_fresh_holdout_selection'):
        raise ValueError('selection receipt kind is not m5b_fresh_holdout_selection')
    source = document.get('official_source')
    if not isinstance(source, dict):
        raise ValueError('selection receipt lacks official_source')
    revision = source.get('revision')
    if not isinstance(revision, str) or SOURCE_REVISION_RE.fullmatch(revision) is None:
        raise ValueError('official source revision must be a full 40-hex commit')
    repository = source.get('repository')
    _official_file_url(repository, revision, 'probe')
    candidates = (document.get('selection_decision') or {}).get('selected_candidates')
    if not isinstance(candidates, dict) or set(candidates) != {
            'fresh_1', 'fresh_2', 'fresh_3'}:
        raise ValueError('selection receipt must contain exactly fresh_1/2/3')
    return document


def _candidate_artifacts(document: dict[str, Any], slot_name: str) -> dict[str, Any]:
    source = document['official_source']
    candidate = document['selection_decision']['selected_candidates'][slot_name]
    sequence = candidate.get('sequence')
    if not isinstance(sequence, str) or not re.fullmatch(r'exp[0-9]{2}', sequence):
        raise ValueError(f'{slot_name}.sequence is invalid')
    bag = candidate.get('bag') or {}
    gt = candidate.get('ground_truth') or {}
    bag_url = _blob_to_resolve_url(bag.get('url'), f'{slot_name}.bag')
    gt_url = _blob_to_resolve_url(gt.get('url'), f'{slot_name}.ground_truth')
    expected_prefix = f'/resolve/{source["revision"]}/'
    if (expected_prefix not in urlparse(bag_url).path or
            expected_prefix not in urlparse(gt_url).path):
        raise ValueError(f'{slot_name} artifact URL revision differs from official source')
    bag_name = Path(urlparse(bag_url).path).name
    gt_name = Path(urlparse(gt_url).path).name
    bag_bytes = bag.get('expected_bytes')
    gt_bytes = gt.get('expected_bytes')
    if (isinstance(bag_bytes, bool) or not isinstance(bag_bytes, int) or
            bag_bytes <= 0):
        raise ValueError(f'{slot_name}.bag.expected_bytes is invalid')
    if (isinstance(gt_bytes, bool) or not isinstance(gt_bytes, int) or
            gt_bytes <= 0):
        raise ValueError(f'{slot_name}.ground_truth.expected_bytes is invalid')
    bag_sha = _require_sha(bag.get('lfs_sha256'), f'{slot_name}.bag.lfs_sha256')
    gt_blob = _require_git_blob(gt.get('git_blob_oid'),
                                f'{slot_name}.ground_truth.git_blob_oid')
    calibration = source.get('calibration') or {}
    tree_path = _require_relative_path(calibration.get('tree_path'),
                                       'calibration.tree_path')
    tree_oid = _require_git_blob(calibration.get('tree_oid'),
                                 'calibration.tree_oid')
    relevant = calibration.get('relevant_files')
    if not isinstance(relevant, dict) or not relevant:
        raise ValueError('official_source.calibration.relevant_files is empty')
    calibration_files = []
    for name, metadata in sorted(relevant.items()):
        relative = _require_relative_path(name, 'calibration filename')
        metadata = metadata if isinstance(metadata, dict) else {}
        expected_bytes = metadata.get('expected_bytes')
        if (isinstance(expected_bytes, bool) or not isinstance(expected_bytes, int) or
                expected_bytes <= 0):
            raise ValueError(f'calibration {relative} expected_bytes is invalid')
        calibration_files.append({
            'path': relative,
            'url': _official_file_url(source['repository'], source['revision'],
                                      f'{tree_path}/{relative}'),
            'expected_bytes': expected_bytes,
            'expected_git_blob_sha1': _require_git_blob(
                metadata.get('git_blob_oid'), f'calibration {relative}.git_blob_oid'),
        })
    return {
        'slot': slot_name,
        'sequence': sequence,
        'dataset': candidate.get('dataset'),
        'source_revision': source['revision'],
        'bag': {
            'name': bag_name, 'url': bag_url, 'expected_bytes': bag_bytes,
            'expected_sha256': bag_sha,
        },
        'ground_truth': {
            'name': gt_name, 'url': gt_url, 'expected_bytes': gt_bytes,
            'expected_git_blob_sha1': gt_blob,
        },
        'calibration': {
            'tree_path': tree_path,
            'tree_oid': tree_oid,
            'files': calibration_files,
        },
    }


def build_plan(selection_path: Path, destination: Path) -> dict[str, Any]:
    """Build a network-free artifact plan from the preregistered receipt."""
    document = _load_selection(selection_path)
    source = document['official_source']
    slots = []
    for slot_name in sorted(
            document['selection_decision']['selected_candidates']):
        candidate = _candidate_artifacts(document, slot_name)
        sequence = candidate['sequence']
        candidate['source_repository'] = source['repository']
        candidate['selection_receipt_sha256'] = sha256_file(selection_path)
        slot_root = Path('slots') / sequence
        candidate['paths'] = {
            'stage': (Path('.staging') / sequence).as_posix(),
            'final': slot_root.as_posix(),
            'manifest': (slot_root / 'manifest.json').as_posix(),
            'state': (Path('.state') / f'{sequence}.json').as_posix(),
        }
        candidate['artifacts'] = [
            {
                'kind': 'raw_bag', 'relative_path':
                (slot_root / 'source' / candidate['bag']['name']).as_posix(),
                'stage_relative_path': (Path('source') /
                                        candidate['bag']['name']).as_posix(),
                **candidate['bag'],
            },
            {
                'kind': 'ground_truth', 'relative_path':
                (slot_root / 'ground_truth' / candidate['ground_truth']['name']).as_posix(),
                'stage_relative_path': (Path('ground_truth') /
                                        candidate['ground_truth']['name']).as_posix(),
                **candidate['ground_truth'],
            },
        ]
        for calibration_file in candidate['calibration']['files']:
            candidate['artifacts'].append({
                'kind': 'calibration',
                'relative_path': (slot_root / 'calibration' / 'files' /
                                  calibration_file['path']).as_posix(),
                'stage_relative_path': (Path('calibration') / 'files' /
                                        calibration_file['path']).as_posix(),
                'logical_path': (Path(candidate['calibration']['tree_path']) /
                                 calibration_file['path']).as_posix(),
                **calibration_file,
            })
        slots.append(candidate)
    plan = {
        'schema_version': MANIFEST_SCHEMA_VERSION,
        'plan_kind': 'competitive_fresh_holdout_download_plan',
        'producer': {
            'path': 'scripts/freeze_competitive_fresh_holdouts.py',
            'sha256': sha256_file(Path(__file__).resolve()),
        },
        'selection_receipt_path': str(selection_path),
        'selection_receipt_sha256': sha256_file(selection_path),
        'official_source': {
            'repository': source['repository'],
            'revision': source['revision'],
            'license': source.get('license'),
        },
        'destination_root': str(destination),
        'hash_contracts': {
            'calibration': CALIBRATION_HASH_KIND,
            'input_manifest': INPUT_HASH_KIND,
            'semantic_equivalence': SEMANTIC_HASH_KIND,
            'ground_truth': 'opaque_file_sha256_only_no_parser',
        },
        'canonical_topic_contract': [
            {'name': name, 'type': msg_type, 'role': role}
            for name, msg_type, role in CANONICAL_TOPIC_CONTRACT],
        'slots': slots,
    }
    plan['plan_sha256'] = canonical_json_sha256(plan)
    for slot in plan['slots']:
        slot['plan_sha256'] = plan['plan_sha256']
    return plan


def _assert_regular(path: Path, label: str) -> None:
    if path.is_symlink():
        raise ValueError(f'{label} must not be a symlink: {path}')
    if path.exists() and not path.is_file():
        raise ValueError(f'{label} must be a regular file: {path}')


def _verify_file(path: Path, artifact: dict[str, Any]) -> dict[str, Any]:
    _assert_regular(path, artifact['kind'])
    if not path.is_file():
        raise ValueError(f'missing {artifact["kind"]}: {path}')
    actual_bytes = path.stat().st_size
    if actual_bytes != artifact['expected_bytes']:
        raise ValueError(
            f'{artifact["kind"]} byte count mismatch: expected '
            f'{artifact["expected_bytes"]}, got {actual_bytes}')
    actual_sha = sha256_file(path)
    expected_sha = artifact.get('expected_sha256')
    if expected_sha is not None and actual_sha != expected_sha:
        raise ValueError(f'{artifact["kind"]} SHA-256 does not match official identity')
    observed = {
        'bytes': actual_bytes,
        'sha256': actual_sha,
    }
    expected_blob = artifact.get('expected_git_blob_sha1')
    if expected_blob is not None:
        actual_blob = git_blob_sha1(path)
        if actual_blob != expected_blob:
            raise ValueError(f'{artifact["kind"]} Git blob OID does not match')
        observed['git_blob_sha1'] = actual_blob
    return observed


def calibration_tree_sha256(files: Iterable[dict[str, Any]]) -> str:
    entries = []
    for item in sorted(files, key=lambda value: value.get('logical_path', value['path'])):
        entries.append({
            'path': item.get('logical_path', item['path']),
            'bytes': item['bytes'],
            'sha256': item['sha256'],
            'git_blob_sha1': item.get('git_blob_sha1'),
        })
    return canonical_json_sha256({
        'hash_kind': CALIBRATION_HASH_KIND,
        'files': entries,
    })


def _atomic_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    _assert_regular(path, 'JSON output')
    descriptor, temporary_name = tempfile.mkstemp(
        dir=path.parent, prefix=f'.{path.name}.', suffix='.tmp')
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
            stream.write(_canonical_json(value).decode('utf-8'))
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def _run_curl(url: str, part: Path, resume: bool) -> None:
    _assert_regular(part, 'partial download')
    command = [
        'curl', '--fail', '--silent', '--show-error', '--location',
        '--retry', '3', '--retry-all-errors', '--connect-timeout', '30',
    ]
    if resume:
        command.extend(['--continue-at', '-'])
    command.extend(['--output', str(part), url])
    subprocess.run(command, check=True)


def _materialize_artifact(stage_root: Path, artifact: dict[str, Any],
                          resume: bool) -> dict[str, Any]:
    final = stage_root / artifact['stage_relative_path']
    part = final.with_name(final.name + '.part')
    final.parent.mkdir(parents=True, exist_ok=True)
    if final.exists():
        return _verify_file(final, artifact)
    if part.exists() and not resume:
        raise ValueError(f'partial download exists; pass --resume: {part}')
    if part.exists() and resume:
        try:
            observed = _verify_file(part, artifact)
        except ValueError:
            _assert_regular(part, 'partial download')
            if part.stat().st_size >= artifact['expected_bytes']:
                raise
        else:
            _assert_regular(final, 'download destination')
            if final.exists():
                raise ValueError(f'download destination appeared concurrently: {final}')
            part.replace(final)
            return observed
    _run_curl(artifact['url'], part, resume=resume)
    observed = _verify_file(part, artifact)
    _assert_regular(final, 'download destination')
    if final.exists():
        raise ValueError(f'download destination appeared concurrently: {final}')
    part.replace(final)
    return observed


def _slot_state_path(root: Path, sequence: str) -> Path:
    return root / '.state' / f'{sequence}.json'


def _ensure_managed_root(root: Path, plan: dict[str, Any], create: bool) -> None:
    marker = root / '.freeze-root.json'
    if root.exists():
        if root.is_symlink() or not root.is_dir():
            raise ValueError(f'destination root is not a managed directory: {root}')
        if not marker.exists():
            if any(root.iterdir()):
                raise ValueError(
                    f'destination contains untracked files; refusing to manage: {root}')
            if not create:
                raise ValueError(f'destination root is not managed: {root}')
    elif create:
        root.mkdir(parents=True)
    else:
        raise ValueError(f'destination root does not exist: {root}')
    if marker.exists():
        _assert_regular(marker, 'freeze root marker')
        document = json.loads(marker.read_text(encoding='utf-8'))
        if (document.get('plan_sha256') != plan['plan_sha256'] or
                document.get('selection_receipt_sha256') !=
                plan['selection_receipt_sha256']):
            raise ValueError('managed destination identity differs from selection plan')
    elif create:
        _atomic_json(marker, {
            'schema_version': MANIFEST_SCHEMA_VERSION,
            'marker_kind': 'competitive_fresh_holdout_managed_root',
            'plan_sha256': plan['plan_sha256'],
            'selection_receipt_sha256': plan['selection_receipt_sha256'],
            'destination_root': str(root),
        })


def _calibration_manifest(plan: dict[str, Any], observed: list[dict[str, Any]]) -> dict[str, Any]:
    files = []
    for artifact, value in zip(
            [item for item in plan['artifacts'] if item['kind'] == 'calibration'],
            observed):
        files.append({
            'path': artifact['relative_path'],
            'logical_path': artifact['logical_path'],
            'bytes': value['bytes'],
            'sha256': value['sha256'],
            'git_blob_sha1': value['git_blob_sha1'],
            'source_url': artifact['url'],
        })
    return {
        'hash_kind': CALIBRATION_HASH_KIND,
        'tree_path': plan['calibration']['tree_path'],
        'tree_oid': plan['calibration']['tree_oid'],
        'files': files,
        'sha256': calibration_tree_sha256(files),
    }


def _download_slot(root: Path, plan: dict[str, Any], resume: bool) -> Path:
    sequence = plan['sequence']
    stage_root = root / plan['paths']['stage']
    final_root = root / plan['paths']['final']
    state_path = _slot_state_path(root, sequence)
    if final_root.exists():
        if stage_root.exists():
            raise ValueError(
                f'final and staging slots coexist; refusing resume: {sequence}')
        manifest_path = final_root / 'manifest.json'
        try:
            status = _verify_manifest(manifest_path, root)
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
            raise ValueError(f'final slot is not a valid managed manifest: {sequence}') from exc
        if status == 'downloaded_hashed':
            return manifest_path
        raise ValueError(f'final slot is already finalized; refusing download: {sequence}')
    if stage_root.exists() and not resume:
        raise ValueError(f'staging slot exists; pass --resume: {stage_root}')
    if resume and stage_root.exists():
        if not state_path.is_file():
            raise ValueError(f'resume state is missing: {state_path}')
        state = json.loads(state_path.read_text(encoding='utf-8'))
        if state.get('plan_sha256') != plan['plan_sha256']:
            raise ValueError(f'resume plan identity mismatch for {sequence}')
    elif resume:
        raise ValueError(f'cannot resume absent staging slot: {stage_root}')
    else:
        stage_root.mkdir(parents=True)
        state = {
            'schema_version': MANIFEST_SCHEMA_VERSION,
            'status': 'downloading',
            'plan_sha256': plan['plan_sha256'],
            'sequence': sequence,
            'artifacts': {},
        }
        _atomic_json(state_path, state)

    observed_by_kind: dict[str, list[dict[str, Any]]] = {}
    for index, artifact in enumerate(plan['artifacts']):
        observed = _materialize_artifact(stage_root, artifact, resume=resume)
        observed_by_kind.setdefault(artifact['kind'], []).append(observed)
        state['artifacts'][f'{artifact["kind"]}:{index}'] = observed
        _atomic_json(state_path, state)

    raw = observed_by_kind['raw_bag'][0]
    gt = observed_by_kind['ground_truth'][0]
    calibration = _calibration_manifest(
        plan, observed_by_kind['calibration'])
    manifest = {
        'schema_version': MANIFEST_SCHEMA_VERSION,
        'manifest_kind': 'competitive_fresh_holdout',
        'status': 'downloaded_hashed',
        'slot': plan['slot'],
        'sequence': sequence,
        'dataset': plan['dataset'],
        'source': {
            'repository': plan['source_repository'],
            'revision': plan['source_revision'],
            'selection_receipt_sha256': plan['selection_receipt_sha256'],
            'plan_sha256': plan['plan_sha256'],
        },
        'hash_policy': {
            'ground_truth': 'opaque_file_sha256_only_no_parser',
            'calibration': CALIBRATION_HASH_KIND,
            'input_manifest': INPUT_HASH_KIND,
            'semantic_equivalence': SEMANTIC_HASH_KIND,
        },
        'raw_bag': {
            'path': plan['artifacts'][0]['relative_path'],
            'bytes': raw['bytes'], 'sha256': raw['sha256'],
        },
        'ground_truth': {
            'path': plan['artifacts'][1]['relative_path'],
            'bytes': gt['bytes'], 'sha256': gt['sha256'],
            'content_opened': False,
        },
        'calibration': calibration,
        'canonical_rosbag2': None,
        'semantic_equivalence_sha256': None,
        'input_manifest_sha256': None,
        'blind_audit': {
            'ground_truth_content_opened': False,
            'trajectory_metrics_computed': False,
            'performance_data_recorded': False,
        },
    }
    _atomic_json(stage_root / 'manifest.json', manifest)
    state['status'] = 'downloaded_hashed'
    _atomic_json(state_path, state)
    final_root.parent.mkdir(parents=True, exist_ok=True)
    if final_root.exists():
        raise ValueError(f'final slot appeared concurrently: {final_root}')
    stage_root.replace(final_root)
    return final_root / 'manifest.json'


def _metadata_topics(ros2_root: Path) -> list[dict[str, str]]:
    metadata_path = ros2_root / 'metadata.yaml'
    if not metadata_path.is_file():
        raise ValueError(f'ROS 2 metadata.yaml is missing: {metadata_path}')
    document = yaml.safe_load(metadata_path.read_text(encoding='utf-8'))
    info = (document or {}).get('rosbag2_bagfile_information')
    rows = info.get('topics_with_message_count') if isinstance(info, dict) else None
    if not isinstance(rows, list):
        raise ValueError('ROS 2 metadata has no topic metadata')
    topics = []
    for row in rows:
        metadata = row.get('topic_metadata') if isinstance(row, dict) else None
        if not isinstance(metadata, dict):
            raise ValueError('ROS 2 metadata topic row is malformed')
        topics.append({
            'name': metadata.get('name'),
            'type': metadata.get('type'),
        })
    return sorted(topics, key=lambda item: (item['name'], item['type']))


def _semantic_payload(report: dict[str, Any]) -> dict[str, Any]:
    if report.get('all_topics_equal') is not True:
        raise ValueError('semantic-equivalence report is not equal')
    rows = report.get('topics')
    required_names = {item[0] for item in CANONICAL_TOPIC_CONTRACT}
    if not isinstance(rows, list):
        raise ValueError('semantic-equivalence report has no topics')
    normalized = []
    for row in rows:
        if not isinstance(row, dict) or row.get('topic') not in required_names:
            continue
        if row.get('equal') is not True:
            raise ValueError(f'semantic topic is not equal: {row.get("topic")}')
        for key in ('aggregate_sha256_left', 'aggregate_sha256_right'):
            _require_sha(row.get(key), f'semantic {row.get("topic")} {key}')
        count_left = row.get('message_count_left')
        count_right = row.get('message_count_right')
        if (isinstance(count_left, bool) or not isinstance(count_left, int) or
                count_left < 0 or count_left != count_right):
            raise ValueError(f'semantic topic count is invalid: {row.get("topic")}')
        normalized.append({
            'topic': row['topic'],
            'message_count': count_left,
            'aggregate_sha256_left': row['aggregate_sha256_left'],
            'aggregate_sha256_right': row['aggregate_sha256_right'],
        })
    if {row['topic'] for row in normalized} != required_names:
        raise ValueError('semantic report does not cover the canonical topic contract')
    return {'algorithm': SEMANTIC_HASH_KIND, 'topics': sorted(normalized,
                                                              key=lambda item: item['topic'])}


def build_ros2_identity(ros2_root: Path, semantic_report: Path) -> dict[str, Any]:
    topics = _metadata_topics(ros2_root)
    metadata_by_name = {item['name']: item['type'] for item in topics}
    for name, msg_type, _role in CANONICAL_TOPIC_CONTRACT:
        if metadata_by_name.get(name) != msg_type:
            raise ValueError(f'ROS 2 topic contract mismatch: {name}')
    report = json.loads(semantic_report.read_text(encoding='utf-8'))
    semantic = _semantic_payload(report)
    semantic_hash = canonical_json_sha256(semantic)
    topic_contract = [
        {'name': name, 'type': msg_type, 'role': role}
        for name, msg_type, role in CANONICAL_TOPIC_CONTRACT]
    return {
        'canonical_rosbag2_tree_sha256': sha256_tree(ros2_root),
        'topics': topics,
        'required_topic_contract': topic_contract,
        'semantic_equivalence_sha256': semantic_hash,
        'semantic_equivalence_hash_kind': SEMANTIC_HASH_KIND,
    }


def _finalize_manifest(path: Path, root: Path, ros2_root: Path,
                       semantic_report: Path) -> dict[str, Any]:
    manifest = json.loads(path.read_text(encoding='utf-8'))
    if manifest.get('status') != 'downloaded_hashed':
        raise ValueError(f'manifest is not downloaded_hashed: {path}')
    if _verify_manifest(path, root) != 'downloaded_hashed':
        raise ValueError(f'manifest verification did not remain downloaded_hashed: {path}')
    identity = build_ros2_identity(ros2_root, semantic_report)
    input_payload = {
        'hash_kind': INPUT_HASH_KIND,
        'raw_bag_sha256': manifest['raw_bag']['sha256'],
        'canonical_rosbag2_tree_sha256': identity['canonical_rosbag2_tree_sha256'],
        'semantic_equivalence_sha256': identity['semantic_equivalence_sha256'],
        'required_topic_contract': identity['required_topic_contract'],
    }
    finalized = dict(manifest)
    finalized['status'] = 'frozen_unopened'
    finalized['canonical_rosbag2'] = identity
    finalized['semantic_equivalence_sha256'] = identity[
        'semantic_equivalence_sha256']
    finalized['input_manifest_sha256'] = canonical_json_sha256(input_payload)
    finalized['input_manifest_hash_kind'] = INPUT_HASH_KIND
    finalized['blind_audit'] = {
        **manifest['blind_audit'],
        'canonical_rosbag2_metadata_only': True,
        'semantic_report_payload_only': True,
    }
    return finalized


def _verify_manifest(manifest_path: Path, root: Path) -> str:
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    status = manifest.get('status')
    if status not in {'downloaded_hashed', 'frozen_unopened'}:
        raise ValueError(f'unsupported manifest status: {status}')
    marker = root / '.freeze-root.json'
    if marker.exists():
        _assert_regular(marker, 'freeze root marker')
        marker_document = json.loads(marker.read_text(encoding='utf-8'))
        if manifest.get('source', {}).get('plan_sha256') != marker_document.get(
                'plan_sha256'):
            raise ValueError('manifest plan identity does not match managed root')
    # The manifest stores paths relative to the destination root.  Resolve and
    # reject traversal before hashing any artifact, especially the opaque GT.
    for key in ('raw_bag', 'ground_truth'):
        item = manifest[key]
        relative = _require_relative_path(item['path'], f'{key}.path')
        path = root / relative
        try:
            path.resolve().relative_to(root.resolve())
        except ValueError:
            raise ValueError(f'{key} escapes destination root')
        observed = _verify_file(path, {
            'kind': key, 'expected_bytes': item['bytes'],
            'expected_sha256': item['sha256'],
        })
        if observed['sha256'] != item['sha256']:
            raise ValueError(f'{key} manifest hash changed')
    calibration_entries = []
    for item in manifest['calibration']['files']:
        relative = _require_relative_path(item['path'], 'calibration.path')
        logical = _require_relative_path(item['logical_path'],
                                         'calibration.logical_path')
        path = root / relative
        try:
            path.resolve().relative_to(root.resolve())
        except ValueError:
            raise ValueError('calibration path escapes destination root')
        observed = _verify_file(path, {
            'kind': 'calibration', 'expected_bytes': item['bytes'],
            'expected_sha256': item['sha256'],
            'expected_git_blob_sha1': item['git_blob_sha1'],
        })
        if observed['sha256'] != item['sha256']:
            raise ValueError('calibration manifest hash changed')
        calibration_entries.append({
            'logical_path': logical,
            'path': relative,
            'bytes': observed['bytes'],
            'sha256': observed['sha256'],
            'git_blob_sha1': observed['git_blob_sha1'],
        })
    if calibration_tree_sha256(calibration_entries) != manifest['calibration']['sha256']:
        raise ValueError('calibration tree hash changed')
    if status == 'frozen_unopened':
        if not manifest.get('input_manifest_sha256'):
            raise ValueError('frozen manifest lacks input_manifest_sha256')
    return status


def _parse_mapping(values: list[str], label: str) -> dict[str, Path]:
    result = {}
    for value in values:
        if '=' not in value:
            raise ValueError(f'{label} must use NAME=PATH: {value}')
        name, raw_path = value.split('=', 1)
        if not name or not raw_path or name in result:
            raise ValueError(f'invalid or duplicate {label}: {value}')
        result[name] = Path(raw_path).expanduser().resolve()
    return result


def _output_json(path: Path | None, value: Any) -> None:
    if path is not None:
        _atomic_json(path, value)
    print(json.dumps(value, indent=2, sort_keys=True))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('plan', 'download', 'finalize', 'verify'),
                        nargs='?', default='plan')
    parser.add_argument('--selection', default=DEFAULT_SELECTION, type=Path)
    parser.add_argument('--root', required=True, type=Path,
                        help='explicit managed destination root; no default')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--ros2-root', action='append', default=[], metavar='SEQ=PATH')
    parser.add_argument('--semantic-report', action='append', default=[], metavar='SEQ=PATH')
    args = parser.parse_args(argv)
    selection = args.selection.expanduser().resolve()
    destination = args.root.expanduser().resolve()
    plan = build_plan(selection, destination)
    if args.action == 'plan':
        _output_json(args.output, plan)
        return 0
    if args.action == 'download':
        if args.output is not None:
            raise ValueError('--output is only valid for plan/verify')
        _ensure_managed_root(destination, plan, create=True)
        lock = destination / '.freeze.lock'
        try:
            descriptor = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
        except FileExistsError as exc:
            raise ValueError(
                f'freeze lock exists; another run may be active: {lock}') from exc
        with os.fdopen(descriptor, 'w', encoding='ascii') as stream:
            stream.write(f'{os.getpid()} {uuid.uuid4()}\n')
        try:
            manifests = []
            for slot in plan['slots']:
                manifests.append(str(_download_slot(destination, slot, args.resume)))
            _output_json(None, {'status': 'downloaded_hashed', 'manifests': manifests})
        finally:
            lock.unlink(missing_ok=True)
        return 0
    if args.action == 'verify':
        _ensure_managed_root(destination, plan, create=False)
        statuses = {}
        invalid = []
        for slot in plan['slots']:
            path = destination / slot['paths']['manifest']
            try:
                statuses[slot['sequence']] = _verify_manifest(path, destination)
            except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
                invalid.append(f'{slot["sequence"]}: {exc}')
        result = {'status': 'INVALID' if invalid else 'PASS', 'statuses': statuses,
                  'errors': invalid}
        _output_json(args.output, result)
        return 1 if invalid else 0
    if args.output is not None:
        raise ValueError('--output is only valid for plan/verify')
    ros2 = _parse_mapping(args.ros2_root, '--ros2-root')
    semantic = _parse_mapping(args.semantic_report, '--semantic-report')
    if set(ros2) != {slot['sequence'] for slot in plan['slots']} or set(semantic) != set(ros2):
        raise ValueError('--ros2-root and --semantic-report must cover all three sequences')
    _ensure_managed_root(destination, plan, create=False)
    finalized = []
    # Validate every slot before changing any manifest.  Each replacement is
    # atomic, and no profile/selection receipt is modified by this command.
    for slot in plan['slots']:
        manifest_path = destination / slot['paths']['manifest']
        if _verify_manifest(manifest_path, destination) != 'downloaded_hashed':
            raise ValueError(
                f'manifest must be downloaded_hashed before finalize: '
                f'{slot["sequence"]}')
    pending = []
    for slot in plan['slots']:
        manifest_path = destination / slot['paths']['manifest']
        pending.append((manifest_path, _finalize_manifest(
            manifest_path, destination, ros2[slot['sequence']],
            semantic[slot['sequence']])))
    for path, manifest in pending:
        _atomic_json(path, manifest)
        finalized.append(str(path))
    _output_json(None, {'status': 'frozen_unopened', 'manifests': finalized,
                        'profile_update_required': True})
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError,
            yaml.YAMLError, subprocess.CalledProcessError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        sys.exit(2)
