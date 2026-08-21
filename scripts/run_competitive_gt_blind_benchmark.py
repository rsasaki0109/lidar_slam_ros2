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
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
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

"""Plan and run the M6a fresh-holdout comparison without opening ground truth.

The default action is a read-only dry-run.  ``--preflight`` additionally
checks every frozen input identity and immutable local image.  Actual replay
requires ``--execute`` and is deliberately a separate action so a plan cannot
accidentally become a performance run.  No scorer, GT parser, or map-quality
tool is called by this driver.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
from typing import Any, Iterable

import yaml

try:
    from scripts.competitive_identity_hash import canonical_profile_sha256
except ModuleNotFoundError:  # direct ``python scripts/<tool>.py`` execution
    from competitive_identity_hash import canonical_profile_sha256  # type: ignore[no-redef]


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROFILE = ROOT / 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml'
DEFAULT_RECEIPT = ROOT / (
    'configs/slam_benchmark_profiles/competitive_execution_selection_2026-08.yaml')
DEFAULT_SELECTION = ROOT / (
    'configs/slam_benchmark_profiles/fresh_holdout_selection_2026-08.yaml')
SYSTEM_ORDER = ('ours', 'glim_cpu', 'fast_livo2')
SLOT_ORDER = ('fresh_1', 'fresh_2', 'fresh_3')
SYSTEM_RECEIPT_NAMES = {
    'ours': 'ours', 'glim_cpu': 'glim', 'fast_livo2': 'fast_livo2'}
EXPECTED_LABELS = {
    'ours': ('benchmark.ours.revision', 'benchmark.rko_lio.initialized'),
    'glim_cpu': ('benchmark.glim.revision', 'benchmark.glim_ros2.revision'),
    'fast_livo2': ('benchmark.fast_livo2.revision',),
}
THREAD_ENV = {
    'OMP_NUM_THREADS': '8',
    'OPENBLAS_NUM_THREADS': '8',
    'MKL_NUM_THREADS': '8',
    'TBB_NUM_THREADS': '8',
}
DENY_WORDS = ('ground_truth', 'ground-truth', 'ape', 'scorer', 'map_quality')
SHA_RE = re.compile(r'^[0-9a-f]{64}$')
CONTAINER_SHA_RE = re.compile(r'^sha256:[0-9a-f]{64}$')


class ContractError(ValueError):
    """Raised when a plan or attempt would violate the frozen contract."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def sha256_tree(path: Path) -> str:
    if not path.is_dir():
        raise ContractError(f'input tree is not a directory: {path}')
    digest = hashlib.sha256()
    files = sorted(item for item in path.rglob('*') if item.is_file())
    if not files:
        raise ContractError(f'input tree is empty: {path}')
    for item in files:
        digest.update(item.relative_to(path).as_posix().encode('utf-8'))
        digest.update(b'\0')
        with item.open('rb') as stream:
            for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
                digest.update(block)
    return digest.hexdigest()


def canonical_sha(value: Any) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(',', ':'),
                         ensure_ascii=True).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


def load_yaml(path: Path) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(value, dict):
        raise ContractError(f'{path}: expected a mapping')
    return value


def resolve_inside(root: Path, value: str, *, label: str) -> Path:
    path = Path(value)
    resolved = (path if path.is_absolute() else root / path).resolve(strict=True)
    root_resolved = root.resolve(strict=True)
    if not resolved.is_relative_to(root_resolved):
        raise ContractError(f'{label} escapes managed root: {value}')
    return resolved


def path_contains(parent: Path, child: Path) -> bool:
    try:
        child.resolve(strict=False).relative_to(parent.resolve(strict=False))
        return True
    except ValueError:
        return False


def assert_roots_are_disjoint(input_root: Path, output_root: Path) -> None:
    input_real = input_root.resolve(strict=True)
    output_real = output_root.resolve(strict=False)
    if input_real == output_real or path_contains(input_real, output_real) or \
            path_contains(output_real, input_real):
        raise ContractError('input and output roots must be disjoint and non-containing')


def schedule() -> list[dict[str, Any]]:
    result = []
    index = 1
    for system in SYSTEM_ORDER:
        for slot in SLOT_ORDER:
            for repetition in range(1, 4):
                result.append({'schedule_index': index, 'system': system,
                               'slot': slot, 'repetition': repetition})
                index += 1
    return result


def schedule_sha(value: Iterable[dict[str, Any]] | None = None) -> str:
    return canonical_sha(list(schedule() if value is None else value))


def selection_path(receipt: dict[str, Any], profile: dict[str, Any]) -> Path:
    common = receipt.get('common_identity')
    if not isinstance(common, dict):
        raise ContractError('receipt.common_identity is missing')
    value = common.get('fresh_holdout_selection_receipt_path')
    if not isinstance(value, str):
        value = profile.get('evidence_gate_v2', {}).get(
            'fresh_holdout_selection_receipt_path')
    if not isinstance(value, str):
        return DEFAULT_SELECTION
    return (ROOT / value).resolve() if not Path(value).is_absolute() else Path(value).resolve()


def _slot_identity(profile: dict[str, Any], slot_id: str) -> dict[str, Any]:
    slots = profile.get('datasets', {}).get('fresh_holdout_slots', {})
    slot = slots.get(slot_id)
    if not isinstance(slot, dict):
        raise ContractError(f'missing fresh holdout slot: {slot_id}')
    if slot.get('status') != 'frozen_unopened':
        raise ContractError(f'{slot_id} is not frozen_unopened')
    identity = slot.get('frozen_identity')
    if not isinstance(identity, dict):
        raise ContractError(f'{slot_id}.frozen_identity is missing')
    return slot


def resolve_slot(input_root: Path, profile: dict[str, Any], slot_id: str) -> dict[str, Any]:
    slot = _slot_identity(profile, slot_id)
    identity = slot['frozen_identity']
    raw = identity.get('raw_bag_path')
    if not isinstance(raw, str):
        raw = identity.get('raw_bag', {}).get('path')
    if not isinstance(raw, str):
        raise ContractError(f'{slot_id}: raw bag path missing')
    manifest_value = identity.get('manifest_path')
    if not isinstance(manifest_value, str):
        raise ContractError(f'{slot_id}: frozen manifest path missing')
    sequence = slot.get('sequence')
    if not isinstance(sequence, str):
        raise ContractError(f'{slot_id}: sequence missing')
    canonical = (input_root / 'slots' / sequence / 'canonical_ros2').resolve(strict=True)
    raw_path = resolve_inside(input_root, raw, label=f'{slot_id}.raw_bag_path')
    manifest = resolve_inside(input_root, manifest_value, label=f'{slot_id}.manifest_path')
    if not raw_path.is_file() or not manifest.is_file():
        raise ContractError(f'{slot_id}: raw bag or manifest is not a file')
    manifest_doc = load_yaml(manifest) if manifest.suffix in ('.yaml', '.yml') else json.loads(
        manifest.read_text(encoding='utf-8'))
    if not isinstance(manifest_doc, dict) or manifest_doc.get('status') != 'frozen_unopened':
        raise ContractError(f'{slot_id}: managed manifest is not frozen_unopened')
    expected_manifest_sha = identity.get('manifest_file_sha256')
    if expected_manifest_sha is None:
        expected_manifest_sha = identity.get('manifest_file_sha256')
    if not isinstance(expected_manifest_sha, str) or not SHA_RE.fullmatch(expected_manifest_sha):
        raise ContractError(f'{slot_id}: manifest SHA is missing or malformed')
    if sha256_file(manifest) != expected_manifest_sha:
        raise ContractError(f'{slot_id}: frozen manifest SHA mismatch')
    manifest_raw = manifest_doc.get('raw_bag')
    manifest_ros = manifest_doc.get('canonical_rosbag2')
    manifest_calibration = manifest_doc.get('calibration')
    expected_raw_sha = slot.get('bag_sha256') or identity.get('raw_bag_sha256') or \
        identity.get('raw_bag', {}).get('sha256')
    expected_ros_sha = identity.get('canonical_rosbag2_tree_sha256')
    expected_semantic_sha = identity.get('semantic_equivalence_sha256')
    expected_input_sha = slot.get('input_manifest_sha256')
    expected_calibration_sha = slot.get('calibration_archive_sha256')
    if not isinstance(manifest_raw, dict) or manifest_raw.get('sha256') != expected_raw_sha:
        raise ContractError(f'{slot_id}: raw bag manifest identity mismatch')
    if not isinstance(manifest_ros, dict) or \
            manifest_ros.get('canonical_rosbag2_tree_sha256') != expected_ros_sha or \
            manifest_ros.get('semantic_equivalence_sha256') != expected_semantic_sha:
        raise ContractError(f'{slot_id}: canonical/semantic manifest identity mismatch')
    if manifest_doc.get('semantic_equivalence_sha256') != expected_semantic_sha:
        raise ContractError(f'{slot_id}: semantic manifest identity mismatch')
    if manifest_doc.get('input_manifest_sha256') != expected_input_sha:
        raise ContractError(f'{slot_id}: input manifest identity mismatch')
    if not isinstance(manifest_calibration, dict) or \
            manifest_calibration.get('sha256') != expected_calibration_sha:
        raise ContractError(f'{slot_id}: calibration manifest identity mismatch')
    gt_value = identity.get('ground_truth_path')
    if not isinstance(gt_value, str):
        gt_value = identity.get('ground_truth', {}).get('path')
    if not isinstance(gt_value, str):
        raise ContractError(f'{slot_id}: ground-truth path missing from frozen identity')
    gt_path = resolve_inside(input_root, gt_value, label=f'{slot_id}.ground_truth_path')
    gt_stat = gt_path.stat()  # metadata only; contents are never opened or hashed
    raw_sha = expected_raw_sha
    raw_bytes = slot.get('bag_expected_bytes') or identity.get('raw_bag_bytes') or \
        identity.get('raw_bag', {}).get('bytes')
    return {
        'slot': slot_id, 'sequence': sequence, 'raw_path': raw_path,
        'canonical_path': canonical, 'manifest_path': manifest,
        'manifest_sha256': expected_manifest_sha, 'gt_realpath': gt_path,
        'gt_device': gt_stat.st_dev, 'gt_inode': gt_stat.st_ino,
        'raw_sha256': raw_sha, 'raw_bytes': raw_bytes,
        'canonical_tree_sha256': expected_ros_sha,
        'semantic_sha256': expected_semantic_sha,
        'input_manifest_sha256': expected_input_sha,
        'calibration_sha256': expected_calibration_sha,
    }


def image_ref_and_labels(
        system: str, receipt: dict[str, Any], *, inspect: bool
        ) -> tuple[str, dict[str, str], str]:
    name = SYSTEM_RECEIPT_NAMES[system]
    container = receipt.get('systems', {}).get(name, {}).get('container', {})
    tag = container.get('image_tag')
    digest = container.get('image_digest')
    if (not isinstance(tag, str) or not isinstance(digest, str) or
            not CONTAINER_SHA_RE.fullmatch(digest)):
        raise ContractError(f'{system}: immutable image tag/digest is incomplete')
    ref = f'{tag}@{digest}'
    if not inspect:
        return ref, {}, digest
    try:
        result = subprocess.run(
            ['docker', 'image', 'inspect', ref], check=True, text=True,
            capture_output=True)
    except (OSError, subprocess.CalledProcessError) as exc:
        raise ContractError(f'{system}: immutable image unavailable: {ref}') from exc
    documents = json.loads(result.stdout)
    if not documents:
        raise ContractError(f'{system}: docker inspect returned no image: {ref}')
    document = documents[0]
    image_id = document.get('Id')
    if image_id != digest:
        raise ContractError(f'{system}: inspected image ID differs from receipt digest')
    labels = document.get('Config', {}).get('Labels') or {}
    expected = receipt.get('systems', {}).get(name, {}).get('repository', {})
    for label in EXPECTED_LABELS[system]:
        if label.endswith('initialized'):
            if labels.get(label) != 'true':
                raise ContractError(f'{system}: RKO-LIO image is not initialized')
        else:
            key = label
            expected_value = expected.get('revision') if system == 'ours' else None
            if system == 'glim_cpu':
                expected_value = (receipt['systems']['glim']['repository'].get(
                    'ros2_revision') if label == 'benchmark.glim_ros2.revision' else
                    receipt['systems']['glim']['repository'].get('revision'))
            if system == 'fast_livo2':
                expected_value = receipt['systems']['fast_livo2']['repository'].get('revision')
            if expected_value is not None and labels.get(key) != expected_value:
                raise ContractError(f'{system}: image label {key} does not match receipt')
    if system == 'ours':
        recipe = container.get('recipe')
        submodules = recipe.get('submodules') if isinstance(recipe, dict) else None
        rko = submodules.get('rko_lio') if isinstance(submodules, dict) else None
        ndt = submodules.get('ndt_omp_ros2') if isinstance(submodules, dict) else None
        expected_labels = {
            'benchmark.rko_lio.gitlink_revision':
                rko.get('gitlink_revision') if isinstance(rko, dict) else None,
            'benchmark.rko_lio.archive_sha256':
                rko.get('archive_sha256') if isinstance(rko, dict) else None,
            'benchmark.ndt_omp.submodule_revision':
                ndt.get('revision') if isinstance(ndt, dict) else None,
        }
        for label, expected_value in expected_labels.items():
            if not isinstance(expected_value, str) or labels.get(label) != expected_value:
                raise ContractError(f'ours: image label {label} does not match receipt')
    return ref, {str(key): str(value) for key, value in labels.items()}, digest


def _slot_mounts(item: dict[str, Any], system: str) -> list[tuple[Path, str, str]]:
    if system in ('ours', 'glim_cpu'):
        return [(item['canonical_path'], '/input/canonical_ros2', 'ro')]
    return [(item['raw_path'], '/input/raw_input.bag', 'ro')]


def _guard_gt(
        item: dict[str, Any], mounts: list[tuple[Path, str, str]],
        argv: list[str], env: dict[str, str]) -> dict[str, Any]:
    gt_path = item['gt_realpath']
    for source, _, _ in mounts:
        if path_contains(source, gt_path):
            raise ContractError(f'GT is reachable through {source}')
    serialized = '\0'.join(argv + [f'{key}={value}' for key, value in sorted(env.items())])
    lowered = serialized.lower()
    if any(word in lowered for word in DENY_WORDS):
        raise ContractError('GT/scorer/map-quality token leaked into container argv or env')
    if str(gt_path) in serialized:
        raise ContractError('frozen GT realpath leaked into container command')
    return {'ground_truth_reachable': False, 'gt_device': item['gt_device'],
            'gt_inode': item['gt_inode'], 'mount_sources': [str(x[0]) for x in mounts]}


def docker_command(
        system: str, item: dict[str, Any], image_ref: str,
        output_dir: Path, schedule_item: dict[str, Any]
        ) -> tuple[list[str], dict[str, str]]:
    run_id = f'{system}-{schedule_item["schedule_index"]:03d}'
    env = dict(THREAD_ENV)
    env.update({'ROS_DOMAIN_ID': str(200 + schedule_item['schedule_index']),
                'RUN_NAME': run_id, 'GT_BLIND': '1'})
    if system == 'glim_cpu':
        env.update({'ROS_HOME': '/out/ros_home', 'ROS_LOG_DIR': '/out/ros_log'})
    elif system == 'fast_livo2':
        env.update({'ROS_MASTER_URI': 'http://127.0.0.1:11311',
                    'ROS_IP': '127.0.0.1', 'ROS_HOSTNAME': '127.0.0.1'})
    common = [
        'taskset', '--cpu-list', '0-7', '/usr/bin/time', '-v',
        '-o', str(output_dir / 'host_time.txt'), 'docker', 'run', '--rm', '--init',
        '--network', 'none', '--cpuset-cpus', '0-7', '--read-only',
        '--tmpfs', '/tmp:rw,noexec,nosuid,size=1024m', '--shm-size', '512m']
    for key, value in env.items():
        common.extend(['-e', f'{key}={value}'])
    common.extend(['-v', f'{ROOT}:/runner:ro', '-v', f'{output_dir}:/out:rw'])
    if system == 'ours':
        common.extend([
            '-e', 'BAG_PATH=/input/canonical_ros2',
            '-e', 'OUT_DIR=/out', '-e', 'LIDAR_TOPIC=/hesai/pandar',
            '-e', 'IMU_TOPIC=/alphasense/imu', '-e', 'LIDARSLAM_WS_ROOT=/opt/ours_ws',
            '-v', f"{item['canonical_path']}:/input/canonical_ros2:ro",
            '--entrypoint', '/bin/bash', image_ref,
            '/runner/scripts/ours_container_gt_blind_run.sh'])
    elif system == 'glim_cpu':
        common.extend([
            '-e', 'BAG_PATH=/input/canonical_ros2',
            '-v', f"{item['canonical_path']}:/input/canonical_ros2:ro",
            '--entrypoint', '/bin/bash', image_ref,
            '/runner/scripts/glim_container_gt_blind_run.sh'])
    else:
        common.extend([
            '-e', 'BAG_PATH=/input/raw_input.bag', '-e', 'RATE=1.0',
            '-e', 'SHUTDOWN_GRACE_SECONDS=5', '-e', 'SAVE_MAP=1',
            '-v', f"{item['raw_path']}:/input/raw_input.bag:ro",
            '-v', f'{output_dir}/fast_log:/bench/FAST-LIVO2/Log:rw',
            '--entrypoint', '/bin/bash', image_ref,
            '/runner/scripts/fast_livo2_container_gt_blind_run.sh'])
    return common, env


def system_identity(receipt: dict[str, Any], system: str) -> dict[str, Any]:
    name = SYSTEM_RECEIPT_NAMES[system]
    document = receipt.get('systems', {}).get(name)
    if not isinstance(document, dict):
        raise ContractError(f'{system}: receipt system identity is missing')
    repository = document.get('repository')
    container = document.get('container')
    toolchain = document.get('toolchain')
    if not isinstance(repository, dict) or not isinstance(container, dict) or \
            not isinstance(toolchain, dict):
        raise ContractError(f'{system}: repository/container/toolchain identity is incomplete')
    revision = repository.get('revision')
    config = document.get('configs')
    if not isinstance(revision, str) or not isinstance(config, list):
        raise ContractError(f'{system}: revision/config identity is incomplete')
    return {
        'revision': revision,
        'revision_status': repository.get('revision_status'),
        'container_digest': container.get('image_digest'),
        'toolchain_fingerprint': toolchain.get('fingerprint'),
        'config': config,
        'runner': document.get('runner'),
        'recipe': container.get('recipe'),
    }


def verify_input_identity(item: dict[str, Any], checked_slots: set[str]) -> None:
    """Verify one frozen slot at most once during a multi-run preflight.

    The schedule deliberately repeats each slot for every system and
    repetition.  Re-reading the same raw/canonical trees for every attempt
    would multiply a large, read-only hash operation by nine without adding
    evidence; the slot identity is immutable for the entire plan.
    """
    slot = item['slot']
    if slot in checked_slots:
        return
    actual_raw_bytes = item['raw_path'].stat().st_size
    if actual_raw_bytes != item['raw_bytes']:
        raise ContractError(f'{slot}: raw bag byte count mismatch')
    if sha256_file(item['raw_path']) != item['raw_sha256']:
        raise ContractError(f'{slot}: raw bag SHA mismatch')
    if sha256_tree(item['canonical_path']) != item['canonical_tree_sha256']:
        raise ContractError(f'{slot}: canonical ROS2 tree SHA mismatch')
    checked_slots.add(slot)


def build_plan(profile_doc: dict[str, Any], receipt: dict[str, Any], selection: dict[str, Any],
               input_root: Path, profile_path: Path, receipt_path: Path,
               selection_path_value: Path, *, inspect_images: bool) -> dict[str, Any]:
    profile = profile_doc.get('competitive_slam_profile', profile_doc)
    if not isinstance(profile, dict):
        raise ContractError('profile has no competitive_slam_profile')
    if receipt.get('status') not in ('ready', 'frozen'):
        raise ContractError('execution identity receipt is not ready/frozen')
    if selection.get('status') != 'frozen_unopened':
        raise ContractError('fresh selection receipt is not frozen_unopened')
    profile_sha = canonical_profile_sha256(profile_doc)
    receipt_sha = sha256_file(receipt_path)
    selection_sha = sha256_file(selection_path_value)
    expected_selection_sha = receipt.get('common_identity', {}).get(
        'fresh_holdout_selection_receipt_sha256')
    if expected_selection_sha != selection_sha:
        raise ContractError('selection receipt SHA differs from execution receipt')
    expected_profile_sha = receipt.get('common_identity', {}).get('profile_sha256')
    if expected_profile_sha != profile_sha:
        raise ContractError('profile canonical SHA differs from execution receipt')
    image_info = {}
    for system in SYSTEM_ORDER:
        image_info[system] = image_ref_and_labels(
            system, receipt, inspect=inspect_images)
    plans = []
    checked_input_slots: set[str] = set()
    for schedule_item in schedule():
        item = resolve_slot(input_root, profile, schedule_item['slot'])
        if inspect_images:
            verify_input_identity(item, checked_input_slots)
        image_ref, labels, image_digest = image_info[schedule_item['system']]
        execution_identity = system_identity(receipt, schedule_item['system'])
        output_placeholder = Path('/M6A_OUTPUT_PLACEHOLDER')
        command, env = docker_command(
            schedule_item['system'], item, image_ref, output_placeholder,
            schedule_item)
        mounts = _slot_mounts(item, schedule_item['system'])
        guard = _guard_gt(item, mounts, command, env)
        plans.append({
            **schedule_item, 'sequence': item['sequence'],
            'image_ref': image_ref, 'image_digest': image_digest,
            'image_labels': labels, 'argv': command, 'env': env,
            'execution_identity': execution_identity,
            'mounts': [{'source': str(source), 'target': target, 'mode': mode}
                       for source, target, mode in mounts],
            'input': {
                'raw_bag_sha256': item['raw_sha256'],
                'raw_bag_bytes': item['raw_bytes'],
                'canonical_rosbag2_tree_sha256': item['canonical_tree_sha256'],
                'semantic_equivalence_sha256': item['semantic_sha256'],
                'input_manifest_sha256': item['input_manifest_sha256'],
                'calibration_tree_sha256': item['calibration_sha256'],
                'manifest_file_sha256': item['manifest_sha256'],
            }, 'gt_blind_guard': guard,
        })
    identity = {
        'profile_canonical_sha256': profile_sha,
        'execution_receipt_file_sha256': receipt_sha,
        'selection_receipt_file_sha256': selection_sha,
        'schedule_sha256': schedule_sha(),
        'systems': list(SYSTEM_ORDER), 'slots': list(SLOT_ORDER),
        'repetitions': 3, 'ground_truth_content_opened': False,
        'scorer_invoked': False,
    }
    return {'schema_version': 1, 'kind': 'm6a_gt_blind_plan',
            'status': 'preflight_ready' if inspect_images else 'planned',
            'identity': identity,
            'paths': {'input_root': str(input_root.resolve()),
                      'profile': str(profile_path), 'receipt': str(receipt_path),
                      'selection': str(selection_path_value)},
            'attempts': plans}


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + '\n', encoding='utf-8')


def parse_time_report(path: Path) -> dict[str, float | int | None]:
    if not path.is_file():
        return {'wall_seconds': None, 'user_seconds': None,
                'sys_seconds': None, 'peak_rss_kb': None}
    text = path.read_text(errors='replace')
    elapsed = re.search(r'Elapsed \(wall clock\) time .*?:\s*([0-9:.]+)', text)
    user = re.search(r'User time \(seconds\):\s*([0-9.]+)', text)
    system = re.search(r'System time \(seconds\):\s*([0-9.]+)', text)
    rss = re.search(r'Maximum resident set size \(kbytes\):\s*(\d+)', text)
    wall = None
    if elapsed:
        values = [float(item) for item in elapsed.group(1).split(':')]
        wall = sum(value * (60 ** index) for index, value in
                   enumerate(reversed(values)))
    return {'wall_seconds': wall,
            'user_seconds': float(user.group(1)) if user else None,
            'sys_seconds': float(system.group(1)) if system else None,
            'peak_rss_kb': int(rss.group(1)) if rss else None}


def output_tree_hash(path: Path) -> str | None:
    files = sorted(item for item in path.rglob('*') if item.is_file())
    if not files:
        return None
    digest = hashlib.sha256()
    for item in files:
        digest.update(item.relative_to(path).as_posix().encode('utf-8'))
        digest.update(b'\0')
        digest.update(hashlib.sha256(item.read_bytes()).digest())
    return digest.hexdigest()


def artifact_hashes(path: Path) -> dict[str, str]:
    result = {}
    for item in sorted(path.rglob('*')):
        if not item.is_file() or item.name in {'host_time.txt', 'stdout.log', 'stderr.log'}:
            continue
        if item.name in {'traj_raw.tum', 'traj_corrected.tum', 'traj_lidar.txt',
                         'odometry.csv'} or item.suffix.lower() in {'.pcd', '.ply'}:
            result[item.relative_to(path).as_posix()] = sha256_file(item)
    return result


def expected_outputs(system: str, output: Path) -> list[Path]:
    if system == 'ours':
        return [output / 'traj_raw.tum']
    if system == 'glim_cpu':
        return [output / 'dump' / 'traj_lidar.txt']
    return [output / 'odometry.csv']


def run_attempt(plan: dict[str, Any], output_root: Path,
                timeout_seconds: float) -> dict[str, Any]:
    index = plan['schedule_index']
    final = output_root / f'attempt_{index:03d}'
    part = output_root / f'attempt_{index:03d}.part'
    if final.exists() or part.exists():
        raise ContractError(f'attempt already exists: {index}')
    part.mkdir(parents=True)
    command = list(plan['argv'])
    command = [token.replace('/M6A_OUTPUT_PLACEHOLDER', str(part))
               for token in command]
    plan['argv'] = command
    started = dt.datetime.now(dt.timezone.utc)
    exit_status = None
    timed_out = False
    signal = None
    try:
        with (part / 'stdout.log').open('w') as stdout, \
                (part / 'stderr.log').open('w') as stderr:
            try:
                completed = subprocess.run(
                    command, cwd=ROOT, stdout=stdout, stderr=stderr,
                    check=False, timeout=timeout_seconds)
                exit_status = completed.returncode
            except subprocess.TimeoutExpired:
                timed_out = True
                exit_status = 124
    except OSError as exc:
        (part / 'driver_error.txt').write_text(str(exc) + '\n', encoding='utf-8')
        exit_status = 127
    finished = dt.datetime.now(dt.timezone.utc)
    expected = expected_outputs(plan['system'], part)
    complete = not timed_out and exit_status == 0 and all(path.is_file() for path in expected)
    proof = dict(plan['gt_blind_guard'])
    proof.update({'ground_truth_content_opened': False, 'scorer_invoked': False,
                  'argv_gt_free': True, 'env_gt_free': True, 'mounts_gt_free': True})
    write_json(part / 'gt_blind_proof.json', proof)
    report = {
        'schema_version': 1, 'kind': 'm6a_gt_blind_attempt',
        'schedule': {key: plan[key] for key in
                     ('schedule_index', 'system', 'slot', 'sequence', 'repetition')},
        'identity': {'profile_canonical_sha256': plan.get('profile_canonical_sha256'),
                     'execution_receipt_file_sha256': plan.get('execution_receipt_file_sha256'),
                     'selection_receipt_file_sha256': plan.get('selection_receipt_file_sha256'),
                     'image_digest': plan['image_digest'],
                     'execution': plan['execution_identity'], 'input': plan['input']},
        'argv': command, 'env': plan['env'], 'mounts': plan['mounts'],
        'started_at': started.isoformat(), 'finished_at': finished.isoformat(),
        'execution': {'exit_status': exit_status, 'timed_out': timed_out, 'signal': signal},
        'completion': {
            'complete': complete,
            'expected_outputs': [str(path.relative_to(part)) for path in expected]},
        'timing': parse_time_report(part / 'host_time.txt'),
        'artifact_hashes': artifact_hashes(part),
        'output_tree_sha256': output_tree_hash(part),
        'gt_blind_proof': proof,
    }
    write_json(part / 'attempt.json', report)
    os.replace(part, final)
    return report


def root_marker(output_root: Path, identity: dict[str, Any]) -> Path:
    return output_root / '.m6a_gt_blind_root.json'


def check_or_create_marker(output_root: Path, identity: dict[str, Any], *, create: bool) -> None:
    marker = root_marker(output_root, identity)
    if marker.exists():
        observed = load_yaml(marker) if marker.suffix in ('.yaml', '.yml') else json.loads(
            marker.read_text(encoding='utf-8'))
        if observed != identity:
            raise ContractError('results root identity marker mismatch')
    elif any(output_root.iterdir()):
        raise ContractError('results root contains unowned files')
    elif create:
        write_json(marker, identity)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-root', type=Path, required=True)
    parser.add_argument('--output-root', type=Path, required=True)
    parser.add_argument('--profile', type=Path, default=DEFAULT_PROFILE)
    parser.add_argument('--receipt', type=Path, default=DEFAULT_RECEIPT)
    parser.add_argument('--selection', type=Path, default=DEFAULT_SELECTION)
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--preflight', action='store_true')
    parser.add_argument('--execute', action='store_true',
                        help='run replays; never combine with --dry-run')
    parser.add_argument('--plan-output', type=Path)
    parser.add_argument('--timeout-seconds', type=float, default=7200.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.dry_run and (args.preflight or args.execute):
        raise ContractError('--dry-run cannot be combined with --preflight/--execute')
    if args.execute and args.dry_run:
        raise ContractError('--execute cannot be combined with --dry-run')
    args.input_root = args.input_root.resolve(strict=True)
    args.output_root = args.output_root.resolve(strict=False)
    args.profile = args.profile.resolve(strict=True)
    args.receipt = args.receipt.resolve(strict=True)
    args.selection = args.selection.resolve(strict=True)
    assert_roots_are_disjoint(args.input_root, args.output_root)
    profile_doc = load_yaml(args.profile)
    receipt = load_yaml(args.receipt)
    selection = load_yaml(args.selection)
    plan = build_plan(profile_doc, receipt, selection, args.input_root,
                      args.profile, args.receipt, args.selection,
                      inspect_images=args.preflight or args.execute)
    plan['paths']['output_root'] = str(args.output_root)
    if args.plan_output:
        args.plan_output.parent.mkdir(parents=True, exist_ok=True)
        write_json(args.plan_output, plan)
    else:
        print(json.dumps(plan, indent=2, sort_keys=True))
    if not args.execute:
        return 0
    args.output_root.mkdir(parents=True, exist_ok=False)
    check_or_create_marker(args.output_root, plan['identity'], create=True)
    reports = []
    for item in plan['attempts']:
        item.update({key: value for key, value in plan['identity'].items()
                     if key.endswith('sha256')})
        print(f"M6a attempt {item['schedule_index']}/27: "
              f"{item['system']} {item['slot']} r{item['repetition']}", flush=True)
        reports.append(run_attempt(item, args.output_root, args.timeout_seconds))
    completion = {
        'schema_version': 1, 'kind': 'm6a_gt_blind_completion',
        'status': 'PASS' if all(report['completion']['complete'] for report in reports)
        else 'INCOMPLETE', 'identity': plan['identity'], 'attempts': reports,
        'ground_truth_content_opened': False, 'scorer_invoked': False,
    }
    write_json(args.output_root / 'completion_manifest.json', completion)
    return 0 if completion['status'] == 'PASS' else 2


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (ContractError, OSError, KeyError, TypeError, json.JSONDecodeError,
            yaml.YAMLError, subprocess.CalledProcessError) as error:
        print(f'error: {error}', file=sys.stderr)
        sys.exit(1)
