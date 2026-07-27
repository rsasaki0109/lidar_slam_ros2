#!/usr/bin/env python3
"""Run the shortest supported Autoware-compatible map-authoring path for a bag."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import uuid
import xml.etree.ElementTree as ET

import yaml


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
SOURCE_LAYOUT = (REPO_ROOT / 'lidarslam' / 'package.xml').is_file()
PACKAGE_SHARE = REPO_ROOT / 'lidarslam' if SOURCE_LAYOUT else REPO_ROOT.parent
SHARE_ROOT = PACKAGE_SHARE.parent
WORK_ROOT = REPO_ROOT if SOURCE_LAYOUT else Path.cwd()
MANIFEST_NAME = 'run_manifest.json'
MANIFEST_SCHEMA_VERSION = 2
MANIFEST_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/run-manifest-v2.schema.json'
)
PROFILE_CHOICES = (
    'rko_lio_graph_public_path',
    'rko_lio_graph_mid360_preset',
    'pointcloud_gnss_smoke',
    'packet_applanix_smoke',
)
PROFILE_HELP = (
    (
        'rko_lio_graph_public_path',
        'PointCloud2 + Imu through RKO-LIO and graph_based_slam.',
    ),
    (
        'rko_lio_graph_mid360_preset',
        'Livox/MID360 PointCloud2 + Imu with tracked tuned params.',
    ),
    ('pointcloud_gnss_smoke', 'PointCloud2 + NavSatFix smoke workflow.'),
    ('packet_applanix_smoke', 'VelodyneScan + Applanix GSOF49 smoke workflow.'),
)
PACKAGE_XML_PATHS = (
    SHARE_ROOT / 'lidarslam' / 'package.xml',
    SHARE_ROOT / 'graph_based_slam' / 'package.xml',
    SHARE_ROOT / 'lidarslam_msgs' / 'package.xml',
    SHARE_ROOT / 'scanmatcher' / 'package.xml',
    (
        REPO_ROOT / 'Thirdparty' / 'rko_lio' / 'package.xml'
        if SOURCE_LAYOUT
        else SHARE_ROOT / 'rko_lio' / 'package.xml'
    ),
)


def _load_script_module(script_name: str, module_name: str):
    script_path = SCRIPT_DIR / script_name
    spec = importlib.util.spec_from_file_location(module_name, script_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'failed to load module {module_name} from {script_path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _select_profile(payload: dict[str, object], forced_profile_id: str | None) -> str:
    if forced_profile_id:
        return forced_profile_id

    recommendations = payload['recommendations']
    recommendation_ids = {item['id'] for item in recommendations}
    summary = payload['summary']
    pointcloud_topics = summary['topics']['pointcloud2']
    imu_topics = summary['topics']['imu']
    bag_path_lower = summary['bag_path'].lower()
    looks_like_livox = (
        'mid360' in bag_path_lower
        or any('livox' in item['name'].lower() for item in pointcloud_topics + imu_topics)
    )
    if looks_like_livox and 'rko_lio_graph_mid360_preset' in recommendation_ids:
        return 'rko_lio_graph_mid360_preset'

    recommended_profile_id = payload['recommended_profile_id']
    if not recommended_profile_id:
        raise RuntimeError('no compatible public path was found for this bag')
    return recommended_profile_id


def build_execution_plan(
    bag_path: Path,
    profile_id: str | None,
    output_dir: Path,
    verify_map: bool,
) -> dict[str, object]:
    preflight = _load_script_module('preflight_autoware_map_bag.py', 'preflight_autoware_map_bag')
    payload = preflight.build_preflight_payload(bag_path)
    selected_profile = _select_profile(payload, profile_id)

    recommendations = {item['id']: item for item in payload['recommendations']}
    if selected_profile not in recommendations:
        available_profiles = ', '.join(recommendations) if recommendations else 'none'
        raise RuntimeError(
            f'profile is not compatible with this bag: {selected_profile}. '
            f'Available profiles: {available_profiles}. '
            'Run preflight to inspect the detected topics and missing requirements.'
        )

    summary = payload['summary']
    output_dir = output_dir.expanduser().resolve()

    if selected_profile == 'rko_lio_graph_public_path':
        pointcloud = summary['topics']['pointcloud2'][0]['name']
        imu = summary['topics']['imu'][0]['name']
        command = [
            'bash',
            str(SCRIPT_DIR / 'run_rko_lio_graph_autoware_dogfood.sh'),
            '--bag', str(bag_path),
            '--lidar-topic', pointcloud,
            '--imu-topic', imu,
            '--lidarslam-param', str(PACKAGE_SHARE / 'param' / 'lidarslam.yaml'),
            '--rko-param', str(PACKAGE_SHARE / 'param' / 'rko_lio_ntu_viral.yaml'),
            '--output-dir', str(output_dir),
            '--wait-for-offline-completion',
            '--skip-viewer',
        ]
    elif selected_profile == 'rko_lio_graph_mid360_preset':
        pointcloud = summary['topics']['pointcloud2'][0]['name']
        imu = summary['topics']['imu'][0]['name']
        command = [
            'bash',
            str(SCRIPT_DIR / 'run_rko_lio_graph_autoware_dogfood.sh'),
            '--bag', str(bag_path),
            '--lidar-topic', pointcloud,
            '--imu-topic', imu,
            '--lidarslam-param',
            str(PACKAGE_SHARE / 'param' / 'lidarslam_mid360_rko_graph.yaml'),
            '--rko-param', str(PACKAGE_SHARE / 'param' / 'rko_lio_mid360.yaml'),
            '--output-dir', str(output_dir),
            '--wait-for-offline-completion',
            '--skip-viewer',
        ]
    elif selected_profile == 'pointcloud_gnss_smoke':
        pointcloud = summary['topics']['pointcloud2'][0]['name']
        gnss = summary['topics']['navsatfix'][0]['name']
        command = [
            'bash',
            str(SCRIPT_DIR / 'run_open_data_gnss_smoke.sh'),
            '--bag', str(bag_path),
            '--points-topic', pointcloud,
            '--gnss-topic', gnss,
            '--param', str(PACKAGE_SHARE / 'param' / 'lidarslam.yaml'),
            '--save-dir', str(output_dir),
        ]
        if summary['capabilities']['has_imu']:
            command.extend(['--imu-topic', summary['topics']['imu'][0]['name']])
        if verify_map:
            command.append('--verify-map')
    elif selected_profile == 'packet_applanix_smoke':
        packet = summary['topics']['velodyne_scan'][0]['name']
        gsof49 = summary['topics']['applanix_gsof49'][0]['name']
        command = [
            'bash',
            str(SCRIPT_DIR / 'run_open_data_applanix_velodyne_gnss_smoke.sh'),
            '--bag', str(bag_path),
            '--packet-topic', packet,
            '--gsof49-topic', gsof49,
            '--param', str(PACKAGE_SHARE / 'param' / 'lidarslam.yaml'),
            '--save-dir', str(output_dir),
        ]
        if summary['capabilities']['has_applanix_gsof50']:
            command.extend(['--gsof50-topic', summary['topics']['applanix_gsof50'][0]['name']])
        if verify_map:
            command.append('--verify-map')
    else:
        raise RuntimeError(f'profile is not executable yet: {selected_profile}')

    return {
        'payload': payload,
        'profile_id': selected_profile,
        'label': recommendations[selected_profile]['label'],
        'command': command,
        'output_dir': output_dir,
    }


def validate_bag_path(bag_path: Path) -> None:
    if bag_path.is_file():
        if bag_path.suffix == '.db3':
            raise FileNotFoundError(
                f'rosbag2 path points to a .db3 file: {bag_path}. '
                'Pass the rosbag2 directory that contains metadata.yaml, not the .db3 file.'
            )
        raise FileNotFoundError(
            f'rosbag2 path is a file, not a directory: {bag_path}. '
            'Pass the rosbag2 directory that contains metadata.yaml.'
        )
    if not bag_path.exists():
        raise FileNotFoundError(
            f'rosbag2 directory does not exist: {bag_path}. '
            'Pass the directory that contains metadata.yaml.'
        )
    if not bag_path.is_dir():
        raise FileNotFoundError(
            f'rosbag2 path is not a directory: {bag_path}. '
            'Pass the directory that contains metadata.yaml.'
        )
    if not (bag_path / 'metadata.yaml').is_file():
        raise FileNotFoundError(
            f'metadata.yaml not found under {bag_path}. '
            'Pass the rosbag2 directory that contains metadata.yaml.'
        )


def validate_output_dir(output_dir: Path) -> None:
    if output_dir.exists() and not output_dir.is_dir():
        raise ValueError(f'output directory path is a file: {output_dir}')
    if output_dir.exists():
        raise ValueError(
            f'output directory already exists: {output_dir}. '
            'Choose a new directory; existing outputs are never overwritten.'
        )

    for parent in output_dir.parents:
        if parent.exists():
            if not parent.is_dir():
                raise ValueError(f'output directory parent is not a directory: {parent}')
            return


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z')


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _file_identity(path: Path, display_path: str) -> dict[str, object]:
    before = path.stat()
    digest = _sha256(path)
    after = path.stat()
    if (
        before.st_size != after.st_size
        or before.st_mtime_ns != after.st_mtime_ns
        or before.st_ino != after.st_ino
    ):
        raise RuntimeError(f'file changed while hashing: {path}')
    return {
        'path': display_path,
        'size_bytes': after.st_size,
        'sha256': digest,
    }


def _git_state() -> dict[str, object]:
    if not SOURCE_LAYOUT:
        return {'commit': None, 'dirty': None}
    commit_result = subprocess.run(
        ['git', 'rev-parse', 'HEAD'],
        cwd=REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    status_result = subprocess.run(
        ['git', 'status', '--porcelain', '--untracked-files=no'],
        cwd=REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    return {
        'commit': (
            commit_result.stdout.strip()
            if commit_result.returncode == 0
            else None
        ),
        'dirty': (
            bool(status_result.stdout.strip())
            if status_result.returncode == 0
            else None
        ),
    }


def _product_version() -> str:
    return (REPO_ROOT / 'VERSION').read_text(encoding='utf-8').strip()


def _package_versions() -> dict[str, str]:
    versions = {}
    for package_xml in PACKAGE_XML_PATHS:
        if not package_xml.is_file():
            continue
        root = ET.parse(package_xml).getroot()
        name = root.findtext('name')
        version = root.findtext('version')
        if name and version:
            versions[name.strip()] = version.strip()
    return dict(sorted(versions.items()))


def _software_identity() -> dict[str, object]:
    git_state = _git_state()
    return {
        'product_version': _product_version(),
        'git_commit': git_state['commit'],
        'git_dirty': git_state['dirty'],
        'package_versions': _package_versions(),
        'ros_distro': os.environ.get('ROS_DISTRO'),
    }


def _bag_identity(bag_path: Path) -> dict[str, object]:
    metadata_path = bag_path / 'metadata.yaml'
    metadata_before = metadata_path.stat()
    metadata_bytes = metadata_path.read_bytes()
    metadata_after = metadata_path.stat()
    if (
        metadata_before.st_size != metadata_after.st_size
        or metadata_before.st_mtime_ns != metadata_after.st_mtime_ns
        or metadata_before.st_ino != metadata_after.st_ino
    ):
        raise RuntimeError(f'file changed while reading: {metadata_path}')
    metadata = yaml.safe_load(metadata_bytes) or {}
    bag_info = metadata.get('rosbag2_bagfile_information') or {}
    relative_paths = bag_info.get('relative_file_paths') or []
    if not isinstance(relative_paths, list) or not all(
        isinstance(path, str) for path in relative_paths
    ):
        raise ValueError(
            'rosbag2 metadata relative_file_paths must be a list of strings'
        )
    if not relative_paths:
        relative_paths = [
            path.relative_to(bag_path).as_posix()
            for pattern in ('*.db3', '*.mcap')
            for path in sorted(bag_path.glob(pattern))
        ]

    storage_files = []
    seen_paths = set()
    resolved_bag_path = bag_path.resolve()
    for relative_path in relative_paths:
        path = (bag_path / str(relative_path)).resolve()
        try:
            normalized_relative = path.relative_to(resolved_bag_path).as_posix()
        except ValueError as exc:
            raise ValueError(
                f'rosbag2 metadata references a file outside the bag: {relative_path}'
            ) from exc
        if normalized_relative in seen_paths:
            continue
        if not path.is_file():
            raise ValueError(
                f'rosbag2 storage file referenced by metadata is missing: {path}'
            )
        seen_paths.add(normalized_relative)
        storage_files.append(_file_identity(path, normalized_relative))

    return {
        'bag_path': str(bag_path),
        'metadata_path': str(metadata_path),
        'metadata_size_bytes': metadata_after.st_size,
        'metadata_sha256': hashlib.sha256(metadata_bytes).hexdigest(),
        'storage_identifier': bag_info.get('storage_identifier'),
        'storage_files': storage_files,
        'identity_algorithm': 'sha256',
    }


def _write_manifest(run_dir: Path, manifest: dict[str, object]) -> None:
    destination = run_dir / MANIFEST_NAME
    temporary = run_dir / f'.{MANIFEST_NAME}.tmp'
    temporary.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    os.replace(temporary, destination)


def _artifact_checksums(run_dir: Path) -> list[dict[str, object]]:
    artifacts = []
    for path in sorted(item for item in run_dir.rglob('*') if item.is_file()):
        relative = path.relative_to(run_dir)
        if relative.as_posix() == MANIFEST_NAME or relative.name.startswith(
            f'.{MANIFEST_NAME}.'
        ):
            continue
        artifacts.append(_file_identity(path, relative.as_posix()))
    return artifacts


def _build_manifest(
    bag_path: Path,
    final_output_dir: Path,
    working_output_dir: Path,
    plan: dict[str, object],
    verify_map: bool = True,
) -> dict[str, object]:
    return {
        'schema_version': MANIFEST_SCHEMA_VERSION,
        'schema_uri': MANIFEST_SCHEMA_URI,
        'run_id': str(uuid.uuid4()),
        'status': 'planned',
        'lifecycle': {
            'stage': 'initialized',
            'resume_count': 0,
            'verification_enabled': verify_map,
            'runner_exit_code': None,
            'last_error': None,
        },
        'input': _bag_identity(bag_path),
        'software': _software_identity(),
        'profile': {
            'id': plan['profile_id'],
            'label': plan['label'],
        },
        'execution': {
            'argv': plan['command'],
            'command_shell': shlex.join(plan['command']),
            'started_at': None,
            'finished_at': None,
            'exit_code': None,
        },
        'output': {
            'requested_dir': str(final_output_dir),
            'working_dir': str(working_output_dir),
            'finalized': False,
            'artifact_checksums': [],
        },
    }


def _load_manifest(run_dir: Path) -> dict[str, object]:
    manifest_path = run_dir / MANIFEST_NAME
    if not manifest_path.is_file():
        raise ValueError(f'run manifest not found: {manifest_path}')
    try:
        manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f'run manifest is not readable JSON: {manifest_path}: {exc}') from exc
    if not isinstance(manifest, dict):
        raise ValueError(f'run manifest root must be an object: {manifest_path}')
    return manifest


def _validate_resume_state(
    bag_path: Path,
    final_output_dir: Path,
    working_output_dir: Path,
    plan: dict[str, object],
    verify_map: bool,
) -> tuple[Path, dict[str, object]]:
    existing_dirs = [
        path for path in (working_output_dir, final_output_dir) if path.exists()
    ]
    if len(existing_dirs) != 1:
        if not existing_dirs:
            raise ValueError(
                'no resumable output found. Expected exactly one of '
                f'{working_output_dir} or {final_output_dir}'
            )
        raise ValueError(
            'ambiguous resume state: both the partial and final output exist: '
            f'{working_output_dir}, {final_output_dir}'
        )

    run_dir = existing_dirs[0]
    if not run_dir.is_dir():
        raise ValueError(f'resume output path is not a directory: {run_dir}')
    manifest = _load_manifest(run_dir)
    if manifest.get('schema_version') != MANIFEST_SCHEMA_VERSION:
        raise ValueError(
            'resume requires run manifest schema v2; older manifests remain '
            'inspectable but cannot be resumed safely'
        )
    if manifest.get('schema_uri') != MANIFEST_SCHEMA_URI:
        raise ValueError('resume manifest schema_uri does not match schema v2')

    lifecycle = manifest.get('lifecycle')
    execution = manifest.get('execution')
    output = manifest.get('output')
    if not all(isinstance(item, dict) for item in (lifecycle, execution, output)):
        raise ValueError('resume manifest is missing lifecycle, execution, or output state')

    stage = lifecycle.get('stage')
    valid_stages = {
        'initialized',
        'workflow_running',
        'workflow_finished',
        'verifying',
        'verified',
        'finalizing',
        'finalized',
        'diagnosing',
        'diagnosed',
        'checksumming',
        'complete',
    }
    if stage not in valid_stages:
        raise ValueError(f'resume manifest has an unknown lifecycle stage: {stage!r}')
    if stage in ('initialized', 'workflow_running'):
        raise ValueError(
            f'refusing resume from lifecycle stage {stage!r}: the workflow may '
            'still be running, so starting post-processing could corrupt evidence'
        )
    if stage == 'complete':
        raise ValueError(
            'run is already complete; use `./scripts/lidarslam inspect '
            f'{final_output_dir}` instead'
        )
    if execution.get('finished_at') is None or execution.get('exit_code') is None:
        raise ValueError('resume requires a durably recorded terminal workflow result')
    if lifecycle.get('verification_enabled') is not verify_map:
        raise ValueError(
            'resume verification option mismatch; use the same --no-verify-map '
            'setting as the original run'
        )

    expected_values = (
        ('input identity', manifest.get('input'), _bag_identity(bag_path)),
        ('software identity', manifest.get('software'), _software_identity()),
        (
            'profile',
            manifest.get('profile'),
            {'id': plan['profile_id'], 'label': plan['label']},
        ),
        ('execution argv', execution.get('argv'), plan['command']),
        ('requested output', output.get('requested_dir'), str(final_output_dir)),
        ('working output', output.get('working_dir'), str(working_output_dir)),
    )
    for label, actual, expected in expected_values:
        if actual != expected:
            raise ValueError(
                f'resume {label} mismatch; use the original bag, software, '
                'profile, options, and output path'
            )

    if run_dir == working_output_dir and output.get('finalized') is True:
        raise ValueError('partial output claims it was already finalized')

    resume_count = lifecycle.get('resume_count')
    if not isinstance(resume_count, int) or isinstance(resume_count, bool):
        raise ValueError('resume manifest lifecycle.resume_count must be an integer')
    lifecycle['resume_count'] = resume_count + 1
    lifecycle['last_error'] = None
    workflow_exit_code = execution['exit_code']
    if manifest.get('status') == 'interrupted' and workflow_exit_code == 130:
        manifest['status'] = 'interrupted'
    else:
        manifest['status'] = 'succeeded' if workflow_exit_code == 0 else 'failed'
    return run_dir, manifest


def _finalize_output(working_dir: Path, final_dir: Path) -> None:
    if final_dir.exists():
        raise RuntimeError(
            f'output collision detected before finalization: {final_dir}'
        )
    os.replace(working_dir, final_dir)


def maybe_open_viewer(args: argparse.Namespace, output_dir: Path) -> None:
    if args.viewer == 'none':
        return

    if args.viewer == 'foxglove':
        command = [
            'bash',
            str(SCRIPT_DIR / 'run_graph_slam_pointcloud_map_in_autoware_foxglove.sh'),
            str(output_dir),
        ]
    else:
        command = [
            'bash',
            str(SCRIPT_DIR / 'run_graph_slam_pointcloud_map_in_autoware.sh'),
            str(output_dir),
        ]
        if args.autoware_core_dir:
            command.extend(['--autoware-core-dir', args.autoware_core_dir])

    if args.work_dir:
        command.extend(['--work-dir', args.work_dir])
    if args.viewer_run_dir:
        command.extend(['--run-dir', args.viewer_run_dir])
    if args.viewer_rebuild:
        command.append('--rebuild')
    if args.auto_exit_secs is not None:
        command.extend(['--auto-exit-secs', str(args.auto_exit_secs)])

    subprocess.run(command, check=True, cwd=WORK_ROOT)


def maybe_verify_map(output_dir: Path, enabled: bool) -> None:
    if not enabled:
        return

    pointcloud_map_dir = output_dir / 'pointcloud_map'
    if not pointcloud_map_dir.is_dir():
        return

    verify_log_path = output_dir / 'verify_autoware_map.log'
    verify_command = [
        'python3',
        str(SCRIPT_DIR / 'verify_autoware_map.py'),
        str(pointcloud_map_dir),
    ]
    with verify_log_path.open('w', encoding='utf-8') as stream:
        result = subprocess.run(
            verify_command,
            check=False,
            cwd=WORK_ROOT,
            stdout=stream,
            stderr=subprocess.STDOUT,
        )
    if result.returncode != 0:
        print(f'Warning: verify_autoware_map.py failed. See {verify_log_path}')


def print_next_steps(args: argparse.Namespace, output_dir: Path) -> None:
    print('Next steps:')
    print(
        '  Diagnosis: '
        f'python3 scripts/diagnose_autoware_map_run.py {shlex.quote(str(output_dir))} --write'
    )
    verify_log = output_dir / 'verify_autoware_map.log'
    if verify_log.is_file():
        print(f'  Verify log: {verify_log}')
    print(f'  Saved map:  {output_dir / "pointcloud_map"}')
    lanelet2_map = output_dir / 'lanelet2_map.osm'
    if lanelet2_map.is_file():
        print(f'  Lanelet2:   {lanelet2_map}')

    if args.viewer == 'none':
        print(
            '  Open in Foxglove: '
            'bash scripts/run_graph_slam_pointcloud_map_in_autoware_foxglove.sh '
            f'{shlex.quote(str(output_dir))}'
        )
        print(
            '  Open in Autoware viewer: '
            'bash scripts/run_graph_slam_pointcloud_map_in_autoware.sh '
            f'{shlex.quote(str(output_dir))}'
        )


def write_diagnostics(output_dir: Path, bag_path: Path) -> dict[str, object]:
    diagnose = _load_script_module('diagnose_autoware_map_run.py', 'diagnose_autoware_map_run')
    summary = diagnose.summarize_run(output_dir, bag_path)
    markdown = diagnose.render_markdown(summary)
    (output_dir / 'autoware_map_diagnosis.md').write_text(markdown, encoding='utf-8')
    (output_dir / 'autoware_map_diagnosis.json').write_text(
        json.dumps(summary, indent=2, sort_keys=True),
        encoding='utf-8',
    )
    return summary


@contextmanager
def _postprocess_lock(output_dir: Path):
    lock_path = output_dir.with_name(f'.{output_dir.name}.postprocess.lock')
    try:
        stream = lock_path.open('a+', encoding='utf-8')
    except OSError as exc:
        raise RuntimeError(f'cannot open post-processing lock {lock_path}: {exc}') from exc
    try:
        try:
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError(
                f'post-processing is already active for {output_dir}'
            ) from exc
        yield
    finally:
        stream.close()


def _postprocess_run(
    args: argparse.Namespace,
    manifest: dict[str, object],
    bag_path: Path,
    working_dir: Path,
    output_dir: Path,
    run_dir: Path,
) -> int:
    lifecycle = manifest['lifecycle']
    execution = manifest['execution']
    workflow_exit_code = execution['exit_code']
    current_dir = run_dir
    try:
        _write_manifest(current_dir, manifest)
        lifecycle['stage'] = 'verifying'
        _write_manifest(current_dir, manifest)
        maybe_verify_map(current_dir, enabled=not args.no_verify_map)
        lifecycle['stage'] = 'verified'
        _write_manifest(current_dir, manifest)

        if current_dir == working_dir:
            lifecycle['stage'] = 'finalizing'
            _write_manifest(current_dir, manifest)
            _finalize_output(working_dir, output_dir)
            current_dir = output_dir
        elif current_dir != output_dir:
            raise RuntimeError(f'unexpected resume directory: {current_dir}')

        manifest['output']['finalized'] = True
        lifecycle['stage'] = 'finalized'
        _write_manifest(current_dir, manifest)

        lifecycle['stage'] = 'diagnosing'
        _write_manifest(current_dir, manifest)
        diagnosis = write_diagnostics(current_dir, bag_path)
        manifest['output']['diagnosis_status'] = diagnosis['status']
        lifecycle['stage'] = 'diagnosed'
        _write_manifest(current_dir, manifest)

        interrupted = manifest['status'] == 'interrupted' and workflow_exit_code == 130
        if interrupted:
            runner_exit_code = 130
        elif workflow_exit_code != 0:
            runner_exit_code = workflow_exit_code
            manifest['status'] = 'failed'
        elif not args.no_verify_map and diagnosis['status'] != 'success':
            runner_exit_code = 1
            manifest['status'] = 'failed'
        else:
            runner_exit_code = 0
            manifest['status'] = 'succeeded'

        lifecycle['stage'] = 'checksumming'
        _write_manifest(current_dir, manifest)
        manifest['output']['artifact_checksums'] = _artifact_checksums(current_dir)
        lifecycle['stage'] = 'complete'
        lifecycle['runner_exit_code'] = runner_exit_code
        lifecycle['last_error'] = None
        _write_manifest(current_dir, manifest)
        return runner_exit_code
    except (OSError, RuntimeError, ValueError, KeyError, TypeError) as exc:
        print(f'error: failed to finalize output: {exc}', file=sys.stderr)
        manifest_dir = output_dir if output_dir.exists() else working_dir
        if manifest_dir.exists():
            if manifest.get('status') not in ('failed', 'interrupted'):
                manifest['status'] = 'failed'
            manifest['output']['finalized'] = output_dir.exists()
            lifecycle['runner_exit_code'] = 70
            lifecycle['last_error'] = str(exc)
            try:
                manifest['output']['artifact_checksums'] = _artifact_checksums(
                    manifest_dir
                )
                _write_manifest(manifest_dir, manifest)
            except (OSError, RuntimeError) as manifest_exc:
                print(
                    f'warning: failed to preserve terminal manifest: {manifest_exc}',
                    file=sys.stderr,
                )
        return 70


def _postprocess_with_lock(
    args: argparse.Namespace,
    manifest: dict[str, object],
    bag_path: Path,
    working_dir: Path,
    output_dir: Path,
    run_dir: Path,
) -> int:
    try:
        with _postprocess_lock(output_dir):
            return _postprocess_run(
                args,
                manifest,
                bag_path,
                working_dir,
                output_dir,
                run_dir,
            )
    except RuntimeError as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 70


def _profile_help_text() -> str:
    lines = ['Workflow profiles:']
    for profile_id, description in PROFILE_HELP:
        lines.append(f'  {profile_id}: {description}')
    return '\n'.join(lines)


def _help_epilog() -> str:
    return '\n'.join([
        'The input must be the rosbag2 directory that contains metadata.yaml.',
        'Pass /path/to/rosbag2, not /path/to/rosbag2_0.db3.',
        '',
        _profile_help_text(),
        '',
        'Expected successful outputs:',
        '  pointcloud_map/',
        '  map_projector_info.yaml',
        '  verify_autoware_map.log',
        '  autoware_map_diagnosis.md',
        '  run_manifest.json',
        '',
        'Examples:',
        '  python3 scripts/run_autoware_map_from_bag.py /path/to/rosbag2 --dry-run',
        (
            '  python3 scripts/run_autoware_map_from_bag.py /path/to/rosbag2 '
            '--output-dir output/my_map'
        ),
        (
            '  python3 scripts/run_autoware_map_from_bag.py /path/to/rosbag2 '
            '--output-dir output/my_map --resume'
        ),
        '  python3 scripts/run_autoware_map_from_bag.py /path/to/rosbag2 --viewer foxglove',
    ])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            'Inspect a rosbag2 directory, choose a supported Autoware-compatible '
            'map workflow, and write map artifacts under output/ by default.'
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=_help_epilog(),
    )
    parser.add_argument(
        'bag',
        metavar='rosbag2_dir',
        help='Directory containing metadata.yaml.',
    )
    parser.add_argument(
        '--profile',
        choices=PROFILE_CHOICES,
        metavar='<id>',
        help='Force a compatible profile instead of the default recommendation.',
    )
    parser.add_argument(
        '--output-dir',
        metavar='<dir>',
        help=(
            'Directory for generated map outputs and logs. Defaults to '
            'output/autoware_map_authoring_<bag>_<timestamp>.'
        ),
    )
    parser.add_argument(
        '--viewer',
        choices=['none', 'autoware', 'foxglove'],
        default='none',
        help='Open the saved map after the run (default: none).',
    )
    parser.add_argument(
        '--autoware-core-dir',
        help='autoware_core checkout used by the Docker viewer.',
    )
    parser.add_argument(
        '--work-dir',
        help='Runtime workspace directory for Autoware/Foxglove viewers.',
    )
    parser.add_argument('--viewer-run-dir', help='Existing built viewer runtime to reuse.')
    parser.add_argument(
        '--viewer-rebuild',
        action='store_true',
        help='Rebuild the viewer runtime before opening.',
    )
    parser.add_argument(
        '--auto-exit-secs',
        type=int,
        help='Auto-close the viewer after N seconds.',
    )
    parser.add_argument(
        '--no-verify-map',
        action='store_true',
        help='Skip verify_autoware_map.py in smoke wrappers.',
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Print the selected command without executing it.',
    )
    parser.add_argument(
        '--resume',
        action='store_true',
        help=(
            'Resume verification, finalization, diagnosis, and checksums for a '
            'terminal schema-v2 run; the map workflow is never re-executed.'
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.resume and args.dry_run:
        print('error: --resume cannot be combined with --dry-run', file=sys.stderr)
        return 2
    if args.resume and not args.output_dir:
        print('error: --resume requires an explicit --output-dir', file=sys.stderr)
        return 2

    bag_path = Path(args.bag).expanduser().resolve()
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    output_dir = Path(args.output_dir).expanduser().resolve() if args.output_dir else (
        WORK_ROOT / 'output' / f'autoware_map_authoring_{bag_path.stem}_{timestamp}'
    )
    working_dir = output_dir.with_name(f'{output_dir.name}.partial')
    try:
        validate_bag_path(bag_path)
        if not args.resume:
            validate_output_dir(output_dir)
            validate_output_dir(working_dir)
        plan = build_execution_plan(
            bag_path=bag_path,
            profile_id=args.profile,
            output_dir=working_dir,
            verify_map=not args.no_verify_map,
        )
    except (FileNotFoundError, RuntimeError, ValueError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2

    print(f"Selected profile: {plan['label']}")
    print(f'Output directory: {output_dir}')
    print(f'Atomic working directory: {working_dir}')
    print('Command:')
    print('  ' + shlex.join(plan['command']))

    if args.dry_run:
        return 0
    if args.resume:
        try:
            with _postprocess_lock(output_dir):
                run_dir, manifest = _validate_resume_state(
                    bag_path,
                    output_dir,
                    working_dir,
                    plan,
                    not args.no_verify_map,
                )
                print(f'Resuming terminal post-processing from: {run_dir}')
                exit_code = _postprocess_run(
                    args,
                    manifest,
                    bag_path,
                    working_dir,
                    output_dir,
                    run_dir,
                )
        except ValueError as exc:
            print(f'error: {exc}', file=sys.stderr)
            return 2
        except RuntimeError as exc:
            print(f'error: {exc}', file=sys.stderr)
            return 70
        return _finish_run(args, plan, output_dir, exit_code)

    try:
        manifest = _build_manifest(
            bag_path,
            output_dir,
            working_dir,
            plan,
            verify_map=not args.no_verify_map,
        )
        working_dir.mkdir(parents=True, exist_ok=False)
        manifest['status'] = 'running'
        manifest['lifecycle']['stage'] = 'workflow_running'
        manifest['execution']['started_at'] = _utc_now()
        _write_manifest(working_dir, manifest)
    except (OSError, RuntimeError, ValueError, ET.ParseError, yaml.YAMLError) as exc:
        print(
            f'error: failed to initialize output directory {working_dir}: {exc}',
            file=sys.stderr,
        )
        return 2

    exit_code = 0
    interrupted = False
    try:
        result = subprocess.run(plan['command'], check=False, cwd=WORK_ROOT)
        exit_code = result.returncode
    except KeyboardInterrupt:
        interrupted = True
        exit_code = 130
    except OSError as exc:
        print(f'error: failed to start map workflow: {exc}', file=sys.stderr)
        exit_code = 70

    manifest['execution']['exit_code'] = exit_code
    manifest['execution']['finished_at'] = _utc_now()
    manifest['status'] = (
        'interrupted' if interrupted else ('succeeded' if exit_code == 0 else 'failed')
    )
    manifest['lifecycle']['stage'] = 'workflow_finished'
    exit_code = _postprocess_with_lock(
        args,
        manifest,
        bag_path,
        working_dir,
        output_dir,
        working_dir,
    )
    return _finish_run(args, plan, output_dir, exit_code)


def _finish_run(
    args: argparse.Namespace,
    plan: dict[str, object],
    output_dir: Path,
    exit_code: int,
) -> int:
    if exit_code != 0:
        print(
            f'error: map run failed with exit code {exit_code}.',
            file=sys.stderr,
        )
        print('failed command:', shlex.join(plan['command']), file=sys.stderr)
        if (output_dir / 'autoware_map_diagnosis.md').is_file():
            print(f'Diagnosis written to: {output_dir / "autoware_map_diagnosis.md"}')
        return exit_code

    print_next_steps(args, output_dir)
    try:
        maybe_open_viewer(args, output_dir)
    except subprocess.CalledProcessError as exc:
        print(f'error: viewer failed with exit code {exc.returncode}.', file=sys.stderr)
        return exc.returncode or 1
    print(f'Diagnosis written to: {output_dir / "autoware_map_diagnosis.md"}')
    print(f'Run manifest: {output_dir / MANIFEST_NAME}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
