#!/usr/bin/env python3
"""Run the shortest supported Autoware-compatible map-authoring path for a bag."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
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


REPO_ROOT = Path(__file__).resolve().parents[1]
MANIFEST_NAME = 'run_manifest.json'
MANIFEST_SCHEMA_VERSION = 1
MANIFEST_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/run-manifest-v1.schema.json'
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
    REPO_ROOT / 'lidarslam' / 'package.xml',
    REPO_ROOT / 'graph_based_slam' / 'package.xml',
    REPO_ROOT / 'lidarslam_msgs' / 'package.xml',
    REPO_ROOT / 'scanmatcher' / 'package.xml',
    REPO_ROOT / 'Thirdparty' / 'rko_lio' / 'package.xml',
)


def _load_script_module(script_name: str, module_name: str):
    script_path = REPO_ROOT / 'scripts' / script_name
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
            str(REPO_ROOT / 'scripts' / 'run_rko_lio_graph_autoware_dogfood.sh'),
            '--bag', str(bag_path),
            '--lidar-topic', pointcloud,
            '--imu-topic', imu,
            '--output-dir', str(output_dir),
            '--wait-for-offline-completion',
            '--skip-viewer',
        ]
    elif selected_profile == 'rko_lio_graph_mid360_preset':
        pointcloud = summary['topics']['pointcloud2'][0]['name']
        imu = summary['topics']['imu'][0]['name']
        command = [
            'bash',
            str(REPO_ROOT / 'scripts' / 'run_rko_lio_graph_autoware_dogfood.sh'),
            '--bag', str(bag_path),
            '--lidar-topic', pointcloud,
            '--imu-topic', imu,
            '--lidarslam-param',
            str(REPO_ROOT / 'lidarslam' / 'param' / 'lidarslam_mid360_rko_graph.yaml'),
            '--rko-param', str(REPO_ROOT / 'lidarslam' / 'param' / 'rko_lio_mid360.yaml'),
            '--output-dir', str(output_dir),
            '--wait-for-offline-completion',
            '--skip-viewer',
        ]
    elif selected_profile == 'pointcloud_gnss_smoke':
        pointcloud = summary['topics']['pointcloud2'][0]['name']
        gnss = summary['topics']['navsatfix'][0]['name']
        command = [
            'bash',
            str(REPO_ROOT / 'scripts' / 'run_open_data_gnss_smoke.sh'),
            '--bag', str(bag_path),
            '--points-topic', pointcloud,
            '--gnss-topic', gnss,
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
            str(REPO_ROOT / 'scripts' / 'run_open_data_applanix_velodyne_gnss_smoke.sh'),
            '--bag', str(bag_path),
            '--packet-topic', packet,
            '--gsof49-topic', gsof49,
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
) -> dict[str, object]:
    git_state = _git_state()
    return {
        'schema_version': MANIFEST_SCHEMA_VERSION,
        'schema_uri': MANIFEST_SCHEMA_URI,
        'run_id': str(uuid.uuid4()),
        'status': 'planned',
        'input': _bag_identity(bag_path),
        'software': {
            'product_version': _product_version(),
            'git_commit': git_state['commit'],
            'git_dirty': git_state['dirty'],
            'package_versions': _package_versions(),
            'ros_distro': os.environ.get('ROS_DISTRO'),
        },
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
            str(REPO_ROOT / 'scripts' / 'run_graph_slam_pointcloud_map_in_autoware_foxglove.sh'),
            str(output_dir),
        ]
    else:
        command = [
            'bash',
            str(REPO_ROOT / 'scripts' / 'run_graph_slam_pointcloud_map_in_autoware.sh'),
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

    subprocess.run(command, check=True, cwd=REPO_ROOT)


def maybe_verify_map(output_dir: Path, enabled: bool) -> None:
    if not enabled:
        return

    pointcloud_map_dir = output_dir / 'pointcloud_map'
    if not pointcloud_map_dir.is_dir():
        return

    verify_log_path = output_dir / 'verify_autoware_map.log'
    verify_command = [
        'python3',
        str(REPO_ROOT / 'scripts' / 'verify_autoware_map.py'),
        str(pointcloud_map_dir),
    ]
    with verify_log_path.open('w', encoding='utf-8') as stream:
        result = subprocess.run(
            verify_command,
            check=False,
            cwd=REPO_ROOT,
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
        '',
        'Examples:',
        '  python3 scripts/run_autoware_map_from_bag.py /path/to/rosbag2 --dry-run',
        (
            '  python3 scripts/run_autoware_map_from_bag.py /path/to/rosbag2 '
            '--output-dir output/my_map'
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
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    bag_path = Path(args.bag).expanduser().resolve()
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    output_dir = Path(args.output_dir).expanduser().resolve() if args.output_dir else (
        REPO_ROOT / 'output' / f'autoware_map_authoring_{bag_path.stem}_{timestamp}'
    )
    working_dir = output_dir.with_name(f'{output_dir.name}.partial')
    try:
        validate_bag_path(bag_path)
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

    try:
        manifest = _build_manifest(bag_path, output_dir, working_dir, plan)
        working_dir.mkdir(parents=True, exist_ok=False)
        manifest['status'] = 'running'
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
        result = subprocess.run(plan['command'], check=False, cwd=REPO_ROOT)
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

    try:
        _write_manifest(working_dir, manifest)
        maybe_verify_map(working_dir, enabled=not args.no_verify_map)
        _finalize_output(working_dir, output_dir)
        manifest['output']['finalized'] = True
        diagnosis = write_diagnostics(output_dir, bag_path)
        manifest['output']['diagnosis_status'] = diagnosis['status']
        if (
            manifest['status'] == 'succeeded'
            and not args.no_verify_map
            and diagnosis['status'] != 'success'
        ):
            manifest['status'] = 'failed'
            if exit_code == 0:
                exit_code = 1
                manifest['execution']['exit_code'] = exit_code
        manifest['output']['artifact_checksums'] = _artifact_checksums(output_dir)
        _write_manifest(output_dir, manifest)
    except (OSError, RuntimeError, ValueError) as exc:
        print(f'error: failed to finalize output: {exc}', file=sys.stderr)
        manifest_dir = output_dir if output_dir.exists() else working_dir
        if manifest_dir.exists():
            manifest['status'] = 'failed'
            manifest['execution']['exit_code'] = 70
            manifest['output']['finalized'] = output_dir.exists()
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

    if exit_code != 0:
        print(
            f'error: map workflow failed with exit code {exit_code}.',
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
