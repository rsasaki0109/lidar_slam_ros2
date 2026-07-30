#!/usr/bin/env python3
"""Validate the clean-prefix lidarslam product CLI installation."""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import tempfile


REPO_ROOT = Path(__file__).resolve().parent.parent
SOURCE_RUNTIME_MANIFEST = (
    REPO_ROOT / 'lidarslam' / 'product-runtime-files.txt'
)
CLI_CONTRACT = REPO_ROOT / 'docs' / 'contracts' / 'cli-v1.json'
OPTION_PATTERN = re.compile(
    r'(?<![A-Za-z0-9-])(--[a-z][a-z0-9-]*|-h)(?![A-Za-z0-9-])'
)
VALUE_OPTION_PATTERN = re.compile(
    r'(?P<option>--[a-z][a-z0-9-]*)[ =]'
    r'(?P<value>\{[^}\n]+\}|<[^>\n]+>|[A-Z][A-Z0-9_]*)'
)


def _runtime_names(path: Path) -> tuple[str, ...]:
    names = tuple(
        line.strip()
        for line in path.read_text(encoding='utf-8').splitlines()
        if line.strip() and not line.lstrip().startswith('#')
    )
    if not names:
        raise RuntimeError(f'product runtime manifest is empty: {path}')
    return names


def _load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'cannot load installed product module: {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _run(
    command: list[str],
    cwd: Path,
    env_updates: dict[str, str] | None = None,
) -> subprocess.CompletedProcess[str]:
    env = os.environ.copy()
    env.pop('LIDARSLAM_CLI_NAME', None)
    if env_updates:
        env.update(env_updates)
    return subprocess.run(
        command,
        cwd=cwd,
        check=False,
        capture_output=True,
        text=True,
        env=env,
    )


def _require_success(
    result: subprocess.CompletedProcess[str],
    label: str,
) -> None:
    if result.returncode != 0:
        raise RuntimeError(
            f'{label} failed with {result.returncode}\n'
            f'stdout:\n{result.stdout}\nstderr:\n{result.stderr}'
        )


def _option_definition_lines(rendered_help: str) -> str:
    return '\n'.join(
        line
        for line in rendered_help.splitlines()
        if line.lstrip().startswith('-')
    )


def _validate_installed_help(
    command_path: Path,
    work_dir: Path,
) -> None:
    """Match clean-prefix full help against the source v1 contract."""
    contract = json.loads(CLI_CONTRACT.read_text(encoding='utf-8'))
    for command, command_contract in contract['commands'].items():
        result = _run(
            [str(command_path), command, '--help-all'],
            work_dir,
        )
        _require_success(result, f'installed {command} --help-all')
        definition_lines = _option_definition_lines(result.stdout)
        rendered_options = set(OPTION_PATTERN.findall(definition_lines))
        expected_options = {
            name
            for option in command_contract['options']
            for name in option['names']
        }
        if rendered_options != expected_options:
            raise RuntimeError(
                f'installed {command} option inventory differs from '
                f'contract: rendered={sorted(rendered_options)}, '
                f'expected={sorted(expected_options)}'
            )

        rendered_values = {
            match.group('option'): match.group('value')
            for match in VALUE_OPTION_PATTERN.finditer(definition_lines)
        }
        expected_values = {
            option: value['value_name']
            for option, value in contract['value_options'][command].items()
        }
        if rendered_values != expected_values:
            raise RuntimeError(
                f'installed {command} value contract differs: '
                f'rendered={rendered_values}, expected={expected_values}'
            )


def _write_bag_fixture(path: Path) -> None:
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Imu, PointCloud2, PointField

    def topic_metadata(topic_id: int, name: str, msg_type: str):
        kwargs = {
            'name': name,
            'type': msg_type,
            'serialization_format': 'cdr',
        }
        try:
            return rosbag2_py.TopicMetadata(id=topic_id, **kwargs)
        except TypeError:  # Humble TopicMetadata predates the numeric id
            return rosbag2_py.TopicMetadata(**kwargs)

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    writer.create_topic(
        topic_metadata(0, '/points', 'sensor_msgs/msg/PointCloud2')
    )
    writer.create_topic(topic_metadata(1, '/imu', 'sensor_msgs/msg/Imu'))

    points = PointCloud2()
    points.header.frame_id = 'lidar'
    points.height = 1
    points.width = 1
    points.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='time', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    points.is_bigendian = False
    points.point_step = 16
    points.row_step = 16
    points.data = list(struct.pack('<ffff', 1.0, 2.0, 3.0, 0.0))
    points.is_dense = True
    writer.write('/points', serialize_message(points), 1_000_000_000)
    writer.write('/imu', serialize_message(Imu()), 1_000_000_001)
    if hasattr(writer, 'close'):
        writer.close()


def _historical_manifest_fixture() -> dict[str, object]:
    return {
        'schema_version': 1,
        'schema_uri': (
            'https://rsasaki0109.github.io/lidar_slam_ros2/'
            'schemas/run-manifest-v1.schema.json'
        ),
        'run_id': '12345678-1234-4234-8234-123456789abc',
        'status': 'succeeded',
        'input': {
            'bag_path': '/data/bag',
            'metadata_path': '/data/bag/metadata.yaml',
            'metadata_size_bytes': 42,
            'metadata_sha256': 'a' * 64,
            'storage_identifier': 'sqlite3',
            'storage_files': [],
            'identity_algorithm': 'sha256',
        },
        'software': {
            'product_version': '0.6.0',
            'git_commit': 'b' * 40,
            'git_dirty': False,
            'package_versions': {'lidarslam': '0.6.0'},
            'ros_distro': 'jazzy',
        },
        'profile': {'id': 'fixture', 'label': 'Fixture'},
        'execution': {
            'argv': ['lidarslam-map', 'run', '/data/bag'],
            'command_shell': 'lidarslam-map run /data/bag',
            'started_at': '2026-07-29T00:00:00Z',
            'finished_at': '2026-07-29T00:01:00Z',
            'exit_code': 0,
        },
        'output': {
            'requested_dir': '/data/map',
            'working_dir': '/data/map.partial',
            'finalized': True,
            'diagnosis_status': 'success',
            'artifact_checksums': [],
        },
    }


def _release_image_fixture() -> dict[str, object]:
    return {
        'schema_version': 1,
        'status': 'PASS',
        'ros_distro': 'jazzy',
        'platform': 'linux/amd64',
        'tag': 'ghcr.io/example/lidar_slam_ros2:v0.6.0-jazzy',
        'digest': f"sha256:{'c' * 64}",
        'git_commit': 'd' * 40,
        'product_version': '0.6.0',
        'cli_version': 'lidarslam_ros2 0.6.0',
    }


def validate_install(
    prefix: Path,
    expected_source_revision: str | None = None,
) -> None:
    """Validate commands, resources, isolation, and delegated behavior."""
    prefix = prefix.expanduser().resolve()
    path_command = prefix / 'bin' / 'lidarslam-map'
    ros_shim = prefix / 'lib' / 'lidarslam' / 'lidarslam-cli'
    historical_node = prefix / 'lib' / 'lidarslam' / 'lidarslam'
    setup_file = prefix / 'setup.bash'
    product_root = prefix / 'share' / 'lidarslam' / 'product'
    product_scripts = product_root / 'scripts'
    bash_completion = product_root / 'completions' / 'lidarslam-map.bash'
    product_schemas = product_root / 'schemas'
    product_build_info = product_root / 'product-build-info.json'
    installed_runtime_manifest = product_root / 'product-runtime-files.txt'

    for path in (path_command, ros_shim, historical_node):
        if not path.is_file() or not os.access(path, os.X_OK):
            raise RuntimeError(f'installed executable is missing: {path}')
    source_runtime_names = _runtime_names(SOURCE_RUNTIME_MANIFEST)
    installed_runtime_names = _runtime_names(installed_runtime_manifest)
    if installed_runtime_names != source_runtime_names:
        raise RuntimeError('installed product runtime manifest differs from source')
    for name in source_runtime_names:
        path = product_scripts / name
        if not path.is_file() or not os.access(path, os.X_OK):
            raise RuntimeError(f'installed product resource is missing: {path}')
    if not (product_root / 'VERSION').is_file():
        raise RuntimeError(f'installed VERSION is missing under {product_root}')
    for schema_name in (
        'run-manifest-v1.schema.json',
        'run-manifest-v2.schema.json',
        'release-image-v1.schema.json',
        'rollback-plan-v1.schema.json',
        'first-map-validation-receipt-v1.schema.json',
    ):
        schema_path = product_schemas / schema_name
        if not schema_path.is_file():
            raise RuntimeError(f'installed product schema is missing: {schema_path}')
    if not bash_completion.is_file():
        raise RuntimeError(f'installed Bash completion is missing: {bash_completion}')
    completion_syntax = _run(
        ['bash', '-n', str(bash_completion)],
        prefix,
    )
    _require_success(completion_syntax, 'installed Bash completion syntax')
    if not product_build_info.is_file():
        raise RuntimeError(
            f'installed product build information is missing: {product_build_info}'
        )
    build_info = json.loads(product_build_info.read_text(encoding='utf-8'))
    revision = build_info.get('revision')
    dirty = build_info.get('dirty')
    if not (
        build_info.get('schema_version') == 1
        and isinstance(revision, str)
        and len(revision) == 40
        and all(character in '0123456789abcdef' for character in revision)
        and isinstance(dirty, bool)
        and build_info.get('source') in ('git', 'override')
    ):
        raise RuntimeError(
            f'installed product build information is incomplete: {build_info}'
        )
    if (
        expected_source_revision is not None
        and revision != expected_source_revision
    ):
        raise RuntimeError(
            f'installed source revision {revision} does not match expected '
            f'{expected_source_revision}'
        )
    runner = _load_module(
        product_scripts / 'run_autoware_map_from_bag.py',
        'installed_run_autoware_map_from_bag',
    )
    software_identity = runner._software_identity()
    if (
        software_identity.get('git_commit') != revision
        or software_identity.get('git_dirty') is not dirty
    ):
        raise RuntimeError(
            'installed runner did not consume product build information: '
            f'{software_identity}'
        )
    if (product_scripts / 'gaussian_splatting_train.py').exists():
        raise RuntimeError('research-only scripts leaked into the product install')
    if shutil.which('ros2') is None:
        raise RuntimeError('ros2 is required to validate the installed ROS shim')
    if not setup_file.is_file():
        raise RuntimeError(f'installed setup file is missing: {setup_file}')

    existing_ament_prefix = os.environ.get('AMENT_PREFIX_PATH', '')
    clean_ament_prefix = str(prefix)
    if existing_ament_prefix:
        clean_ament_prefix += os.pathsep + existing_ament_prefix
    ros_env = {'AMENT_PREFIX_PATH': clean_ament_prefix}

    with tempfile.TemporaryDirectory(prefix='lidarslam-installed-cli-') as tmp:
        work_dir = Path(tmp)
        bag_dir = work_dir / 'sample_bag'
        output_dir = work_dir / 'map_output'
        _write_bag_fixture(bag_dir)

        path_setup = _run(
            [
                'bash',
                '-c',
                (
                    'set -e; source "$1"; '
                    'command -v lidarslam-map; lidarslam-map --version'
                ),
                'bash',
                str(setup_file),
            ],
            work_dir,
        )
        _require_success(path_setup, 'installed PATH setup')
        if str(path_command) not in path_setup.stdout:
            raise RuntimeError('setup.bash did not expose prefix/bin/lidarslam-map')

        for command in (path_command, ros_shim):
            result = _run([str(command), '--version'], work_dir)
            _require_success(result, f'{command.name} --version')
            if not result.stdout.startswith('lidarslam_ros2 '):
                raise RuntimeError(f'unexpected version output: {result.stdout!r}')

        _validate_installed_help(path_command, work_dir)

        historical_run = work_dir / 'historical_run'
        historical_run.mkdir()
        historical_source = historical_run / 'run_manifest.json'
        historical_source.write_text(
            json.dumps(_historical_manifest_fixture()),
            encoding='utf-8',
        )
        migrated_path = work_dir / 'migrated_manifest.json'
        migration = _run(
            [
                str(path_command),
                'migrate-manifest',
                str(historical_run),
                '--output',
                str(migrated_path),
                '--verification',
                'required',
                '--json',
            ],
            work_dir,
        )
        _require_success(migration, 'installed migrate-manifest')
        migrated = json.loads(migrated_path.read_text(encoding='utf-8'))
        if migrated.get('lifecycle', {}).get('stage') != 'complete':
            raise RuntimeError('installed migration was not inspect-only')
        if json.loads(migration.stdout).get('resume_allowed') is not False:
            raise RuntimeError('installed migration did not refuse resume')

        release_record = work_dir / 'release-image-jazzy.json'
        release_record.write_text(
            json.dumps(_release_image_fixture()),
            encoding='utf-8',
        )
        rollback = _run(
            [
                str(path_command),
                'rollback-plan',
                str(release_record),
                '--json',
            ],
            work_dir,
        )
        _require_success(rollback, 'installed rollback-plan')
        rollback_payload = json.loads(rollback.stdout)
        if rollback_payload.get('moving_tag_mutated') is not False:
            raise RuntimeError('installed rollback plan could move a tag')
        immutable_ref = rollback_payload.get('immutable_ref', '')
        if '@sha256:' not in immutable_ref:
            raise RuntimeError('installed rollback plan is not digest-pinned')

        executables = _run(
            ['ros2', 'pkg', 'executables', 'lidarslam'],
            work_dir,
            ros_env,
        )
        _require_success(executables, 'ros2 package executable discovery')
        discovered = set(executables.stdout.splitlines())
        if 'lidarslam lidarslam' not in discovered:
            raise RuntimeError('historical ROS executable is not discoverable')
        if 'lidarslam lidarslam-cli' not in discovered:
            raise RuntimeError('product CLI ROS shim is not discoverable')

        ros_version = _run(
            ['ros2', 'run', 'lidarslam', 'lidarslam-cli', '--version'],
            work_dir,
            ros_env,
        )
        _require_success(ros_version, 'ros2 run lidarslam lidarslam-cli')
        if not ros_version.stdout.startswith('lidarslam_ros2 '):
            raise RuntimeError(
                f'unexpected ROS shim version output: {ros_version.stdout!r}'
            )

        doctor = _run(
            [str(path_command), 'doctor', str(bag_dir), '--json'],
            work_dir,
        )
        _require_success(doctor, 'installed doctor')
        payload = json.loads(doctor.stdout)
        if payload.get('recommended_profile_id') != 'rko_lio_graph_public_path':
            raise RuntimeError('installed doctor selected an unexpected profile')

        dry_run = _run(
            [
                str(path_command),
                'run',
                str(bag_dir),
                '--output-dir',
                str(output_dir),
                '--dry-run',
            ],
            work_dir,
        )
        _require_success(dry_run, 'installed run --dry-run')
        expected_script = (
            product_scripts / 'run_rko_lio_graph_autoware_dogfood.sh'
        )
        expected_param = prefix / 'share' / 'lidarslam' / 'param' / 'lidarslam.yaml'
        if str(expected_script) not in dry_run.stdout:
            raise RuntimeError('dry-run did not resolve the installed workflow script')
        if str(expected_param) not in dry_run.stdout:
            raise RuntimeError('dry-run did not resolve the installed parameter file')
        if output_dir.exists():
            raise RuntimeError('installed dry-run created its output directory')

        output_dir.mkdir()
        inspect = _run(
            [str(path_command), 'inspect', str(output_dir), '--json'],
            work_dir,
        )
        _require_success(inspect, 'installed inspect')
        if json.loads(inspect.stdout).get('status') != 'incomplete':
            raise RuntimeError('installed inspect returned an unexpected status')

        view_help = _run(
            [str(path_command), 'view', '--help'],
            work_dir,
        )
        _require_success(view_help, 'installed view --help')
        if '--viewer {autoware,foxglove}' not in view_help.stdout:
            raise RuntimeError('installed view help is missing viewer choices')
        if '--runtime-dir' in view_help.stdout:
            raise RuntimeError('installed default view help leaked advanced options')
        view_help_all = _run(
            [str(path_command), 'view', '--help-all'],
            work_dir,
        )
        _require_success(view_help_all, 'installed view --help-all')
        if '--runtime-dir' not in view_help_all.stdout:
            raise RuntimeError('installed full view help is missing runtime options')

        view_incomplete = _run(
            [str(path_command), 'view', str(output_dir)],
            work_dir,
        )
        if view_incomplete.returncode != 2:
            raise RuntimeError(
                'installed view did not reject an incomplete map output'
            )
        if 'map output is incomplete' not in view_incomplete.stderr:
            raise RuntimeError(
                'installed view returned an unexpected incomplete-output error'
            )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--prefix',
        type=Path,
        required=True,
        help='Clean CMake install prefix to validate.',
    )
    parser.add_argument(
        '--expected-source-revision',
        help='Require the installed product to record this exact Git commit.',
    )
    args = parser.parse_args()
    try:
        validate_install(args.prefix, args.expected_source_revision)
    except (OSError, RuntimeError, ValueError, json.JSONDecodeError) as exc:
        print(f'error: {exc}', file=__import__('sys').stderr)
        return 1
    print(f'installed product CLI validated: {args.prefix.resolve()}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
