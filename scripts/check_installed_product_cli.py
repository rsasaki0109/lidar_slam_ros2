#!/usr/bin/env python3
"""Validate the clean-prefix lidarslam product CLI installation."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile


REPO_ROOT = Path(__file__).resolve().parent.parent
SOURCE_RUNTIME_MANIFEST = (
    REPO_ROOT / 'lidarslam' / 'product-runtime-files.txt'
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


def validate_install(prefix: Path) -> None:
    """Validate commands, resources, isolation, and delegated behavior."""
    prefix = prefix.expanduser().resolve()
    path_command = prefix / 'bin' / 'lidarslam-map'
    ros_shim = prefix / 'lib' / 'lidarslam' / 'lidarslam-cli'
    historical_node = prefix / 'lib' / 'lidarslam' / 'lidarslam'
    setup_file = prefix / 'setup.bash'
    product_root = prefix / 'share' / 'lidarslam' / 'product'
    product_scripts = product_root / 'scripts'
    bash_completion = product_root / 'completions' / 'lidarslam-map.bash'
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
    if not bash_completion.is_file():
        raise RuntimeError(f'installed Bash completion is missing: {bash_completion}')
    completion_syntax = _run(
        ['bash', '-n', str(bash_completion)],
        prefix,
    )
    _require_success(completion_syntax, 'installed Bash completion syntax')
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
    args = parser.parse_args()
    try:
        validate_install(args.prefix)
    except (OSError, RuntimeError, ValueError, json.JSONDecodeError) as exc:
        print(f'error: {exc}', file=__import__('sys').stderr)
        return 1
    print(f'installed product CLI validated: {args.prefix.resolve()}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
