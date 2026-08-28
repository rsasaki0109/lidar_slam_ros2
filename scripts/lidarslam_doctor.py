#!/usr/bin/env python3
"""Read-only environment checks for the first lidarslam_ros2 map."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Callable


@dataclass(frozen=True)
class Check:
    name: str
    status: str
    detail: str
    fix: str = ''


def _command_check(command: str, *, required: bool = True) -> Check:
    location = shutil.which(command)
    if location:
        return Check(command, 'PASS', location)
    status = 'FAIL' if required else 'WARN'
    return Check(
        command,
        status,
        'command not found on PATH',
        f'install {command}, then open a new shell and rerun this doctor',
    )


def _workspace_setup_candidates(repo_root: Path) -> list[Path]:
    candidates = []
    if repo_root.parent.name == 'src':
        candidates.append(repo_root.parent.parent / 'install' / 'setup.bash')
    candidates.extend((
        repo_root / 'install' / 'setup.bash',
        repo_root.parent / 'install' / 'setup.bash',
        repo_root.parent.parent / 'install' / 'setup.bash',
    ))
    unique = []
    for candidate in candidates:
        if candidate not in unique:
            unique.append(candidate)
    return unique


def _source_checks(repo_root: Path) -> list[Check]:
    checks = [_command_check('ros2'), _command_check('colcon')]
    ros_distro = os.environ.get('ROS_DISTRO', '')
    if ros_distro in {'humble', 'jazzy'}:
        checks.append(Check('ROS_DISTRO', 'PASS', ros_distro))
    elif ros_distro:
        checks.append(Check(
            'ROS_DISTRO', 'WARN', ros_distro,
            'use a supported Humble or Jazzy environment',
        ))
    else:
        checks.append(Check(
            'ROS_DISTRO', 'FAIL', 'not set',
            'source /opt/ros/humble/setup.bash (or /opt/ros/jazzy/setup.bash)',
        ))

    install_candidates = _workspace_setup_candidates(repo_root)
    install_setup = next((path for path in install_candidates if path.is_file()), None)
    if install_setup is not None:
        checks.append(Check('workspace build', 'PASS', str(install_setup)))
    else:
        checked = ', '.join(str(path) for path in install_candidates)
        checks.append(Check(
            'workspace build', 'FAIL', f'no install/setup.bash found (checked: {checked})',
            'build from the workspace root with: colcon build --symlink-install '
            '--cmake-args -DCMAKE_BUILD_TYPE=Release',
        ))

    gitmodules = repo_root / '.gitmodules'
    missing_submodules: list[str] = []
    if gitmodules.is_file():
        for line in gitmodules.read_text(encoding='utf-8').splitlines():
            if line.strip().startswith('path ='):
                path = line.split('=', 1)[1].strip()
                candidate = repo_root / path
                if not candidate.exists() or not any(candidate.iterdir()):
                    missing_submodules.append(path)
    if missing_submodules:
        checks.append(Check(
            'git submodules', 'FAIL', ', '.join(missing_submodules),
            'git submodule update --init --recursive',
        ))
    else:
        checks.append(Check('git submodules', 'PASS', 'initialized'))
    return checks


def _docker_checks(_: Path) -> list[Check]:
    return [_command_check('docker')]


def run_checks(repo_root: Path, profile: str) -> list[Check]:
    checks = [Check(
        'repository',
        'PASS' if (repo_root / 'scripts' / 'run_docker_demo.sh').is_file() else 'FAIL',
        str(repo_root),
        'run this command from a lidarslam_ros2 checkout',
    )]
    profile_checks: dict[str, Callable[[Path], list[Check]]] = {
        'source': _source_checks,
        'docker': _docker_checks,
    }
    checks.extend(profile_checks[profile](repo_root))

    free_gib = shutil.disk_usage(repo_root).free / (1024 ** 3)
    if free_gib >= 5.0:
        checks.append(Check('free disk', 'PASS', f'{free_gib:.1f} GiB'))
    else:
        checks.append(Check(
            'free disk', 'FAIL', f'{free_gib:.1f} GiB',
            'free at least 5 GiB for the demo dataset, build, and map outputs',
        ))
    return checks


def _render_human(checks: list[Check], profile: str) -> None:
    for check in checks:
        print(f'[{check.status}] {check.name}: {check.detail}')
        if check.fix and check.status != 'PASS':
            print(f'       fix: {check.fix}')
    failed = sum(check.status == 'FAIL' for check in checks)
    if failed:
        print(f'doctor: FAIL ({failed} blocking check(s))')
    else:
        command = 'bash scripts/run_first_map.sh --path ' + profile
        print('doctor: PASS')
        print(f'next: {command}')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', choices=('source', 'docker'), default='source')
    parser.add_argument('--repo-root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--json', action='store_true', dest='as_json')
    args = parser.parse_args()

    repo_root = args.repo_root.resolve()
    checks = run_checks(repo_root, args.profile)
    failed = any(check.status == 'FAIL' for check in checks)
    if args.as_json:
        print(json.dumps({
            'profile': args.profile,
            'status': 'FAIL' if failed else 'PASS',
            'checks': [asdict(check) for check in checks],
        }, indent=2))
    else:
        _render_human(checks, args.profile)
    return 2 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
