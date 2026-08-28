"""Regression tests for first-use entrypoints."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
DOCTOR = REPO_ROOT / 'scripts' / 'lidarslam_doctor.py'
FIRST_MAP = REPO_ROOT / 'scripts' / 'run_first_map.sh'
MAP_RUNNER = REPO_ROOT / 'scripts' / 'run_autoware_map_from_bag.py'


def _load_doctor():
    spec = importlib.util.spec_from_file_location('lidarslam_doctor', DOCTOR)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _load_map_runner():
    spec = importlib.util.spec_from_file_location('run_autoware_map_from_bag', MAP_RUNNER)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_doctor_reports_missing_checkout(tmp_path: Path):
    result = subprocess.run(
        ['python3', str(DOCTOR), '--profile', 'docker', '--repo-root', str(tmp_path), '--json'],
        check=False,
        capture_output=True,
        text=True,
    )
    report = json.loads(result.stdout)
    assert result.returncode == 2
    assert report['status'] == 'FAIL'
    assert report['checks'][0]['name'] == 'repository'


def test_doctor_disk_check_is_structured(monkeypatch):
    doctor = _load_doctor()
    monkeypatch.setattr(doctor.shutil, 'which', lambda _: '/usr/bin/docker')
    monkeypatch.setattr(
        doctor.shutil,
        'disk_usage',
        lambda _: doctor.shutil._ntuple_diskusage(20, 10, 10),
    )
    checks = doctor.run_checks(REPO_ROOT, 'docker')
    assert [check.status for check in checks] == ['PASS', 'PASS', 'FAIL']
    assert checks[-1].name == 'free disk'
    assert '5 GiB' in checks[-1].fix


def test_doctor_finds_repo_local_build(monkeypatch, tmp_path: Path):
    doctor = _load_doctor()
    repo = tmp_path / 'lidar_slam_ros2'
    (repo / 'scripts').mkdir(parents=True)
    (repo / 'scripts' / 'run_docker_demo.sh').touch()
    setup = repo / 'install' / 'setup.bash'
    setup.parent.mkdir()
    setup.touch()
    monkeypatch.setenv('ROS_DISTRO', 'jazzy')
    monkeypatch.setattr(doctor.shutil, 'which', lambda command: f'/usr/bin/{command}')
    monkeypatch.setattr(
        doctor.shutil, 'disk_usage',
        lambda _: doctor.shutil._ntuple_diskusage(20, 10, 10),
    )

    checks = doctor.run_checks(repo, 'source')

    workspace = next(check for check in checks if check.name == 'workspace build')
    assert workspace.status == 'PASS'
    assert workspace.detail == str(setup)


def test_doctor_prefers_standard_src_workspace_build(monkeypatch, tmp_path: Path):
    doctor = _load_doctor()
    repo = tmp_path / 'ws' / 'src' / 'lidar_slam_ros2'
    (repo / 'scripts').mkdir(parents=True)
    (repo / 'scripts' / 'run_docker_demo.sh').touch()
    standard_setup = tmp_path / 'ws' / 'install' / 'setup.bash'
    standard_setup.parent.mkdir()
    standard_setup.touch()
    repo_setup = repo / 'install' / 'setup.bash'
    repo_setup.parent.mkdir()
    repo_setup.touch()
    monkeypatch.setenv('ROS_DISTRO', 'humble')
    monkeypatch.setattr(doctor.shutil, 'which', lambda command: f'/usr/bin/{command}')
    monkeypatch.setattr(
        doctor.shutil, 'disk_usage',
        lambda _: doctor.shutil._ntuple_diskusage(20, 10, 10),
    )

    checks = doctor.run_checks(repo, 'source')

    workspace = next(check for check in checks if check.name == 'workspace build')
    assert workspace.status == 'PASS'
    assert workspace.detail == str(standard_setup)


def test_first_map_dry_run_does_not_download_or_map():
    result = subprocess.run(
        ['bash', str(FIRST_MAP), '--path', 'source', '--dry-run'],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0
    assert 'lidarslam_doctor.py --profile source' in result.stdout
    assert 'download_ntu_viral_tnp01.sh' in result.stdout
    assert 'run_autoware_quickstart.sh' in result.stdout


def test_first_map_rejects_unknown_path():
    result = subprocess.run(
        ['bash', str(FIRST_MAP), '--path', 'unknown', '--dry-run'],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 2
    assert '--path must be auto, docker, or source' in result.stderr


def test_map_runner_refuses_nonempty_output_directory(tmp_path: Path):
    runner = _load_map_runner()
    output = tmp_path / 'existing-run'
    output.mkdir()
    (output / 'stale-map.pcd').write_bytes(b'stale')

    try:
        runner.validate_output_dir(output)
    except ValueError as exc:
        assert 'output directory is non-empty' in str(exc)
        assert 'operational resume is not supported yet' in str(exc)
    else:
        raise AssertionError('non-empty output directory was accepted')


def test_map_runner_accepts_missing_or_empty_output_directory(tmp_path: Path):
    runner = _load_map_runner()
    runner.validate_output_dir(tmp_path / 'new-run')
    empty = tmp_path / 'empty-run'
    empty.mkdir()
    runner.validate_output_dir(empty)
