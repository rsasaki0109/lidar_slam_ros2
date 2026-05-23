# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Tests for Jetson MID-360 host readiness checks."""

from __future__ import annotations

import json
from pathlib import Path
import shutil
import subprocess
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_DIR = REPO_ROOT / 'scripts'
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'check_jetson_mid360_host_readiness.py'


def _load_module():
    if str(SCRIPTS_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPTS_DIR))
    import jetson_mid360_host_tools

    return jetson_mid360_host_tools


def _write_fake_host_root(tmp_path: Path, *, temp_millideg: int = 45000) -> Path:
    root = tmp_path / 'host'
    (root / 'proc' / 'device-tree').mkdir(parents=True)
    (root / 'proc' / 'device-tree' / 'model').write_text(
        'NVIDIA Jetson Orin NX\x00',
        encoding='utf-8',
    )
    (root / 'etc').mkdir()
    (root / 'etc' / 'nv_tegra_release').write_text(
        '# R36 (release), REVISION: 4.0',
        encoding='utf-8',
    )
    (root / 'etc' / 'os-release').write_text(
        'NAME="Ubuntu"\nVERSION_ID="22.04"\n',
        encoding='utf-8',
    )
    (root / 'proc' / 'meminfo').write_text(
        'MemTotal:       8388608 kB\nMemAvailable:   4194304 kB\n',
        encoding='utf-8',
    )

    thermal = root / 'sys' / 'class' / 'thermal' / 'thermal_zone0'
    thermal.mkdir(parents=True)
    (thermal / 'type').write_text('gpu-thermal\n', encoding='utf-8')
    (thermal / 'temp').write_text(f'{temp_millideg}\n', encoding='utf-8')

    governor = root / 'sys' / 'devices' / 'system' / 'cpu' / 'cpu0' / 'cpufreq'
    governor.mkdir(parents=True)
    (governor / 'scaling_governor').write_text('performance\n', encoding='utf-8')
    return root


def _options(module, tmp_path: Path):
    output_dir = tmp_path / 'out'
    bag_dir = tmp_path / 'bags'
    bag_dir.mkdir()
    return module.HostReadinessOptions(
        output_dir=output_dir,
        bag_dir=bag_dir,
        expected_bag_minutes=0.01,
        estimated_bag_mbps=1.0,
        bag_reserve_gb=0.0,
        min_bag_free_gb=0.1,
        min_output_free_gb=0.1,
        min_memory_available_gb=0.1,
    )


def _disk_usage(free_gib: float = 100.0):
    total = int(128 * 1024**3)
    free = int(free_gib * 1024**3)
    used = total - free
    return shutil._ntuple_diskusage(total, used, free)


def test_host_readiness_passes_with_fake_jetson(tmp_path: Path):
    module = _load_module()
    root = _write_fake_host_root(tmp_path)
    checker = module.JetsonHostReadiness(
        host_root=root,
        which=lambda name: f'/usr/bin/{name}',
        disk_usage=lambda path: _disk_usage(),
        machine=lambda: 'aarch64',
    )

    report = checker.build_report(_options(module, tmp_path))

    assert report['status'] == 'PASS'
    assert report['host']['model'] == 'NVIDIA Jetson Orin NX'
    assert report['storage']['estimated_recording_gb'] > 0.0
    assert any(check['id'] == 'thermal_max_temp' and check['status'] == 'ok'
               for check in report['checks'])


def test_host_readiness_fails_when_ros2_missing(tmp_path: Path):
    module = _load_module()
    root = _write_fake_host_root(tmp_path)
    checker = module.JetsonHostReadiness(
        host_root=root,
        which=lambda name: None if name == 'ros2' else f'/usr/bin/{name}',
        disk_usage=lambda path: _disk_usage(),
        machine=lambda: 'aarch64',
    )

    report = checker.build_report(_options(module, tmp_path))

    assert report['status'] == 'FAIL'
    assert any(check['id'] == 'tool_ros2' and check['status'] == 'fail'
               for check in report['checks'])


def test_host_readiness_fails_on_low_output_storage(tmp_path: Path):
    module = _load_module()
    root = _write_fake_host_root(tmp_path)
    checker = module.JetsonHostReadiness(
        host_root=root,
        which=lambda name: f'/usr/bin/{name}',
        disk_usage=lambda path: _disk_usage(free_gib=0.01),
        machine=lambda: 'aarch64',
    )

    report = checker.build_report(_options(module, tmp_path))

    assert report['status'] == 'FAIL'
    assert any(check['id'] == 'storage_output_dir' and check['status'] == 'fail'
               for check in report['checks'])


def test_host_readiness_fails_when_bag_dir_is_missing(tmp_path: Path):
    module = _load_module()
    root = _write_fake_host_root(tmp_path)
    options = _options(module, tmp_path)
    missing_bag_dir = tmp_path / 'missing_bag_mount'
    options = module.HostReadinessOptions(
        output_dir=options.output_dir,
        bag_dir=missing_bag_dir,
        expected_bag_minutes=options.expected_bag_minutes,
        estimated_bag_mbps=options.estimated_bag_mbps,
        bag_reserve_gb=options.bag_reserve_gb,
        min_bag_free_gb=options.min_bag_free_gb,
        min_output_free_gb=options.min_output_free_gb,
        min_memory_available_gb=options.min_memory_available_gb,
    )
    checker = module.JetsonHostReadiness(
        host_root=root,
        which=lambda name: f'/usr/bin/{name}',
        disk_usage=lambda path: _disk_usage(),
        machine=lambda: 'aarch64',
    )

    report = checker.build_report(options)

    assert report['status'] == 'FAIL'
    assert any(check['id'] == 'storage_bag_dir' and check['status'] == 'fail'
               for check in report['checks'])


def test_host_readiness_cli_writes_reports(tmp_path: Path):
    root = _write_fake_host_root(tmp_path)
    output_dir = tmp_path / 'out'
    bag_dir = tmp_path / 'bags'
    bag_dir.mkdir()
    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    for name in ('ros2', 'colcon', 'tegrastats', 'nvpmodel', 'jetson_clocks'):
        tool = fake_bin / name
        tool.write_text('#!/usr/bin/env sh\nexit 0\n', encoding='utf-8')
        tool.chmod(0o755)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            '--host-root',
            str(root),
            '--output-dir',
            str(output_dir),
            '--bag-dir',
            str(bag_dir),
            '--expected-bag-minutes',
            '0.01',
            '--estimated-bag-mbps',
            '1.0',
            '--bag-reserve-gb',
            '0.0',
            '--min-bag-free-gb',
            '0.001',
            '--min-output-free-gb',
            '0.001',
            '--min-memory-available-gb',
            '0.001',
            '--json',
        ],
        check=False,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
        env={'PATH': str(fake_bin)},
    )
    report = json.loads((output_dir / 'jetson_mid360_host_readiness.json').read_text())

    assert result.returncode == 0
    assert json.loads(result.stdout)['host']['model'] == 'NVIDIA Jetson Orin NX'
    assert report['counts']['fail'] == 0
    assert (output_dir / 'jetson_mid360_host_readiness.md').is_file()
