"""lidarslam-map from the pip package (CI smoke test).

The package carries the product files in the tree the ROS 2 package installs under
share/lidarslam, from the same file lists, and the lidarslam-map command runs its CLI with this
interpreter. A full map of a real log through `lidarslam-map start` is checked in
docs/roadmap/ros-free-core-2026-11.md.
"""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys

import pytest

lidarslam = pytest.importorskip('lidarslam')
product = pytest.importorskip('lidarslam.product')
if not product.CLI_PATH.is_file():
    pytest.skip('lidarslam built without the product files', allow_module_level=True)

REPO_ROOT = Path(__file__).resolve().parents[2]
SHARE = product.SHARE_DIR


def _listed(name: str) -> list[str]:
    lines = (REPO_ROOT / 'lidarslam' / name).read_text(encoding='utf-8').splitlines()
    return [line.strip() for line in lines if line.strip() and not line.startswith('#')]


def _lidarslam_map(*args: str, env: dict[str, str] | None = None) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, '-m', 'lidarslam.product', *args], check=False, capture_output=True,
        text=True, env=env, timeout=120)


def test_package_holds_the_installed_product_tree() -> None:
    for name in _listed('product-runtime-files.txt'):
        assert (SHARE / 'product' / 'scripts' / name).is_file(), name
    for name in _listed('product-schema-files.txt'):
        assert (SHARE / 'product' / 'schemas' / name).is_file(), name
    for name in ('VERSION', 'product-runtime-files.txt', 'product-build-info.json'):
        assert (SHARE / 'product' / name).is_file(), name
    assert (SHARE / 'product' / 'completions' / 'lidarslam-map.bash').is_file()
    params = REPO_ROOT / 'lidarslam' / 'param'
    for path in params.rglob('*'):
        if path.is_file():
            assert (SHARE / 'param' / path.relative_to(params)).is_file(), path
    assert os.access(product.CLI_PATH, os.X_OK)


def test_lidarslam_map_runs_the_product_cli() -> None:
    version = (REPO_ROOT / 'VERSION').read_text(encoding='utf-8').strip()
    result = _lidarslam_map('--version')
    assert result.returncode == 0, result.stderr
    assert version in result.stdout

    result = _lidarslam_map('run', '--help')
    assert result.returncode == 0, result.stderr
    assert '--engine' in result.stdout


def test_browser_preview_imports_from_the_package() -> None:
    # The preview scripts import the product scripts as lidarslam_benchmark_tools.
    result = subprocess.run(
        [sys.executable, '-c',
         'import lidarslam_benchmark_tools.mid360_robot_3d_map_preview, '
         'lidarslam_benchmark_tools.verify_autoware_map'],
        check=False, capture_output=True, text=True, cwd=SHARE, timeout=120)
    assert result.returncode == 0, result.stderr
