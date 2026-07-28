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
#    copyright notice, this list of conditions and the following disclaimer
#    in the documentation and/or other materials provided with the
#    distribution.
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

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'check_version_alignment.py'


def _load_module():
    spec = importlib.util.spec_from_file_location('check_version_alignment', SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


MODULE = _load_module()
CURRENT_VERSION = (REPO_ROOT / 'VERSION').read_text(encoding='utf-8').strip()


def _copy_contract(tmp_path: Path) -> Path:
    root = tmp_path / 'repo'
    root.mkdir()
    for filename in (
        'VERSION',
        'CHANGELOG.md',
        'CITATION.cff',
        'README.md',
        'CONTRIBUTING.md',
        'mkdocs.yml',
    ):
        shutil.copy2(REPO_ROOT / filename, root / filename)
    for relative in (
        Path('docs/index.md'),
        Path('docs/comparison.md'),
        Path(f'docs/releases/v{CURRENT_VERSION}.md'),
    ):
        destination = root / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(REPO_ROOT / relative, destination)
    for package in MODULE.CORE_PACKAGES:
        destination = root / package
        destination.mkdir()
        shutil.copy2(REPO_ROOT / package / 'package.xml', destination / 'package.xml')
        shutil.copy2(
            REPO_ROOT / package / 'CHANGELOG.rst',
            destination / 'CHANGELOG.rst',
        )
    return root


def test_repository_version_contract_passes():
    result = MODULE.validate_repository(REPO_ROOT)

    assert result.ok, result.errors
    assert result.version == (REPO_ROOT / 'VERSION').read_text().strip()
    assert len(result.checked_surfaces) == 17
    assert result.checked_surfaces[0] == 'VERSION'
    assert all(not Path(surface).is_absolute() for surface in result.checked_surfaces)


@pytest.mark.parametrize(
    ('relative', 'old', 'new', 'error_fragment'),
    (
        ('lidarslam/package.xml', f'<version>{CURRENT_VERSION}</version>',
         '<version>9.9.9</version>', 'lidarslam/package.xml'),
        ('CITATION.cff', f'version: {CURRENT_VERSION}',
         'version: 9.9.9', 'CITATION.cff'),
        (
            'README.md',
            f'[v{CURRENT_VERSION}](docs/releases/v{CURRENT_VERSION}.md)',
            f'[release](docs/releases/v{CURRENT_VERSION}.md)',
            'README.md',
        ),
        (
            'docs/comparison.md',
            f'`v{CURRENT_VERSION}` is the current tagged prerelease.',
            '`v9.9.9` is the current tagged prerelease.',
            'docs/comparison.md',
        ),
    ),
)
def test_version_contract_rejects_drift(
    tmp_path: Path,
    relative: str,
    old: str,
    new: str,
    error_fragment: str,
):
    root = _copy_contract(tmp_path)
    path = root / relative
    original = path.read_text(encoding='utf-8')
    assert old in original
    path.write_text(original.replace(old, new, 1), encoding='utf-8')

    result = MODULE.validate_repository(root)

    assert not result.ok
    assert any(error_fragment in error for error in result.errors)


def test_version_contract_rejects_non_ros_version(tmp_path: Path):
    root = _copy_contract(tmp_path)
    (root / 'VERSION').write_text('0.9.0-rc.1\n', encoding='utf-8')

    result = MODULE.validate_repository(root)

    assert not result.ok
    assert any('ROS-compatible MAJOR.MINOR.PATCH' in error for error in result.errors)


def test_contract_is_not_hardcoded_to_current_version(tmp_path: Path):
    root = _copy_contract(tmp_path)
    next_version = '9.9.9'
    for path in tuple(root.rglob('*')):
        if not path.is_file():
            continue
        text = path.read_text(encoding='utf-8')
        path.write_text(
            text.replace(CURRENT_VERSION, next_version),
            encoding='utf-8',
        )
    old_notes = root / 'docs' / 'releases' / f'v{CURRENT_VERSION}.md'
    old_notes.rename(root / 'docs' / 'releases' / f'v{next_version}.md')

    result = MODULE.validate_repository(root)

    assert result.ok, result.errors
    assert result.version == next_version


def test_cli_json_is_machine_readable():
    completed = subprocess.run(
        [sys.executable, str(SCRIPT_PATH), '--root', str(REPO_ROOT), '--json'],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    result = json.loads(completed.stdout)
    assert result['status'] == 'PASS'
    assert result['version'] == CURRENT_VERSION


def _initialize_tagged_repository(root: Path) -> str:
    subprocess.run(['git', 'init', '-q'], cwd=root, check=True)
    subprocess.run(
        ['git', 'config', 'user.email', 'version-contract@example.invalid'],
        cwd=root,
        check=True,
    )
    subprocess.run(
        ['git', 'config', 'user.name', 'Version Contract Test'],
        cwd=root,
        check=True,
    )
    subprocess.run(['git', 'add', '.'], cwd=root, check=True)
    subprocess.run(['git', 'commit', '-qm', 'release fixture'], cwd=root, check=True)
    tag = f'v{CURRENT_VERSION}'
    subprocess.run(['git', 'tag', tag], cwd=root, check=True)
    return tag


def test_tag_contract_accepts_exact_tag_at_checkout(tmp_path: Path):
    root = _copy_contract(tmp_path)
    tag = _initialize_tagged_repository(root)

    result = MODULE.validate_repository(root, tag=tag)

    assert result.ok, result.errors
    assert f'git:refs/tags/{tag}' in result.checked_surfaces


def test_tag_contract_rejects_tag_on_another_commit(tmp_path: Path):
    root = _copy_contract(tmp_path)
    tag = _initialize_tagged_repository(root)
    (root / 'unreleased-change').write_text('new commit\n', encoding='utf-8')
    subprocess.run(['git', 'add', '.'], cwd=root, check=True)
    subprocess.run(['git', 'commit', '-qm', 'move checkout'], cwd=root, check=True)

    result = MODULE.validate_repository(root, tag=tag)

    assert not result.ok
    assert any('not checkout' in error for error in result.errors)


def test_tag_contract_rejects_wrong_tag_name(tmp_path: Path):
    root = _copy_contract(tmp_path)
    _initialize_tagged_repository(root)

    result = MODULE.validate_repository(root, tag='v9.9.9')

    assert not result.ok
    assert any('does not match' in error for error in result.errors)
