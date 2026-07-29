"""Tests for stable v0.9 release-candidate publication."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'release_channel.py'
SPEC = importlib.util.spec_from_file_location('release_channel', SCRIPT)
assert SPEC is not None and SPEC.loader is not None
CHANNEL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CHANNEL)


@pytest.mark.parametrize('version', ('0.1.0', '0.7.0', '0.8.99'))
def test_versions_before_v09_are_prereleases(version):
    assert CHANNEL.release_channel(version) == 'prerelease'


@pytest.mark.parametrize('version', ('0.9.0', '0.9.7', '1.0.0', '2.1.3'))
def test_v09_and_later_are_stable_publications(version):
    assert CHANNEL.release_channel(version) == 'stable'


@pytest.mark.parametrize('version', ('v0.9.0', '0.9', '00.9.0', '0.9.0-rc1'))
def test_noncanonical_versions_are_rejected(version):
    with pytest.raises(ValueError, match='canonical MAJOR.MINOR.PATCH'):
        CHANNEL.release_channel(version)


def test_cli_reports_current_stable_candidate():
    version = (ROOT / 'VERSION').read_text(encoding='utf-8').strip()
    result = subprocess.run(
        [sys.executable, str(SCRIPT), version],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    assert result.stdout == 'stable\n'
    assert result.stderr == ''


def test_release_workflow_uses_classified_channel():
    workflow = (
        ROOT / '.github' / 'workflows' / 'release.yml'
    ).read_text(encoding='utf-8')

    assert 'scripts/release_channel.py "${VERSION}"' in workflow
    assert 'prerelease=${PRERELEASE}' in workflow
    assert "needs.metadata.outputs.prerelease == 'true'" in workflow
