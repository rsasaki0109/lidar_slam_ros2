"""Tests for release evidence promotion and digest-pinned rollback planning."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
TOOL = REPO_ROOT / 'scripts' / 'manage_last_known_good.py'
TRACKED_LEDGER = REPO_ROOT / 'configs' / 'release' / 'last-known-good.json'
SCHEMA = REPO_ROOT / 'docs' / 'schemas' / 'last-known-good-v1.schema.json'


def _evidence(distro: str, version: str = '0.9.0') -> dict[str, object]:
    return {
        'schema_version': 1,
        'status': 'PASS',
        'ros_distro': distro,
        'platform': 'linux/amd64',
        'tag': (
            f'ghcr.io/rsasaki0109/lidar_slam_ros2:v{version}-{distro}'
        ),
        'digest': f'sha256:{("a" if distro == "humble" else "b") * 64}',
        'git_commit': 'c' * 40,
        'product_version': version,
        'cli_version': f'lidarslam_ros2 {version}',
    }


def _run(*args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(TOOL), *args],
        cwd=REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )


def test_tracked_ledger_is_honestly_unassigned():
    completed = _run('verify', str(TRACKED_LEDGER))
    assert completed.returncode == 0, completed.stderr
    assert 'ledger is unassigned' in completed.stdout

    required = _run('verify', str(TRACKED_LEDGER), '--require-assigned')
    assert required.returncode == 1
    assert 'last-known-good is unassigned' in required.stderr


def test_release_evidence_validation_is_distro_specific(tmp_path):
    evidence = tmp_path / 'release-image-humble.json'
    evidence.write_text(json.dumps(_evidence('humble')), encoding='utf-8')

    valid = _run(
        'validate-evidence',
        str(evidence),
        '--ros-distro', 'humble',
    )
    assert valid.returncode == 0, valid.stderr
    wrong_distro = _run(
        'validate-evidence',
        str(evidence),
        '--ros-distro', 'jazzy',
    )
    assert wrong_distro.returncode == 1
    assert 'ros_distro must be jazzy' in wrong_distro.stderr


def test_promote_and_plan_require_matching_cross_distro_evidence(tmp_path):
    humble = tmp_path / 'release-image-humble.json'
    jazzy = tmp_path / 'release-image-jazzy.json'
    ledger = tmp_path / 'last-known-good.json'
    humble.write_text(json.dumps(_evidence('humble')), encoding='utf-8')
    jazzy.write_text(json.dumps(_evidence('jazzy')), encoding='utf-8')

    promoted = _run(
        'promote',
        '--humble', str(humble),
        '--jazzy', str(jazzy),
        '--output', str(ledger),
        '--reason', 'Both installed image and real-data gates passed.',
        '--promoted-at', '2026-07-28T12:00:00Z',
    )
    assert promoted.returncode == 0, promoted.stderr
    value = json.loads(ledger.read_text(encoding='utf-8'))
    assert value['status'] == 'assigned'
    assert value['release_tag'] == 'v0.9.0'
    assert value['images']['humble']['digest'] == f'sha256:{"a" * 64}'
    assert value['images']['jazzy']['digest'] == f'sha256:{"b" * 64}'

    plan = _run(
        'plan',
        str(ledger),
        '--ros-distro', 'jazzy',
    )
    assert plan.returncode == 0, plan.stderr
    assert f'@sha256:{"b" * 64}' in plan.stdout
    assert 'gh attestation verify' in plan.stdout
    assert 'lidarslam-map --version' in plan.stdout
    assert ':v0.9.0-jazzy' not in plan.stdout


def test_promotion_rejects_mixed_release_versions(tmp_path):
    humble = tmp_path / 'humble.json'
    jazzy = tmp_path / 'jazzy.json'
    ledger = tmp_path / 'last-known-good.json'
    humble.write_text(json.dumps(_evidence('humble', '0.9.0')), encoding='utf-8')
    jazzy.write_text(json.dumps(_evidence('jazzy', '1.0.0')), encoding='utf-8')

    completed = _run(
        'promote',
        '--humble', str(humble),
        '--jazzy', str(jazzy),
        '--output', str(ledger),
        '--reason', 'invalid mixed release',
    )
    assert completed.returncode == 1
    assert 'Humble and Jazzy product versions differ' in completed.stderr
    assert not ledger.exists()


def test_promotion_reports_malformed_evidence_without_traceback(tmp_path):
    humble = tmp_path / 'humble.json'
    jazzy = tmp_path / 'jazzy.json'
    ledger = tmp_path / 'last-known-good.json'
    malformed = _evidence('humble')
    malformed['product_version'] = {'unexpected': 'object'}
    humble.write_text(json.dumps(malformed), encoding='utf-8')
    jazzy.write_text(json.dumps(_evidence('jazzy')), encoding='utf-8')

    completed = _run(
        'promote',
        '--humble', str(humble),
        '--jazzy', str(jazzy),
        '--output', str(ledger),
        '--reason', 'malformed fixture',
    )
    assert completed.returncode == 1
    assert 'product_version must be numeric SemVer' in completed.stderr
    assert 'Traceback' not in completed.stderr
    assert not ledger.exists()


def test_promotion_never_overwrites_an_assigned_ledger(tmp_path):
    humble = tmp_path / 'humble.json'
    jazzy = tmp_path / 'jazzy.json'
    ledger = tmp_path / 'last-known-good.json'
    humble.write_text(json.dumps(_evidence('humble')), encoding='utf-8')
    jazzy.write_text(json.dumps(_evidence('jazzy')), encoding='utf-8')
    args = (
        'promote',
        '--humble', str(humble),
        '--jazzy', str(jazzy),
        '--output', str(ledger),
        '--reason', 'accepted',
        '--promoted-at', '2026-07-28T12:00:00Z',
    )
    assert _run(*args).returncode == 0
    original = ledger.read_bytes()

    repeated = _run(*args)
    assert repeated.returncode == 1
    assert 'refusing to replace assigned ledger' in repeated.stderr
    assert ledger.read_bytes() == original


def test_docs_ci_and_release_bundle_publish_the_rollback_contract():
    schema = json.loads(SCHEMA.read_text(encoding='utf-8'))
    runbook = (REPO_ROOT / 'docs' / 'release-rollback.md').read_text(
        encoding='utf-8'
    )
    distribution = (REPO_ROOT / 'docs' / 'distribution.md').read_text(
        encoding='utf-8'
    )
    reliability = (
        REPO_ROOT / 'docs' / 'operational-reliability.md'
    ).read_text(encoding='utf-8')
    main_workflow = (
        REPO_ROOT / '.github' / 'workflows' / 'main.yml'
    ).read_text(encoding='utf-8')
    release_workflow = (
        REPO_ROOT / '.github' / 'workflows' / 'release.yml'
    ).read_text(encoding='utf-8')

    assert schema['properties']['schema_version']['const'] == 1
    assert set(schema['properties']['status']['enum']) == {
        'assigned',
        'unassigned',
    }
    assert 'intentionally `unassigned`' in runbook
    assert 'repository@sha256:...' in runbook
    assert 'release-rollback.md' in distribution
    assert 'Automated tooling; first promotion pending' in reliability
    assert 'graph_based_slam/test/test_last_known_good.py' in main_workflow
    assert 'graph_based_slam/test/test_last_known_good.py' in release_workflow
    assert 'Validate rollback evidence inputs' in release_workflow
    assert 'configs/release/last-known-good.json' in release_workflow
    assert 'scripts/manage_last_known_good.py' in release_workflow
    assert 'docs/release-rollback.md' in release_workflow
