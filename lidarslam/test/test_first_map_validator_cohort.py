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
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Tests for the bounded independent first-map cohort contract."""

from __future__ import annotations

import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'first_map_validator_cohort.py'
CONTRACT = (
    ROOT / 'docs' / 'contracts' / 'first-map-validator-cohort-v1.json'
)
SCHEMA = ROOT / 'docs' / 'schemas' / 'first-map-validator-cohort-v1.schema.json'


def _load_module():
    spec = importlib.util.spec_from_file_location('cohort_contract', SCRIPT)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _payload(path: Path) -> dict:
    return json.loads(path.read_text(encoding='utf-8'))


def _ready_contract() -> dict:
    contract = _payload(CONTRACT)
    contract['status'] = 'COPY_READY_NOT_AUTHORIZED'
    gates = contract['launch_gates']
    gates.update({
        'public_revision': 'a' * 40,
        'public_revision_resolvable': True,
        'comparable_docker_row': True,
        'comparable_source_row': True,
        'canonical_documentation_path': 'docker-first-map',
        'canonical_documentation_url': (
            'https://rsasaki0109.github.io/lidar_slam_ros2/'
            'getting-started.html#docker-first-map-no-ros-2-workspace'
        ),
        'canonical_runtime_ref': (
            'ghcr.io/rsasaki0109/lidar_slam_ros2@sha256:' + 'b' * 64
        ),
        'copy_ready_handoff_public': True,
    })
    return contract


def test_tracked_contract_is_valid_waiting_and_read_only():
    module = _load_module()

    report = module.validate_contract(_payload(CONTRACT), _payload(SCHEMA))

    assert report['status'] == 'WAITING_FOR_PUBLIC_GATES'
    assert report['copy_ready'] is False
    assert report['accepted_target'] == 3
    assert report['initial_attempt_cap'] == 5
    assert report['hard_attempt_cap'] == 10
    assert report['max_concurrent_attempts'] == 2
    assert report['community_posts_authorized'] is False
    assert report['github_writes_authorized'] is False
    assert report['remote_mutations_performed'] is False


def test_render_is_blocked_until_all_public_product_gates_pass():
    module = _load_module()

    with pytest.raises(module.CohortError, match='public revision'):
        module.render_recruitment(_payload(CONTRACT))

    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--render', '--json'],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 1
    assert json.loads(result.stdout)['status'] == (
        'COHORT_INVALID_OR_BLOCKED'
    )


def test_ready_contract_renders_bounded_privacy_first_recruitment():
    module = _load_module()
    contract = _ready_contract()

    report = module.validate_contract(contract, _payload(SCHEMA))
    rendered = module.render_recruitment(contract)

    assert report['status'] == 'COPY_READY_NOT_AUTHORIZED'
    assert report['copy_ready'] is True
    assert 'a' * 40 in rendered
    assert (
        'Exact product identity: '
        'ghcr.io/rsasaki0109/lidar_slam_ros2@sha256:' + 'b' * 64
    ) in rendered
    assert 'at most 5 attempts' in rendered
    assert 'no more than 2 at once' in rendered
    assert 'not a lidarslam_ros2 maintainer' in rendered
    assert 'Please report both PASS and FAIL outcomes' in rendered
    assert 'Attach only the reviewed' in rendered
    assert 'Do not upload a bag, map, trajectory' in rendered
    assert 'No product telemetry is used' in rendered
    assert 'explicit community-write approval' in rendered


@pytest.mark.parametrize(
    ('path', 'value', 'match'),
    [
        (
            ('authority', 'community_posts_authorized'),
            True,
            'schema validation',
        ),
        (
            ('capacity', 'max_concurrent_attempts'),
            3,
            'schema validation',
        ),
        (
            ('status',),
            'COPY_READY_NOT_AUTHORIZED',
            'status must be WAITING_FOR_PUBLIC_GATES',
        ),
        (
            ('launch_gates', 'canonical_documentation_path'),
            'docker-first-map',
            'path and URL must be set together',
        ),
    ],
)
def test_contract_rejects_authority_capacity_and_state_drift(
    path: tuple[str, ...],
    value,
    match: str,
):
    module = _load_module()
    contract = copy.deepcopy(_payload(CONTRACT))
    target = contract
    for part in path[:-1]:
        target = target[part]
    target[path[-1]] = value

    with pytest.raises(module.CohortError, match=match):
        module.validate_contract(contract, _payload(SCHEMA))
    with pytest.raises(module.CohortError, match='recruitment text is blocked'):
        module.render_recruitment(contract)


def test_every_required_stop_condition_is_fixed():
    contract = _payload(CONTRACT)
    assert set(contract['stop_conditions']) == {
        'public-source-or-documentation-drift',
        'supported-p0-open',
        'failed-release-gate',
        'two-unreviewed-receipts',
        'two-attempts-share-one-blocker',
        'privacy-or-safety-incident',
        'completion-below-80-percent-at-attempt-10',
        'median-active-time-above-10-minutes-at-attempt-10',
    }


@pytest.mark.parametrize(
    ('path', 'url', 'runtime_ref', 'match'),
    [
        (
            'docker-first-map',
            'https://rsasaki0109.github.io/lidar_slam_ros2/'
            'getting-started.html#docker-first-map-no-ros-2-workspace',
            'a' * 40,
            'Docker path requires an immutable GHCR digest',
        ),
        (
            'source-quickstart',
            'https://rsasaki0109.github.io/lidar_slam_ros2/'
            'getting-started.html#1-install-and-build-from-source',
            'ghcr.io/rsasaki0109/lidar_slam_ros2@sha256:' + 'b' * 64,
            'source path runtime identity must equal the public revision',
        ),
    ],
)
def test_runtime_identity_must_match_the_selected_path(
    path: str,
    url: str,
    runtime_ref: str,
    match: str,
):
    module = _load_module()
    contract = _ready_contract()
    gates = contract['launch_gates']
    gates['canonical_documentation_path'] = path
    gates['canonical_documentation_url'] = url
    gates['canonical_runtime_ref'] = runtime_ref

    with pytest.raises(module.CohortError, match=match):
        module.validate_contract(contract, _payload(SCHEMA))
