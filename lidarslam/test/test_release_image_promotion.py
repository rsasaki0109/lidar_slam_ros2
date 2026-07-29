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

"""Regression tests for immutable release image promotion."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tarfile

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = REPO_ROOT / 'scripts'


def _load_module(filename: str, name: str):
    sys.path.insert(0, str(SCRIPTS))
    try:
        spec = importlib.util.spec_from_file_location(name, SCRIPTS / filename)
        assert spec is not None
        assert spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(SCRIPTS))


def _record(distro: str, digest_char: str) -> dict:
    create = _load_module(
        'create_release_image_record.py',
        f'create_release_image_record_{distro}',
    )
    return create.build_release_image_record(
        ros_distro=distro,
        platform='linux/amd64',
        tag=f'ghcr.io/example/lidarslam_ros2:v0.7.0-{distro}',
        digest='sha256:' + digest_char * 64,
        git_commit='c' * 40,
        product_version='0.7.0',
        cli_version='lidarslam_ros2 0.7.0',
    )


def _completed(
    args: list[str],
    *,
    returncode: int = 0,
    stdout: str = '',
    stderr: str = '',
) -> subprocess.CompletedProcess[str]:
    return subprocess.CompletedProcess(args, returncode, stdout, stderr)


def test_record_builder_rejects_cross_field_mismatch():
    """Reject a tag whose distro conflicts with its record."""
    module = _load_module(
        'create_release_image_record.py',
        'create_release_image_record_mismatch',
    )
    with pytest.raises(ValueError, match='tag distro'):
        module.build_release_image_record(
            ros_distro='humble',
            platform='linux/amd64',
            tag='ghcr.io/example/lidarslam_ros2:v0.7.0-jazzy',
            digest='sha256:' + 'a' * 64,
            git_commit='b' * 40,
            product_version='0.7.0',
            cli_version='lidarslam_ros2 0.7.0',
        )


def test_dry_run_preflights_both_digests_without_mutation():
    """Dry-run should inspect both digests without creating tags."""
    module = _load_module(
        'promote_release_images.py',
        'promote_release_images_dry_run',
    )
    records = [_record('humble', 'a'), _record('jazzy', 'b')]
    calls: list[list[str]] = []

    def runner(args, **_kwargs):
        calls.append(args)
        reference = args[4]
        for record in records:
            if reference.endswith('@' + record['digest']):
                return _completed(
                    args,
                    stdout=json.dumps({'digest': record['digest']}),
                )
        return _completed(args, returncode=1, stderr='tag not found')

    report = module.promote_release_images(
        records,
        expected_repository='example/lidarslam_ros2',
        apply=False,
        runner=runner,
    )

    assert report['status'] == 'PASS'
    assert report['mode'] == 'dry_run'
    assert report['created_tags'] == []
    assert report['reused_tags'] == []
    assert {image['action'] for image in report['images']} == {'create'}
    assert not any('create' in args for args in calls)


def test_apply_reuses_matching_tag_and_creates_only_missing_tag():
    """Applied promotion should reuse matches and create only missing tags."""
    module = _load_module(
        'promote_release_images.py',
        'promote_release_images_apply',
    )
    records = [_record('humble', 'a'), _record('jazzy', 'b')]
    tag_digests = {records[0]['tag']: records[0]['digest']}
    create_calls: list[list[str]] = []

    def runner(args, **_kwargs):
        if args[3] == 'create':
            create_calls.append(args)
            tag_digests[args[5]] = args[6].split('@', 1)[1]
            return _completed(args)
        reference = args[4]
        if '@' in reference:
            digest = reference.split('@', 1)[1]
            return _completed(args, stdout=json.dumps({'digest': digest}))
        digest = tag_digests.get(reference)
        if digest is None:
            return _completed(args, returncode=1, stderr='tag not found')
        return _completed(args, stdout=json.dumps({'digest': digest}))

    report = module.promote_release_images(
        records,
        expected_repository='example/lidarslam_ros2',
        apply=True,
        runner=runner,
    )

    assert report['mode'] == 'applied'
    assert report['reused_tags'] == [records[0]['tag']]
    assert report['created_tags'] == [records[1]['tag']]
    assert len(create_calls) == 1
    assert create_calls[0][5] == records[1]['tag']
    assert create_calls[0][6].endswith('@' + records[1]['digest'])


def test_conflicting_version_tag_fails_before_any_creation():
    """A conflicting version tag should fail before registry mutation."""
    module = _load_module(
        'promote_release_images.py',
        'promote_release_images_conflict',
    )
    records = [_record('humble', 'a'), _record('jazzy', 'b')]
    create_calls: list[list[str]] = []

    def runner(args, **_kwargs):
        if args[3] == 'create':
            create_calls.append(args)
            return _completed(args)
        reference = args[4]
        if '@' in reference:
            digest = reference.split('@', 1)[1]
            return _completed(args, stdout=json.dumps({'digest': digest}))
        if reference == records[1]['tag']:
            return _completed(
                args,
                stdout=json.dumps({'digest': 'sha256:' + 'f' * 64}),
            )
        return _completed(args, returncode=1, stderr='tag not found')

    with pytest.raises(ValueError, match='refusing to move'):
        module.promote_release_images(
            records,
            expected_repository='example/lidarslam_ros2',
            apply=True,
            runner=runner,
        )
    assert create_calls == []


def test_promotion_rejects_incomplete_or_foreign_record_set():
    """Promotion should require the exact repository and distro pair."""
    module = _load_module(
        'promote_release_images.py',
        'promote_release_images_invalid_set',
    )
    humble = _record('humble', 'a')
    with pytest.raises(ValueError, match='exactly two'):
        module.promote_release_images(
            [humble],
            expected_repository='example/lidarslam_ros2',
            apply=False,
        )
    with pytest.raises(ValueError, match='repository'):
        module.promote_release_images(
            [humble, _record('jazzy', 'b')],
            expected_repository='other/lidarslam_ros2',
            apply=False,
        )


def test_release_bundle_is_deterministic_and_manifest_backed(tmp_path: Path):
    """Repeated builds should be identical and contain a checksum manifest."""
    module = _load_module(
        'build_release_bundle.py',
        'build_release_bundle_deterministic',
    )
    first = tmp_path / 'first.tar.gz'
    second = tmp_path / 'second.tar.gz'
    kwargs = {
        'tag': 'v0.9.0',
        'git_commit': 'd' * 40,
    }

    first_manifest = module.build_release_bundle(
        REPO_ROOT,
        first,
        **kwargs,
    )
    second_manifest = module.build_release_bundle(
        REPO_ROOT,
        second,
        **kwargs,
    )

    assert hashlib.sha256(first.read_bytes()).digest() == hashlib.sha256(
        second.read_bytes()
    ).digest()
    assert first_manifest == second_manifest
    paths = {item['path'] for item in first_manifest['files']}
    assert 'docs/evidence/bounded-filesystem-exhaustion-2026-07-29.md' in paths
    assert 'docs/evidence/recovery-command-contract-2026-07-29.md' in paths
    assert 'docs/schemas/release-bundle-manifest-v1.schema.json' in paths
    assert 'docs/schemas/release-promotion-v1.schema.json' in paths
    assert 'scripts/promote_release_images.py' in paths
    assert 'scripts/release_channel.py' in paths
    assert 'docs/releases/v0.9.0.md' in paths

    with tarfile.open(first, mode='r:gz') as archive:
        names = archive.getnames()
        assert all(name.startswith('release_bundle/') for name in names)
        assert not any(name.endswith('.pcd') for name in names)
        embedded = json.load(
            archive.extractfile(
                'release_bundle/release-bundle-manifest-v1.json'
            )
        )
    assert embedded == first_manifest


def test_release_bundle_refuses_version_mismatch_and_overwrite(tmp_path: Path):
    """Bundle creation should reject mismatched versions and overwrites."""
    module = _load_module(
        'build_release_bundle.py',
        'build_release_bundle_refusal',
    )
    output = tmp_path / 'bundle.tar.gz'
    with pytest.raises(ValueError, match='does not match VERSION'):
        module.build_release_bundle(
            REPO_ROOT,
            output,
            tag='v0.8.0',
            git_commit='e' * 40,
        )

    module.build_release_bundle(
        REPO_ROOT,
        output,
        tag='v0.9.0',
        git_commit='e' * 40,
    )
    with pytest.raises(ValueError, match='refusing to overwrite'):
        module.build_release_bundle(
            REPO_ROOT,
            output,
            tag='v0.9.0',
            git_commit='e' * 40,
        )
