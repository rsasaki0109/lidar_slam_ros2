#!/usr/bin/env python3
"""Read-only, fail-closed audit of a published lidarslam_ros2 release."""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import os
from pathlib import Path, PurePosixPath
import sys
import tarfile
from typing import Any
import urllib.error
import urllib.parse
import urllib.request

import jsonschema


REPO_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY = 'rsasaki0109/lidar_slam_ros2'
REPORT_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/published-release-v1.schema.json'
)
MAX_JSON_BYTES = 2 * 1024 * 1024
MAX_BUNDLE_BYTES = 128 * 1024 * 1024
MAX_RELEASE_ASSET_BYTES = 160 * 1024 * 1024
MAX_UNPACKED_BUNDLE_BYTES = 256 * 1024 * 1024
MAX_REGISTRY_TOKEN_BYTES = 64 * 1024
MAX_REGISTRY_MANIFEST_BYTES = 4 * 1024 * 1024
OCI_MANIFEST_ACCEPT = ', '.join((
    'application/vnd.oci.image.index.v1+json',
    'application/vnd.docker.distribution.manifest.list.v2+json',
    'application/vnd.oci.image.manifest.v1+json',
    'application/vnd.docker.distribution.manifest.v2+json',
))


class PublishedReleaseError(ValueError):
    """Remote publication evidence is invalid or cannot be inspected."""


def _schema(name: str) -> dict[str, Any]:
    path = REPO_ROOT / 'docs' / 'schemas' / name
    try:
        payload = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise PublishedReleaseError(f'cannot read schema {path}: {exc}') from exc
    if not isinstance(payload, dict):
        raise PublishedReleaseError(f'schema root must be an object: {path}')
    return payload


def _validate(payload: dict[str, Any], schema_name: str) -> None:
    schema = _schema(schema_name)
    try:
        jsonschema.validators.validator_for(schema).check_schema(schema)
        jsonschema.validators.validator_for(schema)(schema).validate(payload)
    except (jsonschema.SchemaError, jsonschema.ValidationError) as exc:
        location = '.'.join(str(item) for item in exc.absolute_path)
        raise PublishedReleaseError(
            f'{schema_name} validation failed at '
            f'{location or "<root>"}: {exc.message}'
        ) from exc


def _request(url: str, *, limit: int) -> tuple[int, bytes]:
    headers = {
        'Accept': 'application/vnd.github+json',
        'User-Agent': 'lidarslam-published-release-audit/1',
    }
    token = os.environ.get('GITHUB_TOKEN')
    if token and url.startswith('https://api.github.com/'):
        headers['Authorization'] = f'Bearer {token}'
    request = urllib.request.Request(
        url,
        headers=headers,
    )
    try:
        with urllib.request.urlopen(request, timeout=30) as response:
            payload = response.read(limit + 1)
            if len(payload) > limit:
                raise PublishedReleaseError(
                    f'response exceeds {limit} bytes: {url}')
            return response.status, payload
    except urllib.error.HTTPError as exc:
        if exc.code == 404:
            return 404, b''
        raise PublishedReleaseError(
            f'HTTP {exc.code} while reading {url}') from exc
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        raise PublishedReleaseError(f'cannot read {url}: {exc}') from exc


def _request_json(url: str) -> tuple[int, dict[str, Any] | None]:
    status, payload = _request(url, limit=MAX_JSON_BYTES)
    if status == 404:
        return status, None
    try:
        value = json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise PublishedReleaseError(f'invalid JSON from {url}: {exc}') from exc
    if not isinstance(value, dict):
        raise PublishedReleaseError(f'JSON root from {url} is not an object')
    return status, value


def _registry_tag_digest(tag: str) -> str | None:
    """Resolve one public GHCR tag without pulling its image layers."""
    token_url = 'https://ghcr.io/token?' + urllib.parse.urlencode({
        'service': 'ghcr.io',
        'scope': f'repository:{REPOSITORY}:pull',
    })
    status, token_payload = _request(
        token_url,
        limit=MAX_REGISTRY_TOKEN_BYTES,
    )
    if status != 200:
        raise PublishedReleaseError(
            f'GHCR token request unexpectedly returned {status}')
    try:
        token_document = json.loads(token_payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise PublishedReleaseError(
            f'invalid GHCR token response: {exc}') from exc
    if not isinstance(token_document, dict):
        raise PublishedReleaseError('GHCR token response is not an object')
    token = token_document.get('token')
    if not isinstance(token, str) or not token:
        raise PublishedReleaseError('GHCR token response has no token')

    url = f'https://ghcr.io/v2/{REPOSITORY}/manifests/{tag}'
    request = urllib.request.Request(
        url,
        headers={
            'Accept': OCI_MANIFEST_ACCEPT,
            'Authorization': f'Bearer {token}',
            'User-Agent': 'lidarslam-published-release-audit/1',
        },
    )
    try:
        with urllib.request.urlopen(request, timeout=30) as response:
            payload = response.read(MAX_REGISTRY_MANIFEST_BYTES + 1)
            if len(payload) > MAX_REGISTRY_MANIFEST_BYTES:
                raise PublishedReleaseError(
                    f'GHCR manifest exceeds '
                    f'{MAX_REGISTRY_MANIFEST_BYTES} bytes: {tag}')
            digest = response.headers.get('Docker-Content-Digest')
    except urllib.error.HTTPError as exc:
        if exc.code == 404:
            return None
        raise PublishedReleaseError(
            f'HTTP {exc.code} while reading GHCR tag {tag}') from exc
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        raise PublishedReleaseError(
            f'cannot read GHCR tag {tag}: {exc}') from exc
    if (
        not isinstance(digest, str)
        or len(digest) != 71
        or not digest.startswith('sha256:')
        or any(character not in '0123456789abcdef' for character in digest[7:])
    ):
        raise PublishedReleaseError(
            f'GHCR tag {tag} returned invalid digest {digest!r}')
    return digest


def inspect_remote(version: str) -> dict[str, Any]:
    """Inspect release metadata and download its public assets."""
    tag = f'v{version}'
    api = f'https://api.github.com/repos/{REPOSITORY}'
    errors: list[str] = []
    tag_commit: str | None = None
    release: dict[str, Any] | None = None
    asset_payloads: dict[str, bytes] = {}
    image_tag_digests: dict[str, str] = {}

    try:
        status, tag_ref = _request_json(f'{api}/git/ref/tags/{tag}')
        if status == 200:
            if tag_ref is None:
                raise PublishedReleaseError('tag ref response is empty')
            _, commit = _request_json(f'{api}/commits/{tag}')
            if commit is None:
                raise PublishedReleaseError(
                    'tag exists but cannot be resolved to a commit')
            candidate = commit.get('sha')
            if not isinstance(candidate, str):
                raise PublishedReleaseError('tag commit response has no sha')
            tag_commit = candidate
    except PublishedReleaseError as exc:
        errors.append(f'tag commit: {exc}')

    try:
        _, release = _request_json(f'{api}/releases/tags/{tag}')
    except PublishedReleaseError as exc:
        errors.append(f'release: {exc}')

    if release is not None:
        assets = release.get('assets')
        if not isinstance(assets, list):
            errors.append('release assets field is not an array')
        elif len(assets) > 12:
            errors.append(
                f'release has too many assets to audit safely: {len(assets)}')
        else:
            declared_total = sum(
                asset.get('size', 0)
                for asset in assets
                if isinstance(asset, dict)
                and isinstance(asset.get('size'), int)
            )
            if declared_total > MAX_RELEASE_ASSET_BYTES:
                errors.append(
                    'release assets exceed the aggregate download limit')
                assets = []
            for asset in assets:
                if not isinstance(asset, dict):
                    errors.append('release contains a non-object asset')
                    continue
                name = asset.get('name')
                url = asset.get('browser_download_url')
                size = asset.get('size')
                if (
                    not isinstance(name, str)
                    or not isinstance(url, str)
                    or not isinstance(size, int)
                ):
                    errors.append('release asset metadata is incomplete')
                    continue
                limit = (
                    MAX_BUNDLE_BYTES if name.endswith('.tar.gz')
                    else MAX_JSON_BYTES
                )
                if size < 1 or size > limit:
                    errors.append(
                        f'asset {name} has invalid size {size}')
                    continue
                try:
                    status, payload = _request(url, limit=limit)
                    if status != 200:
                        raise PublishedReleaseError(
                            f'asset {name} unexpectedly returned 404')
                    if len(payload) != size:
                        raise PublishedReleaseError(
                            f'asset {name} size mismatch: '
                            f'API={size}, downloaded={len(payload)}'
                        )
                    if name in asset_payloads:
                        raise PublishedReleaseError(
                            f'duplicate release asset: {name}')
                    asset_payloads[name] = payload
                except PublishedReleaseError as exc:
                    errors.append(str(exc))

        for distro in ('humble', 'jazzy'):
            image_tag = f'v{version}-{distro}'
            try:
                digest = _registry_tag_digest(image_tag)
                if digest is None:
                    raise PublishedReleaseError(
                        f'GHCR tag {image_tag} is not published')
                image_tag_digests[
                    f'ghcr.io/{REPOSITORY}:{image_tag}'
                ] = digest
            except PublishedReleaseError as exc:
                errors.append(str(exc))

    return {
        'errors': errors,
        'tag_commit': tag_commit,
        'release': release,
        'asset_payloads': asset_payloads,
        'image_tag_digests': image_tag_digests,
    }


def _json_asset(name: str, payload: bytes) -> dict[str, Any]:
    try:
        value = json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise PublishedReleaseError(f'{name} is not valid JSON: {exc}') from exc
    if not isinstance(value, dict):
        raise PublishedReleaseError(f'{name} JSON root is not an object')
    return value


def verify_release_bundle_payload(
    payload: bytes,
    *,
    version: str,
    tag: str,
    commit: str,
) -> None:
    try:
        archive = tarfile.open(fileobj=io.BytesIO(payload), mode='r:gz')
    except (tarfile.TarError, OSError) as exc:
        raise PublishedReleaseError(f'release bundle is not a valid tar.gz: {exc}')
    with archive:
        members = archive.getmembers()
        if any(not member.isfile() for member in members):
            raise PublishedReleaseError(
                'release bundle contains a non-regular member')
        if sum(member.size for member in members) > MAX_UNPACKED_BUNDLE_BYTES:
            raise PublishedReleaseError(
                'release bundle exceeds the uncompressed size limit')
        names = [member.name for member in members if member.isfile()]
        if len(names) != len(set(names)):
            raise PublishedReleaseError(
                'release bundle contains duplicate file paths')
        if any(
            PurePosixPath(name).is_absolute() or '..' in PurePosixPath(name).parts
            for name in names
        ):
            raise PublishedReleaseError(
                'release bundle contains an unsafe file path')
        manifest_name = 'release_bundle/release-bundle-manifest-v1.json'
        if names.count(manifest_name) != 1:
            raise PublishedReleaseError(
                'release bundle has no unique embedded manifest')
        stream = archive.extractfile(manifest_name)
        if stream is None:
            raise PublishedReleaseError('cannot read embedded bundle manifest')
        manifest = _json_asset(manifest_name, stream.read())
        _validate(manifest, 'release-bundle-manifest-v1.schema.json')
        expected = {
            'tag': tag,
            'product_version': version,
            'git_commit': commit,
        }
        for field, value in expected.items():
            if manifest[field] != value:
                raise PublishedReleaseError(
                    f'bundle manifest {field} differs: '
                    f'{manifest[field]!r} != {value!r}'
                )
        manifest_paths = {
            f"release_bundle/{item['path']}": item
            for item in manifest['files']
        }
        expected_names = {manifest_name, *manifest_paths}
        if set(names) != expected_names:
            raise PublishedReleaseError(
                'bundle contents differ from the embedded manifest')
        for name, item in manifest_paths.items():
            member_stream = archive.extractfile(name)
            if member_stream is None:
                raise PublishedReleaseError(f'cannot read bundle member {name}')
            content = member_stream.read()
            if len(content) != item['size_bytes']:
                raise PublishedReleaseError(
                    f'bundle member size differs: {name}')
            if hashlib.sha256(content).hexdigest() != item['sha256']:
                raise PublishedReleaseError(
                    f'bundle member hash differs: {name}')


def _check(check_id: str, passed: bool, detail: str) -> dict[str, str]:
    return {
        'id': check_id,
        'status': 'PASS' if passed else 'FAIL',
        'detail': detail,
    }


def evaluate_publication(
    *,
    version: str,
    snapshot: dict[str, Any],
) -> dict[str, Any]:
    """Validate one injected or live release snapshot."""
    tag = f'v{version}'
    bundle_name = f'lidarslam_ros2_{tag}_release_bundle.tar.gz'
    required_assets = (
        bundle_name,
        'release-image-humble.json',
        'release-image-jazzy.json',
        'rollback-plan-humble.json',
        'rollback-plan-jazzy.json',
        'release-promotion.json',
    )
    errors = list(snapshot.get('errors', []))
    release = snapshot.get('release')
    tag_commit = snapshot.get('tag_commit')
    asset_payloads = snapshot.get('asset_payloads', {})
    image_tag_digests = snapshot.get('image_tag_digests', {})
    checks: list[dict[str, str]] = []
    asset_reports: list[dict[str, Any]] = []

    if errors:
        status = 'BLOCKED'
    elif release is None:
        status = 'IN_PROGRESS' if tag_commit is not None else 'NOT_PUBLISHED'
    else:
        status = 'BLOCKED'
        checks.extend([
            _check(
                'tag-commit',
                isinstance(tag_commit, str) and len(tag_commit) == 40,
                f'tag commit={tag_commit}',
            ),
            _check(
                'release-tag',
                release.get('tag_name') == tag,
                f"expected {tag}; found {release.get('tag_name')}",
            ),
            _check(
                'release-finalized',
                release.get('draft') is False,
                f"draft={release.get('draft')}",
            ),
            _check(
                'stable-release-channel',
                release.get('prerelease') is False,
                f"prerelease={release.get('prerelease')}",
            ),
            _check(
                'release-url',
                release.get('html_url') == (
                    f'https://github.com/{REPOSITORY}/releases/tag/{tag}'
                ),
                f"html_url={release.get('html_url')}",
            ),
        ])
        if not isinstance(asset_payloads, dict):
            errors.append('asset payloads are not an object')
            asset_payloads = {}
        observed_names = list(asset_payloads)
        checks.append(_check(
            'required-assets',
            set(observed_names) == set(required_assets),
            f'expected={sorted(required_assets)}, '
            f'observed={sorted(observed_names)}',
        ))

        parsed: dict[str, dict[str, Any]] = {}
        schema_by_asset = {
            'release-image-humble.json': 'release-image-v1.schema.json',
            'release-image-jazzy.json': 'release-image-v1.schema.json',
            'rollback-plan-humble.json': 'rollback-plan-v1.schema.json',
            'rollback-plan-jazzy.json': 'rollback-plan-v1.schema.json',
            'release-promotion.json': 'release-promotion-v1.schema.json',
        }
        for name in required_assets:
            payload = asset_payloads.get(name)
            passed = isinstance(payload, bytes)
            detail = 'asset is present' if passed else 'asset is missing'
            if passed:
                try:
                    if name == bundle_name:
                        verify_release_bundle_payload(
                            payload,
                            version=version,
                            tag=tag,
                            commit=tag_commit,
                        )
                    else:
                        value = _json_asset(name, payload)
                        _validate(value, schema_by_asset[name])
                        parsed[name] = value
                    detail = 'asset contract passed'
                except PublishedReleaseError as exc:
                    passed = False
                    detail = str(exc)
            asset_reports.append({
                'name': name,
                'status': 'PASS' if passed else 'FAIL',
                'size_bytes': len(payload) if isinstance(payload, bytes) else 0,
                'sha256': (
                    hashlib.sha256(payload).hexdigest()
                    if isinstance(payload, bytes) else None
                ),
                'detail': detail,
            })

        try:
            images = {
                distro: parsed[f'release-image-{distro}.json']
                for distro in ('humble', 'jazzy')
            }
            rollback = {
                distro: parsed[f'rollback-plan-{distro}.json']
                for distro in ('humble', 'jazzy')
            }
            promotion = parsed['release-promotion.json']
            for distro in ('humble', 'jazzy'):
                image = images[distro]
                plan = rollback[distro]
                expected_tag = (
                    f'ghcr.io/{REPOSITORY}:v{version}-{distro}')
                if (
                    image['ros_distro'] != distro
                    or image['product_version'] != version
                    or image['git_commit'] != tag_commit
                    or image['tag'] != expected_tag
                ):
                    raise PublishedReleaseError(
                        f'{distro} image record differs from release identity')
                for field in (
                    'ros_distro',
                    'product_version',
                    'git_commit',
                    'tag',
                    'digest',
                    'platform',
                ):
                    if plan[field] != image[field]:
                        raise PublishedReleaseError(
                            f'{distro} rollback {field} differs from image record')
                if plan['repository'] != REPOSITORY:
                    raise PublishedReleaseError(
                        f'{distro} rollback repository differs')
                if plan['source_record'] != f'release-image-{distro}.json':
                    raise PublishedReleaseError(
                        f'{distro} rollback source record differs')
            if (
                promotion['mode'] != 'applied'
                or promotion['repository'] != REPOSITORY
                or promotion['product_version'] != version
                or promotion['git_commit'] != tag_commit
                or promotion['moving_tag_mutated'] is not False
            ):
                raise PublishedReleaseError(
                    'promotion record differs from release identity')
            promoted = {
                (item['ros_distro'], item['tag'], item['digest'])
                for item in promotion['images']
            }
            expected_promoted = {
                (distro, images[distro]['tag'], images[distro]['digest'])
                for distro in ('humble', 'jazzy')
            }
            if promoted != expected_promoted:
                raise PublishedReleaseError(
                    'promotion images differ from release image records')
            finalized_tags = (
                set(promotion['created_tags'])
                | set(promotion['reused_tags'])
            )
            if (
                set(promotion['created_tags'])
                & set(promotion['reused_tags'])
                or finalized_tags != {
                    images[distro]['tag']
                    for distro in ('humble', 'jazzy')
                }
            ):
                raise PublishedReleaseError(
                    'promotion created/reused tags are incomplete')
            checks.append(_check(
                'cross-asset-identity',
                True,
                'tag commit, image, rollback, and promotion identities agree',
            ))
            expected_live_digests = {
                images[distro]['tag']: images[distro]['digest']
                for distro in ('humble', 'jazzy')
            }
            if not isinstance(image_tag_digests, dict):
                raise PublishedReleaseError(
                    'live GHCR image-tag digests are not an object')
            observed_live_digests = {
                key: value
                for key, value in image_tag_digests.items()
                if isinstance(key, str) and isinstance(value, str)
            }
            if observed_live_digests != expected_live_digests:
                raise PublishedReleaseError(
                    'live GHCR image-tag digests differ: '
                    f'expected={expected_live_digests!r}, '
                    f'observed={observed_live_digests!r}'
                )
            checks.append(_check(
                'live-image-tag-digests',
                True,
                'Humble and Jazzy GHCR tags still resolve to recorded digests',
            ))
        except (KeyError, TypeError, PublishedReleaseError) as exc:
            failed_id = (
                'live-image-tag-digests'
                if 'GHCR image-tag' in str(exc)
                else 'cross-asset-identity'
            )
            checks.append(_check(
                failed_id,
                False,
                str(exc),
            ))

        if (
            not errors
            and all(check['status'] == 'PASS' for check in checks)
            and all(asset['status'] == 'PASS' for asset in asset_reports)
        ):
            status = 'PUBLISHED'

    report = {
        'schema_version': 1,
        'schema_uri': REPORT_SCHEMA_URI,
        'status': status,
        'repository': REPOSITORY,
        'expected_version': version,
        'expected_tag': tag,
        'remote': {
            'tag_present': tag_commit is not None,
            'tag_commit': tag_commit,
            'release_present': release is not None,
            'draft': release.get('draft') if isinstance(release, dict) else None,
            'prerelease': (
                release.get('prerelease')
                if isinstance(release, dict) else None
            ),
            'html_url': (
                release.get('html_url')
                if isinstance(release, dict) else None
            ),
            'errors': errors,
        },
        'checks': checks,
        'assets': asset_reports,
    }
    schema = _schema('published-release-v1.schema.json')
    jsonschema.Draft202012Validator.check_schema(schema)
    jsonschema.Draft202012Validator(schema).validate(report)
    return report


def _summary(report: dict[str, Any]) -> str:
    lines = [
        f"Published release audit: {report['status']}",
        f"Expected: {report['expected_tag']}",
        f"Tag commit: {report['remote']['tag_commit']}",
    ]
    lines.extend(
        f"  [{check['status']}] {check['id']}: {check['detail']}"
        for check in report['checks']
    )
    lines.extend(
        f"  [{asset['status']}] {asset['name']}: {asset['detail']}"
        for asset in report['assets']
    )
    lines.extend(
        f'  [ERROR] {error}' for error in report['remote']['errors'])
    return '\n'.join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--version',
        default=(REPO_ROOT / 'VERSION').read_text(encoding='utf-8').strip(),
    )
    parser.add_argument('--json', action='store_true')
    parser.add_argument('--output-json', type=Path)
    parser.add_argument('--require-published', action='store_true')
    args = parser.parse_args(argv)
    try:
        snapshot = inspect_remote(args.version)
        report = evaluate_publication(
            version=args.version,
            snapshot=snapshot,
        )
        rendered = json.dumps(report, indent=2, sort_keys=True) + '\n'
        if args.output_json:
            args.output_json.write_text(rendered, encoding='utf-8')
        print(rendered if args.json else _summary(report))
    except (
        OSError,
        PublishedReleaseError,
        json.JSONDecodeError,
        jsonschema.SchemaError,
        jsonschema.ValidationError,
    ) as exc:
        print(f'published release audit error: {exc}', file=sys.stderr)
        return 2
    if args.require_published and report['status'] != 'PUBLISHED':
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
