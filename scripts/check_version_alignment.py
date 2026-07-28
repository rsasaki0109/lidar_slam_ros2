#!/usr/bin/env python3
"""Validate the release version contract from the repository VERSION file."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import re
import subprocess
import sys
from typing import Sequence
import xml.etree.ElementTree as ET


REPO_ROOT = Path(__file__).resolve().parents[1]
VERSION_PATTERN = re.compile(r'[0-9]+\.[0-9]+\.[0-9]+')
DATE_PATTERN = r'[0-9]{4}-[0-9]{2}-[0-9]{2}'
CORE_PACKAGES = (
    'lidarslam',
    'graph_based_slam',
    'lidarslam_msgs',
    'scanmatcher',
)


@dataclass(frozen=True)
class VersionValidation:
    """Machine-readable result of a repository version validation."""

    version: str
    release_date: str
    checked_surfaces: tuple[str, ...]
    errors: tuple[str, ...]

    @property
    def ok(self) -> bool:
        """Return whether every version surface matched."""
        return not self.errors

    def as_dict(self) -> dict[str, object]:
        """Return the stable JSON representation."""
        return {
            'schema_version': 1,
            'status': 'PASS' if self.ok else 'FAIL',
            'version': self.version,
            'release_date': self.release_date,
            'checked_surfaces': list(self.checked_surfaces),
            'errors': list(self.errors),
        }


def _read(path: Path, errors: list[str]) -> str:
    try:
        return path.read_text(encoding='utf-8')
    except OSError as exc:
        errors.append(f'{path}: cannot read: {exc}')
        return ''


def _expect_contains(
    path: Path,
    expected: str,
    errors: list[str],
    checked: list[str],
) -> None:
    text = _read(path, errors)
    checked.append(str(path))
    if text and expected not in text:
        errors.append(f'{path}: expected {expected!r}')


def _root_version(root: Path, errors: list[str], checked: list[str]) -> str:
    path = root / 'VERSION'
    raw = _read(path, errors)
    checked.append(str(path))
    if not raw:
        return ''
    if raw not in (raw.strip(), f'{raw.strip()}\n'):
        errors.append(f'{path}: must contain one version line with an optional final newline')
    version = raw.strip()
    if not VERSION_PATTERN.fullmatch(version):
        errors.append(
            f'{path}: {version!r} is not a ROS-compatible MAJOR.MINOR.PATCH version'
        )
    return version


def _changelog_release(
    root: Path,
    version: str,
    errors: list[str],
    checked: list[str],
) -> str:
    path = root / 'CHANGELOG.md'
    text = _read(path, errors)
    checked.append(str(path))
    headings = re.findall(
        rf'^## ([0-9]+\.[0-9]+\.[0-9]+) - ({DATE_PATTERN})$',
        text,
        flags=re.MULTILINE,
    )
    if not headings:
        errors.append(f'{path}: no versioned release heading found')
        return ''
    first_version, release_date = headings[0]
    if first_version != version:
        errors.append(
            f'{path}: newest release {first_version!r} does not match VERSION {version!r}'
        )
    return release_date


def _citation(
    root: Path,
    version: str,
    release_date: str,
    errors: list[str],
    checked: list[str],
) -> None:
    path = root / 'CITATION.cff'
    text = _read(path, errors)
    checked.append(str(path))
    versions = re.findall(r'^version:\s*([^\s#]+)\s*$', text, flags=re.MULTILINE)
    dates = re.findall(r'^date-released:\s*([^\s#]+)\s*$', text, flags=re.MULTILINE)
    if versions != [version]:
        errors.append(f'{path}: version must occur once and equal {version!r}')
    if dates != [release_date]:
        errors.append(
            f'{path}: date-released must occur once and equal {release_date!r}'
        )


def _packages(
    root: Path,
    version: str,
    release_date: str,
    errors: list[str],
    checked: list[str],
) -> None:
    for package in CORE_PACKAGES:
        package_xml = root / package / 'package.xml'
        checked.append(str(package_xml))
        try:
            package_root = ET.parse(package_xml).getroot()
        except (OSError, ET.ParseError) as exc:
            errors.append(f'{package_xml}: cannot parse: {exc}')
            continue
        package_version = package_root.findtext('version')
        if package_version != version:
            errors.append(
                f'{package_xml}: version {package_version!r} != VERSION {version!r}'
            )

        changelog = root / package / 'CHANGELOG.rst'
        _expect_contains(
            changelog,
            f'{version} ({release_date})',
            errors,
            checked,
        )


def _documentation(
    root: Path,
    version: str,
    errors: list[str],
    checked: list[str],
) -> None:
    release_rel = f'docs/releases/v{version}.md'
    release_notes = root / release_rel
    release_text = _read(release_notes, errors)
    checked.append(str(release_notes))
    first_line = next((line for line in release_text.splitlines() if line.strip()), '')
    if release_text and (not first_line.startswith('# ') or f'v{version}' not in first_line):
        errors.append(f'{release_notes}: first heading must identify v{version}')

    expected_markers = (
        (root / 'README.md', f'[v{version}]({release_rel})'),
        (
            root / 'CONTRIBUTING.md',
            f'[docs/releases/v{version}.md]({release_rel})',
        ),
        (root / 'docs' / 'index.md', f'[v{version}](releases/v{version}.md)'),
        (root / 'mkdocs.yml', f'- v{version}: releases/v{version}.md'),
        (
            root / 'docs' / 'comparison.md',
            f'`v{version}` is the current tagged prerelease.',
        ),
    )
    for path, marker in expected_markers:
        _expect_contains(path, marker, errors, checked)


def _tag_commit(root: Path, tag: str, version: str, errors: list[str]) -> None:
    expected = f'v{version}'
    if tag != expected:
        errors.append(f'release tag {tag!r} does not match {expected!r}')
        return
    try:
        tag_commit = subprocess.check_output(
            ['git', 'rev-parse', '--verify', f'refs/tags/{tag}^{{commit}}'],
            cwd=root,
            text=True,
            stderr=subprocess.STDOUT,
        ).strip()
        head_commit = subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'],
            cwd=root,
            text=True,
            stderr=subprocess.STDOUT,
        ).strip()
    except (OSError, subprocess.CalledProcessError) as exc:
        output = getattr(exc, 'output', '')
        detail = output.strip() if output else str(exc)
        errors.append(f'cannot resolve release tag {tag!r}: {detail}')
        return
    if tag_commit != head_commit:
        errors.append(
            f'release tag {tag!r} resolves to {tag_commit}, not checkout {head_commit}'
        )


def validate_repository(root: Path, tag: str | None = None) -> VersionValidation:
    """Validate every authoritative version surface below *root*."""
    root = root.resolve()
    errors: list[str] = []
    checked: list[str] = []
    version = _root_version(root, errors, checked)
    release_date = _changelog_release(root, version, errors, checked)
    _citation(root, version, release_date, errors, checked)
    _packages(root, version, release_date, errors, checked)
    _documentation(root, version, errors, checked)
    if tag is not None:
        _tag_commit(root, tag, version, errors)
        checked.append(f'git:refs/tags/{tag}')
    normalized_surfaces = tuple(
        (
            str(Path(surface).relative_to(root))
            if Path(surface).is_absolute() and Path(surface).is_relative_to(root)
            else surface
        )
        for surface in checked
    )
    return VersionValidation(
        version=version,
        release_date=release_date,
        checked_surfaces=normalized_surfaces,
        errors=tuple(errors),
    )


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Validate release metadata against the root VERSION file.',
    )
    parser.add_argument(
        '--root',
        type=Path,
        default=REPO_ROOT,
        help='Repository root (default: inferred from this script).',
    )
    parser.add_argument(
        '--tag',
        help='Also require this exact v<VERSION> tag to resolve to the checkout.',
    )
    parser.add_argument(
        '--json',
        action='store_true',
        help='Write the version validation result as JSON.',
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    result = validate_repository(args.root, tag=args.tag)
    if args.json:
        print(json.dumps(result.as_dict(), indent=2, sort_keys=True))
    elif result.ok:
        print(
            f'PASS: version {result.version} ({result.release_date}) aligns across '
            f'{len(result.checked_surfaces)} surfaces'
        )
    else:
        print(
            f'FAIL: version alignment has {len(result.errors)} error(s):',
            file=sys.stderr,
        )
        for error in result.errors:
            print(f'  - {error}', file=sys.stderr)
    return 0 if result.ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
