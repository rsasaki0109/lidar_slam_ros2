#!/usr/bin/env python3
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
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Prepare two fail-closed usability scorecard worksheets as one pair.

Common cohort, input, and environment metadata is entered once. The command
creates one worksheet per product with opposite product order, refuses to
overwrite either destination, and performs no network or remote mutation.
The generated files are recording worksheets, not evidence of a completed
trial.
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from prepare_usability_scorecard import (  # noqa: E402
    ScorecardError,
    build_template,
)


PRODUCTS = (
    ('lidarslam', 'lidarslam_ros2'),
    ('glim', 'glim'),
)


def _now() -> str:
    return (
        datetime.now(timezone.utc)
        .replace(microsecond=0)
        .isoformat()
        .replace('+00:00', 'Z')
    )


def _add_product_arguments(
    parser: argparse.ArgumentParser,
    prefix: str,
    product: str,
) -> None:
    """Add the identity and documentation options for one product."""
    title = product.replace('_', ' ')
    parser.add_argument(
        f'--{prefix}-version',
        required=True,
        help=f'{title} product version',
    )
    parser.add_argument(
        f'--{prefix}-revision-kind',
        choices=('git-commit', 'release-tag', 'image-digest'),
        required=True,
        help=f'{title} revision identity kind',
    )
    parser.add_argument(
        f'--{prefix}-revision',
        required=True,
        help=f'{title} exact revision value',
    )
    parser.add_argument(
        f'--{prefix}-documentation-url',
        required=True,
        help=f'{title} public documentation root URL',
    )
    parser.add_argument(f'--{prefix}-trial-id')
    parser.add_argument(
        f'--{prefix}-publicly-resolvable',
        action='store_true',
        help=f'mark the {title} identity public after checking it',
    )
    parser.add_argument(
        f'--{prefix}-machine-fingerprint-sha256',
        help=(
            f'optional {title} host fingerprint; overrides the common '
            'fingerprint'
        ),
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    for prefix, product in PRODUCTS:
        _add_product_arguments(parser, prefix, product)

    parser.add_argument('--captured-at')
    parser.add_argument('--cohort-id', required=True)
    parser.add_argument('--comparison-pair-id', required=True)
    parser.add_argument('--input-id', required=True)
    parser.add_argument(
        '--lidarslam-order',
        choices=('first', 'second'),
        default='first',
        help='which product the operator attempts first (default: first)',
    )
    parser.add_argument(
        '--operator-class',
        choices=('maintainer', 'external'),
        default='external',
    )
    parser.add_argument('--not-first-attempt', action='store_true')
    parser.add_argument('--clean-start', action='store_true')
    parser.add_argument(
        '--ros-distro',
        choices=('humble', 'jazzy'),
        required=True,
    )
    parser.add_argument(
        '--os-family',
        choices=('ubuntu-22.04', 'ubuntu-24.04'),
        required=True,
    )
    parser.add_argument(
        '--architecture',
        choices=('x86_64', 'aarch64'),
        required=True,
    )
    parser.add_argument('--hardware-class', required=True)
    parser.add_argument(
        '--machine-fingerprint-sha256',
        help=(
            'common host fingerprint; provide per-product overrides when '
            'the paired runs use different hosts'
        ),
    )
    parser.add_argument('--output-dir', type=Path, required=True)
    return parser


def _fingerprint(args: argparse.Namespace, prefix: str) -> str:
    value = getattr(args, f'{prefix}_machine_fingerprint_sha256')
    value = value or args.machine_fingerprint_sha256
    if value is None:
        raise ScorecardError(
            'provide --machine-fingerprint-sha256 or both per-product '
            'fingerprint overrides'
        )
    return value


def _product_args(
    args: argparse.Namespace,
    prefix: str,
    product: str,
    product_order: str,
    captured_at: str,
) -> argparse.Namespace:
    """Translate pair options into the single-worksheet builder contract."""
    return argparse.Namespace(
        trial_id=(
            getattr(args, f'{prefix}_trial_id')
            or f'ux-pair-{product}-{datetime.now(timezone.utc):%Y%m%d}'
        ),
        captured_at=captured_at,
        product=product,
        version=getattr(args, f'{prefix}_version'),
        revision_kind=getattr(args, f'{prefix}_revision_kind'),
        revision=getattr(args, f'{prefix}_revision'),
        documentation_url=getattr(args, f'{prefix}_documentation_url'),
        publicly_resolvable=getattr(
            args, f'{prefix}_publicly_resolvable'),
        operator_class=args.operator_class,
        cohort_id=args.cohort_id,
        not_first_attempt=args.not_first_attempt,
        product_order=product_order,
        comparison_pair_id=args.comparison_pair_id,
        clean_start=args.clean_start,
        ros_distro=args.ros_distro,
        os_family=args.os_family,
        architecture=args.architecture,
        hardware_class=args.hardware_class,
        machine_fingerprint_sha256=_fingerprint(args, prefix),
        input_id=args.input_id,
    )


def _validate_pair(
    lidarslam: dict,
    glim: dict,
) -> None:
    """Check the pair invariants before either worksheet is written."""
    for field in (
        'comparison_pair_id',
        'ros_distro',
        'os_family',
        'architecture',
        'hardware_class',
    ):
        if lidarslam['environment'][field] != glim['environment'][field]:
            raise ScorecardError(f'paired environment {field} differs')
    for field in ('class', 'cohort_id', 'first_attempt'):
        if lidarslam['operator'][field] != glim['operator'][field]:
            raise ScorecardError(f'paired operator {field} differs')
    if (
        {
            lidarslam['operator']['product_order'],
            glim['operator']['product_order'],
        }
        != {'first', 'second'}
    ):
        raise ScorecardError(
            'paired product order must contain first and second')
    if lidarslam['trial_id'] == glim['trial_id']:
        raise ScorecardError('paired trial IDs must be different')
    for left_task, right_task in zip(lidarslam['tasks'], glim['tasks']):
        if left_task['input_id'] != right_task['input_id']:
            raise ScorecardError(
                f"paired input differs for {left_task['task_id']}")


def _write_pair(
    output_dir: Path,
    records: Sequence[dict],
) -> list[Path]:
    """Write both records exclusively after preflighting both destinations."""
    output_dir.mkdir(parents=True, exist_ok=True)
    paths = [output_dir / f"{record['trial_id']}.json" for record in records]
    existing = [path for path in paths if path.exists() or path.is_symlink()]
    if existing:
        raise ScorecardError(
            'refusing to overwrite existing worksheet: '
            + str(existing[0])
        )
    payloads = [json.dumps(record, indent=2, sort_keys=True) + '\n'
                for record in records]
    for path, payload in zip(paths, payloads):
        with path.open('x', encoding='utf-8') as stream:
            stream.write(payload)
    return paths


def main(argv: Sequence[str] | None = None) -> int:
    """Prepare one incomplete worksheet per product without overwriting."""
    args = _parser().parse_args(argv)
    try:
        captured_at = args.captured_at or _now()
        lidarslam_order = args.lidarslam_order
        glim_order = 'second' if lidarslam_order == 'first' else 'first'
        lidarslam = build_template(_product_args(
            args, 'lidarslam', 'lidarslam_ros2', lidarslam_order, captured_at))
        glim = build_template(_product_args(
            args, 'glim', 'glim', glim_order, captured_at))
        _validate_pair(lidarslam, glim)
        paths = _write_pair(args.output_dir, (lidarslam, glim))
        manifest = {
            'status': 'PREPARED_INCOMPLETE',
            'comparison_pair_id': args.comparison_pair_id,
            'remote_mutations_performed': False,
            'files': [
                {
                    'filename': path.name,
                    'product': record['product']['id'],
                    'product_order': record['operator']['product_order'],
                    'trial_id': record['trial_id'],
                }
                for path, record in zip(paths, (lidarslam, glim))
            ],
        }
        sys.stdout.write(json.dumps(manifest, indent=2, sort_keys=True) + '\n')
        print(
            'Worksheets are incomplete; do not add them to the reviewed '
            'evidence index until the observed pair is complete.',
            file=sys.stderr,
        )
    except (OSError, ScorecardError) as exc:
        print(f'usability scorecard pair error: {exc}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
