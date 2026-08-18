#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
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

"""Aggregate repeated held-out visual reports with conservative worst cases."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import statistics
import sys
from typing import Any


def sha256(path: Path) -> str:
    """Return the SHA-256 digest of a file."""
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def summarize(reports: list[Path], *, maps: list[Path] | None = None,
              trajectories: list[Path] | None = None,
              transforms: list[Path] | None = None,
              calibrations: list[Path] | None = None) -> dict[str, Any]:
    """Validate protocols and conservatively aggregate repeated reports."""
    if len(reports) != 3:
        raise ValueError('exactly three held-out visual reports are required')
    repeated_inputs = {
        'map': maps, 'trajectory': trajectories, 'transforms': transforms}
    for name, paths in repeated_inputs.items():
        if paths is not None and len(paths) != len(reports):
            raise ValueError(f'{name} and report counts differ')
    runs = []
    protocol = None
    for report_path in reports:
        body = json.loads(report_path.read_text())
        current_protocol = {
            key: body[key] for key in (
                'train_views', 'heldout_views', 'heldout_views_scored',
                'color_source', 'normalize_exposure', 'exposure_scale_limit')}
        if protocol is None:
            protocol = current_protocol
        elif current_protocol != protocol:
            raise ValueError('held-out visual report protocols differ')
        if int(body['scored_points']) <= 0:
            raise ValueError(f'{report_path}: no held-out points were scored')
        row = {
            'report_path': str(report_path.resolve()),
            'report_sha256': sha256(report_path),
            'visible_points': int(body['visible_points']),
            'scored_points': int(body['scored_points']),
            'heldout_scored_fraction': float(body['heldout_scored_fraction']),
            'rgb_l2_median': float(body['rgb_l2_median']),
            'rgb_l2_inlier_20': float(body['rgb_l2_inlier_20']),
        }
        for name, paths in repeated_inputs.items():
            if paths is not None:
                row[f'{name}_path'] = str(paths[len(runs)].resolve())
                row[f'{name}_sha256'] = sha256(paths[len(runs)])
        runs.append(row)
    medians = [run['rgb_l2_median'] for run in runs]
    inliers = [run['rgb_l2_inlier_20'] for run in runs]
    return {
        'schema_version': 1,
        'valid_repetitions': len(runs),
        'aggregation_valid': True,
        'protocol': protocol,
        'aggregation_policy': {
            'lower_is_better_metrics': 'maximum_worst_case',
            'higher_is_better_metrics': 'minimum_worst_case',
        },
        'calibrations': [
            {'path': str(path.resolve()), 'sha256': sha256(path)}
            for path in (calibrations or [])],
        'aggregate': {
            'heldout_rgb_l2_median': max(medians),
            'heldout_rgb_inlier_20': min(inliers),
            'heldout_rgb_l2_repetition_median': statistics.median(medians),
            'heldout_rgb_inlier_20_repetition_median': statistics.median(inliers),
        },
        'runs': runs,
    }


def main() -> int:
    """Run the CLI."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, action='append', required=True)
    parser.add_argument('--map', type=Path, action='append')
    parser.add_argument('--trajectory', type=Path, action='append')
    parser.add_argument('--transforms', type=Path, action='append')
    parser.add_argument('--calibration', type=Path, action='append')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    document = summarize(
        args.report, maps=args.map, trajectories=args.trajectory,
        transforms=args.transforms, calibrations=args.calibration)
    if args.output.exists():
        raise ValueError(f'refusing to overwrite: {args.output}')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(document, indent=2, sort_keys=True) + '\n')
    print(json.dumps(document['aggregate'], indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, TypeError,
            json.JSONDecodeError) as error:
        print(f'error: {error}', file=sys.stderr)
        sys.exit(2)
