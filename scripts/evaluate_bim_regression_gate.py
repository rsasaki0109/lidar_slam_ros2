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
#
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

"""Evaluate reproducible Scan-to-BIM regression profiles from QA manifests."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
TOOL_DIR = REPO_ROOT / 'tools' / 'gaussian_splatting'


def _lookup(data: dict, dotted_key: str) -> Any:
    value: Any = data
    for key in dotted_key.split('.'):
        if not isinstance(value, dict) or key not in value:
            raise KeyError(dotted_key)
        value = value[key]
    return value


def evaluate_case(metrics: dict, rules: dict[str, dict]) -> list[str]:
    """Return human-readable rule violations for one metrics manifest."""
    violations = []
    for key, limits in rules.items():
        try:
            value = _lookup(metrics, key)
        except KeyError:
            violations.append(f'{key}: metric is missing')
            continue
        if value is None:
            violations.append(f'{key}: metric is null')
            continue
        if 'exact' in limits and value != limits['exact']:
            violations.append(f'{key}: {value} != {limits["exact"]}')
        if 'min' in limits and value < limits['min']:
            violations.append(f'{key}: {value} < {limits["min"]}')
        if 'max' in limits and value > limits['max']:
            violations.append(f'{key}: {value} > {limits["max"]}')
    return violations


def evaluate_suite(profile: dict, cases: dict[str, dict]) -> dict:
    results = {}
    for name, definition in profile.get('cases', {}).items():
        metrics = cases.get(name)
        if metrics is None:
            violations = ['required case metrics are missing']
        else:
            violations = evaluate_case(metrics, definition.get('rules', {}))
        results[name] = {
            'passed': not violations,
            'description': definition.get('description', ''),
            'violations': violations,
        }
    return {
        'schema_version': 1,
        'profile': profile.get('name', ''),
        'passed': bool(results) and all(result['passed'] for result in results.values()),
        'cases': results,
    }


def _wall(x0: float, y0: float, x1: float, y1: float) -> dict:
    direction = np.array([x1 - x0, y1 - y0], dtype=np.float64)
    length = float(np.linalg.norm(direction))
    normal = np.array([-direction[1] / length, direction[0] / length, 0.0])
    corners = np.array([[x0, y0, 0.0], [x1, y1, 0.0],
                        [x1, y1, 3.0], [x0, y0, 3.0]])
    return {
        'kind': 'vertical', 'normal': normal,
        'd': float(-corners.mean(axis=0).dot(normal)),
        'corners': corners, 'centroid': corners.mean(axis=0),
        'size': (length, 3.0), 'thickness': 0.15,
        'indices': np.array([], dtype=int),
    }


def _builtin_closed_room() -> dict:
    if str(TOOL_DIR) not in sys.path:
        sys.path.insert(0, str(TOOL_DIR))
    import bim_export

    planes = [_wall(0, 0, 6, 0), _wall(6, 0, 6, 4),
              _wall(6, 4, 0, 4), _wall(0, 4, 0, 0)]
    return bim_export.build_bim_metrics(planes, source='builtin:closed-room')


def load_case(source: str) -> dict:
    if source == 'builtin:closed-room':
        return _builtin_closed_room()
    return json.loads(Path(source).expanduser().read_text(encoding='utf-8'))


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Gate BIM metrics JSON files against a versioned profile.')
    parser.add_argument('--profile', required=True, type=Path)
    parser.add_argument('--case', action='append', default=[], metavar='NAME=PATH',
                        help='case metrics JSON; PATH may be builtin:closed-room')
    parser.add_argument('--output', required=True, type=Path)
    return parser


def main(argv=None) -> int:
    args = _parser().parse_args(argv)
    profile = json.loads(args.profile.read_text(encoding='utf-8'))
    cases = {}
    for value in args.case:
        if '=' not in value:
            raise SystemExit(f'invalid --case {value!r}; expected NAME=PATH')
        name, source = value.split('=', 1)
        cases[name] = load_case(source)
    result = evaluate_suite(profile, cases)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2,
                                      sort_keys=True) + '\n', encoding='utf-8')
    for name, case in result['cases'].items():
        print(f'{"PASS" if case["passed"] else "FAIL"} {name}')
        for violation in case['violations']:
            print(f'  - {violation}')
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
