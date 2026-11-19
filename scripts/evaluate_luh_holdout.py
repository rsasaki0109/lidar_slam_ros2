#!/usr/bin/env python3
"""Apply the preregistered LUH holdout decision to the scored update reports."""

import argparse
import json
import math
from pathlib import Path

CHAINS = (('S0', 'S4', 'S7'), ('S7', 'S4', 'S0'), ('S2', 'S5'), ('S5', 'S2'))
CLEAR = {'S5', 'S7'}


def numeric(report, key):
    value = report.get(key)
    return value if type(value) in (int, float) and math.isfinite(value) else None


def judge(chain, index, map_report, motion):
    previous, update = chain[index - 1:index + 1]
    tests = {measure: bool((motion.get(measure) or {}).get('passed') is True)
             for measure in ('R', 'L', 'T')}
    live = map_report.get('live') or {}
    static = numeric(live, 'S')
    tests['S'] = static is not None and static >= .97
    notes = []
    baseline = map_report.get('baseline') or {}
    if baseline.get('status') == 'refused':
        notes.append('merge --place-recognition refused the comparison')
    if update in CLEAR:
        ghosts, baseline_ghosts = numeric(live, 'G'), numeric(baseline, 'G')
        tests['G'] = (ghosts is not None and baseline_ghosts is not None and
                      ghosts <= .02 and ghosts < baseline_ghosts)
    if previous in CLEAR and update not in CLEAR:
        new = numeric(live, 'N')
        tests['N'] = new is not None and new >= .90
    return {'passed': all(tests.values()), 'checks': tests, 'notes': notes}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest', type=Path,
                        help='JSON mapping CHAIN/UPDATE to relative map and motion report paths')
    args = parser.parse_args()
    reports = json.loads(args.manifest.read_text())
    expected = {f'{"-".join(chain)}/{chain[i]}': (chain, i) for chain in CHAINS
                for i in range(1, len(chain))}
    if reports.keys() != expected.keys():
        parser.error(f'expected these holdout updates: {", ".join(expected)}')
    checks = {}
    for key, (chain, i) in expected.items():
        receipt = reports[key]
        map_report = json.loads((args.manifest.parent / receipt['map']).read_text())
        motion = json.loads((args.manifest.parent / receipt['motion']).read_text())
        checks[key] = judge(chain, i, map_report, motion)
    passed = all(result['passed'] for result in checks.values())
    print(json.dumps({'decision': 'product' if passed else 'opt-in',
                      'updates': checks}, indent=2))


if __name__ == '__main__':
    main()
