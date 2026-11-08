#!/usr/bin/env python3
"""Check 2 of plan multisession_gnss_2026-11-07/plan_build.txt: the join tool's GNSS gate against the offline one.

p3_ms_gnss_build_compare.py RUNS_DIR OFFLINE_DIR

RUNS_DIR holds the tool's runs (p3_ms_gnss_build_check.sh): sg12, sg21, con12, con21, sgsplit1, consplit1, consplit2
(join_report.yaml) and sgcheck12, sgcheck21, concheck12, concheck21 (debug_checks.yaml). OFFLINE_DIR holds the
offline evaluation (rot_*.json of chapter 56.3). Prints one line per set or transform and a summary; exit status 1 if a
decision differs or a median differs by more than 0.01 m or 0.01 deg.
"""
import json
import os
import sys

import yaml

TOL_M, TOL_DEG = 0.01, 0.01
CASES = [('sg12', 'rot_sg12'), ('sg21', 'rot_sg21'), ('con12', 'rot_cons12'), ('con21', 'rot_cons21'),
         ('sgsplit1', 'rot_split_sg1'), ('consplit1', 'rot_split_cons1'), ('consplit2', 'rot_split_cons2')]
CHECKS = [('sgcheck12', 'rot_sg12'), ('sgcheck21', 'rot_sg21'), ('concheck12', 'rot_cons12'),
          ('concheck21', 'rot_cons21')]


def compare(label, tool, offline, problems):
    """Compare one set or transform; tool is the report's gnss dict (or None), offline the evaluation's entry."""
    t_ok = bool(tool and tool['passed'])
    o_ok = bool(offline['gate'])
    dm = abs(tool['median_m'] - offline['median_m']) if tool and offline.get('median_m') is not None else 0.0
    dh = abs(tool['median_heading_deg'] - offline['median_heading_deg']) \
        if tool and offline.get('median_heading_deg') is not None else 0.0
    places_same = (tool['places'] if tool else -1) == offline['measured']
    bad = t_ok != o_ok or dm > TOL_M or dh > TOL_DEG
    if bad:
        problems.append(label)
    print(f"{label:28s} tool {'pass' if t_ok else 'fail'} offline {'pass' if o_ok else 'fail'}"
          f"  places {tool['places'] if tool else '-'}/{offline['measured']}{'' if places_same else ' (differ)'}"
          f"  d {dm:.4f} m  h {dh:.4f} deg{'  <-- DIFFERS' if bad else ''}")


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    runs, offline_dir = sys.argv[1], sys.argv[2]
    problems = []
    for run, off in CASES:
        report = yaml.safe_load(open(os.path.join(runs, run, 'join_report.yaml')))
        part = report['split'] if 'split' in report else next(s for s in report['sessions'] if s.get('session') == 1)
        offline = json.load(open(os.path.join(offline_dir, off + '.json')))
        sets = part.get('sets_tried') or []
        tool_join = next((i for i, s in enumerate(sets) if (s.get('gnss') or {}).get('passed')), None)
        decided = (part.get('gnss') or {}).get('decided')
        same = tool_join == offline['joins'] and bool(part['joined']) == (offline['joins'] is not None)
        if not same:
            problems.append(run + ' join')
        print(f"{run}: GNSS decided {decided}, joined {part['joined']} (set {tool_join}), offline joins set "
              f"{offline['joins']}{'' if same else '  <-- DIFFERS'}")
        for i, s in enumerate(offline['sets']):
            compare(f'  {run} set {i}', sets[i].get('gnss') if i < len(sets) else None, s, problems)
    for run, off in CHECKS:
        checks = yaml.safe_load(open(os.path.join(runs, run, 'debug_checks.yaml')))['debug_checks']['checks']
        offline = json.load(open(os.path.join(offline_dir, off + '.json')))
        by_index = {c['index']: c for c in checks}
        for c in offline['checks']:
            compare(f"  {run} {c['index']:2d} {c['kind'][:5]} {c['axis']} {c['amount']:g}",
                    by_index[c['index']].get('gnss'), c, problems)
    print(f'{len(problems)} differences' + (': ' + ', '.join(problems) if problems else ''))
    sys.exit(1 if problems else 0)


if __name__ == '__main__':
    main()
