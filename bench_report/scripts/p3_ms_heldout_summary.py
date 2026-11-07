#!/usr/bin/env python3
"""Tables of plan multisession_heldout_2026-11-07 (run with python3 -I).

p3_ms_heldout_summary.py RUN_ROOT OUT_DIR

RUN_ROOT holds checks/<pair>/{debug_checks.yaml,perturb.csv} (part B) and runs/<name>/join_report.yaml (part A,
split sessions). Writes OUT_DIR/checks.csv and OUT_DIR/results.csv.
"""
import csv
import os
import sys

import yaml


def checks(root, out):
    rows = []
    base = os.path.join(root, 'checks')
    for pair in sorted(os.listdir(base)):
        path = os.path.join(base, pair, 'debug_checks.yaml')
        if not os.path.isfile(path):
            continue
        d = yaml.safe_load(open(path))['debug_checks']
        labels = {int(r['index']): r for r in csv.DictReader(open(os.path.join(base, pair, 'perturb.csv')))}
        c = d.get('consistency', {})
        seen = set()
        for k in d['checks']:  # per transform: the bar place_min_fraction first, then candidate 2's
            lab = labels[k['index']]
            rule = 'candidate_2' if k['index'] in seen else 'as_is'
            seen.add(k['index'])
            rows.append([pair, k['index'], lab['kind'], lab['axis'], lab['amount'], rule, c.get('a', ''),
                         c.get('b', ''), f"{k['bar']:.4f}", k['places'], f"{k['share_below']:.4f}",
                         f"{k['median']:.3f}", 'pass' if k['passed'] else 'fail'])
    with open(os.path.join(out, 'checks.csv'), 'w', newline='') as f:  # LF, as the other tables
        w = csv.writer(f, lineterminator='\n')
        w.writerow(['pair', 'index', 'kind', 'axis', 'amount', 'rule', 'consistency_a', 'consistency_b', 'bar',
                    'places', 'share_below', 'median', 'passed'])
        w.writerows(rows)
    return rows


def results(root, out):
    rows = []
    base = os.path.join(root, 'runs')
    for name in sorted(os.listdir(base)):
        path = os.path.join(base, name, 'join_report.yaml')
        if not os.path.isfile(path):
            continue
        s = yaml.safe_load(open(path))['split']
        c = s.get('consistency', {})
        sets = ' '.join(f"{t['size']}/{t['places']}/{t['share_below']:.4f}/{t['median']:.3f}/"
                        f"{'pass' if t['passed'] else 'fail'}" for t in s.get('sets_tried') or [])
        rows.append([name, 'candidate_2' if name.startswith('c2_') else 'as_is',
                     'true' if s['excluded_by_debug_hook'] else 'false', str(s['joined']).lower(),
                     str(s['ambiguous']).lower(), s['self_similar'] or '-', s['candidates'], s['accepted'],
                     c.get('a', ''), c.get('b', ''), c.get('skip_neighbors', 0), f"{c.get('bar', 0.5):.4f}",
                     s.get('error_to_identity_rms_m', ''), sets])
    with open(os.path.join(out, 'results.csv'), 'w', newline='') as f:  # LF, as the other tables
        w = csv.writer(f, lineterminator='\n')
        w.writerow(['run', 'rule', 'true_matches_removed', 'joined', 'ambiguous', 'self_similar', 'candidates',
                    'accepted', 'consistency_a', 'consistency_b', 'skip_neighbors', 'bar', 'error_to_identity_rms_m',
                    'sets(size/places/share_below/median/passed)'])
        w.writerows(rows)
    return rows


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    for r in checks(sys.argv[1], sys.argv[2]):
        if r[-1] == 'pass':
            print('pass', *r[:6], 'bar', r[8], 'below', r[10])
    for r in results(sys.argv[1], sys.argv[2]):
        print(*r[:6], 'bar', r[11], 'err', r[12], '|', r[13])


if __name__ == '__main__':
    main()
