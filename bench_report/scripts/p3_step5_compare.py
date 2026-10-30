#!/usr/bin/env python3
"""
Multi-session step 5 (data/grandtour/multisession_2026-10-29/plan.txt): one join of step 5 against the same join of
step 4. Prints one CSV line: name, decision in step 4, decision in step 5, same decision, and for joins in both the
difference of the two transforms (RMS over B's keyframes, mm; angle, deg).
Usage: p3_step5_compare.py <name> <step 4 output dir> <step 5 output dir> <runner dir of B>
  A pair's B is session 1; a split session's B is its keyframes after the middle plus half the gap.
"""
import sys

import numpy as np
import yaml

sys.path.insert(0, '/home/user/lidar_slam_ros2/bench_report/scripts')
from p3_multisession_join import rot_deg  # noqa: E402


def read(path):
    r = yaml.safe_load(open(f'{path}/join_report.yaml'))
    if 'split' in r:
        s, key = r['split'], 'transform'
    else:
        s, key = next(x for x in r['sessions'] if x['session'] == 1), 'transform_to_session_0'
    if s.get('ambiguous'):
        decision = 'ambiguous'
    elif s.get('self_similar'):
        decision = f"self-similar {s['self_similar']}"
    else:
        decision = 'joined' if s['joined'] else 'not joined'
    t = None
    if s['joined']:
        t = np.eye(4)
        t[:3] = np.array(s[key]).reshape(3, 4)
    return decision, t, r.get('split')


def main():
    name, d4, d5, runner = sys.argv[1:5]
    dec4, t4, split = read(d4)
    dec5, t5, _ = read(d5)
    raw = np.loadtxt(f'{runner}/trajectory_raw.tum')
    pos = raw[:, 1:4]
    if split is not None:
        middle = 0.5 * (raw[0, 0] + raw[-1, 0])
        pos = pos[raw[:, 0] > middle + 0.5 * split['gap_s']]
    diff = ''
    if t4 is not None and t5 is not None:
        d = (pos @ t4[:3, :3].T + t4[:3, 3]) - (pos @ t5[:3, :3].T + t5[:3, 3])
        rms = 1000.0 * np.sqrt((np.linalg.norm(d, axis=1) ** 2).mean())
        diff = f'{rms:.3f},{rot_deg(t4[:3, :3].T @ t5[:3, :3]):.4f}'
    print(f'{name},{dec4},{dec5},{dec4 == dec5},{diff}')


if __name__ == '__main__':
    main()
