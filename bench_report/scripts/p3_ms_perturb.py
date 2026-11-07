#!/usr/bin/env python3
"""Perturbed session transforms for debug_check_transforms (plan multisession_heldout_2026-11-07, part B).

p3_ms_perturb.py JOIN_REPORT B_TRAJECTORY_RAW OUT_PREFIX

Reads session 1's transform_to_session_0 T from JOIN_REPORT (a two-session join; a .json: the
debug_exclude_transform of a drift-only reference) and B's submap positions
(the keyframes) from B_TRAJECTORY_RAW (trajectory_raw.tum of B's backend run). Writes
OUT_PREFIX.values (the 12 x 24 values of debug_check_transforms, comma separated) and OUT_PREFIX.csv (one
label per transform, in the same order):
  0      T itself
  1-16   T moved in A's frame by 1, 2, 4, 8 m along +x, -x, +y, -y
  17-23  T turned about A's vertical axis through the centroid of B's keyframes placed by T, by
         +2, -2, +5, -5, +10, -10 and 180 deg
"""
import json
import math
import re
import sys


def read_transform(path):
    if path.endswith('.json'):  # a drift-only reference (p3_rtk_multisession_eval.py): the truth
        v = [float(x) for x in json.load(open(path))['debug_exclude_transform']]
        return [v[0:4], v[4:8], v[8:12], [0.0, 0.0, 0.0, 1.0]]
    with open(path) as f:
        text = f.read()
    found = re.findall(r'transform_to_session_0: \[([^\]]*)\]', text)
    if len(found) != 1:
        sys.exit(f'{path}: expected one transform_to_session_0, found {len(found)}')
    v = [float(x) for x in found[0].split(',')]
    return [v[0:4], v[4:8], v[8:12], [0.0, 0.0, 0.0, 1.0]]


def read_positions(path):
    positions = []
    with open(path) as f:
        for line in f:
            parts = line.split()
            if len(parts) == 8 and not line.startswith('#'):
                positions.append([float(parts[1]), float(parts[2]), float(parts[3])])
    if not positions:
        sys.exit(f'{path}: no poses')
    return positions


def mul(a, b):
    return [[sum(a[r][k] * b[k][c] for k in range(4)) for c in range(4)] for r in range(4)]


def apply(t, p):
    return [sum(t[r][k] * p[k] for k in range(3)) + t[r][3] for r in range(3)]


def translation(x, y, z):
    return [[1.0, 0.0, 0.0, x], [0.0, 1.0, 0.0, y], [0.0, 0.0, 1.0, z], [0.0, 0.0, 0.0, 1.0]]


def yaw_about(deg, c):
    a = math.radians(deg)
    r = [[math.cos(a), -math.sin(a), 0.0, 0.0], [math.sin(a), math.cos(a), 0.0, 0.0],
         [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
    return mul(translation(*c), mul(r, translation(-c[0], -c[1], -c[2])))


def main():
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    t = read_transform(sys.argv[1])
    placed = [apply(t, p) for p in read_positions(sys.argv[2])]
    centroid = [sum(p[i] for p in placed) / len(placed) for i in range(3)]
    rows = [('identity', 'none', 0.0, t)]
    for d in (1.0, 2.0, 4.0, 8.0):
        for axis, (x, y) in (('+x', (d, 0.0)), ('-x', (-d, 0.0)), ('+y', (0.0, d)), ('-y', (0.0, -d))):
            rows.append(('translation', axis, d, mul(translation(x, y, 0.0), t)))
    for deg in (2.0, -2.0, 5.0, -5.0, 10.0, -10.0, 180.0):
        rows.append(('rotation', 'z', deg, mul(yaw_about(deg, centroid), t)))
    with open(sys.argv[3] + '.values', 'w') as f:
        f.write(','.join(f'{m[r][c]:.9f}' for *_, m in rows for r in range(3) for c in range(4)) + '\n')
    with open(sys.argv[3] + '.csv', 'w') as f:
        f.write('index,kind,axis,amount,centroid_x,centroid_y,centroid_z\n')
        for i, (kind, axis, amount, _) in enumerate(rows):
            f.write(f'{i},{kind},{axis},{amount:g},{centroid[0]:.3f},{centroid[1]:.3f},{centroid[2]:.3f}\n')
    print(f'{len(rows)} transforms, centroid {centroid[0]:.2f} {centroid[1]:.2f} {centroid[2]:.2f}')


if __name__ == '__main__':
    main()
