#!/usr/bin/env python3
"""Seal the blind run's outputs (bench_report/data/blind_v7_2026-11-08/preregistration.txt): one sha256 line per file.

usage: p5_blind_seal.py RUNS_DIR OUT_MANIFEST

Lists every trajectory, run record and input summary under RUNS_DIR (/home/user/bench/blind_v7/runs), sorted by path:
traj.tum, traj_imu.tum, run_info.txt, time.txt, exit.txt of each run, and bag_summary.json, convert_summary.txt,
extrinsics.txt, rig.yaml, g_params.yaml of each sequence. No ground-truth file exists when this runs.
"""
import hashlib
import os
import sys

RUN_FILES = ('traj.tum', 'traj_imu.tum', 'run_info.txt', 'time.txt', 'exit.txt')
SEQ_FILES = ('bag_summary.json', 'convert_summary.txt', 'extrinsics.txt', 'rig.yaml', 'g_params.yaml')


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    root = sys.argv[1]
    lines = []
    for seq in sorted(os.listdir(root)):
        sdir = os.path.join(root, seq)
        if not os.path.isdir(sdir):
            continue
        for name in SEQ_FILES:
            p = os.path.join(sdir, name)
            if os.path.isfile(p):
                lines.append(f'{sha256(p)}  {seq}/{name}')
        for run in sorted(os.listdir(sdir)):
            rdir = os.path.join(sdir, run)
            if not os.path.isdir(rdir):
                continue
            for name in RUN_FILES:
                p = os.path.join(rdir, name)
                if os.path.isfile(p):
                    lines.append(f'{sha256(p)}  {seq}/{run}/{name}')
    with open(sys.argv[2], 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print(f'{len(lines)} files sealed -> {sys.argv[2]}')


if __name__ == '__main__':
    main()
