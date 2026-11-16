#!/usr/bin/env python3
"""Odyssey drive, read as a tar stream: write the reference poses and the LiDAR stamps, nothing else.

usage: p8_http_stream.py URL | p8_odyssey_fetch_reference.py OUT_DIR

Only after the H9 runs are sealed and the maintainer has approved
(bench_report/data/blind_v7_run2_2026-11-10/preregistration_h9.txt). Writes OUT_DIR/lidar_poses_ref.txt (the member
<seq>/refsys/lidar_poses_ref.txt, unchanged), OUT_DIR/lidar_stamps.txt (the sorted names of <seq>/ouster/<ns>.bin)
and OUT_DIR/fetch.json (bytes read, sha256 of the archive and of the member). Every other member is skipped.
Run with `python3 -I` from a directory other than the data's: the archive is untrusted input. Nothing is extracted.
"""
import hashlib
import json
import os
import re
import sys
import tarfile


class Hashed:
    def __init__(self, f):
        self.f, self.sha, self.count = f, hashlib.sha256(), 0

    def read(self, n=-1):
        b = self.f.read(n)
        self.sha.update(b)
        self.count += len(b)
        return b


def main():
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    src = Hashed(sys.stdin.buffer)
    tar = tarfile.open(fileobj=src, mode='r|*')
    stamps, ref = [], None
    for m in tar:
        parts = m.name.strip('./').split('/')
        if not m.isfile() or len(parts) != 3:
            continue
        if parts[1] == 'ouster' and re.fullmatch(r'\d+\.bin', parts[2]):
            stamps.append(int(parts[2][:-4]))
        elif parts[1] == 'refsys' and parts[2] == 'lidar_poses_ref.txt':
            ref = tar.extractfile(m).read()
    while src.read(1 << 22):
        pass
    if ref is None:
        sys.exit('no refsys/lidar_poses_ref.txt in the archive')
    open(os.path.join(out, 'lidar_poses_ref.txt'), 'wb').write(ref)
    open(os.path.join(out, 'lidar_stamps.txt'), 'w').write(''.join(f'{s}\n' for s in sorted(stamps)))
    info = {'bytes_read': src.count, 'sha256': src.sha.hexdigest(), 'lidar_stamps': len(stamps),
            'reference_sha256': hashlib.sha256(ref).hexdigest(), 'reference_lines': len(ref.splitlines())}
    open(os.path.join(out, 'fetch.json'), 'w').write(json.dumps(info, indent=1) + '\n')
    print(json.dumps(info))


if __name__ == '__main__':
    main()
