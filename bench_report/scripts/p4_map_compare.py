#!/usr/bin/env python3
"""
Phase 4 step 1 (data/memory/p4_2026-10-30/plan.txt): a run's saved map against a baseline run's.

map.pcd: the number of points, and the largest difference of point i's x, y, z (m) and intensity over every i; the
pointcloud_map tiles: the same file names, and per file the same number of points (and the largest point
difference). Reads binary PCD files with fields x y z intensity (float32).
Usage: p4_map_compare.py <baseline run dir> <run dir>; prints one line.
"""
import os
import sys

import numpy as np


def lzf_decompress(data, size):
    """LZF (liblzf), as PCL's binary_compressed PCD files use it."""
    out = bytearray(size)
    ip = op = 0
    while ip < len(data):
        ctrl = data[ip]
        ip += 1
        if ctrl < 32:  # a run of ctrl + 1 literal bytes
            n = ctrl + 1
            out[op:op + n] = data[ip:ip + n]
            ip += n
            op += n
            continue
        length = ctrl >> 5  # a back reference
        ref = op - ((ctrl & 0x1f) << 8) - 1
        if length == 7:
            length += data[ip]
            ip += 1
        ref -= data[ip]
        ip += 1
        length += 2
        if ref + length <= op:
            out[op:op + length] = out[ref:ref + length]
            op += length
        else:  # overlapping: byte by byte
            for _ in range(length):
                out[op] = out[ref]
                op += 1
                ref += 1
    assert op == size
    return bytes(out)


def read_pcd(path):
    """The points of a binary or binary_compressed x y z intensity PCD file, as an (n, 4) float32 array."""
    with open(path, 'rb') as f:
        header = {}
        while True:
            line = f.readline().decode().strip()
            key, _, value = line.partition(' ')
            header[key] = value
            if key == 'DATA':
                break
        assert header['FIELDS'] == 'x y z intensity', path
        n = int(header['POINTS'])
        if header['DATA'] == 'binary':
            return np.frombuffer(f.read(16 * n), dtype=np.float32).reshape(n, 4)
        assert header['DATA'] == 'binary_compressed', path
        compressed, size = np.frombuffer(f.read(8), dtype=np.uint32)
        raw = lzf_decompress(f.read(int(compressed)), int(size)) if size else b''
        return np.frombuffer(raw, dtype=np.float32).reshape(4, n).T.copy()  # stored field by field


def diff(a, b):
    """Largest |a - b| over x, y, z and over intensity, or None when the counts differ."""
    if a.shape != b.shape:
        return None
    d = np.abs(a.astype(np.float64) - b.astype(np.float64))
    return (float(d[:, :3].max()) if len(d) else 0.0, float(d[:, 3].max()) if len(d) else 0.0)


def main():
    base, run = sys.argv[1:3]
    a, b = read_pcd(f'{base}/map.pcd'), read_pcd(f'{run}/map.pcd')
    out = [f'map.pcd {len(a)} / {len(b)} points']
    d = diff(a, b)
    out.append('count differs' if d is None else f'largest difference xyz {d[0]:.2e} m, intensity {d[1]:.2e}')
    ta, tb = sorted(os.listdir(f'{base}/pointcloud_map')), sorted(os.listdir(f'{run}/pointcloud_map'))
    if ta != tb:
        out.append(f'tiles: different files ({len(ta)} / {len(tb)})')
    else:
        worst, counts_same = (0.0, 0.0), True
        for name in ta:
            if not name.endswith('.pcd'):
                continue
            d = diff(read_pcd(f'{base}/pointcloud_map/{name}'), read_pcd(f'{run}/pointcloud_map/{name}'))
            if d is None:
                counts_same = False
            else:
                worst = (max(worst[0], d[0]), max(worst[1], d[1]))
        out.append(f'tiles: {len(ta)} files, same names, point counts {"the same" if counts_same else "DIFFER"}, '
                   f'largest difference xyz {worst[0]:.2e} m, intensity {worst[1]:.2e}')
    print('; '.join(out))


if __name__ == '__main__':
    main()
