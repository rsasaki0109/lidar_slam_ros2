#!/usr/bin/env python3
"""Top-down image of p6_map_change.py's result: the old map (grey) and the points it removed (red).

usage: p6_map_change_plot.py RESULT_DIR OUT.png
"""
import json
import sys

import matplotlib
import numpy as np

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    d = sys.argv[1]
    pts = np.load(f'{d}/old_points.npy')
    stale = np.load(f'{d}/stale.npy')
    s = json.load(open(f'{d}/summary.json'))
    keep = pts[~stale]
    fig, ax = plt.subplots(figsize=(10, 10), dpi=120)
    sub = keep[np.random.default_rng(0).choice(len(keep), min(len(keep), 400000), replace=False)]
    ax.scatter(sub[:, 0], sub[:, 1], s=0.05, c='0.6', linewidths=0)
    ax.scatter(pts[stale, 0], pts[stale, 1], s=0.6, c='#d62728', linewidths=0)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title(f"removed {s['removed']} of {s['old_points']} old points ({100 * s['removed_fraction']:.2f} %)")
    fig.tight_layout()
    fig.savefig(sys.argv[2])


if __name__ == '__main__':
    main()
