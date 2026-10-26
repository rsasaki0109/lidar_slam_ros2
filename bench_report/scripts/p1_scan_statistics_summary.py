"""Chapter 30: per-sequence medians of RKO-LIO scan_statistics.csv.

Usage: p1_scan_statistics_summary.py <label=csv> ...
"""
import csv
import sys

import numpy as np

cols = ['keypoints', 'nn_p25', 'nn_p50', 'nn_p75', 'nn_p90', 'within_0_25', 'within_0_5', 'matched_normals',
        'normal_eigenvalue_min', 'normal_eigenvalue_mid', 'normal_eigenvalue_max', 'motion_axis_information_ratio']
print('sequence,scans,' + ','.join(cols) + ',axis_ratio_p10,axis_lt_0.1')
for arg in sys.argv[1:]:
    label, path = arg.split('=', 1)
    rows = list(csv.DictReader(open(path)))
    data = {c: np.array([float(r[c]) for r in rows]) for c in cols}
    out = []
    for c in cols:
        v = data[c][np.isfinite(data[c])]
        out.append(f'{np.median(v):.4g}' if v.size else 'nan')
    a = data['motion_axis_information_ratio']
    a = a[np.isfinite(a)]
    out.append(f'{np.percentile(a, 10):.4g}' if a.size else 'nan')
    out.append(f'{np.mean(a < 0.1):.3f}' if a.size else 'nan')
    print(f'{label},{len(rows)},' + ','.join(out))
