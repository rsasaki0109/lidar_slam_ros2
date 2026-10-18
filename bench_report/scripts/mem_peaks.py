#!/usr/bin/env python3
"""When the backend's RSS peaks, from mem_sampler.py timelines.

For each run prefix: the end of the frontend (its last sample), the backend's sampled peak and its time, the
backend's largest RSS while the frontend still runs, after it ends, and at the last sample (all MB = KiB / 1024).
Usage: mem_peaks.py <run_prefix>...  (reads <run_prefix>.mem_timeline.csv)
"""
import csv
import sys

print('run,frontend_end_s,backend_peak_mb,backend_peak_t_s,backend_max_while_frontend_runs_mb,'
      'backend_max_after_frontend_mb,backend_last_mb')
for run in sys.argv[1:]:
    rows = list(csv.DictReader(open(run + '.mem_timeline.csv')))
    be = [(float(r['t_s']), int(r['rss_kb'])) for r in rows if r['name'].startswith('graph_based')]
    fe = [float(r['t_s']) for r in rows if r['name'].startswith('offline_node')]
    fe_end = max(fe) if fe else 0.0
    peak = max(be, key=lambda x: x[1])
    during = max((v for t, v in be if t <= fe_end), default=0)
    after = max((v for t, v in be if t > fe_end), default=0)
    print(f'{run},{fe_end:.1f},{peak[1] / 1024:.0f},{peak[0]:.1f},{during / 1024:.0f},{after / 1024:.0f},'
          f'{be[-1][1] / 1024:.0f}')
