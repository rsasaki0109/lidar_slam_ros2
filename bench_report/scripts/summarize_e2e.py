"""Summarize pipe_e2e.sh CSVs: per-run table, per-label medians and the ratio of two labels.

Columns: label, offline_node_took_s, front_end_s, backend_idle_s, drain_s, map_save_s, submaps,
candidate_lines, loop_edges, front_md5, traj_md5, g2o_md5, loops_md5, backend_cpu_s, front_cpu_s
"""
import csv
import statistics
import sys

BAG_S = 125.814
for path in sys.argv[1:]:
    rows = [r for r in csv.reader(open(path)) if r]
    labels = []
    for r in rows:
        if r[0] not in labels:
            labels.append(r[0])
    print(f"### {path.split('/')[-1]}")
    print("| # | label | frontend s | frontend end s | backend idle s | after frontend s | map_save s | submaps "
          "| candidate lines | loop edges | backend CPU s | frontend CPU s | front md5 | traj md5 | g2o md5 |")
    print("|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|---|")
    for i, r in enumerate(rows, 1):
        print(f"| {i} | {r[0]} | {r[1]} | {r[2]} | {r[3]} | {r[4]} | {r[5]} | {r[6]} | {r[7]} | {r[8]} | {r[13]} | "
              f"{r[14]} | {r[9][:8]} | {r[10][:8]} | {r[11][:8]} |")
    med = {}
    for lab in labels:
        sel = [r for r in rows if r[0] == lab]
        # End to end: both processes are done (the backend may go idle before the frontend exits).
        idle = [max(float(r[2]), float(r[3])) for r in sel if r[3] != "NA" and r[2] != "NA"]
        front = [float(r[2]) for r in sel if r[2] != "NA"]
        cpu = [float(r[13]) for r in sel]
        med[lab] = statistics.median(idle) if idle else float("nan")
        md5s = {(r[9], r[10], r[11], r[12]) for r in sel}
        print(f"- {lab}: n={len(sel)}, median end-to-end {med[lab]:.1f} s (RTF {med[lab] / BAG_S:.3f}), "
              f"median frontend end {statistics.median(front):.1f} s, median backend CPU {statistics.median(cpu):.0f} s, "
              f"distinct output md5 sets: {len(md5s)}")
    if len(labels) == 2:
        a, b = labels
        print(f"- ratio {b}/{a} (median end-to-end) = {med[b] / med[a]:.3f}")
    print()
