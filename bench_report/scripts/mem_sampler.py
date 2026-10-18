#!/usr/bin/env python3
"""Sample the RSS of the pipeline's processes every 0.2 s until <stop_file> exists.

Every process whose command line matches one of the patterns (the ros2 launch and the nodes it starts) is read
from /proc/<pid>/status. Writes <out>_timeline.csv (t_s, pid, name, rss_kb) and <out>_summary.csv with each
process's VmHWM (its own peak RSS) and the repository's metric: the sum of the per-process peaks
(aggregate_process_tree_peak_rss_bytes in scripts/competitive_memory_gate.py), plus the peak of the sum at one
instant. Usage: mem_sampler.py <out_prefix> <stop_file>
"""
import os
import sys
import time

PATTERNS = ('offline_node', 'graph_based_slam_node', 'ros2 launch', 'static_transform_publisher',
            'glim_rosbag', 'ros2 run', 'robot_state_publisher', '_ros2_daemon')
out, stop = sys.argv[1:3]


def status(pid):
    values = {}
    try:
        with open(f'/proc/{pid}/status') as f:
            for line in f:
                key, _, rest = line.partition(':')
                if key in ('Name', 'VmRSS', 'VmHWM'):
                    values[key] = rest.split()[0]
    except OSError:
        return None
    return values


def matching():
    found = []
    for name in os.listdir('/proc'):
        if not name.isdigit():
            continue
        try:
            cmd = open(f'/proc/{name}/cmdline', 'rb').read().replace(b'\0', b' ').decode(errors='replace')
        except OSError:
            continue
        if any(p in cmd for p in PATTERNS) and 'mem_sampler' not in cmd:
            found.append((int(name), cmd))
    return found


t0 = time.monotonic()
hwm, names, cmds = {}, {}, {}
peak_sum, peak_sum_t = 0, 0.0
with open(out + '_timeline.csv', 'w') as tl:
    tl.write('t_s,pid,name,rss_kb\n')
    while not os.path.exists(stop):
        t = time.monotonic() - t0
        total = 0
        for pid, cmd in matching():
            s = status(pid)
            if not s or 'VmRSS' not in s:
                continue
            rss = int(s['VmRSS'])
            names[pid], cmds[pid] = s.get('Name', '?'), cmd[:160]
            hwm[pid] = max(hwm.get(pid, 0), int(s.get('VmHWM', rss)), rss)
            total += rss
            tl.write(f'{t:.1f},{pid},{names[pid]},{rss}\n')
        if total > peak_sum:
            peak_sum, peak_sum_t = total, t
        time.sleep(0.2)
with open(out + '_summary.csv', 'w') as f:
    f.write('pid,name,peak_rss_kb,cmd\n')
    for pid in sorted(hwm, key=lambda p: -hwm[p]):
        f.write(f'{pid},{names[pid]},{hwm[pid]},"{cmds[pid]}"\n')
    f.write(f'sum_of_process_peaks,,{sum(hwm.values())},\n')
    f.write(f'peak_of_sum,,{peak_sum},at {peak_sum_t:.1f} s\n')
