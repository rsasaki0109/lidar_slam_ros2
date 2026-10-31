#!/usr/bin/env python3
"""
Wall time of a graph_slam_multisession_join run from its log: the first INFO line (a session read) to the last one.
Reading the bags before the first line is left out. Usage: p3_join_seconds.py <log> [<log> ...]; prints log, seconds.
"""
import re
import sys

for path in sys.argv[1:]:
    stamps = [float(m.group(1)) for m in re.finditer(r'^\[INFO\] \[(\d+\.\d+)\]', open(path).read(), re.M)]
    print(f'{path},{stamps[-1] - stamps[0]:.0f}' if len(stamps) > 1 else f'{path},')
