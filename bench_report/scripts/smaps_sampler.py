#!/usr/bin/env python3
"""A process's resident memory by kind of mapping, from /proc/<pid>/smaps, every PERIOD seconds.

Kinds: heap ([heap] and anonymous mappings: malloc's arenas and large blocks), stack, shm (/dev/shm: the DDS
shared-memory transport), lib (mapped files ending in .so*), file (other mapped files), other. Waits for a process
whose command line matches PATTERN, samples it until it exits, and writes one CSV row a sample (MiB).
Usage: smaps_sampler.py <pattern> <out.csv> [period_s]
"""
import os
import re
import sys
import time

pattern = re.compile(sys.argv[1])
out = sys.argv[2]
period = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0
KINDS = ['heap', 'stack', 'shm', 'lib', 'file', 'other']


def find():
    for pid in os.listdir('/proc'):
        if not pid.isdigit() or int(pid) == os.getpid():
            continue
        try:
            cmd = open(f'/proc/{pid}/cmdline', 'rb').read().replace(b'\0', b' ').decode(errors='replace')
        except OSError:
            continue
        if pattern.search(cmd) and 'smaps_sampler' not in cmd:
            return int(pid)
    return None


def kind(name):
    if name in ('', '[heap]') or name.startswith('[anon'):
        return 'heap'
    if name.startswith('[stack'):
        return 'stack'
    if name.startswith('/dev/shm') or name.startswith('/memfd') or 'SYSV' in name:
        return 'shm'
    if re.search(r'\.so(\.|$)', name):
        return 'lib'
    if name.startswith('/'):
        return 'file'
    return 'other'


pid = None
while pid is None:
    pid = find()
    time.sleep(0.2)
t0 = time.time()
with open(out, 'w') as f:
    f.write('t_s,' + ','.join(KINDS) + ',total\n')
    while True:
        sums = dict.fromkeys(KINDS, 0)
        name = ''
        try:
            for line in open(f'/proc/{pid}/smaps'):
                parts = line.split()
                if re.match(r'^[0-9a-f]+-[0-9a-f]+ ', line):
                    name = parts[5] if len(parts) > 5 else ''
                elif parts[0] == 'Rss:':
                    sums[kind(name)] += int(parts[1])
        except OSError:
            break
        total = sum(sums.values())
        f.write(f'{time.time() - t0:.1f},' + ','.join(f'{sums[k] / 1024:.1f}' for k in KINDS) + f',{total / 1024:.1f}\n')
        f.flush()
        time.sleep(period)
