#!/usr/bin/env python3
"""Calls /map_save the moment the RKO-LIO offline node exits and reports wall-clock stamps.

Writes <ready_file> once the service client exists, then finds the offline_node process (scanning /proc
every 50 ms until it appears), waits for its exit on a pidfd (no polling), calls /map_save
(std_srvs/srv/Empty) at once and waits for the response.
Output JSON: frontend_exit, save_called, save_done (time.time() stamps).
Usage: save_when_frontend_done.py <out.json> <ready_file>
"""
import json
import os
import select
import sys
import time

import rclpy
from std_srvs.srv import Empty


def find_offline_node():
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                exe = f.read().split(b'\x00')[0]
        except OSError:
            continue
        if exe.endswith(b'/lib/rko_lio/offline_node'):
            return int(pid)
    return None


def main():
    out, ready = sys.argv[1], sys.argv[2]
    rclpy.init()
    node = rclpy.create_node('save_when_frontend_done')
    client = node.create_client(Empty, '/map_save')
    open(ready, 'w').close()
    deadline = time.time() + 120
    pid = None
    while pid is None and time.time() < deadline:
        pid = find_offline_node()
        if pid is None:
            time.sleep(0.05)
    if pid is None:
        sys.exit('offline_node never appeared')
    fd = os.pidfd_open(pid)
    if not client.wait_for_service(timeout_sec=120):
        sys.exit('/map_save never appeared')
    poller = select.poll()
    poller.register(fd, select.POLLIN)
    poller.poll()
    frontend_exit = time.time()
    future = client.call_async(Empty.Request())
    save_called = time.time()
    rclpy.spin_until_future_complete(node, future, timeout_sec=600)
    save_done = time.time()
    if not future.done():
        sys.exit('/map_save timed out')
    with open(out, 'w') as f:
        json.dump({'offline_node_pid': pid, 'frontend_exit': frontend_exit, 'save_called': save_called,
                   'save_done': save_done}, f, indent=2)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
