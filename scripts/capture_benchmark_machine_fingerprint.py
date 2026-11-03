#!/usr/bin/env python3
"""Capture the non-secret hardware fingerprint of a benchmark host."""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import platform
import sys
from typing import Any


def benchmark_machine_fingerprint() -> dict[str, Any]:
    """Return a stable, non-secret fingerprint of the benchmark host."""
    private_values = []
    for path in (Path('/etc/machine-id'),
                 Path('/sys/class/dmi/id/product_uuid'),
                 Path('/sys/class/dmi/id/board_serial')):
        try:
            value = path.read_text().strip()
        except OSError:
            value = ''
        if value:
            private_values.append(value)

    cpu_model = ''
    try:
        for line in Path('/proc/cpuinfo').read_text().splitlines():
            if line.lower().startswith('model name') and ':' in line:
                cpu_model = line.split(':', 1)[1].strip()
                break
    except OSError:
        pass
    memory_total_kb = None
    try:
        for line in Path('/proc/meminfo').read_text().splitlines():
            if line.startswith('MemTotal:'):
                memory_total_kb = int(line.split()[1])
                break
    except (OSError, ValueError, IndexError):
        pass

    public = {
        'architecture': platform.machine(),
        'cpu_model': cpu_model,
        'logical_cpu_count': os.cpu_count(),
        'memory_total_kb': memory_total_kb,
    }
    identity_payload = {
        'private_identifiers': private_values,
        **public,
    }
    machine_id = hashlib.sha256(json.dumps(
        identity_payload, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    return {'machine_id': machine_id, **public}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError(f'refusing to overwrite: {args.output}')
    document = {
        'schema_version': 1,
        'captured_at': dt.datetime.now(dt.timezone.utc).isoformat(),
        **benchmark_machine_fingerprint(),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(document, indent=2, sort_keys=True) + '\n')
    print(json.dumps(document, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError) as error:
        print(f'error: {error}', file=sys.stderr)
        sys.exit(2)
