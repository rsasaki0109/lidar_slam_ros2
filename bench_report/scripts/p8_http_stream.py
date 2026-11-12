#!/usr/bin/env python3
"""Write a remote file to standard output, resuming with a Range request where a dropped connection stopped.

usage: p8_http_stream.py URL | consumer
For the second blind run's bags on Zenodo (2 to 8 GB each, streamed into the converter and never stored): a plain
`curl URL |` cannot continue after "transfer closed with N bytes remaining".
"""
import sys
import time

import requests


def main():
    url, done, size, failures, out = sys.argv[1], 0, None, 0, sys.stdout.buffer
    while size is None or done < size:
        try:
            r = requests.get(url, headers={'Range': f'bytes={done}-'} if done else {}, stream=True, timeout=120)
            if r.status_code not in (200, 206) or (done and r.status_code != 206):
                raise IOError(f'status {r.status_code}')
            if size is None:
                size = int(r.headers['Content-Length'])
            for part in r.iter_content(1 << 20):
                failures = 0
                done += len(part)
                out.write(part)
        except (requests.RequestException, IOError, KeyError) as e:
            failures += 1
            if failures > 30:
                raise
            print(f'p8_http_stream: {e!r} at {done}, retry {failures}', file=sys.stderr, flush=True)
            time.sleep(min(300, 2 ** failures))
    out.flush()


if __name__ == '__main__':
    main()
