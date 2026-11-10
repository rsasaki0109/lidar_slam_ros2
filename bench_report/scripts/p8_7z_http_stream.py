#!/usr/bin/env python3
"""Write the single file of a remote .7z archive (one LZMA2 folder) to standard output, without storing the archive.

For FusionPortable V2: each sensor_data/<name>.7z on Google Drive holds one ROS 1 bag of 7 to 80 GB, more than the free
disk. The archive's header (at its end) is read with ranged requests; the packed stream is then downloaded once, from
front to back, and decoded on the fly. A dropped connection is resumed at the byte it stopped at.

usage: p8_7z_http_stream.py URL [--summary OUT.json] [--list] | consumer
  --list prints the archive's file names and sizes and reads nothing else.
The summary holds the sha256 of the packed bytes, the unpacked size and CRC32 against the archive's header.

Run with `python3 -I`: the archive is untrusted input. Nothing is executed or written except standard output.
"""
import argparse
import hashlib
import io
import json
import lzma
import sys
import time
import zlib

import py7zr
import requests


class HttpFile(io.RawIOBase):
    """Seekable read-only file over HTTP range requests, for the archive header only."""

    def __init__(self, url, block=1 << 20):
        self.url, self.block, self.pos, self.session, self.cache = url, block, 0, requests.Session(), {}
        r = self.session.get(url, headers={'Range': 'bytes=0-0'}, timeout=60)
        r.raise_for_status()
        self.size = int(r.headers['Content-Range'].split('/')[1])

    def readable(self):
        return True

    def seekable(self):
        return True

    def tell(self):
        return self.pos

    def seek(self, off, whence=0):
        self.pos = off if whence == 0 else self.pos + off if whence == 1 else self.size + off
        return self.pos

    def _block(self, i):
        if i not in self.cache:
            a = i * self.block
            b = min(self.size, a + self.block) - 1
            r = self.session.get(self.url, headers={'Range': f'bytes={a}-{b}'}, timeout=120)
            r.raise_for_status()
            if len(r.content) != b - a + 1:
                raise IOError(f'short range {a}-{b}')
            if len(self.cache) > 8:
                self.cache.pop(next(iter(self.cache)))
            self.cache[i] = r.content
        return self.cache[i]

    def read(self, n=-1):
        n = self.size - self.pos if n < 0 else min(n, self.size - self.pos)
        out = bytearray()
        while n > 0:
            i, o = divmod(self.pos, self.block)
            d = self._block(i)[o:o + n]
            out += d
            self.pos += len(d)
            n -= len(d)
        return bytes(out)

    def readinto(self, b):
        d = self.read(len(b))
        b[:len(d)] = d
        return len(d)


def packed_bytes(url, start, length):
    """The bytes [start, start + length) of the URL, in order, resuming after a dropped connection."""
    done, failures = 0, 0
    while done < length:
        try:
            r = requests.get(url, headers={'Range': f'bytes={start + done}-{start + length - 1}'}, stream=True,
                             timeout=120)
            r.raise_for_status()
            for part in r.iter_content(1 << 20):
                failures = 0
                done += len(part)
                yield part
        except (requests.RequestException, IOError) as e:
            failures += 1
            if failures > 20:
                raise
            print(f'p8_7z_http_stream: {e!r} at {done}, retry {failures}', file=sys.stderr)
            time.sleep(min(60, 2 ** failures))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('url')
    ap.add_argument('--summary')
    ap.add_argument('--list', action='store_true')
    a = ap.parse_args()
    z = py7zr.SevenZipFile(HttpFile(a.url))
    files = [e for e in z.list() if not e.is_directory]
    if a.list:
        for e in files:
            print(e.filename, e.uncompressed)
        return
    ms = z.header.main_streams
    folders = ms.unpackinfo.folders
    if len(files) != 1 or len(folders) != 1 or len(folders[0].coders) != 1 or folders[0].coders[0]['method'] != b'!':
        sys.exit('expected one file in one LZMA2 folder')
    prop = folders[0].coders[0]['properties'][0]
    dict_size = 0xFFFFFFFF if prop == 40 else (2 | (prop & 1)) << (prop // 2 + 11)
    start = 32 + ms.packinfo.packpos
    length = ms.packinfo.packsizes[0]
    dec = lzma.LZMADecompressor(format=lzma.FORMAT_RAW, filters=[{'id': lzma.FILTER_LZMA2, 'dict_size': dict_size}])
    sha, crc, n_out, out = hashlib.sha256(), 0, 0, sys.stdout.buffer
    try:
        for part in packed_bytes(a.url, start, length):
            sha.update(part)
            data = dec.decompress(part)
            crc = zlib.crc32(data, crc)
            n_out += len(data)
            out.write(data)
        out.flush()
        complete = True
    except BrokenPipeError:  # the consumer stopped reading (a pipeline check that takes only the first scans)
        complete = False
    summary = {'file': files[0].filename, 'packed_bytes': length, 'packed_sha256': sha.hexdigest() if complete else None,
               'unpacked_bytes': n_out, 'unpacked_expected': files[0].uncompressed, 'crc32': f'{crc:08x}',
               'crc32_expected': f'{files[0].crc32:08x}' if files[0].crc32 is not None else None,
               'complete': complete}
    if a.summary:
        with open(a.summary, 'w') as f:
            f.write(json.dumps(summary, indent=1) + '\n')
    print(json.dumps(summary), file=sys.stderr)
    if complete and (n_out != files[0].uncompressed or
                     (files[0].crc32 is not None and crc != files[0].crc32)):
        sys.exit('size or CRC32 differs from the archive header')


if __name__ == '__main__':
    main()
