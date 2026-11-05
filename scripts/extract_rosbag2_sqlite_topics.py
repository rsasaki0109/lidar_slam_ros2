#!/usr/bin/env python3
r"""Keep chosen topics of a rosbag2 SQLite (.db3) bag while reading it once, front to back.

The source is read sequentially from a local path, stdin, or an http(s) URL; the
whole .db3 never has to be on disk. A cut HTTP transfer is resumed with a Range
request at the same byte offset. As pages pass, table b-tree leaf pages of the
``messages`` table are parsed and the overflow chains of the selected rows are
followed (later pages are awaited, recent pages are kept in a ring buffer). The
selected messages are written to a new rosbag2 bag with rosbag2_py.

Example (RTK-SLAM Stadtgarten 1: 30.3 GB .db3 -> 4.5 GB MCAP with LiDAR, IMU, GNSS)::

    U=https://huggingface.co/datasets/Willyzw/rtk-slam-dataset/resolve/<rev>
    python3 scripts/extract_rosbag2_sqlite_topics.py \\
      --src $U/ros2/stadtgarten_seq1/stadtgarten_seq1.db3 \\
      --size 30263574528 --sha256 <sha256 from download_rtk_slam_dataset.py> \\
      --topics /livox/points /livox/imu /gnss/fix --out stadtgarten_seq1_lio
"""

from __future__ import annotations

import argparse
import collections
import hashlib
import struct
import subprocess
import sys
import time
from typing import Callable, Iterable, Iterator

PAGE_SIZE = 4096
MAX_LOCAL = PAGE_SIZE - 35
MIN_LOCAL = ((PAGE_SIZE - 12) * 32) // 255 - 23
OVERFLOW_DATA = PAGE_SIZE - 4
INT_BYTES = {1: 1, 2: 2, 3: 3, 4: 4, 5: 6, 6: 8}
INT_TYPES = set(INT_BYTES) | {8, 9}
CHUNK = 16 << 20


def varint(buf: bytes, i: int) -> tuple[int, int]:
    """Decode an SQLite varint at ``buf[i]``; return (value, next index)."""
    value = 0
    for k in range(8):
        c = buf[i + k]
        value = (value << 7) | (c & 0x7F)
        if c < 0x80:
            return value, i + k + 1
    return (value << 8) | buf[i + 8], i + 9


def local_payload_size(payload: int) -> int:
    """Bytes of a table-leaf cell payload stored on the leaf page itself."""
    if payload <= MAX_LOCAL:
        return payload
    k = MIN_LOCAL + (payload - MIN_LOCAL) % OVERFLOW_DATA
    return k if k <= MAX_LOCAL else MIN_LOCAL


def column_bytes(serial_type: int) -> int:
    """Size of a record column with the given serial type."""
    if serial_type in INT_BYTES:
        return INT_BYTES[serial_type]
    if serial_type == 7:
        return 8
    if serial_type in (0, 8, 9):
        return 0
    if serial_type >= 12:
        return (serial_type - 12) // 2 if serial_type % 2 == 0 else (serial_type - 13) // 2
    raise ValueError(f'unsupported serial type {serial_type}')


def read_int(buf: bytes, i: int, serial_type: int) -> int:
    """Read an integer column."""
    if serial_type == 8:
        return 0
    if serial_type == 9:
        return 1
    return int.from_bytes(buf[i:i + INT_BYTES[serial_type]], 'big', signed=True)


def record_header(payload: bytes) -> tuple[int, list[int]]:
    """Return (header size, serial types) of a record."""
    size, i = varint(payload, 0)
    types = []
    while i < size:
        t, i = varint(payload, i)
        types.append(t)
    return size, types


class Extractor:
    """Page-by-page parser of a rosbag2 SQLite file that collects selected messages."""

    def __init__(self, topics: Iterable[str], ring_pages: int = 65536) -> None:
        """Select ``topics``; keep the last ``ring_pages`` overflow-like pages in memory."""
        self.wanted_names = set(topics)
        self.topics: dict[int, tuple[str, ...]] = {}
        self.wanted_ids: set[int] = set()
        self.pending: dict[int, int] = {}
        self.inflight: dict[int, list] = {}
        self.done: dict[int, tuple[int, int, bytes]] = {}
        self.late: list[tuple[int, int]] = []
        self.ring: collections.OrderedDict[int, bytes] = collections.OrderedDict()
        self.ring_pages = ring_pages
        self.page_no = 0
        self.counts: dict[int, int] = {}
        self.payload_bytes: dict[int, int] = {}
        self.leaf_pages = 0
        self.message_cells = 0

    def _take(self, rowid: int, page: bytes) -> bool:
        message = self.inflight[rowid]
        n = min(message[3], OVERFLOW_DATA)
        message[2] += page[4:4 + n]
        message[3] -= n
        if message[3] == 0:
            self.done[rowid] = (message[0], message[1], bytes(message[2]))
            del self.inflight[rowid]
            return True
        return False

    def follow(self, rowid: int, next_page: int) -> None:
        """Consume overflow pages of ``rowid`` from ``next_page`` as far as they are known."""
        while True:
            if next_page > self.page_no:
                self.pending[next_page] = rowid
                return
            page = self.ring.get(next_page)
            if page is None:
                self.late.append((rowid, next_page))
                return
            if self._take(rowid, page):
                return
            next_page = struct.unpack_from('>I', page, 0)[0]

    def _leaf(self, page: bytes) -> None:
        self.leaf_pages += 1
        for c in range(struct.unpack_from('>H', page, 3)[0]):
            offset = struct.unpack_from('>H', page, 8 + 2 * c)[0]
            payload_size, i = varint(page, offset)
            rowid, i = varint(page, i)
            local = local_payload_size(payload_size)
            payload = page[i:i + local]
            first_overflow = (struct.unpack_from('>I', page, i + local)[0]
                              if payload_size > local else 0)
            header, types = record_header(payload)
            if (len(types) == 4 and types[0] == 0 and types[1] in INT_TYPES
                    and types[2] in INT_TYPES and types[3] >= 12 and types[3] % 2 == 0):
                self._message(rowid, payload_size, payload, header, types, first_overflow)
            elif (len(types) in (5, 6) and types[0] == 0
                  and all(t >= 13 and t % 2 == 1 for t in types[1:])):
                if payload_size > local:
                    raise RuntimeError('topics row spills into overflow pages; not supported')
                values, j = [], header
                for t in types[1:]:
                    n = column_bytes(t)
                    values.append(payload[j:j + n].decode())
                    j += n
                self.topics[rowid] = tuple(values)
                if values[0] in self.wanted_names:
                    self.wanted_ids.add(rowid)

    def _message(self, rowid: int, payload_size: int, payload: bytes, header: int,
                 types: list[int], first_overflow: int) -> None:
        self.message_cells += 1
        j = header
        topic_id = read_int(payload, j, types[1])
        j += column_bytes(types[1])
        stamp = read_int(payload, j, types[2])
        j += column_bytes(types[2])
        self.counts[topic_id] = self.counts.get(topic_id, 0) + 1
        self.payload_bytes[topic_id] = self.payload_bytes.get(topic_id, 0) + payload_size
        if not self.topics:
            raise RuntimeError('message rows appear before the topics table')
        if topic_id not in self.wanted_ids:
            return
        if j + column_bytes(types[3]) != payload_size:
            raise RuntimeError(f'row {rowid}: data column does not end the record')
        remaining = payload_size - len(payload)
        if remaining == 0:
            self.done[rowid] = (topic_id, stamp, bytes(payload[j:]))
            return
        self.inflight[rowid] = [topic_id, stamp, bytearray(payload[j:]), remaining]
        self.follow(rowid, first_overflow)

    def feed_page(self, page: bytes) -> None:
        """Process the next page of the file."""
        self.page_no += 1
        if self.page_no == 1:
            if page[:16] != b'SQLite format 3\x00':
                raise RuntimeError('not an SQLite database')
            if struct.unpack_from('>H', page, 16)[0] != PAGE_SIZE or page[20] != 0:
                raise RuntimeError(
                    f'only {PAGE_SIZE}-byte pages without reserved bytes are supported')
            return
        rowid = self.pending.pop(self.page_no, None)
        if rowid is not None and not self._take(rowid, page):
            self.follow(rowid, struct.unpack_from('>I', page, 0)[0])
        if page[0] == 0x0D:
            self._leaf(page)
        elif page[0] == 0:  # overflow pages start with the high byte of a page number
            self.ring[self.page_no] = page
            if len(self.ring) > self.ring_pages:
                self.ring.popitem(last=False)

    def ready(self) -> Iterator[tuple[int, tuple[int, int, bytes]]]:
        """Completed messages that no unfinished selected row precedes, in rowid order."""
        limit = min(self.inflight) if self.inflight else None
        for rowid in sorted(r for r in self.done if limit is None or r < limit):
            yield rowid, self.done.pop(rowid)


def http_chunks(url: str, size: int | None, popen: Callable = subprocess.Popen,
                max_resumes: int = 50,
                sleep: Callable[[float], None] = time.sleep) -> Iterator[bytes]:
    """Yield the bytes of ``url`` in order; a cut transfer resumes at the same offset."""
    offset, resumes = 0, 0
    while size is None or offset < size:
        proc = popen(['curl', '-sSfL', '--retry', '5', '-r', f'{offset}-', url],
                     stdout=subprocess.PIPE)
        while True:
            data = proc.stdout.read(CHUNK)
            if not data:
                break
            offset += len(data)
            yield data
        proc.wait()
        if size is None:
            return
        if offset < size:
            resumes += 1
            if resumes > max_resumes:
                raise RuntimeError(
                    f'transfer cut at byte {offset}; gave up after {max_resumes} resumes')
            print(f'transfer cut at byte {offset}; resuming', flush=True)
            sleep(min(60, 2 ** min(resumes, 6)))


def fetch_pages(url: str, first_page: int, count: int) -> list[bytes]:
    """Fetch ``count`` pages starting at ``first_page`` with one Range request."""
    lo = (first_page - 1) * PAGE_SIZE
    hi = lo + count * PAGE_SIZE - 1
    data = subprocess.run(['curl', '-sSfL', '--retry', '5', '-r', f'{lo}-{hi}', url],
                          check=True, capture_output=True).stdout
    return [data[k * PAGE_SIZE:(k + 1) * PAGE_SIZE]
            for k in range(len(data) // PAGE_SIZE)]


def run(chunks: Iterable[bytes], extractor: Extractor,
        emit: Callable[[int, tuple[int, int, bytes]], None],
        progress: Callable[[int], None] | None = None) -> tuple[int, str]:
    """Feed the byte stream through the extractor; return (bytes read, sha256)."""
    digest = hashlib.sha256()
    total = 0
    rest = b''
    for chunk in chunks:
        digest.update(chunk)
        total += len(chunk)
        buf = rest + chunk if rest else chunk
        n = len(buf) // PAGE_SIZE
        view = memoryview(buf)
        for k in range(n):
            extractor.feed_page(bytes(view[k * PAGE_SIZE:(k + 1) * PAGE_SIZE]))
        rest = bytes(view[n * PAGE_SIZE:])
        for rowid, message in list(extractor.ready()):
            emit(rowid, message)
        if progress is not None:
            progress(total)
    if rest:
        raise RuntimeError(f'{len(rest)} trailing bytes: not a whole number of pages')
    return total, digest.hexdigest()


def resolve_late(extractor: Extractor, url: str | None) -> None:
    """Fetch overflow chains that started behind the ring buffer by Range requests."""
    if not extractor.late:
        return
    if url is None:
        raise RuntimeError(f'{len(extractor.late)} overflow chains start behind the ring buffer; '
                           'an http source is needed to fetch them')
    todo, extractor.late = extractor.late, []
    extractor.page_no = 1 << 40
    for rowid, next_page in todo:
        while rowid in extractor.inflight:
            need = -(-extractor.inflight[rowid][3] // OVERFLOW_DATA)
            extractor.ring = collections.OrderedDict(
                (next_page + k, p) for k, p in enumerate(fetch_pages(url, next_page, need)))
            before = extractor.inflight[rowid][3]
            extractor.follow(rowid, next_page)
            if rowid in extractor.inflight:
                if extractor.inflight[rowid][3] == before:
                    raise RuntimeError('range fetch made no progress')
                _, next_page = extractor.late.pop()


class BagWriter:
    """rosbag2_py writer that creates the selected topics on first use."""

    def __init__(self, uri: str, storage: str, preset: str) -> None:
        """Open a new bag at ``uri``."""
        import rosbag2_py
        self._rosbag2_py = rosbag2_py
        self._writer = rosbag2_py.SequentialWriter()
        self._writer.open(
            rosbag2_py.StorageOptions(uri=uri, storage_id=storage,
                                      storage_preset_profile=preset),
            rosbag2_py.ConverterOptions('cdr', 'cdr'))
        self._created: set[str] = set()

    def write(self, topic: tuple[str, ...], data: bytes, stamp: int) -> None:
        """Write one serialized message; ``topic`` is the topics-table row."""
        name = topic[0]
        if name not in self._created:
            metadata = self._rosbag2_py.TopicMetadata
            try:  # Jazzy and later: id, name, type, format, qos, type hash
                meta = metadata(0, name, topic[1], topic[2], [],
                                topic[4] if len(topic) > 4 else '')
            except TypeError:  # Humble: name, type, format
                meta = metadata(name=name, type=topic[1], serialization_format=topic[2])
            self._writer.create_topic(meta)
            self._created.add(name)
        self._writer.write(name, data, stamp)

    def close(self) -> None:
        """Flush and close the bag."""
        del self._writer


def main(argv: list[str] | None = None) -> int:
    """Command-line entry point."""
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--src', required=True, help='path, - for stdin, or http(s) URL of the .db3')
    ap.add_argument('--topics', nargs='+', required=True)
    ap.add_argument('--out', help='output bag directory (must not exist)')
    ap.add_argument('--storage', default='mcap')
    ap.add_argument('--preset', default='zstd_fast', help="storage preset ('' for none)")
    ap.add_argument('--size', type=int,
                    help='expected size in bytes (required for an http source)')
    ap.add_argument('--sha256', help='expected sha256 of the whole .db3')
    ap.add_argument('--list-only', action='store_true',
                    help='parse and report per-topic counts and hashes, write nothing')
    a = ap.parse_args(argv)
    is_url = a.src.startswith(('http://', 'https://'))
    if is_url and a.size is None:
        ap.error('--size is required for an http source')
    if not a.list_only and not a.out:
        ap.error('--out is required unless --list-only')

    extractor = Extractor(a.topics)
    writer = None if a.list_only else BagWriter(a.out, a.storage, a.preset)
    hashes: dict[int, hashlib._Hash] = {}
    kept = [0]

    def emit(rowid: int, message: tuple[int, int, bytes]) -> None:
        topic_id, stamp, data = message
        h = hashes.setdefault(topic_id, hashlib.sha256())
        h.update(struct.pack('<qq', rowid, stamp))
        h.update(data)
        kept[0] += 1
        if writer is not None:
            writer.write(extractor.topics[topic_id], data, stamp)

    start = time.time()
    last = [start]

    def progress(total: int) -> None:
        now = time.time()
        if now - last[0] > 30:
            last[0] = now
            rate = total / 1e6 / (now - start)
            print(f'{total / 1e9:7.2f} GB  {rate:6.1f} MB/s  kept {kept[0]}', flush=True)

    if is_url:
        chunks: Iterable[bytes] = http_chunks(a.src, a.size)
    else:
        src = sys.stdin.buffer if a.src == '-' else open(a.src, 'rb')
        chunks = iter(lambda: src.read(CHUNK), b'')
    total, digest = run(chunks, extractor, emit, progress)
    resolve_late(extractor, a.src if is_url else None)
    if extractor.inflight:
        raise RuntimeError(
            f'{len(extractor.inflight)} selected messages incomplete at end of file')
    for rowid, message in list(extractor.ready()):
        emit(rowid, message)
    if writer is not None:
        writer.close()

    missing = extractor.wanted_names - {extractor.topics[t][0] for t in extractor.wanted_ids}
    names = {tid: v[0] for tid, v in extractor.topics.items()}
    print('messages per topic:', {names.get(t, t): n for t, n in sorted(extractor.counts.items())})
    for t, h in sorted(hashes.items()):
        print(f'kept {names[t]} {extractor.counts[t]} sha256(rowid, stamp, data) {h.hexdigest()}')
    print(f'read {total} bytes, {extractor.message_cells} messages, kept {kept[0]} '
          f'in {time.time() - start:.0f} s; sha256 {digest}')
    if missing:
        raise RuntimeError(f'topics not in the bag: {sorted(missing)}')
    if a.size is not None and total != a.size:
        raise RuntimeError(f'read {total} bytes, expected {a.size}')
    if a.sha256 and digest != a.sha256:
        raise RuntimeError(f'sha256 {digest} != expected {a.sha256}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
