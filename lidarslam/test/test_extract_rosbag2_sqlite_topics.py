# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Tests for the one-pass rosbag2 SQLite topic extractor."""

import hashlib
import importlib.util
import io
from pathlib import Path
import random
import sqlite3

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'extract_rosbag2_sqlite_topics.py'
SPEC = importlib.util.spec_from_file_location('extract_rosbag2_sqlite_topics', SCRIPT)
TOOL = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(TOOL)

TOPICS = [('/livox/points', 'sensor_msgs/msg/PointCloud2'),
          ('/livox/imu', 'sensor_msgs/msg/Imu'),
          ('/camera/image_raw/compressed', 'sensor_msgs/msg/CompressedImage'),
          ('/gnss/fix', 'sensor_msgs/msg/NavSatFix')]
# sizes around the local/overflow boundaries and long chains
SIZES = [1, 100, 489, 490, 4061, 4062, 4093, 4580, 8184, 70000, 300001]


def make_bag(path, humble_schema=False, rows=240, seed=7):
    """Write a rosbag2-style SQLite file; return the expected per-topic rows."""
    con = sqlite3.connect(path)
    con.execute('PRAGMA page_size=4096')
    if humble_schema:
        con.execute('CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT NOT NULL, '
                    'type TEXT NOT NULL, serialization_format TEXT NOT NULL, '
                    'offered_qos_profiles TEXT NOT NULL)')
    else:
        con.execute('CREATE TABLE schema(schema_version INTEGER PRIMARY KEY, '
                    'ros_distro TEXT NOT NULL)')
        con.execute("INSERT INTO schema VALUES (4, 'jazzy')")
        con.execute('CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT NOT NULL, '
                    'type TEXT NOT NULL, serialization_format TEXT NOT NULL, '
                    'offered_qos_profiles TEXT NOT NULL, type_description_hash TEXT NOT NULL)')
    con.execute('CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL, '
                'timestamp INTEGER NOT NULL, data BLOB NOT NULL)')
    con.execute('CREATE INDEX timestamp_idx ON messages (timestamp ASC)')
    for i, (name, typ) in enumerate(TOPICS, start=1):
        if humble_schema:
            con.execute('INSERT INTO topics VALUES (?, ?, ?, ?, ?)', (i, name, typ, 'cdr', ''))
        else:
            con.execute('INSERT INTO topics VALUES (?, ?, ?, ?, ?, ?)',
                        (i, name, typ, 'cdr', '', 'RIHS01_' + '0' * 64))
    rng = random.Random(seed)
    expected = {}
    stamp = 1_762_269_335_000_000_000
    for rowid in range(1, rows + 1):
        topic_id = rng.randint(1, len(TOPICS))
        data = rng.randbytes(rng.choice(SIZES))
        stamp += rng.randint(1, 10_000_000)
        con.execute('INSERT INTO messages VALUES (?, ?, ?, ?)', (rowid, topic_id, stamp, data))
        expected.setdefault(TOPICS[topic_id - 1][0], []).append((rowid, stamp, data))
    con.commit()
    con.close()
    return expected


def extract(path, topics, chunk=1 << 20):
    extractor = TOOL.Extractor(topics)
    kept = {}

    def emit(rowid, message):
        topic_id, stamp, data = message
        kept.setdefault(extractor.topics[topic_id][0], []).append((rowid, stamp, data))

    raw = Path(path).read_bytes()
    stream = io.BytesIO(raw)
    total, digest = TOOL.run(iter(lambda: stream.read(chunk), b''), extractor, emit)
    assert not extractor.inflight
    assert not extractor.late
    for rowid, message in list(extractor.ready()):
        emit(rowid, message)
    assert total == len(raw)
    assert digest == hashlib.sha256(raw).hexdigest()
    return extractor, kept


@pytest.mark.parametrize('humble_schema', [False, True])
@pytest.mark.parametrize('chunk', [5000, 1 << 20])
def test_selected_topics_match_sqlite(tmp_path, humble_schema, chunk):
    db = tmp_path / 'bag_0.db3'
    expected = make_bag(db, humble_schema=humble_schema)
    wanted = ['/livox/points', '/livox/imu', '/gnss/fix']
    extractor, kept = extract(db, wanted, chunk=chunk)
    assert set(kept) == {t for t in wanted if t in expected}
    for topic in kept:
        assert kept[topic] == expected[topic], topic
    names = {tid: v[0] for tid, v in extractor.topics.items()}
    assert {names[t]: n for t, n in extractor.counts.items()} == {
        t: len(rows) for t, rows in expected.items()}
    assert extractor.message_cells == sum(len(rows) for rows in expected.values())


def test_local_payload_size_matches_sqlite_spill(tmp_path):
    """A row exactly at each boundary is read back intact (no off-by-one in the spill)."""
    db = tmp_path / 'edge_0.db3'
    con = sqlite3.connect(db)
    con.execute('PRAGMA page_size=4096')
    con.execute('CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT NOT NULL, '
                'type TEXT NOT NULL, serialization_format TEXT NOT NULL, '
                'offered_qos_profiles TEXT NOT NULL, type_description_hash TEXT NOT NULL)')
    con.execute('CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL, '
                'timestamp INTEGER NOT NULL, data BLOB NOT NULL)')
    con.execute("INSERT INTO topics VALUES (1, '/t', 'std_msgs/msg/String', 'cdr', '', '')")
    rows = []
    for rowid, n in enumerate(range(4040, 4110), start=1):
        data = bytes([rowid % 251]) * n
        con.execute('INSERT INTO messages VALUES (?, 1, ?, ?)', (rowid, rowid, data))
        rows.append((rowid, rowid, data))
    con.commit()
    con.close()
    _, kept = extract(db, ['/t'])
    assert kept['/t'] == rows


class _CutPipe:
    def __init__(self, data, cut):
        self._stream = io.BytesIO(data[:cut] if cut is not None else data)

    def read(self, n):
        return self._stream.read(n)


class _FakeCurl:
    def __init__(self, data, cuts):
        self.data = data
        self.cuts = list(cuts)
        self.offsets = []

    def __call__(self, argv, stdout=None):
        offset = int(argv[argv.index('-r') + 1].rstrip('-'))
        self.offsets.append(offset)
        cut = self.cuts.pop(0) if self.cuts else None
        proc = type('P', (), {})()
        proc.stdout = _CutPipe(self.data[offset:], cut)
        proc.wait = lambda: 0
        return proc


def test_http_chunks_resumes_cut_transfers():
    data = bytes(range(256)) * 4000
    curl = _FakeCurl(data, cuts=[100_000, 7, 300_000])
    got = b''.join(TOOL.http_chunks('https://example.invalid/bag.db3', len(data),
                                    popen=curl, sleep=lambda s: None))
    assert got == data
    assert curl.offsets == [0, 100_000, 100_007, 400_007]


def test_http_chunks_gives_up():
    data = b'x' * 1000
    curl = _FakeCurl(data, cuts=[0] * 10)
    with pytest.raises(RuntimeError, match='gave up'):
        b''.join(TOOL.http_chunks('https://example.invalid/bag.db3', len(data), popen=curl,
                                  max_resumes=3, sleep=lambda s: None))


def test_rejects_non_sqlite():
    extractor = TOOL.Extractor(['/t'])
    with pytest.raises(RuntimeError, match='not an SQLite'):
        extractor.feed_page(b'\0' * TOOL.PAGE_SIZE)


def test_writes_a_readable_bag(tmp_path):
    rosbag2_py = pytest.importorskip('rosbag2_py')
    db = tmp_path / 'src_0.db3'
    expected = make_bag(db, rows=60)
    out = tmp_path / 'out'
    try:
        TOOL.main(['--src', str(db), '--topics', '/livox/imu', '/gnss/fix', '--out', str(out),
                   '--storage', 'sqlite3', '--preset', ''])
    except RuntimeError as exc:  # topic absent from this random draw
        pytest.skip(str(exc))
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(out), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    got = {}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        got.setdefault(topic, []).append((stamp, bytes(data)))
    for topic in ('/livox/imu', '/gnss/fix'):
        assert got[topic] == [(s, d) for _, s, d in expected[topic]]
