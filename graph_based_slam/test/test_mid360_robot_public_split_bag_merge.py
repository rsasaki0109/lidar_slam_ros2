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
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Tests for raw sqlite merge of public MID-360 split rosbag2 bags."""

from __future__ import annotations

import json
from pathlib import Path
import sqlite3
import subprocess
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'scripts'
CLI_PATH = SCRIPT_DIR / 'merge_mid360_robot_public_split_bags.py'
sys.path.insert(0, str(SCRIPT_DIR))

from mid360_robot_public_split_bag_merge import (  # noqa: E402
    PUBLIC_SPLIT_BAG_MERGE_JSON,
    PUBLIC_SPLIT_BAG_MERGE_MARKDOWN,
    render_split_bag_merge_markdown,
    SplitBagMergeOptions,
    SplitBagMerger,
)

import pytest  # noqa: E402
import yaml  # noqa: E402


TOPICS = (
    (
        1,
        '/livox/points',
        'sensor_msgs/msg/PointCloud2',
        'cdr',
        '- reliability: 2',
    ),
    (
        2,
        '/livox/imu',
        'sensor_msgs/msg/Imu',
        'cdr',
        '- reliability: 1',
    ),
)


def _write_sqlite_bag(
    bag_path: Path,
    *,
    db_name: str,
    messages: list[tuple[int, int, bytes]],
) -> None:
    bag_path.mkdir(parents=True)
    db_path = bag_path / db_name
    with sqlite3.connect(db_path) as connection:
        connection.execute(
            'create table topics('
            'id integer primary key,'
            'name text not null,'
            'type text not null,'
            'serialization_format text not null,'
            'offered_qos_profiles text not null)'
        )
        connection.execute(
            'create table messages('
            'id integer primary key,'
            'topic_id integer not null,'
            'timestamp integer not null,'
            'data blob not null)'
        )
        connection.execute(
            'create table schema(schema_version integer primary key,ros_distro text not null)'
        )
        connection.execute(
            'create table metadata('
            'id integer primary key,'
            'metadata_version integer not null,'
            'metadata text not null)'
        )
        connection.executemany(
            'insert into topics(id, name, type, serialization_format, offered_qos_profiles) '
            'values (?, ?, ?, ?, ?)',
            TOPICS,
        )
        for index, (topic_id, timestamp, data) in enumerate(messages, start=1):
            connection.execute(
                'insert into messages(id, topic_id, timestamp, data) values (?, ?, ?, ?)',
                (index, topic_id, timestamp, data),
            )
        connection.execute(
            'insert into schema(schema_version, ros_distro) values (?, ?)',
            (3, 'humble'),
        )
        connection.execute('create index timestamp_idx on messages (timestamp ASC)')

    timestamps = [timestamp for _, timestamp, _ in messages]
    start = min(timestamps)
    end = max(timestamps)
    per_topic = {
        topic_name: sum(1 for topic_id, _, _ in messages if topic_id == source_topic_id)
        for source_topic_id, topic_name, *_ in TOPICS
    }
    metadata = {
        'rosbag2_bagfile_information': {
            'version': 5,
            'storage_identifier': 'sqlite3',
            'duration': {'nanoseconds': end - start},
            'starting_time': {'nanoseconds_since_epoch': start},
            'message_count': len(messages),
            'topics_with_message_count': [
                {
                    'topic_metadata': {
                        'name': name,
                        'type': msgtype,
                        'serialization_format': serialization_format,
                        'offered_qos_profiles': qos,
                    },
                    'message_count': per_topic[name],
                }
                for _, name, msgtype, serialization_format, qos in TOPICS
            ],
            'compression_format': '',
            'compression_mode': '',
            'relative_file_paths': [db_name],
            'files': [
                {
                    'path': db_name,
                    'starting_time': {'nanoseconds_since_epoch': start},
                    'duration': {'nanoseconds': end - start},
                    'message_count': len(messages),
                }
            ],
        }
    }
    (bag_path / 'metadata.yaml').write_text(
        yaml.safe_dump(metadata, sort_keys=False),
        encoding='utf-8',
    )


def test_merger_writes_combined_bag_sidecars_and_gap_report(tmp_path: Path):
    bag_a = tmp_path / 'bag_a'
    bag_b = tmp_path / 'bag_b'
    _write_sqlite_bag(
        bag_a,
        db_name='a.db3',
        messages=[
            (1, 1_000, b'a-points-0'),
            (2, 1_500, b'a-imu-0'),
            (1, 2_000, b'a-points-1'),
        ],
    )
    _write_sqlite_bag(
        bag_b,
        db_name='b.db3',
        messages=[
            (2, 4_000, b'b-imu-0'),
            (1, 5_000, b'b-points-0'),
        ],
    )

    output_bag = tmp_path / 'merged'
    report = SplitBagMerger().merge(
        SplitBagMergeOptions(
            input_bags=(bag_a, bag_b),
            output_bag=output_bag,
            force=True,
        )
    )

    assert report['status'] == 'PASS'
    assert report['copy']['total_messages'] == 5
    assert report['schema'] == {'schema_version': 3, 'ros_distro': 'humble'}
    assert report['copy']['message_counts'] == {
        '/livox/imu': 2,
        '/livox/points': 3,
    }
    assert report['inter_bag_gaps'][0]['gap_ns'] == 2_000
    assert (output_bag / PUBLIC_SPLIT_BAG_MERGE_JSON).is_file()
    assert (output_bag / PUBLIC_SPLIT_BAG_MERGE_MARKDOWN).is_file()

    metadata = yaml.safe_load((output_bag / 'metadata.yaml').read_text(encoding='utf-8'))
    info = metadata['rosbag2_bagfile_information']
    assert info['message_count'] == 5
    assert info['relative_file_paths'] == ['merged_0.db3']

    with sqlite3.connect(output_bag / 'merged_0.db3') as connection:
        assert connection.execute('select * from schema').fetchall() == [(3, 'humble')]
        assert connection.execute(
            "select name from sqlite_master where type='index' and name='timestamp_idx'"
        ).fetchone()
        rows = connection.execute(
            'select id, timestamp, data from messages order by id'
        ).fetchall()
    assert [row[0] for row in rows] == [1, 2, 3, 4, 5]
    assert [row[1] for row in rows] == [1_000, 1_500, 2_000, 4_000, 5_000]

    markdown = render_split_bag_merge_markdown(report)
    assert 'Inter-Bag Gaps' in markdown
    assert '/livox/points' in markdown


def test_merger_rejects_overlapping_bags(tmp_path: Path):
    bag_a = tmp_path / 'bag_a'
    bag_b = tmp_path / 'bag_b'
    _write_sqlite_bag(
        bag_a,
        db_name='a.db3',
        messages=[(1, 1_000, b'a'), (1, 2_000, b'b')],
    )
    _write_sqlite_bag(
        bag_b,
        db_name='b.db3',
        messages=[(1, 1_500, b'overlap'), (1, 3_000, b'c')],
    )

    with pytest.raises(ValueError, match='overlap'):
        SplitBagMerger().merge(
            SplitBagMergeOptions(
                input_bags=(bag_a, bag_b),
                output_bag=tmp_path / 'merged',
                force=True,
            )
        )


def test_cli_writes_json_and_bag(tmp_path: Path):
    bag_a = tmp_path / 'bag_a'
    bag_b = tmp_path / 'bag_b'
    _write_sqlite_bag(bag_a, db_name='a.db3', messages=[(1, 1_000, b'a')])
    _write_sqlite_bag(bag_b, db_name='b.db3', messages=[(1, 2_000, b'b')])

    output_bag = tmp_path / 'merged'
    result = subprocess.run(
        [
            sys.executable,
            str(CLI_PATH),
            '--input-bag',
            str(bag_a),
            '--input-bag',
            str(bag_b),
            '--output-bag',
            str(output_bag),
            '--json',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )

    report = json.loads(result.stdout)
    assert report['status'] == 'PASS'
    assert report['copy']['total_messages'] == 2
    assert (output_bag / 'metadata.yaml').is_file()
    assert (output_bag / 'merged_0.db3').is_file()
