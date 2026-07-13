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

import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'runtime_report', ROOT / 'scripts/write_runtime_report.py')
runtime = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(runtime)


def test_parse_gnu_time_reports_realtime_and_memory():
    text = (
        'Elapsed (wall clock) time (h:mm:ss or m:ss): 2:35.22\n'
        'Maximum resident set size (kbytes): 908668\n'
        'Percent of CPU this job got: 488%\n'
        'Exit status: 0\n')
    report = runtime.parse_gnu_time(text, 125.814128037)
    assert report['wall_time_sec'] == pytest.approx(155.22)
    assert report['realtime_factor'] == pytest.approx(1.2337, rel=1e-3)
    assert report['peak_rss_mb'] == pytest.approx(887.371, rel=1e-3)
    assert report['cpu_percent'] == 488.0


def test_elapsed_seconds_accepts_hours_and_rejects_bad_input():
    assert runtime.elapsed_seconds('1:02:03.5') == 3723.5
    with pytest.raises(ValueError):
        runtime.elapsed_seconds('3.5')


def test_reads_duration_from_rosbag2_metadata(tmp_path):
    metadata = tmp_path / 'metadata.yaml'
    metadata.write_text(
        'rosbag2_bagfile_information:\n'
        '  duration:\n'
        '    nanoseconds: 125814128037\n')

    assert runtime.read_rosbag2_duration(metadata) == pytest.approx(125.814128037)


def test_rejects_metadata_without_duration(tmp_path):
    metadata = tmp_path / 'metadata.yaml'
    metadata.write_text('rosbag2_bagfile_information: {}\n')

    with pytest.raises(ValueError, match='duration.nanoseconds'):
        runtime.read_rosbag2_duration(metadata)
