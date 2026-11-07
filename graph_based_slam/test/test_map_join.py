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

"""Contracts for place-recognition alignment of map bundles (lidarslam-map merge)."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import textwrap

import numpy as np
import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'scripts'))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from map_join import (  # noqa: E402,I100
    initial_transform,
    JoinSettings,
    MapJoinError,
    place_recognition_alignments,
    refusal_reason,
)
from map_merge import merge_map_sessions  # noqa: E402
import merge_map_sessions as merge_cli  # noqa: E402
from test_map_merge import _fixture, _options  # noqa: E402


def _row(yaw_deg: float, x: float, y: float, z: float) -> list[float]:
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    return [c, -s, 0.0, x, s, c, 0.0, y, 0.0, 0.0, 1.0, z]


def _bundle(root: Path, *, backend: bool = True) -> Path:
    root.mkdir(parents=True)
    (root / 'map_bundle.yaml').write_text('artifacts: {}\n')
    if backend:
        (root / 'backend_input').mkdir()
        (root / 'backend_input' / 'metadata.yaml').write_text(
            'rosbag2_bagfile_information: {}\n')
    return root


def _fake_tools(tmp_path: Path, report: str) -> tuple[Path, list[str]]:
    """Return a replay script that writes runner outputs and a join command that writes report."""
    replay = tmp_path / 'replay.sh'
    replay.write_text(textwrap.dedent("""\
        #!/bin/bash
        while [[ $# -gt 0 ]]; do
          case $1 in --output-dir) out=$2; shift 2 ;; *) shift ;; esac
        done
        mkdir -p "$out/run1"
        echo '1.0 0 0 0 0 0 0 1' > "$out/run1/trajectory_raw.tum"
        echo 'from,to,fitness,tx,ty,tz,qx,qy,qz,qw' > "$out/run1/loop_edges.csv"
        """))
    report_file = tmp_path / 'report.yaml'
    report_file.write_text(report)
    join = tmp_path / 'join.sh'
    join.write_text(textwrap.dedent(f"""\
        #!/bin/bash
        for a in "$@"; do
          case $a in output_dir:=*) out=${{a#output_dir:=}} ;; esac
        done
        mkdir -p "$out"
        cp {report_file} "$out/join_report.yaml"
        """))
    return replay, ['bash', str(join)]


def _settings(tmp_path: Path, replay: Path, join: list[str]) -> JoinSettings:
    params = tmp_path / 'params.yaml'
    params.write_text('graph_based_slam:\n  ros__parameters: {}\n')
    return JoinSettings(params_path=params, replay_script=replay, join_command=join)


def test_initial_transform_reads_position_and_yaw():
    x, y, z, yaw = initial_transform(_row(30.0, 1.5, -2.0, 0.25))
    assert (x, y, z) == pytest.approx((1.5, -2.0, 0.25))
    assert yaw == pytest.approx(30.0)
    with pytest.raises(MapJoinError, match='3x4'):
        initial_transform([1.0, 0.0])


def test_refusal_reason_names_the_rule_that_refused():
    assert 'ambiguous' in refusal_reason({'ambiguous': True})
    assert 'joins itself' in refusal_reason({'ambiguous': False, 'self_similar': '1'})
    assert 'too few' in refusal_reason({'accepted': 4, 'candidates': 90, 'sets_tried': []})
    reason = refusal_reason({'sets_tried': [
        {'size': 166, 'places': 256, 'share_below': 0.101562, 'median': 0.69, 'passed': False}]})
    assert '10.2 %' in reason and '166 matches' in reason


def test_joined_sessions_become_initial_transforms(tmp_path: Path):
    row = _row(-20.0, 3.0, 4.0, 0.5)
    replay, join = _fake_tools(tmp_path, textwrap.dedent(f"""\
        sessions:
          - session: 0
            joined: true
          - session: 1
            joined: true
            candidates: 120
            accepted: 40
            ambiguous: false
            self_similar: ''
            sets_tried:
              - {{size: 30, places: 20, share_below: 0.0, median: 0.97, passed: true}}
            transform_to_session_0: {row}
        map_joins:
          - {{session: 1, tries: 1, joined_in_pass: 0, map_sessions: [0]}}
        """))
    bundles = [_bundle(tmp_path / 'a'), _bundle(tmp_path / 'b')]
    transforms, summary = place_recognition_alignments(
        bundles, tmp_path / 'work', _settings(tmp_path, replay, join))
    assert transforms[1] == pytest.approx((3.0, 4.0, 0.5, -20.0))
    assert summary['method'] == 'graph_slam_multisession_join'
    assert summary['sessions'][1]['accepted'] == 40


def test_a_refused_session_stops_with_the_reason(tmp_path: Path):
    replay, join = _fake_tools(tmp_path, textwrap.dedent("""\
        sessions:
          - session: 0
            joined: true
          - session: 1
            joined: false
            candidates: 795
            accepted: 305
            ambiguous: false
            self_similar: ''
            sets_tried:
              - {size: 166, places: 256, share_below: 0.101562, median: 0.686, passed: false}
        """))
    bundles = [_bundle(tmp_path / 'a'), _bundle(tmp_path / 'b')]
    with pytest.raises(MapJoinError, match='whole-map check refused'):
        place_recognition_alignments(bundles, tmp_path / 'work', _settings(tmp_path, replay, join))


def test_a_bundle_without_backend_input_is_refused(tmp_path: Path):
    replay, join = _fake_tools(tmp_path, 'sessions: []\n')
    plain = _bundle(tmp_path / 'plain', backend=False)
    with pytest.raises(MapJoinError, match='--editable'):
        place_recognition_alignments(
            [_bundle(tmp_path / 'a'), plain], tmp_path / 'work', _settings(tmp_path, replay, join))


def test_params_default_to_the_anchor_runs_graph_params(tmp_path: Path):
    replay, join = _fake_tools(tmp_path, textwrap.dedent(f"""\
        sessions:
          - session: 0
            joined: true
          - session: 1
            joined: true
            transform_to_session_0: {_row(0.0, 1.0, 0.0, 0.0)}
        """))
    anchor, other = _bundle(tmp_path / 'a'), _bundle(tmp_path / 'b')
    settings = JoinSettings(replay_script=replay, join_command=join)
    with pytest.raises(MapJoinError, match='graph_params.ros.yaml'):
        place_recognition_alignments([anchor, other], tmp_path / 'work', settings)
    (anchor / 'graph_params.ros.yaml').write_text('graph_based_slam:\n  ros__parameters: {}\n')
    _, summary = place_recognition_alignments([anchor, other], tmp_path / 'work2', settings)
    assert summary['params'] == str((anchor / 'graph_params.ros.yaml').resolve())


def test_merge_records_place_recognition_as_the_initial_transform_source(tmp_path: Path):
    anchor, session, rotation, translation = _fixture(tmp_path)
    yaw = math.degrees(math.atan2(rotation[1, 0], rotation[0, 0]))
    receipt = merge_map_sessions(
        source_dirs=[anchor, session],
        output_dir=tmp_path / 'project',
        options=_options(),
        initial_transforms={1: (*translation.tolist(), yaw)},
        dry_run=True,
        place_recognition={'method': 'graph_slam_multisession_join', 'sessions': []},
    )
    assert receipt['place_recognition']['method'] == 'graph_slam_multisession_join'
    assert receipt['alignments'][1]['initial_transform_source'] == 'place_recognition'
    assert np.allclose(receipt['alignments'][1]['translation'], translation, atol=0.05)


def test_cli_place_recognition_reports_errors_and_excludes_manual_transforms(
        tmp_path: Path, capsys):
    a, b = str(tmp_path / 'a'), str(tmp_path / 'b')
    assert merge_cli.main([a, b, '--place-recognition', '--output-dir', str(tmp_path / 'o')]) == 2
    assert 'map_bundle.yaml was not found' in capsys.readouterr().err
    assert merge_cli.main([
        a, b, '--place-recognition', '--params', 'p.yaml', '--initial-transform', '1:0,0,0,0',
        '--output-dir', str(tmp_path / 'o')]) == 2
    assert 'exclude each other' in capsys.readouterr().err
