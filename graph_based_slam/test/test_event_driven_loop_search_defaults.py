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

"""
Regression tests for the event_driven_loop_search default (v0.6 Phase 3).

event_driven_loop_search runs loop search once per submap arrival in arrival
order, making the (query, db) pair set a pure function of the input stream
instead of wall-clock timer batching (the v0.4 D1 nondeterminism root cause).
It defaults ON since the v0.6 Phase 3 flip; the shipped parameter files must
document the flag and keep it on so the published behaviour matches the
determinism evidence (offline 3-run byte-identical loop edges on the MID-360
and NTU substrates). The retired deterministic_loop_scheduling parameter must
not reappear.
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]

PARAM_FILES = [
    REPO_ROOT / 'graph_based_slam' / 'param' / 'graphbasedslam.yaml',
    REPO_ROOT / 'graph_based_slam' / 'param' / 'graphbasedslam_indoor.yaml',
]


def _load_graph_params(path: Path) -> dict:
    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    assert 'graph_based_slam' in data, f'{path} missing graph_based_slam node'
    params = data['graph_based_slam'].get('ros__parameters')
    assert params is not None, f'{path} missing ros__parameters'
    return params


@pytest.mark.parametrize('path', PARAM_FILES, ids=lambda p: p.name)
def test_event_driven_loop_search_present(path):
    params = _load_graph_params(path)
    assert 'event_driven_loop_search' in params, (
        f'{path.name}: event_driven_loop_search must be documented so '
        'operators can discover the legacy-timer escape hatch'
    )


@pytest.mark.parametrize('path', PARAM_FILES, ids=lambda p: p.name)
def test_event_driven_loop_search_defaults_on(path):
    params = _load_graph_params(path)
    assert params['event_driven_loop_search'] is True, (
        f'{path.name}: event_driven_loop_search must stay on so the shipped '
        'behaviour matches the v0.6 determinism evidence (3-run byte-identical '
        'loop edges); the legacy timer path is an opt-out, not the default'
    )


@pytest.mark.parametrize('path', PARAM_FILES, ids=lambda p: p.name)
def test_deterministic_loop_scheduling_is_retired(path):
    params = _load_graph_params(path)
    assert 'deterministic_loop_scheduling' not in params, (
        f'{path.name}: deterministic_loop_scheduling was retired in v0.6 '
        'Phase 3 (subsumed by event_driven_loop_search) and must not reappear'
    )
