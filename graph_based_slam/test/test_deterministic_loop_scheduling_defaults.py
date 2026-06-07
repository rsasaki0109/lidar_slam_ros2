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
Regression tests for the deterministic_loop_scheduling default (v0.4 D1).

deterministic_loop_scheduling is the opt-in flag that switches searchLoop from
the historical wall-clock-driven single-latest query to a deterministic
catch-up over every un-queried submap. It MUST default off so the published
benchmark behaviour stays the validated single-latest path; flipping it on is a
behaviour change that needs the 8-vs-16 reproducibility validation first.
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
def test_deterministic_loop_scheduling_present(path):
    params = _load_graph_params(path)
    assert 'deterministic_loop_scheduling' in params, (
        f'{path.name}: deterministic_loop_scheduling must be documented so '
        'operators can discover the opt-in flag'
    )


@pytest.mark.parametrize('path', PARAM_FILES, ids=lambda p: p.name)
def test_deterministic_loop_scheduling_defaults_off(path):
    params = _load_graph_params(path)
    assert params['deterministic_loop_scheduling'] is False, (
        f'{path.name}: deterministic_loop_scheduling must default off so the '
        'published single-latest benchmark behaviour stays unchanged until the '
        '8-vs-16 reproducibility validation lands'
    )
