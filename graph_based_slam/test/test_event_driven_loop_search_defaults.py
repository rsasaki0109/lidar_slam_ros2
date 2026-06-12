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
Regression tests for the loop-search scheduling parameter retirements.

Event-driven loop search (one query per submap arrival, in arrival order)
became the default in v0.6 Phase 3 and the ONLY behaviour in v0.7 Phase 0:
the legacy wall-clock timer path was removed after its one-release
deprecation window, together with its parameters. The (query, db) pair set
is a pure function of the input stream — the property behind the offline
3-run byte-identical determinism gates (MID-360 and NTU substrates).

These tests pin the retirements so the removed knobs do not silently
reappear in the shipped parameter files or in the component source:

- ``deterministic_loop_scheduling`` — retired in v0.6 Phase 3 (subsumed).
- ``event_driven_loop_search`` — retired in v0.7 Phase 0 (now the only
  semantics, no opt-out).
- ``loop_detection_period`` — retired in v0.7 Phase 0 (the wall timer it
  paced no longer exists).
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]

RETIRED_PARAMS = (
    'deterministic_loop_scheduling',
    'event_driven_loop_search',
    'loop_detection_period',
)

PARAM_FILES = sorted(
    list((REPO_ROOT / 'graph_based_slam' / 'param').glob('*.yaml')) +
    list((REPO_ROOT / 'lidarslam' / 'param').glob('*.yaml'))
)

COMPONENT_CPP = (
    REPO_ROOT / 'graph_based_slam' / 'src' / 'graph_based_slam_component.cpp'
)


def _flatten_params(node):
    """Yield every parameter key in a ROS 2 params yaml, at any nesting."""
    if isinstance(node, dict):
        for key, value in node.items():
            yield key
            yield from _flatten_params(value)


@pytest.mark.parametrize('path', PARAM_FILES, ids=lambda p: p.name)
def test_param_files_exist(path):
    assert path.is_file()


@pytest.mark.parametrize('path', PARAM_FILES, ids=lambda p: p.name)
@pytest.mark.parametrize('param', RETIRED_PARAMS)
def test_retired_params_absent_from_param_files(path, param):
    keys = set(_flatten_params(yaml.safe_load(path.read_text(encoding='utf-8'))))
    assert param not in keys, (
        f'{path.name}: {param} was retired (deterministic_loop_scheduling in '
        'v0.6 Phase 3, the legacy wall-clock timer path and its knobs in v0.7 '
        'Phase 0) and must not reappear in shipped parameter files'
    )


@pytest.mark.parametrize('param', RETIRED_PARAMS)
def test_retired_params_not_declared_by_component(param):
    # Historical comments may mention the retired names; only an actual
    # declare_parameter() would resurrect the knob.
    source = COMPONENT_CPP.read_text(encoding='utf-8')
    assert f'declare_parameter("{param}"' not in source, (
        f'{param} must stay removed — event-driven loop search is the only '
        'scheduling semantics since v0.7 Phase 0 and the wall timer no '
        'longer exists'
    )
