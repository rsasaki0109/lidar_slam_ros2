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


"""Tests for the three-holdout, two-track suite gate."""

import copy
import importlib.util
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'evaluate_competitive_suite_gate.py'
PROFILE = ROOT / 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml'
SPEC = importlib.util.spec_from_file_location('suite_gate', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)
CONTRACT = yaml.safe_load(PROFILE.read_text())['competitive_slam_profile']
FROZEN_CONTRACT = copy.deepcopy(CONTRACT)
for _slot in FROZEN_CONTRACT['datasets']['holdout_slots'].values():
    _slot['status'] = 'frozen'


def _gates(passed=True):
    gates = []
    for slot in CONTRACT['datasets']['holdout_slots'].values():
        for track in MODULE.REQUIRED_TRACKS:
            gates.append({'sequence': slot['sequence'], 'track': track,
                          'pass': passed})
    return gates


def test_every_holdout_must_pass_both_tracks():
    result = MODULE.evaluate(_gates(), FROZEN_CONTRACT)
    assert result['pass'] is True
    assert len(result['complete_holdout_wins']) == 3
    assert result['expected_gate_count'] == 6


def test_missing_or_failed_track_prevents_suite_claim():
    gates = _gates()
    gates.pop()
    missing = MODULE.evaluate(gates, FROZEN_CONTRACT)
    assert missing['pass'] is False
    assert missing['missing_gates']

    gates = _gates()
    gates[0]['pass'] = False
    failed = MODULE.evaluate(gates, FROZEN_CONTRACT)
    assert failed['pass'] is False
    assert failed['failed_gates']


def test_pending_input_hashes_prevent_suite_claim():
    pending = copy.deepcopy(CONTRACT)
    pending['datasets']['holdout_slots']['holdout_1']['status'] = (
        'assigned_inputs_pending_hash')
    result = MODULE.evaluate(_gates(), pending)
    assert result['checks']['all_holdout_inputs_frozen'] is False
    assert result['pass'] is False


def test_formal_profile_has_all_holdout_inputs_frozen():
    result = MODULE.evaluate(_gates(), CONTRACT)
    assert result['checks']['all_holdout_inputs_frozen'] is True
