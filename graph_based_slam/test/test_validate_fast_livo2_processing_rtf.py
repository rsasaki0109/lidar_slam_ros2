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


"""Tests for accelerated-replay processing RTF validation."""

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'validate_fast_livo2_processing_rtf.py'
SPEC = importlib.util.spec_from_file_location('validate_fast_rtf', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _scored(ape=0.05, samples=(10, 10, 10), valid=3):
    return {
        'valid_repetitions': valid,
        'aggregate': {'ape_rmse_median_m': ape},
        'runs': [{'trajectory_samples': count, 'trajectory_complete': True,
                  'process_exit_status': 0} for count in samples],
    }


def _reports(replay_rtf=0.95, rate=1.05):
    return [
        {'provenance': {'rate': rate}, 'runtime': {
            'bag_duration_seconds': 100.0,
            'replay_wall_realtime_factor': replay_rtf}}
        for _ in range(3)]


def test_valid_bound_requires_count_accuracy_and_drain_budget():
    result = MODULE.validate(_scored(), _scored(0.0504), _reports(),
                             5.0, 3, 1.0, 1.0)
    assert result['valid_processing_rtf_evidence'] is True
    assert result['processing_rtf_upper_bound_median'] == 1.0


def test_pose_drop_or_accuracy_drift_invalidates_accelerated_probe():
    dropped = MODULE.validate(_scored(), _scored(samples=(10, 9, 10)),
                              _reports(), 5.0, 3, 1.0, 1.0)
    drifted = MODULE.validate(_scored(), _scored(0.051), _reports(),
                              5.0, 3, 1.0, 1.0)
    assert dropped['valid_processing_rtf_evidence'] is False
    assert dropped['checks']['trajectory_sample_counts_equal'] is False
    assert drifted['valid_processing_rtf_evidence'] is False
    assert drifted['checks']['ape_drift_within_tolerance'] is False
