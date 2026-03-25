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

"""Regression tests for the dynamic-object-filter benchmark wrapper."""

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'run_dynamic_object_filter_benchmark.sh'


def test_dynamic_filter_benchmark_wrapper_runs_on_off_smoke_and_report():
    """The wrapper should run paired smoke tests and render a report."""
    script = SCRIPT_PATH.read_text(encoding='utf-8')

    assert 'run_open_data_gnss_smoke.sh' in script
    assert 'generate_dynamic_object_filter_report.py' in script
    assert 'write_dynamic_filter_param' in script
    assert '--filter-voxel-size FLOAT' in script
    assert '--filter-min-observations INT' in script
    assert '--filter-temporal-window INT' in script
    assert '--filter-max-range-from-sensor-m M' in script
    assert 'BASELINE_DIR="${SAVE_ROOT}/no_filter"' in script
    assert 'FILTERED_DIR="${SAVE_ROOT}/dynamic_filter"' in script
    assert '--baseline-dir "${BASELINE_DIR}"' in script
    assert '--filtered-dir "${FILTERED_DIR}"' in script
