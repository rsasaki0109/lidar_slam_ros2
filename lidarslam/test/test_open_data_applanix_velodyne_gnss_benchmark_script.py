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

"""Regression tests for the open-data Applanix GNSS benchmark wrapper."""

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'run_open_data_applanix_velodyne_gnss_benchmark.sh'


def test_open_data_benchmark_script_writes_reference_and_metrics():
    """The benchmark wrapper should extract a reference and emit metrics."""
    script = SCRIPT_PATH.read_text(encoding='utf-8')

    assert 'extract_applanix_gsof49_reference.py' in script
    assert 'write_aligned_trajectory_metrics.py' in script
    assert '--reference-kind "cross_validation"' in script
    assert '--reference-source "applanix_gsof49_reference"' in script
    assert '--topic /path' in script
    assert '--topic /modified_path' in script


def test_open_data_benchmark_script_supports_gnss_toggle_and_packet_conversion():
    """The benchmark wrapper should be able to run with or without GNSS."""
    script = SCRIPT_PATH.read_text(encoding='utf-8')

    assert '--use-gnss BOOL' in script
    assert 'create_main_param' in script
    assert 'terminate_pid()' in script
    assert 'velodyne_transform_node' in script
    assert '--qos-profile-overrides-path "${QOS_FILE}"' in script
    assert 'POINTS_TOPIC="/open_data/velodyne_points"' in script
    assert 'if [[ "${USE_GNSS,,}" == "true" ]]; then' in script
