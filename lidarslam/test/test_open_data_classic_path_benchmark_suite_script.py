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

"""Regression tests for the classic-path benchmark suite wrapper."""

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'run_open_data_classic_path_benchmark_suite.sh'


def test_classic_path_suite_wrapper_runs_four_cases_and_renders_report():
    """The wrapper should exercise no-GNSS, GNSS-only, GNSS+odom, and GNSS+IMU cases."""
    script = SCRIPT_PATH.read_text(encoding='utf-8')

    assert 'run_open_data_applanix_velodyne_gnss_benchmark.sh' in script
    assert '--use-gnss false' in script
    assert '--use-odom-prior true' in script
    assert '--odom-frame-id odom' in script
    assert '--odom-prior-planar true' in script
    assert '--odom-prior-velocity-planar true' in script
    assert '--odom-prior-translation-only true' in script
    assert '--odom-prior-weight 1.0' in script
    assert '--robot-frame-id velodyne_front' in script
    assert '--use-imu true' in script
    assert '--robot-frame-id base_link' in script
    assert '--imu-frame-id base_link' in script
    assert 'generate_classic_path_report.py' in script
    assert 'classic_path_report.md' in script
    assert 'classic_path_report.json' in script
    assert 'classic_path_report.svg' in script
