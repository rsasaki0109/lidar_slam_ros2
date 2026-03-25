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

"""Regression tests for packet IMU deskew validation matrix wrapper."""

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = (
    REPO_ROOT / 'scripts' / 'run_open_data_packet_imu_deskew_validation_matrix.sh'
)


def test_packet_imu_deskew_matrix_script_runs_default_open_data_cases():
    script = SCRIPT_PATH.read_text(encoding='utf-8')

    assert 'all-sensors-bag1_compressed' in script
    assert 'all-sensors-bag6_compressed' in script
    assert '/sensing/lidar/front/velodyne_packets' in script
    assert '/sensing/imu/imu_data' in script
    assert 'generate_packet_imu_deskew_validation_report.py' in script
    assert 'run_open_data_applanix_velodyne_gnss_benchmark.sh' in script
    assert '--use-gnss false' in script
    assert '--use-imu false' in script
    assert '--use-imu true' in script
    assert '--no-default-cases' in script
    assert '--case SPEC' in script
    assert '--benchmark-rate FLOAT' in script
    assert 'BENCHMARK_RATE="1.0"' in script
    assert '--rate "${BENCHMARK_RATE}"' in script
    assert '--ros-domain-id-base N' in script
    assert '--ros-domain-id "${no_imu_domain_id}"' in script
    assert '--ros-domain-id "${imu_domain_id}"' in script
