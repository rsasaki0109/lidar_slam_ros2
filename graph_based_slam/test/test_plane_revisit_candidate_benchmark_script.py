# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Contract tests for the one-command plane-revisit benchmark runner."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts/run_plane_revisit_candidate_benchmark.sh'


def test_runner_freezes_off_on_runtime_geometry_and_trajectory_evidence():
    """Both arms must traverse the same evidence pipeline."""
    source = SCRIPT.read_text()

    assert 'for arm in off on' in source
    assert 'run_offline_determinism_check.sh' in source
    assert '/usr/bin/time' in source
    assert 'write_runtime_report.py' in source
    assert 'run_map_quality_check.sh' in source
    assert '--setup "${SETUP_FILE}"' in source
    assert 'run_cross_repo_slam_benchmark.py' in source
    assert '--raw-artifact "backend_bag=${BAG}"' in source
    assert '--raw-artifact "runner_executable=${RUNNER_EXECUTABLE}"' in source


def test_runner_isolates_candidate_switch_and_ros_domains():
    """Only ON receives candidate parameters and each arm gets unique domains."""
    source = SCRIPT.read_text()

    assert 'use_plane_revisit_constraints:="${ENABLED}"' in source
    assert 'DOMAIN_BASE=$((ROS_DOMAIN_BASE + RUNS))' in source
    assert 'if [[ "${arm}" == on ]]; then' in source
    assert 'CANDIDATE_PARAMS' in source


def test_runner_recommends_external_output_and_supports_dry_run():
    """The CLI exposes safe inspection and external-SSD-friendly output."""
    source = SCRIPT.read_text()

    assert '--output-dir <dir>' in source
    assert 'External-SSD output is recommended' in source
    assert '--dry-run' in source
    assert '--dense-raw-tum <tum>' in source
    assert 'RAW_TUM="${DENSE_RAW_TUM}"' in source
