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

"""CLI contract for the CPU-only AIST RGB map benchmark."""

from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts/run_aist_ouster_rgb_map_benchmark.sh'


def test_help_exposes_public_inputs_and_cpu_map_stages():
    result = subprocess.run(
        ['bash', str(SCRIPT), '--help'], cwd=ROOT,
        capture_output=True, text=True, check=False)

    assert result.returncode == 0
    assert '--bag DIR' in result.stdout
    assert '--extrinsic FILE' in result.stdout
    assert '--localization-zoo DIR' in result.stdout
    assert 'held-out RGB' in result.stdout


def test_runner_freezes_rigid_scan_and_real_rgb_reports():
    source = SCRIPT.read_text()

    assert '--no-deskew' in source
    assert 'analyze_colored_point_cloud.py' in source
    assert 'evaluate_heldout_point_colors.py' in source
    assert 'run_map_quality_check.sh' in source
    assert 'write_runtime_report.py' in source
    assert '--bag-metadata "${BAG}/metadata.yaml"' in source
    assert 'runtime.json' in source
    assert 'run_cross_repo_slam_benchmark.py' in source
    assert '--dataset aist_ouster_rgb' in source
    assert '--alignment-report "${MAP_DIR}/heldout_point_colors.json"' in source
    assert '--raw-artifact "aist_rosbag=${BAG}"' in source
    assert '--raw-artifact "official_extrinsic=${EXTRINSIC}"' in source
    assert '--raw-artifact "coloured_map=${MAP_DIR}/colored_map.ply"' in source
