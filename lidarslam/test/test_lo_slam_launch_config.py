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

"""Regression tests for LO (scanmatcher-only) launch defaults."""

from __future__ import annotations

import ast
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
LO_LAUNCH = REPO_ROOT / 'lidarslam' / 'launch' / 'lo_slam.launch.py'
LO_YAML = REPO_ROOT / 'lidarslam' / 'param' / 'lidarslam_lo.yaml'


def test_lo_launch_defaults_to_lidarslam_lo_yaml():
    module = ast.parse(LO_LAUNCH.read_text(encoding='utf-8'))
    strings = [
        n.value
        for n in ast.walk(module)
        if isinstance(n, ast.Constant) and isinstance(n.value, str)
    ]
    assert any(s.endswith('lidarslam_lo.yaml') for s in strings)
    assert LO_YAML.is_file()


def test_lo_yaml_disables_scanmatcher_imu():
    text = LO_YAML.read_text(encoding='utf-8')
    assert 'use_imu: false' in text
    assert 'use_imu_preintegration: false' in text
