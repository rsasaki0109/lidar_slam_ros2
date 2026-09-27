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

"""Legacy and package callers must share the canonical color implementation."""
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize('package_first', [False, True])
def test_color_import_identity(package_first):
    # Fresh interpreters cover both import orders without pytest's cached modules.
    code = f"""
import sys
sys.path[:0] = [{str(ROOT)!r}, {str(ROOT / 'tools/gaussian_splatting')!r}]
if {package_first!r}:
    from lidarslam_benchmark_tools.gaussian_splatting import pointcloud_io as package
    import pointcloud_io as legacy
else:
    import pointcloud_io as legacy
    from lidarslam_benchmark_tools.gaussian_splatting import pointcloud_io as package
from lidarslam_benchmark_tools.gaussian_splatting import build_lidar_init
assert legacy is package
assert build_lidar_init._colorize.__globals__['pcio'] is package
assert legacy.colorize_by_projection_robust is package.colorize_by_projection_robust
"""
    subprocess.run([sys.executable, '-c', code], check=True, cwd=ROOT)
