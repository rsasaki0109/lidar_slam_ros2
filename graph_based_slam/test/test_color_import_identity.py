"""Legacy and package callers must share the canonical color implementation."""
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize('package_first', [False, True])
def test_color_import_identity(package_first):
    # Fresh interpreters cover both import orders without pytest's cached modules.
    code = f'''
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
'''
    subprocess.run([sys.executable, '-c', code], check=True, cwd=ROOT)
