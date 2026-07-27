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

"""Static contract tests for the curated installed product CLI."""

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
PACKAGE_ROOT = REPO_ROOT / 'lidarslam'
RUNTIME_MANIFEST = PACKAGE_ROOT / 'product-runtime-files.txt'


def _runtime_names() -> list[str]:
    return [
        line.strip()
        for line in RUNTIME_MANIFEST.read_text(encoding='utf-8').splitlines()
        if line.strip() and not line.lstrip().startswith('#')
    ]


def test_product_runtime_manifest_is_curated_and_complete():
    names = _runtime_names()

    assert names
    assert len(names) == len(set(names))
    assert 'lidarslam_cli.py' in names
    assert 'run_autoware_map_from_bag.py' in names
    assert 'verify_autoware_map.py' in names
    assert 'gaussian_splatting_train.py' not in names
    for name in names:
        assert Path(name).name == name
        assert (REPO_ROOT / 'scripts' / name).is_file(), name


def test_cmake_preserves_historical_node_and_installs_distinct_cli_names():
    cmake = (PACKAGE_ROOT / 'CMakeLists.txt').read_text(encoding='utf-8')

    assert 'product-runtime-files.txt' in cmake
    assert 'DESTINATION bin' in cmake
    assert 'RENAME lidarslam-map' in cmake
    assert 'DESTINATION lib/${PROJECT_NAME}' in cmake
    assert 'RENAME lidarslam-cli' in cmake
    assert 'install(TARGETS\n  lidarslam' in cmake


def test_primary_workflows_resolve_source_and_installed_runtime_layouts():
    workflow_names = (
        'run_rko_lio_graph_autoware_dogfood.sh',
        'run_open_data_gnss_smoke.sh',
        'run_open_data_applanix_velodyne_gnss_smoke.sh',
    )

    for name in workflow_names:
        script = (REPO_ROOT / 'scripts' / name).read_text(encoding='utf-8')

        assert 'PACKAGE_SHARE=' in script, name
        assert 'WORK_ROOT=' in script, name
        assert 'WORKSPACE_SETUP=' in script, name
        assert '${PACKAGE_SHARE}/param/' in script, name
        assert '${WORK_ROOT}/output/' in script, name
        assert '${REPO_ROOT}/scripts/' not in script, name

    dogfood = (REPO_ROOT / 'scripts' / workflow_names[0]).read_text(
        encoding='utf-8'
    )
    assert 'PATH_TO_TUM_SCRIPT="${SCRIPT_DIR}/path_to_tum.py"' in dogfood
    assert 'ODOM_TO_TUM_SCRIPT="${SCRIPT_DIR}/odom_to_tum.py"' in dogfood
    assert 'APE_FROM_TUM_SCRIPT="${SCRIPT_DIR}/ape_from_tum.py"' in dogfood
    assert '"$SCRIPT_DIR/simple_lanelet2_generator.py"' in dogfood

    for name in workflow_names[1:]:
        script = (REPO_ROOT / 'scripts' / name).read_text(encoding='utf-8')
        assert '"${SCRIPT_DIR}/verify_autoware_map.py"' in script, name
