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

"""Static contract tests for the pinned competitive benchmark recipes."""

import hashlib
from pathlib import Path
import re

import yaml


ROOT = Path(__file__).resolve().parents[2]
RECEIPT = ROOT / (
    'configs/slam_benchmark_profiles/'
    'competitive_execution_selection_2026-08.yaml')
RECIPE_NAMES = {
    'ours': 'docker/ours_competitive_benchmark.Dockerfile',
    'glim': 'docker/glim_cpu_benchmark.Dockerfile',
    'fast_livo2': 'docker/fast_livo2_benchmark.Dockerfile',
}
THREAD_ENV_NAMES = (
    'OMP_NUM_THREADS',
    'OPENBLAS_NUM_THREADS',
    'MKL_NUM_THREADS',
    'TBB_NUM_THREADS',
)


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _receipt():
    return yaml.safe_load(RECEIPT.read_text())


def test_all_owned_recipes_pin_base_and_cpu_contract():
    for recipe_name in RECIPE_NAMES.values():
        recipe = ROOT / recipe_name
        from_line = next(
            line for line in recipe.read_text().splitlines()
            if line.startswith('FROM '))
        assert re.fullmatch(r'FROM \S+@sha256:[0-9a-f]{64}', from_line)
        text = recipe.read_text()
        assert 'BENCHMARK_CPU_ONLY=1' in text
        for env_name in THREAD_ENV_NAMES:
            assert env_name in text


def test_receipt_binds_recipe_and_build_entrypoint_hashes():
    systems = _receipt()['systems']
    for system, recipe_name in RECIPE_NAMES.items():
        container = systems[system]['container']
        recipe = container['recipe']
        assert recipe['path'] == recipe_name
        assert recipe['sha256'] == _sha256(ROOT / recipe_name)
        assert re.fullmatch(r'sha256:[0-9a-f]{64}', recipe['base_digest'])
        entrypoint = ROOT / container['build_entrypoint_path']
        assert container['build_entrypoint_sha256'] == _sha256(entrypoint)
        if system == 'glim':
            assert container['status'] == 'ready'
            assert re.fullmatch(r'sha256:[0-9a-f]{64}', container['image_digest'])
        elif system in ('ours', 'fast_livo2'):
            assert container['status'] == 'ready'
            assert re.fullmatch(r'sha256:[0-9a-f]{64}', container['image_digest'])
        else:
            assert container['status'] in (
                'pending_build', 'pending_selection_or_build')
            assert container['image_digest'] is None
    ours_recipe = systems['ours']['container']['recipe']
    assert ours_recipe['source_repository'] == (
        'https://github.com/rsasaki0109/lidar_slam_ros2.git')
    assert ours_recipe['source_checkout'] == (
        'clone_at_revision_with_required_submodule_only')
    assert systems['glim']['toolchain']['not_applicable_fields'] == ['pcl']
    assert systems['glim']['toolchain']['status'] == 'ready'
    assert systems['glim']['toolchain']['scope'] == 'system_container'
    assert systems['glim']['toolchain']['observed']['pcl'] == 'not_applicable'
    assert systems['fast_livo2']['toolchain']['status'] == 'ready'
    assert systems['fast_livo2']['toolchain']['scope'] == 'system_container'
    assert systems['fast_livo2']['toolchain']['not_applicable_fields'] == []
    assert systems['ours']['toolchain']['status'] == 'ready'
    assert systems['ours']['toolchain']['scope'] == 'system_container'
    assert systems['ours']['toolchain']['not_applicable_fields'] == []


def test_fast_recipe_has_pinned_sources_and_no_local_base():
    text = (ROOT / RECIPE_NAMES['fast_livo2']).read_text()
    assert 'FAST_LIVO2_REVISION=0d2c0346107b75b59934975adec9a6eeeb913c64' in text
    assert 'RPG_VIKIT_REVISION=6c886c8e5d83997806e00294826d528cea3581dd' in text
    assert 'SOPHUS_REVISION=a621ff2e56c56c839a6c40418d42c3c254424b5c' in text
    assert 'FROM hdl_localization_noetic:local' not in text
    assert 'FROM fast-livo2-benchmark:noetic' not in text
    assert 'native_flags=disabled' in text


def test_glim_recipe_does_not_add_unused_pcl_to_toolchain():
    text = (ROOT / RECIPE_NAMES['glim']).read_text()
    assert 'libpcl-dev' not in text


def test_build_entrypoint_is_pull_free_and_revision_pinned():
    text = (ROOT / 'scripts/build_competitive_benchmark_images.sh').read_text()
    assert 'docker build --pull=false' in text
    assert 'OURS_REPOSITORY=https://github.com/rsasaki0109/lidar_slam_ros2.git' in text
    assert 'git archive' not in text
    assert 'cp "$ROOT/docker/ours_competitive_benchmark.Dockerfile"' in text
    assert '--build-arg "OURS_REPOSITORY=$OURS_REPOSITORY"' in text
    assert '--build-arg "OURS_REVISION=$OURS_REVISION"' in text
    assert 'lidarslam-ours:jazzy' in text
    assert 'glim-cpu-benchmark:competitive-v1' in text
    assert 'fast-livo2-benchmark:ros1-pinned' in text
    for revision in (
            '866f733677e92ecb08d67126e463da99dd140d46',
            'faa264a1bce1bda406f73457e35511f56cdc2eaa',
            '4a9e7a4cb084967c8525a1be529ad3ba2a118ae7',
            '0d2c0346107b75b59934975adec9a6eeeb913c64',
            '6c886c8e5d83997806e00294826d528cea3581dd',
            'a621ff2e56c56c839a6c40418d42c3c254424b5c',
    ):
        assert revision in text
    assert 'rosbag' not in text.lower()
    assert 'ground_truth' not in text.lower()


def test_ours_recipe_clones_revision_and_verifies_submodules_in_image():
    text = (ROOT / RECIPE_NAMES['ours']).read_text()
    assert 'ARG OURS_REPOSITORY=https://github.com/rsasaki0109/lidar_slam_ros2.git' in text
    assert 'git clone --no-checkout "$OURS_REPOSITORY"' in text
    assert 'checkout --detach "$OURS_REVISION"' in text
    assert 'submodule sync --recursive' in text
    assert 'submodule update --init --recursive -- Thirdparty/ndt_omp_ros2' in text
    assert 'Thirdparty/rko_lio' in text
    assert 'NDT_OMP_SUBMODULE_REVISION=497411279593eb261a3e3d04cdcbb4717af33ca3' in text
    assert 'RKO_LIO_GITLINK_REVISION=622b74778a41f753d47aa5918043755ebcbd4c75' in text
    assert 'BUILD_TESTING=OFF' in text
    assert 'colcon list --base-paths' in text
    assert 'rev-parse HEAD' in text
    assert 'submodule status --recursive' in text
    assert "awk '$1 ~ /^[-+U]/" in text
    assert 'COPY ' not in text


def test_fast_runner_and_entrypoint_use_owned_workspace():
    runner = (ROOT / 'scripts/run_fast_livo2_benchmark.py').read_text()
    entrypoint = (ROOT / 'scripts/fast_livo2_container_run.sh').read_text()
    assert 'fast-livo2-benchmark:ros1-pinned' in runner
    assert '/opt/fast_livo_ws/devel/setup.bash' in entrypoint
    assert 'hdl_localization_noetic:local' not in entrypoint
