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
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
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

"""Static and clean-install checks for the benchmark Python package surface."""

from __future__ import annotations

import ast
import importlib
from pathlib import Path
import pkgutil
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'scripts'
REQUIRED_MODULES = (
    'competitive_identity_hash',
    'check_competitive_rival_source_closure',
    'check_competitive_dataset_source_closure',
    'benchmark_phase_contract',
    'competitive_memory_gate',
    'verify_competitive_evidence_bundle',
    'competitive_holdout_authorization',
    'ntu_viral_acquisition',
    'authorize_ntu_viral_pin',
    'evaluate_competitive_sequence_gate',
    'evaluate_competitive_suite_gate',
    'compose_competitive_result',
    'run_competitive_gt_blind_benchmark',
    'run_fast_livo2_benchmark',
    'summarize_fast_livo2_benchmark',
    'capture_competitive_execution_identity',
    'check_competitive_execution_selection',
)

OPTIONAL_IMPORT_ROOTS = frozenset({
    'rosbag', 'rospy', 'rclpy', 'rosbag2_py', 'tf2_ros',
    'rosidl_runtime_py', 'std_srvs', 'nav_msgs', 'sensor_msgs_py',
})
SAFETY_MODULES = frozenset({
    'analyze_colored_point_cloud',
    'analyze_mid360_robot_loop_alignment',
    'analyze_mid360_robot_public_bag_segments',
    'analyze_mid360_robot_public_loop_candidates',
    'analyze_mid360_robot_public_loop_cloud',
    'analyze_mid360_robot_public_segment_map_cloud_alignment',
    'densify_corrected_trajectory',
    'evaluate_heldout_point_colors',
    'evaluate_lidar_camera_alignment',
    'fast_livo2_m6a10_feeder',
    'generate_readme_large_loop_map_figure',
    'generate_readme_loop_zoom_figure',
    'generate_readme_mid360_figures',
    'odom_to_tum',
    'path_to_tum',
    'refine_planar_map',
    'repose_posed_images',
    'run_m6a10_fixed10_v8',
    'run_m6a10_fixed10_v9',
    'run_m6a10_fixed10_v10',
    'tf_to_tum',
    'write_rko_lio_benchmark_metrics',
})


def _imported_modules(path: Path) -> set[str]:
    tree = ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
    result: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            result.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            result.add(node.module)
    return result


def test_all_intra_tool_imports_use_installed_package_name():
    """No production script may depend on the ambiguous ``scripts`` package."""
    offenders = []
    for path in sorted(SCRIPTS.glob('*.py')):
        imports = _imported_modules(path)
        if any(
                name == 'scripts' or name.startswith('scripts.')
                for name in imports):
            offenders.append(path.name)
    assert not offenders


def _top_level_imports(tree: ast.AST):
    """Yield imports outside function/class bodies."""
    class Visitor(ast.NodeVisitor):
        def __init__(self):
            self.depth = 0
            self.items = []

        def visit_FunctionDef(self, node):
            self.depth += 1
            self.generic_visit(node)
            self.depth -= 1

        visit_AsyncFunctionDef = visit_FunctionDef
        visit_ClassDef = visit_FunctionDef

        def visit_Import(self, node):
            if self.depth == 0:
                self.items.append(node)

        def visit_ImportFrom(self, node):
            if self.depth == 0:
                self.items.append(node)

    visitor = Visitor()
    visitor.visit(tree)
    return visitor.items


def test_installed_surface_safety_guards_cover_historical_failures():
    """The modules formerly failing clean imports have no unsafe fallbacks."""
    local = {path.stem for path in SCRIPTS.glob('*.py')}
    local.update(
        path.stem for path in (ROOT / 'tools' / 'gaussian_splatting').glob('*.py'))
    bare = []
    optional = []
    path_hacks = []
    for name in sorted(SAFETY_MODULES):
        path = SCRIPTS / f'{name}.py'
        assert path.is_file(), f'missing guarded source: {path}'
        tree = ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
        for node in _top_level_imports(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    root = alias.name.split('.')[0]
                    if root in local and root != 'lidarslam_benchmark_tools':
                        bare.append((name, node.lineno, alias.name))
                    if root in OPTIONAL_IMPORT_ROOTS:
                        optional.append((name, node.lineno, alias.name))
            elif isinstance(node, ast.ImportFrom) and node.module:
                root = node.module.split('.')[0]
                if node.level == 0 and root in local and root != 'lidarslam_benchmark_tools':
                    bare.append((name, node.lineno, node.module))
                if node.level == 0 and root in OPTIONAL_IMPORT_ROOTS:
                    optional.append((name, node.lineno, node.module))
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
                continue
            owner = node.func.value
            if (isinstance(owner, ast.Attribute)
                    and isinstance(owner.value, ast.Name)
                    and owner.value.id == 'sys'
                    and owner.attr == 'path'
                    and node.func.attr in {'insert', 'append'}):
                path_hacks.append((name, node.lineno))
    assert not bare
    assert not optional
    assert not path_hacks


def test_required_modules_are_present_in_canonical_source_tree():
    """The projection has one source file for every required module."""
    missing = [name for name in REQUIRED_MODULES
               if not (SCRIPTS / f'{name}.py').is_file()]
    assert not missing


def test_clean_install_guard_rejects_repository_fallback():
    """The package guard is observable in a clean install."""
    package = importlib.import_module('lidarslam_benchmark_tools')
    if package.is_source_surface():
        with pytest.raises(RuntimeError, match='repository scripts'):
            package.require_installed_surface()
        return
    root = package.require_installed_surface()
    assert (root / 'configs').is_dir()


def test_resource_resolver_rejects_ambiguous_or_external_paths(tmp_path, monkeypatch):
    """Resource lookup must stay inside the selected source/package root."""
    package = importlib.import_module('lidarslam_benchmark_tools')
    source_root = tmp_path / 'scripts'
    source_root.mkdir()
    (source_root / 'helper.py').write_text('value = 1\n', encoding='utf-8')
    monkeypatch.setattr(package, '_SOURCE_SURFACE', True)
    monkeypatch.setattr(package, '_SOURCE_SCRIPTS_DIR', source_root)
    assert package.resolve_benchmark_resource('scripts/helper.py') == (
        source_root / 'helper.py')
    for resource in (
            '/tmp/helper.py', '../helper.py', 'scripts/../helper.py',
            'scripts//helper.py', 'scripts/./helper.py', 'scripts/helper.py/',
            'scripts/missing.py'):
        with pytest.raises(RuntimeError):
            package.resolve_benchmark_resource(resource)
    (source_root / 'external.py').symlink_to(tmp_path / 'outside.py')
    with pytest.raises(RuntimeError):
        package.resolve_benchmark_resource('scripts/external.py')


def test_required_modules_import_from_package_surface():
    """Import the benchmark/evidence dependency closure, not source files."""
    for name in REQUIRED_MODULES:
        module = importlib.import_module(f'lidarslam_benchmark_tools.{name}')
        assert module.__name__ == f'lidarslam_benchmark_tools.{name}'


def test_every_installed_module_imports_without_repository_fallback():
    """Import every projected module, including the two shared subpackages."""
    package = importlib.import_module('lidarslam_benchmark_tools')
    failures = []

    def visit(package_name, package_path):
        for info in pkgutil.iter_modules(package_path):
            qualified = f'{package_name}.{info.name}'
            if info.ispkg:
                child = importlib.import_module(qualified)
                visit(qualified, child.__path__)
                continue
            try:
                importlib.import_module(qualified)
            except Exception as error:  # report all modules in one assertion
                failures.append((qualified, type(error).__name__, str(error)))

    visit(package.__name__, package.__path__)
    assert not failures, '\n'.join(' | '.join(item) for item in failures)


@pytest.mark.parametrize(
    ('module', 'arguments', 'message'),
    (
        ('odom_to_tum', ('--output', 'odom.tum'), 'requires ROS2 rclpy'),
        ('tf_to_tum', ('--output', 'tf.tum'), 'requires ROS2 rclpy'),
        ('fast_livo2_m6a10_feeder',
         ('--bag', 'missing.bag', '--output', 'feeder'),
         'requires ROS1 rosbag'),
    ),
)
def test_optional_dependency_errors_are_explicit_in_clean_process(
        tmp_path, module, arguments, message):
    """Execution-only ROS dependencies fail clearly, not during import."""
    command = [sys.executable, '-m', f'lidarslam_benchmark_tools.{module}']
    command.extend(
        str(tmp_path / value) if value.endswith(('.tum', 'feeder', '.bag'))
        else value for value in arguments)
    environment = {
        'PATH': '/usr/bin:/bin',
        'PYTHONPATH': str(ROOT),
    }
    result = subprocess.run(
        command,
        cwd=ROOT,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode != 0
    assert message in (result.stdout + result.stderr)
