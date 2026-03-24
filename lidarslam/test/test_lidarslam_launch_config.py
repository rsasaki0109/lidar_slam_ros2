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

"""Regression tests for the classic lidarslam launch defaults."""

from __future__ import annotations

import ast
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
LAUNCH_PATH = REPO_ROOT / 'lidarslam' / 'launch' / 'lidarslam.launch.py'


def _parse_launch_ast() -> ast.Module:
    return ast.parse(LAUNCH_PATH.read_text(encoding='utf-8'))


def test_graph_backend_receives_map_save_dir_override():
    """Classic launch should route save_dir into graph_based_slam map_save_dir."""
    module = _parse_launch_ast()

    node_calls = [node for node in ast.walk(module) if isinstance(node, ast.Call)]
    graph_node = None
    for call in node_calls:
        if not isinstance(call.func, ast.Name) or call.func.id != 'Node':
            continue
        package_kw = next((kw for kw in call.keywords if kw.arg == 'package'), None)
        if package_kw is None:
            continue
        if not isinstance(package_kw.value, ast.Constant):
            continue
        if package_kw.value.value == 'graph_based_slam':
            graph_node = call
            break

    assert graph_node is not None
    parameters_kw = next(kw for kw in graph_node.keywords if kw.arg == 'parameters')
    dict_nodes = [value for value in parameters_kw.value.elts if isinstance(value, ast.Dict)]
    assert dict_nodes
    launch_dict = dict_nodes[0]

    found = False
    for key, value in zip(launch_dict.keys, launch_dict.values):
        if not isinstance(key, ast.Constant) or key.value != 'map_save_dir':
            continue
        if not isinstance(value, ast.Call):
            continue
        if not isinstance(value.func, ast.Name) or value.func.id != 'LaunchConfiguration':
            continue
        if not value.args or not isinstance(value.args[0], ast.Constant):
            continue
        if value.args[0].value == 'save_dir':
            found = True
            break

    assert found


def test_graph_backend_receives_gnss_topic_override():
    """Classic launch should route gnss_topic into graph_based_slam."""
    module = _parse_launch_ast()

    node_calls = [node for node in ast.walk(module) if isinstance(node, ast.Call)]
    graph_node = None
    for call in node_calls:
        if not isinstance(call.func, ast.Name) or call.func.id != 'Node':
            continue
        package_kw = next((kw for kw in call.keywords if kw.arg == 'package'), None)
        if package_kw is None:
            continue
        if not isinstance(package_kw.value, ast.Constant):
            continue
        if package_kw.value.value == 'graph_based_slam':
            graph_node = call
            break

    assert graph_node is not None
    parameters_kw = next(kw for kw in graph_node.keywords if kw.arg == 'parameters')
    dict_nodes = [value for value in parameters_kw.value.elts if isinstance(value, ast.Dict)]
    assert dict_nodes
    launch_dict = dict_nodes[0]

    found = False
    for key, value in zip(launch_dict.keys, launch_dict.values):
        if not isinstance(key, ast.Constant) or key.value != 'gnss_topic':
            continue
        if not isinstance(value, ast.Call):
            continue
        if not isinstance(value.func, ast.Name) or value.func.id != 'LaunchConfiguration':
            continue
        if not value.args or not isinstance(value.args[0], ast.Constant):
            continue
        if value.args[0].value == 'gnss_topic':
            found = True
            break

    assert found


def test_launch_declares_gnss_topic_argument():
    """Classic launch should expose a gnss_topic argument."""
    module = _parse_launch_ast()

    node_calls = [node for node in ast.walk(module) if isinstance(node, ast.Call)]
    found = False
    for call in node_calls:
        if not isinstance(call.func, ast.Name) or call.func.id != 'DeclareLaunchArgument':
            continue
        if not call.args or not isinstance(call.args[0], ast.Constant):
            continue
        if call.args[0].value != 'gnss_topic':
            continue
        default_kw = next((kw for kw in call.keywords if kw.arg == 'default_value'), None)
        assert default_kw is not None
        assert isinstance(default_kw.value, ast.Constant)
        assert default_kw.value.value == '/gnss/fix'
        found = True
        break

    assert found
