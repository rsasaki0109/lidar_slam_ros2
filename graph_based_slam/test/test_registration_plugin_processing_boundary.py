#!/usr/bin/env python3

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


"""Adversarial tests for the live registration processing boundary audit."""

import importlib.util
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / \
    'audit_registration_plugin_processing_boundary.py'


def _module():
    spec = importlib.util.spec_from_file_location(
        'registration_processing_boundary', SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_current_live_shells_have_no_raw_processing_bypass():
    module = _module()
    report = module.audit_repository(REPO_ROOT)
    assert report['status'] == 'PASS'
    assert all(item['status'] == 'PASS' for item in report['production_files'])


def test_raw_pcl_alignment_is_rejected():
    module = _module()
    result = module.audit_text(
        'void f() { registration_->align(output, guess); }\n',
        'synthetic.cpp',
    )
    assert result['status'] == 'FAIL'
    assert result['violations'][0]['kind'] == 'RAW_PROCESSING_CALL'


def test_raw_plugin_target_update_is_rejected():
    module = _module()
    result = module.audit_text(
        'bool f() { return registration_plugin_->setInputTarget(target, &error); }\n',
        'synthetic.cpp',
    )
    assert result['status'] == 'FAIL'
    assert any(
        item['kind'] == 'RAW_PROCESSING_CALL' for item in result['violations'])


def test_session_calls_are_the_positive_boundary():
    module = _module()
    result = module.audit_text(
        'session->align(request);\n'
        'registration_plugin_session_->setInputTarget(target, &error);\n',
        'synthetic.cpp',
    )
    assert result['status'] == 'PASS'


def test_session_raw_fallback_ternary_is_rejected():
    module = _module()
    result = module.audit_text(
        'return registration_plugin_session_ != nullptr ? '
        'registration_plugin_session_->align(request) : registration_plugin_->align(request);\n',
        'synthetic.cpp',
    )
    assert result['status'] == 'FAIL'
    assert any(item['kind'] == 'SESSION_RAW_FALLBACK_TERNARY'
               for item in result['violations'])


def test_standalone_allowlist_is_explicit_and_narrow():
    module = _module()
    result = module.audit_text(
        'registration_->align(output, guess);\n',
        'scanmatcher/src/small_gicp_odom_node.cpp',
        allowlisted=True,
    )
    assert result['status'] == 'ALLOWLISTED_STANDALONE'
    assert result['violations'] == []
    assert 'standalone' in result['reason']
