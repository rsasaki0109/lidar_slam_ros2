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


"""
Standalone CMake regression test for installed registration sidecars.

This intentionally builds only a tiny C++14 shared object in a pytest temp
directory.  It does not require ROS, pluginlib discovery, a bag, Docker, or a
runtime benchmark.  The production C++ parser/checker remains covered by the
loader tests; this test protects the install-time CMake path and post-link
hash binding.
"""

import hashlib
import json
from pathlib import Path
import shutil
import subprocess

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
CONTRACT_HELPER = (
    REPO_ROOT / 'lidarslam_registration_loader' / 'cmake' / 'registration_contract.cmake'
)


def _run(command, cwd):
    result = subprocess.run(
        command,
        cwd=str(cwd),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    assert result.returncode == 0, '{}\n{}'.format(' '.join(command), result.stdout)


@pytest.mark.skipif(shutil.which('cmake') is None, reason='cmake is unavailable')
def test_post_link_contract_sidecar_binds_installed_cxx14_dso(tmp_path):
    project = tmp_path / 'project'
    build = tmp_path / 'build'
    prefix = tmp_path / 'install'
    project.mkdir()
    helper = str(CONTRACT_HELPER).replace('\\', '/')
    (project / 'synthetic.cpp').write_text(
        'extern "C" int registration_contract_smoke() { return 14; }\n',
        encoding='utf-8',
    )
    (project / 'registration_plugins.xml').write_text(
        '<library path="synthetic_plugin">\n'
        '  <class name="synthetic/Identity" type="synthetic::Identity" '
        'base_class_type="lidarslam::plugins::registration::RegistrationPlugin"/>\n'
        '</library>\n',
        encoding='utf-8',
    )
    config = '{"schema":"lidarslam.registration.synthetic.v1","schema_version":1}\n'
    (project / 'registration_config.schema.json').write_text(config, encoding='utf-8')
    (project / 'CMakeLists.txt').write_text(
        'cmake_minimum_required(VERSION 3.8)\n'
        'project(registration_contract_smoke LANGUAGES CXX)\n'
        'set(CMAKE_CXX_STANDARD 14)\n'
        'set(CMAKE_CXX_STANDARD_REQUIRED ON)\n'
        'set(CMAKE_CXX_EXTENSIONS OFF)\n'
        'add_library(synthetic_plugin SHARED synthetic.cpp)\n'
        'file(SHA256 "${{CMAKE_CURRENT_SOURCE_DIR}}/registration_config.schema.json" '
        'CONFIG_SHA256)\n'
        'install(TARGETS synthetic_plugin LIBRARY DESTINATION lib)\n'
        'install(FILES registration_plugins.xml registration_config.schema.json '
        'DESTINATION share/${{PROJECT_NAME}})\n'
        'include("{}")\n'
        'lidarslam_install_registration_contract(synthetic_plugin registration_plugins.xml\n'
        '  "synthetic/Identity" 129 64 0 1 0\n'
        '  "lidarslam.registration.synthetic.v1" 1 "${{CONFIG_SHA256}}")\n'.format(helper),
        encoding='utf-8',
    )

    _run(
        [
            'cmake',
            '-S',
            str(project),
            '-B',
            str(build),
            '-DCMAKE_INSTALL_PREFIX={}'.format(prefix),
        ],
        project,
    )
    _run(['cmake', '--build', str(build), '-j2'], project)
    _run(['cmake', '--install', str(build)], project)

    xml = prefix / 'share' / 'registration_contract_smoke' / 'registration_plugins.xml'
    dso = prefix / 'lib' / 'libsynthetic_plugin.so'
    config_path = (
        prefix / 'share' / 'registration_contract_smoke' / 'registration_config.schema.json'
    )
    sidecar = (
        prefix
        / 'share'
        / 'registration_contract_smoke'
        / 'registration_plugins.xml.synthetic_Identity.contract.json'
    )
    assert xml.is_file() and not xml.is_symlink()
    assert dso.is_file() and not dso.is_symlink()
    assert config_path.is_file() and not config_path.is_symlink()
    assert sidecar.is_file() and not sidecar.is_symlink()

    manifest = json.loads(sidecar.read_text(encoding='utf-8'))
    assert manifest['schema'] == 'lidarslam-registration-contract-manifest-v1'
    assert manifest['schema_version'] == 1
    assert manifest['class_id'] == 'synthetic/Identity'
    assert manifest['plugin_xml_sha256'] == hashlib.sha256(xml.read_bytes()).hexdigest()
    assert manifest['dso_sha256'] == hashlib.sha256(dso.read_bytes()).hexdigest()
    assert manifest['config_schema_sha256'] == hashlib.sha256(config_path.read_bytes()).hexdigest()
    assert manifest['api_min_major'] == manifest['api_max_major'] == 1
    assert manifest['api_min_minor'] == manifest['api_max_minor'] == 0
    assert manifest['required_capability_bits'] == 129
    assert manifest['optional_capability_bits'] == 64
    assert manifest['target_policy'] == 0
    assert manifest['correspondence_metric'] == 1
    assert manifest['thread_model'] == 0
    assert manifest['cancellation_model'] == 0
    assert manifest['config_schema_id'] == 'lidarslam.registration.synthetic.v1'
    assert manifest['config_schema_version'] == 1
    assert manifest['abi_epoch'] == 'lidarslam.registration.cpp14.abi1'
    assert ';' in manifest['toolchain_tag']
    assert len(manifest['interface_contract_sha256']) == 64
    assert len(manifest['manifest_sha256']) == 64
