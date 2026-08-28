#!/usr/bin/env python3
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

"""Focused tests for the release-facing external registration DSO gate."""

import argparse
import hashlib
import importlib.util
import os
from pathlib import Path
import subprocess

import pytest

REPO = Path(__file__).resolve().parents[2]
SCRIPT = REPO / 'scripts' / 'check_registration_plugin_dso.py'


def _fresh_install_path(relative_path):
    """Resolve an artifact only from the CTest-provided fresh install root."""
    install_text = os.environ.get('LIDARSLAM_TEST_INSTALL_PREFIX')
    if not install_text:
        return None
    install = Path(install_text).resolve()
    if install == (REPO / 'install').resolve():
        return None
    direct = install / relative_path
    if direct.is_file():
        return direct
    candidates = sorted(install.glob('*/{}'.format(relative_path)))
    if len(candidates) == 1 and candidates[0].is_file():
        return candidates[0]
    return None


INSTALL_TEXT = os.environ.get('LIDARSLAM_TEST_INSTALL_PREFIX')
INSTALL = Path(INSTALL_TEXT).resolve() if INSTALL_TEXT else None
FAKE_DSO = _fresh_install_path(
    'lidarslam_fake_registration_plugins/lib/' 'liblidarslam_fake_registration_plugins.so'
)
FAKE_XML = _fresh_install_path(
    'lidarslam_fake_registration_plugins/share/'
    'lidarslam_fake_registration_plugins/registration_plugins.xml'
)
HOST_DSO = _fresh_install_path('lidarslam_default_plugins/lib/liblidarslam_default_plugins.so')


def _load_gate():
    spec = importlib.util.spec_from_file_location('registration_dso_gate', str(SCRIPT))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


gate = _load_gate()


def _available_install():
    return (
        INSTALL is not None
        and INSTALL != (REPO / 'install').resolve()
        and FAKE_DSO is not None
        and FAKE_XML is not None
        and HOST_DSO is not None
        and FAKE_DSO.is_file()
        and FAKE_XML.is_file()
        and HOST_DSO.is_file()
    )


def _args(tmp_path, skip=True, receipt=None):
    assert _available_install()
    return argparse.Namespace(
        prefix=str(INSTALL),
        dso=str(FAKE_DSO),
        manifest=str(FAKE_XML),
        class_id='lidarslam_fake_registration_plugins/Identity',
        host_dso=[str(HOST_DSO)],
        allow_needed=['libclass_loader.so', 'libconsole_bridge.so.1.0'],
        ros_prefix='/opt/ros/jazzy',
        loader_include=None,
        interface_include=None,
        loader_library=None,
        receipt=str(receipt) if receipt else None,
        skip_load_smoke=skip,
        humble_verified=False,
    )


@pytest.mark.skipif(not _available_install(), reason='the local clean install fixture is absent')
def test_static_gate_passes_with_clean_installed_external_dso(tmp_path):
    result = gate.run_gate(_args(tmp_path))
    assert result['status'] == 'PASS_STATIC_ONLY'
    assert result['static_gate']['dso']['soname'] == FAKE_DSO.name
    assert result['static_gate']['dso']['rpath'] == []
    assert result['static_gate']['dso']['runpath'] == []
    ownership = result['static_gate']['interface_ownership']
    assert ownership['selected_factory_symbols']
    assert ownership['selected_vtable'].endswith('IdentityRegistration')
    assert ownership['selected_typeinfo'].endswith('IdentityRegistration')
    assert result['static_gate']['odr_collisions'] == []


@pytest.mark.skipif(not _available_install(), reason='the local clean install fixture is absent')
def test_loader_session_smoke_builds_cpp14_interface_and_runs_jazzy_loader(tmp_path):
    result = gate.run_gate(_args(tmp_path, skip=False))
    assert result['status'] == 'PASS'
    smoke = result['load_session_smoke']
    assert smoke['status'] == 'PASS'
    assert smoke['interface_standard'] == 'c++14'
    assert smoke['loader_standard'].startswith('c++17')
    assert len(smoke['interface_object_sha256']) == 64
    assert len(smoke['executable_sha256']) == 64


@pytest.mark.skipif(not _available_install(), reason='the local clean install fixture is absent')
def test_receipt_and_sidecar_are_immutable_and_self_consistent(tmp_path):
    receipt = tmp_path / 'odr.receipt.json'
    result = gate.run_gate(_args(tmp_path))
    path, digest = gate.seal_receipt(str(receipt), result)
    assert Path(path) == receipt
    assert int(oct(receipt.stat().st_mode & 0o777), 8) == 0o444
    sidecar = Path(str(receipt) + '.sha256')
    assert int(oct(sidecar.stat().st_mode & 0o777), 8) == 0o444
    assert sidecar.read_text(encoding='utf-8') == '{}  {}\n'.format(digest, receipt.name)
    assert hashlib.sha256(receipt.read_bytes()).hexdigest() == digest
    with pytest.raises(gate.GateError, match='overwrite'):
        gate.seal_receipt(str(receipt), result)
    link = tmp_path / 'receipt-link.json'
    link.symlink_to(receipt)
    with pytest.raises(gate.GateError, match='overwrite or symlink'):
        gate.seal_receipt(str(link), result)


def test_symlinked_dso_is_rejected_before_elf_inspection(tmp_path):
    target = tmp_path / 'libreal.so'
    target.write_bytes(b'not an ELF')
    link = tmp_path / 'liblink.so'
    link.symlink_to(target)
    with pytest.raises(gate.GateError, match='must not be a symlink'):
        gate.parse_elf(link)


@pytest.mark.skipif(not _available_install(), reason='the local clean install fixture is absent')
def test_dependency_allowlist_is_exact_and_fail_closed():
    with pytest.raises(gate.GateError, match='non-allowlisted DT_NEEDED dependency'):
        gate._check_static(
            INSTALL,
            FAKE_DSO,
            gate._parse_manifest(FAKE_XML, 'lidarslam_fake_registration_plugins/Identity'),
            [],
            [],
        )


@pytest.mark.skipif(not _available_install(), reason='the local clean install fixture is absent')
def test_manifest_library_must_bind_the_inspected_dso():
    manifest = gate._parse_manifest(FAKE_XML, 'lidarslam_fake_registration_plugins/Identity')
    manifest['library'] = 'another_external_library'
    with pytest.raises(gate.GateError, match='does not bind the inspected DSO'):
        gate._check_static(
            INSTALL, FAKE_DSO, manifest, [], ['libclass_loader.so', 'libconsole_bridge.so.1.0']
        )


def _compile_shared(tmp_path, name, source, *extra):
    source_path = tmp_path / (name + '.cpp')
    dso_path = tmp_path / ('lib' + name + '.so')
    source_path.write_text(source, encoding='utf-8')
    command = [
        'g++',
        '-std=c++14',
        '-shared',
        '-fPIC',
        '-Wl,-soname,{}'.format(dso_path.name),
        '-o',
        str(dso_path),
        str(source_path),
    ] + list(extra)
    completed = subprocess.run(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        universal_newlines=True,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr
    return dso_path


def test_rpath_and_unresolved_symbol_fixtures_are_rejected(tmp_path):
    rpath_dso = _compile_shared(
        tmp_path, 'rpath_fixture', 'extern "C" void marker() {}', '-Wl,-rpath,/tmp/not-allowed'
    )
    parsed = gate.parse_elf(rpath_dso)
    # GNU ld emits the new-dtags form as RUNPATH on this toolchain; either
    # dynamic tag is forbidden by the release gate.
    assert parsed['rpath'] + parsed['runpath'] == ['/tmp/not-allowed']
    with pytest.raises(gate.GateError, match='RPATH/RUNPATH'):
        gate._check_static(tmp_path, rpath_dso, {}, [], [])

    unresolved = _compile_shared(
        tmp_path,
        'unresolved_fixture',
        'extern "C" void missing_external_symbol();\n'
        'extern "C" void marker() { missing_external_symbol(); }',
        '-Wl,--allow-shlib-undefined',
    )
    parsed_unresolved = gate.parse_elf(unresolved)
    assert 'missing_external_symbol' in gate._undefined_unknown(parsed_unresolved)
    with pytest.raises(gate.GateError, match='unresolved non-allowlisted'):
        gate._check_static(tmp_path, unresolved, {}, [], [])


def test_manifest_rejects_wrong_base_and_duplicate_class_ids(tmp_path):
    wrong = tmp_path / 'wrong.xml'
    wrong.write_text(
        '<library path="plugin"><class name="pkg/One" type="pkg::One" '
        'base_class_type="other::Base"/></library>',
        encoding='utf-8',
    )
    with pytest.raises(gate.GateError, match='base class mismatch'):
        gate._parse_manifest(wrong, 'pkg/One')
    duplicate = tmp_path / 'duplicate.xml'
    duplicate.write_text(
        '<library path="plugin"><class name="pkg/One" type="pkg::One" '
        'base_class_type="{}"/><class name="pkg/One" type="pkg::Other" '
        'base_class_type="{}"/></library>'.format(gate.INTERFACE_BASE, gate.INTERFACE_BASE),
        encoding='utf-8',
    )
    with pytest.raises(gate.GateError, match='duplicate class IDs'):
        gate._parse_manifest(duplicate, 'pkg/One')


def test_interface_factory_rtti_and_vtable_ownership_is_required():
    if not _available_install():
        pytest.skip('the local clean install fixture is absent')
    elf = gate.parse_elf(FAKE_DSO)
    manifest = gate._parse_manifest(FAKE_XML, 'lidarslam_fake_registration_plugins/Identity')
    report = gate._interface_report(elf, manifest)
    assert any('registerPlugin<' in name for name in report['factory_symbols'])
    assert report['selected_typeinfo'] in {entry['name'] for entry in elf['defined']}
    assert report['selected_vtable'] in {entry['name'] for entry in elf['defined']}
    assert report['unexpected_interface_symbols'] == []


def test_odr_high_risk_symbol_intersection_is_detected():
    external = gate._odr_symbols(
        {
            'pclomp::OwnedRegistration::align()',
            'typeinfo for pclomp::OwnedRegistration',
            'vtable for pclomp::OwnedRegistration',
        }
    )
    host = gate._odr_symbols({'pclomp::OwnedRegistration::align()'})
    assert external == {'pclomp::OwnedRegistration::align()'}
    assert external & host == {'pclomp::OwnedRegistration::align()'}


def test_undefined_symbol_allowlist_does_not_accept_prefix_collisions():
    elf = {
        'undefined': [
            {'type': 'U', 'name': 'memcpy'},
            {'type': 'U', 'name': 'member_evil'},
            {'type': 'U', 'name': 'operator new(unsigned long)'},
        ]
    }
    assert gate._undefined_unknown(elf) == ['member_evil']


@pytest.mark.skipif(not _available_install(), reason='the local clean install fixture is absent')
def test_injected_duplicate_odr_and_interface_implementation_fail_closed(monkeypatch):
    manifest = gate._parse_manifest(FAKE_XML, 'lidarslam_fake_registration_plugins/Identity')
    original = gate.parse_elf
    host_path = str(HOST_DSO)
    plugin_path = str(FAKE_DSO)

    def injected(path):
        parsed = original(path)
        if str(path) == plugin_path:
            parsed['defined'] = list(parsed['defined']) + [
                {
                    'type': 'T',
                    'name': 'lidarslam::plugins::registration::RegistrationPlugin::align()',
                },
                {'type': 'T', 'name': 'pclomp::Injected::compute()'},
            ]
        if str(path) == host_path:
            parsed['defined'] = list(parsed['defined']) + [
                {'type': 'T', 'name': 'pclomp::Injected::compute()'},
            ]
        return parsed

    monkeypatch.setattr(gate, 'parse_elf', injected)
    with pytest.raises(gate.GateError, match='unexpected RegistrationPlugin'):
        gate._check_static(
            INSTALL,
            FAKE_DSO,
            manifest,
            [HOST_DSO],
            ['libclass_loader.so', 'libconsole_bridge.so.1.0'],
        )


@pytest.mark.skipif(not _available_install(), reason='the local clean install fixture is absent')
def test_injected_duplicate_host_plugin_odr_is_rejected(monkeypatch):
    manifest = gate._parse_manifest(FAKE_XML, 'lidarslam_fake_registration_plugins/Identity')
    original = gate.parse_elf
    host_path = str(HOST_DSO)
    plugin_path = str(FAKE_DSO)
    duplicate = 'pclomp::Injected::compute()'

    def injected(path):
        parsed = original(path)
        if str(path) in (plugin_path, host_path):
            parsed['defined'] = list(parsed['defined']) + [{'type': 'T', 'name': duplicate}]
        return parsed

    monkeypatch.setattr(gate, 'parse_elf', injected)
    with pytest.raises(gate.GateError, match='duplicate host/plugin ODR'):
        gate._check_static(
            INSTALL,
            FAKE_DSO,
            manifest,
            [HOST_DSO],
            ['libclass_loader.so', 'libconsole_bridge.so.1.0'],
        )
