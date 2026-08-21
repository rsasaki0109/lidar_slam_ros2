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

"""Unit tests for the read-only M6a7 v3 evidence auditor."""

import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'audit_m6a7_v3_evidence', ROOT / 'scripts' / 'audit_m6a7_v3_evidence.py')
AUDIT = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(AUDIT)


def test_parse_host_exit_accepts_indented_gnu_time_line():
    assert AUDIT.parse_host_exit('\tExit status: 0\n') == 0
    assert AUDIT.parse_host_exit('  Exit status: 17\n') == 17


@pytest.mark.parametrize('report', [
    '', 'Exit status: 0\nExit status: 0\n',
    '\tExit status: not-a-number\n', 'Exit status: 0\nother: 1\nExit status: 1\n'])
def test_parse_host_exit_rejects_missing_duplicate_or_malformed(report):
    with pytest.raises(AUDIT.AuditError):
        AUDIT.parse_host_exit(report)


def test_parse_mode_handles_off_without_suffix_truncation():
    assert AUDIT.parse_mode('pair01_off', 'off\n') == 'off'
    assert AUDIT.parse_mode('pair02_on', ' on \n') == 'on'


@pytest.mark.parametrize(('name', 'value'), [
    ('pair01_off', 'on'), ('pair01_off', 'ff'),
    ('pair03_bad', 'off'), ('pair01_off', '')])
def test_parse_mode_rejects_mismatch_and_unknown(name, value):
    with pytest.raises(AUDIT.AuditError):
        AUDIT.parse_mode(name, value)


def test_schedule_rejects_missing_duplicate_and_extra_directories(tmp_path):
    expected = AUDIT.expected_schedule(2)
    for name in expected:
        (tmp_path / name).mkdir()
    assert AUDIT.validate_schedule_names(tmp_path, 2) == expected
    (tmp_path / 'pair02_on').rmdir()
    with pytest.raises(AUDIT.AuditError):
        AUDIT.validate_schedule_names(tmp_path, 2)
    (tmp_path / 'pair02_on').mkdir()
    (tmp_path / 'pair99_off').mkdir()
    with pytest.raises(AUDIT.AuditError):
        AUDIT.validate_schedule_names(tmp_path, 2)


def test_preregistration_binding_rejects_tampered_run_metadata(tmp_path):
    source = tmp_path / 'source.json'
    metadata = tmp_path / 'preregistration.json'
    marker = tmp_path / 'COMPLETE.marker'
    marker_sidecar = tmp_path / 'COMPLETE.marker.sha256'
    source.write_text('{"schema_version":3,"status":"preregistered_not_measured"}\n')
    metadata.write_bytes(source.read_bytes())
    marker.write_text('20 pairs complete\n')
    marker_sidecar.write_text(f'{AUDIT.sha256_file(marker)}  COMPLETE.marker\n')
    # The fixture does not use the production SHA; binding must still be
    # tested as fail-closed rather than silently accepting a different plan.
    with pytest.raises(AUDIT.AuditError, match='SHA binding'):
        AUDIT.preregistration_binding(tmp_path, source)
    metadata.write_text(
        '{"schema_version":3,"status":"preregistered_not_measured",'
        '"tampered":true}\n')
    with pytest.raises(AUDIT.AuditError, match='SHA binding'):
        AUDIT.preregistration_binding(tmp_path, source)
