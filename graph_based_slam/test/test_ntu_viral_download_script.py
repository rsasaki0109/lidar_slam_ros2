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

"""CLI regression tests for the NTU VIRAL demo dataset downloader."""

from __future__ import annotations

from pathlib import Path
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[2]
DOWNLOAD_SCRIPT = REPO_ROOT / 'scripts' / 'download_ntu_viral_tnp01.sh'


def _run_download(*args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ['bash', str(DOWNLOAD_SCRIPT), *args],
        check=False,
        capture_output=True,
        text=True,
    )


def _combined_output(result: subprocess.CompletedProcess[str]) -> str:
    return result.stdout + result.stderr


def test_download_help_exits_successfully():
    result = _run_download('--help')
    output = _combined_output(result)

    assert result.returncode == 0
    assert 'download_ntu_viral_tnp01.sh' in output
    assert '--no-restamp' in output


def test_download_rejects_missing_dest_value_before_dependency_checks():
    result = _run_download('--dest', '--keep-zip')

    assert result.returncode == 2
    assert 'error: option requires a value: --dest' in result.stderr
    assert 'shift' not in result.stderr
    assert 'required command not found' not in result.stderr


def test_download_rejects_unknown_option_with_help_hint():
    result = _run_download('--bogus')

    assert result.returncode == 2
    assert 'error: unknown option: --bogus' in result.stderr
    assert 'download_ntu_viral_tnp01.sh --help' in result.stderr
    assert 'downloading zip' not in _combined_output(result)


def test_download_rejects_destination_file_before_download(tmp_path: Path):
    dest_file = tmp_path / 'not_a_directory'
    dest_file.write_text('', encoding='utf-8')

    result = _run_download(
        '--dest',
        str(dest_file),
        '--no-convert',
        '--no-restamp',
    )

    assert result.returncode == 2
    assert 'error: destination path is not a directory:' in result.stderr
    assert 'downloading zip' not in _combined_output(result)


def test_download_rejects_no_convert_restamp_without_existing_rosbag2(
    tmp_path: Path,
):
    result = _run_download('--dest', str(tmp_path / 'dataset'), '--no-convert')

    assert result.returncode == 2
    assert '--no-convert requires existing rosbag2 metadata' in result.stderr
    assert 'rosbags-convert' not in result.stderr
    assert 'downloading zip' not in _combined_output(result)
