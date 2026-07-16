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

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))

from competitive_candidate_provenance import source_tree_digest  # noqa: E402


def test_source_tree_digest_is_stable_and_content_sensitive(tmp_path):
    scripts = tmp_path / 'scripts'
    scripts.mkdir()
    (scripts / 'runner.py').write_text('first\n')
    first = source_tree_digest(tmp_path, ('scripts',))
    second = source_tree_digest(tmp_path, ('scripts',))
    assert first == second
    (scripts / 'runner.py').write_text('second\n')
    assert source_tree_digest(tmp_path, ('scripts',))['sha256'] != first['sha256']


def test_source_tree_digest_includes_untracked_style_files_and_skips_cache(tmp_path):
    scripts = tmp_path / 'scripts'
    scripts.mkdir()
    (scripts / 'new_file.py').write_text('candidate\n')
    cache = scripts / '__pycache__'
    cache.mkdir()
    (cache / 'new_file.pyc').write_bytes(b'generated')
    result = source_tree_digest(tmp_path, ('scripts',))
    assert result['file_count'] == 1


def test_source_tree_digest_changes_when_filename_changes(tmp_path):
    scripts = tmp_path / 'scripts'
    scripts.mkdir()
    original = scripts / 'a.py'
    original.write_text('same\n')
    first = source_tree_digest(tmp_path, ('scripts',))
    original.rename(scripts / 'b.py')
    assert source_tree_digest(tmp_path, ('scripts',))['sha256'] != first['sha256']
