from pathlib import Path
import sys

import pytest


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
