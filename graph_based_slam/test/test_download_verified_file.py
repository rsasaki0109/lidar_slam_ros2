import hashlib
import importlib.util
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'download_verified_file.py'
SPEC = importlib.util.spec_from_file_location('download_verified_file', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_matches_requires_exact_size_before_hashing(tmp_path):
    path = tmp_path / 'archive.zip'
    path.write_bytes(b'partial')
    valid, record = MODULE.matches(path, 100, 'md5', 'unused')
    assert not valid
    assert record['reason'] == 'size_mismatch'
    assert record['size_bytes'] == 7


def test_existing_verified_file_avoids_network(tmp_path):
    path = tmp_path / 'archive.zip'
    payload = b'complete archive fixture'
    path.write_bytes(payload)
    result = subprocess.run([
        sys.executable, str(SCRIPT),
        '--url', 'https://invalid.example/should-not-be-opened',
        '--output', str(path),
        '--expected-size', str(len(payload)),
        '--expected-checksum', hashlib.md5(payload).hexdigest(),
        '--max-attempts', '1',
    ], check=True, text=True, capture_output=True)
    assert 'already_verified' in result.stdout
    assert path.read_bytes() == payload
