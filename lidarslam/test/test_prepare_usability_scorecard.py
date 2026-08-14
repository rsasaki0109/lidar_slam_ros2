"""Tests for the fail-closed usability scorecard template generator."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'prepare_usability_scorecard.py'
SPEC = importlib.util.spec_from_file_location(
    'prepare_usability_scorecard_test',
    SCRIPT,
)
assert SPEC is not None and SPEC.loader is not None
PREPARE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PREPARE)


def _args(**overrides: str) -> list[str]:
    values = {
        '--product': 'lidarslam_ros2',
        '--version': '0.9.1',
        '--revision-kind': 'git-commit',
        '--revision': 'a' * 40,
        '--documentation-url': 'https://example.test/lidarslam',
        '--cohort-id': 'external-paired-operator-a',
        '--comparison-pair-id': 'paired-jazzy-machine-class-a',
        '--input-id': 'fixed-demo-v1',
        '--product-order': 'first',
        '--ros-distro': 'jazzy',
        '--os-family': 'ubuntu-24.04',
        '--architecture': 'x86_64',
        '--hardware-class': 'eight-core-32gib-x86_64',
        '--machine-fingerprint-sha256': 'b' * 64,
    }
    values.update(overrides)
    result = []
    for key, value in values.items():
        result.extend((key, value))
    return result


def test_stdout_template_is_schema_valid_and_fail_closed(capsys):
    """A new worksheet contains no invented measurements or success."""
    assert PREPARE.main(_args()) == 0
    record = json.loads(capsys.readouterr().out)

    PREPARE.validate_trial(record)
    assert record['product']['publicly_resolvable'] is False
    assert record['environment']['clean_start'] is False
    assert len(record['tasks']) == 6
    assert all(task['exact_commands'] == [] for task in record['tasks'])
    assert all(
        all(value is None for value in task['measurements'].values())
        for task in record['tasks']
    )
    assert all(task['outcome']['status'] == 'FAIL' for task in record['tasks'])
    assert all(
        task['outcome']['finding_codes'] == ['not-recorded']
        for task in record['tasks']
    )


def test_explicit_public_and_clean_flags_are_preserved(capsys):
    """The operator can record verified prerequisites without defaults."""
    args = _args(**{'--product-order': 'second'})
    # Boolean flags are intentionally appended only when requested.
    args.extend(('--publicly-resolvable', '--clean-start'))
    assert PREPARE.main(args) == 0
    record = json.loads(capsys.readouterr().out)
    assert record['product']['publicly_resolvable'] is True
    assert record['environment']['clean_start'] is True
    assert record['operator']['product_order'] == 'second'


def test_output_is_exclusive_and_does_not_overwrite(tmp_path, capsys):
    """A worksheet cannot silently replace an existing observation."""
    output = tmp_path / 'trial.json'
    args = _args(**{'--output': str(output)})
    assert PREPARE.main(args) == 0
    original = output.read_bytes()
    assert PREPARE.main(args) == 2
    assert output.read_bytes() == original
    assert 'File exists' in capsys.readouterr().err


@pytest.mark.parametrize(
    ('option', 'value', 'message'),
    [
        (
            '--machine-fingerprint-sha256',
            'not-a-hash',
            'machine-fingerprint-sha256',
        ),
        (
            '--hardware-class',
            'private machine',
            'hardware-class',
        ),
        (
            '--documentation-url',
            'file:///private/notes',
            'trial schema failed',
        ),
    ],
)
def test_unsafe_or_invalid_metadata_fails_closed(option, value, message,
                                                 capsys):
    """Templates reject private paths and malformed public identity fields."""
    args = _args(**{option: value})
    assert PREPARE.main(args) == 2
    assert message in capsys.readouterr().err
