# Copyright 2026 Sasaki
# All rights reserved.

"""Tests for deterministic NTU-VIRAL timestamp normalization."""

import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'normalize_ntu', ROOT / 'scripts' / 'normalize_ntu_viral_rosbag.py')
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_regularized_stamps_preserve_endpoints_and_are_deterministic():
    assert MODULE.regularized_stamps(100, 110, 4) == [100, 103, 107, 110]
    assert MODULE.regularized_stamps(100, 110, 4) == [100, 103, 107, 110]


def test_regularized_stamps_reject_invalid_series():
    with pytest.raises(ValueError):
        MODULE.regularized_stamps(100, 100, 2)
    with pytest.raises(ValueError):
        MODULE.regularized_stamps(100, 110, 1)
