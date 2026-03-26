# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)

"""Regression tests for Applanix-to-TF conversion helpers."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'convert_applanix_gsof49_to_tf_bag.py'


def _load_module():
    spec = importlib.util.spec_from_file_location(
        'convert_applanix_gsof49_to_tf_bag',
        SCRIPT_PATH,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_heading_deg_to_enu_yaw_deg_converts_north_clockwise_heading():
    module = _load_module()

    assert module.heading_deg_to_enu_yaw_deg(0.0) == pytest.approx(90.0)
    assert module.heading_deg_to_enu_yaw_deg(90.0) == pytest.approx(0.0)
    assert module.heading_deg_to_enu_yaw_deg(180.0) == pytest.approx(-90.0)


def test_lla_to_enu_returns_zero_at_origin():
    module = _load_module()

    east, north, up = module.lla_to_enu(
        latitude_deg=35.0,
        longitude_deg=139.0,
        altitude_m=42.0,
        origin_latitude_deg=35.0,
        origin_longitude_deg=139.0,
        origin_altitude_m=42.0,
    )

    assert east == pytest.approx(0.0, abs=1e-6)
    assert north == pytest.approx(0.0, abs=1e-6)
    assert up == pytest.approx(0.0, abs=1e-6)


def test_sec_nsec_from_ns_preserves_bag_epoch_time():
    module = _load_module()
    sec, nanosec = module.sec_nsec_from_ns(1_654_865_262_868_823_520)

    assert sec == 1_654_865_262
    assert nanosec == 868_823_520
