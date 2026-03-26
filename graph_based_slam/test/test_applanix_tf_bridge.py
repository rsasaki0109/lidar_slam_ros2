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


def test_rpy_deg_to_quaternion_supports_planar_use_case():
    module = _load_module()
    qx, qy, qz, qw = module.rpy_deg_to_quaternion(
        roll_deg=0.0,
        pitch_deg=0.0,
        yaw_deg=90.0,
    )

    assert qx == pytest.approx(0.0)
    assert qy == pytest.approx(0.0)
    assert qz == pytest.approx(0.70710678, rel=1e-6)
    assert qw == pytest.approx(0.70710678, rel=1e-6)


def test_integrate_planar_velocity_uses_elapsed_time():
    module = _load_module()
    east, north = module.integrate_planar_velocity(
        east_m=10.0,
        north_m=20.0,
        velocity_east_mps=3.0,
        velocity_north_mps=-2.0,
        previous_stamp_ns=1_000_000_000,
        current_stamp_ns=1_500_000_000,
    )

    assert east == pytest.approx(11.5)
    assert north == pytest.approx(19.0)
