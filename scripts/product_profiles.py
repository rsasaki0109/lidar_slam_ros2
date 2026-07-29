#!/usr/bin/env python3
"""Single source of truth for maintained product workflow profiles."""

from __future__ import annotations

from typing import NamedTuple


class MaintainedProfile(NamedTuple):
    """Public identity and help text for one executable product profile."""

    profile_id: str
    description: str


MAINTAINED_PROFILES = (
    MaintainedProfile(
        'rko_lio_graph_public_path',
        'PointCloud2 + Imu through RKO-LIO and graph_based_slam.',
    ),
    MaintainedProfile(
        'rko_lio_graph_mid360_preset',
        'Livox/MID360 PointCloud2 + Imu with tracked tuned params.',
    ),
    MaintainedProfile(
        'pointcloud_gnss_smoke',
        'PointCloud2 + NavSatFix smoke workflow.',
    ),
    MaintainedProfile(
        'packet_applanix_smoke',
        'VelodyneScan + Applanix GSOF49 smoke workflow.',
    ),
)

PROFILE_IDS = tuple(profile.profile_id for profile in MAINTAINED_PROFILES)
PROFILE_HELP = tuple(
    (profile.profile_id, profile.description)
    for profile in MAINTAINED_PROFILES
)

if len(PROFILE_IDS) != len(set(PROFILE_IDS)):
    raise RuntimeError('maintained product profile IDs must be unique')
