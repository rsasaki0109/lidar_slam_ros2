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

"""Regression tests for the MID360 cross-validation benchmark wrapper."""

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'run_rko_lio_mid360_crossval_benchmark.sh'


def test_mid360_benchmark_script_supports_scan_context_threshold_override():
    """The wrapper should expose descriptor overrides as first-class overrides."""
    script = SCRIPT_PATH.read_text(encoding='utf-8')

    assert '--scan-context-threshold <f>' in script
    assert '--use-bev-descriptor <bool>' in script
    assert '--bev-descriptor-threshold <f>' in script
    assert '--bev-descriptor-sequence-window <n>' in script
    assert '--bev-descriptor-sequence-threshold <f>' in script
    assert '--bev-descriptor-pose-consistency-threshold-m <f>' in script
    assert '--bev-descriptor-max-euclidean-distance-m <f>' in script
    assert '--bev-descriptor-rerank-weight-m <f>' in script
    assert '--use-solid-descriptor <bool>' in script
    assert '--solid-descriptor-min-similarity <f>' in script
    assert '--solid-descriptor-sequence-window <n>' in script
    assert '--solid-descriptor-sequence-min-similarity <f>' in script
    assert '--solid-descriptor-pose-consistency-threshold-m <f>' in script
    assert '--solid-descriptor-max-euclidean-distance-m <f>' in script
    assert '--use-3d-bbs-for-scan-context <bool>' in script
    assert 'SCAN_CONTEXT_THRESHOLD=""' in script
    assert 'USE_BEV_DESCRIPTOR=""' in script
    assert 'BEV_DESCRIPTOR_THRESHOLD=""' in script
    assert 'BEV_DESCRIPTOR_SEQUENCE_WINDOW=""' in script
    assert 'BEV_DESCRIPTOR_SEQUENCE_THRESHOLD=""' in script
    assert 'BEV_DESCRIPTOR_POSE_CONSISTENCY_THRESHOLD_M=""' in script
    assert 'BEV_DESCRIPTOR_MAX_EUCLIDEAN_DISTANCE_M=""' in script
    assert 'BEV_DESCRIPTOR_RERANK_WEIGHT_M=""' in script
    assert 'USE_SOLID_DESCRIPTOR=""' in script
    assert 'SOLID_DESCRIPTOR_MIN_SIMILARITY=""' in script
    assert 'SOLID_DESCRIPTOR_SEQUENCE_WINDOW=""' in script
    assert 'SOLID_DESCRIPTOR_SEQUENCE_MIN_SIMILARITY=""' in script
    assert 'SOLID_DESCRIPTOR_POSE_CONSISTENCY_THRESHOLD_M=""' in script
    assert 'SOLID_DESCRIPTOR_MAX_EUCLIDEAN_DISTANCE_M=""' in script
    assert 'USE_3D_BBS_FOR_SCAN_CONTEXT=""' in script
    assert "params['scan_context_threshold'] = maybe_float(scan_context_threshold)" in script
    assert "params['use_bev_descriptor'] = use_bev_descriptor.lower() == 'true'" in script
    assert "params['bev_descriptor_threshold'] = maybe_float(bev_descriptor_threshold)" in script
    assert (
        "params['bev_descriptor_sequence_window'] = "
        'maybe_int(bev_descriptor_sequence_window)'
    ) in script
    assert (
        "params['bev_descriptor_sequence_threshold'] = "
        'maybe_float(bev_descriptor_sequence_threshold)'
    ) in script
    assert (
        "params['bev_descriptor_pose_consistency_threshold_m'] = maybe_float("
        in script
    )
    assert (
        "params['bev_descriptor_max_euclidean_distance_m'] = maybe_float("
        in script
    )
    assert (
        "params['bev_descriptor_rerank_weight_m'] = maybe_float("
        in script
    )
    assert (
        "params['use_solid_descriptor'] = "
        "use_solid_descriptor.lower() == 'true'"
    ) in script
    assert (
        "params['solid_descriptor_min_similarity'] = "
        'maybe_float(solid_descriptor_min_similarity)'
    ) in script
    assert (
        "params['solid_descriptor_sequence_window'] = "
        'maybe_int(solid_descriptor_sequence_window)'
    ) in script
    assert (
        "params['solid_descriptor_sequence_min_similarity'] = maybe_float("
        in script
    )
    assert (
        "params['solid_descriptor_pose_consistency_threshold_m'] = maybe_float("
        in script
    )
    assert (
        "params['solid_descriptor_max_euclidean_distance_m'] = maybe_float("
        in script
    )
    assert (
        "params['use_3d_bbs_for_scan_context'] = "
        "use_3d_bbs_for_scan_context.lower() == 'true'"
    ) in script
    assert 'scan_context_threshold:' in script
    assert 'use_bev_descriptor:' in script
    assert 'bev_descriptor_sequence_window:' in script
    assert 'bev_descriptor_pose_consistency_threshold_m:' in script
    assert 'bev_descriptor_max_euclidean_distance_m:' in script
    assert 'bev_descriptor_rerank_weight_m:' in script
    assert 'use_solid_descriptor:' in script
    assert 'solid_descriptor_sequence_window:' in script
    assert 'solid_descriptor_pose_consistency_threshold_m:' in script
    assert 'solid_descriptor_max_euclidean_distance_m:' in script
