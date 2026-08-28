#!/usr/bin/env python3

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


"""Static guards for the bounded registration-session stress contract."""

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
INTERFACE = (
    REPO_ROOT / 'lidarslam_plugin_interfaces/include/lidarslam_plugin_interfaces/registration.hpp'
)
LOADER_HEADER = (
    REPO_ROOT
    / 'lidarslam_registration_loader/include/'
    'lidarslam_registration_loader/registration_plugin_loader.hpp'
)
LOADER_SOURCE = REPO_ROOT / 'lidarslam_registration_loader/src/registration_plugin_loader.cpp'
HARNESS = REPO_ROOT / 'tools/registration_plugin_session_stress.cpp'


def _read(path: Path) -> str:
    return path.read_text(encoding='utf-8')


def test_cancellation_model_is_explicit_and_defaults_noninterruptible():
    text = _read(INTERFACE)
    assert 'kNonInterruptibleAlign' in text
    assert 'kCooperativeCancel' in text
    assert 'cancellation_model_{CancellationModel::kNonInterruptibleAlign}' in text
    assert 'virtual void requestCancel() noexcept {}' in text


def test_session_has_pre_post_checkpoints_and_quiescent_shutdown():
    header = _read(LOADER_HEADER)
    source = _read(LOADER_SOURCE)
    assert 'std::condition_variable quiesced' in header
    assert 'std::size_t in_flight{0U}' in header
    assert 'state->quiesced.wait' in source
    assert 'state->end(&lock)' in source
    assert 'registration plugin session was cancelled during alignment' in source


def test_stress_harness_exercises_required_failure_and_lifetime_edges():
    text = _read(HARNESS)
    for marker in (
        'runHighContention',
        'runCancellation',
        'runFaultAndActivation',
        'runRepeatedLifetime',
        'RegistrationPluginSessionAdapter',
        'RegistrationActivationTransaction',
        'requestCancel',
        'resetDuringCallback',
        '--receipt',
    ):
        assert marker in text


def test_no_detached_or_unsafe_thread_interruption_path():
    text = '\n'.join((_read(HARNESS), _read(LOADER_SOURCE)))
    assert '.detach(' not in text
    assert 'pthread_cancel' not in text
    assert 'std::async' not in text
    assert 'std::terminate' not in text
