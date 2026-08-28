# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Contract tests for deterministic backend-input capture."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts/record_backend_input.sh'


def test_capture_wraps_arbitrary_command_and_records_both_backend_topics():
    """The wrapper must preserve arguments and capture the complete contract."""
    source = SCRIPT.read_text()

    assert 'COMMAND=("$@")' in source
    assert '--include-unpublished-topics' in source
    assert '--storage-preset-profile fastwrite' in source
    assert '--max-cache-size 1073741824' in source
    assert '--qos-profile-overrides-path' in source
    assert '/rko_lio/odometry /rko_lio/frame' in source
    assert 'COMMAND_STATUS=$?' in source
    assert 'exit "${COMMAND_STATUS}"' in source


def test_capture_flushes_and_validates_metadata_counts():
    """A recorder process exiting is insufficient without non-empty topics."""
    source = SCRIPT.read_text()

    assert '/usr/bin/env --default-signal=INT --default-signal=TERM' in source
    assert 'kill -INT "${RECORDER_PID}"' in source
    assert 'kill -KILL "${RECORDER_PID}"' in source
    assert 'ros2 bag reindex' in source
    assert 'topics_with_message_count' in source
    assert 'counts.get(name, 0) <= 0' in source


def test_capture_refuses_to_overwrite_an_existing_bag():
    """Avoid silently mixing two frontend runs in one evidence directory."""
    source = SCRIPT.read_text()

    assert 'output already exists; refusing to mix captures' in source
