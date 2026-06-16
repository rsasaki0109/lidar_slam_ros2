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

"""Tests for the 3DGS driving RL env (pure helpers + state-mode rollout, CPU)."""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
TOOL_DIR = REPO_ROOT / 'tools' / 'gaussian_splatting'


def _load():
    if str(TOOL_DIR) not in sys.path:
        sys.path.insert(0, str(TOOL_DIR))
    import drive_env

    return drive_env


de = _load()


def test_wrap_angle_range():
    # +-pi are the same heading; the wrap maps odd multiples of pi to |pi|
    assert abs(de.wrap_angle(3.0 * np.pi)) == pytest.approx(np.pi)
    assert abs(de.wrap_angle(-3.0 * np.pi)) == pytest.approx(np.pi)
    assert de.wrap_angle(0.0) == 0.0
    assert de.wrap_angle(np.pi / 2) == pytest.approx(np.pi / 2)


def test_unicycle_step_drives_along_heading():
    p = de.unicycle_step([0.0, 0.0, 0.0], 2.0, 0.0, 0.5)
    assert np.allclose(p, [1.0, 0.0, 0.0])
    p2 = de.unicycle_step([0.0, 0.0, np.pi / 2], 2.0, 0.0, 0.5)
    assert np.allclose(p2[:2], [0.0, 1.0], atol=1e-9)


def test_unicycle_step_turns_with_omega():
    p = de.unicycle_step([0.0, 0.0, 0.0], 0.0, 1.0, 0.5)
    assert p[2] == pytest.approx(0.5)
    assert np.allclose(p[:2], [0.0, 0.0])


def test_goal_range_bearing_ahead_and_left():
    dist, bearing = de.goal_range_bearing([0.0, 0.0, 0.0], [3.0, 0.0])
    assert dist == pytest.approx(3.0) and bearing == pytest.approx(0.0)
    _, bl = de.goal_range_bearing([0.0, 0.0, 0.0], [0.0, 5.0])
    assert bl == pytest.approx(np.pi / 2)  # goal to the left


def test_corridor_deviation_nearest_anchor():
    anchors = np.array([[0.0, 0.0], [10.0, 0.0], [20.0, 0.0]])
    assert de.corridor_deviation([10.0, 3.0], anchors) == pytest.approx(3.0)
    assert de.corridor_deviation([0.0, 0.0], anchors) == pytest.approx(0.0)


def test_step_reward_rewards_progress_and_penalises_dev():
    # closing 2 m, inside corridor: progress 2 - step_cost
    r = de.step_reward(10.0, 8.0, 0.1, max_dev=0.25)
    assert r == pytest.approx(2.0 - 0.02)
    # straying beyond max_dev costs dev_weight * excess
    r2 = de.step_reward(10.0, 10.0, 1.25, max_dev=0.25)
    assert r2 == pytest.approx(-0.02 - 0.5 * 1.0)


def test_step_reward_goal_bonus():
    r = de.step_reward(2.0, 0.5, 0.0, max_dev=0.25, goal_tol=1.0)
    assert r > 10.0  # progress + goal_bonus


def test_episode_status_goal_and_bounds_and_corridor():
    g = [5.0, 0.0]
    term, trunc, reason = de.episode_status(
        [5.0, 0.2, 0.0], g, dev=0.0, goal_tol=1.0, bounds=50.0,
        hard_dev=1.0, step=1, max_steps=200)
    assert term and reason == 'goal'
    term, _, reason = de.episode_status(
        [99.0, 0.0, 0.0], g, dev=0.0, goal_tol=1.0, bounds=50.0,
        hard_dev=1.0, step=1, max_steps=200)
    assert term and reason == 'out_of_bounds'
    term, _, reason = de.episode_status(
        [0.0, 0.0, 0.0], g, dev=2.0, goal_tol=1.0, bounds=50.0,
        hard_dev=1.0, step=1, max_steps=200)
    assert term and reason == 'left_corridor'
    _, trunc, reason = de.episode_status(
        [0.0, 0.0, 0.0], g, dev=0.0, goal_tol=1.0, bounds=50.0,
        hard_dev=1.0, step=200, max_steps=200)
    assert trunc and reason == 'timeout'


def _anchors():
    return np.stack([np.linspace(0, 10, 11), np.zeros(11)], axis=1)


def test_env_reset_and_obs_shape():
    pytest.importorskip('gymnasium')
    env = de.make_drive_env(_anchors(), [10.0, 0.0], max_steps=50)
    obs, info = env.reset(seed=0)
    assert obs.shape == (4,) and obs.dtype == np.float32
    assert env.observation_space.contains(obs)
    assert 'pose' in info


def test_env_driving_straight_reaches_goal():
    pytest.importorskip('gymnasium')
    env = de.make_drive_env(_anchors(), [10.0, 0.0], dt=0.5, v_max=2.0,
                            max_steps=100, goal_tol=1.0)
    env.reset(seed=0)
    total, done = 0.0, False
    for _ in range(100):
        obs, r, term, trunc, info = env.step([1.0, 0.0])  # full speed ahead
        total += r
        if term or trunc:
            done = True
            break
    assert done and info['reason'] == 'goal'
    assert total > 0.0  # net positive: progressed and arrived


def test_env_leaving_corridor_terminates_with_penalty():
    pytest.importorskip('gymnasium')
    env = de.make_drive_env(_anchors(), [10.0, 0.0], dt=0.5, v_max=2.0,
                            omega_max=1.0, hard_dev=1.0, max_steps=100)
    env.reset(seed=0)
    # turn hard left and drive: leaves the y=0 corridor quickly
    reason = ''
    for _ in range(100):
        _, _, term, trunc, info = env.step([1.0, 1.0])
        reason = info['reason']
        if term or trunc:
            break
    assert reason in ('left_corridor', 'out_of_bounds')


def test_env_passes_gymnasium_checker():
    pytest.importorskip('gymnasium')
    from gymnasium.utils.env_checker import check_env

    env = de.make_drive_env(_anchors(), [10.0, 0.0], max_steps=50)
    check_env(env, skip_render_check=True)
