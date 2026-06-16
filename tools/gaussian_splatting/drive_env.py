#!/usr/bin/env python3
"""Gymnasium driving environment over a 3DGS scene (closed-loop sim2real RL).

Phase 4 of the 3DGS-as-sim2real track: the RL substrate. The closed-loop pieces
already exist -- a GPU renderer, a sensor-sim node, depth-tested actor
compositing, detection scoring -- so this wraps them as a standard
``gymnasium.Env`` an agent can learn in. The ego is a planar unicycle; the task
is to reach a goal pose while staying inside the **valid viewpoint corridor**
(Phase 0: the 3DGS render is only trustworthy within ``max_dev`` of the recorded
trajectory), within bounds, before timeout.

Two observation modes:

* ``state`` -- a low-dim vector (goal range/bearing, corridor deviation). No GPU,
  so a policy trains in seconds and the env is unit tested on CPU.
* ``camera`` -- the 3DGS render from the ego camera via an injected ``render_fn``
  (a ``GaussianRenderer``-backed closure). This is the sim2real observation; the
  agent sees exactly what the closed-loop sensor-sim would publish.

The dynamics, reward, and termination are pure numpy helpers (unit tested);
``DriveEnv`` only wires them to the Gymnasium API and the optional renderer.
"""

from __future__ import annotations

from typing import Callable, Optional, Sequence

import numpy as np


# --------------------------------------------------------------------------- #
# Pure helpers (no gym/torch/CUDA)
# --------------------------------------------------------------------------- #
def wrap_angle(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return float((a + np.pi) % (2.0 * np.pi) - np.pi)


def unicycle_step(pose: Sequence[float], v: float, omega: float,
                  dt: float) -> np.ndarray:
    """Advance a planar unicycle ``(x, y, yaw)`` by speed ``v`` / yaw-rate ``omega``."""
    x, y, yaw = (float(p) for p in pose)
    x += v * np.cos(yaw) * dt
    y += v * np.sin(yaw) * dt
    yaw = wrap_angle(yaw + omega * dt)
    return np.array([x, y, yaw], dtype=float)


def goal_range_bearing(pose: Sequence[float],
                       goal_xy: Sequence[float]) -> tuple[float, float]:
    """Distance to the goal and its bearing in the ego frame (0 = straight ahead)."""
    dx = float(goal_xy[0]) - float(pose[0])
    dy = float(goal_xy[1]) - float(pose[1])
    dist = float(np.hypot(dx, dy))
    bearing = wrap_angle(np.arctan2(dy, dx) - float(pose[2]))
    return dist, bearing


def corridor_deviation(pose_xy: Sequence[float],
                       anchors: np.ndarray) -> float:
    """Distance from the ego to the nearest recorded-trajectory anchor point."""
    pts = np.asarray(anchors, dtype=float)
    d = pts - np.asarray(pose_xy, dtype=float)[None, :]
    return float(np.sqrt((d * d).sum(axis=1)).min())


def crossing_actor(step: int, *, x: float, y0: float, y1: float,
                   cross_steps: int) -> np.ndarray:
    """A pedestrian crossing the corridor at fixed ``x``, ``y0``->``y1`` then held.

    Deterministic (no RNG) so the env and its tests are reproducible: the actor
    walks linearly from ``y0`` to ``y1`` over ``cross_steps`` steps and stops at
    ``y1`` afterwards.
    """
    f = min(max(step / max(cross_steps, 1), 0.0), 1.0)
    return np.array([float(x), float(y0) + (float(y1) - float(y0)) * f],
                    dtype=float)


def collision(a_xy: Sequence[float], b_xy: Sequence[float],
              radius: float) -> bool:
    """Whether two points are within ``radius`` of each other."""
    a = np.asarray(a_xy, dtype=float)
    b = np.asarray(b_xy, dtype=float)
    return bool(np.hypot(*(a - b)) <= radius)


def step_reward(prev_dist: float, dist: float, dev: float, *, max_dev: float,
                step_cost: float = 0.02, dev_weight: float = 0.5,
                goal_tol: float = 1.0, goal_bonus: float = 10.0) -> float:
    """Progress toward the goal, penalising corridor exits and time.

    Reward = (prev_dist - dist) progress - step_cost - dev_weight * max(0, dev -
    max_dev), plus ``goal_bonus`` on arrival (``dist <= goal_tol``).
    """
    r = (prev_dist - dist) - step_cost
    r -= dev_weight * max(0.0, dev - max_dev)
    if dist <= goal_tol:
        r += goal_bonus
    return float(r)


def episode_status(pose: Sequence[float], goal_xy: Sequence[float], *,
                   dev: float, goal_tol: float, bounds: float,
                   hard_dev: float, step: int, max_steps: int) -> tuple:
    """Return ``(terminated, truncated, reason)`` for the current state.

    Terminates on goal arrival, leaving the world ``bounds``, or straying past
    ``hard_dev`` from the corridor (the render is meaningless there); truncates
    at ``max_steps``.
    """
    dist, _ = goal_range_bearing(pose, goal_xy)
    if dist <= goal_tol:
        return True, False, 'goal'
    if abs(pose[0]) > bounds or abs(pose[1]) > bounds:
        return True, False, 'out_of_bounds'
    if dev > hard_dev:
        return True, False, 'left_corridor'
    if step >= max_steps:
        return False, True, 'timeout'
    return False, False, ''


# --------------------------------------------------------------------------- #
# Gymnasium environment
# --------------------------------------------------------------------------- #
class DriveEnv:
    """Planar driving over a 3DGS scene; ``gymnasium.Env`` subclass at runtime.

    Constructed lazily as a ``gymnasium.Env`` so the pure helpers above stay
    importable without gymnasium. Use :func:`make_drive_env` to instantiate.
    """


def make_drive_env(anchors: np.ndarray, goal_xy: Sequence[float], *,
                   start_pose: Sequence[float] = (0.0, 0.0, 0.0),
                   max_dev: float = 0.25, hard_dev: float = 1.0,
                   bounds: float = 50.0, dt: float = 0.2, v_max: float = 2.0,
                   omega_max: float = 1.0, max_steps: int = 200,
                   goal_tol: float = 1.0, obs_mode: str = 'state',
                   render_fn: Optional[Callable[[np.ndarray], np.ndarray]] = None,
                   render_size: Sequence[int] = (120, 160),
                   actor_fn: Optional[Callable[[int], np.ndarray]] = None,
                   actor_radius: float = 1.0, yield_dist: float = 4.0,
                   collision_penalty: float = 10.0):
    """Build a Gymnasium ``DriveEnv`` instance (imports gymnasium lazily).

    When ``actor_fn`` (step -> world xy) is given, a dynamic actor (e.g. a
    crossing pedestrian) is added: the state obs gains the actor's ego-frame
    range/bearing, a proximity cost applies within ``yield_dist`` ahead, and a
    collision (within ``actor_radius``) ends the episode with ``collision_penalty``.
    """
    import gymnasium as gym
    from gymnasium import spaces

    anchors = np.asarray(anchors, dtype=float)
    goal_xy = np.asarray(goal_xy, dtype=float)
    start_pose = np.asarray(start_pose, dtype=float)
    has_actor = actor_fn is not None

    class _DriveEnv(gym.Env):
        metadata = {'render_modes': []}

        def __init__(self):
            super().__init__()
            self.action_space = spaces.Box(-1.0, 1.0, (2,), dtype=np.float32)
            if obs_mode == 'camera':
                if render_fn is None:
                    raise ValueError('camera obs_mode needs a render_fn')
                h, w = int(render_size[0]), int(render_size[1])
                self.observation_space = spaces.Box(0, 255, (h, w, 3),
                                                    dtype=np.uint8)
            else:
                # [goal_dist, cos(bearing), sin(bearing), corridor_dev]
                # (+ [actor_range, cos, sin] when an actor is present)
                span = 4.0 * float(bounds)
                hi = [span, 1.0, 1.0, span]
                lo = [0.0, -1.0, -1.0, 0.0]
                if has_actor:
                    hi += [span, 1.0, 1.0]
                    lo += [0.0, -1.0, -1.0]
                self.observation_space = spaces.Box(
                    np.array(lo, dtype=np.float32),
                    np.array(hi, dtype=np.float32), dtype=np.float32)
            self._pose = start_pose.copy()
            self._step = 0
            self._prev_dist = 0.0

        def _actor_xy(self):
            return np.asarray(actor_fn(self._step), dtype=float)

        def _obs(self):
            if obs_mode == 'camera':
                return np.asarray(render_fn(self._pose), dtype=np.uint8)
            dist, bearing = goal_range_bearing(self._pose, goal_xy)
            dev = corridor_deviation(self._pose[:2], anchors)
            vec = [dist, np.cos(bearing), np.sin(bearing), dev]
            if has_actor:
                arange, abear = goal_range_bearing(self._pose, self._actor_xy())
                vec += [arange, np.cos(abear), np.sin(abear)]
            return np.array(vec, dtype=np.float32)

        def reset(self, *, seed=None, options=None):
            super().reset(seed=seed)
            self._pose = start_pose.copy()
            self._step = 0
            self._prev_dist, _ = goal_range_bearing(self._pose, goal_xy)
            return self._obs(), {'pose': self._pose.copy()}

        def step(self, action):
            a = np.clip(np.asarray(action, dtype=float), -1.0, 1.0)
            v, omega = float(a[0]) * v_max, float(a[1]) * omega_max
            self._pose = unicycle_step(self._pose, v, omega, dt)
            self._step += 1
            dist, _ = goal_range_bearing(self._pose, goal_xy)
            dev = corridor_deviation(self._pose[:2], anchors)
            reward = step_reward(self._prev_dist, dist, dev, max_dev=max_dev,
                                 goal_tol=goal_tol)
            self._prev_dist = dist
            terminated, truncated, reason = episode_status(
                self._pose, goal_xy, dev=dev, goal_tol=goal_tol, bounds=bounds,
                hard_dev=hard_dev, step=self._step, max_steps=max_steps)
            if reason == 'out_of_bounds' or reason == 'left_corridor':
                reward -= 5.0
            info = {'pose': self._pose.copy(), 'dev': dev, 'dist': dist,
                    'reason': reason}
            if has_actor:
                actor_xy = self._actor_xy()
                arange, abear = goal_range_bearing(self._pose, actor_xy)
                # proximity cost only for an actor ahead within yield_dist
                if arange < yield_dist and abs(abear) < np.pi / 2:
                    reward -= (yield_dist - arange) / yield_dist
                info['actor_xy'] = actor_xy
                info['actor_dist'] = arange
                if collision(self._pose[:2], actor_xy, actor_radius):
                    reward -= collision_penalty
                    terminated, reason = True, 'collision'
                    info['reason'] = reason
            return self._obs(), float(reward), terminated, truncated, info

    return _DriveEnv()
